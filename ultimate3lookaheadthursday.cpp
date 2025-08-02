// *******************************************************************************************
// * File:        line_tracker_node.cpp                                                      *
// * Authors:     Alejandro Caro, Garrison Gralike, Jaewan McPherson                         *
// * Date:        07/30/2025                                                                 *
// * Course:      EEL 4930 – Autonomous Robots                                               *
// * Project:     Mini Project 3: Autonomous Robot Lane Tracking – Pure Pursuit (3 Lookahead)*
// * Description: This file implements a ROS 2 node that subscribes to RGB images from a     *
// *              camera, processes a scanline to find a black line, and calculates a motor  *
// *              command using a pure pursuit algorithm. Serial commands are sent to a      *
// *               an Arduino Mega microcontroller to drive the robot primite motions.       *
// * References:  ROS 2 rclcpp API, OpenCV grayscale formula, and pure pursuit math model.   *
// *******************************************************************************************

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>

class LineTrackerNode : public rclcpp::Node
{
public:
    // *******************************************************************************************
    // * Function:    LineTrackerNode                                                            *
    // * Purpose:     Constructor for the LineTrackerNode class. Opens the serial port and       *
    // *              subscribes to the /image_raw topic for processing incoming frames.         *
    // *******************************************************************************************
    LineTrackerNode() : Node("line_tracker_node")
    {
        serial_port_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY);
        if (serial_port_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            return;
        }

        struct termios options;
        tcgetattr(serial_port_fd_, &options);
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        tcsetattr(serial_port_fd_, TCSANOW, &options);

        std::string stop_cmd = "C 0 0\n";
        write(serial_port_fd_, stop_cmd.c_str(), stop_cmd.length());

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&LineTrackerNode::processImage, this, std::placeholders::_1));
    }
    // *******************************************************************************************
    // * Function:    ~LineTrackerNode                                                           *
    // * Purpose:     Destructor for the LineTrackerNode class. Stops the robot and closes the   *
    // *              serial port safely.                                                        *
    // *******************************************************************************************
    ~LineTrackerNode()
    {
        if (serial_port_fd_ != -1)
        {
            std::string stop_cmd = "C 0 0\n";
            write(serial_port_fd_, stop_cmd.c_str(), stop_cmd.length());
            close(serial_port_fd_);
        }
    }

private:
    // *******************************************************************************************
    // * Function:    processImage                                                               *
    // * Purpose:     Callback that receives image frames from the camera, extracts grayscale    *
    // *              data from 3 scan rows (lookaheads), and calculates a weighted average      *
    // *              line center for curvature and motor control.                               *
    // * Parameters:  msg – Shared pointer to the sensor_msgs::msg::Image message                *
    // * Returns:     void                                                                       *
    // *******************************************************************************************
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "rgb8")
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }

        int width = msg->width;
        int height = msg->height;

        std::vector<int> scan_rows = {
            height * 4 / 8,   // bottom (closest)
            height * 2 / 8,
            height * 1 / 8
        };
        std::vector<double> weights = {1.5, 0.20, 0.01};
        std::vector<double> centers;
        std::vector<double> used_weights;

        double bottom_center = -1;

        for (size_t i = 0; i < scan_rows.size(); i++)
        {
            int row = scan_rows[i];
            std::vector<int> gray_row(width);

            for (int x = 0; x < width; x++)
            {
                int idx = (row * width + x) * 3;
                if (idx + 2 >= static_cast<int>(msg->data.size()))
                    return;

                float r = static_cast<float>(msg->data[idx]);
                float g = static_cast<float>(msg->data[idx + 1]);
                float b = static_cast<float>(msg->data[idx + 2]);
                gray_row[x] = static_cast<int>(0.299f * r + 0.587f * g + 0.114f * b);
            }

            double row_mean = std::accumulate(gray_row.begin(), gray_row.end(), 0.0) / width;
            int threshold = static_cast<int>(row_mean * 0.6);

            int sum_x = 0, count_black = 0;
            int black_left = 0, black_right = 0;

            for (int x = 0; x < width; x++)
            {
                if (gray_row[x] < threshold)
                {
                    sum_x += x;
                    count_black++;
                    if (x < width / 2) black_left++;
                    else black_right++;
                }
            }

            // Allow low count if right dominates
            bool allow_row = count_black > 50 || (black_right > black_left && black_right > 20);

            if (allow_row)
            {
                double weight = weights[i];
                if (black_right > black_left)
                    weight += 0.5;  // bias towards right if needed

                double center = sum_x / static_cast<double>(count_black);
                if (i == 0) bottom_center = center;
                centers.push_back(center * weight);
                used_weights.push_back(weight);
            }
        }

        std::string cmd;

        if (!centers.empty())
        {
            double total_weight = std::accumulate(used_weights.begin(), used_weights.end(), 0.0);
            double weighted_center_x = std::accumulate(centers.begin(), centers.end(), 0.0) / total_weight;

            //  Make sure to go straight and corects in crossed road to keep going forward
            if (bottom_center > 0 && std::abs(bottom_center - width / 2.0) < 30)
            {
                weighted_center_x = width / 2.0;  // Force straight
            }

            double dx = weighted_center_x - (width / 2.0);
            double dy = 7.0;
            double curvature = (2.0 * dx) / (dx * dx + dy * dy);

            double base_speed = 0.02;  // slow speed but steady
            double left_speed, right_speed;

            // Hard turn mode
            if (std::abs(dx) > 95) // better treshold works good in edges
            {
                if (dx < 0)
                {
                    left_speed = 0.0;
                    right_speed = base_speed;
                }
                else
                {
                    left_speed = base_speed;
                    right_speed = 0.0;
                }
            }
            else
            {
                left_speed = base_speed * (1.0 - curvature);
                right_speed = base_speed * (1.0 + curvature);
            }

            int left = std::clamp(static_cast<int>(std::round(left_speed * 100)), 0, 15);
            int right = std::clamp(static_cast<int>(std::round(right_speed * 100)), 0, 15);

            cmd = "C " + std::to_string(left) + " " + std::to_string(right) + "\n";
            write(serial_port_fd_, cmd.c_str(), cmd.length());

            if (++frame_count_ % 30 == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "dx=%.1f curv=%.3f bottom=%.1f => %s",
                            dx, curvature, bottom_center, cmd.c_str());
            }
        }
        else
        {
            cmd = "C 0 0\n";
            write(serial_port_fd_, cmd.c_str(), cmd.length());
            RCLCPP_WARN(this->get_logger(), "Line lost on all rows. Stopping.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int serial_port_fd_;
    unsigned long int frame_count_ = 0;
};
// *******************************************************************************************
// * Function:    main                                                                        *
// * Purpose:     Entry point for the ROS 2 node. Initializes, spins, and shuts down the node *
// * Parameters:  argc – argument count                                                       *
// *              argv – argument vector                                                      *
// * Returns:     int – exit code                                                             *
// *******************************************************************************************
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerNode>());
    rclcpp::shutdown();
    return 0;
}