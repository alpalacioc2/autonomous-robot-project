## Overview

This robot is designed to:

- Use a camera to detect and follow a black line on the floor.
- Compute steering adjustments using the Pure Pursuit algorithm.
- Communicate velocity commands to an Arduino Mega via USB serial.
- Control motor speeds using PWM with encoder feedback and a PD controller.

---

## Hardware Components

- **Raspberry Pi 4** (running Ubuntu 20.04, ROS 2 Humble)
- **Arduino Mega 2560**
- **DC Motors** with encoders
- **L298N Motor Driver**
- **Line sensor or Pi Camera**
- **3D printed chassis**
- **Battery pack**

---

## Software Features

### ROS 2 Node (C++)

- Subscribes to `/image_raw` topic
- Processes frames with OpenCV to detect the line
- Uses Pure Pursuit to calculate steering
- Sends velocity commands over serial (`/dev/ttyS0`) to the Arduino

### Arduino Controller

- Receives `C <vel1> <vel2>` velocity commands
- Uses PWM for motor control
- Uses encoders for RPM feedback
- Implements a simple PD control loop

---

## Getting Started

### Clone and Build

```bash  
cd autonomous-robot-project3/ROS2
colcon build
source install/setup.bash
ros2 run line_tracker line_tracker_node
