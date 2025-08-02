char command[30] = {0};
char param1[15] = {0};
char param2[15] = {0};
float v1 = 0.;
float v2 = 0.;
int cnt = 0;
int ch_count = 0;
int i = 0;

void command_server(void)
{
    if (Serial.available() > 0)
    { 
        command[ch_count] = Serial.read();
        ch_count++;

        if (command[ch_count - 1] == 10)  // newline received
        {
            // === Handle velocity command "C v1 v2" ===
            if ((command[0] == 'C') && (command[1] == 32) && (ch_count > 5))
            {
                Serial.print(command);
                i = 2;
                cnt = 0;

                // extract param1
                while (command[i] != 32 && command[i] != 10)
                {
                    param1[cnt++] = command[i++];
                }
                param1[cnt] = '\0';

                // extract param2
                cnt = 0;
                if (command[i] == 32) i++;  // skip space
                while (command[i] != 10)
                {
                    param2[cnt++] = command[i++];
                }
                param2[cnt] = '\0';

                v1 = atof(param1);
                v2 = atof(param2);

                wheelDesVel[0] = v1;
                wheelDesVel[1] = v2;

                Serial.print(v1);
                Serial.print(" ");
                Serial.println(v2);
            }

            // === Handle motion primitive command "M x" with manual control ===
            else if ((command[0] == 'M') && (command[1] == 32) && (ch_count >= 4))
            {
                int motionCode = command[2] - '0';  // convert ASCII to int
                Serial.print("Received motion code: ");
                Serial.println(motionCode);

                int motionDuration = 0;

                if (motionCode == 0) {
                    // Turn left 90°
                    wheelDesVel[0] = -1.0;
                    wheelDesVel[1] = 1.0;
                    motionDuration = 1200;
                }
                else if (motionCode == 1) {
                    // Move forward 1 ft
                    wheelDesVel[0] = 1.0;
                    wheelDesVel[1] = 1.0;
                    motionDuration = 3000;
                }
                else if (motionCode == 2) {
                    // Turn right 90°
                    wheelDesVel[0] = 1.0;
                    wheelDesVel[1] = -1.0;
                    motionDuration = 1200;
                }

                // === Manual control update during motion ===
                unsigned long t0 = millis();
                while (millis() - t0 < motionDuration) {
                    getCurrentStatus();
                    lowLevelControl();
                    delay(10);  // simulate 100 Hz control loop
                }

                // Stop motion after duration
                wheelDesVel[0] = 0.0;
                wheelDesVel[1] = 0.0;
            }

            // === Handle invalid command ===
            else
            {
                Serial.println("Not a valid command");
            }

            // Clear buffers
            memset(command, 0, sizeof(command));
            memset(param1, 0, sizeof(param1));
            memset(param2, 0, sizeof(param2));
            ch_count = 0;
        }
    }
}
