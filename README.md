# Mecanum Platform – ESP32-S3 + Encoders + IMU + FreeRTOS

This repository contains the firmware for a mecanum-wheeled robotic platform based on the ESP32-S3. The platform is designed as part of a modular robotics system and communicates wirelessly using ESP-NOW or via cable using SPI. It supports odometry using wheel encoders, orientation estimation via an IMU, and real-time motor control through PWM signals.

---

## Main Features
- **ESP32-S3** as the main controller
- **4 DC motors with encoders** for mecanum drive (X, Y, rotation)
- **PWM + DIR motor drivers**
- **Incremental encoders** with ~1000 pulses per minute
- **FreeRTOS** task-based architecture
- **Modular communication via ESP-NOW **
- **Support for external debug monitor and remote control pad**

---

## Components Used

| Part Description                                      | Model / Notes                                                       | Quantity       |
|------------------------------------------------------|----------------------------------------------------------------------|----------------|
| Main microcontroller board                          | ESP32-S3-DEV-KIT-N8R8, Waveshare 24243                              | 1              |
| DC gear motors with encoders                        | SJ01 120:1, 6V, 160RPM + encoder, EAN: 6959420910205               | 4              |
| Dual motor driver                                   | Cytron Maker Drive MX1508, 9.5V/1A, EAN: 5904422321802            | 2              |
| Mecanum platform chassis                            | Smart Robot Car Kit (Amazon), EAN: 500386256                       | 1              |
| Breadboard                                          | justPi 830 points                                                   | 2              |
| Battery holder for 2x 18650                         | Series connection, EAN: 5904422374341                              | 1              |
| 18650 Li-Ion cells                                  | XTAR 18650, 3500mAh                                                 | 2              |


---

## Roadmap – Mecanum Platform (ESP32-S3)

### Stage 1 – MVP: Motor & Encoder Integration
- [x] Initialize PWM control for all four motors
- [x] Integrate encoder reading for each wheel
- [x] Set up FreeRTOS tasks for motor and encoder tasks
- [x] Test basic forward, backward, and rotation movement
- [x] Basic Serial output of encoder counts and motor status

### Stage 2 – Odometry & Motion Logic
- [x] Calculate velocity (X, Y, angular) based on encoder data
- [ ] Implement odometry tracking for position and heading
- [ ] Create basic PID loop for closed-loop velocity control
- [ ] Integrate watchdog for motor task failures
- [ ] Test performance across speed range: min, max, stable

### Stage 3 – IMU & Sensor Fusion
- [ ] Integrate MPU-6050 via I2C
- [ ] Calibrate gyroscope and accelerometer
- [ ] Fuse encoder and IMU data for more accurate orientation
- [ ] Add fallback logic in case IMU or encoder fails

### Stage 4 – Communication
- [ ] Implement SPI slave communication with controller (pad)
- [ ] Implement ESP-NOW as backup or for remote telemetry
- [ ] Create structured message format for motor commands and telemetry
- [ ] Add message parsing with CRC/checksum validation

### Stage 5 – Diagnostics & Debug Monitor
- [ ] Send periodic status to monitor (errors, voltage, current)
- [ ] Display system state visually via monitor screen
- [ ] Log system faults (motor error, encoder failure, IMU missing)

### Stage 6 – Physical Optimization
- [ ] Move from breadboard to protoboard or PCB
- [ ] Add power switch and battery voltage display
- [ ] Add mounting for IMU and vibration isolation
- [ ] Shield analog lines for current sensors

---

## License
This project is licensed under the **GNU General Public License v3.0**. See the [LICENSE](LICENSE) file for details.

---

## Notes
This platform is part of a broader modular robotic system consisting of:
- A **controller pad** (ESP32-S3 + display + joysticks)
- The **platform** (this repo)
- A **debug monitor** (telemetry and diagnostic module)

Development focus: clean modular code, reproducible experiments, future integration with ROS2 and machine learning components.

