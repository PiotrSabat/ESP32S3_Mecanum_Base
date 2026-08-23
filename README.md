# Mecanum Platform – ESP32-S3 + Encoders + FreeRTOS

This repository contains the firmware for a mecanum-wheeled robotic platform based on the ESP32-S3. The platform is one half of a two-device system — the other is a handheld pad that both drives the robot and displays its telemetry. The two communicate wirelessly over ESP-NOW. Wheel speed is measured with incremental encoders and regulated by a per-motor PID loop driving PWM outputs.

IMU-based orientation estimation and odometry are planned but not implemented yet — see the roadmap below.

---

## Main Features
- **ESP32-S3** as the main controller
- **4 DC motors with encoders** for mecanum drive (X, Y, rotation)
- **PWM + DIR motor drivers**
- **Incremental encoders**, ~960 pulses per wheel revolution
- **Closed-loop speed control** — independent PID per motor, tunable at runtime
- **FreeRTOS** task-based architecture
- **ESP-NOW link to the control pad**, carrying commands one way and telemetry the other

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
- [x] Create basic PID loop for closed-loop velocity control
- [ ] Implement odometry tracking for position and heading
- [ ] Fail-safe: stop the platform when the link to the pad goes silent
- [ ] Scale and normalise pad input so combined axes cannot exceed `MAX_RPM`
- [ ] Test performance across speed range: min, max, stable

### Stage 3 – IMU & Sensor Fusion
- [ ] Integrate MPU-6050 via I2C
- [ ] Calibrate gyroscope and accelerometer
- [ ] Fuse encoder and IMU data for more accurate orientation
- [ ] Add fallback logic in case IMU or encoder fails

### Stage 4 – Communication
- [x] Structured message format for motor commands and telemetry (`src/messages.h`)
- [x] ESP-NOW as the single link between pad and platform
- [ ] Keep `messages.h` in sync across both repositories automatically
- [ ] Add message parsing with CRC/checksum validation

SPI to the pad was explored and dropped; ESP-NOW is now the only channel.

### Stage 5 – Diagnostics on the Pad
- [x] Send periodic telemetry (wheel RPM, task timing, message counters)
- [ ] Redirect telemetry from the retired monitor module to the pad
- [ ] Display system state on the pad screen
- [ ] Report battery voltage and log faults (motor, encoder, link loss)

### Stage 6 – Physical Optimization
- [ ] Move from breadboard to protoboard or PCB
- [ ] Add power switch and battery voltage display
- [ ] Add mounting for IMU and vibration isolation
- [ ] Shield analog lines for current sensors

---

## License

Released under the MIT License. See [LICENSE](LICENSE).

Third-party libraries retain their own licenses:
- [ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder) – BSD 4-clause
- ESP32 Arduino Core (including ESP-NOW and WiFi) – Apache 2.0 / LGPL

This product includes software developed by the "Universidad de Palermo,
Argentina" (http://www.palermo.edu/) — acknowledgment required by the
ESP32Encoder license.

---

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

```bash
git clone https://github.com/PiotrSabat/ESP32S3_Mecanum_Base.git
cd ESP32S3_Mecanum_Base
pio run -t upload
```

**MAC addresses.** The project builds out of the box using the placeholder
addresses in `src/mac_addresses.h`. Edit that file with the MAC addresses of
your own devices, or create `src/mac_addresses_private.h` — if present, it takes
precedence and is excluded from version control.

Every push and pull request is built by GitHub Actions, so a broken build shows
up before it reaches the hardware.

---

## Notes
This platform is one half of a two-device robotic system:
- A **controller pad** (Xiao ESP32-S3 + TFT display + joysticks) — drives the
  robot and doubles as its telemetry display
  ([Pad_Adafruit_Xiao](https://github.com/CableAndCode/Pad_Adafruit_Xiao))
- The **platform** (this repo)

An earlier design included a separate debug monitor module and a companion
iPhone app. Both were dropped in favour of putting the diagnostics on the pad
itself, which shortens the path to a system that simply works.

Development focus: clean modular code, reproducible experiments, future integration with ROS2 and machine learning components.

