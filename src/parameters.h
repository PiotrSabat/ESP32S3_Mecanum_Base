#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

// ===== Motor PWM Definitions =====
// PWM signal frequency for motor drivers: 20 kHz, 9-bit resolution

// Front Left Motor pins and PWM channels
#define FL_PIN1 9       // GPIO pin for front-left motor input A (M1A)
#define FL_PIN2 10      // GPIO pin for front-left motor input B (M1B)
#define FL_CHANNEL1 0   // LEDC PWM channel for M1A
#define FL_CHANNEL2 1   // LEDC PWM channel for M1B

// Front Right Motor pins and PWM channels
#define FR_PIN1 11      // GPIO pin for front-right motor input A (M2A)
#define FR_PIN2 12      // GPIO pin for front-right motor input B (M2B)
#define FR_CHANNEL1 2   // LEDC PWM channel for M2A
#define FR_CHANNEL2 3   // LEDC PWM channel for M2B

// Rear Left Motor pins and PWM channels
#define RL_PIN1 13      // GPIO pin for rear-left motor input A (M2A)
#define RL_PIN2 14      // GPIO pin for rear-left motor input B (M2B)
#define RL_CHANNEL1 4   // LEDC PWM channel for M2A
#define RL_CHANNEL2 5   // LEDC PWM channel for M2B

// Rear Right Motor pins and PWM channels
#define RR_PIN1 15      // GPIO pin for rear-right motor input A (M1A)
#define RR_PIN2 16      // GPIO pin for rear-right motor input B (M1B)
#define RR_CHANNEL1 6   // LEDC PWM channel for M1A
#define RR_CHANNEL2 7   // LEDC PWM channel for M1B

// ===== Encoder Pin Definitions =====
// GPIO pins connected to quadrature encoder channels
#define FL_ENCODER_A 1  // Front-left encoder channel A
#define FL_ENCODER_B 2  // Front-left encoder channel B
#define FR_ENCODER_A 4  // Front-right encoder channel A
#define FR_ENCODER_B 5  // Front-right encoder channel B
#define RL_ENCODER_A 6  // Rear-left encoder channel A
#define RL_ENCODER_B 7  // Rear-left encoder channel B
#define RR_ENCODER_A 17 // Rear-right encoder channel A
#define RR_ENCODER_B 18 // Rear-right encoder channel B

// Number of pulses per revolution for all encoders
#define ENCODER_RESOLUTION 960

// Maximum motor speed in RPM (used for conversion calculations)
#define MAX_RPM 180

// ===== Task Scheduling Rates =====
// FreeRTOS task delay intervals (in milliseconds)
static const int rate_1 = 50;  // Interval for Task 1
static const int rate_2 = 25;  // Interval for Task 2
static const int rate_3 = 35;  // Interval for Task 3

// ===== ESP-NOW Peer Addresses =====
// MAC addresses for broadcasting between ESP32 boards
static const uint8_t macPadXiao[] = {0x34, 0x85, 0x18, 0x9E, 0x87, 0xD4};  // Seeduino XIAO ESP32 S3 controller
static const uint8_t macPlatformMecanum[] = {0xDC, 0xDA, 0x0C, 0x55, 0xD5, 0xB8}; // Mecanum platform ESP32 S3 DEVKIT
static const uint8_t macMonitorDebug[] = {0xA0, 0xB7, 0x65, 0x4B, 0xC5, 0x30}; // ESP32 NodeMCU Dev Kit C V2

// ===== Data Message Structures =====

/**
 * @struct Message_from_Pad
 * @brief Structure for joystick data sent from remote controller to platform.
 */
typedef struct Message_from_Pad {
    uint32_t timeStamp = 0;            // Timestamp (millis) for heartbeat
    uint32_t messageSequenceNumber = 0;// Sequence number of messages sent
    int16_t L_Joystick_x_message = 0;  // Left joystick X-axis value
    int16_t L_Joystick_y_message = 0;  // Left joystick Y-axis value
    int16_t R_Joystick_x_message = 0;  // Right joystick X-axis value
    int16_t R_Joystick_y_message = 0;  // Right joystick Y-axis value
    uint32_t L_Joystick_buttons_message = 0; // Left joystick button states
    uint32_t R_Joystick_buttons_message = 0; // Right joystick button states
    int16_t L_Joystick_raw_x = 0;      // Raw sensor X-value for calibration
    int16_t L_Joystick_raw_y = 0;      // Raw sensor Y-value for calibration
    int16_t R_Joystick_raw_x = 0;      // Raw sensor X-value for calibration
    int16_t R_Joystick_raw_y = 0;      // Raw sensor Y-value for calibration
} Message_from_Pad;

/**
 * @struct Message_from_Platform_Mecanum
 * @brief Structure for telemetry data sent from platform to monitor.
 */
typedef struct Message_from_Platform_Mecanum {
    uint32_t timestamp = 0;            // Timestamp (millis) for heartbeat
    uint32_t totalMessages = 0;        // Total number of messages sent
    float frontLeftSpeedRPM = 0;       // Measured front-left wheel RPM
    float frontRightSpeedRPM = 0;      // Measured front-right wheel RPM
    float rearLeftSpeedRPM = 0;        // Measured rear-left wheel RPM
    float rearRightSpeedRPM = 0;       // Measured rear-right wheel RPM
    int64_t frontLeftEncoder = 0;      // Raw pulse count front-left encoder
    int64_t frontRightEncoder = 0;     // Raw pulse count front-right encoder
    int64_t rearLeftEncoder = 0;       // Raw pulse count rear-left encoder
    int64_t rearRightEncoder = 0;      // Raw pulse count rear-right encoder
    float pitch = 0;                   // IMU pitch angle (future use)
    float roll = 0;                    // IMU roll angle (future use)
    float yaw = 0;                     // IMU yaw angle (future use)
    float batteryVoltage = 0;          // Battery voltage reading (future use)
} Message_from_Platform_Mecanum;

#endif // PARAMETERS_H
