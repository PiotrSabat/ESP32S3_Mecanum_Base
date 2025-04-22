#ifndef MESSAGES_H
#define MESSAGES_H
#include <Arduino.h>
// ===== Data Message Structures =====

/**
 * @struct Message_from_Pad
 * @brief Structure for joystick data sent from remote controller to platform.
 */
typedef struct Message_from_Pad {
    uint32_t timeStamp;            // Timestamp (millis) for heartbeat
    uint32_t messageSequenceNumber;// Sequence number of messages sent

    int16_t L_Joystick_x_message;  // Left joystick X-axis value
    int16_t L_Joystick_y_message;  // Left joystick Y-axis value
    int16_t R_Joystick_x_message;  // Right joystick X-axis value
    int16_t R_Joystick_y_message;  // Right joystick Y-axis value

    uint32_t L_Joystick_buttons_message; // Left joystick button states
    uint32_t R_Joystick_buttons_message; // Right joystick button states
    
    int16_t L_Joystick_raw_x;      // Raw sensor X-value for calibration
    int16_t L_Joystick_raw_y;      // Raw sensor Y-value for calibration
    int16_t R_Joystick_raw_x;      // Raw sensor X-value for calibration
    int16_t R_Joystick_raw_y;      // Raw sensor Y-value for calibration
} Message_from_Pad;

/**
/**
 * @struct Message_from_Platform_Mecanum
 * @brief Structure for telemetry data sent from platform to monitor.
 */
typedef struct Message_from_Platform_Mecanum {
    uint32_t timestamp;
    uint32_t totalMessages;
    float frontLeftSpeedRPM;
    float frontRightSpeedRPM;
    float rearLeftSpeedRPM;
    float rearRightSpeedRPM;
    int64_t frontLeftEncoder;
    int64_t frontRightEncoder;
    int64_t rearLeftEncoder;
    int64_t rearRightEncoder;
    float pitch;
    float roll;
    float yaw;
    float batteryVoltage;
    // PID Controller Parameters (snapshot for debug)
    float KP_message;
    float KI_message;
    float KD_message;
    float MAX_OUT_message;
    float MIN_OUT_message;
} Message_from_Platform_Mecanum;

// Sanity check to avoid oversizing ESP-NOW messages
static_assert(sizeof(Message_from_Pad) <= 230, "Message_from_Pad too large for ESP-NOW");
static_assert(sizeof(Message_from_Platform_Mecanum) <= 230, "Message_from_Platform_Mecanum too large for ESP-NOW");


#endif // MESSAGES_H