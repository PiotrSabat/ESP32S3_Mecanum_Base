#pragma once

#include <Arduino.h>

// ===== Motor PWM Definitions =====
// PWM signal frequency for motor drivers: 20 kHz, 9-bit resolution

// Front Left Motor
constexpr int FL_PIN1 = 9;        // M1A
constexpr int FL_PIN2 = 10;       // M1B
constexpr int FL_CHANNEL1 = 0;    // PWM channel for M1A
constexpr int FL_CHANNEL2 = 1;    // PWM channel for M1B

// Front Right Motor
constexpr int FR_PIN1 = 11;       // M2A
constexpr int FR_PIN2 = 12;       // M2B
constexpr int FR_CHANNEL1 = 2;
constexpr int FR_CHANNEL2 = 3;

// Rear Left Motor
constexpr int RL_PIN1 = 13;       // M2A
constexpr int RL_PIN2 = 14;       // M2B
constexpr int RL_CHANNEL1 = 4;
constexpr int RL_CHANNEL2 = 5;

// Rear Right Motor
constexpr int RR_PIN1 = 15;       // M1A
constexpr int RR_PIN2 = 16;       // M1B
constexpr int RR_CHANNEL1 = 6;
constexpr int RR_CHANNEL2 = 7;

// ===== Encoder Pin Definitions =====

// Front Left Encoder
constexpr int FL_ENCODER_A = 1;
constexpr int FL_ENCODER_B = 2;

// Front Right Encoder
constexpr int FR_ENCODER_A = 4;
constexpr int FR_ENCODER_B = 5;

// Rear Left Encoder
constexpr int RL_ENCODER_A = 6;
constexpr int RL_ENCODER_B = 7;

// Rear Right Encoder
constexpr int RR_ENCODER_A = 17;
constexpr int RR_ENCODER_B = 18;

// ===== Encoder and Motor Configuration =====


constexpr int MAX_RPM = 180;             // Maximum motor speed

// ===== Task Scheduling Rates (in milliseconds) =====

constexpr int INTERVAL_MOTOR_CONTROL = 50;   // Interval for motor control task
constexpr int INTERVAL_SENSOR_READ = 25;     // Interval for sensor read task
constexpr int INTERVAL_DEBUG_OUTPUT = 35;    // Interval for debug/telemetry

// ===== PID Control Constants =====

constexpr float KP = 0.5f;
constexpr float KI = 0.0f;
constexpr float KD = 0.00f;
constexpr float MAX_OUT = 511.0f;
constexpr float MIN_OUT = -511.0f;

// ===== Timing Constants =====
constexpr TickType_t INTERVAL_1MS = pdMS_TO_TICKS(1);
constexpr TickType_t INTERVAL_5MS = pdMS_TO_TICKS(5);
constexpr TickType_t INTERVAL_10MS = pdMS_TO_TICKS(10);
constexpr TickType_t INTERVAL_20MS = pdMS_TO_TICKS(20);
constexpr TickType_t INTERVAL_50MS = pdMS_TO_TICKS(50);
constexpr TickType_t INTERVAL_100MS = pdMS_TO_TICKS(100);

// ===== ESP-NOW Configuration =====
constexpr int ESP_CHANNEL = 0;  // ESP-NOW channel
constexpr int ESP_MAX_DATA_SIZE = 250;  // Maximum data size for ESP-NOW


// ===== Default Motor Configuration =====
constexpr int DEFAULT_GEAR_RATIO = 960; // Gear ratio for the motors
constexpr int DEFAULT_PWM_RESOLUTION = 9; // PWM resolution (9 bits)
constexpr int DEFAULT_PWM_FREQUENCY = 20000; // PWM frequency (20 kHz)