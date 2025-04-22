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




#endif // PARAMETERS_H
