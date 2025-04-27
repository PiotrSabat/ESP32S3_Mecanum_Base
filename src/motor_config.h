#pragma once
#include <Arduino.h>
#include "Motor.h"

// --- Definicje konfiguracji dla czterech silników ---
static const MotorConfig FL_CONFIG = {
    // PWM
    .pwmPin1       = FL_PIN1,
    .pwmPin2       = FL_PIN2,
    .pwmChannel1   = FL_CHANNEL1,
    .pwmChannel2   = FL_CHANNEL2,
    // Enkoder
    .encoderPinA   = FL_ENCODER_A,
    .encoderPinB   = FL_ENCODER_B,
    // Przełożenie i PWM
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    // PID
    .Kp            = 0.55,
    .Ki            = 0.03,
    .Kd            = 0.001,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT
};

static const MotorConfig FR_CONFIG = {
    .pwmPin1       = FR_PIN1,
    .pwmPin2       = FR_PIN2,
    .pwmChannel1   = FR_CHANNEL1,
    .pwmChannel2   = FR_CHANNEL2,
    .encoderPinA   = FR_ENCODER_B,
    .encoderPinB   = FR_ENCODER_A,
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 0.55,
    .Ki            = 0.03,
    .Kd            = 0.001,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT
};

static const MotorConfig RL_CONFIG = {
    .pwmPin1       = RL_PIN1,
    .pwmPin2       = RL_PIN2,
    .pwmChannel1   = RL_CHANNEL1,
    .pwmChannel2   = RL_CHANNEL2,
    .encoderPinA   = RL_ENCODER_A,
    .encoderPinB   = RL_ENCODER_B,
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 0.55,
    .Ki            = 0.03,
    .Kd            = 0.001,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT
};

static const MotorConfig RR_CONFIG = {
    .pwmPin1       = RR_PIN1,
    .pwmPin2       = RR_PIN2,
    .pwmChannel1   = RR_CHANNEL1,
    .pwmChannel2   = RR_CHANNEL2,
    .encoderPinA   = RR_ENCODER_B,
    .encoderPinB   = RR_ENCODER_A,
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 0.55,
    .Ki            = 0.03,
    .Kd            = 0.001,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT
};