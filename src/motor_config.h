#pragma once
#include <Arduino.h>
#include "Motor.h"

// PID GAIN UNITS: computePID() receives dt in MILLISECONDS, not seconds.
// That is why Kd = 100 next to Kp = 6 is not a typo — in second-based units it
// would read Kd = 0.1, and Ki = 0.06 would read Ki = 60. Before changing any
// value, convert: Ki_ms = Ki_s / 1000, Kd_ms = Kd_s * 1000.
//
// The gains were DOUBLED on 2026-08-28 together with the encoder count fix.
// Measured and commanded speed both halved at that point, so the error halved
// too — doubling the gains keeps the controller output IDENTICAL. That was a
// change of units, not a retune.

// --- Configuration for the four motors ---
static const MotorConfig FL_CONFIG = {
    // PWM
    .pwmPin1       = FL_PIN1,
    .pwmPin2       = FL_PIN2,
    .pwmChannel1   = FL_CHANNEL1,
    .pwmChannel2   = FL_CHANNEL2,
    // Encoder
    .encoderPinA   = FL_ENCODER_A,
    .encoderPinB   = FL_ENCODER_B,
    // Wiring direction (true = this side is mounted mirrored)
    .invertDirection = false,
    // Gearing and PWM
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    // PID
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};

static const MotorConfig FR_CONFIG = {
    .pwmPin1       = FR_PIN1,
    .pwmPin2       = FR_PIN2,
    .pwmChannel1   = FR_CHANNEL1,
    .pwmChannel2   = FR_CHANNEL2,
    .encoderPinA   = FR_ENCODER_A,
    .encoderPinB   = FR_ENCODER_B,
    // Wiring direction (true = this side is mounted mirrored)
    .invertDirection = true,
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};

static const MotorConfig RL_CONFIG = {
    .pwmPin1       = RL_PIN1,
    .pwmPin2       = RL_PIN2,
    .pwmChannel1   = RL_CHANNEL1,
    .pwmChannel2   = RL_CHANNEL2,
    .encoderPinA   = RL_ENCODER_A,
    .encoderPinB   = RL_ENCODER_B,
    // Wiring direction (true = this side is mounted mirrored)
    .invertDirection = false,
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};

static const MotorConfig RR_CONFIG = {
    .pwmPin1       = RR_PIN1,
    .pwmPin2       = RR_PIN2,
    .pwmChannel1   = RR_CHANNEL1,
    .pwmChannel2   = RR_CHANNEL2,
    .encoderPinA   = RR_ENCODER_A,
    .encoderPinB   = RR_ENCODER_B,
    // Wiring direction (true = this side is mounted mirrored)
    .invertDirection = true,
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};