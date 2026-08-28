#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "parameters.h"

/**
 * Per-motor configuration: pins, gearing, PWM and PID.
 * Instances live in motor_config.h, built from the constants in parameters.h.
 */
struct MotorConfig {
    int   pwmPin1;          ///< PWM pin, channel 1 (one direction)
    int   pwmPin2;          ///< PWM pin, channel 2 (the other direction)
    int   pwmChannel1;      ///< LEDC channel for pwmPin1
    int   pwmChannel2;      ///< LEDC channel for pwmPin2
    int   encoderPinA;      ///< Encoder channel A
    int   encoderPinB;      ///< Encoder channel B
    bool  invertDirection;  ///< true = this motor is wired the other way round
    float gearRatio;        ///< Encoder COUNTS per wheel revolution (not pulses)
    int   pwmResolution;    ///< PWM resolution in bits
    int   pwmFrequency;     ///< PWM frequency in Hz
    float Kp;               ///< Proportional gain (note: dt is in MILLISECONDS)
    float Ki;               ///< Integral gain
    float Kd;               ///< Derivative gain
    float outputMin;        ///< Lower controller output bound (e.g. -maxPWM)
    float outputMax;        ///< Upper controller output bound (e.g. +maxPWM)
    // --- Safety ---
    uint32_t softStopDurationMs;  ///< Default soft-stop ramp length (ms)
    uint32_t hardStopDurationMs;  ///< Default hard-stop duration (ms)
};

/**
 * Operating state of a motor — used by the emergency stop paths.
 */
enum class MotorState {
    Active,         ///< Normal operation
    SoftStopping,   ///< Soft stop ramp in progress
    HardStopped     ///< Drive cut immediately
};

/**
 * One DC motor with an encoder and a PID speed controller.
 *
 * Usage:
 *   1. Define a MotorConfig (see motor_config.h).
 *   2. Construct:  Motor motorFL(FL_CONFIG);
 *      The constructor configures pins, PWM and the encoder — nothing else
 *      is needed in setup().
 *   3. Drive it:
 *        motorFL.setTargetRPM(desiredRPM);
 *        motorFL.update();            // every INTERVAL_MOTOR_CONTROL ms
 *        float rpm = motorFL.getCurrentRPM();
 */
class Motor {
public:
    explicit Motor(const MotorConfig& config);

    /// Sets the commanded speed in RPM. Applies invertDirection, so callers
    /// always work in the robot convention (positive = forward).
    void setTargetRPM(float rpm);

    /**
     * Control loop step: read the encoder, derive speed, run one PID
     * iteration, update the PWM outputs.
     * Call every INTERVAL_MOTOR_CONTROL milliseconds.
     */
    void update();

    /// Last measured speed in RPM (motor convention, i.e. inverted on the
    /// right-hand side).
    float getCurrentRPM() const;

    /// Currently commanded speed in RPM (same convention as above).
    float getTargetRPM() const;

    /// Last controller output, in PWM units.
    int getControlOutput() const;

    /**
     * Starts a gentle stop: the commanded speed is ramped down to zero over
     * durationMs, with the PID still regulating.
     * @param durationMs  ramp length; 0 = the value from MotorConfig
     */
    void softStop(uint32_t durationMs = 0);

    /// Cuts the drive immediately (a few milliseconds).
    void hardStop();

    /// Runtime PID retuning, driven remotely via MSG_SET_PID.
    void setPID(float Kp, float Ki, float Kd);
    void setOutputLimits(int min, int max);

    float getKp() const;
    float getKi() const;
    float getKd() const;
    int   getOutputMin() const;
    int   getOutputMax() const;

private:
    MotorConfig  _cfg;
    ESP32Encoder _encoder;
    int          _maxPwmValue;      ///< 2^resolution - 1

    // --- PID state ---
    float _targetRPM    = 0.0f;     ///< Commanded speed
    float _rampedTarget = 0.0f;     ///< Commanded speed after the accel limit
    float _currentRPM   = 0.0f;
    float _errorSum     = 0.0f;
    float _lastError    = 0.0f;
    int   _controlOut   = 0;

    int64_t  _lastCount  = 0;       ///< Previous encoder reading
    uint32_t _lastTimeMs = 0;       ///< Timestamp of that reading

    // --- Stop handling ---
    MotorState _state;
    uint32_t   _softStopStartMs;    ///< When the soft stop began
    uint32_t   _softStopDurationMs; ///< How long it should take
    float      _initialTargetRPM;   ///< Commanded speed when it began

    // --- Live-tunable PID (copies of the config values) ---
    float _Kp;
    float _Ki;
    float _Kd;
    float _outputMin;
    float _outputMax;

    void setupPWM();
    void setupEncoder();

    /**
     * One PID step.
     * @param error  targetRPM - currentRPM
     * @param dt     time since the previous call, in MILLISECONDS
     * @return       controller output in PWM units
     */
    float computePID(float error, float dt);
};
