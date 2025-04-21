#include "PIDController.h"

/**
 * @class PIDController
 * @brief A basic PID control loop implementation.
 *
 * Contains proportional, integral, and derivative terms, with output clamping.
 */

PIDController::PIDController(float kp, float ki, float kd, float maxOutput, float minOutput)
    : _kp(kp), _ki(ki), _kd(kd), _maxOutput(maxOutput), _minOutput(minOutput),
      _integral(0.0), _prevError(0.0)
{
    // Initialize integral and previous error to zero
}

/**
 * @brief Reset the internal state of the controller (integral and previous error).
 */
void PIDController::reset() {
    _integral = 0.0;
    _prevError = 0.0;
}

/**
 * @brief Compute the PID output for a given setpoint and measurement.
 *
 * Calculates error = setpoint - measured, updates integral and derivative terms,
 * and clamps the final output between configured min/max values.
 *
 * @param setpoint Desired target value.
 * @param measured Current measured value.
 * @param dt Time elapsed since last update in seconds.
 * @return PID controller output (clamped).
 */
float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    _integral += error * dt;
    float derivative = (error - _prevError) / dt;
    float output = _kp * error + _ki * _integral + _kd * derivative;

    // Clamp output to configured limits
    if (output > _maxOutput)
        output = _maxOutput;
    else if (output < _minOutput)
        output = _minOutput;

    _prevError = error;
    return output;
}
