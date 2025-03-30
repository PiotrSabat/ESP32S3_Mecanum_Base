#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float maxOutput, float minOutput)
    : _kp(kp), _ki(ki), _kd(kd), _maxOutput(maxOutput), _minOutput(minOutput),
      _integral(0.0), _prevError(0.0)
{
}

void PIDController::reset() {
    _integral = 0.0;
    _prevError = 0.0;
}

float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    _integral += error * dt;
    float derivative = (error - _prevError) / dt;
    float output = _kp * error + _ki * _integral + _kd * derivative;

    // Ograniczenie wyjścia do zadanych wartości
    if (output > _maxOutput)
        output = _maxOutput;
    else if (output < _minOutput)
        output = _minOutput;

    _prevError = error;
    return output;
}
