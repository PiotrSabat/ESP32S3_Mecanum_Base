#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

/**
 * @class PIDController
 * @brief Basic PID (Proportional-Integral-Derivative) controller implementation.
 *
 * This class calculates a control output based on the difference between a
 * setpoint and a measured value, using configurable PID gains and output limits.
 */
class PIDController {
public:
    /**
     * Constructor: initializes PID gains and output clamping limits.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param maxOutput Maximum output value (upper clamp).
     * @param minOutput Minimum output value (lower clamp), default -1.0.
     */
    PIDController(float kp, float ki, float kd, float maxOutput, float minOutput = -1.0);
    
    /**
     * Reset the internal state of the controller.
     * Clears the accumulated integral term and previous error.
     * Useful when control conditions or setpoints change significantly.
     */
    void reset();
    
    /**
     * Compute the PID control output.
     * @param setpoint Desired target value.
     * @param measured Currently measured value.
     * @param dt Time elapsed since last compute call (in seconds).
     * @return Control output clamped to configured limits.
     */
    float compute(float setpoint, float measured, float dt);

private:
    float _kp;          ///< Proportional gain
    float _ki;          ///< Integral gain
    float _kd;          ///< Derivative gain
    float _maxOutput;   ///< Upper clamp for output
    float _minOutput;   ///< Lower clamp for output
    float _integral;    ///< Accumulated integral term
    float _prevError;   ///< Previous error for derivative calculation
};

#endif // PIDCONTROLLER_H
