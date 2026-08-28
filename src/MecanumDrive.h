#pragma once

#include "Motor.h"

/**
 * Mecanum mixer: turns a platform motion command into four wheel speeds.
 *
 * All three inputs are in RPM (already converted from stick units by
 * pad_input.h), so the mixer never has to know anything about the pad.
 */
class MecanumDrive {
public:
    MecanumDrive(Motor* fl, Motor* fr, Motor* rl, Motor* rr);

    /// @param vx     sideways, positive = right
    /// @param vy     forward, positive = ahead
    /// @param omega  rotation, positive = counter-clockwise
    void drive(float vx, float vy, float omega);

    /// Runs one PID step on every wheel.
    void update();

    /// Gentle stop on all four wheels.
    /// @param durationMs ramp length; 0 = the default from MotorConfig
    void softStop(uint32_t durationMs = 0);

    /// Cuts the drive on all four wheels immediately.
    void hardStop();

private:
    Motor* _fl;
    Motor* _fr;
    Motor* _rl;
    Motor* _rr;
};
