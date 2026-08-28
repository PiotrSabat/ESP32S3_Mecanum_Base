#pragma once
#include <cmath>
#include "parameters.h"

// =====================================================================
//  Stick deflection -> the quantities the mecanum mixer consumes.
//
//  This header exists so that the firmware and the simulators compute it
//  with ONE piece of code. The simulator used to keep its own copy of
//  padAxisToRPM with a comment saying "same arithmetic" — exactly the kind
//  of duplicate that silently drifts apart and invalidates the tests.
//
//  Deliberately free of any Arduino.h dependency, so the simulators build
//  on a desktop machine.
// =====================================================================

/// Drive axis: stick deflection -> wheel speed in RPM.
/// Outside the dead zone the scale is linear, and stretched so that speed
/// starts from zero right at the edge of the zone — otherwise the stick would
/// "catch" with a jump.
inline float padAxisToRPM(int16_t raw) {
    float v = (float)raw;
    if (v >  (float)JOYSTICK_MAX) v =  (float)JOYSTICK_MAX;
    if (v < -(float)JOYSTICK_MAX) v = -(float)JOYSTICK_MAX;
    if (fabsf(v) < (float)JOYSTICK_DEADZONE) return 0.0f;

    float sign = (v > 0.0f) ? 1.0f : -1.0f;
    float mag  = (fabsf(v) - (float)JOYSTICK_DEADZONE) /
                 (float)(JOYSTICK_MAX - JOYSTICK_DEADZONE);
    return sign * mag * (float)MAX_RPM;
}

/// Stick expo curve. Preserves sign and the full range — it only redistributes
/// sensitivity, concentrating it around centre.
inline float applyExpo(float norm, float expo) {
    return (1.0f - expo) * norm + expo * norm * norm * norm;
}

/// Turn axis: stick deflection -> the rotation term, in RPM.
///
/// The right stick controls how TIGHT the arc is rather than the rotation rate
/// directly: while driving, the rotation term scales with speed, so a small
/// stick movement gives a wide arc even at full throttle. With the platform
/// stopped the stick reverts to spinning in place, because an arc at zero
/// speed does not exist.
inline float padAxisToOmega(int16_t rawYaw, float vx, float vy) {
    float v = (float)rawYaw;
    if (v >  (float)JOYSTICK_MAX) v =  (float)JOYSTICK_MAX;
    if (v < -(float)JOYSTICK_MAX) v = -(float)JOYSTICK_MAX;
    if (fabsf(v) < (float)JOYSTICK_DEADZONE) return 0.0f;

    float sign = (v > 0.0f) ? 1.0f : -1.0f;
    float mag  = (fabsf(v) - (float)JOYSTICK_DEADZONE) /
                 (float)(JOYSTICK_MAX - JOYSTICK_DEADZONE);
    float stick = sign * applyExpo(mag, TURN_EXPO);

    // The LENGTH of the travel vector, not just the forward component: with
    // mecanum wheels, driving sideways is a first-class motion and an arc
    // makes sense there too.
    float speed = sqrtf(vx * vx + vy * vy);

    float omegaArc   = stick * speed * TURN_GAIN;   // arc — grows with speed
    float omegaPivot = stick * (float)MAX_RPM;      // pivot — full authority at rest

    float blend = speed / PIVOT_BLEND_RPM;
    if (blend > 1.0f) blend = 1.0f;

    return omegaPivot * (1.0f - blend) + omegaArc * blend;
}
