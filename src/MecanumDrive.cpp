#include "MecanumDrive.h"

MecanumDrive::MecanumDrive(
    MotorDriverCytronH_Bridge* frontLeft,
    MotorDriverCytronH_Bridge* frontRight,
    MotorDriverCytronH_Bridge* rearLeft,
    MotorDriverCytronH_Bridge* rearRight
) {
    _frontLeft = frontLeft;
    _frontRight = frontRight;
    _rearLeft = rearLeft;
    _rearRight = rearRight;
}

void MecanumDrive::move(int x, int y, int yaw) {
    applyDeadZone(x, y, yaw);

    int frontLeft  = y + x + yaw;
    int frontRight = y - x - yaw;
    int rearLeft   = y - x + yaw;
    int rearRight  = y + x - yaw;

    // Normalize motor power
    normalizeMotorPower(frontLeft, frontRight, rearLeft, rearRight);

    // Control motors
    _frontLeft->setSpeed(frontLeft);
    _frontRight->setSpeed(frontRight);
    _rearLeft->setSpeed(rearLeft);
    _rearRight->setSpeed(rearRight);
}

void MecanumDrive::applyDeadZone(int& x, int& y, int& yaw) {
    if (abs(x) < DEAD_ZONE) x = 0;
    if (abs(y) < DEAD_ZONE) y = 0;
    if (abs(yaw) < DEAD_ZONE) yaw = 0;
}

// Prevent exceeding the maximum speed; can be removed if PID works correctly
void MecanumDrive::normalizeMotorPower(int& fl, int& fr, int& rl, int& rr) {
    int maxPower = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
    if (maxPower > MAX_SPEED) {
        fl = fl * MAX_SPEED / maxPower;
        fr = fr * MAX_SPEED / maxPower;
        rl = rl * MAX_SPEED / maxPower;
        rr = rr * MAX_SPEED / maxPower;
    }
}

// New methods: attempts to integrate PID control

float MecanumDrive::convertToRPM(int padValue) {
    // If value is within the dead zone, return 0
    if (abs(padValue) < DEAD_ZONE) {
        return 0.0;
    }

    // Preserve sign (direction)
    int sign = (padValue > 0) ? 1 : -1;

    // Effective value is absolute value minus the dead zone
    int effectiveValue = abs(padValue) - DEAD_ZONE;

    // Maximum effective value; assuming input range is -511 to 511
    int maxEffectiveValue = MAX_SPEED - DEAD_ZONE;

    // Protect against division by zero
    if (maxEffectiveValue == 0) {
        return 0.0;
    }

    // Map effective value to RPM range: if effectiveValue == maxEffectiveValue then RPM == MAX_RPM
    float rpm = (effectiveValue / (float)maxEffectiveValue) * MAX_RPM;
    return sign * rpm;
}

void MecanumDrive::moveRPM(int x, int y, int yaw) {
    // Apply dead zone
    applyDeadZone(x, y, yaw);

    normalizeInputVector(x, y, yaw);
    
    // Compute raw values for each wheel
    int rawFL = y + x + yaw;
    int rawFR = y - x - yaw;
    int rawRL = y - x + yaw;
    int rawRR = y + x - yaw;

    // Perform standard movement
    move(x, y, yaw);

    // Convert controller values to RPM
    rpmFL = convertToRPM(rawFL);
    rpmFR = convertToRPM(rawFR);
    rpmRL = convertToRPM(rawRL);
    rpmRR = convertToRPM(rawRR);
}

// Normalize input vector (x, y, yaw); if its magnitude > MAX_INPUT (e.g., 511), scale all components
void MecanumDrive::normalizeInputVector(int &x, int &y, int &yaw) {
    float norm = sqrt((float)x * x + (float)y * y + (float)yaw * yaw);
    const float MAX_INPUT_PWM = 511.0; // maximum allowed input magnitude
    if (norm > MAX_INPUT_PWM) {
        float scale = MAX_INPUT_PWM / norm;
        x = (int)(x * scale);
        y = (int)(y * scale);
        yaw = (int)(yaw * scale);
    }
}

RPMData MecanumDrive::readRPMs() {
    RPMData data;
    data.frontLeft = rpmFL;
    data.frontRight = rpmFR;
    data.rearLeft = rpmRL;
    data.rearRight = rpmRR;
    return data;
}
