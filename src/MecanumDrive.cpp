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

    // Normalizacja mocy
    normalizeMotorPower(frontLeft, frontRight, rearLeft, rearRight);

    // Sterowanie silnikami
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

void MecanumDrive::normalizeMotorPower(int& fl, int& fr, int& rl, int& rr) {
    int maxPower = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
    if (maxPower > MAX_SPEED) {
        fl = fl * MAX_SPEED / maxPower;
        fr = fr * MAX_SPEED / maxPower;
        rl = rl * MAX_SPEED / maxPower;
        rr = rr * MAX_SPEED / maxPower;
    }
}
