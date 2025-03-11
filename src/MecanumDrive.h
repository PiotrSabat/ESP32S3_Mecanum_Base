#ifndef MECANUMDRIVE_H
#define MECANUMDRIVE_H

#include "MotorDriverCytronH_Bridge.h"

#define DEAD_ZONE 15
#define MAX_SPEED 255

class MecanumDrive {
public:
    MecanumDrive(
        MotorDriverCytronH_Bridge* frontLeft,
        MotorDriverCytronH_Bridge* frontRight,
        MotorDriverCytronH_Bridge* rearLeft,
        MotorDriverCytronH_Bridge* rearRight
    );

    void move(int x, int y, int yaw);
    int getCurrentSpeed();

private:
    void normalizeMotorPower(int& fl, int& fr, int& rl, int& rr);
    void applyDeadZone(int& x, int& y, int& yaw);

    MotorDriverCytronH_Bridge* _frontLeft;
    MotorDriverCytronH_Bridge* _frontRight;
    MotorDriverCytronH_Bridge* _rearLeft;
    MotorDriverCytronH_Bridge* _rearRight;
};

#endif
