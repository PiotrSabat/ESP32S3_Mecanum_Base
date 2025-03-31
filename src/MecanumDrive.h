#ifndef MECANUMDRIVE_H
#define MECANUMDRIVE_H

#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"

struct RPMData {
    float frontLeft;
    float frontRight;
    float rearLeft;
    float rearRight;
};


#define DEAD_ZONE 15
#define MAX_SPEED 511

class MecanumDrive {
public:
    MecanumDrive(
        MotorDriverCytronH_Bridge* frontLeft,
        MotorDriverCytronH_Bridge* frontRight,
        MotorDriverCytronH_Bridge* rearLeft,
        MotorDriverCytronH_Bridge* rearRight
    );

    void move(int x, int y, int yaw);
    void moveRPM(int x, int y, int yaw);
    RPMData readRPMs();
    


private:
    void normalizeMotorPower(int& fl, int& fr, int& rl, int& rr);
    void applyDeadZone(int& x, int& y, int& yaw);
    float convertToRPM(int padValue);
    void normalizeInputVector(int& x, int& y, int& yaw);


    float rpmFL = 0;
    float rpmFR = 0;
    float rpmRL = 0;
    float rpmRR = 0;
    

    MotorDriverCytronH_Bridge* _frontLeft;
    MotorDriverCytronH_Bridge* _frontRight;
    MotorDriverCytronH_Bridge* _rearLeft;
    MotorDriverCytronH_Bridge* _rearRight;
};

#endif
