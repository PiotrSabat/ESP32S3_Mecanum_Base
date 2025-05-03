#pragma once

#include "Motor.h"
#include "RPMData.h"

class MecanumDrive {
public:
    MecanumDrive(Motor* fl, Motor* fr, Motor* rl, Motor* rr);

    void drive(float vx, float vy, float omega);
    void update();

    void softStop();
    void hardStop();

    RPMData readRPMs() const;

private:
    Motor* _fl;
    Motor* _fr;
    Motor* _rl;
    Motor* _rr;
};
