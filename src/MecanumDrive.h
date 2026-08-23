#pragma once

#include "Motor.h"
#include "RPMData.h"

class MecanumDrive {
public:
    MecanumDrive(Motor* fl, Motor* fr, Motor* rl, Motor* rr);

    void drive(float vx, float vy, float omega);
    void update();

    /// Łagodne zatrzymanie wszystkich kół.
    /// @param durationMs czas wyhamowania; 0 = wartość domyślna z konfiguracji silnika
    void softStop(uint32_t durationMs = 0);
    void hardStop();

    RPMData readRPMs() const;

private:
    Motor* _fl;
    Motor* _fr;
    Motor* _rl;
    Motor* _rr;
};
