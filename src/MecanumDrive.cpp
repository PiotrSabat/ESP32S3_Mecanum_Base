#include "MecanumDrive.h"

MecanumDrive::MecanumDrive(Motor* fl, Motor* fr, Motor* rl, Motor* rr)
    : _fl(fl), _fr(fr), _rl(rl), _rr(rr) {}

void MecanumDrive::drive(float vx, float vy, float omega) {
    // Prosta kinematyka dla kół mecanum (zakładamy bezwzględny współczynnik)
    float frontLeftRPM  = vy + vx + omega;
    float frontRightRPM = vy - vx - omega;
    float rearLeftRPM   = vy - vx + omega;
    float rearRightRPM  = vy + vx - omega;

    _fl->setTargetRPM(frontLeftRPM);
    _fr->setTargetRPM(frontRightRPM);
    _rl->setTargetRPM(rearLeftRPM);
    _rr->setTargetRPM(rearRightRPM);
}

void MecanumDrive::update() {
    _fl->update();
    _fr->update();
    _rl->update();
    _rr->update();
}

void MecanumDrive::softStop(uint32_t durationMs) {
    _fl->softStop(durationMs);
    _fr->softStop(durationMs);
    _rl->softStop(durationMs);
    _rr->softStop(durationMs);
}

void MecanumDrive::hardStop() {
    _fl->hardStop();
    _fr->hardStop();
    _rl->hardStop();
    _rr->hardStop();
}

RPMData MecanumDrive::readRPMs() const {
    RPMData data;
    data.frontLeft  = _fl->getCurrentRPM();
    data.frontRight = _fr->getCurrentRPM();
    data.rearLeft   = _rl->getCurrentRPM();
    data.rearRight  = _rr->getCurrentRPM();
    return data;
}
