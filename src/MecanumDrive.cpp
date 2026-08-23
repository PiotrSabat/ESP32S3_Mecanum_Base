#include "MecanumDrive.h"

MecanumDrive::MecanumDrive(Motor* fl, Motor* fr, Motor* rl, Motor* rr)
    : _fl(fl), _fr(fr), _rl(rl), _rr(rr) {}

void MecanumDrive::drive(float vx, float vy, float omega) {
    // Kinematyka kół mecanum. vx, vy i omega są w RPM (przeliczone z drążka).
    float frontLeftRPM  = vy + vx + omega;
    float frontRightRPM = vy - vx - omega;
    float rearLeftRPM   = vy - vx + omega;
    float rearRightRPM  = vy + vx - omega;

    // Normalizacja. Osie sumują się, więc przy jeździe po skosie z obrotem
    // żądanie może sięgnąć trzykrotności MAX_RPM. Gdyby zostawić to bez zmian,
    // koła nasycałyby się w RÓŻNYCH momentach i przestały zachowywać wzajemne
    // proporcje — a to one decydują o kierunku jazdy, więc platforma jechałaby
    // nie tam, gdzie wskazuje drążek.
    //
    // Dzielenie WSZYSTKICH czterech przez ten sam współczynnik ogranicza
    // prędkość, ale zachowuje proporcje, czyli wierność kierunku.
    float maxMag = fmaxf(fmaxf(fabsf(frontLeftRPM), fabsf(frontRightRPM)),
                         fmaxf(fabsf(rearLeftRPM),  fabsf(rearRightRPM)));
    if (maxMag > (float)MAX_RPM) {
        float scale = (float)MAX_RPM / maxMag;
        frontLeftRPM  *= scale;
        frontRightRPM *= scale;
        rearLeftRPM   *= scale;
        rearRightRPM  *= scale;
    }

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
