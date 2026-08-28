#include "MecanumDrive.h"

MecanumDrive::MecanumDrive(Motor* fl, Motor* fr, Motor* rl, Motor* rr)
    : _fl(fl), _fr(fr), _rl(rl), _rr(rr) {}

void MecanumDrive::drive(float vx, float vy, float omega) {
    // Mecanum kinematics. NO TWO WHEELS MAY SHARE A PATTERN: if two rows of
    // this mixing matrix become identical it loses rank and the robot becomes
    // physically unable to drive sideways — it turns instead. That bug reached
    // the repo once already (commit d64fbf7, fixed in 76f6cb2) and got through
    // review because the commit message matched the diff exactly. Check these
    // four lines against mecanum maths, not against what a change claims.
    float frontLeftRPM  = vy + vx + omega;
    float frontRightRPM = vy - vx - omega;
    float rearLeftRPM   = vy - vx + omega;
    float rearRightRPM  = vy + vx - omega;

    // Normalisation. The axes add up, so a diagonal drive with rotation can
    // demand three times MAX_RPM. Left alone, the wheels would saturate at
    // DIFFERENT moments and stop holding their relative proportions — and it
    // is those proportions that determine the direction of travel, so the
    // platform would go somewhere other than where the stick points.
    //
    // Dividing ALL FOUR by the same factor limits the speed but preserves the
    // proportions, i.e. keeps the direction faithful.
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
