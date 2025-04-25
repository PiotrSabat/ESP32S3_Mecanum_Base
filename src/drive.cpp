#include "drive.h"
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"
#include "MecanumDrive.h"
#include "sensors.h"
#include "PIDController.h"



// --- Motor driver instances for each wheel ---
static MotorDriverCytronH_Bridge rearRight(
    RR_PIN2, RR_PIN1, RR_CHANNEL2, RR_CHANNEL1
);
static MotorDriverCytronH_Bridge frontRight(
    FR_PIN2, FR_PIN1, FR_CHANNEL2, FR_CHANNEL1
);
static MotorDriverCytronH_Bridge frontLeft(
    FL_PIN1, FL_PIN2, FL_CHANNEL1, FL_CHANNEL2
);
static MotorDriverCytronH_Bridge rearLeft(
    RL_PIN1, RL_PIN2, RL_CHANNEL1, RL_CHANNEL2
);

// --- Mecanum drive vectoring instance ---
static MecanumDrive drive(&frontLeft, &frontRight, &rearLeft, &rearRight);

// --- PID controllers (internal to module) ---
static PIDController pidFL(KP, KI, KD, MAX_OUT, MIN_OUT);
static PIDController pidFR(KP, KI, KD, MAX_OUT, MIN_OUT);
static PIDController pidRL(KP, KI, KD, MAX_OUT, MIN_OUT);
static PIDController pidRR(KP, KI, KD, MAX_OUT, MIN_OUT);

//--------------------------------------------------------------------------------
// Function: initDrive
// Description: Stops all motors and resets any internal drive state.
//--------------------------------------------------------------------------------
void initDrive() {
    frontLeft.setSpeed(0);
    frontRight.setSpeed(0);
    rearLeft.setSpeed(0);
    rearRight.setSpeed(0);
}

//--------------------------------------------------------------------------------
// Function: moveRPM
// Description: Updates the target wheel RPMs based on joystick inputs.
// Parameters:
//   x   - lateral (strafe) command
//   y   - forward/backward command
//   yaw - rotational command
//--------------------------------------------------------------------------------
void moveRPM(int x, int y, int yaw) {
    drive.moveRPM(x, y, yaw);
}

//--------------------------------------------------------------------------------
// Function: readRPMs
// Description: Returns the current target wheel RPMs as computed by MecanumDrive.
//--------------------------------------------------------------------------------
RPMData readRPMs() {
    return drive.readRPMs();
}

//--------------------------------------------------------------------------------
// Function: updatePID
// Description: Performs one PID control iteration for each wheel and applies
//              the resulting motor commands.
// Parameters:
//   dt - elapsed time in seconds since last PID update
//--------------------------------------------------------------------------------
void updatePID(float dt) {
    // 1) Read target RPMs
    RPMData target = readRPMs();
    // 2) Read measured RPMs from encoders
    RPMData measured = getRPMs();

    // 3) Compute PID outputs
    int cmdFL = (int)pidFL.compute(target.frontLeft,  measured.frontLeft,  dt);
    int cmdFR = (int)pidFR.compute(target.frontRight, measured.frontRight, dt);
    int cmdRL = (int)pidRL.compute(target.rearLeft,   measured.rearLeft,   dt);
    int cmdRR = (int)pidRR.compute(target.rearRight,  measured.rearRight,  dt);

    // 4) Apply motor commands
    frontLeft.setSpeed(cmdFL);
    frontRight.setSpeed(cmdFR);
    rearLeft.setSpeed(cmdRL);
    rearRight.setSpeed(cmdRR);
}
