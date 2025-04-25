#pragma once
#include <Arduino.h>
#include "MecanumDrive.h"

//Initialize motor driver and MecanumDrive
void initDrive();

//Update targets wheel RPMs based on joystick input (x, y, yaw)
void moveRPM(int x, int y, int yaw);

//Read target wheel RPMs (frontLeft, frontRight, rearLeft, rearRight)
RPMData readRPMs();

//Set the speed of the motors (output PID)
void applyMotorCommands(int cmdFL, int cmdFR, int cmdRL, int cmdRR);

// Perform PID control for each wheel based on target and measured RPMs, then apply motor commands
// dt: elapsed time in seconds since last call
void updatePID(float dt);