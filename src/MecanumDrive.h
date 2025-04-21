#ifndef MECANUMDRIVE_H
#define MECANUMDRIVE_H

#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"

// Structure to hold RPM data for each wheel
struct RPMData {
    float frontLeft;
    float frontRight;
    float rearLeft;
    float rearRight;
};

// Threshold below which joystick input is ignored
#define DEAD_ZONE 15
// Maximum input value for motor speed
#define MAX_SPEED 511

class MecanumDrive {
public:
    /**
     * Constructor: initializes the Mecanum drive with motor driver instances
     * @param frontLeft Pointer to front-left motor driver
     * @param frontRight Pointer to front-right motor driver
     * @param rearLeft Pointer to rear-left motor driver
     * @param rearRight Pointer to rear-right motor driver
     */
    MecanumDrive(
        MotorDriverCytronH_Bridge* frontLeft,
        MotorDriverCytronH_Bridge* frontRight,
        MotorDriverCytronH_Bridge* rearLeft,
        MotorDriverCytronH_Bridge* rearRight
    );

    /**
     * Drive the robot with raw joystick values
     * @param x Lateral movement value
     * @param y Longitudinal movement value
     * @param yaw Rotation value
     */
    void move(int x, int y, int yaw);
    
    /**
     * Drive the robot and calculate target RPMs based on joystick values
     * @param x Lateral movement value
     * @param y Longitudinal movement value
     * @param yaw Rotation value
     */
    void moveRPM(int x, int y, int yaw);

    /**
     * Read the last calculated RPM values for each wheel
     * @return RPMData struct containing four wheel RPMs
     */
    RPMData readRPMs();
    
private:
    /**
     * Normalize motor power so that no wheel command exceeds MAX_SPEED
     */
    void normalizeMotorPower(int& fl, int& fr, int& rl, int& rr);
    
    /**
     * Apply dead zone to joystick inputs to ignore small values
     */
    void applyDeadZone(int& x, int& y, int& yaw);

    /**
     * Convert a raw joystick value to RPM, taking dead zone and MAX_RPM into account
     * @param padValue Raw input value
     * @return Target RPM
     */
    float convertToRPM(int padValue);

    /**
     * Scale input vector (x, y, yaw) if its magnitude exceeds allowed maximum
     */
    void normalizeInputVector(int& x, int& y, int& yaw);

    // Last calculated RPM values for each wheel
    float rpmFL = 0;
    float rpmFR = 0;
    float rpmRL = 0;
    float rpmRR = 0;
    
    // Motor driver pointers for each wheel
    MotorDriverCytronH_Bridge* _frontLeft;
    MotorDriverCytronH_Bridge* _frontRight;
    MotorDriverCytronH_Bridge* _rearLeft;
    MotorDriverCytronH_Bridge* _rearRight;
};

#endif // MECANUMDRIVE_H
