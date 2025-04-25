#include "sensors.h"
#include "parameters.h"

// Encoder objects for each wheel
static ESP32Encoder frontLeftEncoder;
static ESP32Encoder frontRightEncoder;
static ESP32Encoder rearLeftEncoder;
static ESP32Encoder rearRightEncoder;

// Reader that wraps all encoders and computes RPM
static EncoderReader encoderReader(
    &frontLeftEncoder,
    &frontRightEncoder,
    &rearLeftEncoder,
    &rearRightEncoder,
    ENCODER_RESOLUTION
);

void initEncoders() {
    // Attach encoder signal pins (single-edge mode)
    frontLeftEncoder.attachSingleEdge(FL_ENCODER_A, FL_ENCODER_B);
    frontRightEncoder.attachSingleEdge(FR_ENCODER_A, FR_ENCODER_B);
    rearLeftEncoder.attachSingleEdge(RL_ENCODER_A, RL_ENCODER_B);
    rearRightEncoder.attachSingleEdge(RR_ENCODER_A, RR_ENCODER_B);

    // Initialize the reader
    encoderReader.begin();
}

EncoderData readEncoders() {
    // Return raw counts
    return encoderReader.readEncoders();
}

RPMData getRPMs() {
    // Compute RPMs from encoder counts
    float fl, fr, rl, rr;
    encoderReader.getRPMs(fl, fr, rl, rr);
    RPMData rpm = { fl, fr, rl, rr };
    return rpm;
}
