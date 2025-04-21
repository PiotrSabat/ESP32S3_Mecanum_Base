#ifndef ENCODERREADER_H
#define ENCODERREADER_H

#include <ESP32Encoder.h>

// Structure holding encoder readings
struct EncoderData {
    int64_t frontLeft;
    int64_t frontRight;
    int64_t rearLeft;
    int64_t rearRight;
};

class EncoderReader {
public:
    // Constructor – accepts pointers to the encoders and the resolution (pulses per revolution)
    EncoderReader(ESP32Encoder* fl, ESP32Encoder* fr, ESP32Encoder* rl, ESP32Encoder* rr, uint16_t encoderResolution);
    
    // Initialization method to be called in setup()
    void begin();
    
    // Read current encoder counts
    EncoderData readEncoders();
    
    // Reset encoder counts and helper variables (for RPM calculations)
    void resetEncoders();
    
    // Calculate RPM for all encoders – result computed based on the provided resolution
    void getRPMs(float& rpmFL, float& rpmFR, float& rpmRL, float& rpmRR);
    
private:
    ESP32Encoder* encoderFL;
    ESP32Encoder* encoderFR;
    ESP32Encoder* encoderRL;
    ESP32Encoder* encoderRR;
    
    uint16_t _encoderResolution;
    
    // Separate helper variables for each encoder
    unsigned long lastTimeFL;
    unsigned long lastTimeFR;
    unsigned long lastTimeRL;
    unsigned long lastTimeRR;
    
    int64_t lastCountFL;
    int64_t lastCountFR;
    int64_t lastCountRL;
    int64_t lastCountRR;
};

#endif
