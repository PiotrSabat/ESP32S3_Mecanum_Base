#include "EncoderReader.h"
#include <Arduino.h>

EncoderReader::EncoderReader(ESP32Encoder* fl, ESP32Encoder* fr, ESP32Encoder* rl, ESP32Encoder* rr)
    : encoderFL(fl), encoderFR(fr), encoderRL(rl), encoderRR(rr)
{
    // Inicjalizacja oddzielnych znaczników czasu i liczników
    lastTimeFL = millis();
    lastTimeFR = millis();
    lastTimeRL = millis();
    lastTimeRR = millis();

    lastCountFL = encoderFL->getCount();
    lastCountFR = encoderFR->getCount();
    lastCountRL = encoderRL->getCount();
    lastCountRR = encoderRR->getCount();
}

EncoderData EncoderReader::readEncoders() {
    EncoderData data;
    data.frontLeft  = encoderFL->getCount();
    data.frontRight = encoderFR->getCount();
    data.rearLeft   = encoderRL->getCount();
    data.rearRight  = encoderRR->getCount();
    return data;
}

void EncoderReader::resetEncoders() {
    encoderFL->clearCount();
    encoderFR->clearCount();
    encoderRL->clearCount();
    encoderRR->clearCount();
    
    lastTimeFL = millis();
    lastTimeFR = millis();
    lastTimeRL = millis();
    lastTimeRR = millis();
    
    lastCountFL = 0;
    lastCountFR = 0;
    lastCountRL = 0;
    lastCountRR = 0;
}

float EncoderReader::calculateRPM(ESP32Encoder* encoder, int64_t& lastCount, unsigned long& lastTime, uint16_t pulsesPerRevolution) {
    unsigned long currentTime = millis();
    int64_t currentCount = encoder->getCount();
    unsigned long deltaTime = currentTime - lastTime;
    int64_t deltaCount = currentCount - lastCount;

    // Aktualizacja zmiennych pomocniczych dla tego enkodera
    lastTime = currentTime;
    lastCount = currentCount;

    if (deltaTime == 0) return 0.0;
    float rpm = (deltaCount * 60000.0) / (pulsesPerRevolution * deltaTime);
    return rpm;
}

void EncoderReader::getRPMs(uint16_t pulsesPerRevolution, float& rpmFL, float& rpmFR, float& rpmRL, float& rpmRR) {
    rpmFL = calculateRPM(encoderFL, lastCountFL, lastTimeFL, pulsesPerRevolution);
    rpmFR = calculateRPM(encoderFR, lastCountFR, lastTimeFR, pulsesPerRevolution);
    rpmRL = calculateRPM(encoderRL, lastCountRL, lastTimeRL, pulsesPerRevolution);
    rpmRR = calculateRPM(encoderRR, lastCountRR, lastTimeRR, pulsesPerRevolution);
}
