#include "EncoderReader.h"
#include <Arduino.h>

EncoderReader::EncoderReader(ESP32Encoder* fl, ESP32Encoder* fr, ESP32Encoder* rl, ESP32Encoder* rr, uint16_t encoderResolution)
    : encoderFL(fl), encoderFR(fr), encoderRL(rl), encoderRR(rr), _encoderResolution(encoderResolution)
{
    // Konstruktor nie wykonuje operacji sprzętowych – inicjalizację wykonamy w begin().
}

void EncoderReader::begin() {
    lastTimeFL = millis();
    lastTimeFR = millis();
    lastTimeRL = millis();
    lastTimeRR = millis();
    
    lastCountFL = encoderFL->getCount();
    lastCountFR = encoderFR->getCount();
    lastCountRL = encoderRL->getCount();
    lastCountRR = encoderRR->getCount();
    
    //Serial.println("EncoderReader::begin() wykonane.");
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
    
    Serial.println("EncoderReader::resetEncoders() wykonane.");
}

void EncoderReader::getRPMs(float& rpmFL, float& rpmFR, float& rpmRL, float& rpmRR) {
    unsigned long currentTime = millis();
    
    // Jednorazowy odczyt dla każdego enkodera
    int64_t currentCountFL = encoderFL->getCount();
    int64_t currentCountFR = encoderFR->getCount();
    int64_t currentCountRL = encoderRL->getCount();
    int64_t currentCountRR = encoderRR->getCount();
    
    // Obliczenie różnic czasu i impulsów
    unsigned long deltaTimeFL = currentTime - lastTimeFL;
    unsigned long deltaTimeFR = currentTime - lastTimeFR;
    unsigned long deltaTimeRL = currentTime - lastTimeRL;
    unsigned long deltaTimeRR = currentTime - lastTimeRR;
    
    int64_t deltaCountFL = currentCountFL - lastCountFL;
    int64_t deltaCountFR = currentCountFR - lastCountFR;
    int64_t deltaCountRL = currentCountRL - lastCountRL;
    int64_t deltaCountRR = currentCountRR - lastCountRR;
    
    // Obliczanie RPM – używamy _encoderResolution podanej w konstruktorze
    rpmFL = (deltaTimeFL == 0) ? 0.0 : (deltaCountFL * 60000.0) / (_encoderResolution * deltaTimeFL);
    rpmFR = (deltaTimeFR == 0) ? 0.0 : (deltaCountFR * 60000.0) / (_encoderResolution * deltaTimeFR);
    rpmRL = (deltaTimeRL == 0) ? 0.0 : (deltaCountRL * 60000.0) / (_encoderResolution * deltaTimeRL);
    rpmRR = (deltaTimeRR == 0) ? 0.0 : (deltaCountRR * 60000.0) / (_encoderResolution * deltaTimeRR);
    
    // Aktualizacja zmiennych pomocniczych
    lastTimeFL = currentTime;
    lastTimeFR = currentTime;
    lastTimeRL = currentTime;
    lastTimeRR = currentTime;
    
    lastCountFL = currentCountFL;
    lastCountFR = currentCountFR;
    lastCountRL = currentCountRL;
    lastCountRR = currentCountRR;
}
