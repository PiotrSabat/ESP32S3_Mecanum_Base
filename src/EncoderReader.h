#ifndef ENCODERREADER_H
#define ENCODERREADER_H

#include <ESP32Encoder.h>

// Struktura przechowująca odczyty z enkoderów
struct EncoderData {
    int64_t frontLeft;
    int64_t frontRight;
    int64_t rearLeft;
    int64_t rearRight;
};

class EncoderReader {
public:
    // Konstruktor przyjmujący wskaźniki do obiektów enkoderów
    EncoderReader(ESP32Encoder* fl, ESP32Encoder* fr, ESP32Encoder* rl, ESP32Encoder* rr);

    // Odczyt aktualnych liczników enkoderów
    EncoderData readEncoders();

    // Zerowanie liczników enkoderów oraz zmiennych pomocniczych (dla RPM)
    void resetEncoders();

    // Obliczenie RPM dla danego enkodera
    float calculateRPM(ESP32Encoder* encoder, int64_t& lastCount, unsigned long& lastTime, uint16_t pulsesPerRevolution);

    // Obliczenie RPM dla wszystkich enkoderów
    void getRPMs(uint16_t pulsesPerRevolution, float& rpmFL, float& rpmFR, float& rpmRL, float& rpmRR);

private:
    ESP32Encoder* encoderFL;
    ESP32Encoder* encoderFR;
    ESP32Encoder* encoderRL;
    ESP32Encoder* encoderRR;
    
    // Oddzielne zmienne lastTime dla każdego enkodera
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
