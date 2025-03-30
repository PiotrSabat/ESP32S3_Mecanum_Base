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
    // Konstruktor – przyjmuje wskaźniki do enkoderów oraz rozdzielczość (pulses per revolution)
    EncoderReader(ESP32Encoder* fl, ESP32Encoder* fr, ESP32Encoder* rl, ESP32Encoder* rr, uint16_t encoderResolution);
    
    // Metoda inicjalizacyjna, którą należy wywołać w setup()
    void begin();
    
    // Odczyt aktualnych liczników enkoderów
    EncoderData readEncoders();
    
    // Zerowanie liczników enkoderów oraz zmiennych pomocniczych (dla obliczeń RPM)
    void resetEncoders();
    
    // Obliczenie RPM dla wszystkich enkoderów – wynik wyliczany na podstawie przekazanej rozdzielczości
    void getRPMs(float& rpmFL, float& rpmFR, float& rpmRL, float& rpmRR);
    
private:
    ESP32Encoder* encoderFL;
    ESP32Encoder* encoderFR;
    ESP32Encoder* encoderRL;
    ESP32Encoder* encoderRR;
    
    uint16_t _encoderResolution;
    
    // Oddzielne zmienne pomocnicze dla każdego enkodera
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
