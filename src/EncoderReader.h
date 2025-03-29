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

    // Funkcja pobierająca aktualne wartości enkoderów
    EncoderData readEncoders();

    // Funkcja zerująca liczniki enkoderów (jeśli potrzebne)
    void resetEncoders();

private:
    ESP32Encoder* encoderFL;
    ESP32Encoder* encoderFR;
    ESP32Encoder* encoderRL;
    ESP32Encoder* encoderRR;
};

#endif
