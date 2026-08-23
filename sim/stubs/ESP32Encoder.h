// Zaślepka enkodera — licznik globalny, sterowany przez test
// (pole _encoder w Motor jest prywatne, więc nie da się go ustawić przez obiekt).
#pragma once
#include <cstdint>

extern int64_t g_encoderCount;

class ESP32Encoder {
public:
    void attachHalfQuad(int, int) {}
    void clearCount() { g_encoderCount = 0; }
    int64_t getCount() { return g_encoderCount; }
};
