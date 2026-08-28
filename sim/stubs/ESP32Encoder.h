// Encoder stub - a global counter driven by the test
// (Motor::_encoder is private, so it cannot be set through the object).
#pragma once
#include <cstdint>

extern int64_t g_encoderCount;

class ESP32Encoder {
public:
    void attachHalfQuad(int, int) {}
    void clearCount() { g_encoderCount = 0; }
    int64_t getCount() { return g_encoderCount; }
};
