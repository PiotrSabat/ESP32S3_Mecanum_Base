// Minimal Arduino/ESP32 stubs, enough to run the real Motor.cpp on the host.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>

typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) ((TickType_t)(x))

#define OUTPUT 1
inline void pinMode(int, int) {}

// Driven clock - the test decides how much time has passed.
extern uint32_t g_fakeMillis;
inline uint32_t millis() { return g_fakeMillis; }

// Last PWM duty written, per channel.
extern int g_pwm[16];
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(int ch, int val) { if (ch >= 0 && ch < 16) g_pwm[ch] = val; }

template <typename T, typename L, typename H>
inline T constrain(T v, L lo, H hi) {
    if (v < (T)lo) return (T)lo;
    if (v > (T)hi) return (T)hi;
    return v;
}
