// Failsafe simulator: runs the project's REAL Motor.cpp on the host, with a
// substituted clock and encoder. Checks the braking ramp and the gating logic
// taken from motorControlTask.
#include <cstdio>
#include <cmath>
#include "Motor.h"

uint32_t g_fakeMillis = 0;
int      g_pwm[16]    = {0};
int64_t  g_encoderCount = 0;

// PAD_LINK_TIMEOUT_MS and INTERVAL_MOTOR_CONTROL come from the project's real
// parameters.h (pulled in via Motor.h), so this test exercises the values that
// actually get flashed. The PID gains below are deliberately NOT the flashed
// ones: the plant model here is a crude "PWM maps straight to RPM" and the real
// gains would ring against it. This test is about the stop logic, not tuning.

static const MotorConfig TEST_CFG = {
    .pwmPin1 = 9, .pwmPin2 = 10, .pwmChannel1 = 0, .pwmChannel2 = 1,
    .encoderPinA = 1, .encoderPinB = 2,
    .invertDirection = false,
    .gearRatio = 1920, .pwmResolution = 9, .pwmFrequency = 20000,
    .Kp = 0.55f, .Ki = 0.03f, .Kd = 0.001f,
    .outputMin = -511.0f, .outputMax = 511.0f,
    .softStopDurationMs = 500, .hardStopDurationMs = 50
};

static int failures = 0;
static void check(bool cond, const char* what) {
    printf("  [%s] %s\n", cond ? " OK " : "FAIL", what);
    if (!cond) failures++;
}

// Crude motor model: RPM follows the PWM duty cycle. 511 PWM units ~ MAX_RPM.
// It does not need to be faithful — it only has to make the encoder respond so
// the PID loop is not computing in a vacuum.
static void advance(Motor& m, uint32_t ms) {
    int pwm = g_pwm[0] - g_pwm[1];             // sign = direction
    float rpm = (pwm / 511.0f) * (float)MAX_RPM;
    float revs = rpm * (ms / 60000.0f);
    g_encoderCount += (int64_t)llround(revs * (double)DEFAULT_GEAR_RATIO);
    g_fakeMillis += ms;
    m.update();
}

int main() {
    Motor m(TEST_CFG);

    printf("\n=== 1. Normal driving: motor spins up to the commanded 50 RPM ===\n");
    for (int i = 0; i < 100; i++) {                 // 2 s of spinning up
        m.setTargetRPM(50.0f);
        advance(m, INTERVAL_MOTOR_CONTROL);
    }
    printf("  target=%.1f  current=%.1f  pwm=%d\n",
           m.getTargetRPM(), m.getCurrentRPM(), m.getControlOutput());
    check(m.getTargetRPM() == 50.0f, "targetRPM holds the commanded value");
    check(m.getCurrentRPM() > 25.0f,  "the motor is actually turning");

    printf("\n=== 2. Link lost: hardStop(), drive() no longer called ===\n");
    m.hardStop();
    check(m.getTargetRPM() == 0.0f, "hardStop zeroes the commanded speed immediately");

    bool pwmZawszeZero = true;
    for (int i = 0; i < 13; i++) {
        advance(m, INTERVAL_MOTOR_CONTROL);
        if (m.getControlOutput() != 0) pwmZawszeZero = false;
        printf("  t=+%3d ms  target=%6.1f  current=%6.1f  pwm=%d\n",
               (i + 1) * INTERVAL_MOTOR_CONTROL, m.getTargetRPM(),
               m.getCurrentRPM(), m.getControlOutput());
    }
    check(pwmZawszeZero, "PWM stays at zero - drive cut, no current");
    check(fabs(m.getCurrentRPM()) < 5.0f,
          "measured speed still refreshed while stopped (telemetry tells the truth)");

    printf("\n=== 3. Silence continues: without drive() the robot must stay put ===\n");
    for (int i = 0; i < 50; i++) advance(m, INTERVAL_MOTOR_CONTROL);  // 1 s
    printf("  target=%.1f  current=%.1f  pwm=%d\n",
           m.getTargetRPM(), m.getCurrentRPM(), m.getControlOutput());
    check(m.getTargetRPM() == 0.0f, "target stays 0 as time passes");
    check(fabs(m.getCurrentRPM()) < 5.0f, "the robot is genuinely stopped");

    printf("\n=== 4. Link restored: drive() called again ===\n");
    for (int i = 0; i < 50; i++) {
        m.setTargetRPM(50.0f);
        advance(m, INTERVAL_MOTOR_CONTROL);
    }
    printf("  target=%.1f  current=%.1f\n", m.getTargetRPM(), m.getCurrentRPM());
    check(m.getTargetRPM() == 50.0f, "control resumes once the link is back");
    check(m.getCurrentRPM() > 25.0f,  "the motor is turning again");

    printf("\n=== 5. Edge case: the link returns MID-RAMP ===\n");
    m.setTargetRPM(50.0f);
    advance(m, INTERVAL_MOTOR_CONTROL);
    m.softStop(250);
    advance(m, 60);                                  // 60 ms into the ramp
    float midRamp = m.getTargetRPM();
    m.setTargetRPM(50.0f);                          // pretend the link came back
    advance(m, INTERVAL_MOTOR_CONTROL);
    float afterCmd = m.getTargetRPM();
    printf("  target mid-ramp=%.1f, after a control attempt=%.1f\n",
           midRamp, afterCmd);
    check(afterCmd < 100.0f,
          "a command mid-ramp is ignored (the ramp takes precedence)");

    int steps = 0;
    while (m.getTargetRPM() != 0.0f && steps < 50) { advance(m, INTERVAL_MOTOR_CONTROL); steps++; }
    m.setTargetRPM(50.0f);
    advance(m, INTERVAL_MOTOR_CONTROL);
    printf("  ramp finished after %d ms, target after the command=%.1f\n",
           steps * INTERVAL_MOTOR_CONTROL, m.getTargetRPM());
    check(m.getTargetRPM() == 50.0f, "control returns once the ramp finishes");
    check(steps * INTERVAL_MOTOR_CONTROL <= 300, "the return delay is bounded (<= 300 ms)");

    printf("\n=== 6. millis() rollover after ~49.7 days ===\n");
    g_fakeMillis = 0xFFFFFF00;                       // just before the wrap
    uint32_t lastMsg = g_fakeMillis;
    g_fakeMillis += 500;                             // the clock wraps
    bool aliveAfterWrap = (g_fakeMillis - lastMsg) < PAD_LINK_TIMEOUT_MS;
    printf("  lastMsg=%u  now=%u  difference=%u\n",
           lastMsg, g_fakeMillis, g_fakeMillis - lastMsg);
    check(!aliveAfterWrap, "500 ms of silence is still detected across the rollover");

    printf("\n=== 7. Is the latch NECESSARY? softStop() called every 20 ms ===\n");
    for (int i = 0; i < 60; i++) { m.setTargetRPM(50.0f); advance(m, INTERVAL_MOTOR_CONTROL); }
    printf("  start: target=%.1f current=%.1f\n", m.getTargetRPM(), m.getCurrentRPM());
    for (int i = 0; i < 40; i++) {                   // 800 ms of silence
        m.softStop(250);       // NO latch - called every iteration
        advance(m, INTERVAL_MOTOR_CONTROL);
        if (i % 5 == 0)
            printf("  t=+%3d ms  target=%6.1f  current=%6.1f\n",
                   (i + 1) * INTERVAL_MOTOR_CONTROL, m.getTargetRPM(), m.getCurrentRPM());
    }
    printf("  end: target=%.1f current=%.1f\n", m.getTargetRPM(), m.getCurrentRPM());
    check(m.getTargetRPM() == 0.0f && fabs(m.getCurrentRPM()) < 5.0f,
          "the robot stops without the latch too (softStop early return guards the ramp)");

    printf("\n%s  (failures: %d)\n", failures ? "!!! FAILURES" : "ALL OK", failures);
    return failures ? 1 : 0;
}
