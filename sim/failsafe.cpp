// Symulator failsafe: uruchamia PRAWDZIWY Motor.cpp z projektu na hoście,
// z podstawionym zegarem i enkoderem. Sprawdza zachowanie rampy hamowania
// oraz logikę bramkowania z motorControlTask.
#include <cstdio>
#include <cmath>
#include "Motor.h"

uint32_t g_fakeMillis = 0;
int      g_pwm[16]    = {0};
int64_t  g_encoderCount = 0;

// PAD_LINK_TIMEOUT_MS, FAILSAFE_STOP_DURATION_MS i INTERVAL_MOTOR_CONTROL
// pochodzą z prawdziwego parameters.h projektu (wciąganego przez Motor.h),
// więc test sprawdza wartości faktycznie wgrywane na sprzęt.

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

// Prosty model silnika: obroty nadążają za wypełnieniem PWM.
// 511 jednostek PWM ~ MAX_RPM. Nie musi być wierny — chodzi o to,
// by enkoder w ogóle reagował i pętla PID nie liczyła w próżni.
static void advance(Motor& m, uint32_t ms) {
    int pwm = g_pwm[0] - g_pwm[1];             // znak = kierunek
    float rpm = (pwm / 511.0f) * (float)MAX_RPM;
    float revs = rpm * (ms / 60000.0f);
    g_encoderCount += (int64_t)llround(revs * (double)DEFAULT_GEAR_RATIO);
    g_fakeMillis += ms;
    m.update();
}

int main() {
    Motor m(TEST_CFG);

    printf("\n=== 1. Jazda normalna: silnik rozpedza sie do zadanych 50 RPM ===\n");
    for (int i = 0; i < 100; i++) {                 // 2 s rozpedzania
        m.setTargetRPM(50.0f);
        advance(m, INTERVAL_MOTOR_CONTROL);
    }
    printf("  target=%.1f  current=%.1f  pwm=%d\n",
           m.getTargetRPM(), m.getCurrentRPM(), m.getControlOutput());
    check(m.getTargetRPM() == 50.0f, "targetRPM trzyma zadana wartosc");
    check(m.getCurrentRPM() > 25.0f,  "silnik faktycznie sie kreci");

    printf("\n=== 2. Utrata lacznosci: hardStop(), drive() juz NIE wolane ===\n");
    m.hardStop();
    check(m.getTargetRPM() == 0.0f, "hardStop natychmiast zeruje zadana predkosc");

    bool pwmZawszeZero = true;
    for (int i = 0; i < 13; i++) {
        advance(m, INTERVAL_MOTOR_CONTROL);
        if (m.getControlOutput() != 0) pwmZawszeZero = false;
        printf("  t=+%3d ms  target=%6.1f  current=%6.1f  pwm=%d\n",
               (i + 1) * INTERVAL_MOTOR_CONTROL, m.getTargetRPM(),
               m.getCurrentRPM(), m.getControlOutput());
    }
    check(pwmZawszeZero, "PWM pozostaje zerowy - naped odciety, zero pradu");
    check(fabs(m.getCurrentRPM()) < 5.0f,
          "zmierzona predkosc odswiezana mimo stopu (telemetria mowi prawde)");

    printf("\n=== 3. Cisza trwa: bez drive() robot ma stac ===\n");
    for (int i = 0; i < 50; i++) advance(m, INTERVAL_MOTOR_CONTROL);  // 1 s
    printf("  target=%.1f  current=%.1f  pwm=%d\n",
           m.getTargetRPM(), m.getCurrentRPM(), m.getControlOutput());
    check(m.getTargetRPM() == 0.0f, "target pozostaje 0 mimo uplywu czasu");
    check(fabs(m.getCurrentRPM()) < 5.0f, "robot faktycznie stoi");

    printf("\n=== 4. Powrot lacznosci: drive() znow wolane ===\n");
    for (int i = 0; i < 50; i++) {
        m.setTargetRPM(50.0f);
        advance(m, INTERVAL_MOTOR_CONTROL);
    }
    printf("  target=%.1f  current=%.1f\n", m.getTargetRPM(), m.getCurrentRPM());
    check(m.getTargetRPM() == 50.0f, "sterowanie wraca po odzyskaniu lacznosci");
    check(m.getCurrentRPM() > 25.0f,  "silnik znow sie kreci");

    printf("\n=== 5. Przypadek brzegowy: lacznosc wraca W TRAKCIE rampy ===\n");
    m.setTargetRPM(50.0f);
    advance(m, INTERVAL_MOTOR_CONTROL);
    m.softStop(250);
    advance(m, 60);                                  // 60 ms rampy
    float midRamp = m.getTargetRPM();
    m.setTargetRPM(50.0f);                          // niby-powrot lacznosci
    advance(m, INTERVAL_MOTOR_CONTROL);
    float afterCmd = m.getTargetRPM();
    printf("  target w trakcie rampy=%.1f, po probie sterowania=%.1f\n",
           midRamp, afterCmd);
    check(afterCmd < 100.0f,
          "komenda w trakcie rampy jest ignorowana (rampa ma pierwszenstwo)");

    int steps = 0;
    while (m.getTargetRPM() != 0.0f && steps < 50) { advance(m, INTERVAL_MOTOR_CONTROL); steps++; }
    m.setTargetRPM(50.0f);
    advance(m, INTERVAL_MOTOR_CONTROL);
    printf("  rampa zakonczona po %d ms, target po komendzie=%.1f\n",
           steps * INTERVAL_MOTOR_CONTROL, m.getTargetRPM());
    check(m.getTargetRPM() == 50.0f, "po zakonczeniu rampy sterowanie wraca");
    check(steps * INTERVAL_MOTOR_CONTROL <= 300, "opoznienie powrotu ograniczone (<= 300 ms)");

    printf("\n=== 6. Przepelnienie millis() po ~49.7 dnia ===\n");
    g_fakeMillis = 0xFFFFFF00;                       // tuz przed przewinieciem
    uint32_t lastMsg = g_fakeMillis;
    g_fakeMillis += 500;                             // zegar sie przewija
    bool aliveAfterWrap = (g_fakeMillis - lastMsg) < PAD_LINK_TIMEOUT_MS;
    printf("  lastMsg=%u  teraz=%u  roznica=%u\n",
           lastMsg, g_fakeMillis, g_fakeMillis - lastMsg);
    check(!aliveAfterWrap, "po przewinieciu zegara cisza 500 ms wciaz wykryta");

    printf("\n=== 7. Czy zatrzask jest KONIECZNY? softStop() wolany co 20 ms ===\n");
    for (int i = 0; i < 60; i++) { m.setTargetRPM(50.0f); advance(m, INTERVAL_MOTOR_CONTROL); }
    printf("  start: target=%.1f current=%.1f\n", m.getTargetRPM(), m.getCurrentRPM());
    for (int i = 0; i < 40; i++) {                   // 800 ms ciszy
        m.softStop(250);       // BEZ zatrzasku - za kazdym razem
        advance(m, INTERVAL_MOTOR_CONTROL);
        if (i % 5 == 0)
            printf("  t=+%3d ms  target=%6.1f  current=%6.1f\n",
                   (i + 1) * INTERVAL_MOTOR_CONTROL, m.getTargetRPM(), m.getCurrentRPM());
    }
    printf("  koniec: target=%.1f current=%.1f\n", m.getTargetRPM(), m.getCurrentRPM());
    check(m.getTargetRPM() == 0.0f && fabs(m.getCurrentRPM()) < 5.0f,
          "bez zatrzasku robot TEZ staje (wczesny return w softStop chroni rampe)");

    printf("\n%s  (bledow: %d)\n", failures ? "!!! SA BLEDY" : "WSZYSTKO OK", failures);
    return failures ? 1 : 0;
}
