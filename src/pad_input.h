#pragma once
#include <cmath>
#include "parameters.h"

// =====================================================================
//  Przeliczenie wychyleń drążków na wielkości zadane dla kinematyki.
//
//  Plik istnieje po to, żeby firmware i symulatory liczyły to JEDNYM
//  kodem. Wcześniej symulator trzymał własną kopię `padAxisToRPM`
//  z komentarzem „ta sama arytmetyka" — czyli dokładnie taki duplikat,
//  który po cichu się rozjeżdża i unieważnia testy.
//
//  Bez zależności od Arduino.h, żeby symulator budował się na komputerze.
// =====================================================================

/// Oś jazdy: wychylenie drążka na prędkość koła w RPM.
/// Poza martwą strefą skala jest liniowa i rozciągnięta tak, by tuż za jej
/// krawędzią prędkość startowała od zera — inaczej drążek „łapałby" skokiem.
inline float padAxisToRPM(int16_t raw) {
    float v = (float)raw;
    if (v >  (float)JOYSTICK_MAX) v =  (float)JOYSTICK_MAX;
    if (v < -(float)JOYSTICK_MAX) v = -(float)JOYSTICK_MAX;
    if (fabsf(v) < (float)JOYSTICK_DEADZONE) return 0.0f;

    float sign = (v > 0.0f) ? 1.0f : -1.0f;
    float mag  = (fabsf(v) - (float)JOYSTICK_DEADZONE) /
                 (float)(JOYSTICK_MAX - JOYSTICK_DEADZONE);
    return sign * mag * (float)MAX_RPM;
}

/// Krzywa wykładnicza drążka. Zachowuje znak i pełny zakres — zmienia tylko
/// rozkład czułości, zagęszczając ją wokół środka.
inline float applyExpo(float norm, float expo) {
    return (1.0f - expo) * norm + expo * norm * norm * norm;
}

/// Oś obrotu: wychylenie drążka na człon obrotu w RPM.
///
/// Prawy drążek steruje CIASNOŚCIĄ ŁUKU, nie wprost prędkością obrotu — przy
/// jeździe człon obrotu rośnie z prędkością, więc lekki ruch drążka daje
/// szeroki łuk także przy pełnym gazie. Przy zatrzymanej platformie drążek
/// wraca do roli obrotu w miejscu, bo łuk o zerowej prędkości nie istnieje.
inline float padAxisToOmega(int16_t rawYaw, float vx, float vy) {
    float v = (float)rawYaw;
    if (v >  (float)JOYSTICK_MAX) v =  (float)JOYSTICK_MAX;
    if (v < -(float)JOYSTICK_MAX) v = -(float)JOYSTICK_MAX;
    if (fabsf(v) < (float)JOYSTICK_DEADZONE) return 0.0f;

    float sign = (v > 0.0f) ? 1.0f : -1.0f;
    float mag  = (fabsf(v) - (float)JOYSTICK_DEADZONE) /
                 (float)(JOYSTICK_MAX - JOYSTICK_DEADZONE);
    float stick = sign * applyExpo(mag, TURN_EXPO);

    // Długość wektora jazdy, a nie sama składowa wzdłużna: przy mecanum jazda
    // bokiem jest pełnoprawnym ruchem i łuk ma sens także wtedy.
    float speed = sqrtf(vx * vx + vy * vy);

    float omegaArc   = stick * speed * TURN_GAIN;   // łuk — rośnie z prędkością
    float omegaPivot = stick * (float)MAX_RPM;      // piruet — pełna władza na postoju

    float blend = speed / PIVOT_BLEND_RPM;
    if (blend > 1.0f) blend = 1.0f;

    return omegaPivot * (1.0f - blend) + omegaArc * blend;
}
