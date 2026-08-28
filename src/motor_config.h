#pragma once
#include <Arduino.h>
#include "Motor.h"

// UWAGA do nastaw PID: computePID() dostaje dt w MILISEKUNDACH, nie w sekundach.
// Dlatego Kd = 100 obok Kp = 6 nie jest literówką — w konwencji sekundowej to
// Kd = 0.1, a Ki = 0.06 to Ki = 60. Przed zmianą którejkolwiek wartości
// przelicz: Ki_ms = Ki_s / 1000, Kd_ms = Kd_s * 1000.
//
// Wzmocnienia zostały PODWOJONE 2026-08-28 razem z poprawką liczby zliczeń
// enkodera. Zmierzona prędkość i zadana zmalały wtedy dwukrotnie, więc błąd
// też — podwojenie wzmocnień sprawia, że wyjście regulatora pozostaje
// identyczne. To nie jest strojenie, tylko zmiana jednostek.

// --- Definicje konfiguracji dla czterech silników ---
static const MotorConfig FL_CONFIG = {
    // PWM
    .pwmPin1       = FL_PIN1,
    .pwmPin2       = FL_PIN2,
    .pwmChannel1   = FL_CHANNEL1,
    .pwmChannel2   = FL_CHANNEL2,
    // Enkoder
    .encoderPinA   = FL_ENCODER_A,
    .encoderPinB   = FL_ENCODER_B,
    // Kierunek obrotu silnika (true = odwrotny)
    .invertDirection = false,         
    // Przełożenie i PWM
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    // PID
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};

static const MotorConfig FR_CONFIG = {
    .pwmPin1       = FR_PIN1,
    .pwmPin2       = FR_PIN2,
    .pwmChannel1   = FR_CHANNEL1,
    .pwmChannel2   = FR_CHANNEL2,
    .encoderPinA   = FR_ENCODER_A,
    .encoderPinB   = FR_ENCODER_B,
    // Kierunek obrotu silnika (true = odwrotny)
    .invertDirection = true,         
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};

static const MotorConfig RL_CONFIG = {
    .pwmPin1       = RL_PIN1,
    .pwmPin2       = RL_PIN2,
    .pwmChannel1   = RL_CHANNEL1,
    .pwmChannel2   = RL_CHANNEL2,
    .encoderPinA   = RL_ENCODER_A,
    .encoderPinB   = RL_ENCODER_B,
    // Kierunek obrotu silnika (true = odwrotny)
    .invertDirection = false,         
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};

static const MotorConfig RR_CONFIG = {
    .pwmPin1       = RR_PIN1,
    .pwmPin2       = RR_PIN2,
    .pwmChannel1   = RR_CHANNEL1,
    .pwmChannel2   = RR_CHANNEL2,
    .encoderPinA   = RR_ENCODER_A,
    .encoderPinB   = RR_ENCODER_B,
    // Kierunek obrotu silnika (true = odwrotny)
    .invertDirection = true,         
    .gearRatio     = DEFAULT_GEAR_RATIO,
    .pwmResolution = DEFAULT_PWM_RESOLUTION,
    .pwmFrequency  = DEFAULT_PWM_FREQUENCY,
    .Kp            = 6.0,
    .Ki            = 0.06,
    .Kd            = 100.0,
    .outputMin     = MIN_OUT,
    .outputMax     = MAX_OUT,
    // Safety
    .softStopDurationMs = DEFAULT_SOFT_STOP_DURATION_MS,
    .hardStopDurationMs = DEFAULT_HARD_STOP_DURATION_MS
};