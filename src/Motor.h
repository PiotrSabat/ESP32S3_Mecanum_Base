#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "parameters.h"

/**
 * Struktura konfiguracji silnika.
 * Wszystkie parametry (piny, przełożenie, PWM, PID) definiuje się np. w moto_config.h lub parameters.h
 */
struct MotorConfig {
    int   pwmPin1;        ///< Pin PWM dla kierunku/prędkości (kanał 1)
    int   pwmPin2;        ///< Pin PWM dla kierunku/prędkości (kanał 2)
    int   pwmChannel1;    ///< Kanał LEDC dla pwmPin1
    int   pwmChannel2;    ///< Kanał LEDC dla pwmPin2
    int   encoderPinA;    ///< Pin A enkodera
    int   encoderPinB;    ///< Pin B enkodera
    bool invertDirection;         ///< Kierunek obrotu silnika (true = odwrotny)
    float gearRatio;      ///< Przełożenie przekładni
    int   pwmResolution;  ///< Rozdzielczość PWM (liczba bitów)
    int   pwmFrequency;   ///< Częstotliwość PWM (Hz)
    float Kp;             ///< Wzmocnienie proporcjonalne PID
    float Ki;             ///< Wzmocnienie całkujące PID
    float Kd;             ///< Wzmocnienie różniczkujące PID
    float outputMin;      ///< Minimalne wyjście regulatora (np. -maxPWM)
    float outputMax;      ///< Maksymalne wyjście regulatora (np. maxPWM)
    // --- Parametry bezpieczeństwa ---
    uint32_t softStopDurationMs;  ///< Czas trwania soft stopu (ms)
    uint32_t hardStopDurationMs;  ///< Czas trwania hard stopu (ms)
    
};

/**
 * Klasa sterująca pojedyńczym silnikiem DC z enkoderem i regulatorem PID.
 *
 * Użycie:
 * 1. W parameters.h lub moto_config.h zdefiniuj zmienne MotorConfig,
 *    np.:
 *      extern const MotorConfig FL_CONFIG;
 * 2. W main.cpp utwórz obiekt:
 *      Motor motorFL(FL_CONFIG);
 * 3. W setup(): nic więcej — konstruktor automatycznie skonfiguruje pini,
 *    PWM i enkoder.
 * 4. W loop(): regularnie wywołuj:
 *      motorFL.setTargetRPM(desiredRPM);
 *      motorFL.update();           // co INTERVAL_MOTOR_CONTROL ms
 *      float rpm = motorFL.getCurrentRPM();
 */


/**
 * Stany pracy silnika — przydatne do obsługi zatrzymań awaryjnych.
 */
enum class MotorState {
    Active,         ///< Normalna praca
    SoftStopping,   ///< Trwa łagodne zatrzymanie (softStop)
    HardStopped     ///< Zatrzymany natychmiastowo (hardStop)
};


class Motor {
public:
    /**
     * Konstruktor.
     * @param config  Konfiguracja silnika (piny, przełożenie, PWM, PID)
     */
    explicit Motor(const MotorConfig& config);

    /**
     * Ustawia wartość zadanej prędkości w RPM.
     * @param rpm  Prędkość obrotowa w obr./min.
     */
    void setTargetRPM(float rpm);

    /**
     * Główna pętla kontrolna:
     * - Odczytuje aktualny stan enkodera,
     * - Oblicza prędkość obrotową,
     * - Wykonuje krok regulatora PID,
     * - Aktualizuje wyjście PWM.
     *
     * Powinna być wywoływana cyklicznie co INTERVAL_MOTOR_CONTROL.
     */
    void update();

    /**
     * Zwraca ostatnio zmierzone RPM.
     */
    float getCurrentRPM() const;
    
    // Zwraca aktualnie ustawiony target RPM dla silnika.
    float getTargetRPM() const;


    /**
     * Zwraca ostatnio obliczone wyjście regulatora (wartość PWM).
     */
    int getControlOutput() const;

    //modyfikacja przygotowywująca do softStop i hardStop
        /**
     * Rozpoczyna łagodne zatrzymanie silnika (soft stop).
     * Silnik zatrzymuje się stopniowo w określonym czasie,
     * zmniejszając targetRPM aż do zera przy użyciu PID.
     *
     * @param durationMs  Czas trwania w ms
     */
    void softStop(uint32_t durationMs = 0);
    /**
     * Rozpoczyna natychmiastowe zatrzymanie silnika (hard stop).
     * Silnik zatrzymuje się natychmiastowo (kilka milisekund).
     */
    void hardStop();

   
 



private:
    MotorConfig  _cfg;         ///< Parametry konfiguracyjne silnika
    ESP32Encoder _encoder;     ///< Obiekt enkodera
    int          _maxPwmValue; ///< Maksymalna wartość PWM (2^resolution - 1)

    // PID
    float        _targetRPM   = 0.0f;
    float        _currentRPM  = 0.0f;
    float        _errorSum    = 0.0f;
    float        _lastError   = 0.0f;
    int          _controlOut  = 0;

    int64_t      _lastCount   = 0;      ///< Poprzedni odczyt enkodera
    uint32_t     _lastTimeMs  = 0;      ///< Poprzedni czas pomiaru


    //modyfikacja przygotowywująca do softStop i hardStop
    // Stan silnika
    MotorState _state;

    // Parametry softStop
    uint32_t   _softStopStartMs;     ///< Czas rozpoczęcia softStop
    uint32_t   _softStopDurationMs;  ///< Czas trwania softStop
    float      _initialTargetRPM;    ///< TargetRPM przed softStop


    /**
     * Konfiguracja i uruchomienie PWM.
     */
    void setupPWM();

    /**
     * Konfiguracja i podłączenie enkodera.
     */
    void setupEncoder();

    /**
     * Oblicza sygnał sterujący PID w oparciu o błąd i odstęp czasu.
     * @param error  Aktualny błąd (targetRPM - currentRPM)
     * @param dt     Czas od ostatniego wywołania (ms)
     * @return       Wartość wyjściowa regulatora (PWM)
     */
    float computePID(float error, float dt);
};
