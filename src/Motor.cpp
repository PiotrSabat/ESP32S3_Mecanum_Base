#include "Motor.h"
#include <Arduino.h>

Motor::Motor(const MotorConfig& config)
    : _cfg(config),
      _maxPwmValue((1 << config.pwmResolution) - 1),
      _encoder()
{
    setupPWM();
    setupEncoder();
    _lastTimeMs = millis();
    _lastCount = _encoder.getCount();
}

void Motor::setupPWM() {
    pinMode(_cfg.pwmPin1, OUTPUT);
    pinMode(_cfg.pwmPin2, OUTPUT);
    ledcSetup(_cfg.pwmChannel1, _cfg.pwmFrequency, _cfg.pwmResolution);
    ledcSetup(_cfg.pwmChannel2, _cfg.pwmFrequency, _cfg.pwmResolution);
    ledcAttachPin(_cfg.pwmPin1, _cfg.pwmChannel1);
    ledcAttachPin(_cfg.pwmPin2, _cfg.pwmChannel2);
}

void Motor::setupEncoder() {
    _encoder.attachHalfQuad(_cfg.encoderPinA, _cfg.encoderPinB);
    _encoder.clearCount();
}

void Motor::setTargetRPM(float rpm) {
    _targetRPM = rpm;
}

void Motor::update() {
    uint32_t now = millis();
    uint32_t dt = now - _lastTimeMs;
    if (dt == 0) return;

    int64_t count = _encoder.getCount();
    int64_t deltaCount = count - _lastCount;

    // Obliczenie bieżącego RPM (przy założeniu gearRatio jako liczby impulsów na obrót)
    float deltaRevs = deltaCount / _cfg.gearRatio;
    _currentRPM = (deltaRevs * 60000.0f) / dt;

    float error = _targetRPM - _currentRPM;
    float pidOut = computePID(error, dt);

    // Ograniczenie sygnału PWM przy użyciu funkcji Arduino
    int pwmVal = static_cast<int>(pidOut);
    pwmVal = constrain(pwmVal, -_maxPwmValue, _maxPwmValue);
    _controlOut = pwmVal;

    // Ustawienie kierunku i wartości PWM
    if (pwmVal >= 0) {
        ledcWrite(_cfg.pwmChannel1, pwmVal);
        ledcWrite(_cfg.pwmChannel2, 0);
    } else {
        ledcWrite(_cfg.pwmChannel1, 0);
        ledcWrite(_cfg.pwmChannel2, -pwmVal);
    }

    // Aktualizacja wartości dla następnego kroku
    _lastTimeMs = now;
    _lastCount = count;
    _lastError = error;
}

float Motor::computePID(float error, float dt) {
    // Sumowanie całki
    _errorSum += error * dt;
    // Pochodna
    float dError = (error - _lastError) / dt;

    // Regulacja PID z użyciem danych z konfiguracji
    float output = _cfg.Kp * error
                 + _cfg.Ki * _errorSum
                 + _cfg.Kd * dError;

    // Ograniczenie wyjścia
    output = constrain(output, _cfg.outputMin, _cfg.outputMax);

    return output;
}

float Motor::getCurrentRPM() const {
    return _currentRPM;
}

int Motor::getControlOutput() const {
    return _controlOut;
}
