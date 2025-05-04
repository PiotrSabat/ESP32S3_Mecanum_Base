#include "Motor.h"
#include <Arduino.h>

Motor::Motor(const MotorConfig& config)
    : _cfg(config),
      _maxPwmValue((1 << config.pwmResolution) - 1),
      _encoder()
{
    setupPWM();
    setupEncoder();

    // Czas i licznik enkodera na start
    _lastTimeMs = millis();
    _lastCount = _encoder.getCount();

    // Inicjalizacja stanów awaryjnych
    _state = MotorState::Active;
    _softStopStartMs = 0;
    _softStopDurationMs = 0;
    _initialTargetRPM = 0.0f;

    // Inicjalizacja PID, rowniez po to, by moc zdalnie zmieniac PID w locie
    _Kp = _cfg.Kp;
    _Ki = _cfg.Ki;
    _Kd = _cfg.Kd;
    _outputMin = _cfg.outputMin;
    _outputMax = _cfg.outputMax;
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
    _targetRPM = _cfg.invertDirection ? -rpm : rpm;
}

// modyfikacja metoddy update() i computePID() do softStop
void Motor::update() {
    uint32_t now = millis();
    uint32_t dt = now - _lastTimeMs;
    if (dt == 0) return;

    // Obsługa stanu SoftStopping
    if (_state == MotorState::SoftStopping) {
        uint32_t elapsedTime = now - _softStopStartMs;
        
        if (elapsedTime >= _softStopDurationMs) {
            // Po upływie czasu softStop: zakończ
            _state = MotorState::Active;
            _targetRPM = 0.0f;  // Zatrzymaj, ale z wyzerowanym targetRPM
        } else {
            // Zmniejszamy targetRPM linearnie do zera
            float targetDelta = (_initialTargetRPM / _softStopDurationMs) * elapsedTime;
            _targetRPM = _initialTargetRPM - targetDelta;
        }
    }

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

// Zwraca aktualnie ustawiony target RPM dla silnika.
float Motor::getTargetRPM() const {
    return _targetRPM;
}


int Motor::getControlOutput() const {
    return _controlOut;
}

//modyfikacja przygotowywująca do softStop i hardStop
void Motor::softStop(uint32_t durationMs) {
    if (_state != MotorState::Active)
        return; // ignorujemy jeśli już w trybie stopu

    _state = MotorState::SoftStopping;
    _softStopStartMs = millis();
    _softStopDurationMs = (durationMs > 0) ? durationMs : _cfg.softStopDurationMs;
    _initialTargetRPM = _targetRPM;
}

void Motor::hardStop() {
    softStop(_cfg.hardStopDurationMs); //szybkie zatrzymanie wykorzystujące softStop
}

//metod do zmiany PID
void Motor::setPID(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void Motor::setOutputLimits(int min, int max) {
    _outputMin = min;
    _outputMax = max;
}

float Motor::getKp() const {
    return _Kp;
}
float Motor::getKi() const {
    return _Ki;
}
float Motor::getKd() const {
    return _Kd;
}
int Motor::getOutputMin() const {
    return _outputMin;
}
int Motor::getOutputMax() const {
    return _outputMax;
}




