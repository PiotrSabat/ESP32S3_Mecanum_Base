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
    if (_state == MotorState::HardStopped)
        _state = MotorState::Active;
    _targetRPM = _cfg.invertDirection ? -rpm : rpm;
}

void Motor::update() {
    uint32_t now = millis();
    uint32_t dt = now - _lastTimeMs;
    if (dt == 0) return;

    // Pomiar prędkości wykonujemy ZAWSZE, niezależnie od stanu silnika.
    // Telemetria musi pokazywać prawdę także wtedy, gdy silnik jest zatrzymany —
    // wcześniej w stanie HardStopped raportowana była prędkość sprzed stopu.
    int64_t count = _encoder.getCount();
    int64_t deltaCount = count - _lastCount;
    float deltaRevs = deltaCount / _cfg.gearRatio;  // gearRatio = impulsy na obrót
    _currentRPM = (deltaRevs * 60000.0f) / dt;
    _lastTimeMs = now;
    _lastCount  = count;

    // HardStopped: odcinamy zasilanie silnika i czekamy na nową komendę.
    // Przekładnia 120:1 zatrzymuje platformę sama, więc aktywne hamowanie
    // silnikiem niczego by nie przyspieszyło, a ciągnęłoby prąd.
    if (_state == MotorState::HardStopped) {
        ledcWrite(_cfg.pwmChannel1, 0);
        ledcWrite(_cfg.pwmChannel2, 0);
        _controlOut = 0;
        return;
    }

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

    // Ograniczenie przyspieszenia. Narastanie zadanej prędkości jest limitowane,
    // żeby regulator nie wystawiał od razu pełnego PWM — to właśnie skok prądu
    // przy ruszaniu czterech silników wywala zabezpieczenie ogniw.
    // Ruch zadanej W STRONĘ ZERA jest swobodny, żeby nie spowalniać softStop
    // ani hamowania awaryjnego po utracie łączności.
    {
        float maxDelta = MAX_ACCEL_RPM_PER_S * (dt / 1000.0f);
        float delta = _targetRPM - _rampedTarget;
        bool towardZero = fabsf(_targetRPM) < fabsf(_rampedTarget) &&
                          (_targetRPM * _rampedTarget) >= 0.0f;
        if (!towardZero) {
            if (delta >  maxDelta) delta =  maxDelta;
            if (delta < -maxDelta) delta = -maxDelta;
        }
        _rampedTarget += delta;
    }

    float error = _rampedTarget - _currentRPM;
    float pidOut = computePID(error, dt);
    

    // Ograniczenie sygnału PWM przy użyciu funkcji Arduino
    int pwmVal = static_cast<int>(pidOut);
    pwmVal = constrain(pwmVal, -_maxPwmValue, _maxPwmValue);

    // Ograniczenie momentu przeciwnego do kierunku obrotu, zależne od prędkości.
    //
    // Gdy koło kręci się w jedną stronę, a regulator żąda napięcia w przeciwną,
    // prąd wynosi (U_baterii + SEM) / R — czyli WIĘCEJ niż przy zwarciu.
    // Ale SEM rośnie z prędkością, więc ten sam PWM kosztuje tym więcej prądu,
    // im szybciej kręci się koło. Dlatego limit maleje liniowo z prędkością:
    //   - koło stoi  -> pełny limit: platforma trzyma pozycję pod naciskiem
    //   - pełna prędkość -> limit zero: hamowanie wybiegiem, zero ryzyka
    {
        float speedFrac = fabsf(_currentRPM) / (float)MAX_RPM;
        if (speedFrac > 1.0f) speedFrac = 1.0f;
        int counterLimit = (int)(MAX_HOLDING_PWM * (1.0f - speedFrac));

        if (_currentRPM > STANDSTILL_RPM && pwmVal < -counterLimit) {
            pwmVal = -counterLimit;
        } else if (_currentRPM < -STANDSTILL_RPM && pwmVal > counterLimit) {
            pwmVal = counterLimit;
        }
    }

    // Postój: przy zerowej zadanej i NIERUCHOMYM kole odcinamy zasilanie
    // i zerujemy całkę. Bez tego zostaje resztkowe wypełnienie, które nie
    // porusza kołem (jest poniżej progu tarcia), a mimo to grzeje silnik.
    //
    // Próg musi być wąski: gdy ktoś kręci kołem ręcznie, prędkość przekracza
    // STANDSTILL_RPM, warunek nie zachodzi i regulator może się przeciwstawić.
    // Przy szerszym progu wygaszanie zjadałoby całkę w kółko i platforma nie
    // trzymałaby pozycji.
    if (_rampedTarget == 0.0f && fabsf(_currentRPM) < STANDSTILL_RPM) {
        pwmVal = 0;
        _errorSum = 0.0f;
    }

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
    // (_lastTimeMs i _lastCount ustawione już przy pomiarze na początku)
    _lastError = error;
}


float Motor::computePID(float error, float dt) {
    float dError = (error - _lastError) / dt;

    // Całkowanie warunkowe (anti-windup). Całkę powiększamy tylko wtedy, gdy
    // nie wypchnie to wyjścia poza zakres regulatora.
    //
    // Poprzednia wersja całkowała zawsze i tylko przycinała sumę. Skutek:
    // przy hamowaniu z dużej prędkości błąd był duży i ujemny przez wiele
    // cykli, całka ładowała się do minimum, a gdy koło się zatrzymało i błąd
    // wracał do zera — całka nadal żądała pełnego rewersu i wyrzucała silnik
    // w tył. Stąd część „gumowatego" zachowania przy zwalnianiu.
    float candidateSum = _errorSum + error * dt;
    float output = _Kp * error + _Ki * candidateSum + _Kd * dError;

    if (output >= _outputMin && output <= _outputMax) {
        _errorSum = candidateSum;              // wyjście w zakresie — całkujemy
    } else {
        // Wyjście nasycone — pomijamy nowy wkład do całki.
        output = _Kp * error + _Ki * _errorSum + _Kd * dError;
    }

    return constrain(output, _outputMin, _outputMax);
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

void Motor::softStop(uint32_t durationMs) {
    if (_state != MotorState::Active)
        return;

    _errorSum = 0.0f;
    _lastError = 0.0f;
    _state = MotorState::SoftStopping;
    _softStopStartMs = millis();
    _softStopDurationMs = (durationMs > 0) ? durationMs : _cfg.softStopDurationMs;
    _initialTargetRPM = _targetRPM;
}

void Motor::hardStop() {
    // Natychmiastowy stop — działa zawsze, niezależnie od bieżącego stanu
    _state = MotorState::HardStopped;
    _targetRPM = 0.0f;
    _rampedTarget = 0.0f;
    _errorSum = 0.0f;
    _lastError = 0.0f;
    ledcWrite(_cfg.pwmChannel1, 0);
    ledcWrite(_cfg.pwmChannel2, 0);
}

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




