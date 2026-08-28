#include "Motor.h"
#include <Arduino.h>

Motor::Motor(const MotorConfig& config)
    : _cfg(config),
      _maxPwmValue((1 << config.pwmResolution) - 1),
      _encoder()
{
    setupPWM();
    setupEncoder();

    // Seed the speed measurement so the first update() has a valid baseline.
    _lastTimeMs = millis();
    _lastCount  = _encoder.getCount();

    _state = MotorState::Active;
    _softStopStartMs    = 0;
    _softStopDurationMs = 0;
    _initialTargetRPM   = 0.0f;

    // Working copies of the gains, so they can be retuned at runtime.
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
    uint32_t dt  = now - _lastTimeMs;
    if (dt == 0) return;

    // Speed is measured ALWAYS, whatever state the motor is in. Telemetry has
    // to tell the truth while the drive is cut too — an earlier version kept
    // reporting the speed from before a HardStopped, which is worse than
    // useless during a failsafe.
    int64_t count      = _encoder.getCount();
    int64_t deltaCount = count - _lastCount;
    float   deltaRevs  = deltaCount / _cfg.gearRatio;  // gearRatio = COUNTS per wheel rev
    _currentRPM = (deltaRevs * 60000.0f) / dt;
    _lastTimeMs = now;
    _lastCount  = count;

    // HardStopped: cut the drive and wait for a new command. Braking is by
    // coasting, on the assumption that gearbox friction pulls the platform up
    // by itself. That assumption held only in part — the 120:1 gearbox turned
    // out to be back-drivable (a wheel can be turned by hand with the power
    // off), so on a slope this may not be enough. See "Known gaps" in
    // CLAUDE.md; the fix, if it is needed, is softStop() instead.
    if (_state == MotorState::HardStopped) {
        ledcWrite(_cfg.pwmChannel1, 0);
        ledcWrite(_cfg.pwmChannel2, 0);
        _controlOut = 0;
        return;
    }

    // Soft stop: walk the commanded speed linearly down to zero.
    if (_state == MotorState::SoftStopping) {
        uint32_t elapsedTime = now - _softStopStartMs;

        if (elapsedTime >= _softStopDurationMs) {
            _state     = MotorState::Active;
            _targetRPM = 0.0f;
        } else {
            float targetDelta = (_initialTargetRPM / _softStopDurationMs) * elapsedTime;
            _targetRPM = _initialTargetRPM - targetDelta;
        }
    }

    // Acceleration limit. Ramping the command up is capped so the controller
    // does not put out full duty on the first cycle — that current spike, with
    // four motors starting at once, is what trips the cell protection.
    // Movement of the command TOWARDS ZERO is free, so that soft stops and the
    // failsafe cutoff are not slowed down. This looks needlessly convoluted and
    // someone will want to "simplify" it into a symmetric limit: don't. It is
    // the difference between stopping in 320 ms and in 600 ms.
    {
        float maxDelta = MAX_ACCEL_RPM_PER_S * (dt / 1000.0f);
        float delta    = _targetRPM - _rampedTarget;
        bool  towardZero = fabsf(_targetRPM) < fabsf(_rampedTarget) &&
                           (_targetRPM * _rampedTarget) >= 0.0f;
        if (!towardZero) {
            if (delta >  maxDelta) delta =  maxDelta;
            if (delta < -maxDelta) delta = -maxDelta;
        }
        _rampedTarget += delta;
    }

    float error  = _rampedTarget - _currentRPM;
    float pidOut = computePID(error, dt);

    int pwmVal = static_cast<int>(pidOut);
    pwmVal = constrain(pwmVal, -_maxPwmValue, _maxPwmValue);

    // Speed-dependent limit on torque opposing the current rotation.
    //
    // When a wheel turns one way and the controller demands voltage the other
    // way, the current is (V_batt + EMF) / R — MORE than a short circuit,
    // because back-EMF adds instead of subtracting. EMF grows with speed, so
    // the same duty cycle costs more current the faster the wheel spins.
    // Hence the limit falls linearly with speed:
    //   - wheel standing  -> full limit: the platform holds position under load
    //   - full speed      -> limit zero: braking by coasting, no risk
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

    // Standstill: with a zero command and a MOTIONLESS wheel, cut the output
    // and clear the integral. Without this a residual duty cycle remains that
    // is below the friction threshold — it moves nothing and only heats the
    // motor.
    //
    // The threshold has to stay NARROW: when someone turns a wheel by hand the
    // speed exceeds STANDSTILL_RPM, the condition does not hold and the
    // controller is free to push back. A wider threshold would keep eating the
    // integral and position holding would stop working.
    if (_rampedTarget == 0.0f && fabsf(_currentRPM) < STANDSTILL_RPM) {
        pwmVal    = 0;
        _errorSum = 0.0f;
    }

    _controlOut = pwmVal;

    // Sign selects the direction; the other channel is held at zero.
    if (pwmVal >= 0) {
        ledcWrite(_cfg.pwmChannel1, pwmVal);
        ledcWrite(_cfg.pwmChannel2, 0);
    } else {
        ledcWrite(_cfg.pwmChannel1, 0);
        ledcWrite(_cfg.pwmChannel2, -pwmVal);
    }

    // (_lastTimeMs and _lastCount were already updated by the measurement above)
    _lastError = error;
}

float Motor::computePID(float error, float dt) {
    float dError = (error - _lastError) / dt;

    // Conditional integration (anti-windup). The integral only grows when
    // doing so would not push the output out of range.
    //
    // The previous version always integrated and merely clamped the sum. The
    // consequence: braking from speed left a large negative error for many
    // cycles, the integral charged to its minimum, and once the wheel stopped
    // and the error returned to zero the integral still demanded full reverse
    // and threw the motor backwards. That was part of the "rubbery" feel when
    // slowing down.
    float candidateSum = _errorSum + error * dt;
    float output = _Kp * error + _Ki * candidateSum + _Kd * dError;

    if (output >= _outputMin && output <= _outputMax) {
        _errorSum = candidateSum;              // in range — accept the integral
    } else {
        // Saturated — drop this contribution to the integral.
        output = _Kp * error + _Ki * _errorSum + _Kd * dError;
    }

    return constrain(output, _outputMin, _outputMax);
}

float Motor::getCurrentRPM() const {
    return _currentRPM;
}

float Motor::getTargetRPM() const {
    return _targetRPM;
}

int Motor::getControlOutput() const {
    return _controlOut;
}

void Motor::softStop(uint32_t durationMs) {
    // Early return: a soft stop already under way must not be restarted, or a
    // caller looping every 20 ms would keep resetting the ramp to its start.
    if (_state != MotorState::Active)
        return;

    _errorSum  = 0.0f;
    _lastError = 0.0f;
    _state              = MotorState::SoftStopping;
    _softStopStartMs    = millis();
    _softStopDurationMs = (durationMs > 0) ? durationMs : _cfg.softStopDurationMs;
    _initialTargetRPM   = _targetRPM;
}

void Motor::hardStop() {
    // Immediate stop — works from any state.
    _state        = MotorState::HardStopped;
    _targetRPM    = 0.0f;
    _rampedTarget = 0.0f;
    _errorSum     = 0.0f;
    _lastError    = 0.0f;
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

float Motor::getKp() const { return _Kp; }
float Motor::getKi() const { return _Ki; }
float Motor::getKd() const { return _Kd; }
int   Motor::getOutputMin() const { return _outputMin; }
int   Motor::getOutputMax() const { return _outputMax; }
