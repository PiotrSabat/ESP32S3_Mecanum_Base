#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    // Konstruktor przyjmuje współczynniki PID oraz maksymalne/minimalne wyjście
    PIDController(float kp, float ki, float kd, float maxOutput, float minOutput = -1.0);
    
    // Resetuje całkę i poprzedni błąd – przydatne przy zmianie warunków
    void reset();
    
    // Oblicza nowe wyjście na podstawie setpointu, zmierzonej wartości oraz delta czasu (w sekundach)
    float compute(float setpoint, float measured, float dt);

private:
    float _kp, _ki, _kd;
    float _maxOutput, _minOutput;
    float _integral;
    float _prevError;
};

#endif
