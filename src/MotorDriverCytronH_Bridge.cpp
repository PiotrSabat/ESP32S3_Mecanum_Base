#include "MotorDriverCytronH_Bridge.h"

MotorDriverCytronH_Bridge::MotorDriverCytronH_Bridge(uint8_t pin1, uint8_t pin2, uint8_t channel1, uint8_t channel2)
  : _pin1(pin1),
    _pin2(pin2),
    _channel1(channel1),
    _channel2(channel2),
    currentSpeed(0)  // Inicjalizacja currentSpeed na 0
{
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    delay(200); // Oczekiwanie na ustabilizowanie się pinów

    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
    

    ledcSetup(_channel1, 20000, 9);
    ledcSetup(_channel2, 20000, 9);
    
    

    ledcAttachPin(_pin1, _channel1);
    ledcAttachPin(_pin2, _channel2);
}

void MotorDriverCytronH_Bridge::setSpeed(int speed)
{
    // Ograniczenie prędkości do zakresu [-511, 511] – wartość większa/mniejsza jest przycięta do 9 bitów, inna wartość nie jest akceptowalna
    if (speed > 511) {
        speed = 511;
    } else if (speed < -511) {
        speed = -511;
    }

    // Ustawienie kierunku i prędkości
    if (speed >= 0) {
        ledcWrite(_channel1, speed);
        ledcWrite(_channel2, 0);
    } else {
        ledcWrite(_channel1, 0);
        ledcWrite(_channel2, -speed);
    }
}



MotorDriverCytronH_Bridge::~MotorDriverCytronH_Bridge()
{
    // Ewentualne sprzątanie, jeśli będzie potrzebne
}
