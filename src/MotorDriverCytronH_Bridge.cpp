#include "MotorDriverCytronH_Bridge.h"

MotorDriverCytronH_Bridge::MotorDriverCytronH_Bridge(uint8_t pin1, uint8_t pin2, uint8_t channel1, uint8_t channel2)
  : _pin1(pin1),
    _pin2(pin2),
    _channel1(channel1),
    _channel2(channel2),
    currentSpeed(0)  // Initialize currentSpeed to 0
{
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    delay(200); // Allow pins to stabilize

    // Ensure both outputs start low
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);

    // Configure PWM channels at 20 kHz, 9-bit resolution
    ledcSetup(_channel1, 20000, 9);
    ledcSetup(_channel2, 20000, 9);

    // Attach PWM channels to the output pins
    ledcAttachPin(_pin1, _channel1);
    ledcAttachPin(_pin2, _channel2);
}

void MotorDriverCytronH_Bridge::setSpeed(int speed)
{
    // Clamp speed to valid 9-bit range [-511, 511]
    if (speed > 511) {
        speed = 511;
    } else if (speed < -511) {
        speed = -511;
    }

    // Set direction and PWM duty cycle
    if (speed >= 0) {
        // Forward: apply duty to channel1
        ledcWrite(_channel1, speed);
        ledcWrite(_channel2, 0);
    } else {
        // Reverse: apply duty to channel2
        ledcWrite(_channel1, 0);
        ledcWrite(_channel2, -speed);
    }
}

MotorDriverCytronH_Bridge::~MotorDriverCytronH_Bridge()
{
    // Destructor: clean up if needed
}
