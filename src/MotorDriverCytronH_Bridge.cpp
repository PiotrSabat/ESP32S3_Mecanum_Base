#include "MotorDriverCytronH_Bridge.h"
MotorDriverCytronH_Bridge::MotorDriverCytronH_Bridge(uint8_t pin1, uint8_t pin2, uint8_t channel1, uint8_t channel2)
{
    _pin1 = pin1;
    _pin2 = pin2;
    _channel1 = channel1;
    _channel2 = channel2;

    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);

    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);

    ledcAttachPin(_pin1, channel1);
    ledcAttachPin(_pin2, channel2);

    ledcSetup(channel1, 20000, 8);
    ledcSetup(channel2, 20000, 8);


}

void MotorDriverCytronH_Bridge::setSpeed(int speed)
{
    //Make sure the speed is within the limit
     if (speed > 100)
      {
            speed = 100;
      }
      else if (speed < -100)
      {
            speed = -100;
      }

      //Set speed and direction
      if (speed >= 0)
      {
        ledcWrite(_channel1, speed);
        ledcWrite(_channel2, 0);
      } else 
      {
        ledcWrite(_channel1, 0);
        ledcWrite(_channel2, -speed);
      }


}

MotorDriverCytronH_Bridge::~MotorDriverCytronH_Bridge()
{
}
