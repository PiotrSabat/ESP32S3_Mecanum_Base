#ifndef MOTORDRIVERCYTRONH_BRIDGE_H
#define MOTORDRIVERCYTRONH_BRIDGE_H
#include <Arduino.h>

class MotorDriverCytronH_Bridge
{

public:
    MotorDriverCytronH_Bridge(uint8_t pin1, uint8_t pin2, uint8_t channel1, uint8_t channel2);

    void setSpeed(int speed);
    
    ~MotorDriverCytronH_Bridge();

private:
    uint8_t _pin1;
    uint8_t _pin2;
    uint8_t _channel1;
    uint8_t _channel2;
};



#endif
