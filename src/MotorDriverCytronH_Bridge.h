#ifndef MOTORDRIVERCYTRONH_BRIDGE_H
#define MOTORDRIVERCYTRONH_BRIDGE_H

#include <Arduino.h>

/**
 * MotorDriverCytronH_Bridge
 *
 * A bridge class to control a DC motor via the Cytron H-Bridge driver using
 * two PWM channels on an ESP32. This class handles direction and speed control.
 */
class MotorDriverCytronH_Bridge
{
public:
    /**
     * Constructor: initializes the motor driver pins and PWM channels.
     * @param pin1 GPIO pin connected to the first driver input.
     * @param pin2 GPIO pin connected to the second driver input.
     * @param channel1 PWM channel index for pin1 (0-15 on ESP32).
     * @param channel2 PWM channel index for pin2 (0-15 on ESP32).
     */
    MotorDriverCytronH_Bridge(uint8_t pin1, uint8_t pin2, uint8_t channel1, uint8_t channel2);

    /**
     * Set the motor speed and direction.
     * Positive values drive forward, negative values drive in reverse.
     * @param speed Signed speed value in range [-511, 511], corresponding to duty cycle.
     */
    void setSpeed(int speed);

    /**
     * Get the last set speed value.
     * @return The signed speed value most recently applied.
     */
    int getCurrentSpeed();
    
    /**
     * Destructor: cleans up resources if necessary.
     */
    ~MotorDriverCytronH_Bridge();

private:
    uint8_t _pin1;       ///< GPIO pin for forward/reverse PWM channel 1
    uint8_t _pin2;       ///< GPIO pin for forward/reverse PWM channel 2
    uint8_t _channel1;   ///< ESP32 PWM channel assigned to pin1
    uint8_t _channel2;   ///< ESP32 PWM channel assigned to pin2
    int currentSpeed;    ///< Last commanded speed value
};

#endif // MOTORDRIVERCYTRONH_BRIDGE_H
