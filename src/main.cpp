#include <Arduino.h>
#include "Motor.h"
#include "parameters.h"   // piny, PID, PWM itp.
#include "motor_config.h" // konfiguracja silników



// --- Tworzymy obiekty silników ---
static Motor motorFL(FL_CONFIG);
static Motor motorFR(FR_CONFIG);
static Motor motorRL(RL_CONFIG);
static Motor motorRR(RR_CONFIG);

void setup() {
    Serial.begin(115200);
    while (!Serial) { }
    Serial.println("Start testu 4 silników");
}

void loop() {
    // Przykładowy test: ustawiamy na wszystkich +100 RPM, później -100 RPM
    const float testRPM = 5.0f;

    // 1) Forward
    Serial.println("\n--- Wszystkie silniki: forward 100 RPM ---");
    motorFL.setTargetRPM( testRPM );
    motorFR.setTargetRPM( -testRPM );
    motorRL.setTargetRPM( testRPM );
    motorRR.setTargetRPM( -testRPM );

    for (uint32_t t=millis(); millis()-t < 2000; ) {
        motorFL.update();  motorFR.update();
        motorRL.update();  motorRR.update();

        Serial.printf("FL: RPM=%.1f PWM=%d | FR: RPM=%.1f PWM=%d\n",
                      motorFL.getCurrentRPM(), motorFL.getControlOutput(),
                      motorFR.getCurrentRPM(), motorFR.getControlOutput());
        Serial.printf("RL: RPM=%.1f PWM=%d | RR: RPM=%.1f PWM=%d\n",
                      motorRL.getCurrentRPM(), motorRL.getControlOutput(),
                      motorRR.getCurrentRPM(), motorRR.getControlOutput());
        delay(INTERVAL_MOTOR_CONTROL);
    }

    // 2) Backward
    Serial.println("\n--- Wszystkie silniki: backward 100 RPM ---");
    motorFL.setTargetRPM( -testRPM );
    motorFR.setTargetRPM( testRPM );
    motorRL.setTargetRPM( -testRPM );
    motorRR.setTargetRPM( testRPM );

    for (uint32_t t=millis(); millis()-t < 2000; ) {
        motorFL.update();  motorFR.update();
        motorRL.update();  motorRR.update();

        Serial.printf("FL: RPM=%.1f PWM=%d | FR: RPM=%.1f PWM=%d\n",
                      motorFL.getCurrentRPM(), motorFL.getControlOutput(),
                      motorFR.getCurrentRPM(), motorFR.getControlOutput());
        Serial.printf("RL: RPM=%.1f PWM=%d | RR: RPM=%.1f PWM=%d\n",
                      motorRL.getCurrentRPM(), motorRL.getControlOutput(),
                      motorRR.getCurrentRPM(), motorRR.getControlOutput());
        delay(INTERVAL_MOTOR_CONTROL);
    }

    // 3) Stop
    Serial.println("\n--- Stop ---");
    motorFL.setTargetRPM( 0.0f );
    motorFR.setTargetRPM( 0.0f );
    motorRL.setTargetRPM( 0.0f );
    motorRR.setTargetRPM( 0.0f );

    for (uint32_t t=millis(); millis()-t < 2000; ) {
        motorFL.update();  motorFR.update();
        motorRL.update();  motorRR.update();

        // wystarczy co kilka iteracji
        delay(INTERVAL_MOTOR_CONTROL);
    }

    // powtórz sekwencję
}
