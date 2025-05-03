#include <Arduino.h>
#include "Motor.h"
#include "motor_config.h"
#include "MecanumDrive.h"
#include "RPMData.h"

Motor motorFL(FL_CONFIG);
Motor motorFR(FR_CONFIG);
Motor motorRL(RL_CONFIG);
Motor motorRR(RR_CONFIG);

MecanumDrive drive(&motorFL, &motorFR, &motorRL, &motorRR);

unsigned long lastActionTime = 0;
int state = 0;
const int interval = 2000; // ms

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n✅ Start platform test with MecanumDrive");
}

void loop() {
  unsigned long now = millis();

  // co interval milisekund przechodzimy do kolejnego kroku testu
  if (now - lastActionTime > interval) {
    lastActionTime = now;
    state++;

    Serial.printf("\n▶️ State %d\n", state);

    switch (state) {
      case 1:
        Serial.println("Jazda do przodu");
        drive.drive(0, 100, 0);
        break;
      case 2:
        Serial.println("Hard stop po jeździe do przodu");
        drive.hardStop();
        break;
      case 3:
        Serial.println("Jazda do tyłu");
        drive.drive(0, -100, 0);
        break;
      case 4:
        Serial.println("Hard stop po jeździe do tyłu");
        drive.hardStop();
        break;
      case 5:
        Serial.println("Jazda w lewo (w bok)");
        drive.drive(-100, 0, 0);
        break;
      case 6:
        Serial.println("Hard stop po jeździe w lewo");
        drive.hardStop();
        break;
      case 7:
        Serial.println("Jazda w prawo (w bok)");
        drive.drive(100, 0, 0);
        break;
      case 8:
        Serial.println("Hard stop po jeździe w prawo");
        drive.hardStop();
        break;
      case 9:
        Serial.println("Obrót CW");
        drive.drive(0, 0, -100);
        break;
      case 10:
        Serial.println("Hard stop po obrocie CW");
        drive.hardStop();
        break;
      case 11:
        Serial.println("Obrót CCW");
        drive.drive(0, 0, 100);
        break;
      case 12:
        Serial.println("Hard stop po obrocie CCW");
        drive.hardStop();
        break;
      default:
        Serial.println("🛑 Test zakończony — zatrzymanie platformy");
        drive.hardStop();
        while (true) delay(1000);
    }
  }

  // aktualizacja PID w każdej pętli
  drive.update();
  delay(20);
}
