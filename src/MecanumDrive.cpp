#include "MecanumDrive.h"


MecanumDrive::MecanumDrive(
    MotorDriverCytronH_Bridge* frontLeft,
    MotorDriverCytronH_Bridge* frontRight,
    MotorDriverCytronH_Bridge* rearLeft,
    MotorDriverCytronH_Bridge* rearRight
) {
    _frontLeft = frontLeft;
    _frontRight = frontRight;
    _rearLeft = rearLeft;
    _rearRight = rearRight;
}

void MecanumDrive::move(int x, int y, int yaw) {
    applyDeadZone(x, y, yaw);

    int frontLeft  = y + x + yaw;
    int frontRight = y - x - yaw;
    int rearLeft   = y - x + yaw;
    int rearRight  = y + x - yaw;

    // Normalizacja mocy
    normalizeMotorPower(frontLeft, frontRight, rearLeft, rearRight);

    // Sterowanie silnikami
    _frontLeft->setSpeed(frontLeft);
    _frontRight->setSpeed(frontRight);
    _rearLeft->setSpeed(rearLeft);
    _rearRight->setSpeed(rearRight);
}

void MecanumDrive::applyDeadZone(int& x, int& y, int& yaw) {
    if (abs(x) < DEAD_ZONE) x = 0;
    if (abs(y) < DEAD_ZONE) y = 0;
    if (abs(yaw) < DEAD_ZONE) yaw = 0;
}

//zabezpieczenie przed przekroczeniem maksymalnej prędkości, do usunięcia, jeśli PID działa poprawnie
void MecanumDrive::normalizeMotorPower(int& fl, int& fr, int& rl, int& rr) {
    int maxPower = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
    if (maxPower > MAX_SPEED) {
        fl = fl * MAX_SPEED / maxPower;
        fr = fr * MAX_SPEED / maxPower;
        rl = rl * MAX_SPEED / maxPower;
        rr = rr * MAX_SPEED / maxPower;
    }
}

//nowe metody, proby okiełznania PID

float MecanumDrive::convertToRPM(int padValue) {
//Jeśli wartość jest w obszarze martwym, zwroc 0
    if(abs(padValue) < DEAD_ZONE) {
        return 0.0;
    }

    //zachowujemy znak (kierunek)
    int sign = (padValue > 0) ? 1 : -1;

    // Efektywna wartość to cała wartość minus wartość martwa
    int effectiveValue = abs(padValue) - DEAD_ZONE;

    //Maksymalna efektywna wartość, przyjmujemy ze zakres jest -511 do 511
    int maxEffectiveValue = MAX_SPEED - DEAD_ZONE;

    //ochrona dzielenia przez 0
    if(maxEffectiveValue == 0) {
        return 0.0;
    }

    // Mapowanie efektywnej wartości na zakres RPM: 
    // jeśli effectiveValue == maxEffective to RPM == MAX_RPM
    float rpm = (effectiveValue / (float)maxEffectiveValue) * MAX_RPM;
    return sign * rpm;
}

void MecanumDrive::moveRPM(int x, int y, int yaw) {
    // Zastosowanie martwej strefy
    applyDeadZone(x, y, yaw);

    normalizeInputVector(x, y, yaw);
    
    // obliczenie surowych wartości dla każdego koła
    int rawFL = y + x + yaw;
    int rawFR = y - x - yaw;
    int rawRL = y - x + yaw;
    int rawRR = y + x - yaw;

    move(x, y, yaw);

    // Konwersja wartości z pada na RPM
    rpmFL = convertToRPM(rawFL);
    rpmFR = convertToRPM(rawFR);
    rpmRL = convertToRPM(rawRL);
    rpmRR = convertToRPM(rawRR);
   
}

// Normalizacja wektora wejściowego (x, y, yaw)
// Jeśli długość wektora > MAX_INPUT (np. 511), skalujemy wszystkie składowe
void MecanumDrive::normalizeInputVector(int &x, int &y, int &yaw) {
    float norm = sqrt((float)x * x + (float)y * y + (float)yaw * yaw);
    const float MAX_INPUT_PWM = 511.0; // maksymalny dopuszczalny norm wejścia
    if(norm > MAX_INPUT) {
        float scale = MAX_INPUT / norm;
        x = (int)(x * scale);
        y = (int)(y * scale);
        yaw = (int)(yaw * scale);
    }
}

RPMData MecanumDrive::readRPMs() {
    RPMData data;
    data.frontLeft = rpmFL;
    data.frontRight = rpmFR;
    data.rearLeft = rpmRL;
    data.rearRight = rpmRR;
    return data;
}