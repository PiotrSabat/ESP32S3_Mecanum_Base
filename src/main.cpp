#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"
#include "MecanumDrive.h"
#include <ESP32Encoder.h>
#include "EncoderReader.h"
#include "PIDController.h"
#include "messages.h"
//#include "mac_addresses.h"
//#include "mac_addresses_private.h"  // Include only real MACs via build flag instead of both files

#include "network.h"
#include "sensor.h"
#include "drive.h"
#include "tasks.h"

// Data structure from the controller


int32_t totalMessages = 0;

// Motor driver objects
MotorDriverCytronH_Bridge rearRight(RR_PIN2, RR_PIN1, RR_CHANNEL2, RR_CHANNEL1);
MotorDriverCytronH_Bridge frontRight(FR_PIN2, FR_PIN1, FR_CHANNEL2, FR_CHANNEL1);
MotorDriverCytronH_Bridge frontLeft(FL_PIN1, FL_PIN2, FL_CHANNEL1, FL_CHANNEL2);
MotorDriverCytronH_Bridge rearLeft(RL_PIN1, RL_PIN2, RL_CHANNEL1, RL_CHANNEL2);

// Mecanum drive vectoring
MecanumDrive drive(&frontLeft, &frontRight, &rearLeft, &rearRight);

// Encoders and reader
ESP32Encoder frontLeftEncoder, frontRightEncoder, rearLeftEncoder, rearRightEncoder;
EncoderReader encoderReader(&frontLeftEncoder, &frontRightEncoder, &rearLeftEncoder, &rearRightEncoder, ENCODER_RESOLUTION);

// PID controller objects for each wheel
// PID Controller Parameters

PIDController pidFL(KP, KI, KD, MAX_OUT, MIN_OUT);
PIDController pidFR(KP, KI, KD, MAX_OUT, MIN_OUT);
PIDController pidRL(KP, KI, KD, MAX_OUT, MIN_OUT);
PIDController pidRR(KP, KI, KD, MAX_OUT, MIN_OUT);

// ESP-NOW peer info
esp_now_peer_info_t peerInfo, peerInfoMonitor;

// FreeRTOS task handles
TaskHandle_t espNowTaskHandle, motorControlTaskHandle, pidTaskHandle, debugTaskHandle;

// ESP-NOW receive callback
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(myData_from_Pad) && xSemaphoreTake(padDataMutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&myData_from_Pad, incomingData, sizeof(myData_from_Pad));
        xSemaphoreGive(padDataMutex);
    }
}

// TASK 1: Send telemetry via ESP-NOW
void espNowTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(20);
    while (true) {
        Message_from_Platform_Mecanum debugMsg = {}; // Initialize to zero
        if (xSemaphoreTake(padDataMutex, portMAX_DELAY) == pdTRUE) {
            debugMsg.timestamp = millis();
            debugMsg.totalMessages = totalMessages++;
            auto rpm = drive.readRPMs();
            debugMsg.frontLeftSpeedRPM  = rpm.frontLeft;
            debugMsg.frontRightSpeedRPM = rpm.frontRight;
            debugMsg.rearLeftSpeedRPM   = rpm.rearLeft;
            debugMsg.rearRightSpeedRPM  = rpm.rearRight;
            debugMsg.frontLeftEncoder   = frontLeftEncoder.getCount();
            debugMsg.frontRightEncoder  = frontRightEncoder.getCount();
            debugMsg.rearLeftEncoder    = rearLeftEncoder.getCount();
            debugMsg.rearRightEncoder   = rearRightEncoder.getCount();
            debugMsg.KP_message         = KP;
            debugMsg.KI_message         = KI;
            debugMsg.KD_message         = KD;
            debugMsg.MAX_OUT_message    = MAX_OUT;
            debugMsg.MIN_OUT_message    = MIN_OUT;
            xSemaphoreGive(padDataMutex);
        }
        esp_now_send(macMonitorDebug, (uint8_t *)&debugMsg, sizeof(debugMsg));
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

// TASK 2: Read pad and set drive target RPMs
void motorControlTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(20);
    while (true) {
        int x=0, y=0, yaw=0;
        if (xSemaphoreTake(padDataMutex, portMAX_DELAY) == pdTRUE) {
            x = myData_from_Pad.L_Joystick_x_message;
            y = myData_from_Pad.L_Joystick_y_message;
            yaw = myData_from_Pad.R_Joystick_x_message;
            xSemaphoreGive(padDataMutex);
        }
        drive.moveRPM(x, y, yaw);  // update target RPMs only
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

// TASK 3: PID control loop, applies control to motors
void pidTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(20);
    unsigned long prevMillis = millis();

    while (true) {
        unsigned long now = millis();
        float dt = (now - prevMillis) / 1000.0f;
        prevMillis = now;

        RPMData target = drive.readRPMs();
        float measFL, measFR, measRL, measRR;
        encoderReader.getRPMs(measFL, measFR, measRL, measRR);

        int cmdFL = (int)pidFL.compute(target.frontLeft,  measFL, dt);
        int cmdFR = (int)pidFR.compute(target.frontRight, measFR, dt);
        int cmdRL = (int)pidRL.compute(target.rearLeft,   measRL, dt);
        int cmdRR = (int)pidRR.compute(target.rearRight,  measRR, dt);

        frontLeft.setSpeed(cmdFL);
        frontRight.setSpeed(cmdFR);
        rearLeft.setSpeed(cmdRL);
        rearRight.setSpeed(cmdRR);

        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

// TASK 4: Debug display, original format
void debugTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(100);
    while (true) {
        Serial.write("\033[2J\033[H");
        Serial.println("---------------------------------------------------------");
        Serial.println("       ENCODER COUNTS             RPM");
        Serial.println("---------------------------------------------------------");

        // Read raw encoder counts
        EncoderData enc = encoderReader.readEncoders();
        Serial.print("FL: "); Serial.print(enc.frontLeft);
        Serial.print("	 FR: "); Serial.print(enc.frontRight);
        Serial.print("	 RL: "); Serial.print(enc.rearLeft);
        Serial.print("	 RR: "); Serial.println(enc.rearRight);

        // Read target RPMs
        Serial.println("---------------------------------------------------------");
        Serial.println("       TARGET RPM");
        Serial.println("---------------------------------------------------------");
        RPMData tgt = drive.readRPMs();
        Serial.print("FL: "); Serial.print(tgt.frontLeft);
        Serial.print("	 FR: "); Serial.print(tgt.frontRight);
        Serial.print("	 RL: "); Serial.print(tgt.rearLeft);
        Serial.print("	 RR: "); Serial.println(tgt.rearRight);

        // Read measured RPMs from encoders
        float measFL, measFR, measRL, measRR;
        encoderReader.getRPMs(measFL, measFR, measRL, measRR);
        Serial.println("---------------------------------------------------------");
        Serial.println("       MEASURED RPM");
        Serial.println("---------------------------------------------------------");
        Serial.print("FL: "); Serial.print(measFL, 1);
        Serial.print("	 FR: "); Serial.print(measFR, 1);
        Serial.print("	 RL: "); Serial.print(measRL, 1);
        Serial.print("	 RR: "); Serial.println(measRR, 1);

        vTaskDelayUntil(&lastWakeTime, interval);
    }
}
    

void setup() {
    Serial.begin(115200);
    initNetwork();

    
  

    padDataMutex = xSemaphoreCreateMutex();

    frontLeftEncoder.attachSingleEdge(FL_ENCODER_A, FL_ENCODER_B);
    frontRightEncoder.attachSingleEdge(FR_ENCODER_A, FR_ENCODER_B);
    rearLeftEncoder.attachSingleEdge(RL_ENCODER_A, RL_ENCODER_B);
    rearRightEncoder.attachSingleEdge(RR_ENCODER_A, RR_ENCODER_B);
    encoderReader.begin();

    // Ensure motors start stopped and reset PID controllers
    frontLeft.setSpeed(0);
    frontRight.setSpeed(0);
    rearLeft.setSpeed(0);
    rearRight.setSpeed(0);
    pidFL.reset(); pidFR.reset(); pidRL.reset(); pidRR.reset();

    xTaskCreatePinnedToCore(espNowTask,       "ESPNowTask",       2048, NULL, 2, &espNowTaskHandle,    0);
    xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask",2048, NULL, 1, &motorControlTaskHandle,1);
    xTaskCreatePinnedToCore(pidTask,          "PIDTask",         2048, NULL, 1, &pidTaskHandle,         1);
    xTaskCreatePinnedToCore(debugTask,        "DebugTask",       4096, NULL, 1, &debugTaskHandle,       1);
}

void loop() {
    // Empty: tasks are running under FreeRTOS
}
