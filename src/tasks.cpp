#include "tasks.h"
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "network.h"
#include "sensors.h"
#include "drive.h"
#include "parameters.h"
#include "messages.h"
#include "mac_addresses_private.h"


// Counter for total telemetry messages sent
static int32_t totalMessages = 0;

//--------------------------------------------------------------------------------
// Function: createTasks
// Description: Creates and pins all FreeRTOS tasks used in the system.
//--------------------------------------------------------------------------------
void createTasks() {
    // ESP-NOW telemetry task: low priority, pinned to core 0
    xTaskCreatePinnedToCore(
        espNowTask,
        "ESPNowTask",
        2048,
        NULL,
        1,
        NULL,
        0
    );

    // Motor control task: medium priority, pinned to core 1
    xTaskCreatePinnedToCore(
        motorControlTask,
        "MotorControlTask",
        2048,
        NULL,
        1,
        NULL,
        1
    );

    // PID regulation task: highest priority, pinned to core 1
    xTaskCreatePinnedToCore(
        pidTask,
        "PIDTask",
        2048,
        NULL,
        1,
        NULL,
        1
    );

    // Debug display task: lowest priority, pinned to core 1
    xTaskCreatePinnedToCore(
        debugTask,
        "DebugTask",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}

//--------------------------------------------------------------------------------
// Task: espNowTask
// Description: Periodically sends telemetry over ESP-NOW.
//--------------------------------------------------------------------------------
void espNowTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = INTERVAL_20MS;

    for (;;) {
        Message_from_Platform_Mecanum debugMsg = {};
        if (xSemaphoreTake(padDataMutex, portMAX_DELAY) == pdTRUE) {
            debugMsg.timestamp     = millis();
            debugMsg.totalMessages = totalMessages++;

            RPMData target = readRPMs();
            debugMsg.frontLeftSpeedRPM  = target.frontLeft;
            debugMsg.frontRightSpeedRPM = target.frontRight;
            debugMsg.rearLeftSpeedRPM   = target.rearLeft;
            debugMsg.rearRightSpeedRPM  = target.rearRight;

            EncoderData counts = readEncoders();
            debugMsg.frontLeftEncoder  = counts.frontLeft;
            debugMsg.frontRightEncoder = counts.frontRight;
            debugMsg.rearLeftEncoder   = counts.rearLeft;
            debugMsg.rearRightEncoder  = counts.rearRight;

            debugMsg.KP_message      = KP;
            debugMsg.KI_message      = KI;
            debugMsg.KD_message      = KD;
            debugMsg.MAX_OUT_message = MAX_OUT;
            debugMsg.MIN_OUT_message = MIN_OUT;

            xSemaphoreGive(padDataMutex);
        }

        esp_now_send(macMonitorDebug, (uint8_t *)&debugMsg, sizeof(debugMsg));
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

//--------------------------------------------------------------------------------
// Task: motorControlTask
// Description: Reads joystick data and updates drive targets.
//--------------------------------------------------------------------------------
void motorControlTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = INTERVAL_20MS;

    for (;;) {
        int x = 0, y = 0, yaw = 0;
        if (xSemaphoreTake(padDataMutex, portMAX_DELAY) == pdTRUE) {
            x   = myData_from_Pad.L_Joystick_x_message;
            y   = myData_from_Pad.L_Joystick_y_message;
            yaw = myData_from_Pad.R_Joystick_x_message;
            xSemaphoreGive(padDataMutex);
        }

        moveRPM(x, y, yaw);
       
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

//--------------------------------------------------------------------------------
// Task: pidTask
// Description: Runs PID loops and applies motor commands.
//--------------------------------------------------------------------------------
void pidTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = INTERVAL_20MS;
    unsigned long prevMillis = millis();

    for (;;) {
        unsigned long now = millis();
        float dt = (now - prevMillis) * 0.001f;
        prevMillis = now;

        updatePID(dt);
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

//--------------------------------------------------------------------------------
// Task: debugTask
// Description: Prints encoder counts, target and measured RPMs to Serial.
//--------------------------------------------------------------------------------
void debugTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(100);

    for (;;) {
        Serial.write("\033[2J\033[H");
        Serial.println("---------------------------------------------------------");
        Serial.println("       ENCODER COUNTS             RPM");
        Serial.println("---------------------------------------------------------");

        EncoderData counts = readEncoders();
        Serial.printf("FL: %ld FR: %ld RL: %ld RR: %ld\n",
                      counts.frontLeft, counts.frontRight,
                      counts.rearLeft,  counts.rearRight);

        Serial.println("---------------------------------------------------------");
        Serial.println("       TARGET RPM");
        Serial.println("---------------------------------------------------------");
        RPMData target = readRPMs();
        Serial.printf("FL: %.0f FR: %.0f RL: %.0f RR: %.0f\n",
                      target.frontLeft, target.frontRight,
                      target.rearLeft,  target.rearRight);

        Serial.println("---------------------------------------------------------");
        Serial.println("       MEASURED RPM");
        Serial.println("---------------------------------------------------------");
        RPMData measured = getRPMs();
        Serial.printf("FL: %.1f FR: %.1f RL: %.1f RR: %.1f\n",
                      measured.frontLeft, measured.frontRight,
                      measured.rearLeft,  measured.rearRight);

        vTaskDelayUntil(&lastWakeTime, interval);
    }
}
