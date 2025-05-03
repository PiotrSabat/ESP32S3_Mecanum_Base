#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "parameters.h"
#include "motor_config.h"
#include "Motor.h"
#include "MecanumDrive.h"
#include "messages.h"
#include "mac_addresses_private.h"

// Struktura odbierana z pada
static Message_from_Pad myData_from_Pad;
// Mutex chroniący dostęp do danych
static SemaphoreHandle_t movementMutex;
// Licznik wiadomości wysłanych w debugu
static int32_t totalMessages = 0;

// Obiekty silników
static Motor frontLeftMotor(FL_CONFIG);
static Motor frontRightMotor(FR_CONFIG);
static Motor rearLeftMotor(RL_CONFIG);
static Motor rearRightMotor(RR_CONFIG);

// Kontroler Mecanum – odpowiada za kierunek i rozdział prędkości
static MecanumDrive drive(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// Peer info dla ESP-NOW
static esp_now_peer_info_t peerInfo;
static esp_now_peer_info_t peerInfoMonitor;

// Callback ESP-NOW
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
        if (len == sizeof(Message_from_Pad)) {
            memcpy(&myData_from_Pad, incomingData, sizeof(Message_from_Pad));
        }
        xSemaphoreGive(movementMutex);
    }
}

// Zadanie wysyłające dane debugowe przez ESP-NOW
void espNowTask(void* parameter) {
    for (;;) {
        Message_from_Platform_Mecanum debugMsg;
        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            debugMsg.timestamp = millis();
            debugMsg.totalMessages = totalMessages;

            // Prędkości zadane
            RPMData target = drive.readRPMs();
            debugMsg.frontLeftSpeedRPM  = target.frontLeft;
            debugMsg.frontRightSpeedRPM = target.frontRight;
            debugMsg.rearLeftSpeedRPM   = target.rearLeft;
            debugMsg.rearRightSpeedRPM  = target.rearRight;

            // Brak surowych liczników – zostawiamy zero
            debugMsg.frontLeftEncoder  = 0;
            debugMsg.frontRightEncoder = 0;
            debugMsg.rearLeftEncoder   = 0;
            debugMsg.rearRightEncoder  = 0;

            debugMsg.pitch = 0;
            debugMsg.roll  = 0;
            debugMsg.yaw   = 0;
            debugMsg.batteryVoltage = 0;

            xSemaphoreGive(movementMutex);
        }
        // Wysyłka
        esp_err_t res = esp_now_send(macMonitorDebug, reinterpret_cast<const uint8_t*>(&debugMsg), sizeof(debugMsg));
        if (res == ESP_OK) totalMessages++;
        vTaskDelay(pdMS_TO_TICKS(INTERVAL_DEBUG_OUTPUT));
    }
}

// Zadanie sterowania silnikami
void motorControlTask(void* parameter) {
    int16_t x, y, yaw;
    for (;;) {
        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            x   = myData_from_Pad.L_Joystick_x_message;
            y   = myData_from_Pad.L_Joystick_y_message;
            yaw = myData_from_Pad.R_Joystick_x_message;
            xSemaphoreGive(movementMutex);
        }
        // Kinematyka Mecanum
        drive.drive((float)x, (float)y, (float)yaw);
        // Aktualizacja pętli PID
        drive.update();
        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MOTOR_CONTROL));
    }
}

// Zadanie debugowe – wypis prędkości
void debugTask(void* parameter) {
    for (;;) {
        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            // Odczyt prędkości rzeczywistej
            float rpmFL = frontLeftMotor.getCurrentRPM();
            float rpmFR = frontRightMotor.getCurrentRPM();
            float rpmRL = rearLeftMotor.getCurrentRPM();
            float rpmRR = rearRightMotor.getCurrentRPM();
            // Prędkości zadane
            RPMData target = drive.readRPMs();
            xSemaphoreGive(movementMutex);

            // Czyszczenie terminala
            Serial.write("\033[2J");
            Serial.write("\033[H");
            Serial.println("----- TARGET RPM -----");
            Serial.printf("FL: %.1f\tFR: %.1f\tRL: %.1f\tRR: %.1f\n",
                          target.frontLeft, target.frontRight, target.rearLeft, target.rearRight);
            Serial.println("----- ACTUAL RPM -----");
            Serial.printf("FL: %.1f\tFR: %.1f\tRL: %.1f\tRR: %.1f\n",
                          rpmFL, rpmFR, rpmRL, rpmRR);
        }
        vTaskDelay(pdMS_TO_TICKS(INTERVAL_DEBUG_OUTPUT));
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    // Dodanie peerów
    memcpy(peerInfo.peer_addr, macPadXiao, 6);
    peerInfo.channel = ESP_CHANNEL;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    memcpy(peerInfoMonitor.peer_addr, macMonitorDebug, 6);
    peerInfoMonitor.channel = ESP_CHANNEL;
    peerInfoMonitor.encrypt = false;
    esp_now_add_peer(&peerInfoMonitor);

    // Mutex
    movementMutex = xSemaphoreCreateMutex();
    if (!movementMutex) {
        Serial.println("❌ Nie udało się utworzyć mutexu");
        while (1) vTaskDelay(100);
    }

    // Zadania FreeRTOS
    xTaskCreatePinnedToCore(espNowTask,       "ESPNowTask",     4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(motorControlTask, "MotorCtrlTask", 4096, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(debugTask,        "DebugTask",      4096, nullptr, 1, nullptr, 1);

    Serial.println("✅ System ready");
}

void loop() {
    // Obsługa w zadaniach
}
