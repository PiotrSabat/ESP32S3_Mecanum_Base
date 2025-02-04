#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"

// Struktura danych z Pada
Message_from_Pad myData_from_Pad;

// Definicja silników
MotorDriverCytronH_Bridge motorFL(LF_M1A, LF_M1B, 0, 1);
MotorDriverCytronH_Bridge motorFR(FR_M2A, FR_M2B, 2, 3);
MotorDriverCytronH_Bridge motorRL(LB_M2A, LB_M2B, 4, 5);
MotorDriverCytronH_Bridge motorRR(BR_M1A, BR_M1B, 6, 7);

// Peer info
esp_now_peer_info_t peerInfo;

// FreeRTOS Task Handlers
TaskHandle_t espNowTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t batteryMonitorTaskHandle;

// ===== CALLBACK ODBIORU ESP-NOW =====
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(Message_from_Pad)) {
        memcpy(&myData_from_Pad, incomingData, sizeof(myData_from_Pad));
    }
}

// ===== TASK 1: ODBIERANIE DANYCH Z ESP-NOW =====
void espNowTask(void *parameter) {
    while (1) {
        vTaskDelay(20 / portTICK_PERIOD_MS); // Odświeżanie co 20ms (50Hz)
    }
}

// ===== TASK 2: STEROWANIE SILNIKAMI =====
void motorControlTask(void *parameter) {
    while (1) {
        // Mapowanie wartości joysticka na zakres PWM (-100 do 100)
        int speedFL = myData_from_Pad.L_Joystick_y_message + myData_from_Pad.L_Joystick_x_message;
        int speedFR = myData_from_Pad.L_Joystick_y_message - myData_from_Pad.L_Joystick_x_message;
        int speedRL = myData_from_Pad.L_Joystick_y_message + myData_from_Pad.L_Joystick_x_message;
        int speedRR = myData_from_Pad.L_Joystick_y_message - myData_from_Pad.L_Joystick_x_message;

        // Ograniczenie wartości do przedziału -255 do 255
        speedFL = constrain(speedFL, -255, 255);
        speedFR = constrain(speedFR, -255, 255);
        speedRL = constrain(speedRL, -255, 255);
        speedRR = constrain(speedRR, -255, 255);

        // Ustawienie prędkości silników
        motorFL.setSpeed(speedFL);
        motorFR.setSpeed(speedFR);
        motorRL.setSpeed(speedRL);
        motorRR.setSpeed(speedRR);

        vTaskDelay(50 / portTICK_PERIOD_MS); // Odświeżanie co 50ms (20Hz)
    }
}

// ===== TASK 3: MONITOROWANIE BATERII (opcjonalnie) =====
void batteryMonitorTask(void *parameter) {
    while (1) {
        float batteryVoltage = analogRead(34) * (3.3 / 4095.0) * 2.0; // Przykładowe odczytanie napięcia
        Serial.printf("Napięcie baterii: %.2fV\n", batteryVoltage);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Odświeżanie co 1s
    }
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    memcpy(peerInfo.peer_addr, macPadFireBeetle, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Tworzenie tasków FreeRTOS
    xTaskCreatePinnedToCore(espNowTask, "ESPNowTask", 2048, NULL, 1, &espNowTaskHandle, 0);
    xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask", 2048, NULL, 1, &motorControlTaskHandle, 1);
    xTaskCreatePinnedToCore(batteryMonitorTask, "BatteryMonitorTask", 2048, NULL, 1, &batteryMonitorTaskHandle, 1);

    Serial.println("✅ System gotowy!");
}

void loop() {
    // Pusta, wszystko działa w FreeRTOS
}
