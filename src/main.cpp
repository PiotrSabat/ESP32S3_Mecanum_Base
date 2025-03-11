#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"
#include "MecanumDrive.h"

// Struktura danych z Pada
Message_from_Pad myData_from_Pad;


// Inicjalizacja silników
MotorDriverCytronH_Bridge frontLeft(FL_PIN1, FL_PIN2, FL_CHANNEL1, FL_CHANNEL2);
MotorDriverCytronH_Bridge frontRight(FR_PIN1, FR_PIN2, FR_CHANNEL1, FR_CHANNEL2);
MotorDriverCytronH_Bridge rearLeft(RL_PIN1, RL_PIN2, RL_CHANNEL1, RL_CHANNEL2);
MotorDriverCytronH_Bridge rearRight(RR_PIN1, RR_PIN2, RR_CHANNEL1, RR_CHANNEL2);

MecanumDrive drive(&frontLeft, &frontRight, &rearLeft, &rearRight);


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
        int x = myData_from_Pad.L_Joystick_x_message;
        int y = myData_from_Pad.L_Joystick_y_message;
        int yaw = myData_from_Pad.R_Joystick_x_message;

        // Martwa strefa
        if (abs(x) < DEAD_ZONE) x = 0;
        if (abs(y) < DEAD_ZONE) y = 0;
        if (abs(yaw) < DEAD_ZONE) yaw = 0;

        // Przekształcenie wartości joysticków na ruch mecanum
        drive.move(x, y, yaw);

        // Wysłanie danych zwrotnych
        Message_from_Platform_Mecanum feedback;
        feedback.seqNum++;
        feedback.frontLeftSpeed = frontLeft.getCurrentSpeed();
        feedback.frontRightSpeed = frontRight.getCurrentSpeed();
        feedback.rearLeftSpeed = rearLeft.getCurrentSpeed();
        feedback.rearRightSpeed = rearRight.getCurrentSpeed();

        feedback.pitch = 0.0; // Możesz podpiąć odczyt z IMU
        feedback.roll = 0.0;
        feedback.yaw = yaw;

        feedback.batteryVoltage = analogRead(34) * (3.3 / 4095.0) * 2.0;

        esp_err_t result = esp_now_send(macPadXiao, (uint8_t *) &feedback, sizeof(feedback));
        if (result != ESP_OK) {
            Serial.println("❌ Błąd wysyłania feedbacku");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Odświeżanie co 50 ms (20Hz)
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

    memcpy(peerInfo.peer_addr, macPadXiao, 6);
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
