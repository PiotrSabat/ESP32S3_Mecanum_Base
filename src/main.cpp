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
//Średni czas tasku do debugowania
static float motorCtrlAvgTime = 0.0f;  // w µs

// Obiekty silników
static Motor frontLeftMotor(FL_CONFIG);
static Motor frontRightMotor(FR_CONFIG);
static Motor rearLeftMotor(RL_CONFIG);
static Motor rearRightMotor(RR_CONFIG);

// Kontroler Mecanum – odpowiada za kierunek i rozdział prędkości
static MecanumDrive drive(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// Peer info dla ESP-NOW
static esp_now_peer_info_t peerPad;
static esp_now_peer_info_t peerDebugMonitor;

// Callback ESP-NOW
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
        if (len == sizeof(Message_from_Pad)) {
            memcpy(&myData_from_Pad, incomingData, sizeof(Message_from_Pad));
        }
        xSemaphoreGive(movementMutex);
    }
}


// Zadanie sterowania silnikami
void motorControlTask(void* parameter) {
    //-----------dane do debugowania: czas wykonania tasku, do wykasowania w przyszłości
    static uint64_t sumTime = 0;
    static uint32_t count   = 0;
    //-----------koniec deklaracji zmiennych debugowania


    int16_t x, y, yaw;
    for (;;) {
        //-----------debugowanie: czas wykonania tasku
        uint64_t start = esp_timer_get_time();  // ✱ początek pomiaru
        //--------------koniec debugowania
        
        // Odczyt danych z pada


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

        //--------------debugowanie: czas wykonania tasku

        uint64_t duration = esp_timer_get_time() - start;  // ✱ koniec pomiaru
        sumTime += duration;
        count++;
        motorCtrlAvgTime = (float)sumTime / (float)count;  // średnia
        //--------------koniec debugowania


        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MOTOR_CONTROL));
    }
}

// Zadanie debugowe – wypis prędkości i wysyłanie ich przez ESP-NOW
// Wysyła również licznik wiadomości
void debugTask(void* parameter) {
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

            // Czas wykonania tasku - debugownie do wykasowania w przyszlosci
            debugMsg.taskTime = motorCtrlAvgTime / 1000.0f;  // w ms

            xSemaphoreGive(movementMutex);
        }
        // Wysyłka
        esp_err_t res = esp_now_send(macMonitorDebug, reinterpret_cast<const uint8_t*>(&debugMsg), sizeof(debugMsg));
        if (res == ESP_OK) totalMessages++;
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
    memcpy(peerPad.peer_addr, macPadXiao, 6);
    peerPad.channel = ESP_CHANNEL;
    peerPad.encrypt = false;
    esp_now_add_peer(&peerPad);

    memcpy(peerDebugMonitor.peer_addr, macMonitorDebug, 6);
    peerDebugMonitor.channel = ESP_CHANNEL;
    peerDebugMonitor.encrypt = false;
    esp_now_add_peer(&peerDebugMonitor);

    // Mutex
    movementMutex = xSemaphoreCreateMutex();
    if (!movementMutex) {
        Serial.println("❌ Nie udało się utworzyć mutexu");
        while (1) vTaskDelay(100);
    }

    // Zadania FreeRTOS
    xTaskCreatePinnedToCore(motorControlTask, "MotorCtrlTask", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(debugTask,        "DebugTask",      4096, nullptr, 1, nullptr, 1);

    Serial.println("✅ System ready");
}

void loop() {
    // Obsługa w zadaniach
}
