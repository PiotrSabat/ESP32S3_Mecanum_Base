#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"
#include "MecanumDrive.h"
#include <ESP32Encoder.h>
#include "EncoderReader.h"

// Struktura danych z pada
Message_from_Pad myData_from_Pad;
 
// Lokalne dane do sterowania kołami
typedef struct {
    int x;
    int y;
    int yaw;
} MovementData;

// Globalna zmienna i mutex do ochrony struktury danych
volatile MovementData movementData;
SemaphoreHandle_t movementMutex;

// Inicjalizacja silników
MotorDriverCytronH_Bridge rearRight(RR_PIN2, RR_PIN1, RR_CHANNEL2, RR_CHANNEL1);
MotorDriverCytronH_Bridge frontRight(FR_PIN2, FR_PIN1, FR_CHANNEL2, FR_CHANNEL1);
MotorDriverCytronH_Bridge frontLeft(FL_PIN1, FL_PIN2, FL_CHANNEL1, FL_CHANNEL2);
MotorDriverCytronH_Bridge rearLeft(RL_PIN1, RL_PIN2, RL_CHANNEL1, RL_CHANNEL2);

MecanumDrive drive(&frontLeft, &frontRight, &rearLeft, &rearRight);

// Definicje enkoderów
ESP32Encoder frontLeftEncoder;
ESP32Encoder frontRightEncoder;
ESP32Encoder rearLeftEncoder;
ESP32Encoder rearRightEncoder;

// Deklaracja wskaźnika do obiektu EncoderReader – obiekt zostanie stworzony w setup()
EncoderReader* encoderReader = nullptr;

// Peer info
esp_now_peer_info_t peerInfo;

// FreeRTOS Task Handlers
TaskHandle_t espNowTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t encoderTaskHandle;

// CALLBACK ODBIORU ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(Message_from_Pad)) {
        memcpy(&myData_from_Pad, incomingData, sizeof(myData_from_Pad));
    }
}

// TASK 1: ODBIERANIE DANYCH Z ESP-NOW
void espNowTask(void *parameter) {
    while (1) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// TASK 2: STEROWANIE SILNIKAMI
void motorControlTask(void *parameter) {
    while (1) {
        int x = myData_from_Pad.L_Joystick_x_message;
        int y = myData_from_Pad.L_Joystick_y_message;
        int yaw = myData_from_Pad.R_Joystick_x_message;

        if (abs(x) < DEAD_ZONE) x = 0;
        if (abs(y) < DEAD_ZONE) y = 0;
        if (abs(yaw) < DEAD_ZONE) yaw = 0;

        drive.move(x, y, yaw);

        Message_from_Platform_Mecanum feedback;
        feedback.seqNum++;
        feedback.frontLeftSpeed = frontLeft.getCurrentSpeed();
        feedback.frontRightSpeed = frontRight.getCurrentSpeed();
        feedback.rearLeftSpeed = rearLeft.getCurrentSpeed();
        feedback.rearRightSpeed = rearRight.getCurrentSpeed();
        feedback.pitch = 0.0;
        feedback.roll = 0.0;
        feedback.yaw = yaw;

        esp_err_t result = esp_now_send(macPadXiao, (uint8_t*)&feedback, sizeof(feedback));
        if (result != ESP_OK) {
            Serial.println("❌ Błąd wysyłania feedbacku");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// TASK 4: ODCZYTYWANIE ENKODERÓW
void encoderTask(void *parameter) {
    while (1) {
        EncoderData data = encoderReader->readEncoders();
        Serial.print("Front Left: ");
        Serial.print(data.frontLeft);
        Serial.print(", Front Right: ");
        Serial.print(data.frontRight);
        Serial.print(", Rear Left: ");
        Serial.print(data.rearLeft);
        Serial.print(", Rear Right: ");
        Serial.println(data.rearRight);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// SETUP
void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

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

    movementMutex = xSemaphoreCreateMutex();
    if (movementMutex == NULL) {
        Serial.println("❌ Błąd tworzenia mutexu!");
        while (1) delay(100);
    }

    frontLeftEncoder.attachSingleEdge(FL_ENCODER_A, FL_ENCODER_B);
    frontRightEncoder.attachSingleEdge(FR_ENCODER_A, FR_ENCODER_B);
    rearLeftEncoder.attachSingleEdge(RL_ENCODER_A, RL_ENCODER_B);
    rearRightEncoder.attachSingleEdge(RR_ENCODER_A, RR_ENCODER_B);
    //delay(500);
    frontLeftEncoder.clearCount();
    frontRightEncoder.clearCount();
    rearLeftEncoder.clearCount();
    rearRightEncoder.clearCount();
    //delay(500);

    // Tworzymy obiekt EncoderReader przekazując rozdzielczość z parameters.h
    encoderReader = new EncoderReader(&frontLeftEncoder, &frontRightEncoder, &rearLeftEncoder, &rearRightEncoder, ENCODER_RESOLUTION);
    encoderReader->begin();
    //delay(500);
    Serial.println("✅ Inicjalizacja enkoderów zakończona");
    

    xTaskCreatePinnedToCore(espNowTask, "ESPNowTask", 2048, NULL, 1, &espNowTaskHandle, 0);
    xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask", 2048, NULL, 1, &motorControlTaskHandle, 1);
    xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 2048, NULL, 1, &encoderTaskHandle, 1);

    Serial.println("✅ System gotowy!");
}

void loop() {
    // Pusta – działamy w FreeRTOS
}
