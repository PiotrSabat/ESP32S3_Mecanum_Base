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


        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// TASK 4: ODCZYTYWANIE ENKODERÓW
void encoderTask(void *parameter) {
    while (1) {
        // Przykładowy fragment kodu wyświetlający dane z enkoderów oraz RPM

// Najpierw czyszczenie ekranu (ANSI escape codes)
Serial.write("\033[2J");  // Czyści cały ekran
Serial.write("\033[H");   // Ustawia kursor w górnym lewym rogu

// Pobieramy RPM z enkoderów
float rpmFL, rpmFR, rpmRL, rpmRR;
encoderReader->getRPMs(rpmFL, rpmFR, rpmRL, rpmRR);

// Pobieramy surowe odczyty liczników
EncoderData data = encoderReader->readEncoders();

// Wypisanie nagłówków
Serial.println("---------------------------------------------------------");
Serial.println("       ENCODER COUNTS             RPM");
Serial.println("---------------------------------------------------------");

// Wypisanie liczników enkoderów
Serial.print("FL: ");
Serial.print(data.frontLeft);
Serial.print("\t FR: ");
Serial.print(data.frontRight);
Serial.print("\t RL: ");
Serial.print(data.rearLeft);
Serial.print("\t RR: ");
Serial.println(data.rearRight);

// Wypisanie RPM (zaokrąglone do 1 miejsca po przecinku)
Serial.print("RPM: ");
Serial.print(rpmFL, 1);
Serial.print("\t ");
Serial.print(rpmFR, 1);
Serial.print("\t ");
Serial.print(rpmRL, 1);
Serial.print("\t ");
Serial.println(rpmRR, 1);



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
