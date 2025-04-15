#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"
#include "MecanumDrive.h"
#include <ESP32Encoder.h>
#include "EncoderReader.h"
#include "funkcje.h"

// Struktura danych z pada
Message_from_Pad myData_from_Pad;


// Lokalne dane do sterowania (możesz ich nie używać, jeśli pracujesz tylko na myData_from_Pad)
typedef struct {
  int x;
  int y;
  int yaw;
} MovementData;

int32_t totalMessages = 0;

// Globalna zmienna i mutex do ochrony dostępu do danych
volatile MovementData movementData;
SemaphoreHandle_t movementMutex;

// Inicjalizacja silników
MotorDriverCytronH_Bridge rearRight(RR_PIN2, RR_PIN1, RR_CHANNEL2, RR_CHANNEL1);
MotorDriverCytronH_Bridge frontRight(FR_PIN2, FR_PIN1, FR_CHANNEL2, FR_CHANNEL1);
MotorDriverCytronH_Bridge frontLeft(FL_PIN1, FL_PIN2, FL_CHANNEL1, FL_CHANNEL2);
MotorDriverCytronH_Bridge rearLeft(RL_PIN1, RL_PIN2, RL_CHANNEL1, RL_CHANNEL2);

// Utworzenie obiektu klasy MecanumDrive
MecanumDrive drive(&frontLeft, &frontRight, &rearLeft, &rearRight);

// Definicje enkoderów
ESP32Encoder frontLeftEncoder;
ESP32Encoder frontRightEncoder;
ESP32Encoder rearLeftEncoder;
ESP32Encoder rearRightEncoder;

// Deklaracja wskaźnika do obiektu EncoderReader – zostanie stworzony w setup()
EncoderReader* encoderReader = nullptr;

// Peer info
esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfoMonitor;

// Task Handlers
TaskHandle_t espNowTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t encoderTaskHandle;

// CALLBACK ODBIORU ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Zabezpieczamy dostęp do myData_from_Pad
  if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
    if (len == sizeof(Message_from_Pad)) {
      memcpy((void*)&myData_from_Pad, incomingData, sizeof(Message_from_Pad));
    }
    xSemaphoreGive(movementMutex);
  }
}

// TASK 1: Wysyłanie danych przez ESP-NOW (tutaj tylko pusty loop)
void espNowTask(void *parameter) {
  while (1) {
    // Struktura danych do debugowania z platformy mecanum lokalnie dla zwiekszenia wydajnosci
    Message_from_Platform_Mecanum debugMsg;
    // Zabezpieczenie dostepu do danych za pomocą mutexu
    if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
      debugMsg.timestamp = millis();
      debugMsg.totalMessages = totalMessages;
      debugMsg.frontLeftSpeedRPM = drive.readRPMs().frontLeft;
      debugMsg.frontRightSpeedRPM = drive.readRPMs().frontRight;
      debugMsg.rearLeftSpeedRPM = drive.readRPMs().rearLeft;
      debugMsg.rearRightSpeedRPM = drive.readRPMs().rearRight;
      debugMsg.frontLeftEncoder = frontLeftEncoder.getCount();
      debugMsg.frontRightEncoder = frontRightEncoder.getCount();
      debugMsg.rearLeftEncoder = rearLeftEncoder.getCount();
      debugMsg.rearRightEncoder = rearRightEncoder.getCount();
      // do zaimplementowania
      debugMsg.pitch = 0;       
      debugMsg.roll = 0;
      debugMsg.yaw = 0;
      debugMsg.batteryVoltage = 0;
      xSemaphoreGive(movementMutex);
    }
    //wysłanie pakietu do MonitorDebug adres macMonotorDebug
    esp_err_t result = esp_now_send(macMonitorDebug, (uint8_t *)&debugMsg, sizeof(Message_from_Platform_Mecanum));
    if (result == ESP_OK) {
      //Serial.println("Wysłano dane do monitora");
      totalMessages++;
    }
    else {
        //Serial.println("Błąd wysyłania danych do monitora");
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// TASK 2: Sterowanie silnikami – pobieramy dane z pada chronione mutexem
void motorControlTask(void *parameter) {
  int x, y, yaw;
  while (1) {
    // Zabezpieczamy odczyt danych z myData_from_Pad
    if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
      x = myData_from_Pad.L_Joystick_x_message;
      y = myData_from_Pad.L_Joystick_y_message;
      yaw = myData_from_Pad.R_Joystick_x_message;
      xSemaphoreGive(movementMutex);
    }
    // Wywołujemy sterowanie, np. przez metodę moveRPM()
    drive.moveRPM(x, y, yaw);

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// TASK 3: Debug – odczytujemy i wyświetlamy dane (zabezpieczone mutexem)
void encoderTask(void *parameter) {
  while (1) {
    // Zabezpieczamy odczyt wspólnych danych – np. myData_from_Pad lub danych enkoderowych
    if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
      // Pobieramy RPM z enkoderów
      float rpmFL, rpmFR, rpmRL, rpmRR;
      encoderReader->getRPMs(rpmFL, rpmFR, rpmRL, rpmRR);
      // Pobieramy surowe odczyty z enkoderów
      EncoderData dataEncoder = encoderReader->readEncoders();
      // Pobieramy dane z MecanumDrive o zadanej prędkości
      RPMData dataRPM = drive.readRPMs();
      xSemaphoreGive(movementMutex);
      
      // Czyszczenie ekranu i wyświetlanie wyników w formacie tabelarycznym:
      Serial.write("\033[2J"); // Czyści ekran
      Serial.write("\033[H");  // Ustawia kursor w górnym lewym rogu

      // Wypisanie nagłówków
Serial.println("---------------------------------------------------------");
Serial.println("       ENCODER COUNTS             RPM");
Serial.println("---------------------------------------------------------");

// Wypisanie liczników enkoderów
Serial.print("FL: ");
Serial.print(dataEncoder.frontLeft);
Serial.print("\t FR: ");
Serial.print(dataEncoder.frontRight);
Serial.print("\t RL: ");
Serial.print(dataEncoder.rearLeft);
Serial.print("\t RR: ");
Serial.println(dataEncoder.rearRight);

// Wypisanie RPM z enkoderów


//wyświetlenie zadanej prędkości w RPM
Serial.println("---------------------------------------------------------");
Serial.println("       ZADANE RPM");
Serial.println("---------------------------------------------------------");
Serial.print("FL: ");
Serial.print(dataRPM.frontLeft);    
Serial.print("\t FR: ");
Serial.print(dataRPM.frontRight);
Serial.print("\t RL: ");
Serial.print(dataRPM.rearLeft);
Serial.print("\t RR: ");
Serial.println(dataRPM.rearRight);

Serial.print("FL: ");
Serial.print(rpmFL, 1);
Serial.print("\t FR: ");
Serial.print(rpmFR, 1);
Serial.print("\t RL: ");
Serial.print(rpmRL, 1);
Serial.print("\t RR: ");
Serial.println(rpmRR, 1);


    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

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

  //dodanie peera Monitor Debug
  
  memcpy(peerInfoMonitor.peer_addr, macMonitorDebug, 6);
  peerInfoMonitor.channel = 0;
  peerInfoMonitor.encrypt = false;
  if (esp_now_add_peer(&peerInfoMonitor) != ESP_OK) {
    Serial.println("Failed to add peer Monitor Debug");
    return;
  }

  // Tworzymy mutex
  movementMutex = xSemaphoreCreateMutex();
  if (movementMutex == NULL) {
    Serial.println("❌ Błąd tworzenia mutexu!");
    while (1) delay(100);
  }

  // Inicjalizacja enkoderów
  frontLeftEncoder.attachSingleEdge(FL_ENCODER_A, FL_ENCODER_B);
  frontRightEncoder.attachSingleEdge(FR_ENCODER_A, FR_ENCODER_B);
  rearLeftEncoder.attachSingleEdge(RL_ENCODER_A, RL_ENCODER_B);
  rearRightEncoder.attachSingleEdge(RR_ENCODER_A, RR_ENCODER_B);
  
  frontLeftEncoder.clearCount();
  frontRightEncoder.clearCount();
  rearLeftEncoder.clearCount();
  rearRightEncoder.clearCount();

  // Tworzymy obiekt EncoderReader przekazując rozdzielczość z parameters.h
  encoderReader = new EncoderReader(&frontLeftEncoder, &frontRightEncoder, &rearLeftEncoder, &rearRightEncoder, ENCODER_RESOLUTION);
  encoderReader->begin();
  
  Serial.println("✅ Inicjalizacja enkoderów zakończona");

  // Tworzenie tasków FreeRTOS
  xTaskCreatePinnedToCore(espNowTask, "ESPNowTask", 2048, NULL, 1, &espNowTaskHandle, 0);
  xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask", 2048, NULL, 1, &motorControlTaskHandle, 1);
  xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 2048, NULL, 1, &encoderTaskHandle, 1);

  Serial.println("✅ System gotowy!");
}

void loop() {
  // Pusta – działamy w FreeRTOS
}
