#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"
#include "MecanumDrive.h"
#include <ESP32Encoder.h>
#include "EncoderReader.h"

// Data structure from the controller
Message_from_Pad myData_from_Pad;

// Local control data (you can ignore these if you only use myData_from_Pad)
typedef struct {
  int x;
  int y;
  int yaw;
} MovementData;

int32_t totalMessages = 0;

// Global variable and mutex to protect data access
volatile MovementData movementData;
SemaphoreHandle_t movementMutex;

// Motor initialization
MotorDriverCytronH_Bridge rearRight(RR_PIN2, RR_PIN1, RR_CHANNEL2, RR_CHANNEL1);
MotorDriverCytronH_Bridge frontRight(FR_PIN2, FR_PIN1, FR_CHANNEL2, FR_CHANNEL1);
MotorDriverCytronH_Bridge frontLeft(FL_PIN1, FL_PIN2, FL_CHANNEL1, FL_CHANNEL2);
MotorDriverCytronH_Bridge rearLeft(RL_PIN1, RL_PIN2, RL_CHANNEL1, RL_CHANNEL2);

// Create MecanumDrive object
MecanumDrive drive(&frontLeft, &frontRight, &rearLeft, &rearRight);

// Encoder definitions
ESP32Encoder frontLeftEncoder;
ESP32Encoder frontRightEncoder;
ESP32Encoder rearLeftEncoder;
ESP32Encoder rearRightEncoder;

// Pointer to EncoderReader object — will be created in setup()
EncoderReader* encoderReader = nullptr;

// Peer info
esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfoMonitor;

// Task handles
TaskHandle_t espNowTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t encoderTaskHandle;

// ESP-NOW receive callback
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Protect access to myData_from_Pad
  if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
    if (len == sizeof(Message_from_Pad)) {
      memcpy((void*)&myData_from_Pad, incomingData, sizeof(Message_from_Pad));
    }
    xSemaphoreGive(movementMutex);
  }
}

// TASK 1: Sending data via ESP-NOW (empty loop here)
void espNowTask(void *parameter) {
  while (1) {
    // Local debug data structure for the mecanum platform to improve performance
    Message_from_Platform_Mecanum debugMsg;
    // Protect data access with mutex
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
      // Not yet implemented
      debugMsg.pitch = 0;       
      debugMsg.roll = 0;
      debugMsg.yaw = 0;
      debugMsg.batteryVoltage = 0;
      xSemaphoreGive(movementMutex);
    }
    // Send packet to MonitorDebug (address macMonitorDebug)
    esp_err_t result = esp_now_send(macMonitorDebug, (uint8_t *)&debugMsg, sizeof(Message_from_Platform_Mecanum));
    if (result == ESP_OK) {
      //Serial.println("Sent data to monitor");
      totalMessages++;
    }
    else {
      //Serial.println("Error sending data to monitor");
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// TASK 2: Motor control – fetch controller data protected by mutex
void motorControlTask(void *parameter) {
  int x, y, yaw;
  while (1) {
    // Protect read of myData_from_Pad
    if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
      x = myData_from_Pad.L_Joystick_x_message;
      y = myData_from_Pad.L_Joystick_y_message;
      yaw = myData_from_Pad.R_Joystick_x_message;
      xSemaphoreGive(movementMutex);
    }
    // Call control method, e.g., moveRPM()
    drive.moveRPM(x, y, yaw);

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// TASK 3: Debug – read and display data (protected by mutex)
void encoderTask(void *parameter) {
  while (1) {
    // Protect shared data access – e.g., myData_from_Pad or encoder data
    if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
      // Get RPMs from encoders
      float rpmFL, rpmFR, rpmRL, rpmRR;
      encoderReader->getRPMs(rpmFL, rpmFR, rpmRL, rpmRR);
      // Get raw encoder counts
      EncoderData dataEncoder = encoderReader->readEncoders();
      // Get target RPM data from MecanumDrive
      RPMData dataRPM = drive.readRPMs();
      xSemaphoreGive(movementMutex);
      
      // Clear screen and display results in table format:
      Serial.write("\033[2J"); // Clear screen
      Serial.write("\033[H");  // Move cursor to top-left

      // Print headers
      Serial.println("---------------------------------------------------------");
      Serial.println("       ENCODER COUNTS             RPM");
      Serial.println("---------------------------------------------------------");

      // Print encoder counts
      Serial.print("FL: ");
      Serial.print(dataEncoder.frontLeft);
      Serial.print("\t FR: ");
      Serial.print(dataEncoder.frontRight);
      Serial.print("\t RL: ");
      Serial.print(dataEncoder.rearLeft);
      Serial.print("\t RR: ");
      Serial.println(dataEncoder.rearRight);

      // Print target RPM values
      Serial.println("---------------------------------------------------------");
      Serial.println("       TARGET RPM");
      Serial.println("---------------------------------------------------------");
      Serial.print("FL: ");
      Serial.print(dataRPM.frontLeft);    
      Serial.print("\t FR: ");
      Serial.print(dataRPM.frontRight);
      Serial.print("\t RL: ");
      Serial.print(dataRPM.rearLeft);
      Serial.print("\t RR: ");
      Serial.println(dataRPM.rearRight);

      // Print actual RPM from encoders
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

  // Add Monitor Debug peer
  memcpy(peerInfoMonitor.peer_addr, macMonitorDebug, 6);
  peerInfoMonitor.channel = 0;
  peerInfoMonitor.encrypt = false;
  if (esp_now_add_peer(&peerInfoMonitor) != ESP_OK) {
    Serial.println("Failed to add Monitor Debug peer");
    return;
  }

  // Create mutex
  movementMutex = xSemaphoreCreateMutex();
  if (movementMutex == NULL) {
    Serial.println("❌ Error creating mutex!");
    while (1) delay(100);
  }

  // Initialize encoders
  frontLeftEncoder.attachSingleEdge(FL_ENCODER_A, FL_ENCODER_B);
  frontRightEncoder.attachSingleEdge(FR_ENCODER_A, FR_ENCODER_B);
  rearLeftEncoder.attachSingleEdge(RL_ENCODER_A, RL_ENCODER_B);
  rearRightEncoder.attachSingleEdge(RR_ENCODER_A, RR_ENCODER_B);
  
  frontLeftEncoder.clearCount();
  frontRightEncoder.clearCount();
  rearLeftEncoder.clearCount();
  rearRightEncoder.clearCount();

  // Create EncoderReader object with resolution from parameters.h
  encoderReader = new EncoderReader(&frontLeftEncoder, &frontRightEncoder, &rearLeftEncoder, &rearRightEncoder, ENCODER_RESOLUTION);
  encoderReader->begin();
  
  Serial.println("✅ Encoder initialization complete");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(espNowTask, "ESPNowTask", 2048, NULL, 1, &espNowTaskHandle, 0);
  xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask", 2048, NULL, 1, &motorControlTaskHandle, 1);
  xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 2048, NULL, 1, &encoderTaskHandle, 1);

  Serial.println("✅ System ready!");
}

void loop() {
  // Empty – running under FreeRTOS
}
