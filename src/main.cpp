#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>

#include "parameters.h"
#include "motor_config.h"
#include "Motor.h"
#include "MecanumDrive.h"
#include "messages.h"
#include "mac_addresses_private.h"

// ================= ESP-NOW =================
static Message_from_Pad myData_from_Pad;
static Message_from_Monitor receivedMonitorData;

SemaphoreHandle_t movementMutex;
SemaphoreHandle_t monitorMutex;

static int32_t totalMessages = 0;
static float motorCtrlAvgTime = 0.0f;

// ================= Silniki =================
Motor frontLeftMotor(FL_CONFIG);
Motor frontRightMotor(FR_CONFIG);
Motor rearLeftMotor(RL_CONFIG);
Motor rearRightMotor(RR_CONFIG);
MecanumDrive drive(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// ================= SPI (Arduino) =================
#define PIN_CS   21
#define PIN_MOSI 47
#define PIN_MISO 48
#define PIN_SCLK 45

void init_spi_master() {
    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI); // SCLK, MISO, MOSI
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);  // CS idle high
    Serial.println("✅ SPI (Arduino-style) initialized");
}

void spiReceiveTask(void* parameter) {
    uint8_t recvBuf[12];  // 3x float
    float pidKp, pidKi, pidKd;

    while (true) {
        digitalWrite(PIN_CS, LOW);
        for (int i = 0; i < 12; i++) {
            recvBuf[i] = SPI.transfer(0x00);
        }
        digitalWrite(PIN_CS, HIGH);

        memcpy(&pidKp, &recvBuf[0], sizeof(float));
        memcpy(&pidKi, &recvBuf[4], sizeof(float));
        memcpy(&pidKd, &recvBuf[8], sizeof(float));

        frontLeftMotor.setPID(pidKp, pidKi, pidKd);
        frontRightMotor.setPID(pidKp, pidKi, pidKd);
        rearLeftMotor.setPID(pidKp, pidKi, pidKd);
        rearRightMotor.setPID(pidKp, pidKi, pidKd);

        Serial.printf("🎯 PID from SPI: Kp=%.3f Ki=%.3f Kd=%.3f\n", pidKp, pidKi, pidKd);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ================= CALLBACK =================
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (len == sizeof(Message_from_Pad)) {
        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&myData_from_Pad, incomingData, sizeof(Message_from_Pad));
            xSemaphoreGive(movementMutex);
        }
    } else if (len == sizeof(Message_from_Monitor)) {
        if (xSemaphoreTake(monitorMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&receivedMonitorData, incomingData, sizeof(Message_from_Monitor));
            xSemaphoreGive(monitorMutex);
        }
    }
}

// ================= TASKI =================
void motorControlTask(void* parameter) {
    static uint64_t sumTime = 0;
    static uint32_t count = 0;

    int16_t x, y, yaw;

    for (;;) {
        uint64_t start = esp_timer_get_time();

        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            x = myData_from_Pad.L_Joystick_x_message;
            y = myData_from_Pad.L_Joystick_y_message;
            yaw = myData_from_Pad.R_Joystick_x_message;
            xSemaphoreGive(movementMutex);
        }

        drive.drive((float)x, (float)y, (float)yaw);
        drive.update();

        uint64_t duration = esp_timer_get_time() - start;
        sumTime += duration;
        count++;
        motorCtrlAvgTime = (float)sumTime / (float)count;

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MOTOR_CONTROL));
    }
}

void debugTask(void* parameter) {
    for (;;) {
        Message_from_Platform_Mecanum debugMsg;

        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            debugMsg.timestamp = millis();
            debugMsg.totalMessages = totalMessages;

            RPMData target = drive.readRPMs();
            debugMsg.frontLeftSpeedRPM = target.frontLeft;
            debugMsg.frontRightSpeedRPM = target.frontRight;
            debugMsg.rearLeftSpeedRPM = target.rearLeft;
            debugMsg.rearRightSpeedRPM = target.rearRight;

            debugMsg.frontLeftEncoder = 0;
            debugMsg.frontRightEncoder = 0;
            debugMsg.rearLeftEncoder = 0;
            debugMsg.rearRightEncoder = 0;

            debugMsg.pitch = 0;
            debugMsg.roll = 0;
            debugMsg.yaw = 0;
            debugMsg.batteryVoltage = 0;

            debugMsg.taskTime = motorCtrlAvgTime / 1000.0f;

            xSemaphoreGive(movementMutex);
        }

        esp_err_t res = esp_now_send(macMonitorDebug, reinterpret_cast<const uint8_t*>(&debugMsg), sizeof(debugMsg));
        if (res == ESP_OK) totalMessages++;

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_DEBUG_OUTPUT));
    }
}

void monitorUpdateTask(void* parameter) {
    while (1) {
        if (xSemaphoreTake(monitorMutex, portMAX_DELAY) == pdTRUE) {
            if (receivedMonitorData.type == MSG_SET_PID) {
                frontLeftMotor.setPID(receivedMonitorData.Kp, receivedMonitorData.Ki, receivedMonitorData.Kd);
                frontRightMotor.setPID(receivedMonitorData.Kp, receivedMonitorData.Ki, receivedMonitorData.Kd);
                rearLeftMotor.setPID(receivedMonitorData.Kp, receivedMonitorData.Ki, receivedMonitorData.Kd);
                rearRightMotor.setPID(receivedMonitorData.Kp, receivedMonitorData.Ki, receivedMonitorData.Kd);

                Serial.printf("✅ PID updated from MONITOR: Kp=%.3f Ki=%.3f Kd=%.3f\n",
                              receivedMonitorData.Kp, receivedMonitorData.Ki, receivedMonitorData.Kd);
                receivedMonitorData.type = 0;
            }
            xSemaphoreGive(monitorMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ================= SETUP + LOOP =================
void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerPad = {};
    memcpy(peerPad.peer_addr, macPadXiao, 6);
    peerPad.channel = ESP_CHANNEL;
    peerPad.encrypt = false;
    esp_now_add_peer(&peerPad);

    esp_now_peer_info_t peerDebug = {};
    memcpy(peerDebug.peer_addr, macMonitorDebug, 6);
    peerDebug.channel = ESP_CHANNEL;
    peerDebug.encrypt = false;
    esp_now_add_peer(&peerDebug);

    movementMutex = xSemaphoreCreateMutex();
    monitorMutex = xSemaphoreCreateMutex();

    init_spi_master();

    xTaskCreatePinnedToCore(motorControlTask,  "MotorCtrl",   4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(debugTask,         "Debug",       4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(monitorUpdateTask, "MonitorPID",  2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(spiReceiveTask,    "SPIReceive",  4096, nullptr, 1, nullptr, 1);

    Serial.println("✅ MainController ready");
}

void loop() {
    // All logic is in FreeRTOS tasks
}
