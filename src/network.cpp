#include "network.h"


// Global mutex and data buffer
SemaphoreHandle_t padDataMutex = nullptr;
Message_from_Pad myData_from_Pad = {};

// ESP-NOW receive callback (WiFi task context)
static void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(myData_from_Pad)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Copy safely from ISR context
        if (xSemaphoreTakeFromISR(padDataMutex, &xHigherPriorityTaskWoken) == pdTRUE) {
            memcpy(&myData_from_Pad, incomingData, sizeof(myData_from_Pad));
            xSemaphoreGiveFromISR(padDataMutex, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR();
        }
    }
}

void initNetwork() {
    // Create mutex for controller data
    padDataMutex = xSemaphoreCreateMutex();
    if (padDataMutex == nullptr) {
        Serial.println("[network] Failed to create padDataMutex");
    }

    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[network] ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    // Add controller peer
    {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, macPadXiao, 6);
        peerInfo.channel = ESP_CHANNEL;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("[network] Failed to add pad peer");
        }
    }

    // Add monitor/debug peer
    {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, macMonitorDebug, 6);
        peerInfo.channel = ESP_CHANNEL;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("[network] Failed to add monitor peer");
        }
    }
}
