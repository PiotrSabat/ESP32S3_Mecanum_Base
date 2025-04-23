#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "messages.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mac_addresses_private.h"
#include "parameters.h"


// Mutex protecting access to pad input data
extern SemaphoreHandle_t padDataMutex;
// Latest data received from controller
extern Message_from_Pad myData_from_Pad;

// Initialize WiFi/ESP-NOW and create necessary synchronization primitives
void initNetwork();