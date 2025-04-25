#include <Arduino.h>



//#include "mac_addresses.h"
//#include "mac_addresses_private.h"  // Include only real MACs via build flag instead of both files

#include "network.h"
#include "sensors.h"
#include "drive.h"
#include "tasks.h"

   

void setup() {
    Serial.begin(115200);
    initNetwork();
    initEncoders();
    initDrive();
    createTasks();

}

void loop() {
    // Empty: tasks are running under FreeRTOS
}
