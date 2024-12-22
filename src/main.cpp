#include <Arduino.h>

/*********************
* Receiver XIAO ESP32S3
* 0.1 version with freeRTOS
* ESP NOW
* Two way communication
* Date: 10.10.2023
* Author: Sabat
* License: 0BSD
**********************/

// Include Libraries
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "MotorDriverCytronH_Bridge.h"


/****** ESP NOW Connection ******/

// Data from Sender
// Define a data structure
typedef struct struct_message_from_joystick {
  int a;
  int b;
  int c;
  int d;
} struct_message_from_joystick;

// Create a structured object
struct_message_from_joystick myData_from_joystick;
// End of Data from Sender

// Data to send back to Sender
// MAC Address of Sender
uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0x4B, 0xC5, 0x30}; //ESP32 1 A0:B7:65:4B:C5:30

// Define a data structure
typedef struct struct_message_from_xiao {
  int a;
  int b;
  int c;
  int d;
} struct_message_from_xiao;

//Create a object Motor FL
MotorDriverCytronH_Bridge MotorFL(LF_M1A, LF_M1B, 0, 1);
MotorDriverCytronH_Bridge MotorFR(FR_M2A, FR_M2B, 2, 3);
MotorDriverCytronH_Bridge MotorBL(LB_M2A, LB_M2B, 4, 5);
MotorDriverCytronH_Bridge MotorBR(BR_M1A, BR_M1B, 6, 7);

// Create a structured object
struct_message_from_xiao myData_to_send;

//Peer info
esp_now_peer_info_t peerInfo;

//Callback function alled when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData_from_joystick, incomingData, sizeof(myData_from_joystick));
  
}

/******** TASKS ********/
/******* freeRTOS ******/

void ESP_Now_Send_Meesages(void *parameter) {
  while(1) {
    myData_to_send.a = myData_from_joystick.a;
    myData_to_send.b = myData_from_joystick.b;
    myData_to_send.c = myData_from_joystick.c;
    myData_to_send.d = myData_from_joystick.d;
    MotorFL.setSpeed(myData_from_joystick.b + myData_from_joystick.a + myData_from_joystick.c);
    MotorFR.setSpeed(myData_from_joystick.b + myData_from_joystick.a - myData_from_joystick.c);
    MotorBL.setSpeed(myData_from_joystick.b - myData_from_joystick.a + myData_from_joystick.c);
    MotorBR.setSpeed(myData_from_joystick.b - myData_from_joystick.a - myData_from_joystick.c);
  // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData_to_send, sizeof(myData_to_send));
   
    if (result != ESP_OK) {
    Serial.println("Sending error!!!!");
     }
    vTaskDelay(rate_1 / portTICK_PERIOD_MS);
  }
}

/******* END TASKS **********/
/******* freeRTOS ***********/







void setup() {
  

  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);


   // Define callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Task to run ESP NOW
  xTaskCreatePinnedToCore(                    // Use xTaskCreate() in vanilla FreeRTOS
              ESP_Now_Send_Meesages,          // Function to be called
              "ESP Now send message",         // Name of task
              2048,                           // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,                           // Parameter to pass to function
              1,                              // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,                           // Task handle
              0);

}
 
void loop() {

}