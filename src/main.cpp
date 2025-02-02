#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "parameters.h"
#include "funkcje.h"
//#include "FirstSetupScreen.h"

// Definicja struktury danych z joysticków i przycisków
typedef struct struct_message_esp_now {
  uint32_t timestamp; // Znacznik czasu w milisekundach (4 bajty)
  int L_Joystick_x; // Wartość analogowa z lewego joysticka (4 bajty)
  int L_Joystick_y; // Wartość analogowa z lewego joysticka (4 bajty)
  int R_Joystick_x; // Wartość analogowa z prawego joysticka (4 bajty)
  int R_Joystick_y; // Wartość analogowa z prawego joysticka (4 bajty)
  bool button1; // Stan przycisku 1 (1 bajt)
  bool button2; // Stan przycisku 2 (1 bajt)
  bool button3; // Stan przycisku 3 (1 bajt)
  bool button4; // Stan przycisku 4 (1 bajt)
  bool button5; // Stan przycisku 5 (1 bajt)
  bool button6; // Stan przycisku 6 (1 bajt)
  bool button7; // Stan przycisku 7 (1 bajt)
  bool button8; // Stan przycisku 8 (1 bajt)
  bool button9; // Stan przycisku 9 (1 bajt)
  bool button10; // Stan przycisku 10 (1 bajt)
  bool button11; // Stan przycisku 11 (1 bajt)
  uint32_t message_id; // Unikalny identyfikator wiadomości (4 bajty)
  bool ack; // Potwierdzenie odbioru (1 bajt)
} struct_message_esp_now; // Łącznie: 4 + 4*4 + 11*1 + 4 + 1 = 33 bajty

// Create a structured object to receive
struct_message_esp_now myData_from_sender;


// Peer info
esp_now_peer_info_t peerInfo;

// Variables to track message reception
uint32_t last_received_message_id = 0;
int lost_message_count = 0;
const int max_lost_messages = 5; // Threshold for emergency stop
unsigned long last_received_time = 0;
const unsigned long message_timeout = 100; // Timeout in milliseconds

// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData_from_sender, incomingData, sizeof(myData_from_sender));
  // Check if the message is new and not a duplicate
  if (myData_from_sender.message_id > last_received_message_id) {
    last_received_message_id = myData_from_sender.message_id;
    lost_message_count = 0; // Reset lost message count
    last_received_time = millis(); // Update last received time
    // Process the received data
    Serial.println("Data received");
    // Send an acknowledgment back
    struct_message_esp_now ackData;
    ackData.ack = true;
    ackData.message_id = myData_from_sender.message_id;
    esp_now_send(macPadFireBeetle, (uint8_t *) &ackData, sizeof(ackData));
  }
}

// Function to check for lost messages and perform emergency stop
void CheckForLostMessages(void *parameter) {
  while (1) {
    if (millis() - last_received_time > message_timeout) {
      lost_message_count++;
      last_received_time = millis(); // Update last received time to avoid multiple increments
    }
    if (lost_message_count > max_lost_messages) {
      // Perform emergency stop
      Serial.println("Emergency stop triggered");
      // Add code to stop motors or perform other emergency actions
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Check every 10 ms
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Add peer
  memcpy(peerInfo.peer_addr, macPadFireBeetle, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Create task to check for lost messages
  xTaskCreate(CheckForLostMessages, "CheckForLostMessages", 2048, NULL, 1, NULL);
}

void loop() {
  // Nothing to do here
}