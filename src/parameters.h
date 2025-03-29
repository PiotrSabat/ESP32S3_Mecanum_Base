#include <Arduino.h>
//PWM Syglal for motors max 20kHz
//H-Bridge 

//Motor pins PWM LED
// Definicje pinów i kanałów
#define FL_PIN1 9       //Przedni lewy motor 1A M1A
#define FL_PIN2 10      //Przedni lewy motor 1B M1B
#define FL_CHANNEL1 0   //Kanał LEDC dla M1A
#define FL_CHANNEL2 1   //Kanał LEDC dla M1B

#define FR_PIN1 11    //Przedni prawy motor 2A M2A
#define FR_PIN2 12    //Przedni prawy motor 2B M2B
#define FR_CHANNEL1 2 //Kanał LEDC dla M2A
#define FR_CHANNEL2 3 //Kanał LEDC dla M2B

#define RL_PIN1 13      //Tylny lewy motor 2A M2A
#define RL_PIN2 14      //Tylny lewy motor 2B M2B
#define RL_CHANNEL1 4   //Kanał LEDC dla M2A
#define RL_CHANNEL2 5   //Kanał LEDC dla M2B

#define RR_PIN1 15      //Tylny prawy motor 1A M1A
#define RR_PIN2 16      //Tylny prawy motor 1B M1B
#define RR_CHANNEL1 6   //Kanał LEDC dla M1A
#define RR_CHANNEL2 7   //Kanał LEDC dla M1B

// Definicje pinów i kanałów dla enkoderów
// Enkoder Fron Left
#define FL_ENCODER_A 1
#define FL_ENCODER_B 2
// Enkoder Front Right
#define FR_ENCODER_A 4
#define FR_ENCODER_B 5
// Enkoder Rear Left
#define RL_ENCODER_A 6
#define RL_ENCODER_B 7
// Enkoder Rear Right
#define RR_ENCODER_A 17
#define RR_ENCODER_B 18


// delay    Opóźnienia, które ustawiają jak często ma być włączony TASK freeRTOS
static const int rate_1 = 50;    // ms
static const int rate_2 = 25;    // ms
static const int rate_3 = 35;    // ms

//Broadcast Address other ESP32 boards maximal 19
//uint8_t macPadFireBeetle[] = {0xEC, 0x62, 0x60, 0x5A, 0x6E, 0xFC};      Pad FireBeetle ESP32 wycofany z użycia, przeznaczony do pźniejszego wykorzystania
uint8_t macPadXiao[] = {0x34, 0x85, 0x18, 0x9E, 0x87, 0xD4};            // Pad Seeduino Xiao ESP32 S3
uint8_t macPlatformMecanum[] = {0xDC, 0xDA, 0x0C, 0x55, 0xD5, 0xB8};   //platforma mecanum z ESP32 S3 DEVKIT C-1 N8R2
uint8_t macMonitorDebug[] = {0xA0, 0xB7, 0x65,0x4B, 0xC5, 0x30};        //ESP 32 NodeMCU Dev Kit C V2 mit CP2102

// messages
// Struktura wiadomości z pada
typedef struct Message_from_Pad {
    uint32_t timestamp = 0;  // Heartbeat – bieżący czas (millis())
    uint32_t totalMessages = 0; // Liczba wysłanych wiadomości
    int16_t L_Joystick_x_message = 0;
    int16_t L_Joystick_y_message = 0;
    int16_t R_Joystick_x_message = 0;
    int16_t R_Joystick_y_message = 0;
    uint32_t L_Joystick_buttons_message = 0;
    uint32_t R_Joystick_buttons_message = 0;
    int16_t L_Joystick_raw_x = 0;
    int16_t L_Joystick_raw_y = 0;
    int16_t R_Joystick_raw_x = 0;
    int16_t R_Joystick_raw_y = 0;
} Message_from_Pad;

// Struktura wiadomości z platformy mecanum
typedef struct Message_from_Platform_Mecanum {
    uint16_t seqNum;
    int16_t frontLeftSpeed;
    int16_t frontRightSpeed;
    int16_t rearLeftSpeed;
    int16_t rearRightSpeed;
    float pitch;
    float roll;
    float yaw;
    float batteryVoltage;
} Message_from_Platform_Mecanum;







