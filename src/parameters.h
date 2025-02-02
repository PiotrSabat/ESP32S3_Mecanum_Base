//PWM Syglal for motors max 20kHz
//H-Bridge 

//Motor pins PWM LED
#define LF_M1A 9       //Przedni lewy motor 1A
#define LF_M1B 10       //Przedni lewy motor 1B
#define FR_M2A 11       //Przedni prawy motor 2A 
#define FR_M2B 12       //Przedni prawy motor 2B

#define LB_M2A 13       //Tylny lewy motor 2A
#define LB_M2B 14       //Tylny lewy motor 2B
#define BR_M1A 15       //Tylny prawy motor 1A
#define BR_M1B 16      //Tylny prawy motor 1B



// delay    Opóźnienia, które ustawiają jak często ma być włączony TASK freeRTOS
static const int rate_1 = 50;    // ms
static const int rate_2 = 25;    // ms
static const int rate_3 = 35;    // ms

//Broadcast Address other ESP32 boards maximal 19
uint8_t macPadFireBeetle[] = {0xEC, 0x62, 0x60, 0x5A, 0x6E, 0xFC};      //Pad FireBeetle ESP32
uint8_t macPlatformMecanum[] = {0xDC, 0xDA, 0x0C, 0x55, 0xD5, 0xB8};   //platforma mecanum z ESP32 S3 DEVKIT C-1 N8R2
uint8_t macModulXiao[] = {0x34, 0x85, 0x18, 0x9E, 0x87, 0xD4};          //Seeduino Xiao ESP32 S3
uint8_t macMonitorDebug[] = {0xA0, 0xB7, 0x65,0x4B, 0xC5, 0x30};        //ESP 32 NodeMCU Dev Kit C V2 mit CP2102

// messages
typedef struct Message_from_Pad {
    uint16_t seqNum;
    int16_t L_Joystick_x_message;
    int16_t L_Joystick_y_message;
    int16_t R_Joystick_x_message;
    int16_t R_Joystick_y_message;
    bool L_Joystick_button_message;
    bool R_Joystick_button_message;
    int16_t L_Joystick_raw_x;
    int16_t L_Joystick_raw_y;
    int16_t R_Joystick_raw_x;
    int16_t R_Joystick_raw_y;
} struct_message_from_Pad;

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
} struct_message_from_Platform_Mecanum;






