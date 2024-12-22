//PWM Syglal for motors max 20kHz
//H-Bridge 

//Motor pins PWM LED
#define LF_M1A 9       //Przedni lewy motor 1A
#define LF_M1B 10       //Przedni lewy motor 1B
#define LB_M2A 6       //Tylny lewy motor 2A
#define LB_M2B 7       //Tylny lewy motor 2B

#define FR_M2A 11       //Przedni prawy motor 2A 
#define FR_M2B 12       //Przedni prawy motor 2B
#define BR_M1A 15       //Tylny prawy motor 1A
#define BR_M1B 16      //Tylny prawy motor 1B



// delay    Opóźnienia, które ustawiają jak często ma być włączony TASK freeRTOS
static const int rate_1 = 50;    // ms
static const int rate_2 = 25;     // ms
static const int rate_3 = 35;     // ms




