#pragma once

#include <Arduino.h>

// ===== Motor PWM Definitions =====
// PWM signal frequency for motor drivers: 20 kHz, 9-bit resolution

// Front Left Motor
constexpr int FL_PIN1 = 9;        // M1A
constexpr int FL_PIN2 = 10;       // M1B
constexpr int FL_CHANNEL1 = 0;    // PWM channel for M1A
constexpr int FL_CHANNEL2 = 1;    // PWM channel for M1B

// Front Right Motor
constexpr int FR_PIN1 = 11;       // M2A
constexpr int FR_PIN2 = 12;       // M2B
constexpr int FR_CHANNEL1 = 2;
constexpr int FR_CHANNEL2 = 3;

// Rear Left Motor
constexpr int RL_PIN1 = 13;       // M3A
constexpr int RL_PIN2 = 14;       // M3B
constexpr int RL_CHANNEL1 = 4;
constexpr int RL_CHANNEL2 = 5;

// Rear Right Motor
constexpr int RR_PIN1 = 15;       // M4A
constexpr int RR_PIN2 = 16;       // M4B
constexpr int RR_CHANNEL1 = 6;
constexpr int RR_CHANNEL2 = 7;

// ===== Encoder Pin Definitions =====

// Front Left Encoder
constexpr int FL_ENCODER_A = 1;
constexpr int FL_ENCODER_B = 2;

// Front Right Encoder
constexpr int FR_ENCODER_A = 4;
constexpr int FR_ENCODER_B = 5;

// Rear Left Encoder
constexpr int RL_ENCODER_A = 6;
constexpr int RL_ENCODER_B = 7;

// Rear Right Encoder
constexpr int RR_ENCODER_A = 17;
constexpr int RR_ENCODER_B = 18;

// ===== Encoder and Motor Configuration =====


constexpr int MAX_RPM = 180;             // Maximum motor speed
// ===== Safety Parameters =====
constexpr uint32_t DEFAULT_SOFT_STOP_DURATION_MS = 500;  // Czas domyślnego soft stopu (ms)
constexpr uint32_t DEFAULT_HARD_STOP_DURATION_MS = 50;  // Czas domyślnego hard stopu (ms)

// ----- Ograniczenie przyspieszenia (ochrona zabezpieczenia prądowego) -----
// O ile RPM na sekundę może NARASTAĆ zadana prędkość koła. Bez tego regulator
// przy ruszaniu wystawia od razu pełne wypełnienie PWM, cztery silniki ruszają
// jak zwarcie i zabezpieczenie prądowe ogniw odcina zasilanie.
// Zjazd zadanej do zera NIE jest ograniczany — hamowanie awaryjne musi zostać
// tak szybkie, jak było.
// Mniejsza wartość = łagodniejszy rozruch i mniejszy prąd, ale wolniejsza reakcja.
constexpr float MAX_ACCEL_RPM_PER_S = 300.0f;

// ----- Ograniczenie hamowania silnikiem (hamowanie przeciwprądem) -----
// Gdy regulator chce podać napięcie PRZECIWNE do bieżącego kierunku obrotu
// (puszczony drążek, zmiana kierunku), prąd nie jest ograniczony rezystancją
// uzwojenia jak przy rozruchu — do napięcia baterii dodaje się siła
// elektromotoryczna wirującego silnika. Przy pełnym rewersie z maksymalnej
// prędkości prąd jest WIĘKSZY niż przy zwarciu i zabezpieczenie ogniw odcina.
//
// Przy przekładni 120:1 platforma i tak zatrzymuje się sama na tarciu, więc
// hamowanie silnikiem jest tylko dodatkiem, a nie koniecznością.
// 0 = całkowity brak hamowania silnikiem (czysty wybieg, zero prądu hamowania).
// Większa wartość = mocniejsze hamowanie, ale wyższy prąd.
// Zaczynamy od 0, bo to jedyna wartość gwarantująca brak zadziałania
// zabezpieczenia; przekładnia i tak zatrzymuje platformę. Jeśli hamowanie
// okaże się zbyt leniwe, podnoś po 50 i sprawdzaj po każdym kroku.
constexpr int MAX_BRAKING_PWM = 0;

// Poniżej tej prędkości koło uznajemy za stojące i ograniczenie hamowania
// przestaje obowiązywać — inaczej nie dałoby się ruszyć w drugą stronę.
constexpr float BRAKING_RPM_THRESHOLD = 8.0f;

// ----- Failsafe: utrata łączności z padem -----
// Pad nadaje co 50 ms. Cisza dłuższa niż PAD_LINK_TIMEOUT_MS oznacza zerwane
// łącze (6 zgubionych ramek) i uruchamia automatyczne hamowanie.
// Za mała wartość = fałszywe alarmy przy chwilowych zakłóceniach,
// za duża = robot dłużej jedzie bez kontroli. Strojone na sprzęcie.
constexpr uint32_t PAD_LINK_TIMEOUT_MS = 300;

// Po wykryciu ciszy napęd jest ODCINANY (hardStop), a nie hamowany silnikiem:
// przy przekładni 120:1 platforma zatrzymuje się sama na tarciu przekładni,
// więc aktywne hamowanie tylko ciągnęłoby prąd bez zysku na drodze hamowania.



// ===== Task Scheduling Rates (in milliseconds) =====

constexpr int INTERVAL_MOTOR_CONTROL = 20;   // Interval for motor control task
constexpr int INTERVAL_SENSOR_READ = 25;     // Interval for sensor read task
constexpr int INTERVAL_DEBUG_OUTPUT = 50;    // Interval for debug/telemetry



// ===== PID Control Constants =====

constexpr float KP = 0.5f;
constexpr float KI = 0.0f;
constexpr float KD = 0.00f;
constexpr float MAX_OUT = 511.0f;
constexpr float MIN_OUT = -511.0f;

// ===== Timing Constants =====
constexpr TickType_t INTERVAL_1MS = pdMS_TO_TICKS(1);
constexpr TickType_t INTERVAL_5MS = pdMS_TO_TICKS(5);
constexpr TickType_t INTERVAL_10MS = pdMS_TO_TICKS(10);
constexpr TickType_t INTERVAL_20MS = pdMS_TO_TICKS(20);
constexpr TickType_t INTERVAL_50MS = pdMS_TO_TICKS(50);
constexpr TickType_t INTERVAL_100MS = pdMS_TO_TICKS(100);

// ===== ESP-NOW Configuration =====
constexpr int ESP_CHANNEL = 0;  // ESP-NOW channel
constexpr int ESP_MAX_DATA_SIZE = 250;  // Maximum data size for ESP-NOW


// ===== Default Motor Configuration =====
constexpr int DEFAULT_GEAR_RATIO = 960; // Gear ratio for the motors
constexpr int DEFAULT_PWM_RESOLUTION = 9; // PWM resolution (9 bits)
constexpr int DEFAULT_PWM_FREQUENCY = 20000; // PWM frequency (20 kHz)