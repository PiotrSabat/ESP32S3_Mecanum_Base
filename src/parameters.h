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


// Prędkość maksymalna koła. Wartość ZMIERZONA (3 m w 10,3 s oraz 5 obrotów
// platformy w 13 s, 2026-08-28), nie założona. Silnik ma 160 RPM bez
// obciążenia przy 6 V, pod obciążeniem wychodzi około 95 — stąd 90 z zapasem,
// żeby regulator miał czym dociągnąć do zadanej.
constexpr int MAX_RPM = 90;              // Maximum motor speed

// ===== Wejście z pada =====
// Pad zwraca wartości z zakresu ±511 (getCorrectedValue sprowadza się do
// raw - offset przy 10-bitowym ADC). Bez przeliczenia na MAX_RPM pełną
// prędkość osiągało się już przy ~35% wychylenia drążka, a reszta skoku
// nie robiła nic.
constexpr int JOYSTICK_MAX = 511;

// Martwa strefa wokół położenia spoczynkowego. Poza wyeliminowaniem pełzania
// od szumu ADC sprawia, że zadana prędkość jest DOKŁADNIE zerowa przy
// puszczonych drążkach — a od tego zależy wygaszanie wyjścia i całki na postoju.
constexpr int JOYSTICK_DEADZONE = 12;
// ===== Sterowanie skrętem =====
// Prawy drążek steruje CIASNOŚCIĄ ŁUKU, a nie wprost prędkością obrotu.
//
// Wcześniej sterował wprost obrotem i miał tę samą władzę co drążek jazdy.
// Skutek: przy pełnym gazie i pełnym obrocie wychodziło lewa 180 / prawa 0 —
// koła po wewnętrznej stały, a platforma zaciągała zamiast wybierać łuk.
// Koła wewnętrzne nie mogły zacząć kontrować, bo normalizacja skaluje wszystkie
// cztery tym samym współczynnikiem i zero pozostaje zerem. Cały zakres
// łagodnych łuków ściskał się w pierwszej jednej trzeciej skoku drążka.
//
// Wariant TERENOWY (wybrany świadomie): ostry zakręt przy pełnej prędkości
// pozostaje dostępny. Zmienia się rozkład czułości, a nie maksimum.

// Jak mocno mają pracować koła po WEWNĘTRZNEJ stronie przy pełnym wychyleniu
// drążka obrotu i pełnym gazie. Wartość jest ułamkiem prędkości kół
// zewnętrznych: 0 = stoją, wartość ujemna = kontrują.
//
// To jest właściwa pokrętka do strojenia odczucia zakrętu, bo opisuje wprost
// to, co widać na podłodze. Przy -0.23 (tak wychodziło przy pierwszym podejściu)
// wewnętrzne koła głównie hamowały; przy -0.55 obie strony pracują.
//
// Uwaga: to jest wartość ZADANA. Ile z niej faktycznie dojdzie do skutku
// w chwili wchodzenia w zakręt, ogranicza limit momentu przeciwnego
// (MAX_HOLDING_PWM) — koło toczące się jeszcze do przodu nie dostanie pełnego
// rewersu, bo to właśnie ten przypadek wywala zabezpieczenie ogniw.
constexpr float TURN_INNER_RATIO_FULL = -0.55f;

// Wzmocnienie wynikające z powyższego. Przy pełnym gazie i pełnym wychyleniu
// koła dostają vy*(1+G) i vy*(1-G), a normalizacja dzieli przez (1+G), więc
// stosunek wewnętrznych do zewnętrznych wynosi (1-G)/(1+G) = r. Stąd:
constexpr float TURN_GAIN = (1.0f - TURN_INNER_RATIO_FULL) /
                            (1.0f + TURN_INNER_RATIO_FULL);

// Krzywa drążka obrotu: 0 = liniowa, 1 = czysto sześcienna. Zagęszcza
// rozdzielczość wokół środka, nie ruszając końcówki zakresu. Podniesiona
// razem z TURN_GAIN, żeby mocniejsza końcówka nie zabrała łagodnych łuków
// z okolic środka drążka.
constexpr float TURN_EXPO = 0.75f;

// Poniżej tej prędkości jazdy drążek wraca do roli obrotu w miejscu. Bez tego
// przy zatrzymanej platformie nie dałoby się obrócić w ogóle, bo łuk przy
// zerowej prędkości nie istnieje. Przejście jest płynne, nie skokowe.
constexpr float PIVOT_BLEND_RPM = 20.0f;

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
// Wartość przeskalowana razem z MAX_RPM (było 300 przy MAX_RPM = 180), żeby
// fizyczne przyspieszenie pozostało dokładnie takie samo.
constexpr float MAX_ACCEL_RPM_PER_S = 150.0f;

// ----- Ograniczenie hamowania silnikiem (hamowanie przeciwprądem) -----
// Gdy regulator chce podać napięcie PRZECIWNE do bieżącego kierunku obrotu
// (puszczony drążek, zmiana kierunku), prąd nie jest ograniczony rezystancją
// uzwojenia jak przy rozruchu — do napięcia baterii dodaje się siła
// elektromotoryczna wirującego silnika. Przy pełnym rewersie z maksymalnej
// prędkości prąd jest WIĘKSZY niż przy zwarciu i zabezpieczenie ogniw odcina.
//
// Przy przekładni 120:1 platforma i tak zatrzymuje się sama na tarciu, więc
// hamowanie silnikiem jest tylko dodatkiem, a nie koniecznością.
// Limit jest ZALEŻNY OD PRĘDKOŚCI, bo koszt prądowy momentu przeciwnego też
// od niej zależy: prąd = (U_baterii + SEM) / R, a SEM rośnie z obrotami.
// Przy stojącym kole SEM wynosi zero, więc przeciwstawianie się obrotowi jest
// tanie — i właśnie dzięki temu platforma trzyma pozycję, gdy ktoś próbuje
// pchnąć koło ręką. Przy pełnej prędkości limit spada do zera, czyli
// hamowanie odbywa się wybiegiem, bez ryzyka dla zabezpieczenia ogniw.
//
// To wartość przy ZEROWEJ prędkości; maleje liniowo do zera przy MAX_RPM.
// Mniejsza = słabsze trzymanie pozycji, ale niższy prąd.
constexpr int MAX_HOLDING_PWM = 250;

// Poniżej tej prędkości koło uznajemy za stojące. Próg musi być wyraźnie
// powyżej szumu pomiaru (przy 1920 zliczeniach na obrót i cyklu 20 ms jedno
// zliczenie to ok. 1,6 RPM), a jednocześnie na tyle niski, by ręczne kręcenie
// kołem zawsze go przekraczało — inaczej silnik nie zauważyłby, że ktoś go rusza.
// Przeskalowany razem z MAX_RPM (było 5.0), żeby próg fizyczny się nie zmienił.
constexpr float STANDSTILL_RPM = 2.5f;

// ----- Failsafe: utrata łączności z padem -----
// Pad nadaje co 20 ms. Cisza dłuższa niż PAD_LINK_TIMEOUT_MS oznacza zerwane
// łącze (15 zgubionych ramek) i uruchamia automatyczne hamowanie. Wartość
// została w milisekundach bez zmian mimo przyspieszenia nadawania: dla
// bezpieczeństwa liczy się czas jazdy bez kontroli, a nie liczba ramek.
// Za mała wartość = fałszywe alarmy przy chwilowych zakłóceniach,
// za duża = robot dłużej jedzie bez kontroli. Strojone na sprzęcie.
constexpr uint32_t PAD_LINK_TIMEOUT_MS = 300;

// Po wykryciu ciszy napęd jest ODCINANY (hardStop), a nie hamowany silnikiem:
// przy przekładni 120:1 platforma zatrzymuje się sama na tarciu przekładni,
// więc aktywne hamowanie tylko ciągnęłoby prąd bez zysku na drodze hamowania.



// ===== Task Scheduling Rates (in milliseconds) =====

constexpr int INTERVAL_MOTOR_CONTROL = 20;   // Interval for motor control task
constexpr int INTERVAL_SENSOR_READ = 25;     // Interval for sensor read task
// Telemetria NIE chodzi na własnym timerze — jest odpowiedzią na ramkę z Pada.
// Własny timer oznaczał trzecią niezsynchronizowaną pętlę i przypadkową fazę
// wobec wysyłki z Pada, przez co droga w obie strony skakała o cały okres.
// Odpowiadanie od razu daje krótszy i STABILNY czas reakcji przy mniejszej
// liczbie ramek — czyli mniej prądu i mniej ciepła. Ten sam kierunek jest
// wymuszony przez docelowe łącze (ELRS Air Port): żadnej ramki „bo minął czas".
constexpr uint32_t TELEMETRY_EVERY_N_PAD_FRAMES = 2;  // Pad 50 Hz -> telemetria 25 Hz

// Awaryjne tempo, gdy z Pada nic nie przychodzi — żeby telemetria nie zamilkła
// zupełnie i dało się zobaczyć stan platformy także po utracie łączności.
constexpr uint32_t TELEMETRY_IDLE_MS = 200;

// Ogłaszanie wersji protokołu (MSG_HELLO). Dopóki nie widać partnera nadajemy
// często, żeby po jego restarcie szybko wrócić do jazdy; potem rzadko, bo
// zgodność wersji nie zmienia się w trakcie pracy.
constexpr int HELLO_INTERVAL_SEARCH_MS = 1000;
constexpr int HELLO_INTERVAL_IDLE_MS   = 5000;



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
// ZLICZENIA enkodera na obrót KOŁA — nie impulsy!
// Producent podaje 8 impulsów na obrót wału silnika, czyli 960 na obrót koła
// po przekładni 120:1. Ale ESP32Encoder w trybie attachHalfQuad zlicza OBA
// ZBOCZA kanału A (pos_mode=DEC, neg_mode=INC), więc na jeden impuls przypadają
// DWA zliczenia: 960 * 2 = 1920.
//
// Wpisane tu wcześniej 960 sprawiało, że platforma mierzyła prędkość DWA RAZY
// ZA WYSOKĄ. Regulator dochodził do zadanej, która w rzeczywistości była
// połową żądanej, i o tym nie wiedział — wykryte dopiero stoperem: ekran
// pokazywał 0,55 m/s, a 3 m zajmowały 10,3 s, czyli 0,29 m/s.
constexpr int DEFAULT_GEAR_RATIO = 1920;
constexpr int DEFAULT_PWM_RESOLUTION = 9; // PWM resolution (9 bits)
constexpr int DEFAULT_PWM_FREQUENCY = 20000; // PWM frequency (20 kHz)