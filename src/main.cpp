#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "parameters.h"
#include "motor_config.h"
#include "Motor.h"
#include "MecanumDrive.h"
#include "messages.h"
#include "pad_input.h"
#if __has_include("mac_addresses_private.h")
  #include "mac_addresses_private.h"
#else
  #include "mac_addresses.h"
#endif

// Skalowanie wskaźników na Padzie opiera się na MAX_RPM_TELEMETRY z messages.h.
// Ten warunek pilnuje, żeby nie rozjechało się z rzeczywistym MAX_RPM platformy.
static_assert(MAX_RPM_TELEMETRY == MAX_RPM * 10,
              "MAX_RPM_TELEMETRY w messages.h nie zgadza sie z MAX_RPM!");

// Ostatnia ramka sterująca odebrana z pada
static Msg_PadControl padCtrl;

// Ostatnia odebrana komenda zmiany nastaw PID
static Msg_SetPID receivedPidCmd;
static volatile bool pidCmdPending = false;

// Mutex chroniący dostęp do danych
static SemaphoreHandle_t movementMutex;

// Mutex chroniący dostęp do przysłanych nastaw PID
SemaphoreHandle_t pidMutex;


// ---- Failsafe: nadzór łączności z padem ----
// Zapisywane w OnDataRecv (task WiFi, rdzeń 0), czytane w motorControlTask
// (rdzeń 1). Oba dostępy odbywają się pod movementMutex.
static volatile uint32_t lastPadMsgMs = 0;    // millis() ostatniej ramki z pada
static volatile bool     padEverSeen  = false; // czy przyszła choć jedna ramka
static volatile bool     failsafeActive = false; // podglądane przez telemetrię

// ---- Stan protokołu (patrz nagłówek messages.h) ----
// padProtoOk jest ZATRZASKIEM: raz zobaczona zgodna wersja zostaje. Pojedyncze
// zgubione HELLO nie może zatrzymać jadącej platformy — od wykrywania ciszy
// jest failsafe, który mierzy czas od ostatniej ramki sterującej.
static volatile uint8_t  padProtoVersion = 0;
static volatile bool     padProtoOk      = false;
static volatile uint32_t padHelloCount   = 0;
static volatile uint32_t protoErrorCount = 0;   // ramki nieznanego typu/długości
static volatile uint8_t  lastUnknownType = 0;
static volatile int      lastUnknownLen  = 0;

// ---- Statystyka strat ramek z pada (luki w numeracji seq) ----
static volatile uint32_t padSeqLast   = 0;
static volatile uint32_t padRecvCount = 0;
static volatile uint32_t padMissCount = 0;

// Licznik wiadomości wysłanych w debugu
static int32_t totalMessages = 0;
//Średni czas tasku do debugowania
static volatile float motorCtrlAvgTime = 0.0f;  // w µs

// Obiekty silników
static Motor frontLeftMotor(FL_CONFIG);
static Motor frontRightMotor(FR_CONFIG);
static Motor rearLeftMotor(RL_CONFIG);
static Motor rearRightMotor(RR_CONFIG);

// Kontroler Mecanum – odpowiada za kierunek i rozdział prędkości
static MecanumDrive drive(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// Uchwyt zadania telemetrii — budzone po odebraniu ramki z Pada, zamiast
// chodzić na własnym timerze.
static TaskHandle_t telemetryTaskHandle = nullptr;

// Peer info dla ESP-NOW
static esp_now_peer_info_t peerPad;

// Callback ESP-NOW.
// Typ wiadomości rozpoznajemy po PIERWSZYM BAJCIE, a długość sprawdzamy przed
// memcpy jako walidację (patrz nagłówek messages.h). Ramka nieznanego typu albo
// o niezgodnej długości jest LICZONA I ZGŁASZANA, a nie ignorowana po cichu —
// wcześniej nie odróżnialiśmy „nic nie przyszło" od „przyszło coś obcego".
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    static uint32_t padFrameCounter = 0;
    if (len < 1) return;
    const uint8_t type = incomingData[0];

    switch (type) {
    case MSG_PAD_CONTROL:
        if (len != (int)sizeof(Msg_PadControl)) break;
        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&padCtrl, incomingData, sizeof(Msg_PadControl));
            // Znacznik żywotności łącza — jedyne miejsce, gdzie jest odświeżany
            lastPadMsgMs = millis();
            padEverSeen  = true;
            // Luka w numeracji = ramki zgubione po drodze. Pierwsza ramka po
            // starcie tylko synchronizuje licznik. Restart Pada cofa seq, więc
            // warunek „>" chroni przed policzeniem ujemnej straty.
            if (padRecvCount > 0 && padCtrl.seq > padSeqLast + 1) {
                padMissCount += padCtrl.seq - padSeqLast - 1;
            }
            padSeqLast = padCtrl.seq;
            padRecvCount++;
            xSemaphoreGive(movementMutex);
        }
        // Telemetria jako ODPOWIEDŹ, nie jako niezależny timer. Wysyłka nie
        // odbywa się tutaj — z callbacku ESP-NOW nie należy nadawać — tylko
        // budzimy zadanie, które to zrobi w swoim kontekście.
        if (++padFrameCounter % TELEMETRY_EVERY_N_PAD_FRAMES == 0 &&
            telemetryTaskHandle != nullptr) {
            xTaskNotifyGive(telemetryTaskHandle);
        }
        return;

    case MSG_HELLO: {
        if (len != (int)sizeof(Msg_Hello)) break;
        Msg_Hello hello;
        memcpy(&hello, incomingData, sizeof(hello));
        if (hello.role != ROLE_PAD) break;
        padProtoVersion = hello.protoVersion;
        padHelloCount++;
        if (hello.protoVersion == PROTO_VERSION) padProtoOk = true;
        return;
    }

    case MSG_SET_PID:
        if (len != (int)sizeof(Msg_SetPID)) break;
        if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&receivedPidCmd, incomingData, sizeof(Msg_SetPID));
            pidCmdPending = true;
            xSemaphoreGive(pidMutex);
        }
        return;

    default:
        break;
    }

    // Wszystko, czego nie umiemy zinterpretować.
    protoErrorCount++;
    lastUnknownType = type;
    lastUnknownLen  = len;
}

// Ogłaszanie wersji protokołu. Powtarzane, a nie uzgadniane raz na starcie:
// ESP-NOW nie zna pojęcia sesji, więc Pad może zniknąć i wrócić po resecie
// w dowolnej chwili, a wtedy musi dowiedzieć się wszystkiego od nowa.
void helloTask(void* parameter) {
    for (;;) {
        Msg_Hello hello = {};
        hello.msgType      = MSG_HELLO;
        hello.protoVersion = PROTO_VERSION;
        hello.role         = ROLE_PLATFORM;
        hello.fwBuildId    = FW_BUILD_ID;
        hello.uptimeMs     = millis();
        esp_now_send(macPadXiao, reinterpret_cast<const uint8_t*>(&hello), sizeof(hello));

        // Dopóki nie widać Pada — często. Potem rzadko: zgodność wersji nie
        // zmienia się w trakcie pracy, więc nie ma po co zajmować eteru.
        vTaskDelay(pdMS_TO_TICKS(padProtoOk ? HELLO_INTERVAL_IDLE_MS
                                            : HELLO_INTERVAL_SEARCH_MS));
    }
}


// Zadanie sterowania silnikami
void motorControlTask(void* parameter) {
    //-----------dane do debugowania: czas wykonania tasku, do wykasowania w przyszłości
    static uint64_t sumTime = 0;
    static uint32_t count   = 0;
    //-----------koniec deklaracji zmiennych debugowania


    int16_t x = 0, y = 0, yaw = 0;
    bool linkAlive = false;
    bool failsafeEngaged = false;  // zatrzask: hamowanie zlecane raz na epizod

    // vTaskDelayUntil, NIE vTaskDelay — patrz komentarz przy końcu pętli.
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        //-----------debugowanie: czas wykonania tasku
        uint64_t start = esp_timer_get_time();  // ✱ początek pomiaru
        //--------------koniec debugowania

        // Odczyt danych z pada + ocena żywotności łącza


        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            x   = padCtrl.axisLX;
            y   = padCtrl.axisLY;
            yaw = padCtrl.axisRX;
            // Odejmowanie na uint32_t jest odporne na przepełnienie millis().
            // padProtoOk w tym warunku znaczy: NIE JEDZIEMY, dopóki nie wiemy,
            // że Pad mówi tą samą wersją protokołu. Jazda na danych czytanych
            // według cudzej wersji struktury byłaby gorsza niż stanie.
            linkAlive = padEverSeen && padProtoOk &&
                        (millis() - lastPadMsgMs) < PAD_LINK_TIMEOUT_MS;
            xSemaphoreGive(movementMutex);
        }

        if (linkAlive) {
            if (failsafeEngaged) {
                failsafeEngaged = false;
                failsafeActive  = false;
                Serial.println("✅ Łączność z padem przywrócona");
            }
            // Kinematyka Mecanum — wejście przeliczone z jednostek drążka na RPM.
            // Obrót liczony na końcu, bo zależy od prędkości jazdy.
            const float vxRpm = padAxisToRPM(x);
            const float vyRpm = padAxisToRPM(y);
            drive.drive(vxRpm, vyRpm, padAxisToOmega(yaw, vxRpm, vyRpm));
        } else if (!failsafeEngaged) {
            // Odcinamy zasilanie silników dokładnie raz. Przekładnia 120:1
            // zatrzyma platformę sama — hamowanie silnikiem tylko ciągnęłoby
            // prąd, ryzykując zadziałanie zabezpieczenia ogniw.
            // Potem NIE wołamy drive(), bo setTargetRPM() skasowałoby stan
            // HardStopped i wznowiło jazdę na starych danych z joysticka.
            failsafeEngaged = true;
            failsafeActive  = true;
            drive.hardStop();
            if (!padProtoOk) {
                Serial.printf("⛔ Pad mówi protokołem v%u, my v%u — nie jadę\n",
                              (unsigned)padProtoVersion, (unsigned)PROTO_VERSION);
            } else {
                Serial.println("⚠️ Utrata łączności z padem — odcięcie napędu");
            }
        }

        // Aktualizacja pętli PID — wołana ZAWSZE, także w failsafe: podtrzymuje
        // odcięcie napędu i odświeża zmierzoną prędkość na potrzeby telemetrii.
        drive.update();

        //--------------debugowanie: czas wykonania tasku

        uint64_t duration = esp_timer_get_time() - start;  // ✱ koniec pomiaru
        sumTime += duration;
        count++;
        motorCtrlAvgTime = (float)sumTime / (float)count;  // średnia
        //--------------koniec debugowania


        // Okres MUSI być stały, dlatego vTaskDelayUntil zamiast vTaskDelay.
        // vTaskDelay odmierza przerwę OD ZAKOŃCZENIA pracy, więc okres wynosił
        // 20 ms plus czas wykonania pętli — realnie 21 ms. Regulator dostawał
        // przez to inne dt, niż zakładały nastawy, a zmierzone obroty siadały
        // na siatce kwantyzacji przesuniętej względem tej właściwej (stąd
        // „zawsze dziewiątka" na końcu odczytu RPM na Padzie).
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(INTERVAL_MOTOR_CONTROL));
    }
}

// Zadanie telemetryczne — zbiera stan napędu i wysyła go przez ESP-NOW.
//
// UWAGA co do znaków: targetRPM i measuredRPM są w wewnętrznej konwencji
// silnika, czyli dla prawej strony ODWRÓCONE względem „dodatnie = do przodu"
// (patrz invertDirection w motor_config.h). Są przez to spójne WZGLĘDEM SIEBIE
// i o to chodzi — trójka target/measured/pwm ma służyć porównaniu kół, a nie
// odczytaniu kierunku jazdy.
void telemetryTask(void* parameter) {
    Motor* wheels[4] = { &frontLeftMotor, &frontRightMotor,
                         &rearLeftMotor,  &rearRightMotor };
    const bool wheelInverted[4] = { FL_CONFIG.invertDirection, FR_CONFIG.invertDirection,
                                    RL_CONFIG.invertDirection, RR_CONFIG.invertDirection };
    uint32_t telemetrySeq = 0;

    for (;;) {
        Msg_Telemetry msg = {};          // pola zarezerwowane zostają zerami
        msg.msgType = MSG_TELEMETRY;

        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            msg.timestamp = millis();
            msg.seq       = ++telemetrySeq;

            // Echo osi — odsyłamy to, co realnie przeczytaliśmy z ramki,
            // a nie to, co z tego wyszło po przeliczeniu na RPM.
            msg.echoAxisLX = padCtrl.axisLX;
            msg.echoAxisLY = padCtrl.axisLY;
            msg.echoAxisRX = padCtrl.axisRX;
            msg.echoAxisRY = padCtrl.axisRY;
            msg.echoSeq    = padCtrl.seq;

            // Odkręcamy invertDirection: w eter idzie konwencja ROBOTA, czyli
            // dodatnie = do przodu dla każdego koła. Dzięki temu Pad może
            // odwrócić mieszanie mecanum, nie wiedząc nic o okablowaniu.
            for (int i = 0; i < 4; i++) {
                const float sign = wheelInverted[i] ? -1.0f : 1.0f;
                msg.targetRPM[i]   = (int16_t)lroundf(sign * wheels[i]->getTargetRPM()  * 10.0f);
                msg.measuredRPM[i] = (int16_t)lroundf(sign * wheels[i]->getCurrentRPM() * 10.0f);
                msg.pwm[i]         = (int16_t)wheels[i]->getControlOutput();
            }

            if (failsafeActive)      msg.flags |= TFLAG_FAILSAFE;
            if (protoErrorCount > 0) msg.flags |= TFLAG_PROTO_ERROR;

            // Strata ramek z pada liczona w oknie MIĘDZY ramkami telemetrii,
            // nie od startu — inaczej po godzinie jazdy jedna zgubiona ramka
            // rozpuszczałaby się w średniej i wskaźnik przestałby cokolwiek mówić.
            uint32_t total = padRecvCount + padMissCount;
            msg.padLossPermille = (total > 0)
                ? (uint8_t)((padMissCount * 1000u) / total > 255u ? 255u
                                                                  : (padMissCount * 1000u) / total)
                : 0;
            padRecvCount = 0;
            padMissCount = 0;

            msg.motorCtrlTimeUs = (uint16_t)constrain((long)motorCtrlAvgTime, 0L, 65535L);

            xSemaphoreGive(movementMutex);
        }

        // Telemetria idzie do Pada — to on przejął rolę monitora.
        esp_err_t res = esp_now_send(macPadXiao,
                                     reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
        if (res == ESP_OK) totalMessages++;

        // Czekamy na kolejną ramkę z Pada. Timeout jest siatką bezpieczeństwa:
        // gdy Pad milczy, telemetria idzie rzadko, ale nie zamiera.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TELEMETRY_IDLE_MS));
    }
}

// Zastosowanie zdalnie przysłanych nastaw PID (MSG_SET_PID).
// Nadawcą będzie Pad; nazwy po porzuconym monitorze zostały usunięte.
void pidCommandTask(void* parameter) {
    Motor* wheels[4] = { &frontLeftMotor, &frontRightMotor,
                         &rearLeftMotor,  &rearRightMotor };

    for (;;) {
        if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
            if (pidCmdPending) {
                pidCmdPending = false;   // jedna komenda = jedno zastosowanie
                for (int i = 0; i < 4; i++) {
                    if (receivedPidCmd.motorIndex == 0xFF ||
                        receivedPidCmd.motorIndex == (uint8_t)i) {
                        wheels[i]->setPID(receivedPidCmd.Kp,
                                          receivedPidCmd.Ki,
                                          receivedPidCmd.Kd);
                    }
                }
                Serial.printf("✅ PID: koło %u  Kp=%.3f Ki=%.3f Kd=%.3f\n",
                              (unsigned)receivedPidCmd.motorIndex,
                              receivedPidCmd.Kp, receivedPidCmd.Ki, receivedPidCmd.Kd);
            }
            xSemaphoreGive(pidMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Muteksy MUSZĄ być gotowe zanim zarejestrujemy callback ESP-NOW —
    // WiFi task (rdzeń 0) może wywołać OnDataRecv natychmiast po rejestracji,
    // podczas gdy setup() jeszcze trwa na rdzeniu 1.
    movementMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    // Dodanie peerów
    memcpy(peerPad.peer_addr, macPadXiao, 6);
    peerPad.channel = ESP_CHANNEL;
    peerPad.encrypt = false;
    esp_now_add_peer(&peerPad);

    // Mutexy
    
    if (!movementMutex) {
        Serial.println("❌ Nie udało się utworzyć mutexu do ruchu");
        while (1) vTaskDelay(100);
    }
    
    
    if (!pidMutex) {
        Serial.println("❌ Nie udało się utworzyć mutexu do komend PID");
        while (1) vTaskDelay(100);
    }
    // Zadania FreeRTOS
    xTaskCreatePinnedToCore(motorControlTask, "MotorCtrlTask", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(telemetryTask,    "TelemetryTask",  4096, nullptr, 1,
                            &telemetryTaskHandle, 1);
    xTaskCreatePinnedToCore(pidCommandTask, "PidCommand", 2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(helloTask,        "HelloTask",      2048, nullptr, 1, nullptr, 0);


    Serial.println("✅ System ready");
}

void loop() {
    // Obsługa w zadaniach
}
