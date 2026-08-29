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

// The pad scales its gauges from MAX_RPM_TELEMETRY in messages.h. This guard
// keeps that constant from drifting away from the platform's real MAX_RPM.
static_assert(MAX_RPM_TELEMETRY == MAX_RPM * 10,
              "MAX_RPM_TELEMETRY in messages.h does not match MAX_RPM!");

// Most recent control frame from the pad
static Msg_PadControl padCtrl;

// Guards padCtrl and the link state
static SemaphoreHandle_t movementMutex;

// ---- Failsafe: watching the link to the pad ----
// Written in OnDataRecv (WiFi task, core 0), read in motorControlTask
// (core 1). Both accesses happen under movementMutex.
static volatile uint32_t lastPadMsgMs   = 0;      // millis() of the last pad frame
static volatile bool     padEverSeen    = false;  // has any frame arrived at all
static volatile bool     failsafeActive = false;  // observed by telemetry

// ---- Protocol state (see the header of messages.h) ----
// padProtoOk survives a LOST hello, but not a MISMATCHED one.
//
// A hello that never arrives is an absence of evidence: a single dropped frame
// must not stop a moving platform, and detecting silence is the failsafe's job
// anyway. A hello that does arrive carrying a different version is evidence to
// the contrary. Treating both the same way — as an earlier version did, by only
// ever setting this flag to true — meant a platform that had agreed once would
// keep driving after the pad was reflashed with an incompatible protocol.
//
// The length check on MSG_PAD_CONTROL catches a change of struct SIZE, but not
// a rearrangement of fields at the same size. That second case is precisely
// what PROTO_VERSION exists to catch, so it has to be able to say "no" again.
static volatile uint8_t  padProtoVersion = 0;
static volatile bool     padProtoOk      = false;
static volatile uint32_t protoErrorCount = 0;   // frames of unknown type/length

// ---- Frames rejected because of the sender (see OnDataRecv) ----
// Without this, a wrong MAC in mac_addresses_private.h and a pad that is
// switched off look EXACTLY the same from here: silence. The counter tells
// the two apart — nothing arriving at all versus something arriving and being
// thrown away — and the recorded address says what to paste into the config.
// Written in the WiFi task, read in helloTask, deliberately without a mutex:
// a torn read costs a garbled diagnostic line, never a wrong driving decision.
static volatile uint32_t foreignFrameCount = 0;
static uint8_t           lastForeignMac[6] = {0};

// Motors
static Motor frontLeftMotor(FL_CONFIG);
static Motor frontRightMotor(FR_CONFIG);
static Motor rearLeftMotor(RL_CONFIG);
static Motor rearRightMotor(RR_CONFIG);

// Mecanum mixer — turns a motion command into four wheel speeds
static MecanumDrive drive(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// Handle of the telemetry task — woken by an incoming pad frame instead of
// running on a timer of its own.
static TaskHandle_t telemetryTaskHandle = nullptr;

// ESP-NOW peer
static esp_now_peer_info_t peerPad;

// ESP-NOW receive callback.
// The message type is taken from the FIRST BYTE; the length is checked before
// memcpy as validation (see the header of messages.h). A frame of unknown type
// or mismatched length is COUNTED AND REPORTED rather than dropped in silence —
// previously "nothing arrived" and "something foreign arrived" looked alike.
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    static uint32_t padFrameCounter = 0;

    // Only the pad may talk to us. ESP-NOW raises the receive callback for ANY
    // sender on the channel — registering a peer governs sending, not
    // receiving — and this device's MAC travels in clear text in the header of
    // every frame it transmits, so it is there for the taking. Without this
    // check a foreign device could drive the platform; and now that a
    // mismatched HELLO clears padProtoOk, it could also stop it at will.
    if (mac == nullptr || memcmp(mac, macPadXiao, 6) != 0) {
        if (mac != nullptr) {
            memcpy(lastForeignMac, mac, 6);
            foreignFrameCount++;   // reported from helloTask, not from here
        }
        return;
    }

    if (len < 1) return;
    const uint8_t type = incomingData[0];

    switch (type) {
    case MSG_PAD_CONTROL:
        if (len != (int)sizeof(Msg_PadControl)) break;
        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&padCtrl, incomingData, sizeof(Msg_PadControl));
            // Liveness mark for the link — the only place it is refreshed
            lastPadMsgMs = millis();
            padEverSeen  = true;
            xSemaphoreGive(movementMutex);
        }
        // Telemetry is a REPLY, not an independent timer. It is not sent from
        // here — one must not transmit from inside the ESP-NOW callback — we
        // only wake the task that will do it in its own context.
        if (++padFrameCounter % TELEMETRY_EVERY_N_PAD_FRAMES == 0 &&
            telemetryTaskHandle != nullptr) {
            xTaskNotifyGive(telemetryTaskHandle);
        }
        return;

    case MSG_HELLO: {
        // Bytes 0 and 1 are frozen across every version of this protocol:
        // message type, then protocol version. The version is read BEFORE the
        // length check ON PURPOSE. A bump that changes the SIZE of Msg_Hello
        // makes the two sides reject each other's HELLO on its length, and the
        // version would then be unreadable in the one case it matters —
        // v3 -> v4 was exactly such a bump, 12 bytes down to 8.
        //
        // padProtoOk is therefore set before the role check as well. That is
        // safe because the MAC filter above has already let through nobody but
        // the pad; do not "tidy" this by moving it back below the role test.
        if (len >= 2) {
            padProtoVersion = incomingData[1];
            padProtoOk      = (incomingData[1] == PROTO_VERSION);
        }
        // A frame of the wrong length still falls through to protoErrorCount.
        // "Unreadable frame" and "version mismatch" are separate states and
        // both have to stay visible.
        if (len != (int)sizeof(Msg_Hello)) break;
        Msg_Hello hello;
        memcpy(&hello, incomingData, sizeof(hello));
        if (hello.role != ROLE_PAD) break;
        return;
    }

    default:
        break;
    }

    // Anything we cannot interpret.
    protoErrorCount++;
}

// Announcing the protocol version. Repeated rather than agreed once at
// startup: ESP-NOW has no notion of a session, so the pad may disappear and
// come back after a reset at any moment, and then has to learn everything
// again from scratch.
void helloTask(void* parameter) {
    for (;;) {
        Msg_Hello hello = {};
        hello.msgType      = MSG_HELLO;
        hello.protoVersion = PROTO_VERSION;
        hello.role         = ROLE_PLATFORM;
        hello.fwBuildId    = FW_BUILD_ID;
        esp_now_send(macPadXiao, reinterpret_cast<const uint8_t*>(&hello), sizeof(hello));

        // Report frames thrown away for their sender. Reported HERE rather
        // than in OnDataRecv because one must not print from inside the
        // ESP-NOW callback. The rhythm fits by itself: while the pad has not
        // been agreed with, this loop runs once a second — which is exactly
        // when a wrong address is the thing worth suspecting.
        static uint32_t foreignReported = 0;
        uint32_t foreignNow = foreignFrameCount;
        if (foreignNow != foreignReported) {
            Serial.printf("Foreign sender: %u frame(s), last "
                          "%02X:%02X:%02X:%02X:%02X:%02X\n",
                          foreignNow - foreignReported,
                          lastForeignMac[0], lastForeignMac[1], lastForeignMac[2],
                          lastForeignMac[3], lastForeignMac[4], lastForeignMac[5]);
            foreignReported = foreignNow;
        }

        // Often while the pad has not been seen. Rarely afterwards: version
        // agreement does not change during operation, so there is no reason to
        // occupy the air with it.
        vTaskDelay(pdMS_TO_TICKS(padProtoOk ? HELLO_INTERVAL_IDLE_MS
                                            : HELLO_INTERVAL_SEARCH_MS));
    }
}

// Motor control task — the PID loop, at a fixed period.
void motorControlTask(void* parameter) {
    int16_t x = 0, y = 0, yaw = 0;
    bool linkAlive = false;
    bool failsafeEngaged = false;  // latch: braking is commanded once per episode

    // vTaskDelayUntil, NOT vTaskDelay — see the comment at the end of the loop.
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        // Read the pad data and judge whether the link is alive
        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            x   = padCtrl.axisLX;
            y   = padCtrl.axisLY;
            yaw = padCtrl.axisRX;
            // Unsigned subtraction survives the millis() rollover.
            // padProtoOk in this condition means: WE DO NOT DRIVE until we know
            // the pad speaks the same protocol version. Driving on data read
            // through someone else's struct layout would be worse than
            // standing still.
            linkAlive = padEverSeen && padProtoOk &&
                        (millis() - lastPadMsgMs) < PAD_LINK_TIMEOUT_MS;
            xSemaphoreGive(movementMutex);
        }

        if (linkAlive) {
            if (failsafeEngaged) {
                failsafeEngaged = false;
                failsafeActive  = false;
                Serial.println("Link to the pad restored");
            }
            // Mecanum kinematics — stick units converted to RPM first.
            // Rotation is computed last, because it depends on travel speed.
            const float vxRpm = padAxisToRPM(x);
            const float vyRpm = padAxisToRPM(y);
            drive.drive(vxRpm, vyRpm, padAxisToOmega(yaw, vxRpm, vyRpm));
        } else if (!failsafeEngaged) {
            // Cut the drive exactly once, then STOP calling drive(), because
            // setTargetRPM() would clear the HardStopped state and resume
            // driving on stale joystick data.
            //
            // Braking is by coasting. That was justified by gearbox friction
            // stopping the platform on its own — an assumption that turned out
            // to hold only in part, since the 120:1 gearbox is back-drivable.
            // Not verified on a slope; see "Known gaps" in CLAUDE.md.
            failsafeEngaged = true;
            failsafeActive  = true;
            drive.hardStop();
            if (!padProtoOk) {
                Serial.printf("Pad speaks protocol v%u, we speak v%u - refusing to drive\n",
                              (unsigned)padProtoVersion, (unsigned)PROTO_VERSION);
            } else {
                Serial.println("Link to the pad lost - drive cut");
            }
        }

        // The PID step runs ALWAYS, failsafe included: it sustains the cutoff
        // and keeps the measured speed fresh for telemetry.
        drive.update();

        // The period MUST be constant, hence vTaskDelayUntil rather than
        // vTaskDelay. vTaskDelay measures the pause FROM THE END OF THE WORK,
        // so the period was the requested 20 ms plus the execution time —
        // 21 ms in practice. The controller was therefore given a different dt
        // from the one the gains assume, and the measured RPM landed on a
        // quantisation grid offset from the correct one (which is where the
        // "always a 9" at the end of the pad's RPM readout came from).
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(INTERVAL_MOTOR_CONTROL));
    }
}

// Telemetry task — gathers the drive state and sends it over ESP-NOW.
// Woken by an incoming pad frame; the timeout is only a safety net.
void telemetryTask(void* parameter) {
    Motor* wheels[4] = { &frontLeftMotor, &frontRightMotor,
                         &rearLeftMotor,  &rearRightMotor };
    const bool wheelInverted[4] = { FL_CONFIG.invertDirection, FR_CONFIG.invertDirection,
                                    RL_CONFIG.invertDirection, RR_CONFIG.invertDirection };
    uint32_t telemetrySeq = 0;

    for (;;) {
        Msg_Telemetry msg = {};
        msg.msgType = MSG_TELEMETRY;

        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            msg.seq = ++telemetrySeq;

            // Axis echo — we send back what we actually read out of the frame,
            // not what it turned into after conversion to RPM.
            msg.echoAxisLX = padCtrl.axisLX;
            msg.echoAxisLY = padCtrl.axisLY;
            msg.echoAxisRX = padCtrl.axisRX;
            msg.echoAxisRY = padCtrl.axisRY;
            msg.echoSeq    = padCtrl.seq;

            // Undo invertDirection: what goes on air is the ROBOT convention,
            // i.e. positive = forward for every wheel. That lets the pad invert
            // the mecanum mixing without knowing anything about our wiring.
            for (int i = 0; i < 4; i++) {
                const float sign = wheelInverted[i] ? -1.0f : 1.0f;
                msg.targetRPM[i]   = (int16_t)lroundf(sign * wheels[i]->getTargetRPM()  * 10.0f);
                msg.measuredRPM[i] = (int16_t)lroundf(sign * wheels[i]->getCurrentRPM() * 10.0f);
                msg.pwm[i]         = (int16_t)wheels[i]->getControlOutput();
            }

            if (failsafeActive)      msg.flags |= TFLAG_FAILSAFE;
            if (protoErrorCount > 0) msg.flags |= TFLAG_PROTO_ERROR;

            xSemaphoreGive(movementMutex);
        }

        // Telemetry goes to the pad — it took over the monitor's role.
        esp_now_send(macPadXiao, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));

        // Wait for the next pad frame. The timeout is a safety net: when the
        // pad goes quiet, telemetry slows down but does not die.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TELEMETRY_IDLE_MS));
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Both addresses, printed before anything can go wrong with them. The MAC
    // is not visible on the board and mac_addresses_private.h is not in the
    // repo, so a mistyped address used to show up only as a platform that
    // never moves. OWN is what to paste into the pad's config; PAD is what
    // this build believes the pad to be — all zeros means the private header
    // is missing and the template was used.
    Serial.printf("MAC own %s\n", WiFi.macAddress().c_str());
    Serial.printf("MAC pad %02X:%02X:%02X:%02X:%02X:%02X\n",
                  macPadXiao[0], macPadXiao[1], macPadXiao[2],
                  macPadXiao[3], macPadXiao[4], macPadXiao[5]);

    // The mutexes MUST exist before the ESP-NOW callback is registered: the
    // WiFi task (core 0) can call OnDataRecv the instant it is registered,
    // while setup() is still running on core 1.
    movementMutex = xSemaphoreCreateMutex();
    if (!movementMutex) {
        Serial.println("Failed to create a mutex");
        while (1) vTaskDelay(100);
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    // The pad is the only peer.
    memcpy(peerPad.peer_addr, macPadXiao, 6);
    peerPad.channel = ESP_CHANNEL;
    peerPad.encrypt = false;
    esp_now_add_peer(&peerPad);

    // FreeRTOS tasks. Motor control and telemetry are pinned to core 1;
    // HELLO sits on core 0 alongside the WiFi stack it feeds.
    xTaskCreatePinnedToCore(motorControlTask, "MotorCtrlTask", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(telemetryTask,    "TelemetryTask", 4096, nullptr, 1,
                            &telemetryTaskHandle, 1);
    // HelloTask got 3072 rather than 2048 when the foreign-sender report was
    // added to it: Serial.printf pulls in vsnprintf, and a stack overflow here
    // would surface as a random reset with nothing pointing back to this line.
    xTaskCreatePinnedToCore(helloTask,        "HelloTask",     3072, nullptr, 1, nullptr, 0);

    Serial.println("System ready");
}

void loop() {
    // Everything runs in the FreeRTOS tasks.
}
