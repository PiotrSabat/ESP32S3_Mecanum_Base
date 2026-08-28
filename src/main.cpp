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

// Most recent remote PID command
static Msg_SetPID receivedPidCmd;
static volatile bool pidCmdPending = false;

// Guards padCtrl and the link/telemetry counters
static SemaphoreHandle_t movementMutex;

// Guards the incoming PID settings
static SemaphoreHandle_t pidMutex;

// ---- Failsafe: watching the link to the pad ----
// Written in OnDataRecv (WiFi task, core 0), read in motorControlTask
// (core 1). Both accesses happen under movementMutex.
static volatile uint32_t lastPadMsgMs   = 0;      // millis() of the last pad frame
static volatile bool     padEverSeen    = false;  // has any frame arrived at all
static volatile bool     failsafeActive = false;  // observed by telemetry

// ---- Protocol state (see the header of messages.h) ----
// padProtoOk is a LATCH: once a matching version has been seen it stays. A
// single lost HELLO must not stop a moving platform — detecting silence is the
// failsafe's job, and it measures the time since the last control frame.
static volatile uint8_t  padProtoVersion = 0;
static volatile bool     padProtoOk      = false;
static volatile uint32_t protoErrorCount = 0;   // frames of unknown type/length

// ---- Pad frame loss (gaps in the seq numbering) ----
// padSynced and padRecvCount are deliberately SEPARATE. padSynced says "we know
// what the previous seq was"; padRecvCount is the size of the current
// statistics window and gets cleared on every telemetry frame. Using one
// variable for both meant that clearing the window also cleared the
// synchronisation, so the first frame after each telemetry send was never
// checked for a gap — at TELEMETRY_EVERY_N_PAD_FRAMES = 2 that is every second
// transition, and reported loss came out roughly half of the truth.
static volatile uint32_t padSeqLast   = 0;
static volatile bool     padSynced    = false;
static volatile uint32_t padRecvCount = 0;
static volatile uint32_t padMissCount = 0;

// Execution time of motorControlTask, in microseconds, reported in telemetry.
static volatile float motorCtrlAvgTime = 0.0f;

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
            // A gap in the numbering means frames lost on the way. The very
            // first frame after startup only synchronises the counter. A pad
            // restart rewinds seq, so the ">" guard keeps a negative loss out.
            if (padSynced && padCtrl.seq > padSeqLast + 1) {
                padMissCount += padCtrl.seq - padSeqLast - 1;
            }
            padSeqLast = padCtrl.seq;
            padSynced  = true;
            padRecvCount++;
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
        if (len != (int)sizeof(Msg_Hello)) break;
        Msg_Hello hello;
        memcpy(&hello, incomingData, sizeof(hello));
        if (hello.role != ROLE_PAD) break;
        padProtoVersion = hello.protoVersion;
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
        hello.uptimeMs     = millis();
        esp_now_send(macPadXiao, reinterpret_cast<const uint8_t*>(&hello), sizeof(hello));

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
        uint64_t start = esp_timer_get_time();

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

        // Exponential moving average, not a mean since boot: a mean since boot
        // stops reacting after a minute and quietly becomes a constant, which
        // is worse than no number at all.
        const uint64_t duration = esp_timer_get_time() - start;
        motorCtrlAvgTime += ((float)duration - motorCtrlAvgTime) / 64.0f;

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
        Msg_Telemetry msg = {};          // reserved fields stay zero
        msg.msgType = MSG_TELEMETRY;

        if (xSemaphoreTake(movementMutex, portMAX_DELAY) == pdTRUE) {
            msg.timestamp = millis();
            msg.seq       = ++telemetrySeq;

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

            // Pad frame loss is counted over the window BETWEEN telemetry
            // frames, not since boot — otherwise, after an hour of driving, a
            // single lost frame would dissolve into the average and the
            // indicator would stop saying anything.
            uint32_t total = padRecvCount + padMissCount;
            msg.padLossPermille = (total > 0)
                ? (uint8_t)((padMissCount * 1000u) / total > 255u ? 255u
                                                                  : (padMissCount * 1000u) / total)
                : 0;
            padRecvCount = 0;      // window only — padSynced/padSeqLast survive
            padMissCount = 0;

            msg.motorCtrlTimeUs = (uint16_t)constrain((long)motorCtrlAvgTime, 0L, 65535L);

            xSemaphoreGive(movementMutex);
        }

        // Telemetry goes to the pad — it took over the monitor's role.
        esp_now_send(macPadXiao, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));

        // Wait for the next pad frame. The timeout is a safety net: when the
        // pad goes quiet, telemetry slows down but does not die.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TELEMETRY_IDLE_MS));
    }
}

// Applies PID settings received remotely (MSG_SET_PID). The sender will be
// the pad.
void pidCommandTask(void* parameter) {
    Motor* wheels[4] = { &frontLeftMotor, &frontRightMotor,
                         &rearLeftMotor,  &rearRightMotor };

    for (;;) {
        if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
            if (pidCmdPending) {
                pidCmdPending = false;   // one command = one application
                for (int i = 0; i < 4; i++) {
                    if (receivedPidCmd.motorIndex == 0xFF ||
                        receivedPidCmd.motorIndex == (uint8_t)i) {
                        wheels[i]->setPID(receivedPidCmd.Kp,
                                          receivedPidCmd.Ki,
                                          receivedPidCmd.Kd);
                    }
                }
                Serial.printf("PID: wheel %u  Kp=%.3f Ki=%.3f Kd=%.3f\n",
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

    // The mutexes MUST exist before the ESP-NOW callback is registered: the
    // WiFi task (core 0) can call OnDataRecv the instant it is registered,
    // while setup() is still running on core 1.
    movementMutex = xSemaphoreCreateMutex();
    pidMutex      = xSemaphoreCreateMutex();
    if (!movementMutex || !pidMutex) {
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

    // FreeRTOS tasks. Motor control, telemetry and PID commands are pinned to
    // core 1; HELLO sits on core 0 alongside the WiFi stack it feeds.
    xTaskCreatePinnedToCore(motorControlTask, "MotorCtrlTask", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(telemetryTask,    "TelemetryTask", 4096, nullptr, 1,
                            &telemetryTaskHandle, 1);
    xTaskCreatePinnedToCore(pidCommandTask,   "PidCommand",    2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(helloTask,        "HelloTask",     2048, nullptr, 1, nullptr, 0);

    Serial.println("System ready");
}

void loop() {
    // Everything runs in the FreeRTOS tasks.
}
