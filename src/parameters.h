#pragma once

#include <Arduino.h>

// ===== Motor PWM pins =====
// Motor drivers are fed a 20 kHz PWM signal at 9-bit resolution.

// Front Left
constexpr int FL_PIN1 = 9;        // M1A
constexpr int FL_PIN2 = 10;       // M1B
constexpr int FL_CHANNEL1 = 0;    // LEDC channel for M1A
constexpr int FL_CHANNEL2 = 1;    // LEDC channel for M1B

// Front Right
constexpr int FR_PIN1 = 11;       // M2A
constexpr int FR_PIN2 = 12;       // M2B
constexpr int FR_CHANNEL1 = 2;
constexpr int FR_CHANNEL2 = 3;

// Rear Left
constexpr int RL_PIN1 = 13;       // M3A
constexpr int RL_PIN2 = 14;       // M3B
constexpr int RL_CHANNEL1 = 4;
constexpr int RL_CHANNEL2 = 5;

// Rear Right
constexpr int RR_PIN1 = 15;       // M4A
constexpr int RR_PIN2 = 16;       // M4B
constexpr int RR_CHANNEL1 = 6;
constexpr int RR_CHANNEL2 = 7;

// ===== Encoder pins =====

constexpr int FL_ENCODER_A = 1;
constexpr int FL_ENCODER_B = 2;

constexpr int FR_ENCODER_A = 4;
constexpr int FR_ENCODER_B = 5;

constexpr int RL_ENCODER_A = 6;
constexpr int RL_ENCODER_B = 7;

constexpr int RR_ENCODER_A = 17;
constexpr int RR_ENCODER_B = 18;

// ===== Speed limits =====

// Top wheel speed. This value is MEASURED (3 m in 10.3 s, and 5 platform
// rotations in 13 s, 2026-08-28), not assumed. The motor is rated 160 RPM
// no-load at 6 V; under load it settles around 95, so 90 leaves the controller
// some headroom to actually reach the setpoint.
constexpr int MAX_RPM = 90;

// ===== Pad input =====
// The pad sends +/-511 (getCorrectedValue reduces to raw - offset on a 10-bit
// ADC). Without rescaling to MAX_RPM, full speed was reached at roughly 35 %
// of stick travel and the rest of the throw did nothing.
constexpr int JOYSTICK_MAX = 511;

// Dead zone around the resting position. Besides killing ADC-noise creep, it
// makes the commanded speed EXACTLY zero when the sticks are released — and
// the standstill cutoff of PWM output and integral term depends on that.
constexpr int JOYSTICK_DEADZONE = 12;

// ===== Turn shaping =====
// The right stick controls how TIGHT the arc is, not the rotation rate.
//
// It used to command rotation directly, with the same authority as the drive
// stick. At full throttle and full rotation that produced left 180 / right 0 —
// the inner wheels stood still and the platform dragged itself round like a
// tank with one track, instead of carving an arc. The inner wheels could not
// start counter-rotating either, because normalisation scales all four wheels
// by the same factor and zero stays zero. The entire range of gentle arcs was
// squeezed into the first third of stick travel.
//
// OFF-ROAD variant, chosen deliberately: a sharp turn at full speed stays
// available. What changes is the distribution of sensitivity, not the maximum.

// How hard the INNER wheels should work at full turn stick and full throttle,
// as a fraction of the outer wheels' speed: 0 = stopped, negative = counter-
// rotating.
//
// This is the right knob for tuning turn feel, because it states directly what
// you see on the floor. At -0.23 (what the first attempt produced) the inner
// wheels mostly braked; at -0.55 both sides do work.
//
// Note: this is the COMMANDED value. How much of it actually happens at the
// moment of entering a turn is capped by the counter-torque limit
// (MAX_HOLDING_PWM) — a wheel still rolling forward will not be given full
// reverse, because that is precisely the case that trips the cell protection.
constexpr float TURN_INNER_RATIO_FULL = -0.55f;

// Gain derived from the ratio above. At full throttle and full stick the wheels
// get vy*(1+G) and vy*(1-G); normalisation divides by (1+G), so the inner/outer
// ratio comes out as (1-G)/(1+G) = r. Hence:
constexpr float TURN_GAIN = (1.0f - TURN_INNER_RATIO_FULL) /
                            (1.0f + TURN_INNER_RATIO_FULL);

// Turn-stick curve: 0 = linear, 1 = pure cubic. Concentrates resolution around
// centre WITHOUT touching the end of the range — full deflection gives the same
// result regardless of this value.
//
// Set to pure cubic because at 0.75 one fifth of stick travel already produced
// a pronounced turn (inner wheels dropping from 90 to 64 RPM) and steering felt
// like a button rather than a stick. At 1.0 that same fifth costs only a drop
// to 86 RPM — a gentle course correction — while the sharp turn still waits at
// the end of the throw.
//
// If centre ever feels too dead, 0.9 is the intermediate setting.
constexpr float TURN_EXPO = 1.0f;

// Below this travel speed the stick reverts to spinning in place. Without it a
// stationary platform could not rotate at all, because an arc at zero speed
// does not exist. The transition is blended, not switched.
constexpr float PIVOT_BLEND_RPM = 20.0f;

// ===== Safety =====
constexpr uint32_t DEFAULT_SOFT_STOP_DURATION_MS = 500;
constexpr uint32_t DEFAULT_HARD_STOP_DURATION_MS = 50;

// ----- Acceleration limit (protects the cell current cutoff) -----
// How fast, in RPM per second, a wheel's commanded speed may RISE. Without
// this the controller puts out full duty on the first cycle after a step,
// four motors start like a short circuit and the cells' current protection
// cuts power.
// Ramping the command DOWN TO ZERO is not limited — emergency braking has to
// stay as fast as it was.
// Lower value = gentler start and less current, but slower response.
// Rescaled together with MAX_RPM (was 300 when MAX_RPM was 180) so that the
// physical acceleration stayed exactly the same.
constexpr float MAX_ACCEL_RPM_PER_S = 150.0f;

// ----- Counter-torque limit (plugging / reverse braking) -----
// When the controller wants to apply voltage OPPOSITE to the wheel's current
// direction (stick released, direction reversed), current is not limited by
// winding resistance the way it is at startup: the motor's back-EMF ADDS to
// the battery voltage. At full reverse from top speed the current is HIGHER
// than a short circuit and the cell protection trips.
//
// The limit is SPEED-DEPENDENT, because the current cost of counter-torque is
// speed-dependent too: I = (V_batt + EMF) / R, and EMF grows with RPM.
// At a standing wheel EMF is zero, so opposing rotation is cheap — and that is
// exactly what lets the platform hold position when someone pushes a wheel by
// hand. At full speed the limit falls to zero, so braking is by coasting, with
// no risk to the cell protection.
//
// This is the value at ZERO speed; it falls linearly to zero at MAX_RPM.
// Lower = weaker position holding, but lower current.
constexpr int MAX_HOLDING_PWM = 250;

// Below this speed a wheel counts as standing still. The threshold must sit
// clearly above measurement noise (at 1920 counts per revolution and a 20 ms
// cycle, one count is about 1.6 RPM) yet low enough that turning a wheel by
// hand always exceeds it — otherwise the motor would not notice being moved.
// Rescaled together with MAX_RPM (was 5.0) so the physical threshold is
// unchanged.
constexpr float STANDSTILL_RPM = 2.5f;

// ----- Failsafe: link to the pad lost -----
// The pad transmits every 20 ms. Silence longer than PAD_LINK_TIMEOUT_MS means
// a broken link (15 lost frames) and triggers the drive cutoff. The value
// stayed in milliseconds when the transmit rate went up: what matters for
// safety is how long the robot drives uncontrolled, not a frame count.
// Too small = false alarms on brief interference, too large = the robot keeps
// driving blind for longer. Tuned on hardware.
constexpr uint32_t PAD_LINK_TIMEOUT_MS = 300;



// ===== Task scheduling (milliseconds) =====

constexpr int INTERVAL_MOTOR_CONTROL = 20;   // PID loop period

// Telemetry does NOT run on its own timer — it is a reply to a pad frame.
// A timer of its own meant a third unsynchronised loop at a random phase
// relative to the pad's transmit loop, which made the round trip jump by a
// whole period. Replying immediately gives a shorter and STABLE response time
// with fewer frames on air — so less current and less heat. The eventual link
// (ELRS Air Port) forces the same discipline: no frame sent just because a
// timer expired.
constexpr uint32_t TELEMETRY_EVERY_N_PAD_FRAMES = 2;  // pad 50 Hz -> telemetry 25 Hz

// Fallback rate when nothing arrives from the pad, so telemetry does not go
// completely silent and the platform's state stays visible after a link loss.
constexpr uint32_t TELEMETRY_IDLE_MS = 200;

// Protocol version announcements (MSG_HELLO). Frequent while the partner has
// not been seen, so that driving resumes quickly after its reset; rare
// afterwards, because version agreement does not change during operation.
constexpr int HELLO_INTERVAL_SEARCH_MS = 1000;
constexpr int HELLO_INTERVAL_IDLE_MS   = 5000;

// ===== PID output range =====
// Matches the 9-bit PWM resolution: +/-511 is full duty in either direction.
constexpr float MAX_OUT =  511.0f;
constexpr float MIN_OUT = -511.0f;

// ===== ESP-NOW =====
constexpr int ESP_CHANNEL = 0;

// ===== Default motor configuration =====
// Encoder COUNTS per WHEEL revolution — not pulses!
// The manufacturer quotes 8 pulses per motor shaft revolution, i.e. 960 per
// wheel revolution through the 120:1 gearbox. But ESP32Encoder in
// attachHalfQuad mode counts BOTH EDGES of channel A (pos_mode=DEC,
// neg_mode=INC), so one pulse yields TWO counts: 960 * 2 = 1920.
//
// The 960 that used to sit here made the platform measure speed TWICE TOO
// HIGH. The controller reached a setpoint that was in reality half of what was
// asked for, and had no way of knowing — found only with a stopwatch: the
// screen said 0.55 m/s while 3 m took 10.3 s, i.e. 0.29 m/s.
constexpr int DEFAULT_GEAR_RATIO = 1920;
constexpr int DEFAULT_PWM_RESOLUTION = 9;     // 9-bit PWM
constexpr int DEFAULT_PWM_FREQUENCY = 20000;  // 20 kHz
