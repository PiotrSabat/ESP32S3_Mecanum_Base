# Simulators — testing the control code without hardware

These programs run the **real code from `src/`** (`Motor.cpp`, `MecanumDrive.cpp`)
on a desktop machine, substituting the clock, the encoder and the PWM outputs.
That makes it possible to inspect controller behaviour you cannot see on a
moving platform: internal state, waveforms over time, and the current the motors
draw.

This does not replace a run on the floor. A clean compile and green simulations
only prove that the logic does what was intended; whether the platform actually
goes where the stick points is settled by hardware alone.

## Running them

```bash
./sim/run.sh              # everything
./sim/run.sh failsafe     # one of them
```

Only `g++` is required. Output lands in `sim/build/` (a git-ignored directory).

## What each one covers

| File | Scope |
|---|---|
| `failsafe.cpp` | detecting the loss of the pad link, cutting the drive, resuming control, `millis()` rollover after ~49.7 days |
| `kinematics.cpp` | stick-to-RPM conversion, dead zone, four-wheel normalisation (does the direction of travel survive saturation), arc steering |
| `braking.cpp` | current during startup and braking, reverse kick after stopping |
| `holding.cpp` | resistance to turning a wheel by hand, and braking current from full speed |

## The motor model — read this before believing the numbers

The drive parameters are **estimated, not measured**: winding resistance derived
from an assumed 1.5 A stall current at 6 V, a 150 ms time constant, a 25-unit
PWM dead band. Current is computed in two variants (pessimistic, and one that
accounts for coasting during the off phase), because reality depends on the
MX1508 bridge's decay mode.

Since 2026-08-28 the model includes a **load term** (`I_LOAD`), and with it the
model hits the measured top speed (~90 RPM) and the right order of magnitude for
startup current (6 A against 5.61 A measured passing and 7.40 A measured
tripping the cell protection). Before that term existed, the model understated
current almost fourfold.

Treat absolute values as an order of magnitude and **comparisons between
variants** as trustworthy. That is exactly how the decisions here were made:
what mattered was that one option drew three times less current than another,
not how many amperes precisely.

The episode of 2026-08-23 shows the limits of the method: the simulator picked
PID gains that were correct as far as speed went, but it did not model current
at the time — and the flashed firmware tripped the cell protection. A model only
answers the question you ask it.
