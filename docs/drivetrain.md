# Drivetrain — the numbers behind the conversions

The values needed to turn revolutions into speed, so that the next change does
not have to guess. Extended as measurements are taken.

## Wheels and encoders

| quantity | value | source |
|---|---|---|
| mecanum wheel diameter | **60 mm** | measured, 2026-08-27 |
| circumference | 188.5 mm | π × 60 mm |
| gearbox | 120:1 | back-drivable (a wheel turns by hand) |
| encoder, per motor shaft revolution | 8 pulses | motor datasheet |
| encoder, per wheel revolution | 960 pulses = **1920 counts** | 8 × 120, doubled by the library |
| `DEFAULT_GEAR_RATIO` | **1920** | `parameters.h` |

The pulse/count distinction is the one that bit hardest. `ESP32Encoder::attachHalfQuad`
sets `pos_mode = DEC` and `neg_mode = INC`, so it counts **both edges** of
channel A — two counts per pulse. A datasheet pulse count and a library count
are two different things, and confusing them is invisible to everything except
a measurement in the physical world.

## Chassis geometry

Measured **centre of wheel to centre of wheel**, 2026-08-28.

| quantity | value |
|---|---|
| wheelbase (front–rear) | **12 cm** |
| track (left–right) | **15 cm** |
| wheel width | almost 3 cm |

The left–right figure is easy to get wrong: the gap between the wheels is 12 cm,
but the wheels are wide, so centre to centre comes out at 15 cm. **Kinematics
needs the distance between centres**, because that is where the force acts.

## Slopes and what stops the platform

Measured on 2026-08-30, driving: up, down, braking, and with the link cut
mid-drive (the pad switched off, which puts every motor into the coasting
`hardStop()` state).

The slope was a **200 cm board with one end raised**, on white melamine-faced
furniture board — a deliberately slippery surface, not a floor covering.

| rise over 200 cm | grade | angle | behaviour |
|---|---|---|---|
| **40 cm** | ~20 % | ~11.5° | drives well, and stops and stays put with the link cut |
| **50 cm** | ~26 % | ~14.5° | the **tyres** let go — the wheels slip |

Two things follow, and the second one is the useful one.

The failsafe's coasting stop is **enough** at this grade. `hardStop()` zeroes
both PWM channels and leaves the motors floating, on the assumption that the
120:1 gearbox holds the platform by friction alone. The gearbox is back-drivable
— a wheel turns by hand with the power off — so the assumption was open to
doubt, and this is the measurement that settled it. `softStop()` stays written,
tested and unused; swapping it in would replace a verified behaviour with an
unverified one.

More useful: what runs out first is **grip, not torque**. The wheels slip before
the controller gives up holding position, so `MAX_HOLDING_PWM = 250` has margin
and there is nothing to gain by raising it.

Read the second row carefully: it measures **the surface, not the robot**.
Melamine board is about as slippery as anything found indoors, so that figure is
close to a worst case — on carpet or concrete the wheels would hold considerably
further. It records where traction ran out on that board, and says nothing about
the machine's capability.

The first row is the one worth trusting, and even it is tied to today's mass.
A heavier version of the machine invalidates it and the test has to be run
again. It takes two minutes: a board, something to prop it on, and a tape
measure — rise over length is the grade.

## Revolutions to speed

    v [m/s]  = RPM × π × D / 60 = RPM × 0.0031416   (for D = 60 mm)
    v [km/h] = v [m/s] × 3.6

| wheel RPM | m/s | km/h |
|---|---|---|
| 30 | 0.094 | 0.34 |
| 60 | 0.188 | 0.68 |
| 90 (`MAX_RPM`) | 0.283 | 1.02 |

`MAX_RPM = 90` is **measured**, not assumed. Two independent runs on 2026-08-28
agreed: 3 m from a standing start in 10.3 s (0.29 m/s), and 5 platform rotations
in 13 s in both directions. Repeated on 2026-08-29 after a fresh flash: 3 m in
10.5 s (0.286 m/s, i.e. 90.9 RPM at the wheel) and 5 rotations in 13.1 s,
repeatable in both directions.

Those runs are also what exposed the encoder scaling error. The display was
claiming 0.55 m/s while the stopwatch said 0.29 — the platform was measuring
exactly twice the truth, so the PID reached a setpoint that was in reality half
of what was asked for, and had no way of noticing.

## Platform motion from wheel revolutions

Inverting the mecanum mixing (`MecanumDrive::drive`) recovers the platform's
velocity from the four wheels:

    vy = (FL + FR + RL + RR) / 4      forward
    vx = (FL − FR − RL + RR) / 4      sideways
    ω  = (FL − FR + RL − RR) / 4      rotation

The result is in RPM and converts to m/s with the formula above.

### Rotation in degrees per second

The rotation term from `mecanumInverse()` is in wheel-revolution units. The
linear speed a wheel must have for the platform to rotate at angular rate ω is
`ω × (lx + ly)`, where `lx` and `ly` are **half** the spacings:

    lx + ly = (0.12 + 0.15) / 2 = 0.135 m

Hence `RPM01_TO_DEG_S` in `messages.h`. At full turn deflection this works out
to **120 °/s**, i.e. one third of a rotation per second.

**The measurement disagrees, and the reason is not yet known.** Five platform
rotations in 13 s (2026-08-28) is **138.5 °/s**, and 13.1 s on 2026-08-29 is
**137.4 °/s** — both directions, repeatable, on either side of a reflash. That
is about 15 % more than the geometry predicts.

Some explanations have been ruled out. The spacings above were measured
centre-to-centre and re-confirmed, and a pure rotation never triggers the
mixer's normalisation, so no wheel is being scaled.

**Roller behaviour has NOT been ruled out — only half of it has.** The two
cases point in opposite directions and must not be lumped together:

- **Rollers slipping along their own axis** — the wheel travels less than its
  revolutions claim. This makes the measurement come out *lower* than predicted,
  so it cannot explain a figure that is too *high*. This is the case the
  paragraph above rules out, and the only one.
- **Rollers not turning freely** — a stiff or loaded roller scrubs instead of
  rolling, and the wheel starts behaving like a plain wheel. A pivot then shifts
  away from mecanum kinematics and towards skid-steer, where the effective lever
  arm is `ly` alone instead of `lx + ly`. A shorter lever arm at the same wheel
  speed means a *higher* angular rate — which is the direction actually
  observed.

The two models bracket the measurement. Both predictions are computed from the
same straight-line run (3 m in 10.5 s = 0.286 m/s, i.e. 90.9 RPM at the wheel),
so they are directly comparable with the pivot measured on the same day. That is
also why the mecanum figure reads 121.3 rather than the 120.0 quoted above: 120
is what the constants give at exactly `MAX_RPM = 90`, 121.3 is the same model
evaluated at the 90.9 RPM actually measured. Same formula, different input.

| model | lever arm | predicted rotation |
|---|---|---|
| pure mecanum, rollers free | `lx + ly` = 0.135 m | **121.3 °/s** |
| pure skid-steer, rollers locked | `ly` = 0.075 m | **218.3 °/s** |
| **measured** | — | **137.4 °/s** |

The measurement sits about **17 % of the way** from one model to the other —
which is what you would expect from rollers that mostly roll but scrub a little.
That is a hypothesis, not a conclusion: the bracket shows the idea is
dimensionally plausible, nothing more.

**The wheels spinning faster is not the explanation**, and the two measurements
together are what show it. Turning at 137.4 °/s *while the mecanum model holds*
would require a wheel speed of 0.324 m/s, i.e. **103 RPM** — above the ~91 RPM
the straight-line run measures as the ceiling, on the same battery and the same
floor. A pivot loads the drive more than driving straight, not less, so the
wheels cannot be turning faster there.

Note what this does and does not establish. It rules out *overspeeding motors*.
It does **not** rule out the roller hypothesis above — that one keeps the wheels
at 90 RPM and changes the lever arm instead, which is exactly why it survives
this argument.

### Two cheap tests, before reaching for an IMU

Neither needs any hardware that is not already on the robot, and each one
eliminates a whole class of cause:

1. **The same pivot on a different surface** — tiles against carpet. If the time
   for five rotations changes, the rollers are involved and the geometry is not.
   If it does not change, the rollers are exonerated.
2. **Read `measuredRPM` on the wheel screen DURING a pivot.** If it shows about
   90, the wheels are doing exactly what they are told and the suspect is the
   model that converts their revolutions into platform rotation. If it shows
   more, the wheels are overspeeding and the mixing is not at fault.

An IMU is the *third* option, not the first: its gyroscope measures the
platform's angular rate directly instead of inferring it from the wheels, which
settles the question outright — but it is also the only one of the three that
requires buying and mounting a part.

Until one of them is done, treat 120 °/s as what the constants say and 137 °/s
as what the floor says.

An earlier version of this file claimed 240 °/s. That figure was correct back
when `MAX_RPM` was 180 and it was carried over unchanged when the encoder fix
halved the speed scale — the same class of stale-number error this document
exists to prevent.

**Caveat:** the **sideways** speed comes out optimistic. Mecanum rollers slip
sideways by the very principle they work on, so real `vx` is smaller than the
computed one. Driving straight, the discrepancy is small; driving sideways, it
is noticeable. That is not measurement error — it is how the drive works.
