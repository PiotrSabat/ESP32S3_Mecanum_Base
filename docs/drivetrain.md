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
in 13 s in both directions.

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
rotations in 13 s (2026-08-28, both directions, repeatable) is **138.5 °/s** —
about 15 % more than the geometry predicts.

The obvious explanations have been ruled out. The spacings above were measured
centre-to-centre and re-confirmed. Roller slip would push the measurement the
other way, making it *lower* than predicted, not higher. And a pure rotation
never triggers the mixer's normalisation, so no wheel is being scaled.

It is left open rather than explained away; it is a candidate for the first
real job for the IMU, whose gyroscope measures the platform's angular rate
directly instead of inferring it from the wheels. Until then, treat 120 °/s as
what the constants say and 138.5 °/s as what the floor says.

An earlier version of this file claimed 240 °/s. That figure was correct back
when `MAX_RPM` was 180 and it was carried over unchanged when the encoder fix
halved the speed scale — the same class of stale-number error this document
exists to prevent.

**Caveat:** the **sideways** speed comes out optimistic. Mecanum rollers slip
sideways by the very principle they work on, so real `vx` is smaller than the
computed one. Driving straight, the discrepancy is small; driving sideways, it
is noticeable. That is not measurement error — it is how the drive works.
