# Current sensing — what fits and what does not

Compiled 2026-08-27 from manufacturer documentation. The starting question: can
an INA226 measure motor current over −10…+10 A, given that an H-bridge reverses
the polarity?

## Short answer

The INA226 **is bidirectional**, so the sign is not the obstacle. The obstacle
is the **common-mode range**: 0…36 V, with no excursion below ground. In the
wire feeding a motor driven by an H-bridge, the potential swings between ground
and the cell voltage at the PWM frequency, and during current decay it goes
*below* ground by a diode drop. **The INA226 does not belong in a motor lead** —
it belongs on a supply rail.

## INA226 (Texas Instruments)

| parameter | value |
|---|---|
| bidirectional | yes |
| shunt voltage range | **±81.92 mV** (full scale) |
| resolution | signed 16-bit → **2.5 µV** per bit |
| common-mode (bus) voltage | **0…36 V**, never below ground |
| interface | I2C — no host ADC needed |

Shunt for ±10 A: `81.92 mV / 10 A ≈ 8.2 mΩ`. At 2.5 µV per bit that is about
**0.3 mA of resolution** — far more than needed.

Conclusion: **one device on the supply rail** (or one per cell) gives current
and voltage over I2C without occupying an ADC on the microcontroller. That is
precisely the measurement that speaks to the real problem here, the cell
protection cutting out.

## INA3221 (Texas Instruments), three channels

| parameter | value |
|---|---|
| channels | 3, independently enabled |
| bidirectional | **yes** |
| shunt range | **±163.8 mV**, step **40 µV** (signed 13-bit) |
| common-mode voltage | **0…26 V** — less than the INA226 |
| interface | I2C |

The resolution is 16× worse than the INA226's, but with an 8.2 mΩ shunt that is
still about **5 mA per step** — irrelevant at currents on the order of amperes.

It carries **exactly the same limitation as the INA226**: the common mode does
not go below ground, so it does not belong in a motor lead either. Three
channels change nothing about that.

## The key observation: the motor lead is not the only option

"Per-wheel" current can be measured **at the driver's supply input**, between
the cell rail and the driver board. There the common mode is a **quiet supply
rail** rather than a switching output, and the INA family works without trouble.
The measurement stays bidirectional, because current flows back into the cells
during braking.

**What this measurement does NOT give you:** current on the supply side is
approximately motor current multiplied by the PWM duty cycle. So it reports
**cell loading and power draw** — exactly what trips the protection — but not
shaft torque.

### A limitation of the present hardware

The platform has **two dual-channel Cytron Maker Drive MX1508 boards** (see the
README), and each has a single supply input shared by both bridges. Measuring at
the driver's supply therefore gives **current per pair of wheels, not per
wheel**. Per-wheel current would need an isolated sensor in the motor lead
(ACS724) or surgery on the driver board.

If the wheels are split left/right between the two boards, then a per-pair
measurement still answers the question that matters: which side is dragging.

### An aside, unrelated to measurement but important

The README lists the MX1508 as **1 A per channel**. The recorded draw figures
are 5.61 A (passed) and 7.40 A (tripped) for the **whole platform**, i.e. about
1.4–1.9 A per motor at startup. **The drivers are running at or above their
rating.** Regardless of any measurement: larger motors will need different
drivers.

## Measuring at the motor — two routes

### ACS724LLCTR-10AB (Allegro), Hall effect

| parameter | value |
|---|---|
| range | **−10…+10 A**, exactly what was asked for |
| principle | Hall effect, **galvanic isolation to 2.4 kV** |
| conductor resistance | ~1.2 mΩ |
| bandwidth | 120 kHz |
| output | analogue, 200 mV/A, centred at 2.5 V on a 5 V supply |

The isolation means **common-mode voltage stops mattering** — it can be inserted
anywhere in the path, motor leads included. That is the answer to the original
question.

**A trap worth checking before buying:** the output is centred on 2.5 V and
swings 0.5…4.5 V on a 5 V supply. The ESP32-S3's ADC works to 3.3 V, so a
divider is required — which eats part of the resolution and adds error of its
own.

### INA240 (Texas Instruments), shunt amplifier

Designed explicitly for in-line motor measurement: common mode **−4…+80 V** and
**PWM common-mode rejection**. Not isolated, analogue output. More accurate than
the Hall sensor, but it needs a shunt and a decent ADC.

## Recommended order

1. **One INA226 on the supply rail first.** It addresses the real problem (the
   protection cutting out), it speaks I2C, and it occupies no ADC.
2. **Measure what one wheel actually draws** before buying four sensors. Order
   of magnitude from the records: startup at 5.61 A total passed, 7.40 A tripped.
   Per wheel that is well under 10 A, and a ±10 A sensor used for ~2 A wastes
   resolution — **±5 A could give twice the measurement quality**.
3. **Four sensors on the motors only once there is a better ADC.** An analogue
   output plus a divider plus the ESP32-S3's non-linear ADC is three error
   sources stacked.

## Sources

- https://www.ti.com/product/INA226
- https://www.ti.com/lit/gpn/INA240
- https://www.ti.com/product/INA3221
- https://www.pololu.com/product/4043 (ACS724LLCTR-10AB)
- https://www.allegromicro.com/-/media/files/datasheets/acs724-datasheet.ashx
