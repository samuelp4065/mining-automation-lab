---
title: Actuation
layout: default
nav_order: 3
---

# Actuation

How a joint angle becomes rod movement, and why PWM generation lives off-board.

---

## Why PWM is not on the Jetson header

The Orin Nano has exactly **three** hardware-PWM pins on its 40-pin header — BOARD 15, 32
and 33. This project was built around them, and then none of them would drive their pads.

What was ruled out, each with a direct measurement:

| Suspect | How it was eliminated |
|---|---|
| The library | Kernel-level `gpioset` (libgpiod 1.6.3) failed identically to `Jetson.GPIO` |
| The register | `gpioinfo` showed the line claimed as output, controller read back `1` when set high — the pad did not follow |
| PWM configuration | Registers verified: 20 ms period, 1.0/1.5/2.0 ms pulses, enabled, normal polarity |
| The pinmux | `pin 85 (SOC_GPIO39_PN1): (HOG) function gp` — NVIDIA's own AGX Orin overlay declares the identical pad identically |
| The measurement | Pin 1 read 3.3 V, pins 2/4 read 5 V, grounds 0 V, same probe |
| PWM itself | Plain GPIO pin 7, needing no configuration at all, also refused to go high |

A pad commanded high measured **~1.6 V** — half rail, the signature of a floating pin. The
leading explanation is overvoltage damage, since Jetson header pins are 3.3 V and not 5 V
tolerant and a 6 V rail sat beside the header throughout bring-up.

{: .highlight }
> **Measuring PWM with a multimeter is counter-intuitive.** A meter on DC volts averages the
> waveform, so a correct 50 Hz RC signal reads only **0.17–0.33 V** — *lower* than the
> ~1.6 V a floating pin shows. Worse, testing at 50% duty is the one case that cannot
> distinguish working from dead, because 50% of 3.3 V is also ~1.65 V. Probe at 10% or 90%
> duty instead.

---

## The PCA9685

PWM generation moved to a **PCA9685** on I2C bus 7 at address `0x40`. This removed the
three-pin constraint entirely — the chip drives sixteen channels in hardware.

| PCA9685 pin | Connects to | Note |
|---|---|---|
| VCC | Jetson 3.3 V (pin 1) | Logic supply — **sets the output signal level** |
| GND | Jetson GND, common with the 6 V supply | A shared ground reference is mandatory |
| SDA | Jetson pin 3 | I2C bus 7 |
| SCL | Jetson pin 5 | |
| V+ | 6 V supply | Separate servo power rail for the 3-pin headers |

Confirm the board is present with:

```bash
i2cdetect -y -r 7
```

It answers at `0x40`, and also on `0x70` (the All Call address, enabled by default).

The L16-R datasheet specifies a 5 V pulse, but **3.3 V signalling works** — confirmed
because these actuators previously ran from a Raspberry Pi 4, whose GPIO is 3.3 V and not
5 V tolerant. No level shifter is required.

---

## The driver

`excavator_control/pca9685.py` has **no third-party dependency**. The chip needs only
single- and multi-byte register writes, which go straight to `/dev/i2c-N` through an
ioctl; adding `smbus` would buy nothing.

At 50 Hz the mapping is:

| Angle | Pulse | 12-bit counts | Rod position |
|---|---|---|---|
| −90° | 1.000 ms | 205 / 4096 | 0 mm, retracted |
| 0° | 1.500 ms | 307 / 4096 | 50 mm, mid stroke |
| +90° | 2.000 ms | 410 / 4096 | 100 mm, extended |

---

## Control interface

`PCAServo` presents the same `.angle` / `.detach()` surface as `gpiozero.AngularServo`,
which the original Raspberry Pi scripts were written against. Every angle in those scripts,
and everything the calibration module produces, therefore carries over without translation.

```python
from excavator_control.pca9685 import PCA9685, PCAServo

pca = PCA9685(bus=7, address=0x40, frequency=50)
bucket = PCAServo(pca, channel=2)
try:
    bucket.angle = 45          # extend to 75 mm
finally:
    bucket.detach()            # release; the actuator holds position
```

Two deliberate design decisions:

1. **Out-of-range angles raise rather than clamp.** Clamping would let a bad calibration
   constant hold an actuator against its end stop — indistinguishable from a reached
   position, and exactly the silent saturation the joint-limit check exists to prevent.
2. **Releasing disables the signal**, it does not command a neutral position. See
   [Hardware](hardware) for why that parks the arm rather than dropping it.

---

## Bench tools

All of these run standalone, without a ROS environment.

| Tool | Purpose |
|---|---|
| `tools/actuators/pca_identify.py` | Work out which channel drives which joint, one at a time |
| `tools/actuators/pca_set.py` | Command one channel to one angle and hold it |
| `tools/actuators/pca_test.py` | Sweep one channel, then an interactive angle prompt |
| `tools/actuators/actuator_teleop.py` | Keyboard teleop for all three joints |

```bash
python3 tools/actuators/pca_identify.py        # map channels to joints
python3 tools/actuators/pca_set.py 45          # extend channel 0 to 75 mm
python3 tools/actuators/actuator_teleop.py     # u/i boom, j/k arm, n/m bucket
```

{: .warning }
> The first command to an actuator is **absolute**. An RC actuator's pulse width encodes a
> position, so it drives there at full speed from wherever the rod physically sits — and
> nothing can read that back. At 8 mm/s a worst-case opening move crosses the whole stroke
> in 12.5 s. Tools here default to mid-stroke for that first command, which is as far from
> both end stops as it gets.
