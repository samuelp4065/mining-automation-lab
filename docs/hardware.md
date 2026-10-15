---
title: Hardware
layout: default
nav_order: 2
---

# Hardware topology

Two control interfaces, and knowing which actuator sits on which is what makes the
control code readable.

---

## LSS smart servos

One shared serial bus on `/dev/ttyUSB0`, addressed by ID. These have position feedback.

| ID | Role | Control |
|---|---|---|
| 0 | Left track | `wheelRPM()` |
| 1 | Right track | `wheelRPM()` |
| 2 | Swingdrive (cab rotation) | `move()` to absolute position, `getPosition()` |

The left servo is mounted mirrored, so forward motion needs a **negative** RPM command on
it:

```
forward  =>  left.wheelRPM(-r),  right.wheelRPM(+r)
```

That sign convention lives in exactly one place, `drive_config.py`, because the drive node
and the odometry node both depend on it. If they disagree, the machine drives correctly
while reporting that it went backwards — and nav2 believes the odometry, not the machine.

---

## Linear actuators

Three **Actuonix L16-100-150-6-R** units drive boom, stick and bucket. They are open loop
from the computer's point of view: nothing can read the rod position back.

| Spec | Value | Why it matters |
|---|---|---|
| Stroke | 100 mm | Maps to a 1.0–2.0 ms pulse, so 0.56 mm per command unit |
| Gearing | 150:1 | Slowest in the range — maximum force, minimum speed |
| Speed | 8 mm/s no-load | A full stroke takes **12.5 s**, up to 25 s under load |
| Max force | 200 N | |
| Back drive force | 102 N | Holds position when released; not movable by hand |
| Supply | **6 V only** | Reversed polarity may permanently damage the unit |
| Max duty cycle | 20% thermal | A real constraint on dig-cycle design |
| Repeatability | 0.4 mm | |

Wiring is three leads: **white** signal, **red** +6 V, **black** ground.

{: .warning }
> Stalling is damaging, not merely untidy. The datasheet is explicit that stalling for more
> than a few seconds significantly shortens actuator life. That is why joint limits are
> enforced before any command is issued, and why releasing an actuator means disabling its
> signal rather than commanding a "safe" position.

Because the L16 back-drives only above 102 N, a released actuator **parks** rather than
dropping. That makes `try`/`finally` around every motion loop a safety guarantee rather
than a tidiness habit.

See [Actuation](actuators) for how PWM reaches these.

---

## Power

The actuators run from a dedicated **6 V** supply, separate from the Jetson. Two
requirements that are easy to miss:

- **Common ground.** The actuator supply ground must tie to a Jetson ground pin. A PWM
  pulse is a voltage relative to ground; without a shared reference the actuator sees
  nothing valid, even with perfect power and a perfect signal.
- **Current capacity.** Each actuator draws up to ~650 mA and more at stall. USB power
  banks are a poor choice: most switch off when draw falls below ~50–100 mA, which is
  exactly what happens once an actuator reaches position and goes idle.

{: .warning }
> **Jetson header pins are 3.3 V and not 5 V tolerant.** Applying a higher voltage damages
> the pin's output driver, often while leaving the register readable so the fault looks
> like a configuration problem. Keep the 6 V rail physically away from the header.
