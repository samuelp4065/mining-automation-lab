#!/usr/bin/env python3
"""Drive one L16-R from the Jetson, with gpiozero's AngularServo semantics.

Why the angle API matters
-------------------------
Every value in the old Raspberry Pi scripts (dig_V1-V3, Body.py, bodycontrol.py)
is a ``gpiozero.AngularServo.angle`` in [-90, 90], and so is everything
``excavator_kinematics.calibration`` produces. Reimplementing that mapping here
means those angles and their measured sleep timings carry over unchanged.

gpiozero computes the pulse width as a straight linear interpolation::

    fraction = (angle - min_angle) / (max_angle - min_angle)
    pulse    = min_pulse_width + fraction * (max_pulse_width - min_pulse_width)

With the defaults used throughout this project -- min_angle -90, max_angle +90,
1.0 ms to 2.0 ms -- that gives:

    angle -90  ->  1.0 ms  (fully retracted,  0 mm)
    angle   0  ->  1.5 ms  (mid stroke,      50 mm)
    angle +90  ->  2.0 ms  (fully extended, 100 mm)

Jetson.GPIO wants a duty cycle in percent rather than a pulse width, which is
the only real difference from the Pi code: ``duty = pulse_ms / period_ms * 100``.
"""

import os
import time

os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")
import Jetson.GPIO as GPIO

PIN = 33                 # BOARD pin. 32 and 33 work; 15 does not drive.
FREQ_HZ = 50             # RC standard, 20 ms period
PERIOD_MS = 1000.0 / FREQ_HZ

MIN_PULSE_MS = 1.0       # angle -90, fully retracted
MAX_PULSE_MS = 2.0       # angle +90, fully extended
MIN_ANGLE = -90.0
MAX_ANGLE = 90.0
STROKE_MM = 100.0


def angle_to_pulse_ms(angle):
    """gpiozero AngularServo's exact mapping: angle in [-90, 90] -> pulse ms."""
    if not MIN_ANGLE <= angle <= MAX_ANGLE:
        raise ValueError(f"angle {angle} outside [{MIN_ANGLE}, {MAX_ANGLE}]")
    fraction = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE)
    return MIN_PULSE_MS + fraction * (MAX_PULSE_MS - MIN_PULSE_MS)


def pulse_to_duty(pulse_ms):
    return pulse_ms / PERIOD_MS * 100.0


def set_angle(pwm, angle):
    """Command an angle, the same value the Pi scripts and calibration.py use."""
    pulse = angle_to_pulse_ms(angle)
    pwm.ChangeDutyCycle(pulse_to_duty(pulse))
    return pulse


def set_position(pwm, frac):
    """Command by stroke fraction instead: 0.0 retracted, 1.0 extended."""
    return set_angle(pwm, MIN_ANGLE + frac * (MAX_ANGLE - MIN_ANGLE))


def move_to_angle(pwm, angle, wait_s):
    """Command an angle and wait. The wait is the only completion feedback
    there is -- these are open loop, so nothing can report arrival."""
    pulse = set_angle(pwm, angle)
    mm = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE) * STROKE_MM
    print(f"  angle {angle:+7.1f}  ->  {pulse:.3f} ms  "
          f"({pulse_to_duty(pulse):5.2f}% duty, ~{mm:5.1f} mm)   "
          f"waiting {wait_s}s", flush=True)
    time.sleep(wait_s)


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.OUT)
    pwm = GPIO.PWM(PIN, FREQ_HZ)
    pwm.start(pulse_to_duty(MIN_PULSE_MS))

    print(f"Pin {PIN} at {FREQ_HZ} Hz. Ctrl-C to stop.\n")
    try:
        print("  retracting to -90 first, so the start point is known")
        move_to_angle(pwm, -90, 14)      # full stroke at 8 mm/s needs ~13 s

        print("\n  sweep")
        move_to_angle(pwm, 0, 8)
        move_to_angle(pwm, 90, 8)
        move_to_angle(pwm, -45, 10)
        move_to_angle(pwm, -90, 8)

        print("\n  manual: type an angle -90..90, or q to quit")
        while True:
            raw = input("  angle> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                set_angle(pwm, float(raw))
            except ValueError as exc:
                print(f"    {exc}" if "outside" in str(exc)
                      else "    enter a number between -90 and 90, or q")
    except (KeyboardInterrupt, EOFError):
        print("\nstopped")
    finally:
        # Open loop: anything left driven keeps pushing. This must always run.
        pwm.stop()
        GPIO.cleanup()
        print("PWM stopped, GPIO cleaned up.")


if __name__ == "__main__":
    main()
