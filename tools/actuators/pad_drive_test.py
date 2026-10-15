#!/usr/bin/env python3
"""Find out which code paths can actually hold a pad driven, with a multimeter.

Background
----------
On this board pin 15 measured 0.003 V during ``GPIO.setup(..., initial=LOW)``
-- genuinely driven -- and then floated at ~1.56 V through every subsequent
``GPIO.output()`` call and through PWM. So the pad hardware works, but
something releases it. That is a different fault from a dead pin, and it needs
a different fix.

This walks the distinct ways of driving the pad and has you read each one, so
we learn exactly which path holds and which lets go:

  A  setup(initial=HIGH)   -- can the pad source at all?
  B  setup(initial=LOW)    -- the one case already known to work
  C  output(HIGH) after setup  -- the call that appeared to fail
  D  PWM at 95% duty       -- the path we actually need

Setup
-----
Nothing connected to the pin. Black probe on a Jetson ground pin
(6, 9, 14, 20, 25, 30, 34, 39), red probe on the pin, meter on DC volts.

Usage::

    python3 tools/actuators/pad_drive_test.py            # pin 15
    python3 tools/actuators/pad_drive_test.py --pin 33
"""

import argparse
import os
import sys
import time

os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")
import Jetson.GPIO as GPIO  # noqa: E402

VCC = 3.3


def step(label, expect, action, teardown=None):
    print(f"\n  [{label}]  EXPECT ~{expect}")
    try:
        action()
    except Exception as exc:
        print(f"      !! raised {type(exc).__name__}: {exc}")
        return
    try:
        input("      read the meter, then press Enter ")
    except (EOFError, KeyboardInterrupt):
        raise KeyboardInterrupt
    finally:
        if teardown:
            try:
                teardown()
            except Exception:
                pass


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--pin", type=int, default=15)
    args = p.parse_args(argv)
    pin = args.pin

    print(f"\n  Pad drive test on BOARD pin {pin}")
    print("  Nothing should be connected to this pin.")
    print("  Black probe on GND, red on the pin, meter on DC volts.")

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    def fresh(initial):
        def go():
            GPIO.cleanup(pin)
            GPIO.setup(pin, GPIO.OUT, initial=initial)
        return go

    try:
        step("A  setup(initial=HIGH)", "3.30 V", fresh(GPIO.HIGH))
        step("B  setup(initial=LOW)", "0.00 V", fresh(GPIO.LOW))

        def c():
            GPIO.cleanup(pin)
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(pin, GPIO.HIGH)
        step("C  output(HIGH) after setup", "3.30 V", c)

        state = {}

        def d():
            GPIO.cleanup(pin)
            state["pwm"] = GPIO.PWM(pin, 476)
            state["pwm"].start(95.0)
        def d_stop():
            if "pwm" in state:
                state["pwm"].stop()
                del state["pwm"]
        step("D  PWM 476 Hz at 95% duty", f"{0.95 * VCC:.2f} V", d, d_stop)

    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass
        print("\n  cleaned up\n")

    print("""  How to read it:

    A high, B low        -> the pad sources and sinks; only output() is
                            failing to hold. Workaround: re-setup() instead
                            of output(), and PWM should still be fine.
    A floats, B low      -> the pad can sink but not source. That is
                            open-drain behaviour and needs a pull-up to 3.3V
                            on the signal line to form a usable high.
    D correct            -> PWM holds the pad. The actuator problem is then
                            purely the 3.3V vs 5V signal level.
    D floats             -> PWM cannot hold the pad, whatever GPIO does.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
