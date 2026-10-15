#!/usr/bin/env python3
"""Hold steady PWM duty cycles on a pin so they can be read with a multimeter.

Why 476 Hz by default
---------------------
A multimeter on DC volts reads the average of the waveform, which is
``duty x 3.3 V``. At the RC-standard 50 Hz frame a 1-2 ms pulse is only 5-10%
duty, so the meter shows 0.17-0.33 V -- small enough to be mistaken for noise.
Packing the same 1-2 ms pulses into a 2.1 ms frame gives 48-95% duty and a
1.6-3.1 V reading, where the steps are unmistakable.

This measures the *pad*, which is the one thing software cannot check: the PWM
controller's registers read correct whether or not its output reaches the pin.

Setup
-----
Disconnect whatever is on the pin first, so nothing loads it. Black probe on any
Jetson ground pin (6, 9, 14, 20, 25, 30, 34, 39), red probe on the pin under
test, meter on DC volts.

Usage::

    python3 tools/actuators/pin_voltage.py                 # pin 15, press Enter to step
    python3 tools/actuators/pin_voltage.py --pin 33
    python3 tools/actuators/pin_voltage.py --hold 15       # timed instead of Enter
    python3 tools/actuators/pin_voltage.py --frame-width 0.02   # 50 Hz, small readings
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_control"))

from excavator_control.jetson_servo import JetsonServo  # noqa: E402

VCC = 3.3


def run(pin, frame_width, hold, lead_in, commands):
    print(f"\n  Pin under test : BOARD {pin}")
    print(f"  Frame          : {frame_width * 1000:.2f} ms "
          f"({1.0 / frame_width:.1f} Hz)")
    print(f"  Meter          : DC volts, black probe on any GND pin "
          f"(6, 9, 14, 20, 25, 30, 34, 39)")
    print(f"\n  Disconnect anything attached to pin {pin} first so it cannot "
          f"load the output.\n")

    if lead_in:
        print(f"  Starting in {lead_in}s -- get the probes placed.")
        for remaining in range(lead_in, 0, -5):
            print(f"    {remaining}s ...", flush=True)
            time.sleep(min(5, remaining))
        print()

    servo = JetsonServo(pin, frame_width=frame_width)
    try:
        for cmd in commands:
            pulse_ms = 1.0 + (cmd + 90) / 180.0
            duty = pulse_ms / (frame_width * 1000.0) * 100.0
            print(f"  command {cmd:+4}   pulse {pulse_ms:.3f} ms   "
                  f"duty {duty:5.2f}%   EXPECT ~{duty / 100 * VCC:.2f} V")
            servo.angle = cmd
            if hold:
                time.sleep(hold)
            else:
                try:
                    input("      holding -- read the meter, then press Enter ")
                except (EOFError, KeyboardInterrupt):
                    print()
                    return 130
    finally:
        servo.close()
        time.sleep(1)
        print("\n  released -- EXPECT ~0.00 V")

    print("""
  How to read the result:

    steps up roughly as predicted -> the pad is alive. The Jetson side is
        proven and the actuator not moving is a signal-level problem
        (it wants 5 V, this pin gives 3.3 V).
    flat 0 V throughout           -> the pad is not driving at all, despite
        the pinmux and PWM registers both reading correct.
    steady 3.3 V, never changing  -> stuck high; no pulse is forming.
""")
    return 0


def run_gpio(pin, hold, lead_in):
    """Drive the pin as a plain GPIO output, high and low.

    This is the test that separates "the pad cannot drive at all" from "the pad
    works but the PWM controller's output is not reaching it". It bypasses PWM
    entirely -- no timing, no duty cycle, just the output driver.
    """
    import os
    os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")
    import Jetson.GPIO as GPIO

    print(f"\n  Driving BOARD pin {pin} as a plain GPIO output (no PWM).")
    print(f"  Meter on DC volts, black probe on GND.\n")

    if lead_in:
        print(f"  Starting in {lead_in}s ...")
        time.sleep(lead_in)
        print()

    GPIO.setwarnings(False)
    if GPIO.getmode() is None:
        GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    try:
        for level, expect in ((0, "0.00 V"), (1, "3.30 V"),
                              (0, "0.00 V"), (1, "3.30 V")):
            GPIO.output(pin, level)
            print(f"  pin {pin} driven {'HIGH' if level else 'LOW '}   "
                  f"EXPECT ~{expect}")
            if hold:
                time.sleep(hold)
            else:
                try:
                    input("      read the meter, then press Enter ")
                except (EOFError, KeyboardInterrupt):
                    print()
                    return 130
    finally:
        GPIO.cleanup(pin)

    print("""
  How to read the result:

    swings 0 V <-> 3.3 V  -> the pad drives fine. The fault is specific to
        getting the PWM controller's output onto it.
    stays ~1.58 V floating -> the pad cannot drive at all, which is a lower
        level fault than anything PWM-related.
""")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--gpio", action="store_true",
                   help="drive the pin as a plain GPIO high/low instead of "
                        "PWM -- tests whether the pad can drive at all")
    p.add_argument("--pin", type=int, default=15,
                   help="BOARD pin to drive (default 15; 32 and 33 are the "
                        "other two PWM-capable pins)")
    p.add_argument("--frame-width", type=float, default=0.0021,
                   help="frame period in seconds (default 0.0021 = 476 Hz, "
                        "chosen so the meter reading is large)")
    p.add_argument("--hold", type=float, default=None,
                   help="seconds to hold each step; omit to advance on Enter")
    p.add_argument("--lead-in", type=int, default=20,
                   help="seconds before the first step (default 20)")
    args = p.parse_args(argv)

    try:
        if args.gpio:
            return run_gpio(args.pin, args.hold, args.lead_in)
        return run(args.pin, args.frame_width, args.hold, args.lead_in,
                   commands=(-90, 0, 90))
    except KeyboardInterrupt:
        print("\n  interrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
