#!/usr/bin/env python3
"""Drive one L16-R from the PCA9685. Replaces the dead Jetson PWM pins.

Wiring check before running:
  PCA9685 VCC -> Jetson 3.3V (pin 1)   -- sets the output signal level
  PCA9685 GND -> Jetson GND (pin 6)    -- and common with the 6V supply ground
  PCA9685 SDA -> Jetson pin 3
  PCA9685 SCL -> Jetson pin 5
  PCA9685 V+  -> 6V supply             -- the servo power rail
  Actuator on a channel header: white/signal to PWM, red to V+, black to GND

Usage::

    python3 tools/actuators/pca_test.py              # channel 0
    python3 tools/actuators/pca_test.py --channel 2
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_control"))

from excavator_control.pca9685 import PCA9685, PCAServo  # noqa: E402

STROKE_MM = 100.0
MAX_SPEED_MM_S = 8.0     # L16-100-150-6-R, no load


def move(servo, angle, wait_s):
    servo.angle = angle
    mm = (angle + 90.0) / 180.0 * STROKE_MM
    pulse_ms = 1.0 + (angle + 90.0) / 180.0
    print(f"  angle {angle:+7.1f}  ->  {pulse_ms:.3f} ms  (~{mm:5.1f} mm)   "
          f"waiting {wait_s}s", flush=True)
    time.sleep(wait_s)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--channel", type=int, default=0, help="PCA9685 channel 0-15")
    p.add_argument("--bus", type=int, default=7)
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x40)
    args = p.parse_args(argv)

    pca = PCA9685(bus=args.bus, address=args.address, frequency=50)
    print(f"PCA9685 on /dev/i2c-{args.bus} at {args.address:#04x}, "
          f"{pca.frequency:.2f} Hz, channel {args.channel}\n")

    servo = PCAServo(pca, args.channel)
    try:
        # Retract first so the start point is known -- these are open loop and
        # nothing can read the current position. A full stroke needs ~13s.
        print("  retracting to -90 so the start point is known")
        move(servo, -90, 14)

        print("\n  sweep")
        move(servo, 0, 8)
        move(servo, 90, 8)
        move(servo, -45, 10)
        move(servo, -90, 8)

        print("\n  manual: type an angle -90..90, or q to quit")
        while True:
            raw = input("  angle> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                servo.angle = float(raw)
            except ValueError as exc:
                print(f"    {exc}" if "outside" in str(exc)
                      else "    enter a number between -90 and 90, or q")
    except (KeyboardInterrupt, EOFError):
        print("\nstopped")
    finally:
        # Open loop: anything left driven keeps pushing against its end stop.
        servo.detach()
        pca.close()
        print("detached, PCA9685 closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
