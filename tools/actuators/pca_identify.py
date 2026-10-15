#!/usr/bin/env python3
"""Work out which PCA9685 channel drives which joint, one channel at a time.

Nothing can detect this electrically -- a PCA9685 output is push-pull into a
high-impedance servo input, so an empty channel is indistinguishable from a
connected one. The only way to map channels to joints is to move one and watch.

Safety shape, deliberate:

* **One channel per step**, each gated on you pressing Enter. Nothing moves
  while you are looking at something else.
* **Settles at mid-stroke first.** The first command to an actuator is absolute
  and it drives there at full speed from wherever it sits, so the opening move
  goes to the middle of travel -- furthest from both end stops -- rather than
  to an extreme.
* **Identifies with a small wiggle.** Once at mid-stroke, +/-15 degrees is
  about 17 mm of rod: clearly visible, far from anything.
* **Always detaches**, including on Ctrl-C.

Usage::

    python3 tools/actuators/pca_identify.py             # channels 0, 1, 2
    python3 tools/actuators/pca_identify.py --channels 0 1 2 3
    python3 tools/actuators/pca_identify.py --skip-centre   # already centred
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_control"))

from excavator_control.pca9685 import PCA9685, PCAServo  # noqa: E402

STROKE_MM = 100.0
MAX_SPEED_MM_S = 8.0          # L16-100-150-6-R, no load
#: Worst case the first command crosses the whole stroke. 100mm at 8mm/s is
#: 12.5s unloaded and longer under load, so allow generous settling.
FULL_STROKE_S = STROKE_MM / MAX_SPEED_MM_S
WIGGLE_DEG = 15.0


def mm_for(angle):
    return (angle + 90.0) / 180.0 * STROKE_MM


def ask(prompt):
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        raise KeyboardInterrupt


def identify(pca, channels, skip_centre):
    found = {}
    for channel in channels:
        print(f"\n{'=' * 58}")
        print(f"  CHANNEL {channel}")
        print(f"{'=' * 58}")

        answer = ask(f"  Move channel {channel}? [Enter to go, s to skip, q to quit] ")
        if answer.lower() == "q":
            raise KeyboardInterrupt
        if answer.lower() == "s":
            found[channel] = "skipped"
            continue

        servo = PCAServo(pca, channel)
        try:
            if not skip_centre:
                wait = FULL_STROKE_S * 1.5
                print(f"  centring (angle 0, ~{mm_for(0):.0f} mm) -- up to "
                      f"{wait:.0f}s if it starts at an extreme")
                servo.angle = 0.0
                time.sleep(wait)

            print(f"  wiggling +/-{WIGGLE_DEG:.0f} deg "
                  f"(~{WIGGLE_DEG / 180 * STROKE_MM * 2:.0f} mm of rod) -- WATCH")
            for angle in (+WIGGLE_DEG, -WIGGLE_DEG, +WIGGLE_DEG, 0.0):
                servo.angle = angle
                time.sleep(4)

            joint = ask(f"  Which joint moved on channel {channel}? "
                        f"(boom/arm/bucket, or 'none') ")
            found[channel] = joint or "no answer"
        finally:
            servo.detach()
            print(f"  channel {channel} detached")
    return found


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--channels", type=int, nargs="+", default=[0, 1, 2],
                   help="channels to step through (default 0 1 2)")
    p.add_argument("--skip-centre", action="store_true",
                   help="skip the centring move; only use when the joints are "
                        "already near mid-stroke")
    p.add_argument("--bus", type=int, default=7)
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x40)
    args = p.parse_args(argv)

    print("\n  THIS MOVES THE MACHINE, one channel at a time.")
    print("  Check the bucket is clear of the chassis and nothing is in the arc.")
    print("  Ctrl-C at any point detaches everything.")

    pca = PCA9685(bus=args.bus, address=args.address, frequency=50)
    print(f"\n  PCA9685 on /dev/i2c-{args.bus} at {args.address:#04x}, "
          f"{pca.frequency:.2f} Hz")

    found = {}
    try:
        found = identify(pca, args.channels, args.skip_centre)
    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        pca.all_off()
        pca.close()
        print("\n  all channels off, PCA9685 closed")

    if found:
        print("\n  Channel map:")
        for channel, joint in sorted(found.items()):
            print(f"    channel {channel}  ->  {joint}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
