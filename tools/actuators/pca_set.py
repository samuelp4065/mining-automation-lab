#!/usr/bin/env python3
"""Command one PCA9685 channel to one angle and hold it. No sweep, no prompts.

Angles follow gpiozero's AngularServo convention, the same one the legacy Pi
scripts and ``excavator_kinematics.calibration`` use::

    -90  =  1.0 ms  =  fully retracted   (  0 mm)
      0  =  1.5 ms  =  mid stroke        ( 50 mm)
    +90  =  2.0 ms  =  fully extended    (100 mm)

The hold matters. These are open loop, so the signal has to stay present until
the rod arrives -- detaching early stops it partway. At 8 mm/s a full 100 mm
stroke needs ~13 s unloaded and longer under load, which is why the default
hold is generous.

Usage::

    python3 tools/actuators/pca_set.py 90              # extend, channel 0
    python3 tools/actuators/pca_set.py -90             # retract
    python3 tools/actuators/pca_set.py 45 --channel 2
    python3 tools/actuators/pca_set.py 0 --hold 20
    python3 tools/actuators/pca_set.py 90 --mm         # read the arg as mm
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_control"))

from excavator_control.pca9685 import PCA9685, PCAServo  # noqa: E402

STROKE_MM = 100.0
MAX_SPEED_MM_S = 8.0     # L16-100-150-6-R, no load


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("value", type=float,
                   help="angle -90..90 (or mm 0..100 with --mm)")
    p.add_argument("--mm", action="store_true",
                   help="read the value as rod extension in mm instead")
    p.add_argument("--channel", type=int, default=0)
    p.add_argument("--hold", type=float, default=15.0,
                   help="seconds to keep the signal present (default 15, "
                        "enough for a full stroke unloaded)")
    p.add_argument("--stay", action="store_true",
                   help="keep driving until Ctrl-C instead of detaching")
    p.add_argument("--bus", type=int, default=7)
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x40)
    args = p.parse_args(argv)

    if args.mm:
        if not 0.0 <= args.value <= STROKE_MM:
            p.error(f"--mm value must be 0..{STROKE_MM:.0f}")
        angle = -90.0 + args.value / STROKE_MM * 180.0
    else:
        angle = args.value

    mm = (angle + 90.0) / 180.0 * STROKE_MM
    pulse_ms = 1.0 + (angle + 90.0) / 180.0
    direction = "extend" if angle > 0 else "retract" if angle < 0 else "centre"

    pca = PCA9685(bus=args.bus, address=args.address, frequency=50)
    servo = PCAServo(pca, args.channel)
    try:
        servo.angle = angle
        print(f"  channel {args.channel}: angle {angle:+.1f} ({direction})  "
              f"{pulse_ms:.3f} ms  ~{mm:.1f} mm")
        print(f"  worst case travel from the far end is "
              f"{STROKE_MM / MAX_SPEED_MM_S:.1f}s")
        if args.stay:
            print("  holding until Ctrl-C ...")
            while True:
                time.sleep(1)
        else:
            print(f"  holding {args.hold:.0f}s ...")
            time.sleep(args.hold)
    except (KeyboardInterrupt, EOFError):
        print("\n  interrupted")
    except ValueError as exc:
        print(f"  error: {exc}", file=sys.stderr)
        return 1
    finally:
        # Open loop: anything left driven keeps pushing. The L16 holds its
        # position mechanically once released, so detaching parks it.
        servo.detach()
        pca.close()
        print("  detached")
    return 0


if __name__ == "__main__":
    sys.exit(main())
