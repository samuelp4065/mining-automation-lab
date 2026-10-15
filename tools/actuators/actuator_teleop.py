"""
This file uses Teleoperation to control the linear actuators on the excavator. The actuators are
controlled by a PCA9685 PWM driver, which is connected to the Jetson Nano via I2C. The actuators are
controlled by sending PWM signals to the PCA9685, which then drives the actuators.

u / i   boom    extend / retract
j / k   arm     extend / retract
n / m   bucket  extend / retract

[ / ]      smaller / larger step
space bar  stop all actuators
q          quit

When program runs, the user can press the keys to extend or retract the actuators. The actuators will
move a small step in the direction of the key pressed. The user can also press the space bar to stop all
the actuators. The user can also press 'q' to quit the program. The program will also print the current
position of the actuators in mm.

Also,at the start of the program will only add the step size to the current position of the actuator meaning,
there will not be movement to a certain position, but rather a movement of a certain step size.

Every move is saved to STATE_FILE, and the next run starts from those saved angles. That is only true
if nothing moved the joints while the program was off. If something did, start with --reset and --start
set to where the joints really are.

    python3 tools/actuators/actuator_teleop.py                    # resume from the saved positions
    python3 tools/actuators/actuator_teleop.py --step 2           # finer steps
    python3 tools/actuators/actuator_teleop.py --reset --start 90 # forget the file, assume mid stroke
"""


import argparse
import json
import os
import sys
import termios
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_control"))

from excavator_control.pca9685 import PCA9685, PCAServo  # noqa: E402

BOOM_MAXMIN = (0, 180)
ARM_MAXMIN = (0, 180)
BUCKET_MAXMIN = (0, 180)

STROKE_MM = 100.0

# joint -> (extend key, retract key, PCA9685 channel, (min, max) angle)
JOINTS = {
    "boom": ("u", "i", 0, BOOM_MAXMIN),
    "arm": ("j", "k", 1, ARM_MAXMIN),
    "bucket": ("n", "m", 2, BUCKET_MAXMIN),
}

# Next to this script, and listed in .gitignore: a checkout must never overwrite it with a stale
# position, or the next key press drives from a place the arm isn't.
STATE_FILE = Path(__file__).with_name("actuator_state.json")


def angle_to_mm(angle, limits):
    low, high = limits
    return (angle - low) / (high - low) * STROKE_MM


def load_positions(start):
    """Return ({joint: angle}, restored). Joints missing from the file get `start`."""
    if not STATE_FILE.exists():
        return {name: start for name in JOINTS}, False

    saved = json.loads(STATE_FILE.read_text())
    positions = {}
    for name, (_, _, _, (low, high)) in JOINTS.items():
        angle = float(saved.get(name, start))
        if not low <= angle <= high:
            # Don't quietly clamp it: a bad saved value means the file no longer describes the arm.
            sys.exit(f"saved {name} angle {angle} is outside {low}..{high}; "
                     f"rerun with --reset --start <angle>")
        positions[name] = angle
    return positions, True


def save_positions(positions):
    """Write to a temp file, then rename it over the real one.

    The rename is atomic, so a crash or power cut leaves either the old file or the new one,
    never a half-written file that fails to load next time.
    """
    tmp = STATE_FILE.with_suffix(".tmp")
    with open(tmp, "w") as f:
        json.dump(positions, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, STATE_FILE)


def read_key(settings):
    """One keypress, without waiting for Enter."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def status(positions, step, note=""):
    cells = "   ".join(
        f"{name} {positions[name]:5.1f} ({angle_to_mm(positions[name], JOINTS[name][3]):5.1f}mm)"
        for name in JOINTS
    )
    sys.stdout.write(f"\r  {cells}   step {step:4.1f}   {note:<24}")
    sys.stdout.flush()


def main(argv=None):
    p = argparse.ArgumentParser(description="Step the linear actuators from the keyboard.")
    p.add_argument("--step", type=float, default=5.0, help="degrees per key press (default 5)")
    p.add_argument("--start", type=float, default=90.0,
                   help="angle to assume when there is no saved file, or with --reset (default 90)")
    p.add_argument("--reset", action="store_true", help="ignore the saved file and use --start")
    p.add_argument("--bus", type=int, default=7)
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x40)
    args = p.parse_args(argv)

    if args.reset:
        positions, restored = {name: args.start for name in JOINTS}, False
    else:
        positions, restored = load_positions(args.start)

    keymap = {}
    for name, (extend, retract, _, _) in JOINTS.items():
        keymap[extend] = (name, +1)
        keymap[retract] = (name, -1)

    print(f"\n  {'restored from' if restored else 'no saved positions, assuming'} "
          f"{STATE_FILE if restored else args.start}")
    print("  u/i boom   j/k arm   n/m bucket   [ ] step   space release   q quit\n")

    pca = PCA9685(bus=args.bus, address=args.address, frequency=50)
    # No initial_angle: nothing is sent until the first key press, so starting the program never
    # moves anything.
    servos = {
        name: PCAServo(pca, channel, min_angle=low, max_angle=high)
        for name, (_, _, channel, (low, high)) in JOINTS.items()
    }
    settings = termios.tcgetattr(sys.stdin)
    step = args.step

    try:
        status(positions, step, "ready")
        while True:
            key = read_key(settings)

            if key in ("q", "\x03"):            # q or Ctrl-C (raw mode delivers Ctrl-C as a key)
                break

            if key == " ":
                for servo in servos.values():
                    servo.detach()
                status(positions, step, "released")
                continue

            if key in ("[", "]"):
                step = max(0.5, min(45.0, step * (0.5 if key == "[" else 2.0)))
                status(positions, step, "step changed")
                continue

            if key not in keymap:
                continue

            name, direction = keymap[key]
            low, high = JOINTS[name][3]
            target = max(low, min(high, positions[name] + direction * step))
            if target == positions[name]:
                status(positions, step, f"{name} at {'max' if direction > 0 else 'min'}")
                continue

            servos[name].angle = target
            positions[name] = target
            save_positions(positions)            # every move, not just on exit, in case of a crash
            status(positions, step, f"{name} {'extend' if direction > 0 else 'retract'}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # Open loop: anything left driven keeps pushing. The L16 holds position once released.
        for servo in servos.values():
            servo.detach()
        pca.close()
        save_positions(positions)
        print("\n\n  all channels released, positions saved\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
