"""
This file uses Teleoperation to control the linear actuators on the excavator. The actuators are
controlled by a PCA9685 PWM driver, which is connected to the Jetson Nano via I2C. The actuators are 
controlled by sending PWM signals to the PCA9685, which then drives the actuators. 

u / i   boom    extend / retract
j / k   arm     extend / retract
n / m   bucket  extend / retract

space bar  stop all actuators

When program runs, the user can press the keys to extend or retract the actuators. The actuators will
move a small step in the direction of the key pressed. The user can also press the space bar to stop all
the actuators. The user can also press 'q' to quit the program. The program will also print the current
position of the actuators in mm. 

Also,at the start of the program will only add the step size to the current position of the actuator meaning, 
there will not be movement to a certain position, but rather a movement of a certain step size.
"""


import argparse
import sys
import termios
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_control"))

from excavator_control.pca9685 import PCA9685, PCAServo  # noqa: E402

MIN_ANGLE, MAX_ANGLE = -90.0, 90.0
STROKE_MM = 100.0
MAX_SPEED_MM_S = 8.0          # L16-100-150-6-R, no load
MM_PER_DEG = STROKE_MM / (MAX_ANGLE - MIN_ANGLE)

#: joint -> (extend key, retract key, default PCA9685 channel)
JOINTS = {
    "boom": ("u", "i", 0),
    "arm": ("j", "k", 1),
    "bucket": ("n", "m", 2),
}


def clamp(x):
    return max(MIN_ANGLE, min(MAX_ANGLE, x))


def read_key(settings):
    """One keypress, unbuffered. Same approach as teleop_keyboard.py."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def banner(channels, step, start):
    print("\n  Linear actuator teleop\n")
    for name, (up, down, _) in JOINTS.items():
        print(f"    {up} / {down}   {name:7} extend / retract   "
              f"(channel {channels[name]})")
    print("\n    [ / ]   smaller / larger step")
    print("    c       centre the joint last moved")
    print("    space   stop all actuators")
    print("    q       quit")
    print(f"\n  step {step:.0f} deg = {step * MM_PER_DEG:.1f} mm "
          f"(~{step * MM_PER_DEG / MAX_SPEED_MM_S:.1f}s of travel)")
    print(f"  assuming every joint starts at {start:+.0f} deg "
          f"({(start + 90) / 180 * STROKE_MM:.0f} mm)")
    print("\n  THE FIRST PRESS ON A JOINT MAY MOVE IT A LONG WAY if that")
    print("  assumption is wrong. Keep the bucket clear of the chassis.\n")


def status(angles, step, note=""):
    cells = "   ".join(
        f"{name} {angles[name]:+6.1f} ({(angles[name] + 90) / 180 * STROKE_MM:5.1f}mm)"
        for name in JOINTS
    )
    sys.stdout.write(f"\r  {cells}   step {step:4.1f}   {note:<28}")
    sys.stdout.flush()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.strip().split("\n")[1])
    p.add_argument("--step", type=float, default=5.0,
                   help="degrees per press (default 5 = 2.8 mm)")
    p.add_argument("--start", type=float, default=0.0,
                   help="angle to assume each joint currently sits at "
                        "(default 0 = mid stroke)")
    p.add_argument("--boom", type=int, default=JOINTS["boom"][2])
    p.add_argument("--arm", type=int, default=JOINTS["arm"][2])
    p.add_argument("--bucket", type=int, default=JOINTS["bucket"][2])
    p.add_argument("--bus", type=int, default=7)
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x40)
    args = p.parse_args(argv)

    channels = {"boom": args.boom, "arm": args.arm, "bucket": args.bucket}
    step = args.step
    # Commanded, not measured -- these are open loop and nothing reads the rod
    # back. This tracks what has been asked for and assumes the actuator got there.
    angles = {name: clamp(args.start) for name in JOINTS}
    keymap = {}
    for name, (up, down, _) in JOINTS.items():
        keymap[up] = (name, +1)
        keymap[down] = (name, -1)

    banner(channels, step, args.start)

    pca = PCA9685(bus=args.bus, address=args.address, frequency=50)
    servos = {name: PCAServo(pca, channels[name]) for name in JOINTS}
    settings = termios.tcgetattr(sys.stdin)
    last = None

    try:
        status(angles, step, "ready")
        while True:
            key = read_key(settings)

            if key in ("q", "\x03"):            # q or Ctrl-C
                break

            if key == " ":
                for servo in servos.values():
                    servo.detach()
                status(angles, step, "stopped")
                continue

            if key in ("[", "]"):
                step = max(0.5, min(45.0, step * (0.5 if key == "[" else 2.0)))
                status(angles, step, f"step {step:.1f} deg")
                continue

            if key == "c" and last is not None:
                angles[last] = 0.0
                servos[last].angle = 0.0
                status(angles, step, f"{last} -> centre")
                continue

            if key not in keymap:
                continue

            name, direction = keymap[key]
            target = clamp(angles[name] + direction * step)
            if target == angles[name]:
                # At a travel limit. Say so rather than silently doing nothing --
                # a dead key feels like a broken program.
                status(angles, step,
                       f"{name} at {'max' if direction > 0 else 'min'}")
                continue

            angles[name] = target
            servos[name].angle = target
            last = name
            status(angles, step,
                   f"{name} {'extend' if direction > 0 else 'retract'}")
    except Exception as exc:
        print(f"\n  error: {exc}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # Open loop: anything left driven keeps pushing. The L16 holds position
        # mechanically once released, so this parks the arm rather than dropping it.
        for servo in servos.values():
            servo.detach()
        pca.all_off()
        pca.close()
        print("\n\n  all channels released\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

