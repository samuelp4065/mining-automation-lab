"""Interactive IK REPL: tip pose in, joint angles and actuator commands out.

Replaces the input loops that used to live in ``InverseKinematics/normalize.py``
and ``LSS_Library_Python/src/coordinate.py``. Unlike the latter, this touches no
hardware -- it imports neither gpiozero nor lss, so it runs anywhere and is safe
to use for checking a pose before committing the machine to it.

    ros2 run excavator_kinematics ik_repl
    python3 -m excavator_kinematics.ik_repl
"""

import numpy as np

from excavator_kinematics.calibration import JointLimitError, angles_to_commands
from excavator_kinematics.planar_ik import (
    DEFAULT_LINKS, fk_3link_planar, ik_3link_planar,
)


def solve_and_report(x, z, phi_deg, elbow="up"):
    """Return a printable report for one target pose."""
    phi = np.deg2rad(phi_deg)
    th1, th2, th3 = ik_3link_planar(x, z, phi, elbow=elbow)
    th_deg = np.rad2deg([th1, th2, th3])

    lines = [
        f"  joint angles (deg): boom {th_deg[0]:8.2f}  "
        f"stick {th_deg[1]:8.2f}  bucket {th_deg[2]:8.2f}"
    ]

    try:
        boom_cmd, stick_cmd, bucket_cmd = angles_to_commands(th1, th2, th3)
        lines.append(
            f"  servo commands:     boom {boom_cmd:8.2f}  "
            f"stick {stick_cmd:8.2f}  bucket {bucket_cmd:8.2f}"
        )
    except JointLimitError as exc:
        lines.append(f"  !! reachable but not commandable: {exc}")

    xt, zt, phit = fk_3link_planar(th1, th2, th3)
    lines.append(
        f"  FK check:           x {xt:7.2f}  z {zt:7.2f}  "
        f"phi {np.rad2deg(phit):7.2f}"
    )
    return "\n".join(lines)


def main():
    L1, L2, L3 = DEFAULT_LINKS.as_tuple()
    print(f"Link lengths (cm): L1={L1} L2={L2} L3={L3}   elbow-up")
    print("Enter 'x z phi_deg' in centimetres/degrees (phi optional), or 'exit'.")

    while True:
        try:
            command = input("target> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not command:
            continue
        if command.lower() in ("exit", "quit", "q"):
            break

        parts = command.replace(",", " ").split()
        if len(parts) not in (2, 3):
            print("  Enter 2 or 3 values: x z [phi_deg]")
            continue

        try:
            values = [float(p) for p in parts]
        except ValueError:
            print("  Values must be numeric.")
            continue

        x, z = values[0], values[1]
        phi_deg = values[2] if len(values) == 3 else 0.0

        try:
            print(solve_and_report(x, z, phi_deg))
        except ValueError as exc:
            print(f"  IK error: {exc}")


if __name__ == "__main__":
    main()
