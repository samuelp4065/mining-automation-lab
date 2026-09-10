"""Interactive Matplotlib simulator for the 3-link planar excavator arm.

Consolidates the former ``Simulator/sim.py``, ``Sim2.py`` and ``sim3.py``. The
IK/FK is no longer inlined here -- it imports :mod:`excavator_kinematics`, the
same solve the ROS nodes use, so the simulator can no longer drift away from
the machine's actual kinematics.

Two things changed from ``sim3.py``:

* **Centimetres throughout.** The old simulator used ~10 cm units (4.6, 2.0,
  1.2) while everything else used centimetres, so a target of ``6 1 0`` meant
  ``60 cm, 10 cm``. Enter real centimetres now: ``60 10 0``.
* **Actuator travel is shown.** A joint the IK can reach but the linear
  actuators cannot hold is drawn in red and named in the status line.

Run::

    python3 tools/simulator/arm_simulator.py      # after sourcing install/setup.bash
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider, TextBox

# Allow running straight from a source checkout without sourcing the workspace.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_kinematics"))

from excavator_kinematics.calibration import (  # noqa: E402
    JOINT_LIMITS_DEG, JointLimitError, check_joint_limits,
)
from excavator_kinematics.planar_ik import (  # noqa: E402
    DEFAULT_LINKS, ik_3link_planar, joint_points,
)

L1, L2, L3 = DEFAULT_LINKS.as_tuple()
REACH = L1 + L2 + L3
INITIAL_TARGET = "60 10 0"

# ---------- Figure ----------
fig, ax = plt.subplots(figsize=(7, 7))
plt.subplots_adjust(left=0.12, bottom=0.33)

ax.set_title("3-DOF Planar Excavator Simulator")
ax.set_xlabel("x (cm)")
ax.set_ylabel("z (cm)")
ax.grid(True)
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(-10, REACH + 10)
ax.set_ylim(-REACH * 0.4, REACH + 10)

ax.axhline(0.0, color="k", linewidth=1)  # ground

# Reach envelope, for context on what is geometrically possible.
_arc = np.linspace(-np.pi / 2, np.pi / 2, 200)
ax.plot(REACH * np.cos(_arc), REACH * np.sin(_arc), ":", color="0.7", linewidth=1)

theta1_0, theta2_0, theta3_0 = 40.0, -80.0, -100.0  # a pose inside actuator travel

link_line, = ax.plot([], [], "-o", linewidth=4, markersize=8, color="tab:blue")
status_text = ax.text(0.02, 0.97, "", transform=ax.transAxes,
                      va="top", ha="left", fontsize=9, family="monospace")

# ---------- Sliders ----------
ax_t1 = plt.axes([0.12, 0.21, 0.76, 0.03])
ax_t2 = plt.axes([0.12, 0.16, 0.76, 0.03])
ax_t3 = plt.axes([0.12, 0.11, 0.76, 0.03])

s_t1 = Slider(ax_t1, "θ1 (boom)", -90, 90, valinit=theta1_0, valstep=1)
s_t2 = Slider(ax_t2, "θ2 (arm)", -180, 180, valinit=theta2_0, valstep=1)
s_t3 = Slider(ax_t3, "θ3 (bucket)", -180, 180, valinit=theta3_0, valstep=1)


def update(_=None):
    t1, t2, t3 = s_t1.val, s_t2.val, s_t3.val
    pts = joint_points(*np.deg2rad([t1, t2, t3]))

    link_line.set_data([p[0] for p in pts], [p[1] for p in pts])

    try:
        check_joint_limits(t1, t2, t3)
        link_line.set_color("tab:blue")
        limit_note = "within actuator travel"
    except JointLimitError as exc:
        link_line.set_color("tab:red")
        limit_note = str(exc)

    tip = pts[-1]
    status_text.set_text(
        f"tip  x={tip[0]:7.2f}  z={tip[1]:7.2f}  phi={t1 + t2 + t3:7.2f}\n{limit_note}"
    )
    fig.canvas.draw_idle()


for _s in (s_t1, s_t2, s_t3):
    _s.on_changed(update)

# ---------- Target box ----------
ax_tb = plt.axes([0.12, 0.04, 0.5, 0.04])
text_box = TextBox(ax_tb, "Target (x z phi_deg): ", initial=INITIAL_TARGET)


def submit_target(text):
    """Parse 'x z [phi_deg]' in centimetres/degrees, solve, drive the sliders."""
    parts = text.replace(",", " ").split()
    if len(parts) < 2:
        print("Need at least x and z (centimetres).")
        return

    try:
        x, z = float(parts[0]), float(parts[1])
        phi_deg = float(parts[2]) if len(parts) >= 3 else 0.0
    except ValueError:
        print("Target values must be numeric.")
        return

    try:
        th = ik_3link_planar(x, z, np.deg2rad(phi_deg))
    except ValueError as exc:
        print(f"IK error: {exc}")
        status_text.set_text(f"unreachable: {x:.1f}, {z:.1f} @ {phi_deg:.1f} deg")
        fig.canvas.draw_idle()
        return

    th1, th2, th3 = np.rad2deg(th)
    s_t1.set_val(th1)
    s_t2.set_val(th2)
    s_t3.set_val(th3)


text_box.on_submit(submit_target)

# ---------- Reset ----------
btn_reset = Button(plt.axes([0.75, 0.04, 0.13, 0.04]), "Reset")


def reset(_event):
    s_t1.reset()
    s_t2.reset()
    s_t3.reset()
    text_box.set_val(INITIAL_TARGET)


btn_reset.on_clicked(reset)


def main():
    update()
    print(f"Link lengths (cm): L1={L1} L2={L2} L3={L3}")
    print("Actuator travel (deg): " + ", ".join(
        f"{k} [{v[0]:.0f}, {v[1]:.0f}]" for k, v in JOINT_LIMITS_DEG.items()))
    plt.show()


if __name__ == "__main__":
    main()
