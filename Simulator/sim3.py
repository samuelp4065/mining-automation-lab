import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox

# ---------- Helpers ----------
def cosd(t): return np.cos(np.deg2rad(t))
def sind(t): return np.sin(np.deg2rad(t))

def forward(theta1, theta2, theta3, L1, L2, L3):
    """Planar 3-link FK. Returns joint points and total angle (deg)."""
    x0, y0 = 0.0, 0.0

    x1 = x0 + L1 * cosd(theta1)
    y1 = y0 + L1 * sind(theta1)

    x2 = x1 + L2 * cosd(theta1 + theta2)
    y2 = y1 + L2 * sind(theta1 + theta2)

    total = theta1 + theta2 + theta3
    x3 = x2 + L3 * cosd(total)
    y3 = y2 + L3 * sind(total)

    return (x0,y0), (x1,y1), (x2,y2), (x3,y3), total

# ---------- IK in radians ----------
def ik_3link_planar(x, z, phi, L1, L2, L3):
    """
    Inverse kinematics for a 3-link planar arm (radians).
    Elbow-up solution for theta2.
    """
    # wrist position (remove L3 along phi)
    xw = x - L3 * np.cos(phi)
    zw = z - L3 * np.sin(phi)

    rw2 = xw**2 + zw**2
    rw = np.sqrt(rw2)

    if rw > (L1 + L2) or rw < abs(L1 - L2):
        raise ValueError("Target unreachable for given L1, L2, L3.")

    c2 = (rw2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = np.clip(c2, -1.0, 1.0)
    s2 = -np.sqrt(1 - c2**2)    # elbow-up
    theta2 = np.arctan2(s2, c2)

    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = np.arctan2(zw, xw) - np.arctan2(k2, k1)

    theta3 = phi - (theta1 + theta2)
    return theta1, theta2, theta3

# ---------- Parameters ----------
L1, L2, L3 = 4.6, 2.0, 1.2  # boom, arm, bucket lengths

# ---------- Figure ----------
fig, ax = plt.subplots(figsize=(7,7))
plt.subplots_adjust(left=0.12, bottom=0.33)  # extra space for textbox

ax.set_title("3-DOF Planar Excavator Simulator (units ~ 10 cm)")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True)
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(0, 9)
ax.set_ylim(0, 9)

# ground
ax.plot([0, 9], [0, 0], 'k-')

# initial angles
theta1_0, theta2_0, theta3_0 = 0.0, 0.0, 0.0

# initial draw objects
(p0, p1, p2, p3, total) = forward(theta1_0, theta2_0, theta3_0, L1, L2, L3)
xs = [p0[0], p1[0], p2[0], p3[0]]
ys = [p0[1], p1[1], p2[1], p3[1]]

link_line, = ax.plot(xs, ys, "-o", linewidth=4, markersize=8)

# ---------- Sliders ----------
ax_t1 = plt.axes([0.12, 0.21, 0.76, 0.03])
ax_t2 = plt.axes([0.12, 0.16, 0.76, 0.03])
ax_t3 = plt.axes([0.12, 0.11, 0.76, 0.03])

s_t1 = Slider(ax_t1, "θ1 (boom)",   -90,   90, valinit=theta1_0, valstep=1)
s_t2 = Slider(ax_t2, "θ2 (arm)",   -180,  180, valinit=theta2_0, valstep=1)
s_t3 = Slider(ax_t3, "θ3 (bucket)",-180,  180, valinit=theta3_0, valstep=1)

# ---------- Update ----------
def update(_=None):
    t1, t2, t3 = s_t1.val, s_t2.val, s_t3.val
    (p0, p1, p2, p3, total) = forward(t1, t2, t3, L1, L2, L3)

    xs = [p0[0], p1[0], p2[0], p3[0]]
    ys = [p0[1], p1[1], p2[1], p3[1]]
    link_line.set_data(xs, ys)

    fig.canvas.draw_idle()

s_t1.on_changed(update)
s_t2.on_changed(update)
s_t3.on_changed(update)

# ---------- Coordinate TextBox (x z phi_deg) ----------
ax_tb = plt.axes([0.12, 0.04, 0.5, 0.04])
text_box = TextBox(ax_tb, "Target (x z phi_deg): ", initial="6 1 0")

def submit_target(text):
    """
    Parse 'x z phi_deg' from the textbox,
    run IK, and update sliders if reachable.
    """
    try:
        parts = text.replace(",", " ").split()
        if len(parts) < 2:
            print("Need at least x and z.")
            return

        x = float(parts[0])
        z = float(parts[1])
        if len(parts) >= 3:
            phi_deg = float(parts[2])
        else:
            phi_deg = 0.0  # default orientation

        phi = np.deg2rad(phi_deg)

        th1, th2, th3 = ik_3link_planar(x, z, phi, L1, L2, L3)

        # convert to deg and move sliders
        s_t1.set_val(np.rad2deg(th1))
        s_t2.set_val(np.rad2deg(th2))
        s_t3.set_val(np.rad2deg(th3))

    except ValueError as e:
        print(f"IK / parse error: {e}")

text_box.on_submit(submit_target)

# ---------- Reset Button ----------
reset_ax = plt.axes([0.75, 0.04, 0.13, 0.04])
btn_reset = Button(reset_ax, "Reset")

def reset(event):
    s_t1.reset()
    s_t2.reset()
    s_t3.reset()
    text_box.set_val("6 1 0")

btn_reset.on_clicked(reset)

update()
plt.show()
