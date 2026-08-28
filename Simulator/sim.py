import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# ---------- Helpers ----------
def cosd(t): return np.cos(np.deg2rad(t))
def sind(t): return np.sin(np.deg2rad(t))

def forward(theta1, theta2, theta3, L1, L2, L3):
    """Planar 3-link FK. Returns joint points and total angle."""
    x0, y0 = 0.0, 0.0

    x1 = x0 + L1 * cosd(theta1)
    y1 = y0 + L1 * sind(theta1)

    x2 = x1 + L2 * cosd(theta1 + theta2)
    y2 = y1 + L2 * sind(theta1 + theta2)

    total = theta1 + theta2 + theta3
    x3 = x2 + L3 * cosd(total)
    y3 = y2 + L3 * sind(total)

    return (x0,y0), (x1,y1), (x2,y2), (x3,y3), total

# ---------- Parameters ----------
L1, L2, L3 = 4.6, 2.0, 1.2  # boom, arm, bucket lengths

# ---------- Figure ----------
fig, ax = plt.subplots(figsize=(7,7))
plt.subplots_adjust(left=0.12, bottom=0.27)

ax.set_title("3-DOF Planar Excavator Simulator Unit: x10 cm")
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

# # gripper / rays
# grip_line, = ax.plot([], [], "r-", linewidth=3)
# ray1, = ax.plot([], [], "r-", linewidth=2)
# ray2, = ax.plot([], [], "r-", linewidth=2)

# ---------- Sliders ----------
ax_t1 = plt.axes([0.12, 0.17, 0.76, 0.03])
ax_t2 = plt.axes([0.12, 0.12, 0.76, 0.03])
ax_t3 = plt.axes([0.12, 0.07, 0.76, 0.03])

s_t1 = Slider(ax_t1, "θ1 (boom)",  -90,  90, valinit=theta1_0, valstep=1)
s_t2 = Slider(ax_t2, "θ2 (arm)",   -180,  180, valinit=theta2_0, valstep=1)
s_t3 = Slider(ax_t3, "θ3 (bucket)",-180,  180, valinit=theta3_0, valstep=1)

# ---------- Update ----------
def update(_=None):
    t1, t2, t3 = s_t1.val, s_t2.val, s_t3.val
    (p0, p1, p2, p3, total) = forward(t1, t2, t3, L1, L2, L3)

    xs = [p0[0], p1[0], p2[0], p3[0]]
    ys = [p0[1], p1[1], p2[1], p3[1]]
    link_line.set_data(xs, ys)

    # bucket direction unit vector
    u = np.array([cosd(total), sind(total)])
    p = np.array([-u[1], u[0]])  # perpendicular

    # gripper segment centered at p2
    grip_half = 0.3
    A = np.array([p2[0], p2[1]]) + grip_half * p
    B = np.array([p2[0], p2[1]]) - grip_half * p
    # grip_line.set_data([A[0], B[0]], [A[1], B[1]])

    # rays forward along bucket direction
    # ray_len = 2.5
    # ray1.set_data([A[0], A[0] + ray_len*u[0]], [A[1], A[1] + ray_len*u[1]])
    # ray2.set_data([B[0], B[0] + ray_len*u[0]], [B[1], B[1] + ray_len*u[1]])

    fig.canvas.draw_idle()

s_t1.on_changed(update)
s_t2.on_changed(update)
s_t3.on_changed(update)

# ---------- Reset Button ----------
reset_ax = plt.axes([0.8, 0.01, 0.1, 0.04])
btn_reset = Button(reset_ax, "Reset")

def reset(event):
    s_t1.reset()
    s_t2.reset()
    s_t3.reset()

btn_reset.on_clicked(reset)

update()
plt.show()
