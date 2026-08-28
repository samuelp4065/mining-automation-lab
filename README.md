# Mining Automation Lab

Working repository for an **automated mining-excavator arm**. It brings together
the four pieces needed to take a target dig point and turn it into servo motion:

1. **Kinematics** — closed-form inverse/forward kinematics for a 3-link planar arm.
2. **Simulation** — Matplotlib tools to check reachability and poses before touching hardware.
3. **Vision** — an ArUco-marker ROS workspace running on a Jetson Orin Nano with a RealSense camera, to locate the dig target.
4. **Hardware plumbing** — the CH341 USB-serial kernel driver and the LSS smart-servo library that drive the joints.

---

## Table of contents

- [The arm model](#the-arm-model)
- [Pipeline](#pipeline)
- [Repository layout](#repository-layout)
- [Quick start](#quick-start)
- [`InverseKinematics/`](#inversekinematics)
- [`Simulator/`](#simulator)
- [`aruco_recognition/`](#aruco_recognition)
- [`CH341SER_LINUX/`](#ch341ser_linux)
- [Servo library](#servo-library-lss)
- [Units — read this before comparing numbers](#units--read-this-before-comparing-numbers)
- [What is not tracked here](#what-is-not-tracked-here)
- [Troubleshooting](#troubleshooting)

---

## The arm model

The excavator is modelled as a **3-link planar (3-DOF) manipulator** moving in the
vertical `x`–`z` plane. Slew/rotation about the base is out of scope for these modules.

| Link | Physical part | Length | Joint angle | Convention |
| ---- | ------------- | ------ | ----------- | ---------- |
| `L1` | Boom (shoulder → elbow) | 46 cm | `theta1` | **Absolute**, measured from the `+x` axis |
| `L2` | Stick / arm (elbow → wrist) | 20 cm | `theta2` | **Relative** to the boom |
| `L3` | Bucket / tool (wrist → tip) | 12 cm | `theta3` | **Relative** to the stick |

A pose is given as `(x, z, phi)`:

- `x`, `z` — bucket-**tip** position in the base frame (base joint at the origin, ground at `z = 0`)
- `phi` — tool orientation, the absolute angle of the bucket direction from the `+x` axis.
  `phi = 0` points the tool horizontally forward; negative `phi` tilts it down.

The three joint angles compose to the tool angle, which is what makes the arm solvable
in closed form:

```
phi = theta1 + theta2 + theta3
```

### How the IK is solved

Because `phi` is fully determined by the joint sum, the problem decouples into a
2-link solve plus a single subtraction:

1. **Back off the tool.** Since the tip sits `L3` along the tool direction from the
   wrist, the wrist position is known directly:
   `xw = x − L3·cos(phi)`, `zw = z − L3·sin(phi)`.
2. **Solve boom + stick as a 2-link arm** reaching that wrist point.
   With `rw = hypot(xw, zw)`, the law of cosines gives the elbow angle:
   `cos(theta2) = (rw² − L1² − L2²) / (2·L1·L2)`, then
   `theta1 = atan2(zw, xw) − atan2(L2·sin(theta2), L1 + L2·cos(theta2))`.
3. **Set the bucket** to make up the difference: `theta3 = phi − (theta1 + theta2)`.

**Two solutions.** Step 2 has a `±` on `sin(theta2)`, giving *elbow-down* (`+`, the
default) and *elbow-up* (`−`) configurations that reach the same tip pose through
different arm shapes. `IK.py` exposes both via `elbow="down"|"up"`.

**Reachability.** The wrist must lie inside the 2-link annulus, so
`|L1 − L2| ≤ rw ≤ L1 + L2`. Outside that band the solver raises
`ValueError("Target unreachable for given L1, L2, L3.")` — note the check is on the
*wrist*, so a tip pose can be unreachable purely because of an aggressive `phi`.
`cos(theta2)` is clipped to `[-1, 1]` before the `arccos` to absorb float error at
the workspace boundary.

## Pipeline

```
 RealSense camera ──▶ aruco_ros ──▶ marker pose (x, z) in the dig plane
                                          │
                                          ▼
                        ik_3link_planar(x, z, phi, L1, L2, L3)
                                          │
                                          ▼
                           theta1, theta2, theta3  (radians)
                                          │
                          normalize.py: angle → encoder counts
                                          │
                                          ▼
              LSS servo library ──▶ CH341 USB-serial ──▶ boom / stick / bucket
```

## Repository layout

| Directory | Language | What it is |
| --------- | -------- | ---------- |
| `InverseKinematics/` | Python | Closed-form IK/FK and the angle → encoder calibration |
| `Simulator/` | Python | Interactive Matplotlib arm simulators (FK sliders + IK target entry) |
| `aruco_recognition/` | C++ / ROS 1 | Catkin workspace for ArUco marker detection |
| `CH341SER_LINUX/` | C | CH341 USB-to-serial kernel driver for the servo bus adapter |

## Quick start

```bash
git clone https://github.com/samuelp4065/mining-automation-lab.git
cd mining-automation-lab

# Kinematics + simulator only — no hardware needed
python3 -m pip install numpy matplotlib
python3 InverseKinematics/IK.py     # prints an IK solve with an FK round-trip check
python3 Simulator/sim3.py           # interactive simulator
```

---

## `InverseKinematics/`

| File | Purpose |
| ---- | ------- |
| `IK.py` | Reference implementation. Both elbow configurations, plus FK for verification. |
| `IK_elbowUP.py` | Elbow-up-only variant. |
| `Conversion.py` | IK paired with its forward kinematics for round-trip checks. |
| `normalize.py` | Interactive angle → encoder-count converter for the three actuators. |

### `IK.py`

```python
ik_3link_planar(x, z, phi, L1, L2, L3, elbow="down") -> (theta1, theta2, theta3)
fk_3link_planar(theta1, theta2, theta3, L1, L2, L3)  -> (x, z, phi)
```

Angles are in **radians** in and out; `elbow` selects the configuration; a target
outside the workspace raises `ValueError`.

```python
import numpy as np
from IK import ik_3link_planar, fk_3link_planar

L1, L2, L3 = 46.0, 20.0, 12.0        # cm
x, z, phi = 60, 10, np.deg2rad(-30)  # tip pose, tool angled 30° down

th1, th2, th3 = ik_3link_planar(x, z, phi, L1, L2, L3, elbow="down")
print(np.rad2deg([th1, th2, th3]))

# Round-trip: should return (60, 10, -30)
print(fk_3link_planar(th1, th2, th3, L1, L2, L3))
```

Running the file directly (`python3 InverseKinematics/IK.py`) solves that example
for both elbow configurations and prints the FK check for each — a fast way to
confirm a change to the solver didn't break it.

### `normalize.py` — angle to encoder counts

Each joint has its own linear calibration mapping a joint angle in **degrees** to the
actuator's encoder count. The offsets absorb the mechanical zero of each joint; the
negative slopes on the stick and bucket mean those actuators count **backwards**
relative to the angle convention.

| Joint | Function | Mapping (`angle` in degrees) | Slope (counts/deg) |
| ----- | -------- | ---------------------------- | ------------------ |
| Boom | `angleToEncoder1` | `(θ₁ − 40) × 9/2` | `+4.5` |
| Stick | `angleToEncoder2` | `(θ₂ + 79) × (−90/41)` | `≈ −2.195` |
| Bucket | `angleToEncoder3` | `(θ₃ + 385/3) × (−27/25)` | `−1.08` |

Results are truncated to `int`, so expect sub-count rounding. To go the other way:

```
θ₁ = enc × (2/9)   + 40
θ₂ = enc × (−41/90) − 79
θ₃ = enc × (−25/27) − 385/3
```

Run it as a REPL — enter three space-separated angles, get three encoder values,
`exit` to quit:

```
$ python3 InverseKinematics/normalize.py
Enter angles (th1 th2 th3) in degrees, separated by spaces (or 'exit' to quit): 45 -30 10
Encoder values:
  Boom   → 22
  Stick  → -107
  Bucket → -149
```

> If you re-zero a joint or change a linkage, these three constants are the only
> place that needs updating.

---

## `Simulator/`

Three successive revisions of the same Matplotlib tool. All of them draw the arm
over a ground line on an equal-aspect plot and work in **degrees** at the UI,
converting to radians internally.

| File | What it adds |
| ---- | ------------ |
| `sim.py` | FK sliders for `θ1/θ2/θ3` + Reset. Contains commented-out gripper/ray overlays. |
| `Sim2.py` | Same as `sim.py` with the ray-overlay experiment stripped out. |
| `sim3.py` | **Use this one.** Adds a target text box that runs IK and drives the sliders. |

`sim3.py` controls:

- **Sliders** — `θ1 (boom)` over `[−90, 90]`, `θ2 (arm)` and `θ3 (bucket)` over
  `[−180, 180]`, in 1° steps. Dragging any one re-draws via forward kinematics.
- **Target box** — type `x z phi_deg` (commas or spaces, e.g. `6 1 0`). It runs the
  IK, then *sets the sliders* to the solution, so the arm snaps to the pose and the
  joint angles are readable off the sliders. `phi_deg` is optional and defaults to `0`.
  An unreachable target or malformed input prints the error to the terminal and
  leaves the arm where it was.
- **Reset** — returns all joints to `0°` and the target box to `6 1 0`.

```bash
python3 -m pip install numpy matplotlib
python3 Simulator/sim3.py
```

The simulators use elbow-up only and hard-code `L1, L2, L3 = 4.6, 2.0, 1.2` at the
top of the file — see [Units](#units--read-this-before-comparing-numbers).

---

## `aruco_recognition/`

A **ROS 1 catkin workspace** that detects ArUco fiducial markers and publishes their
poses, used to locate the dig target and reference frames in the work area.

Tracked packages:

| Package | Role |
| ------- | ---- |
| `aruco` | The core ArUco detection library (marker detector, board detector, camera parameters). |
| `aruco_msgs` | Message definitions: `Marker.msg`, `MarkerArray.msg`. |
| `aruco_ros` | ROS nodes wrapping the library: `single`, `double`, `marker_publisher`. |

Two upstream dependencies are **not** tracked here (they carry their own git
history) — clone them into `src/` before building:

```bash
cd aruco_recognition/src
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
```

Build and source the workspace:

```bash
cd aruco_recognition
catkin_make
source devel/setup.bash
```

### Nodes and launch files

| Node | Launch file | What it does |
| ---- | ----------- | ------------ |
| `single` | `single.launch`, `single_usb.launch` | Tracks one marker by ID and publishes its pose + TF frame. |
| `double` | `double.launch` | Tracks two markers (e.g. tool frame and object frame) at once. |
| `marker_publisher` | `marker_publisher.launch` | Detects *all* visible markers and publishes an `aruco_msgs/MarkerArray`. |

Every node subscribes to `/image` and `/camera_info`, which the launch files remap
onto the actual camera topics — point those remaps at your RealSense stream. Key
parameters:

- `marker_size` — marker side length **in metres** (the shipped defaults are `0.04`–`0.05` m).
- `marker_id`, `marker_id1`, `marker_id2` — which marker IDs to track.
- `image_is_rectified` — set `True` when subscribing to a rectified image topic.
- `ref_frame`, `parent_name`, `child_name1/2` — TF frame naming for the published poses.
- `normalizeImage`, `dct_components_to_remove` — DCT-based lighting normalization,
  worth enabling under uneven pit lighting.

Printable calibration markers at known sizes are in `aruco_ros/etc/`
(e.g. `marker26_5cm.jpg`, `marker582_5cm_margin_2cm.jpg`). **Print at 100 % scale**
and measure the printed marker — if the physical size doesn't match `marker_size`,
the pose range will be wrong by exactly that ratio.

---

## `CH341SER_LINUX/`

Vendor kernel driver for the **CH341 USB-to-serial** chip, the adapter between the
host and the servo bus. Needed on hosts where the in-tree `ch341` module is absent
or misbehaves.

```bash
cd CH341SER_LINUX/driver
make
sudo make load          # insert the module
sudo make unload        # remove it
```

Build output (`*.o`, `*.ko`, `Module.symvers`, …) is gitignored. Building requires
kernel headers matching the running kernel — on the Jetson that means the JetPack
kernel sources, not the generic `linux-headers` package. After loading, the adapter
appears as `/dev/ttyCH341USB0` or `/dev/ttyUSB0`.

---

## Servo library (LSS)

Joint actuation uses the **Lynxmotion Smart Servo (LSS)** Python library. It is kept
as a separate upstream clone rather than vendored, so it is not tracked in this repo:

```bash
git clone https://github.com/Lynxmotion/LSS_Library_Python.git
```

Its `src/` ships useful references — `lss.py` (the servo class), `lss_const.py`
(protocol constants), and examples for sweeping, querying, and one-time configuration.

---

## Units — read this before comparing numbers

The modules do **not** share a unit scale, which is the easiest way to get confused
when moving numbers between them:

| Context | `L1, L2, L3` | Scale |
| ------- | ------------ | ----- |
| `InverseKinematics/IK.py`, `Conversion.py` | `46.0, 20.0, 12.0` | centimetres |
| `Simulator/*.py` | `4.6, 2.0, 1.2` | ~10 cm per unit (the plot title says so) |

Same arm, ratio `46 : 20 : 12` either way — only the scale differs. So a simulator
target of `6 1 0` is `60 cm, 10 cm, 0°` in the kinematics modules. The IK is scale
invariant: as long as the target and the link lengths use the same unit, the angles
come out identical.

Angle conventions worth repeating: the solver functions take and return **radians**,
the simulator UI and `normalize.py` both work in **degrees**.

## What is not tracked here

Deliberately excluded via `.gitignore`, with recovery instructions above:

| Excluded | Why | How to get it |
| -------- | --- | ------------- |
| `jetson-orin-nano-devkit-super-SD-image_JP6.2.1/` | 23 GB SD-card image, far past any git size limit | Download JetPack 6.2.1 for the Orin Nano dev kit from NVIDIA |
| `LSS_Library_Python/` | Third-party clone with its own history | `git clone` — see [Servo library](#servo-library-lss) |
| `aruco_recognition/src/realsense-ros/` | Third-party clone (~103 MB) | `git clone` — see [`aruco_recognition/`](#aruco_recognition) |
| `aruco_recognition/src/ddynamic_reconfigure/` | Third-party clone | `git clone` — see [`aruco_recognition/`](#aruco_recognition) |

Also ignored: catkin build output (`build/`, `devel/`, `install/`, `*.bag`), Python
caches and virtualenvs, kernel module build artifacts, and editor/OS junk.

## Troubleshooting

| Symptom | Likely cause |
| ------- | ------------ |
| `ValueError: Target unreachable for given L1, L2, L3.` | The *wrist* point falls outside `[\|L1−L2\|, L1+L2]`. Often the tip is fine but `phi` pushes the wrist out — try a less aggressive tool angle before assuming the target is too far. |
| Simulator target does nothing, error printed to terminal | `sim3.py` reports IK and parse failures on stdout, not in the GUI. Check the terminal you launched it from. |
| IK angles look sane, arm moves to the wrong place | Unit mismatch (cm vs. simulator units), or a stale constant in `normalize.py` after re-zeroing a joint. |
| Arm moves the wrong direction on one joint | The stick and bucket encoder mappings have **negative** slopes; a sign flip there inverts exactly one joint. |
| `catkin_make` fails on a missing package | The two untracked upstream clones aren't in `src/` yet. |
| No `/dev/ttyUSB*` after plugging in the adapter | CH341 module not loaded (`sudo make load`), or your user isn't in the `dialout` group. |
| ArUco poses off by a constant scale factor | `marker_size` doesn't match the physically printed marker — printers rescale by default. |
