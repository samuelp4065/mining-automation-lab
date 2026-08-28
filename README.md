# Mining Automation Lab

Workspace for an automated mining-excavator arm: planar kinematics, a
desktop motion simulator, ArUco-marker vision on a Jetson Orin Nano, and the
serial/servo plumbing that drives the physical hardware.

The arm is modelled as a **3-link planar manipulator**:

| Link | Meaning | Joint angle |
| ---- | ------- | ----------- |
| `L1` | boom (shoulder → elbow)  | `theta1` — absolute, from +x axis |
| `L2` | stick (elbow → wrist)    | `theta2` — relative to boom |
| `L3` | bucket (wrist → tip)     | `theta3` — relative to stick |

A pose is specified as `(x, z, phi)`: bucket-tip position in the base frame plus
the tool orientation angle.

## Layout

| Directory | What it is |
| --------- | ---------- |
| `InverseKinematics/` | Closed-form IK/FK for the 3-link arm, plus angle → encoder conversion |
| `Simulator/` | Matplotlib simulators with sliders for interactive FK/IK exploration |
| `aruco_recognition/` | ROS workspace for ArUco marker detection with a RealSense camera |
| `CH341SER_LINUX/` | CH341 USB-serial kernel driver for the servo bus adapter |

### `InverseKinematics/`

- `IK.py` — `ik_3link_planar(x, z, phi, L1, L2, L3, elbow="down"|"up")`, returns
  `(theta1, theta2, theta3)` in radians and raises `ValueError` on an
  unreachable target.
- `IK_elbowUP.py` — elbow-up-only variant.
- `Conversion.py` — IK together with the matching forward kinematics, for
  round-trip checking.
- `normalize.py` — interactive prompt mapping joint angles (degrees) to the
  per-joint encoder counts used by the boom, stick, and bucket actuators.

### `Simulator/`

`sim.py`, `Sim2.py`, and `sim3.py` are successive revisions of a Matplotlib
tool that draws the arm and lets you drag `theta1/theta2/theta3` (FK) or type a
target pose (IK, `sim3.py`). Angles are handled in degrees at the UI and
converted internally.

```bash
python3 -m pip install numpy matplotlib
python3 Simulator/sim3.py
```

### `aruco_recognition/`

A catkin workspace containing `aruco`, `aruco_msgs`, `aruco_ros`, and the
RealSense ROS driver, used to locate fiducial markers in the dig area.

```bash
cd aruco_recognition
catkin_make
source devel/setup.bash
```

The upstream `realsense-ros` and `ddynamic_reconfigure` clones are not tracked
here — clone them into `aruco_recognition/src/` before building:

```bash
git clone https://github.com/IntelRealSense/realsense-ros.git aruco_recognition/src/realsense-ros
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git aruco_recognition/src/ddynamic_reconfigure
```

### `CH341SER_LINUX/`

Vendor kernel driver for the CH341 USB-to-serial chip, needed for the host to
talk to the servo bus.

```bash
cd CH341SER_LINUX/driver
make
sudo make load
```

## Servo library

Joint actuation uses the Lynxmotion Smart Servo (LSS) Python library, kept as a
separate upstream clone rather than vendored:

```bash
git clone https://github.com/Lynxmotion/LSS_Library_Python.git
```

## Not tracked

- `jetson-orin-nano-devkit-super-SD-image_JP6.2.1/` — the 23 GB JetPack 6.2.1 SD
  card image for the Jetson Orin Nano dev kit. Download it from NVIDIA rather
  than pulling it from git.
