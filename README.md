# Mining Automation Lab

Working repository for an **automated mining excavator** on a Jetson Orin Nano.
A RealSense D435i locates the dig target, closed-form inverse kinematics turns that
target into joint angles, and LSS smart servos plus RC linear actuators execute the
dig cycle.

This is a **ROS 2 Humble workspace** holding all first-party code. It builds as an
overlay on `~/ros2_ws`, which carries only third-party sources (librealsense,
realsense-ros, rtabmap, rtabmap_ros), so rebuilding this repo never triggers a
multi-hour third-party compile.

---

## Table of contents

- [Layout](#layout)
- [Quick start](#quick-start)
- [The arm model](#the-arm-model)
- [Pipeline](#pipeline)
- [Calibration](#calibration)
- [Hardware topology](#hardware-topology)
- [Navigation and obstacle avoidance](#navigation-and-obstacle-avoidance)
- [Before it can drive](#before-it-can-drive)
- [Reference scripts](#reference-scripts)
- [What is not tracked here](#what-is-not-tracked-here)
- [Troubleshooting](#troubleshooting)

---

## Layout

| Path | What it is |
| ---- | ---------- |
| `src/excavator_kinematics/` | **The** IK/FK and joint calibration. Pure Python — no ROS, no hardware. 32 tests. |
| `src/excavator_control/` | LSS bus, track drive, swingdrive, linear actuators. `scripts/` holds pre-ROS reference scripts. |
| `src/excavator_perception/` | ArUco detection; depth-based dig-target extraction. |
| `src/excavator_description/` | URDF/xacro and the TF tree. **Contains unmeasured placeholders** — see [Before it can drive](#before-it-can-drive). |
| `src/excavator_bringup/` | Launch files, nav2 parameters, EKF configuration. |
| `tools/simulator/` | Interactive Matplotlib arm simulator. |
| `tools/drivers/CH341SER_LINUX/` | CH341 USB-to-serial kernel driver for the servo bus adapter. |
| `upstream.repos` | vcstool manifest reconstructing the third-party overlay. |

## Quick start

```bash
git clone https://github.com/samuelp4065/mining-automation-lab.git
cd mining-automation-lab

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash     # third-party overlay
colcon build --symlink-install
source install/setup.bash
```

No hardware required:

```bash
ros2 run excavator_kinematics ik_repl        # pose → angles → servo commands
python3 tools/simulator/arm_simulator.py     # interactive arm simulator
ros2 launch excavator_description description.launch.py joint_mode:=simulate
```

On the machine:

```bash
ros2 launch excavator_bringup mapping.launch.py           # ONE TIME: build the pit map
ros2 run nav2_map_server map_saver_cli -f ~/maps/pit_map

ros2 launch excavator_bringup navigation.launch.py map:=~/maps/pit_map.yaml
ros2 run excavator_control motor_node                     # track drive from /cmd_vel
```

Tests need neither ROS nor hardware:

```bash
cd src/excavator_kinematics && python3 -m pytest test/ -q
```

On a fresh machine, rebuild the third-party overlay first:

```bash
mkdir -p ~/ros2_ws/src && vcs import ~/ros2_ws/src < upstream.repos
cd ~/ros2_ws && colcon build --symlink-install
```

---

## The arm model

The excavator's boom, stick and bucket form a **3-link planar (3-DOF) manipulator**
in the vertical `x`–`z` plane of the cab. Cab rotation is a separate DOF driven by the
swingdrive and is not part of this solve.

| Link | Physical part | Length | Joint angle | Convention |
| ---- | ------------- | ------ | ----------- | ---------- |
| `L1` | Boom (shoulder → elbow) | 46 cm | `theta1` | **Absolute**, from the `+x` axis |
| `L2` | Stick / arm (elbow → wrist) | 20 cm | `theta2` | **Relative** to the boom |
| `L3` | Bucket / tool (wrist → tip) | 12 cm | `theta3` | **Relative** to the stick |

A pose is `(x, z, phi)`:

- `x`, `z` — bucket-**tip** position in the base frame, centimetres (base joint at the
  origin, ground at `z = 0`)
- `phi` — tool orientation, the absolute angle of the bucket direction from `+x`.
  `phi = 0` points the tool horizontally forward; negative `phi` tilts it down.

The joint angles compose to the tool angle, which is what makes the arm solvable in
closed form:

```
phi = theta1 + theta2 + theta3
```

### How the IK is solved

Because `phi` is fully determined by the joint sum, the problem decouples into a
2-link solve plus a single subtraction:

1. **Back off the tool.** The tip sits `L3` along the tool direction from the wrist, so
   the wrist is known directly: `xw = x − L3·cos(phi)`, `zw = z − L3·sin(phi)`.
2. **Solve boom + stick as a 2-link arm** reaching that wrist. With `rw = hypot(xw, zw)`,
   the law of cosines gives the elbow angle:
   `cos(theta2) = (rw² − L1² − L2²) / (2·L1·L2)`, then
   `theta1 = atan2(zw, xw) − atan2(L2·sin(theta2), L1 + L2·cos(theta2))`.
3. **Set the bucket** to make up the difference: `theta3 = phi − (theta1 + theta2)`.

**Two solutions.** Step 2 has a `±` on `sin(theta2)`: *elbow-up* (`−`) and *elbow-down*
(`+`). **Elbow-up is the default**, because every piece of hardware code on this machine
has always used it. The retired `InverseKinematics/IK.py` defaulted to `"down"` and was
the odd one out; it never drove the arm.

**Reachability.** The wrist must lie in the 2-link annulus, `|L1 − L2| ≤ rw ≤ L1 + L2`.
Outside it the solver raises `ValueError("Target unreachable for given L1, L2, L3.")`.
The check is on the **wrist**, so a tip pose well inside the workspace can still be
unreachable purely because an aggressive `phi` swings the wrist out — try a shallower
tool angle before concluding the target is too far. `cos(theta2)` is clipped to
`[-1, 1]` to absorb float error at the workspace boundary.

### API

```python
from excavator_kinematics import ik_3link_planar, fk_3link_planar, angles_to_commands
import numpy as np

th1, th2, th3 = ik_3link_planar(60, 10, np.deg2rad(-30))   # cm, radians, elbow-up
print(fk_3link_planar(th1, th2, th3))                      # → (60.0, 10.0, -0.5236)
print(angles_to_commands(th1, th2, th3))                   # → AngularServo commands
```

Link lengths default to this machine's measured values and can be overridden per call.
Angles are **radians** in and out; degrees appear only at UI boundaries.

## Pipeline

```
 track encoders ──┐
                  ├──▶ EKF ──▶ /odometry/filtered ──┐
 D435i IMU ───────┘                                 │
                                                    ▼
 RealSense D435i ──▶ depth → /scan ──▶ AMCL ──▶ nav2 ──▶ tracks   [ navigation ]
                  └▶ point cloud ──▶ height-banded costmap ──┘

 RealSense D435i ──▶ live aligned depth ──▶ transform to arm base frame
                                                   │
                                                   ▼
                                    plane slice → (x, z) dig target
                                                   │
                                                   ▼
                              ik_3link_planar(x, z, phi)  → theta1..3
                                                   │
                                 check_joint_limits + angles_to_commands
                                                   │
                                                   ▼
                          gpiozero PWM ──▶ boom / stick / bucket actuators
```

Navigation and digging stay separate on purpose: **drive from the map, dig from the
live frame.** The occupancy grid is downsampled and accumulates drift, so it is the
wrong source for a bucket target even though it is the right source for planning a
route.

## Calibration

`excavator_kinematics/calibration.py` holds the per-joint affine maps. They convert a
joint angle in **degrees** to the value written to `AngularServo.angle`, which gpiozero
clamps to **[-90, 90]**:

| Joint | Function | Mapping | Slope |
| ----- | -------- | ------- | ----- |
| Boom | `angle_to_boom_cmd` | `(θ₁ − 40) × 9/2` | `+4.5` |
| Stick | `angle_to_stick_cmd` | `(θ₂ + 79) × (−90/41)` | `≈ −2.195` |
| Bucket | `angle_to_bucket_cmd` | `(θ₃ + 385/3) × (−27/25)` | `−1.08` |

> These are **not** LSS encoder counts, despite earlier revisions of this document
> saying so. The LSS bus drives only the tracks and swingdrive; boom, stick and bucket
> are PWM linear actuators and never touch it.

The negative slopes on stick and bucket mean those actuators travel backwards relative
to the angle convention — a sign error in one inverts exactly one joint.

Because the command saturates at ±90, each joint has an implied mechanical travel limit,
enforced by `check_joint_limits()` before anything is commanded:

| Joint | Travel (deg) |
| ----- | ------------ |
| Boom | `[20.0, 60.0]` |
| Stick | `[-120.0, -38.0]` |
| Bucket | `[-211.7, -45.0]` |

**Open discrepancy.** Boom and stick travel agrees with the IK angle convention — about
61–64 % of sampled workspace poses land inside travel, which is what you would expect
from actuators that bound the reachable set. The bucket does not: only **12.3 %**, and
its IK range barely overlaps its travel band. Either that constant was derived against a
different zero or its convention differs. This needs one measurement on the machine.
Do not resolve it by loosening the limit check — that check is currently the only thing
preventing silent saturation against the end stops.

If you re-zero a joint or change a linkage, `calibration.py` is the only place to edit.

## Hardware topology

**LSS smart servos** — one shared serial bus on `/dev/ttyUSB0`, addressed by ID:

| ID | Role | Control |
| -- | ---- | ------- |
| 0 | Left track | `wheelRPM()` |
| 1 | Right track | `wheelRPM()` |
| 2 | Swingdrive (cab rotation) | `move()` to absolute position, `getPosition()` |

**RC linear actuators** — PWM via `gpiozero.AngularServo`, open-loop, no feedback:

| BCM pin | Joint |
| ------- | ----- |
| 17 | Boom |
| 27 | Stick / arm |
| 22 | Bucket |

`frame_width=0.0021` on every `AngularServo` is deliberate: a smaller frame width gives
finer control but less torque. Because the actuators are open-loop, `stop_all()` calls
`.detach()` rather than commanding a position, and scripts wrap their main loop in
`try/finally` to guarantee detach on exit — leaving actuators driven on exit stalls them
against their end stops.

### CH341 driver

Vendor kernel driver for the CH341 USB-to-serial chip, needed where the in-tree module
is absent or misbehaves:

```bash
cd tools/drivers/CH341SER_LINUX/driver
make
sudo make load        # sudo make unload to remove
```

Building requires kernel headers matching the running kernel — on the Jetson that means
JetPack kernel sources, not the generic `linux-headers` package. After loading, the
adapter appears as `/dev/ttyCH341USB0` or `/dev/ttyUSB0`, and your user must be in
`dialout`.

## Navigation and obstacle avoidance

The map is built **once** and localised against thereafter. Running SLAM continuously
would accumulate drift through every shift for no benefit, since the pit does not
change shape.

```bash
ros2 launch excavator_bringup mapping.launch.py           # drive the whole working area
ros2 run nav2_map_server map_saver_cli -f ~/maps/pit_map  # then never map again
ros2 launch excavator_bringup navigation.launch.py map:=~/maps/pit_map.yaml
```

### Why not RTAB-Map

RTAB-Map is appearance-based visual SLAM: its strength is recognising places from
visual features, which requires visual texture. This machine works against a
low-texture limestone face, from a chassis that vibrates, steering by slipping its
tracks. That is all three classic visual-odometry failure modes at once, and it is the
normal operating condition rather than an edge case — so the stack was paying RTAB-Map's
considerable RAM and CPU cost for a capability the environment defeats.

`full_launch.py` is kept as the legacy RTAB-Map launch for comparison. Nothing in the
current navigation path uses it.

### Odometry

Two sensors, each doing what it is good at:

| Source | Contributes | Why |
| ------ | ----------- | --- |
| Track encoders (`track_odometry`) | forward velocity | Tracks measure distance well |
| D435i gyro (`imu_filter_madgwick`) | yaw and yaw rate | A gyro is unaffected by slip |

A skid-steer machine turns by making its tracks slip sideways **on purpose**, so
heading derived from track encoders is wrong by construction, not by poor calibration.
No amount of tuning `counts_per_metre` fixes it. `config/ekf.yaml` therefore fuses only
track *velocity* and IMU *heading*, and the node's yaw covariance is deliberately
pessimistic (0.30) to say so.

Exactly one node may own the `odom → base_footprint` transform. When the EKF runs it
takes that job, and `track_odometry` is launched with `publish_tf:=false`; otherwise TF
flips between two disagreeing transforms for the same frame pair.

### Height-banded obstacle avoidance

A 2D laser scan collapses the depth image to one horizontal slice, in which a limestone
pile and a concrete wall are indistinguishable. That is exactly the information needed
to tell a dig target from an obstacle, so obstacle marking uses the **point cloud**
instead, and both costmaps use `VoxelLayer` with two sources doing different jobs:

| Source | Role | Reason |
| ------ | ---- | ------ |
| Point cloud | **marks**, height-filtered | Height separates material from obstacle |
| `/scan` | **clears** only, never marks | So it cannot reintroduce what the filter removed |

Bands, measured above `base_footprint`:

| Height | Interpretation |
| ------ | -------------- |
| below 0.10 m | ground and loose material — ignored |
| 0.10 – 0.40 m | limestone pile — diggable, not an obstacle |
| above 0.40 m | genuine obstacle — marked and avoided |

Raising `min_obstacle_height` makes the machine willing to drive into bigger piles;
lowering it makes it treat rubble as a wall. The global costmap uses `VoxelLayer` too,
not the stock `ObstacleLayer` — otherwise a pile the local costmap correctly ignored
would burn into the global map as a permanent wall.

The same point cloud, transformed into `arm_base_frame`, is what supplies the dig
target. One pipeline serves both navigation and digging.

### Sensor limits worth designing around

- The D435i is **blind closer than ~0.3 m**. On a machine with an arm swinging in front
  of it, do not rely on the camera for close-in protection.
- FOV is **~87° × 58°, forward only**. Nothing to the sides or rear; when the cab
  rotates or the machine reverses, it is driving blind.

## Before it can drive

Everything above builds and the TF tree is verified, but three things are placeholders.
Until they are real, the navigation stack will run and do the wrong thing rather than
fail loudly.

### 1. Measure the machine

Every value marked `MEASURE` in `src/excavator_description/urdf/excavator.urdf.xacro`
is currently a guess: chassis dimensions, track separation, and the camera mount
(`cam_x`, `cam_y`, `cam_z`, `cam_pitch`).

The camera transform matters most. Every height threshold above is measured from
`base_footprint` through that transform, so a wrong `cam_pitch` tilts the whole point
cloud and the ground in front of the machine reads as an untraversable wall. This is
the most common way this setup fails, and it looks like a costmap bug rather than a
measurement error.

Rebuild after editing — the URDF is installed to `share/`, so source edits do not take
effect until `colcon build`.

### 2. Calibrate the odometry scale

`counts_per_metre` in `track_odometry` defaults to `1000.0`, which is a placeholder.
Every distance nav2 believes it has travelled is arbitrary until this is measured:

```bash
ros2 run excavator_control track_odometry
ros2 topic echo /odom --field pose.pose.position.x
# drive a measured distance, compare, then set the parameter
```

`track_separation` in both the URDF and the odometry node must match the real
centre-to-centre track spacing, or every turn will be scaled wrong.

### 3. Map only once the first two are right

Map quality is bounded by odometry quality. Every yaw error present during the mapping
run is baked permanently into the map that AMCL then localises against for the life of
the machine. Mapping against uncalibrated odometry produces a map you will have to
throw away.

## Reference scripts

`src/excavator_control/scripts/` holds pre-ROS hardware scripts, kept for reference.
They are successive iterations rather than a module hierarchy — each re-declares its own
hardware setup:

| Script | What it is |
| ------ | ---------- |
| `dig_V1.py` | Actuator sequences only, no swingdrive. |
| `dig_V2.py` | Adds swingdrive with `setMaxSpeed`. |
| `dig_V3.py` | The working cycle: scoop → up → rotate → dump → return. Sequenced by hardcoded `sleep()`, with no feedback that a move completed. |
| `dig_V4.py` | Design notes only. Records the unresolved bucket-vs-chassis collision boundary. |
| `Body.py`, `bodycontrol.py` | Near-duplicate curses teleop for boom/stick/bucket; `Body.py` also drives the swingdrive. |
| `keyboard_track.py`, `tracktest.py` | Curses teleop for the two tracks. |
| `coordinate_legacy.py` | The former IK REPL. Superseded by `ros2 run excavator_kinematics ik_repl`, which touches no hardware. |

These still carry duplicated hardware constants and import `lss` by bare name, so they
only run from inside their own directory. Port them into `excavator_control` nodes
rather than extending them in place.

## What is not tracked here

| Excluded | Why | How to get it |
| -------- | --- | ------------- |
| `~/ros2_ws/src/{librealsense,realsense-ros,rtabmap,rtabmap_ros}` | Third-party clones, ~800 MB, own histories | `vcs import ~/ros2_ws/src < upstream.repos` |
| `LSS_Library_Python/` | Upstream vendor clone | `git clone https://github.com/Lynxmotion/LSS_Library_Python.git` (the bus library itself is vendored into `excavator_control`) |
| `jetson-orin-nano-devkit-super-SD-image_JP6.2.1/` | 23 GB SD-card image | Download JetPack 6.2.1 for the Orin Nano dev kit from NVIDIA |

Also ignored: colcon build output (`build/`, `devel/`, `install/`, `log/`, `*.bag`),
Python caches and virtualenvs, kernel module build artifacts, and editor/OS junk.

## Troubleshooting

| Symptom | Likely cause |
| ------- | ------------ |
| `ValueError: Target unreachable for given L1, L2, L3.` | The **wrist** falls outside `[\|L1−L2\|, L1+L2]`. Often the tip is fine but `phi` pushes the wrist out — try a shallower tool angle first. |
| `JointLimitError: … outside travel` | The pose is kinematically reachable but past an actuator's mechanical travel. For the bucket specifically, see the open discrepancy under [Calibration](#calibration). |
| Simulator target does nothing, error printed to terminal | Parse and IK failures go to stdout, not the GUI. Check the launching terminal. |
| IK angles look sane, arm moves to the wrong place | A stale constant in `calibration.py` after re-zeroing a joint, or a pose that saturated at an end stop. |
| Arm moves the wrong direction on one joint | Stick and bucket have **negative** slopes; a sign flip inverts exactly one joint. |
| `colcon build` can't find `realsense2_camera` / `rtabmap_*` | The third-party overlay isn't sourced: `source ~/ros2_ws/install/setup.bash` before building. |
| The ground in front of the machine shows as an obstacle | Almost always `cam_pitch` or `cam_z` in the xacro, not a costmap problem. See [Before it can drive](#before-it-can-drive). |
| Costmap stays empty; no camera transform | `swing_joint` is revolute, so `robot_state_publisher` emits nothing for the camera until `/joint_states` exists. Check `joint_state_bridge` is running. |
| TF jitters between two poses for `base_footprint` | Both the EKF and `track_odometry` are publishing `odom → base_footprint`. The EKF owns it; `track_odometry` needs `publish_tf:=false`. |
| `ekf_node`: `libdiagnostic_updater.so: cannot open shared object file` | `ros-humble-diagnostic-updater` 4.0.6 ships headers with no library. Upgrade to 4.0.7. |
| Robot drifts in heading on every turn | Expected without the EKF — skid-steer yaw from tracks alone is wrong by construction. Launch with `use_ekf:=true` (the default). |
| Nav2 reports no odometry / won't activate | `track_odometry` needs `/dev/ttyUSB0` and membership of `dialout`. Without the LSS bus there is no odometry source at all. |
| No `/dev/ttyUSB*` after plugging in the adapter | CH341 module not loaded (`sudo make load`), or your user isn't in `dialout`. |
| ArUco poses off by a constant scale factor | `marker_size` doesn't match the physically printed marker — printers rescale by default. |
