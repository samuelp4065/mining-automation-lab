---
title: Software
layout: default
nav_order: 6
---

# Software layout

---

## Packages

| Package | Responsibility |
|---|---|
| `excavator_kinematics` | Closed-form 3-link planar IK/FK and joint calibration. Pure Python, no ROS or hardware |
| `excavator_control` | LSS servo bus, track drive, swingdrive, linear actuators, teleop |
| `excavator_perception` | ArUco detection, depth-based dig-target extraction, proximity interlock |
| `excavator_description` | URDF/xacro description and TF tree |
| `excavator_bringup` | Launch files and RViz configs |

## Nodes

| Node | Package | Does |
|---|---|---|
| `motor_node` | control | Track drive from `/cmd_vel` |
| `track_odometry` | control | Integrates encoder counts into odometry |
| `joint_state_bridge` | control | Publishes joint states, drives the swingdrive |
| `teleop_keyboard` | control | Keyboard drive |
| `aruco_detection_node` | perception | Marker detection |
| `proximity_monitor` | perception | Obstacle interlock |
| `fake_depth` | perception | Synthetic depth for testing without a camera |
| `ik_repl` | kinematics | Pose → angles → servo commands, no hardware |

## Launch files

`sensors` · `description` · `mapping` · `navigation` · `proximity_drive` · `full_launch`

---

## Building

The repository builds as an **overlay** on a separate workspace holding third-party
sources. Source that overlay first, or the build will not find librealsense and rtabmap.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To reconstruct the third-party workspace on a fresh machine:

```bash
vcs import ~/ros2_ws/src < upstream.repos && cd ~/ros2_ws && colcon build
```

---

## Testing

The kinematics package needs no ROS and no hardware, so its tests run anywhere:

```bash
colcon test --packages-select excavator_kinematics && colcon test-result --verbose

# faster loop
cd src/excavator_kinematics && python3 -m pytest test/ -q
```

32 tests, under a second. The control, perception, mapping and depth tools carry their own
test files as well.

---

## Running

```bash
ros2 launch excavator_bringup full_launch.py   # RealSense + RTAB-Map SLAM
ros2 run excavator_control motor_node          # track drive from /cmd_vel
ros2 run excavator_kinematics ik_repl          # IK REPL, no hardware
python3 tools/simulator/arm_simulator.py       # interactive arm simulator
```

---

## Standalone tools

Everything under `tools/` runs **without a ROS environment**, which is the point of keeping
it separate from the packages. Bring-up tools that need no sourced workspace are far more
useful when something is broken.

| Directory | Contents |
|---|---|
| `tools/actuators/` | Channel identification, single-angle commands, teleop, PWM diagnostics |
| `tools/simulator/` | Interactive Matplotlib arm simulator |
| `tools/mapping/` | Hand-authored nav2 occupancy maps |
| `tools/perception/` | Depth frame inspection |
| `tools/drivers/` | USB-serial driver source for the LSS bus |

---

## Platform notes

{: .note }
> **Jetson.GPIO does not recognise the Orin Nano Super devkit.** Its device tree reports
> `nvidia,p3768-0000+p3767-0005-super`, and version 2.1.7's model table only lists the
> string without the `-super` suffix, so importing it fails with *"Could not determine
> Jetson model"*. The supported workaround is an environment override:
>
> ```bash
> export JETSON_MODEL_NAME=JETSON_ORIN_NANO
> ```
>
> Modules that need it set this defensively at import time, so scripts work regardless of
> shell configuration.

{: .note }
> **`gpiozero` does not work on a Jetson.** Its pin factories expect Broadcom hardware. Any
> Raspberry Pi code being ported needs its GPIO layer replaced — see
> [Actuation](actuators) for the interface-compatible replacement.
