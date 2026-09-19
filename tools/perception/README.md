# Depth pipeline (terminal)

`depth_cli.py` reads the RealSense D435i and reports distances as text. No ROS,
no OpenCV, no X server — it runs over SSH on the Orin, and answers "is the
camera actually seeing the face?" For "is SLAM converging?", use
`ros2 launch excavator_bringup full_launch.py` instead.

```bash
source ~/ros2_ws/install/setup.bash     # for pyrealsense2

python3 tools/perception/depth_cli.py list            # enumerate cameras
python3 tools/perception/depth_cli.py view            # live ASCII heatmap
python3 tools/perception/depth_cli.py stats           # coverage + percentiles
python3 tools/perception/depth_cli.py probe           # 3x3 grid of distances
python3 tools/perception/depth_cli.py slice --units cm  # face profile
python3 tools/perception/depth_cli.py record out.bag --seconds 10
python3 tools/perception/depth_cli.py view --replay out.bag
```

| Command | Use it for |
| ------- | ---------- |
| `list` | First thing to run when something is off. Confirms USB3 and firmware. |
| `view` | Is the camera pointed at the right thing? Warm = close, cool = far, blank = no return. |
| `stats` | Numbers to paste into a bug report. Low coverage means a dark or glossy surface. |
| `probe` | Point at a wall, tape-measure it, compare. Catches a units or depth-scale error before it reaches the IK solver. |
| `slice` | The dig-relevant one: nearest surface per column across a horizontal band, deprojected to metric points. `--units cm` matches the kinematics solver. |
| `record` | Capture a `.bag` so a field problem can be replayed at a desk. |

## Notes

**Zero is not a distance.** A zero in a z16 frame means *no measurement*.
`as_metres` turns those into NaN — letting them through as `0.0` makes naive
nearest-distance code report a surface pressed against the lens.

**Hole filling is deliberately off.** It invents surfaces, and a dig target
invented out of a hole in a limestone face is worse than no target at all.

**Filter order matters.** Spatial and temporal filters operate on disparity,
not depth, so they sit inside the disparity transform pair. Getting that
backwards is the usual reason filtering makes depth worse.

**`slice` output is in the optical frame** (+x right, +y down, +z forward).
Transforming into `arm_base_frame` is the caller's job — the TF tree in
`excavator_description` is what does it.

`RANGE_MIN_M` / `RANGE_MAX_M` must stay in step with `sensors.launch.py`, or
this tool and the nav2 costmap will disagree about what the camera can see.

## Tests

```bash
python3 -m pytest tools/perception/ -q
```

Nine tests, no camera required — they run on synthetic z16 frames. The parts
most likely to be wrong (unit conversion, invalid pixels, nearest-surface
selection) are exactly the parts that need no hardware.
