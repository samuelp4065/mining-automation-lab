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
python3 tools/perception/depth_cli.py center          # live distance dead ahead
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
| `center` | Live distance straight ahead. Hold something in front of the lens and read the number. |
| `record` | Capture a `.bag` so a field problem can be replayed at a desk. |

## Where is "straight ahead"?

The depth stream's optical centre on this camera is pixel **(319.2, 242.3)** of
640x480 — within a pixel of the geometric middle, so the middle of the frame
*is* the optical axis. Deprojecting the image centre at 1 m lands within 6 mm
of zero.

| command | where the centre is |
| ------- | ------------------- |
| `center` | the whole output — a patch at the optical centre |
| `probe` | the **middle cell** of the 3x3 grid |
| `view` | the middle of the ASCII grid |
| `slice` | the column where the printed `x` is nearest zero |

Coordinates are the **optical frame**: `x` right, `y` down, `z` forward, all
from the lens. So `z` is the distance you care about, and `x`/`y` near zero
confirm you are reading the centre rather than straddling an edge.

To sanity-check the camera, hold a flat object in front of the lens and run:

```bash
python3 tools/perception/depth_cli.py center
```

Then tape-measure it. Remember the D435i sees nothing closer than ~0.3 m, so
measure at 0.5 m or more — a hand pressed against the lens reads as no return,
not as zero.

One physical note: depth originates at the **left infrared imager**, not the
colour lens in the middle of the camera. At arm's length that ~2 cm offset does
not matter; for calibrating the arm it does.

## Low coverage is usually a distant scene, not a broken camera

`stats` reporting 10-20% coverage looks alarming and generally is not. The
threshold filter drops everything outside 0.3-8 m, and in a large space most
of the scene is beyond that. Measured on one indoor scene:

| range | share of raw returns |
| ----- | -------------------- |
| below 0.3 m | 13% (noise -- the D435i cannot see this close) |
| 0.3-8 m | 41% (kept) |
| beyond 8 m | 43% (dropped) |

Raw coverage was 70%. Widen the window to see what is really out there:

```bash
python3 tools/perception/depth_cli.py stats --once --max-range 16
```

librealsense caps the filter at 16 m; asking for more prints a note and uses
16. Keep the *default* at 8 m — it matches `sensors.launch.py`, and the two
disagreeing would mean the costmap and this tool see different worlds.

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
