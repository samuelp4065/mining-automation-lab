# Occupancy map builder

`occupancy_map.py` builds a nav2-compatible 2D occupancy grid **by hand**, in
metres — no SLAM, no ROS, no camera. For a site whose boundaries you already
know, this is faster and more accurate than driving `slam_toolbox` around it:
SLAM adds scan noise to walls that are actually perfectly straight.

```bash
# quick start: a bordered rectangle at your known site dimensions
python3 tools/mapping/occupancy_map.py 20 15 ~/maps/pit_map --resolution 0.1

# then drive it
ros2 launch excavator_bringup navigation.launch.py map:=~/maps/pit_map.yaml
```

## Drawing something more than a rectangle

```python
from occupancy_map import OccupancyMap

m = OccupancyMap(width_m=20.0, height_m=15.0, resolution=0.1)
m.mark_free_rect(0, 0, 20, 15)          # interior is drivable
m.add_border_wall(0.3)                  # the site's known perimeter
m.add_rect_obstacle(x=8, y=6, w=2, h=1.5)   # a fixed pile or structure
m.add_circle_obstacle(cx=15, cy=3, radius_m=1.0)   # a support column
m.add_line_obstacle(0, 10, 12, 14, thickness_m=0.3)  # an angled boundary

print(m.preview())          # ASCII check before saving -- do this every time
m.save("~/maps/pit_map")    # writes pit_map.yaml + pit_map.pgm
```

Everything is in **metres**, in the same `+x forward, +y left` frame as the
rest of this repo. Never pass pixel indices — that is the whole point of not
hand-editing a `.pgm` with a paint program and hoping the scale lines up.

## Editing the image directly

`--format png` writes a PNG instead of PGM, which opens in any normal image
editor for detail a few `add_rect_obstacle` calls can't express — black
(0,0,0) is occupied, white (255,255,255) is free, mid-grey (205,205,205) is
unknown. Re-load and check with `preview()` before trusting it; a stray
anti-aliased edge from a paint tool can leave pixels at ambiguous grey values
that fall on the wrong side of `occupied_thresh`/`free_thresh` in the YAML.

## Getting the map from your laptop instead

If the site is irregular enough that hand-drawing is more trouble than it's
worth, run `slam_toolbox` on a laptop (far more spare compute than the
Jetson) instead of on the machine itself:

```bash
# on the laptop, with a depth/lidar source and odometry available
ros2 launch slam_toolbox online_async_launch.py
ros2 run nav2_map_server map_saver_cli -f ~/pit_map
# copy pit_map.yaml + pit_map.pgm to the Jetson's ~/maps/
```

Either path produces the same `.yaml` + `.pgm` pair `navigation.launch.py`
expects. The point of both is the same: pay the SLAM cost once, somewhere
compute is cheap, and never run it on the Jetson during normal operation.

## Why this saves compute on the Jetson

`mapping.launch.py` runs `slam_toolbox` continuously: scan matching every
frame, pose-graph optimisation, loop closure. Supplying the map yourself
skips all of it — `navigation.launch.py` only ever needs **AMCL**, a particle
filter comparing the current scan against a map that never changes. No graph
optimisation, no loop closure, no continuous scan matching.

The depth sensor's job doesn't change: `proximity_monitor` and the nav2
height-banded costmap still watch for whatever the static map can't know
about — something that moved into the site since the map was drawn. The
static map handles known, fixed geometry; the live sensor handles what's
different right now. That split is what "2.5D" means here in practice.

## Nav2 map format, in case you're debugging by hand

```
image: pit_map.pgm
resolution: 0.1          # metres per pixel
origin: [0.0, 0.0, 0.0]  # world (x, y, yaw) of the image's BOTTOM-LEFT pixel
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

Image row 0 is the **top** of the map (largest y) — `OccupancyMap` handles
that flip internally so every method here takes plain metre coordinates with
+y up. Getting this backwards by hand is the classic way a map opens fine in
an image viewer but drives mirrored in nav2.

## Tests

```bash
python3 -m pytest tools/mapping/ -q
```

14 tests, no ROS required. Worth knowing: an earlier version of this file had
a real fencepost bug — a 0.5 m border wall came out 2 pixels wide on one side
and 1 pixel wide on the other, because the two edges of a rectangle were
located by converting independent corner points through a rounding
conversion rather than treating each axis as one continuous span. The tests
for exact pixel counts and border symmetry are pinned to that failure mode.
The fix, plus a live load through nav2's actual `map_server` node (lifecycle
`configure`/`activate`, both succeeded, `Read map ... 20 X 12 map @ 0.5
m/cell`), is what the current geometry is checked against.
