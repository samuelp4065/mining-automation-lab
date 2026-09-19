#!/usr/bin/env python3
"""Terminal-only depth pipeline for the RealSense D435i.

No ROS, no OpenCV, no X server -- this runs over SSH on the Orin and is the
tool to reach for when the question is "is the camera actually seeing the
face?" rather than "is SLAM converging?". For the latter, use
`ros2 launch excavator_bringup full_launch.py`.

The pipeline is three separable stages:

    open_pipeline()  -> a started rs.pipeline (live device, or a .bag replay)
    build_filters()  -> the post-processing chain applied to every frame
    a consumer       -> view / stats / probe / slice / record

`slice` is the one that matters for digging. It does what the architecture
notes call for -- take the *live* depth frame, deproject a horizontal band
into metric points, and report the nearest surface per column -- rather than
reading a dig target out of the drift-accumulating RTAB-Map grid.

Usage:
    python3 tools/perception/depth_cli.py list
    python3 tools/perception/depth_cli.py view
    python3 tools/perception/depth_cli.py stats
    python3 tools/perception/depth_cli.py probe
    python3 tools/perception/depth_cli.py center
    python3 tools/perception/depth_cli.py slice --units cm
    python3 tools/perception/depth_cli.py record out.bag --seconds 10
    python3 tools/perception/depth_cli.py view --replay out.bag
"""

import argparse
import shutil
import signal
import sys
import time
import warnings

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit(
        "pyrealsense2 not found. It ships with the librealsense build in "
        "~/ros2_ws; source that overlay first:\n"
        "    source ~/ros2_ws/install/setup.bash"
    )

# The D435i is blind closer than ~0.3 m, and past ~8 m the stereo disparity is
# noise. Same numbers as the depthimage_to_laserscan node in sensors.launch.py.
RANGE_MIN_M = 0.30
RANGE_MAX_M = 8.0

_RAMP = " .:-=+*#%@"
# Near = warm, far = cool, invalid = blank. xterm-256 indices.
_PALETTE = [196, 202, 208, 214, 220, 190, 154, 118, 82, 48, 44, 39, 33, 27, 21]

_running = True


def _stop(_signum, _frame):
    global _running
    _running = False


# --------------------------------------------------------------------------
# Stage 1: open
# --------------------------------------------------------------------------

def list_devices():
    """Enumerate attached cameras. First thing to run when something is off."""
    devices = list(rs.context().query_devices())
    if not devices:
        print("No RealSense device found.")
        print("Check: USB3 cable seated, `lsusb | grep Intel`, udev rules "
              "installed (99-realsense-libusb.rules).")
        return 1
    for dev in devices:
        print(f"{dev.get_info(rs.camera_info.name)}")
        print(f"  serial   {dev.get_info(rs.camera_info.serial_number)}")
        print(f"  firmware {dev.get_info(rs.camera_info.firmware_version)}")
        print(f"  usb      {dev.get_info(rs.camera_info.usb_type_descriptor)}")
    return 0


def open_pipeline(args, record_to=None):
    """Start streaming and return (pipeline, depth_scale).

    depth_scale converts the raw uint16 z16 values to metres. It is a
    per-device constant (typically 0.001) and must never be hardcoded.
    """
    pipeline = rs.pipeline()
    config = rs.config()

    if args.replay:
        # Replay makes every consumer below testable with no hardware present.
        config.enable_device_from_file(args.replay, repeat_playback=False)
    else:
        if args.serial:
            config.enable_device(args.serial)
        config.enable_stream(rs.stream.depth, args.width, args.height,
                             rs.format.z16, args.fps)
        if record_to:
            config.enable_record_to_file(record_to)

    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    if args.replay:
        # Without this, playback races ahead and drops frames we asked for.
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)

    return pipeline, depth_scale


# --------------------------------------------------------------------------
# Stage 2: filter
# --------------------------------------------------------------------------

def build_filters(decimate, min_range=RANGE_MIN_M, max_range=RANGE_MAX_M):
    """The recommended librealsense post-processing order.

    Spatial and temporal filters operate on *disparity*, not depth, so they
    are wrapped in the disparity transform pair. Getting this order wrong is
    the usual reason filtering makes depth worse instead of better.

    Hole filling is deliberately omitted: it invents surfaces, and a dig
    target invented out of a hole in a limestone face is worse than no target.
    """
    to_disparity = rs.disparity_transform(True)
    to_depth = rs.disparity_transform(False)

    threshold = rs.threshold_filter()
    # librealsense caps this filter at 16 m and rejects anything beyond with
    # an opaque "out of range value" error, so clamp and say so instead.
    for option, value, label in (
        (rs.option.min_distance, min_range, "--min-range"),
        (rs.option.max_distance, max_range, "--max-range"),
    ):
        limits = threshold.get_option_range(option)
        clamped = min(max(value, limits.min), limits.max)
        if clamped != value:
            print(f"note: {label} {value} m is outside the filter's "
                  f"{limits.min:g}-{limits.max:g} m range; using {clamped:g} m",
                  file=sys.stderr)
        threshold.set_option(option, clamped)

    chain = [threshold]
    if decimate > 1:
        dec = rs.decimation_filter()
        dec.set_option(rs.option.filter_magnitude, float(decimate))
        chain.append(dec)
    chain += [to_disparity, rs.spatial_filter(), rs.temporal_filter(), to_depth]
    return chain


def apply_filters(frame, chain):
    for f in chain:
        frame = f.process(frame)
    return frame.as_depth_frame()


def grab(pipeline, chain, timeout_ms=5000):
    """One filtered depth frame, or None if the stream ended or stalled."""
    try:
        frames = pipeline.wait_for_frames(timeout_ms)
    except RuntimeError:
        return None
    depth = frames.get_depth_frame()
    if not depth:
        return None
    return apply_filters(depth, chain)


def as_metres(depth_frame, depth_scale):
    """Depth frame -> float32 metres, with invalid pixels as NaN.

    Zero in a z16 frame means "no measurement", not "zero distance". Letting
    those zeros through as 0.0 is what makes naive min-distance code report a
    surface pressed against the lens.
    """
    raw = np.asanyarray(depth_frame.get_data())
    metres = raw.astype(np.float32) * depth_scale
    metres[raw == 0] = np.nan
    return metres


# --------------------------------------------------------------------------
# Stage 3: consumers
# --------------------------------------------------------------------------

def _colorize(norm, ch):
    idx = _PALETTE[min(int(norm * len(_PALETTE)), len(_PALETTE) - 1)]
    return f"\x1b[38;5;{idx}m{ch}\x1b[0m"


def render(metres, near, far, cols, rows, color=True):
    """Downsample to a character grid and paint it.

    Terminal cells are roughly twice as tall as wide, so rows are sampled at
    half the column rate to keep the scene's aspect ratio honest.
    """
    h, w = metres.shape
    ys = np.linspace(0, h - 1, rows).astype(int)
    xs = np.linspace(0, w - 1, cols).astype(int)
    grid = metres[np.ix_(ys, xs)]

    span = max(far - near, 1e-6)
    lines = []
    for row in grid:
        out = []
        for d in row:
            if np.isnan(d):
                out.append(" ")
                continue
            norm = float(np.clip((d - near) / span, 0.0, 1.0))
            ch = _RAMP[min(int((1.0 - norm) * len(_RAMP)), len(_RAMP) - 1)]
            out.append(_colorize(norm, ch) if color else ch)
        lines.append("".join(out))
    return "\n".join(lines)


def cmd_view(args):
    pipeline, scale = open_pipeline(args)
    chain = build_filters(args.decimate, args.min_range, args.max_range)
    color = not args.no_color and sys.stdout.isatty()
    frames = 0
    t0 = time.time()
    try:
        sys.stdout.write("\x1b[2J\x1b[?25l")  # clear, hide cursor
        while _running:
            depth = grab(pipeline, chain)
            if depth is None:
                break
            metres = as_metres(depth, scale)
            term = shutil.get_terminal_size((100, 30))
            cols = min(args.cols or term.columns, term.columns)
            rows = max(term.lines - 4, 8)
            frames += 1
            fps = frames / max(time.time() - t0, 1e-6)

            valid = np.isfinite(metres)
            coverage = 100.0 * valid.mean()
            nearest = float(np.nanmin(metres)) if valid.any() else float("nan")

            sys.stdout.write("\x1b[H")
            sys.stdout.write(render(metres, args.near, args.far, cols, rows,
                                    color) + "\n")
            sys.stdout.write(
                f"\x1b[Knear {args.near:.2f}m -> far {args.far:.2f}m  "
                f"warm=close cool=far blank=no data\n"
                f"\x1b[Kcoverage {coverage:5.1f}%   nearest {nearest:5.2f}m   "
                f"{fps:4.1f} fps   frame {frames}   (ctrl-c to quit)\n")
            sys.stdout.flush()
    finally:
        pipeline.stop()
        sys.stdout.write("\x1b[?25h\n")
    return 0


def cmd_stats(args):
    """Numbers rather than pictures -- what to paste into a bug report."""
    pipeline, scale = open_pipeline(args)
    chain = build_filters(args.decimate, args.min_range, args.max_range)
    try:
        while _running:
            depth = grab(pipeline, chain)
            if depth is None:
                break
            m = as_metres(depth, scale)
            valid = m[np.isfinite(m)]
            if valid.size == 0:
                print("no valid pixels -- lens capped, or everything out of range")
                continue
            p5, p50, p95 = np.percentile(valid, [5, 50, 95])
            print(f"coverage {100.0 * valid.size / m.size:5.1f}%  "
                  f"min {valid.min():5.2f}  p5 {p5:5.2f}  med {p50:5.2f}  "
                  f"p95 {p95:5.2f}  max {valid.max():5.2f}  (m)")
            if args.once:
                break
    finally:
        pipeline.stop()
    return 0


def cmd_probe(args):
    """Distance at a 3x3 grid of points, averaged over several frames.

    Point the camera at a wall you can tape-measure and compare. This is the
    check that catches a wrong depth scale or a units bug before it reaches
    the IK solver.
    """
    pipeline, scale = open_pipeline(args)
    chain = build_filters(args.decimate, args.min_range, args.max_range)
    stack = []
    try:
        while _running and len(stack) < args.samples:
            depth = grab(pipeline, chain)
            if depth is None:
                break
            stack.append(as_metres(depth, scale))
    finally:
        pipeline.stop()

    if not stack:
        print("no frames captured")
        return 1

    # An all-NaN stack at some pixel is normal -- it just means nothing
    # returned there across every frame. numpy warns anyway, and the warning
    # lands in the middle of the table it is printing.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        avg = np.nanmedian(np.dstack(stack), axis=2)
    h, w = avg.shape
    print(f"median of {len(stack)} frames, {w}x{h}\n")
    for fy in (0.25, 0.5, 0.75):
        cells = []
        for fx in (0.25, 0.5, 0.75):
            d = avg[int(fy * h), int(fx * w)]
            cells.append("    n/a" if np.isnan(d) else f"{d:6.3f}m")
        print("   ".join(cells))
    return 0


def cmd_slice(args):
    """Nearest surface per column across a horizontal band -- a face profile.

    Deprojects with the frame's own intrinsics (which change under decimation,
    so they are re-read every frame rather than cached). Output is in the
    optical frame: +x right, +y down, +z forward. Transforming that into the
    arm base frame is the caller's job.
    """
    pipeline, scale = open_pipeline(args)
    chain = build_filters(args.decimate, args.min_range, args.max_range)
    unit_scale = 100.0 if args.units == "cm" else 1.0
    try:
        while _running:
            depth = grab(pipeline, chain)
            if depth is None:
                break
            intr = depth.profile.as_video_stream_profile().intrinsics
            m = as_metres(depth, scale)
            h, w = m.shape

            half = max(args.band // 2, 1)
            mid = int(args.row * h)
            band = m[max(mid - half, 0):min(mid + half, h), :]

            # Nearest valid return per column: the face, not the sky behind
            # it. An all-NaN column is normal (a hole), so silence the warning
            # numpy raises for it -- errstate does not cover this one.
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                profile = np.nanmin(band, axis=0)

            cols = np.linspace(0, w - 1, args.columns).astype(int)
            print(f"--- band rows {max(mid-half,0)}..{min(mid+half,h)} of {h} "
                  f"({args.units}) ---")
            for x in cols:
                d = profile[x]
                if np.isnan(d):
                    print(f"  u={x:4d}   no return")
                    continue
                px, py, pz = rs.rs2_deproject_pixel_to_point(
                    intr, [float(x), float(mid)], float(d))
                print(f"  u={x:4d}   x {px * unit_scale:8.1f}   "
                      f"y {py * unit_scale:8.1f}   z {pz * unit_scale:8.1f}")
            if args.once:
                break
    finally:
        pipeline.stop()
    return 0


def sample_point(intr, at_pixel, at_frac):
    """Which pixel to sample, as (u, v).

    Defaults to the *optical* centre -- where the lens actually points --
    rather than the geometric middle of the image. On this D435i those differ
    by under a pixel, but the distinction matters on a camera with more
    decentring, and the optical centre is the point whose deprojected x and y
    are genuinely zero.

    Fractions are resolution-independent, which matters because decimation
    changes the frame size: --at-frac 0.5,0.5 is the middle whatever
    --decimate is set to, while a fixed pixel index is not.
    """
    if at_frac is not None:
        fx, fy = at_frac
        return (int(round(fx * (intr.width - 1))),
                int(round(fy * (intr.height - 1))))
    if at_pixel is not None:
        return int(at_pixel[0]), int(at_pixel[1])
    return int(round(intr.ppx)), int(round(intr.ppy))


def _pair(text):
    """Parse 'a,b' into a float pair."""
    parts = text.replace(" ", "").split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected two values as 'a,b'")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        raise argparse.ArgumentTypeError("values must be numeric")


def cmd_center(args):
    """Live distance straight ahead of the lens.

    The one to run while holding something in front of the camera. Reports
    the median of a small patch at the *optical* centre -- which is where the
    lens actually points, not quite the geometric middle of the image -- so a
    single dead pixel cannot make the reading jump.

    The printed x and y are the patch's lateral and vertical offset from the
    optical axis. Both should sit near zero; if they do not, the patch is
    straddling an edge and the distance is a blend of two surfaces.
    """
    pipeline, scale = open_pipeline(args)
    chain = build_filters(args.decimate, args.min_range, args.max_range)
    unit_scale = 100.0 if args.units == "cm" else 1.0
    half = max(args.patch // 2, 1)

    # Hold the last good reading briefly instead of flashing "no return".
    # A dropout of a frame or two is normal on a textured surface, and a
    # readout that alternates between a number and an error is unreadable --
    # you cannot tell a real loss of signal from ordinary flicker.
    last = None
    last_seen = 0.0
    try:
        while _running:
            depth = grab(pipeline, chain)
            if depth is None:
                break
            intr = depth.profile.as_video_stream_profile().intrinsics
            m = as_metres(depth, scale)

            cx, cy = sample_point(intr, args.at, args.at_frac)
            cx = int(np.clip(cx, 0, intr.width - 1))
            cy = int(np.clip(cy, 0, intr.height - 1))
            patch = m[max(cy - half, 0):cy + half, max(cx - half, 0):cx + half]
            valid = patch[np.isfinite(patch)]

            now = time.time()
            if valid.size:
                d = float(np.median(valid))
                px, py, pz = rs.rs2_deproject_pixel_to_point(
                    intr, [float(cx), float(cy)], d)
                last = (pz, px, py, 100.0 * valid.size / patch.size)
                last_seen = now

            if last is None:
                line = f"  --.- {args.units}   waiting for a return"
            else:
                pz, px, py, pct = last
                stale = now - last_seen
                line = f"{pz * unit_scale:8.1f} {args.units}"
                if args.verbose:
                    line += (f"   x {px * unit_scale:+6.1f}  "
                             f"y {py * unit_scale:+6.1f}   "
                             f"({cx},{cy})  {pct:3.0f}% valid")
                if stale > args.hold:
                    line += f"   [no return for {stale:.1f}s]"

            sys.stdout.write("\r\x1b[K" + line)
            sys.stdout.flush()
            if args.once:
                break
    finally:
        pipeline.stop()
        print()
    return 0


def cmd_record(args):
    """Capture a .bag so a field problem can be replayed at a desk."""
    pipeline, _ = open_pipeline(args, record_to=args.path)
    chain = build_filters(1, args.min_range, args.max_range)  # record raw
    t0 = time.time()
    n = 0
    try:
        while _running and time.time() - t0 < args.seconds:
            if grab(pipeline, chain) is None:
                break
            n += 1
            print(f"\r{n} frames, {time.time() - t0:4.1f}s", end="", flush=True)
    finally:
        pipeline.stop()
        print(f"\nwrote {args.path}")
    return 0


# --------------------------------------------------------------------------

def main(argv=None):
    signal.signal(signal.SIGINT, _stop)

    parser = argparse.ArgumentParser(
        description="Terminal depth pipeline for the RealSense D435i.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    def common(p, decimate=2):
        p.add_argument("--serial", help="target a specific camera")
        p.add_argument("--replay", metavar="BAG",
                       help="read from a .bag instead of a live camera")
        p.add_argument("--width", type=int, default=640)
        p.add_argument("--height", type=int, default=480)
        p.add_argument("--fps", type=int, default=30)
        p.add_argument("--decimate", type=int, default=decimate,
                       help="decimation magnitude; 1 disables")
        p.add_argument("--min-range", type=float, default=RANGE_MIN_M,
                       dest="min_range", metavar="M",
                       help="drop returns closer than this (default %(default)s)")
        p.add_argument("--max-range", type=float, default=RANGE_MAX_M,
                       dest="max_range", metavar="M",
                       help="drop returns beyond this (default %(default)s). "
                            "Raise it when most of the scene reads as no-return "
                            "-- low coverage usually means a distant scene, not "
                            "a broken camera")
        return p

    sub.add_parser("list", help="enumerate attached cameras")

    p = common(sub.add_parser("view", help="live ASCII depth heatmap"))
    p.add_argument("--near", type=float, default=RANGE_MIN_M)
    p.add_argument("--far", type=float, default=4.0)
    p.add_argument("--cols", type=int, help="width in characters")
    p.add_argument("--no-color", action="store_true")

    p = common(sub.add_parser("stats", help="per-frame depth statistics"))
    p.add_argument("--once", action="store_true")

    p = common(sub.add_parser("probe", help="distance at a 3x3 point grid"))
    p.add_argument("--samples", type=int, default=15)

    p = common(sub.add_parser("slice", help="nearest surface per column"))
    p.add_argument("--row", type=float, default=0.5,
                   help="band centre as a fraction of image height")
    p.add_argument("--band", type=int, default=20, help="band height in rows")
    p.add_argument("--columns", type=int, default=12, help="samples across")
    p.add_argument("--units", choices=("m", "cm"), default="m",
                   help="cm matches the kinematics solver")
    p.add_argument("--once", action="store_true")

    p = common(sub.add_parser(
        "center", help="live distance straight ahead of the lens"))
    p.add_argument("--patch", type=int, default=9,
                   help="patch size in pixels, median-filtered. 1 samples a "
                        "single pixel, which on this sensor is noisy and often "
                        "invalid (default %(default)s)")
    p.add_argument("--at", type=_pair, metavar="U,V",
                   help="sample at this pixel instead of the optical centre")
    p.add_argument("--at-frac", type=_pair, metavar="X,Y", dest="at_frac",
                   help="sample at this fraction of the frame, e.g. 0.5,0.5 "
                        "for the middle. Survives --decimate, which a fixed "
                        "pixel index does not")
    p.add_argument("--units", choices=("m", "cm"), default="cm")
    p.add_argument("--verbose", action="store_true",
                   help="also show x/y offset, pixel and patch validity")
    p.add_argument("--hold", type=float, default=0.5, metavar="S",
                   help="keep showing the last reading for this long through "
                        "a dropout before flagging it (default %(default)s)")
    p.add_argument("--once", action="store_true")

    p = common(sub.add_parser("record", help="capture a .bag"), decimate=1)
    p.add_argument("path")
    p.add_argument("--seconds", type=float, default=10.0)

    args = parser.parse_args(argv)
    handlers = {
        "list": lambda a: list_devices(), "view": cmd_view, "stats": cmd_stats,
        "probe": cmd_probe, "slice": cmd_slice, "center": cmd_center,
        "record": cmd_record,
    }
    try:
        return handlers[args.cmd](args)
    except RuntimeError as exc:
        print(f"librealsense error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
