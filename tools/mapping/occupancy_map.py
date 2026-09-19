"""Build a nav2-compatible 2D occupancy grid map by hand, in metres.

No ROS, no SLAM, no camera. For a site whose boundaries you already know,
drawing the map is faster and more accurate than driving slam_toolbox around
it -- SLAM adds scan noise to walls that are actually perfectly straight.

The output is exactly what map_server expects: a .yaml sidecar plus a .pgm
(or .png) image, ready for

    ros2 launch excavator_bringup navigation.launch.py map:=/path/to/site.yaml

Coordinate convention
----------------------
Matches the rest of this repo: metres, +x forward/right, +y left, with
``origin`` set to the world coordinate of the image's bottom-left pixel. All
drawing methods here take **metre** coordinates in that frame -- never pixel
indices -- so a wall at "3.0 m from the corner" is written as 3.0, not as
some resolution-dependent pixel count.

nav2 map image convention (the "trinary" mode map_saver writes)
-----------------------------------------------------------------
    255 (white) = free            0 (black) = occupied            205 (gray) = unknown

Row 0 of the image is the TOP of the map (largest y); this class handles that
flip internally so callers only ever think in metres with +y up.

Usage
-----
    from occupancy_map import OccupancyMap

    m = OccupancyMap(width_m=20.0, height_m=15.0, resolution=0.05)
    m.fill_unknown()
    m.add_border_wall(thickness_m=0.2)          # the site's known perimeter
    m.add_rect_obstacle(x=8, y=6, w=2, h=1.5)   # a known fixed obstacle
    m.mark_free_rect(x=0.5, y=0.5, w=19, h=14)  # interior is drivable
    print(m.preview())                          # ASCII check before saving
    m.save("~/maps/pit_map")                    # writes pit_map.yaml + .pgm
"""

import os
from dataclasses import dataclass, field

import numpy as np

try:
    from PIL import Image
    _HAVE_PIL = True
except ImportError:
    _HAVE_PIL = False

__all__ = ["OccupancyMap", "FREE", "OCCUPIED", "UNKNOWN"]

# nav2 "trinary" pixel values -- see module docstring.
FREE = 255
OCCUPIED = 0
UNKNOWN = 205

_ASCII = {FREE: ".", OCCUPIED: "#", UNKNOWN: "?"}


@dataclass
class OccupancyMap:
    """A hand-drawable occupancy grid, in real-world metres.

    Args:
        width_m, height_m: site size in metres.
        resolution: metres per pixel. nav2's usual default is 0.05; a known,
            mostly-static site can go coarser (0.1-0.2) since there is no
            scan-matching accuracy to lose, which keeps the image small.
        origin: (x, y) world coordinate of the image's bottom-left pixel.
            (0, 0) is the natural choice when the excavator's start pose is
            treated as one corner of the site.
    """

    width_m: float
    height_m: float
    resolution: float = 0.05
    origin: tuple = (0.0, 0.0)
    grid: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        self.px_width = max(int(round(self.width_m / self.resolution)), 1)
        self.px_height = max(int(round(self.height_m / self.resolution)), 1)
        # Start UNKNOWN, not FREE: an area you have not deliberately marked
        # is exactly what "unknown" means, and nav2 treats it as
        # non-traversable by default -- the safer default for a hand-built
        # map where a forgotten spot should not silently become driveable.
        self.grid = np.full((self.px_height, self.px_width), UNKNOWN, dtype=np.uint8)

    # ---- metres <-> pixels ------------------------------------------------
    #
    # Two different conversions are needed, and conflating them is exactly
    # what caused an earlier version of this file to draw a 2-pixel-wide wall
    # on one side of the map and a 1-pixel wall on the other:
    #
    #  * a single METRE POINT (a circle centre) rounds to its nearest pixel.
    #  * a metre SPAN [x, x+w) covers every pixel it overlaps at all, via
    #    floor/ceil on a half-open range. Two corner-points converted with
    #    the point method and then treated as an inclusive range are NOT the
    #    same thing -- rounding is not additive, so opposite edges of a
    #    symmetric rectangle can end up covering different pixel counts.

    def _point_to_rowcol(self, x, y):
        """World metres -> (row, col) of the pixel containing that point."""
        col = int(np.floor((x - self.origin[0]) / self.resolution))
        row_from_bottom = int(np.floor((y - self.origin[1]) / self.resolution))
        row = self.px_height - 1 - row_from_bottom
        return row, col

    def _col_range(self, x, w):
        """Metre span [x, x+w) -> half-open pixel column range, clamped."""
        c0 = int(np.floor((x - self.origin[0]) / self.resolution))
        c1 = int(np.ceil((x + w - self.origin[0]) / self.resolution))
        return max(c0, 0), min(c1, self.px_width)

    def _row_range(self, y, h):
        """Metre span [y, y+h) -> half-open pixel row range, clamped.

        Rows run top-down while y runs bottom-up, so the bottom-up pixel
        range [k0, k1) maps to the top-down image range [H-k1, H-k0).
        """
        k0 = int(np.floor((y - self.origin[1]) / self.resolution))
        k1 = int(np.ceil((y + h - self.origin[1]) / self.resolution))
        r0, r1 = self.px_height - k1, self.px_height - k0
        return max(r0, 0), min(r1, self.px_height)

    def in_bounds(self, x, y):
        row, col = self._point_to_rowcol(x, y)
        return 0 <= row < self.px_height and 0 <= col < self.px_width

    # ---- drawing, all in metres --------------------------------------------

    def fill_unknown(self):
        self.grid[:] = UNKNOWN

    def mark_free_rect(self, x, y, w, h):
        """Mark a rectangle (corner at x,y, extending +w,+h) as free/driveable."""
        self._fill_rect(x, y, w, h, FREE)

    def add_rect_obstacle(self, x, y, w, h):
        """Mark a rectangle as occupied -- a wall, a fixed pile, machinery."""
        self._fill_rect(x, y, w, h, OCCUPIED)

    def add_border_wall(self, thickness_m):
        """Occupy a ring of the given thickness around the whole site.

        The common case for a known site: everything inside the perimeter is
        free, the perimeter itself is a wall. Call mark_free_rect for the
        interior first (or rely on the caller doing so) -- this only paints
        the border.
        """
        t = thickness_m
        self.add_rect_obstacle(0, 0, self.width_m, t)                      # bottom
        self.add_rect_obstacle(0, self.height_m - t, self.width_m, t)      # top
        self.add_rect_obstacle(0, 0, t, self.height_m)                     # left
        self.add_rect_obstacle(self.width_m - t, 0, t, self.height_m)      # right

    def add_line_obstacle(self, x1, y1, x2, y2, thickness_m=None):
        """A wall segment between two points, e.g. an angled boundary.

        Rasterised as a sequence of small squares along the line rather than
        a single-pixel Bresenham line, so `thickness_m` reliably produces a
        wall the depth sensor's own resolution can actually see -- a 1-pixel
        line at a coarse resolution is invisible to nav2's costmap inflation.
        """
        thickness_m = thickness_m or self.resolution * 2
        length = float(np.hypot(x2 - x1, y2 - y1))
        steps = max(int(length / (self.resolution / 2)), 1)
        for i in range(steps + 1):
            t = i / steps
            self._fill_rect(x1 + t * (x2 - x1) - thickness_m / 2,
                            y1 + t * (y2 - y1) - thickness_m / 2,
                            thickness_m, thickness_m, OCCUPIED)

    def add_circle_obstacle(self, cx, cy, radius_m):
        """A round obstacle -- a stockpile, a support column."""
        rows, cols = np.ogrid[:self.px_height, :self.px_width]
        row_c, col_c = self._point_to_rowcol(cx, cy)
        radius_px = radius_m / self.resolution
        mask = (rows - row_c) ** 2 + (cols - col_c) ** 2 <= radius_px ** 2
        self.grid[mask] = OCCUPIED

    def add_polygon_wall(self, points, thickness_m=None):
        """A closed wall following a sequence of (x, y) vertices, in metres.

        For an irregular site (an L-shape, a stepped boundary) rather than a
        simple rectangle. The polygon is closed automatically -- the last
        vertex connects back to the first -- so list each corner once, in
        order, walking the perimeter either direction.

        Reuses add_line_obstacle per edge, so thickness and rasterisation
        behave identically to a manually-drawn line.
        """
        if len(points) < 3:
            raise ValueError("a polygon needs at least 3 vertices")
        thickness_m = thickness_m or self.resolution * 2
        n = len(points)
        for i in range(n):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % n]
            self.add_line_obstacle(x1, y1, x2, y2, thickness_m=thickness_m)

    def mark_polygon_free(self, points):
        """Mark every pixel whose centre lies inside the polygon as free.

        Standard even-odd ray-casting test, vectorised over the whole grid at
        once. Works for any simple polygon regardless of winding direction or
        convexity -- an L-shape's reflex corner needs no special handling.
        Call this before add_polygon_wall so the boundary is not overwritten.
        """
        if len(points) < 3:
            raise ValueError("a polygon needs at least 3 vertices")

        cols = np.arange(self.px_width)
        rows = np.arange(self.px_height)
        # Pixel-CENTRE world coordinates (+0.5), not corners, so a boundary
        # running exactly along a pixel edge does not flicker in or out
        # depending on floating-point rounding.
        xs = self.origin[0] + (cols + 0.5) * self.resolution
        ys = self.origin[1] + (self.px_height - 1 - rows + 0.5) * self.resolution
        x_grid, y_grid = np.meshgrid(xs, ys)

        inside = _points_in_polygon(x_grid, y_grid, points)
        self.grid[inside] = FREE

    def _fill_rect(self, x, y, w, h, value):
        r0, r1 = self._row_range(y, h)
        c0, c1 = self._col_range(x, w)
        self.grid[r0:r1, c0:c1] = value

    # ---- checking your work -------------------------------------------------

    def preview(self, max_cols=100):
        """ASCII rendering: '.' free, '#' occupied, '?' unknown.

        Downsamples to fit a terminal rather than printing one character per
        pixel, which for anything but a tiny map would scroll off-screen.
        Check this before saving -- it is much faster than round-tripping
        through RViz to discover a wall is in the wrong place.
        """
        step = max(self.px_width // max_cols, 1)
        small = self.grid[::step * 2, ::step]   # rows sampled sparser: terminal
                                                 # cells are roughly 2x taller than wide
        lines = ["".join(_ASCII[v] for v in row) for row in small]
        legend = f"{self.width_m:g}m x {self.height_m:g}m, {self.resolution:g} m/px  ('.' free '#' wall '?' unknown)"
        return legend + "\n" + "\n".join(lines)

    def occupied_fraction(self):
        return float((self.grid == OCCUPIED).mean())

    # ---- nav2 output --------------------------------------------------------

    def save(self, path_stem, image_format="pgm"):
        """Write ``<path_stem>.yaml`` and ``<path_stem>.<image_format>``.

        image_format: "pgm" (nav2's native, no extra dependency) or "png"
        (needs Pillow, easier to inspect/edit in a normal image viewer).
        """
        path_stem = os.path.expanduser(path_stem)
        image_name = f"{os.path.basename(path_stem)}.{image_format}"
        image_path = f"{path_stem}.{image_format}"
        yaml_path = f"{path_stem}.yaml"

        # Image row 0 must be the TOP of the map for map_server; self.grid is
        # already stored that way (see _to_px), so no flip needed here.
        if image_format == "pgm":
            self._write_pgm(image_path)
        elif image_format == "png":
            if not _HAVE_PIL:
                raise RuntimeError("Pillow is required for png output; use "
                                   "image_format='pgm' instead")
            Image.fromarray(self.grid, mode="L").save(image_path)
        else:
            raise ValueError("image_format must be 'pgm' or 'png'")

        with open(yaml_path, "w") as f:
            f.write(
                f"image: {image_name}\n"
                f"resolution: {self.resolution}\n"
                f"origin: [{self.origin[0]}, {self.origin[1]}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )
        return yaml_path, image_path

    def _write_pgm(self, path):
        """Raw binary PGM (P5) -- no dependency beyond the standard library."""
        header = f"P5\n{self.px_width} {self.px_height}\n255\n".encode("ascii")
        with open(path, "wb") as f:
            f.write(header)
            f.write(self.grid.tobytes())


def _points_in_polygon(x, y, polygon):
    """Even-odd ray-casting test, vectorised. x, y: same-shape arrays.

    Casts a horizontal ray from each point and counts polygon edge crossings;
    odd means inside. Works for convex and reflex (concave) corners alike,
    which is the case an L-shaped site needs and a naive bounding-box test
    cannot express.
    """
    inside = np.zeros(x.shape, dtype=bool)
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        crosses = (yi > y) != (yj > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= crosses & (x < x_intersect)
        j = i
    return inside


def walk_to_vertices(start, heading_deg, steps):
    """Turn a perimeter walk into absolute (x, y) vertices.

    This is the natural way to enter measurements taken by walking a site's
    boundary with a tape measure: at each corner you know how far you are
    about to walk and which way you turned, not the vertex's coordinates.

    Args:
        start: (x, y) in metres, where the walk begins.
        heading_deg: starting direction, standard math convention -- 0 is
            +x (east), 90 is +y (north), degrees, increasing counterclockwise.
        steps: list of (distance_m, turn_deg) pairs. distance_m is how far
            you walk BEFORE turning; turn_deg is how much you turn at the
            corner you just reached, positive for a left (counterclockwise)
            turn, negative for right. A convex corner on a rectilinear
            (right-angle) site is +90 or -90 depending on which way you are
            walking around it; a REFLEX corner -- the inward notch of an
            L-shape -- is the opposite sign of its neighbours. Getting a
            single sign wrong here is exactly what turns an L into the wrong
            shape, so check the result with preview() before trusting it.

    Returns:
        A list of vertices, one per step, NOT including the repeated closing
        vertex -- pass this straight to add_polygon_wall / mark_polygon_free,
        which close the loop themselves.
    """
    x, y = start
    heading = float(heading_deg)
    vertices = []
    for distance, turn in steps:
        x += distance * np.cos(np.radians(heading))
        y += distance * np.sin(np.radians(heading))
        vertices.append((x, y))
        heading += turn
    return vertices


def _cli():
    import argparse

    p = argparse.ArgumentParser(
        description="Generate a blank, bordered nav2 map from known site "
                    "dimensions. Edit the .pgm/.png afterwards in any image "
                    "editor for anything more detailed than a rectangle.")
    p.add_argument("width_m", type=float, help="site width, metres")
    p.add_argument("height_m", type=float, help="site depth, metres")
    p.add_argument("out", help="output path stem, e.g. ~/maps/pit_map")
    p.add_argument("--resolution", type=float, default=0.05,
                   help="metres per pixel (default %(default)s)")
    p.add_argument("--border", type=float, default=0.2,
                   help="perimeter wall thickness, metres (default %(default)s)")
    p.add_argument("--format", choices=("pgm", "png"), default="pgm")
    p.add_argument("--no-preview", action="store_true")
    args = p.parse_args()

    m = OccupancyMap(args.width_m, args.height_m, resolution=args.resolution)
    m.mark_free_rect(0, 0, args.width_m, args.height_m)
    m.add_border_wall(args.border)

    if not args.no_preview:
        print(m.preview())
        print()

    yaml_path, image_path = m.save(args.out, image_format=args.format)
    print(f"wrote {yaml_path}")
    print(f"wrote {image_path}")
    print(f"\nedit {image_path} in any image editor to add real obstacles, "
          f"then re-check with:\n"
          f"  python3 -c \"from occupancy_map import OccupancyMap; "
          f"import numpy as np; from PIL import Image; "
          f"g = np.array(Image.open('{image_path}').convert('L')); "
          f"print(g.shape)\"")


if __name__ == "__main__":
    _cli()
