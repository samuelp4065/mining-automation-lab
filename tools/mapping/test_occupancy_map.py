"""Tests for the hand-drawn occupancy map builder.

No ROS. The geometry tests exist because an earlier version of this file had
a real fencepost bug -- a 0.5 m border wall came out 2 pixels wide on the
left and 1 pixel wide on the right, from converting a rectangle's two
corners independently through a rounding conversion instead of treating each
axis as one continuous span. Every "exact pixel count" test below is a case
that bug would have failed.

    python3 -m pytest tools/mapping/ -q
"""

import numpy as np
import pytest

from occupancy_map import FREE, OCCUPIED, UNKNOWN, OccupancyMap


def test_dimensions_match_resolution():
    m = OccupancyMap(width_m=10.0, height_m=6.0, resolution=0.5)
    assert (m.px_width, m.px_height) == (20, 12)


def test_starts_entirely_unknown():
    """Unmarked area must read as unknown, not free.

    A hand-drawn map has no scan data backing it up; a forgotten spot
    defaulting to FREE would let nav2 plan a route across ground nobody
    actually confirmed was open.
    """
    m = OccupancyMap(width_m=4.0, height_m=4.0, resolution=1.0)
    assert (m.grid == UNKNOWN).all()


def test_mark_free_rect_fills_exactly_that_area():
    m = OccupancyMap(width_m=4.0, height_m=4.0, resolution=1.0)
    m.mark_free_rect(0, 0, 4, 4)
    assert (m.grid == FREE).all()


# --- the fencepost bug, pinned down ----------------------------------------

def test_border_wall_is_symmetric():
    """A wall of thickness t must be the same pixel width on both sides.

    This is the exact case that failed before: the border came out as
    2 columns on the left, 1 on the right, because each edge was located by
    converting two independent corner points instead of one continuous span.
    """
    m = OccupancyMap(width_m=10.0, height_m=6.0, resolution=0.5)
    m.mark_free_rect(0, 0, 10, 6)
    m.add_border_wall(0.5)

    middle_row = m.grid[m.px_height // 2]
    occupied_cols = np.flatnonzero(middle_row == OCCUPIED)
    assert list(occupied_cols) == [0, m.px_width - 1]

    middle_col = m.grid[:, m.px_width // 2]
    occupied_rows = np.flatnonzero(middle_col == OCCUPIED)
    assert list(occupied_rows) == [0, m.px_height - 1]


def test_rect_area_matches_resolution_exactly():
    """A grid-aligned 2x2 m obstacle at 0.5 m/px must cover exactly 4x4 px."""
    m = OccupancyMap(width_m=8.0, height_m=8.0, resolution=0.5)
    m.add_rect_obstacle(x=2, y=2, w=2, h=2)
    assert int((m.grid == OCCUPIED).sum()) == 16


def test_non_grid_aligned_rect_matches_floor_ceil_math():
    """A wall that does not land on a pixel boundary must still be exact.

    Real-world coordinates rarely land on a clean multiple of the
    resolution, so this path is the one that runs in practice.
    """
    m = OccupancyMap(width_m=8.0, height_m=8.0, resolution=0.3)
    m.add_rect_obstacle(x=1.15, y=1.15, w=1.7, h=1.7)

    c0 = int(np.floor(1.15 / 0.3))
    c1 = int(np.ceil((1.15 + 1.7) / 0.3))
    expected = (c1 - c0) ** 2
    assert int((m.grid == OCCUPIED).sum()) == expected


# --- axis orientation -------------------------------------------------------

def test_y_zero_is_the_bottom_of_the_image():
    """An obstacle near y=0 must land in the LAST image row, not the first.

    Image row 0 is conventionally the top of a picture, but the map's y axis
    points up from the origin -- getting this flip backwards silently
    mirrors every obstacle vertically while producing a file that opens fine
    in an image viewer, which is a bad way to find out.
    """
    m = OccupancyMap(width_m=4.0, height_m=4.0, resolution=1.0)
    m.add_rect_obstacle(x=1, y=0, w=1, h=1)
    assert m.grid[m.px_height - 1, 1] == OCCUPIED
    assert m.grid[0, 1] == UNKNOWN


def test_y_max_is_the_top_of_the_image():
    m = OccupancyMap(width_m=4.0, height_m=4.0, resolution=1.0)
    m.add_rect_obstacle(x=1, y=3, w=1, h=1)
    assert m.grid[0, 1] == OCCUPIED
    assert m.grid[m.px_height - 1, 1] == UNKNOWN


# --- other primitives --------------------------------------------------------

def test_circle_obstacle_is_centred_and_roughly_round():
    m = OccupancyMap(width_m=10.0, height_m=10.0, resolution=0.1)
    m.add_circle_obstacle(cx=5.0, cy=5.0, radius_m=1.0)
    n = int((m.grid == OCCUPIED).sum())
    expected_area_px = np.pi * (1.0 / 0.1) ** 2
    assert n == pytest.approx(expected_area_px, rel=0.05)


def test_line_obstacle_connects_its_endpoints():
    m = OccupancyMap(width_m=10.0, height_m=10.0, resolution=0.2)
    m.add_line_obstacle(1, 1, 8, 1, thickness_m=0.3)
    row, _ = m._point_to_rowcol(1, 1)
    assert m.grid[row, m.grid.shape[1] // 2] == OCCUPIED, \
        "midpoint of the line should be occupied"


def test_rects_outside_the_grid_are_clamped_not_an_error():
    m = OccupancyMap(width_m=4.0, height_m=4.0, resolution=1.0)
    m.add_rect_obstacle(x=-10, y=-10, w=100, h=100)   # absurdly oversized
    assert (m.grid == OCCUPIED).all()


# --- nav2 file output --------------------------------------------------------

def test_pgm_round_trips_byte_exact(tmp_path):
    m = OccupancyMap(width_m=4.0, height_m=3.0, resolution=1.0)
    m.mark_free_rect(0, 0, 4, 3)
    m.add_rect_obstacle(1, 1, 1, 1)

    yaml_path, pgm_path = m.save(str(tmp_path / "map"))

    with open(pgm_path, "rb") as f:
        assert f.readline().strip() == b"P5"
        w, h = (int(v) for v in f.readline().split())
        assert (w, h) == (m.px_width, m.px_height)
        assert f.readline().strip() == b"255"
        data = f.read()

    assert len(data) == w * h
    readback = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
    assert np.array_equal(readback, m.grid)


def test_yaml_matches_nav2_map_server_schema(tmp_path):
    """Field names and format nav2's map_io actually parses.

    Getting a key name wrong here fails silently in the worst way: nav2
    falls back to a default rather than raising, so a typo like
    'occupied_thres' produces a map that loads but classifies obstacles
    wrong everywhere.
    """
    m = OccupancyMap(width_m=2.0, height_m=2.0, resolution=0.5, origin=(1.5, -2.0))
    yaml_path, _ = m.save(str(tmp_path / "map"))

    text = open(yaml_path).read()
    for key in ("image:", "resolution:", "origin:", "negate:",
                "occupied_thresh:", "free_thresh:"):
        assert key in text

    assert "resolution: 0.5" in text
    assert "origin: [1.5, -2.0, 0.0]" in text


def test_resolution_must_be_positive():
    with pytest.raises(ValueError):
        OccupancyMap(width_m=1.0, height_m=1.0, resolution=0.0)
