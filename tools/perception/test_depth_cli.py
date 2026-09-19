"""Tests for the depth pipeline's pure maths.

Everything here runs without a camera, using synthetic z16 frames. That is
deliberate: the parts most likely to be wrong -- unit conversion, invalid-pixel
handling, nearest-surface selection -- are exactly the parts that need no
hardware to exercise, and the Orin is usually nowhere near the excavator when
this code is being edited.

    python3 -m pytest tools/perception/ -q
"""

import warnings

import numpy as np
import pytest

import depth_cli as d


class FakeFrame:
    """Stands in for an rs.depth_frame; only get_data() is used."""

    def __init__(self, raw):
        self._raw = raw

    def get_data(self):
        return self._raw


def scene():
    """A wall at 1.5 m, a pile at 0.8 m, and a patch with no return."""
    raw = np.full((48, 64), 1500, dtype=np.uint16)
    raw[30:, 20:44] = 800      # pile
    raw[10:20, 50:60] = 0      # dropout
    return raw


def test_depth_scale_converts_to_metres():
    m = d.as_metres(FakeFrame(scene()), 0.001)
    assert m[5, 5] == pytest.approx(1.5, abs=1e-6)
    assert m[40, 30] == pytest.approx(0.8, abs=1e-6)


def test_a_different_depth_scale_is_honoured():
    """depth_scale is per-device and must never be assumed to be 0.001."""
    m = d.as_metres(FakeFrame(scene()), 0.0005)
    assert m[5, 5] == pytest.approx(0.75, abs=1e-6)


def test_zero_pixels_become_nan_not_zero_distance():
    """Zero in a z16 frame means 'no measurement', not 'touching the lens'.

    Letting zeros through as 0.0 is what makes naive nearest-distance code
    report an obstacle pressed against the camera.
    """
    m = d.as_metres(FakeFrame(scene()), 0.001)
    assert np.isnan(m[15, 55])
    assert not np.isnan(m[5, 5])


def test_nearest_surface_ignores_dropouts():
    m = d.as_metres(FakeFrame(scene()), 0.001)
    assert float(np.nanmin(m)) == pytest.approx(0.8, abs=1e-6)


def test_column_profile_picks_the_pile_not_the_wall_behind_it():
    """The band's nearest return per column is the face to dig."""
    m = d.as_metres(FakeFrame(scene()), 0.001)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        profile = np.nanmin(m[28:34, :], axis=0)

    assert profile[30] == pytest.approx(0.8, abs=1e-6)   # over the pile
    assert profile[5] == pytest.approx(1.5, abs=1e-6)    # over the wall


def test_a_fully_empty_column_stays_nan():
    """A hole spanning the whole band is normal and must not crash."""
    blank = np.full((6, 4), np.nan, dtype=np.float32)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        profile = np.nanmin(blank, axis=0)
    assert np.isnan(profile).all()


def test_render_marks_near_far_and_missing_differently():
    m = d.as_metres(FakeFrame(scene()), 0.001)
    art = d.render(m, near=0.3, far=2.0, cols=56, rows=12, color=False)
    lines = art.splitlines()

    assert len(lines) == 12
    assert all(len(line) == 56 for line in lines)
    assert " " in art, "dropouts should render blank"
    near_char = lines[-1][30]
    far_char = lines[0][5]
    assert d._RAMP.index(near_char) > d._RAMP.index(far_char), \
        "closer surfaces must use a denser ramp character"


def test_render_handles_an_entirely_invalid_frame():
    m = np.full((20, 20), np.nan, dtype=np.float32)
    art = d.render(m, near=0.3, far=2.0, cols=10, rows=5, color=False)
    assert art.strip() == ""


def test_range_limits_match_the_laserscan_node():
    """These must stay in step with sensors.launch.py, or the costmap and
    this tool will disagree about what the camera can see."""
    assert d.RANGE_MIN_M == 0.30
    assert d.RANGE_MAX_M == 8.0
