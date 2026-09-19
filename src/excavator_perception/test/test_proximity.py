"""Tests for the proximity interlock's decision logic.

No ROS, no camera. Every case here is one you would otherwise have to
reproduce by walking in front of a moving excavator.
"""

import numpy as np
import pytest

from excavator_perception.proximity import (
    ProximityConfig, evaluate, patch_distance,
)

CFG = ProximityConfig(stop_below_m=1.0, release_above_m=1.2,
                      min_valid_fraction=0.25)


def frame(distance, shape=(48, 64), dropout=0.0, rng=None):
    """A depth frame at a uniform distance, optionally punched with holes."""
    m = np.full(shape, distance, dtype=np.float32)
    if dropout:
        rng = rng or np.random.default_rng(0)
        m[rng.random(shape) < dropout] = np.nan
    return m


# --- patch sampling --------------------------------------------------------

def test_patch_returns_the_distance_and_full_validity():
    d, frac = patch_distance(frame(1.5), centre=(32, 24), size=9)
    assert d == pytest.approx(1.5)
    assert frac == pytest.approx(1.0)


def test_patch_median_ignores_a_single_hot_pixel():
    """One bad pixel must not stop the machine.

    A minimum would trip on it every frame it appears; the median does not.
    """
    m = frame(1.5)
    m[24, 32] = 0.05
    d, _ = patch_distance(m, centre=(32, 24), size=9)
    assert d == pytest.approx(1.5)


def test_patch_median_is_not_dragged_by_background_through_a_gap():
    """A mean would be; a near object with a hole in it must still read near."""
    m = frame(0.8)
    m[20:24, 28:36] = 6.0          # far background showing through
    d, _ = patch_distance(m, centre=(32, 24), size=9)
    assert d == pytest.approx(0.8)


def test_patch_reports_dropouts_as_reduced_validity():
    d, frac = patch_distance(frame(1.5, dropout=0.5), centre=(32, 24), size=9)
    assert np.isfinite(d)
    assert 0.2 < frac < 0.8


def test_patch_with_no_returns_at_all():
    d, frac = patch_distance(frame(np.nan), centre=(32, 24), size=9)
    assert np.isnan(d)
    assert frac == 0.0


def test_patch_near_an_edge_does_not_crash():
    for centre in [(0, 0), (63, 47), (-5, 100)]:
        d, frac = patch_distance(frame(2.0), centre=centre, size=9)
        assert np.isfinite(d)
        assert frac > 0


# --- the decision ----------------------------------------------------------

def test_far_object_is_clear():
    assert evaluate(2.0, 1.0, CFG, previously_clear=True).clear


def test_close_object_stops_the_machine():
    state = evaluate(0.8, 1.0, CFG, previously_clear=True)
    assert not state.clear
    assert "0.80" in state.reason


def test_exactly_at_the_stop_threshold_is_still_clear():
    """The threshold is 'stop below', so 1.00 m itself is permitted."""
    assert evaluate(1.0, 1.0, CFG, previously_clear=True).clear


def test_hysteresis_keeps_the_machine_stopped_until_well_clear():
    """Between the two thresholds, the previous state decides.

    Without this the machine chatters when an obstacle sits near the limit:
    stop, creep, clear, stop, at whatever rate the camera runs.
    """
    assert not evaluate(1.1, 1.0, CFG, previously_clear=False).clear
    assert evaluate(1.1, 1.0, CFG, previously_clear=True).clear


def test_hysteresis_releases_above_the_upper_threshold():
    assert evaluate(1.25, 1.0, CFG, previously_clear=False).clear


def test_no_data_is_treated_as_blocked_not_clear():
    """Fail safe. A sudden loss of returns most likely means something close
    and dark filling the frame -- exactly when driving on would be worst."""
    state = evaluate(float("nan"), 0.0, CFG, previously_clear=True)
    assert not state.clear
    assert "insufficient data" in state.reason


def test_too_few_valid_pixels_is_treated_as_blocked():
    state = evaluate(2.0, 0.10, CFG, previously_clear=True)
    assert not state.clear, "a confident median from 10% of a patch is not trustworthy"


def test_enough_valid_pixels_is_trusted():
    assert evaluate(2.0, 0.30, CFG, previously_clear=True).clear


def test_config_rejects_thresholds_that_would_chatter():
    with pytest.raises(ValueError, match="chatter"):
        ProximityConfig(stop_below_m=1.0, release_above_m=1.0)
    with pytest.raises(ValueError):
        ProximityConfig(stop_below_m=1.2, release_above_m=1.0)


# --- an approach, end to end ----------------------------------------------

def test_an_object_approaching_then_retreating():
    """Walk an object in and back out; the interlock must latch and release
    at the right points, and not oscillate in the band between them."""
    approach = [2.0, 1.6, 1.3, 1.15, 1.05, 0.95, 0.8, 0.6]
    retreat = [0.8, 0.95, 1.05, 1.15, 1.25, 1.5]

    clear = True
    log = []
    for d in approach + retreat:
        clear = evaluate(d, 1.0, CFG, previously_clear=clear).clear
        log.append(clear)

    # Stops once, below 1.0 m, and stays stopped through the band on the way out.
    assert log[:5] == [True, True, True, True, True]     # down to 1.05 m
    assert log[5:8] == [False, False, False]             # 0.95 and closer
    assert log[8:12] == [False, False, False, False]     # 0.8 .. 1.15 still latched
    assert log[12:] == [True, True]                      # released above 1.20
