"""Tests for the skid-steer mixing shared by motor_node and track_odometry.

These two nodes must agree: one commands the tracks, the other integrates
them. If they disagree the machine drives correctly while reporting that it
went somewhere else, and nav2 trusts the report.
"""

import pytest

from excavator_control.drive_config import (
    LEFT_SIGN, RIGHT_SIGN, twist_to_track_speeds,
)

SEP = 0.36


def integrate(v_left, v_right, separation=SEP):
    """What track_odometry computes from track speeds -- the inverse."""
    return (v_left + v_right) / 2.0, (v_right - v_left) / separation


@pytest.mark.parametrize("v,w", [
    (0.5, 0.0), (0.0, 1.0), (0.5, 0.5), (-0.3, 0.2), (0.0, -1.5), (1.0, -0.4),
])
def test_mixing_round_trips(v, w):
    """Commanding a twist and integrating it back must return that twist."""
    v_left, v_right = twist_to_track_speeds(v, w, SEP)
    v_back, w_back = integrate(v_left, v_right)
    assert v_back == pytest.approx(v, abs=1e-12)
    assert w_back == pytest.approx(w, abs=1e-12)


def test_straight_drive_moves_both_tracks_equally():
    assert twist_to_track_speeds(0.5, 0.0, SEP) == pytest.approx((0.5, 0.5))


def test_spin_in_place_is_equal_and_opposite():
    v_left, v_right = twist_to_track_speeds(0.0, 1.0, SEP)
    assert v_left == pytest.approx(-v_right)
    assert v_right > 0  # positive yaw is counter-clockwise: right track forward


def test_left_servo_is_mirrored():
    """Forward motion needs a negative RPM on the left servo.

    Inherited from scripts/keyboard_track.py, the teleop that has actually
    driven this machine. Flipping this drives the excavator backwards.
    """
    v_left, v_right = twist_to_track_speeds(1.0, 0.0, SEP)
    assert LEFT_SIGN * v_left < 0
    assert RIGHT_SIGN * v_right > 0


def test_wider_track_separation_needs_more_differential():
    """A wider machine must spin its tracks faster for the same yaw rate."""
    narrow = twist_to_track_speeds(0.0, 1.0, 0.30)[1]
    wide = twist_to_track_speeds(0.0, 1.0, 0.50)[1]
    assert wide > narrow


def test_saturation_scaling_preserves_turn_shape():
    """Clamping must scale both tracks together, not straighten the turn.

    Mirrors the scaling in MotorNode._on_cmd_vel: clipping each track
    independently would make a saturated turn drive straighter than commanded,
    which is exactly the wrong failure at speed.
    """
    max_rpm = 60.0
    v_left, v_right = twist_to_track_speeds(1.0, 1.0, SEP)
    rpm_left, rpm_right = v_left * 100.0, v_right * 100.0
    ratio_before = rpm_left / rpm_right

    peak = max(abs(rpm_left), abs(rpm_right))
    assert peak > max_rpm, "test needs a saturating command"
    scale = max_rpm / peak
    rpm_left *= scale
    rpm_right *= scale

    assert rpm_left / rpm_right == pytest.approx(ratio_before)
    assert max(abs(rpm_left), abs(rpm_right)) == pytest.approx(max_rpm)
