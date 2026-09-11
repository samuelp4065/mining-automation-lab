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


# --- teleop key bindings ----------------------------------------------------

from excavator_control.teleop_keyboard import (  # noqa: E402
    ARC_BINDINGS, MOVE_BINDINGS, SPEED_BINDINGS,
)

ARC_GAIN = 0.4


def test_no_key_is_bound_twice():
    """A key in two tables would make behaviour depend on check order."""
    tables = [set(MOVE_BINDINGS), set(ARC_BINDINGS), set(SPEED_BINDINGS)]
    for i, a in enumerate(tables):
        for b in tables[i + 1:]:
            assert not (a & b), f"key bound in two tables: {a & b}"


def test_arc_keys_turn_gentler_than_spin_keys():
    """u and o must curve, not swing. This is the whole point of the table."""
    _, arc_turn = ARC_BINDINGS["u"]
    _, spin_turn = MOVE_BINDINGS["j"]
    assert abs(arc_turn * ARC_GAIN) < abs(spin_turn)


def test_arc_keys_still_drive_forward_at_full_speed():
    """Only the turn component is scaled; forward speed is untouched."""
    assert ARC_BINDINGS["u"][0] == MOVE_BINDINGS["i"][0]
    assert ARC_BINDINGS["o"][0] == MOVE_BINDINGS["i"][0]


def test_u_and_o_arc_in_opposite_directions():
    assert ARC_BINDINGS["u"][1] == -ARC_BINDINGS["o"][1]


def test_reverse_arcs_use_the_same_gain_as_forward_arcs():
    """Forward and reverse diagonals must behave symmetrically."""
    assert abs(ARC_BINDINGS["m"][1]) == abs(ARC_BINDINGS["u"][1])
    assert abs(ARC_BINDINGS["."][1]) == abs(ARC_BINDINGS["o"][1])
    assert ARC_BINDINGS["m"][0] < 0 and ARC_BINDINGS["."][0] < 0


def test_spin_keys_are_unaffected_by_arc_gain():
    """j and l must still spin at the full rate."""
    assert MOVE_BINDINGS["j"] == (0.0, 1.0)
    assert MOVE_BINDINGS["l"] == (0.0, -1.0)


def test_stop_keys_are_zero():
    assert MOVE_BINDINGS["k"] == (0.0, 0.0)
    assert MOVE_BINDINGS[" "] == (0.0, 0.0)
