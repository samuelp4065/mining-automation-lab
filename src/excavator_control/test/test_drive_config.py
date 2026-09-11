"""Tests for the skid-steer mixing shared by motor_node and track_odometry.

These two nodes must agree: one commands the tracks, the other integrates
them. If they disagree the machine drives correctly while reporting that it
went somewhere else, and nav2 trusts the report.
"""

import pytest

from excavator_control.drive_config import (
    LEFT_SIGN, RIGHT_SIGN, scale_to_max_rpm, twist_to_track_speeds,
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
    """Limiting must scale both tracks together, not straighten the turn.

    Calls the real scale_to_max_rpm rather than reimplementing it. An earlier
    version of this test duplicated the maths, so it kept passing while
    motor_node clamped each track independently and silently flattened every
    arc at speed.
    """
    max_rpm = 60.0
    v_left, v_right = twist_to_track_speeds(1.0, 1.0, SEP)
    rpm_left, rpm_right = v_left * 100.0, v_right * 100.0
    ratio_before = rpm_left / rpm_right
    assert max(abs(rpm_left), abs(rpm_right)) > max_rpm, "test needs saturation"

    scaled_left, scaled_right = scale_to_max_rpm(rpm_left, rpm_right, max_rpm)

    assert scaled_left / scaled_right == pytest.approx(ratio_before)
    assert max(abs(scaled_left), abs(scaled_right)) == pytest.approx(max_rpm)


@pytest.mark.parametrize("speed", [0.1, 0.2, 0.3, 0.5, 1.0, 3.0])
def test_arc_ratio_survives_any_speed(speed):
    """The inner/outer ratio must not drift as commanded speed rises.

    This is the regression that prompted scale_to_max_rpm: with a per-track
    clamp the ratio crept from 0.2 toward 1.0 as speed increased, until the
    machine drove straight while still being told to arc.
    """
    from excavator_control.teleop_keyboard import arc_to_twist

    max_rpm, rpm_per_mps, ratio = 20.0, 100.0, 0.2
    v, w = arc_to_twist("forward", "left", ratio, speed, SEP)
    v_left, v_right = twist_to_track_speeds(v, w, SEP)
    rpm_left, rpm_right = scale_to_max_rpm(
        v_left * rpm_per_mps, v_right * rpm_per_mps, max_rpm
    )

    assert rpm_left / rpm_right == pytest.approx(ratio, abs=1e-9)
    assert max(abs(rpm_left), abs(rpm_right)) <= max_rpm + 1e-9


def test_scaling_is_a_no_op_below_the_ceiling():
    assert scale_to_max_rpm(10.0, 20.0, 60.0) == (10.0, 20.0)


def test_scaling_handles_a_stopped_machine():
    assert scale_to_max_rpm(0.0, 0.0, 60.0) == (0.0, 0.0)


# --- teleop key bindings ----------------------------------------------------

from excavator_control.teleop_keyboard import (  # noqa: E402
    ARC_BINDINGS, MOVE_BINDINGS, SPEED_BINDINGS, arc_to_twist,
)

SPEED = 0.3


def arc_tracks(key, speed=SPEED, separation=SEP):
    """Track speeds an arc key produces, via the same path the node uses."""
    travel, inner, ratio = ARC_BINDINGS[key]
    v, w = arc_to_twist(travel, inner, ratio, speed, separation)
    return twist_to_track_speeds(v, w, separation)


def test_no_key_is_bound_twice():
    """A key in two tables would make behaviour depend on check order."""
    tables = [set(MOVE_BINDINGS), set(ARC_BINDINGS), set(SPEED_BINDINGS)]
    for i, a in enumerate(tables):
        for b in tables[i + 1:]:
            assert not (a & b), f"key bound in two tables: {a & b}"


@pytest.mark.parametrize("key", list(ARC_BINDINGS))
@pytest.mark.parametrize("speed", [0.05, 0.3, 1.0, 3.0])
def test_arc_keys_never_reverse_a_track(key, speed):
    """The whole point of the ratio parameterisation.

    Unlike a clamp, this must hold at every speed without any guarding code,
    because the inner track is defined as a fraction of the outer one.
    """
    left, right = arc_tracks(key, speed=speed)
    same_way = (left >= -1e-12 and right >= -1e-12) or \
               (left <= 1e-12 and right <= 1e-12)
    assert same_way, f"{key} at speed {speed}: left={left:.4f} right={right:.4f}"


@pytest.mark.parametrize("key", list(ARC_BINDINGS))
def test_inner_track_runs_at_the_requested_ratio(key):
    """The number in the table means what it says."""
    _, inner, ratio = ARC_BINDINGS[key]
    left, right = arc_tracks(key)
    inner_speed, outer_speed = (left, right) if inner == "left" else (right, left)
    assert inner_speed == pytest.approx(ratio * outer_speed, abs=1e-12)


@pytest.mark.parametrize("key", list(ARC_BINDINGS))
def test_outer_track_runs_at_full_speed(key):
    """Speed is defined as the outer track's speed."""
    _, inner, _ = ARC_BINDINGS[key]
    left, right = arc_tracks(key)
    outer = right if inner == "left" else left
    assert abs(outer) == pytest.approx(SPEED)


def test_ratio_one_drives_straight():
    v, w = arc_to_twist("forward", "left", 1.0, SPEED, SEP)
    assert w == pytest.approx(0.0)
    assert v == pytest.approx(SPEED)


def test_ratio_zero_stops_the_inner_track_without_reversing_it():
    v, w = arc_to_twist("forward", "left", 0.0, SPEED, SEP)
    left, right = twist_to_track_speeds(v, w, SEP)
    assert left == pytest.approx(0.0)
    assert right == pytest.approx(SPEED)


def test_ratio_outside_zero_to_one_is_clamped():
    """A negative ratio would mean a reversing inner track; refuse it."""
    assert arc_to_twist("forward", "left", -0.5, SPEED, SEP) == \
           arc_to_twist("forward", "left", 0.0, SPEED, SEP)
    assert arc_to_twist("forward", "left", 2.0, SPEED, SEP) == \
           arc_to_twist("forward", "left", 1.0, SPEED, SEP)


def test_left_and_right_arcs_steer_opposite_ways():
    _, w_left = arc_to_twist("forward", "left", 0.2, SPEED, SEP)
    _, w_right = arc_to_twist("forward", "right", 0.2, SPEED, SEP)
    assert w_left == pytest.approx(-w_right)
    assert w_left > 0        # slowing the left track turns left


def test_reverse_arcs_travel_backwards():
    for key in (".", "m"):
        left, right = arc_tracks(key)
        assert left < 0 and right < 0


def test_every_steering_direction_is_reachable():
    """Forward-left, forward-right, reverse-left and reverse-right all exist."""
    assert {(travel, inner) for travel, inner, _ in ARC_BINDINGS.values()} == {
        ("forward", "left"), ("forward", "right"),
        ("reverse", "left"), ("reverse", "right"),
    }


def test_spin_keys_still_counter_rotate():
    """The arc parameterisation must not take pivoting away."""
    for key in ("j", "l"):
        _, turn = MOVE_BINDINGS[key]
        left, right = twist_to_track_speeds(0.0, turn * 0.8, SEP)
        assert left * right < 0, "j/l must drive the tracks opposite ways"


def test_stop_keys_are_zero():
    assert MOVE_BINDINGS["k"] == (0.0, 0.0)
    assert MOVE_BINDINGS[" "] == (0.0, 0.0)
