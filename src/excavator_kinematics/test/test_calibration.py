"""Calibration round-trips and mechanical travel limits."""

import numpy as np
import pytest

from excavator_kinematics.calibration import (
    JOINT_LIMITS_DEG, JointLimitError, angle_to_boom_cmd, angle_to_bucket_cmd,
    angle_to_stick_cmd, angles_to_commands, boom_cmd_to_angle,
    bucket_cmd_to_angle, check_joint_limits, stick_cmd_to_angle,
)

PAIRS = [
    (angle_to_boom_cmd, boom_cmd_to_angle, 45.0),
    (angle_to_stick_cmd, stick_cmd_to_angle, -80.0),
    (angle_to_bucket_cmd, bucket_cmd_to_angle, -100.0),
]


@pytest.mark.parametrize("forward,inverse,angle", PAIRS)
def test_calibration_round_trip(forward, inverse, angle):
    assert inverse(forward(angle)) == pytest.approx(angle)


def test_stick_and_bucket_slopes_are_negative():
    """These actuators travel backwards; a sign flip inverts one joint."""
    assert angle_to_stick_cmd(-70.0) < angle_to_stick_cmd(-80.0)
    assert angle_to_bucket_cmd(-90.0) < angle_to_bucket_cmd(-100.0)
    assert angle_to_boom_cmd(50.0) > angle_to_boom_cmd(40.0)


@pytest.mark.parametrize("joint,cmd_fn", [
    ("boom", angle_to_boom_cmd),
    ("stick", angle_to_stick_cmd),
    ("bucket", angle_to_bucket_cmd),
])
def test_travel_limits_map_to_full_servo_range(joint, cmd_fn):
    """The documented limits are exactly where the command saturates at +/-90."""
    lo, hi = JOINT_LIMITS_DEG[joint]
    assert sorted([abs(cmd_fn(lo)), abs(cmd_fn(hi))]) == pytest.approx([90.0, 90.0])


def test_check_joint_limits_accepts_in_range_pose():
    check_joint_limits(40.0, -80.0, -100.0)


@pytest.mark.parametrize("pose", [
    (90.0, -80.0, -100.0),    # boom past travel
    (40.0, 0.0, -100.0),      # stick past travel
    (40.0, -80.0, 0.0),       # bucket past travel
])
def test_check_joint_limits_rejects_out_of_range(pose):
    with pytest.raises(JointLimitError):
        check_joint_limits(*pose)


def test_angles_to_commands_accepts_radians_by_default():
    rad = np.deg2rad([40.0, -80.0, -100.0])
    assert angles_to_commands(*rad) == pytest.approx(
        angles_to_commands(40.0, -80.0, -100.0, degrees=True)
    )


def test_angles_to_commands_enforces_limits_by_default():
    with pytest.raises(JointLimitError):
        angles_to_commands(90.0, -80.0, -100.0, degrees=True)
    # ...and can be told not to, for diagnostics
    angles_to_commands(90.0, -80.0, -100.0, degrees=True, check_limits=False)


def test_commands_stay_within_servo_range_for_valid_pose():
    cmds = angles_to_commands(40.0, -80.0, -100.0, degrees=True)
    assert all(-90.0 <= c <= 90.0 for c in cmds)
