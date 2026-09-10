"""Per-joint calibration: joint angle <-> actuator command.

What these numbers actually are
-------------------------------
The boom, stick and bucket are **RC linear actuators driven by PWM** through
``gpiozero.AngularServo`` on BCM pins 17, 27 and 22. The functions below map a
joint angle in degrees to the value assigned to ``AngularServo.angle``, which
gpiozero clamps to **[-90, 90]**.

They are *not* LSS encoder counts, despite older docs describing them that way.
The LSS smart-servo bus drives only the two tracks (IDs 0, 1) and the
swingdrive (ID 2); it is not involved in boom/stick/bucket motion at all.

The distinction matters: the [-90, 90] command range implies a hard mechanical
travel limit per joint (see :data:`JOINT_LIMITS_DEG`), which an "encoder count"
reading would not.

The negative slopes on stick and bucket mean those actuators travel backwards
relative to the angle convention. A sign error in one of them inverts exactly
one joint on the real machine.

These constants are empirical to this linkage. If a joint is re-zeroed or a
linkage changes, edit them here -- this module is the only place they live.
"""

import numpy as np

__all__ = [
    "angle_to_boom_cmd", "angle_to_stick_cmd", "angle_to_bucket_cmd",
    "boom_cmd_to_angle", "stick_cmd_to_angle", "bucket_cmd_to_angle",
    "angles_to_commands", "JOINT_LIMITS_DEG", "check_joint_limits",
    "JointLimitError",
]

# Affine calibration: cmd = (angle_deg + OFFSET) * SLOPE
_BOOM_OFFSET, _BOOM_SLOPE = -40.0, 9 / 2          # +4.5   cmd/deg
_STICK_OFFSET, _STICK_SLOPE = 79.0, -90 / 41      # -2.195 cmd/deg
_BUCKET_OFFSET, _BUCKET_SLOPE = 385 / 3, -27 / 25  # -1.08  cmd/deg

#: Joint travel implied by the +/-90 AngularServo command range, in degrees.
JOINT_LIMITS_DEG = {
    "boom": (20.0, 60.0),
    "stick": (-120.0, -38.0),
    "bucket": (-211.666667, -45.0),
}


class JointLimitError(ValueError):
    """Raised when a solved joint angle is outside its mechanical travel."""


def angle_to_boom_cmd(angle_deg):
    """Boom joint angle (deg) -> AngularServo command."""
    return float((angle_deg + _BOOM_OFFSET) * _BOOM_SLOPE)


def angle_to_stick_cmd(angle_deg):
    """Stick joint angle (deg) -> AngularServo command."""
    return float((angle_deg + _STICK_OFFSET) * _STICK_SLOPE)


def angle_to_bucket_cmd(angle_deg):
    """Bucket joint angle (deg) -> AngularServo command."""
    return float((angle_deg + _BUCKET_OFFSET) * _BUCKET_SLOPE)


def boom_cmd_to_angle(cmd):
    """Inverse of :func:`angle_to_boom_cmd`."""
    return float(cmd / _BOOM_SLOPE - _BOOM_OFFSET)


def stick_cmd_to_angle(cmd):
    """Inverse of :func:`angle_to_stick_cmd`."""
    return float(cmd / _STICK_SLOPE - _STICK_OFFSET)


def bucket_cmd_to_angle(cmd):
    """Inverse of :func:`angle_to_bucket_cmd`."""
    return float(cmd / _BUCKET_SLOPE - _BUCKET_OFFSET)


def check_joint_limits(th1_deg, th2_deg, th3_deg):
    """Raise :class:`JointLimitError` if any joint is outside its travel.

    Call this on an IK solution before commanding the actuators. The solver
    happily returns kinematically valid poses the machine physically cannot
    hold; without this check those silently saturate at the actuator end
    stops and the bucket ends up somewhere other than the commanded point.
    """
    for name, value in (("boom", th1_deg), ("stick", th2_deg), ("bucket", th3_deg)):
        lo, hi = JOINT_LIMITS_DEG[name]
        if not (lo - 1e-6) <= value <= (hi + 1e-6):
            raise JointLimitError(
                f"{name} angle {value:.2f} deg outside travel [{lo:.2f}, {hi:.2f}]"
            )


def angles_to_commands(th1, th2, th3, degrees=False, check_limits=True):
    """Convert an IK solution to the three actuator commands.

    Args:
        th1, th2, th3: joint angles, radians unless ``degrees=True``.
        degrees: treat the inputs as degrees.
        check_limits: validate against :data:`JOINT_LIMITS_DEG` first.

    Returns:
        ``(boom_cmd, stick_cmd, bucket_cmd)`` for ``AngularServo.angle``.
    """
    if not degrees:
        th1, th2, th3 = np.rad2deg([th1, th2, th3])

    if check_limits:
        check_joint_limits(th1, th2, th3)

    return (
        angle_to_boom_cmd(th1),
        angle_to_stick_cmd(th2),
        angle_to_bucket_cmd(th3),
    )
