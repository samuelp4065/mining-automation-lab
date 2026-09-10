"""Excavator arm kinematics and joint calibration (no ROS, no hardware)."""

from excavator_kinematics.planar_ik import (
    DEFAULT_LINKS, LinkLengths, fk_3link_planar, ik_3link_planar,
    is_reachable, joint_points,
)
from excavator_kinematics.calibration import (
    JOINT_LIMITS_DEG, JointLimitError, angles_to_commands, check_joint_limits,
)

__all__ = [
    "DEFAULT_LINKS", "LinkLengths", "ik_3link_planar", "fk_3link_planar",
    "is_reachable", "joint_points", "angles_to_commands", "check_joint_limits",
    "JOINT_LIMITS_DEG", "JointLimitError",
]
