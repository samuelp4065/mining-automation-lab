"""Closed-form kinematics for the excavator's 3-link planar arm.

This is the single source of truth for the arm solve. It deliberately has no
ROS and no hardware imports so that it can be used from ROS 2 nodes, the
Matplotlib simulator, and plain REPL tools alike, and unit-tested without
either a robot or a running ROS graph.

Geometry
--------
The boom/stick/bucket form a 3-DOF planar manipulator in the vertical x-z
plane of the cab. Cab rotation (the swingdrive) is a separate DOF and is
handled in :mod:`excavator_kinematics.swing`, not here.

``theta1`` is absolute, measured from +x. ``theta2`` and ``theta3`` are each
relative to the previous link. The tool angle is therefore the joint sum::

    phi = theta1 + theta2 + theta3

which is what makes the arm solvable in closed form: the wrist can be backed
off the tip by L3 along phi, leaving a 2-link problem with a known target.

Units: lengths in centimetres, angles in radians. Callers working in degrees
convert at their own boundary.
"""

from dataclasses import dataclass

import numpy as np

__all__ = ["LinkLengths", "DEFAULT_LINKS", "ik_3link_planar", "fk_3link_planar", "is_reachable"]


@dataclass(frozen=True)
class LinkLengths:
    """Arm link lengths in centimetres."""

    L1: float = 46.0  # boom, shoulder -> elbow
    L2: float = 20.0  # stick/arm,  elbow -> wrist
    L3: float = 12.0  # bucket/tool, wrist -> tip

    def as_tuple(self):
        return self.L1, self.L2, self.L3


#: Measured link lengths of this machine.
DEFAULT_LINKS = LinkLengths()


def ik_3link_planar(x, z, phi, L1=None, L2=None, L3=None, elbow="up"):
    """Solve joint angles for a bucket-tip pose.

    Args:
        x, z: bucket **tip** position in the arm base frame, centimetres.
            The base joint is the origin and the ground is z = 0.
        phi: tool orientation in radians, the absolute angle of the bucket
            direction from +x. ``phi=0`` points the tool horizontally
            forward; negative phi tilts it down.
        L1, L2, L3: link lengths in centimetres. Default to
            :data:`DEFAULT_LINKS`.
        elbow: ``"up"`` or ``"down"``. **Defaults to "up"**, which is the
            configuration every piece of hardware code on this machine has
            always used. (The legacy ``InverseKinematics/IK.py`` defaulted to
            ``"down"`` and was the odd one out; it never drove the arm.)

    Returns:
        ``(theta1, theta2, theta3)`` in radians.

    Raises:
        ValueError: if the wrist point is outside the reachable annulus, or
            if ``elbow`` is not "up"/"down".

    Note:
        Reachability is checked on the **wrist**, not the tip. An otherwise
        comfortable tip position can be unreachable purely because an
        aggressive ``phi`` swings the wrist out of range -- try a shallower
        tool angle before concluding the target is too far away.
    """
    L1 = DEFAULT_LINKS.L1 if L1 is None else L1
    L2 = DEFAULT_LINKS.L2 if L2 is None else L2
    L3 = DEFAULT_LINKS.L3 if L3 is None else L3

    # Step 1: back the wrist off the tip along the tool direction.
    xw = x - L3 * np.cos(phi)
    zw = z - L3 * np.sin(phi)

    # Step 2: solve boom + stick as a 2-link arm reaching that wrist.
    rw2 = xw**2 + zw**2
    rw = np.sqrt(rw2)

    if rw > (L1 + L2) or rw < abs(L1 - L2):
        raise ValueError("Target unreachable for given L1, L2, L3.")

    c2 = (rw2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = np.clip(c2, -1.0, 1.0)  # absorb float error at the workspace boundary

    if elbow == "up":
        s2 = -np.sqrt(1 - c2**2)
    elif elbow == "down":
        s2 = +np.sqrt(1 - c2**2)
    else:
        raise ValueError("elbow must be 'up' or 'down'.")

    theta2 = np.arctan2(s2, c2)

    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = np.arctan2(zw, xw) - np.arctan2(k2, k1)

    # Step 3: the bucket makes up whatever is left of the tool angle.
    theta3 = phi - (theta1 + theta2)

    return theta1, theta2, theta3


def fk_3link_planar(theta1, theta2, theta3, L1=None, L2=None, L3=None):
    """Forward kinematics: joint angles (radians) -> ``(x, z, phi)``."""
    L1 = DEFAULT_LINKS.L1 if L1 is None else L1
    L2 = DEFAULT_LINKS.L2 if L2 is None else L2
    L3 = DEFAULT_LINKS.L3 if L3 is None else L3

    x1 = L1 * np.cos(theta1)
    z1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    z2 = z1 + L2 * np.sin(theta1 + theta2)

    xt = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    zt = z2 + L3 * np.sin(theta1 + theta2 + theta3)

    return xt, zt, theta1 + theta2 + theta3


def joint_points(theta1, theta2, theta3, L1=None, L2=None, L3=None):
    """Return ``(base, elbow, wrist, tip)`` as (x, z) pairs, for drawing."""
    L1 = DEFAULT_LINKS.L1 if L1 is None else L1
    L2 = DEFAULT_LINKS.L2 if L2 is None else L2
    L3 = DEFAULT_LINKS.L3 if L3 is None else L3

    base = (0.0, 0.0)
    elbow = (L1 * np.cos(theta1), L1 * np.sin(theta1))
    wrist = (elbow[0] + L2 * np.cos(theta1 + theta2),
             elbow[1] + L2 * np.sin(theta1 + theta2))
    tip = (wrist[0] + L3 * np.cos(theta1 + theta2 + theta3),
           wrist[1] + L3 * np.sin(theta1 + theta2 + theta3))
    return base, elbow, wrist, tip


def is_reachable(x, z, phi, L1=None, L2=None, L3=None):
    """True if :func:`ik_3link_planar` would find a solution for this pose."""
    try:
        ik_3link_planar(x, z, phi, L1, L2, L3)
    except ValueError:
        return False
    return True
