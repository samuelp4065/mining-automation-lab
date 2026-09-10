"""Round-trip and boundary tests for the arm solve.

These replace the ad-hoc ``__main__`` demo blocks that used to serve as the
project's only regression check.
"""

import numpy as np
import pytest

from excavator_kinematics.planar_ik import (
    DEFAULT_LINKS, fk_3link_planar, ik_3link_planar, is_reachable, joint_points,
)

L1, L2, L3 = DEFAULT_LINKS.as_tuple()


@pytest.mark.parametrize("elbow", ["up", "down"])
@pytest.mark.parametrize("pose", [
    (60.0, 10.0, -30.0),
    (55.0, 20.0, 0.0),
    (40.0, -10.0, -45.0),
    (30.0, 30.0, 15.0),
    (65.0, 0.0, -80.0),
])
def test_ik_fk_round_trip(pose, elbow):
    """FK of an IK solution must reproduce the requested tip pose."""
    x, z, phi_deg = pose
    phi = np.deg2rad(phi_deg)
    th1, th2, th3 = ik_3link_planar(x, z, phi, elbow=elbow)
    xt, zt, phit = fk_3link_planar(th1, th2, th3)

    assert xt == pytest.approx(x, abs=1e-9)
    assert zt == pytest.approx(z, abs=1e-9)
    assert phit == pytest.approx(phi, abs=1e-9)


def test_tool_angle_is_joint_sum():
    """phi = theta1 + theta2 + theta3 is the identity the solve relies on."""
    phi = np.deg2rad(-30.0)
    th1, th2, th3 = ik_3link_planar(60.0, 10.0, phi)
    assert th1 + th2 + th3 == pytest.approx(phi, abs=1e-12)


def test_elbow_configs_differ_but_reach_same_tip():
    """Both configurations hit the tip through genuinely different shapes."""
    phi = np.deg2rad(-30.0)
    up = ik_3link_planar(60.0, 10.0, phi, elbow="up")
    down = ik_3link_planar(60.0, 10.0, phi, elbow="down")

    assert up != down
    assert np.sign(up[1]) != np.sign(down[1])  # elbow angle flips sign
    assert fk_3link_planar(*up)[:2] == pytest.approx(fk_3link_planar(*down)[:2])


def test_elbow_up_is_the_default():
    """The default must match what the hardware has always used."""
    assert ik_3link_planar(60.0, 10.0, 0.0) == ik_3link_planar(60.0, 10.0, 0.0, elbow="up")


def test_unreachable_target_raises():
    far = L1 + L2 + L3 + 10.0
    with pytest.raises(ValueError, match="unreachable"):
        ik_3link_planar(far, 0.0, 0.0)


def test_aggressive_phi_can_make_a_close_tip_unreachable():
    """Reachability is checked on the wrist, not the tip.

    A tip well inside the workspace still fails if phi throws the wrist out.
    """
    x, z = L1 + L2 - 1.0, 0.0
    assert is_reachable(x, z, 0.0)              # tool forward: wrist pulled in
    assert not is_reachable(x, z, np.pi)        # tool reversed: wrist pushed out


def test_bad_elbow_argument_raises():
    with pytest.raises(ValueError, match="elbow"):
        ik_3link_planar(60.0, 10.0, 0.0, elbow="sideways")


def test_workspace_boundary_does_not_blow_up_on_float_error():
    """c2 clipping must absorb rounding exactly at full extension."""
    x = L1 + L2 + L3
    th1, th2, th3 = ik_3link_planar(x, 0.0, 0.0)
    assert th2 == pytest.approx(0.0, abs=1e-6)
    assert fk_3link_planar(th1, th2, th3)[0] == pytest.approx(x, abs=1e-6)


def test_joint_points_chain_is_consistent_with_fk():
    th = ik_3link_planar(60.0, 10.0, np.deg2rad(-30.0))
    base, elbow, wrist, tip = joint_points(*th)

    assert base == (0.0, 0.0)
    assert np.hypot(*np.subtract(elbow, base)) == pytest.approx(L1)
    assert np.hypot(*np.subtract(wrist, elbow)) == pytest.approx(L2)
    assert np.hypot(*np.subtract(tip, wrist)) == pytest.approx(L3)
    assert tip == pytest.approx(fk_3link_planar(*th)[:2])
