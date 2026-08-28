
import numpy as np

def ik_3link_planar(x, z, phi, L1, L2, L3, elbow="down"):
    """
    Inverse kinematics for a 3-link planar arm.

    Links:
      L1 = boom length (shoulder -> elbow)
      L2 = stick/arm length (elbow -> wrist)
      L3 = bucket/tool length (wrist -> tip)

    Target:
      (x, z) = desired bucket tip position in base frame
      phi    = desired end-effector orientation angle (radians)
              measured from +x axis to tool direction.
              Example: phi=0 means tool points forward horizontally.

    elbow:
      "down" or "up" configuration.

    Returns:
      (theta1, theta2, theta3) in radians
        theta1 = boom/base shoulder angle
        theta2 = stick elbow angle (relative to boom)
        theta3 = bucket angle (relative to stick)

    Raises:
      ValueError if the target is unreachable.
    """

    # --- Step 1: compute wrist (joint between L2 and L3) target ---
    # tip = wrist + L3 * [cos(phi), sin(phi)]
    xw = x - L3 * np.cos(phi)
    zw = z - L3 * np.sin(phi)

    # --- Step 2: solve 2-link IK for boom+stick to reach wrist ---
    rw2 = xw**2 + zw**2
    rw = np.sqrt(rw2)

    # reachability check
    if rw > (L1 + L2) or rw < abs(L1 - L2):
        raise ValueError("Target unreachable for given L1, L2, L3.")

    # cosine law for elbow angle
    c2 = (rw2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = np.clip(c2, -1.0, 1.0)  # numerical safety

    if elbow == "up":
        s2 = -np.sqrt(1 - c2**2)
    elif elbow == "down":
        s2 = +np.sqrt(1 - c2**2)
    else:
        raise ValueError("elbow must be 'down' or 'up'.")

    theta2 = np.arctan2(s2, c2)

    # shoulder angle
    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = np.arctan2(zw, xw) - np.arctan2(k2, k1)

    # --- Step 3: bucket joint to match desired tool orientation ---
    theta3 = phi - (theta1 + theta2)

    return theta1, theta2, theta3


def fk_3link_planar(theta1, theta2, theta3, L1, L2, L3):
    """Forward kinematics to verify IK."""
    x1 = L1 * np.cos(theta1)
    z1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    z2 = z1 + L2 * np.sin(theta1 + theta2)

    xt = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    zt = z2 + L3 * np.sin(theta1 + theta2 + theta3)

    phi = theta1 + theta2 + theta3
    return xt, zt, phi


if __name__ == "__main__":
    # Example numbers (meters). Replace with your actual excavator link lengths.

    L1 = 46.0  # cm
    L2 = 20.0  # cm
    L3 = 12.0  # cm  # bucket/tool

    # Desired bucket tip pose:
    x, z = 60, 10           # target position
    phi = np.deg2rad(-30)       # tool points slightly downward

    for cfg in ["down", "up"]:
        th1, th2, th3 = ik_3link_planar(x, z, phi, L1, L2, L3, elbow=cfg)
        xt, zt, phit = fk_3link_planar(th1, th2, th3, L1, L2, L3)

        print(f"\nConfig: {cfg}")
        print("theta1, theta2, theta3 (deg):",
              np.rad2deg([th1, th2, th3]))
        print("FK check (x,z,phi):",
              (xt, zt, np.rad2deg(phit)))
