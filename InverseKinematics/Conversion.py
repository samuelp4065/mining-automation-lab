import numpy as np


# ---------- IK / FK ----------

def ik_3link_planar(x, z, phi, L1, L2, L3):
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

    Returns:
      (theta1, theta2, theta3) in radians
        theta1 = boom/base shoulder angle
        theta2 = stick elbow angle (relative to boom), ALWAYS ELBOW-UP
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

    # ALWAYS elbow-up solution
    s2 = -np.sqrt(1 - c2**2)
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

# ---------- Angle → Encoder mappings ----------

# Boom
def angleToEncoder1(angle_deg):
    return float((angle_deg - 40) * 9/2)

# Stick
def angleToEncoder2(angle_deg):
    return float((angle_deg + 79) * (-90/41))

# Bucket
def angleToEncoder3(angle_deg):
    return float((angle_deg + 385/3) * (-27/25))


if __name__ == "__main__":
    # Link lengths (cm) – adjust to your excavator
    L1 = 46.0  # boom
    L2 = 20.0  # stick/arm
    L3 = 12.0  # bucket/tool

    while True:
        command = input(
            "Enter target (x z phi_deg) "
            "e.g. '60 10 -80', or 'exit' to quit: "
        )

        if command.lower() == "exit":
            break

        parts = command.split()
        if len(parts) != 3:
            print("❌ Please enter exactly 3 values: x z phi_deg")
            continue

        try:
            x, z, phi_deg = map(float, parts)
        except ValueError:
            print("❌ Invalid input. Please enter numeric values.")
            continue

        phi = np.deg2rad(phi_deg)

        try:
            th1, th2, th3 = ik_3link_planar(x, z, phi, L1, L2, L3)
        except ValueError as e:
            print(f"❌ IK error: {e}")
            continue

        # Joint angles in degrees
        th1_deg, th2_deg, th3_deg = np.rad2deg([th1, th2, th3])

        # Convert to encoder values
        enc1 = angleToEncoder1(th1_deg)
        enc2 = angleToEncoder2(th2_deg)
        enc3 = angleToEncoder3(th3_deg)

        

        # # Optional FK check
        # xt, zt, phit = fk_3link_planar(th1, th2, th3, L1, L2, L3)
        # phit_deg = np.rad2deg(phit)

        # print("\n=== IK solution ===")
        # print(f"theta1, theta2, theta3 (deg): {th1_deg:.2f}, {th2_deg:.2f}, {th3_deg:.2f}")
        # print("Encoder values:")
        # print(f"  Boom   → {enc1}")
        # print(f"  Stick  → {enc2}")
        # print(f"  Bucket → {enc3}")
        # print("FK check (x, z, phi_deg): "
        #       f"{xt:.2f}, {zt:.2f}, {phit_deg:.2f}\n")
