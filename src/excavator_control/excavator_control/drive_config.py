"""Single source of truth for the track drive's IDs, signs and geometry.

motor_node commands the tracks and track_odometry integrates them. If those
two disagree about which track is mirrored, the machine drives correctly while
reporting that it went backwards -- and nav2 believes the odometry, not the
machine. Both import from here so that cannot happen.

The sign convention is inherited from scripts/keyboard_track.py, which is the
teleop that has actually driven this machine:

    forward  =>  left.wheelRPM(-r),  right.wheelRPM(+r)

i.e. the LEFT servo is mounted mirrored, so positive forward motion needs a
negative RPM command on it.
"""

#: LSS bus IDs. The swingdrive is ID 2, handled by joint_state_bridge.
LEFT_ID = 0
RIGHT_ID = 1

#: Which way each servo turns for forward motion. Left is mirrored.
LEFT_SIGN = -1
RIGHT_SIGN = +1

#: Serial port shared by every LSS device.
DEFAULT_PORT = "/dev/ttyUSB0"

#: MEASURE: centre-to-centre track spacing, metres. Sets how fast a commanded
#: angular velocity actually turns the machine, and must match the value in
#: excavator_description/urdf/excavator.urdf.xacro.
DEFAULT_TRACK_SEPARATION = 0.36

#: CALIBRATE: RPM commanded per metre/second of track speed. Depends on the
#: drive sprocket diameter and gearing, so it cannot be derived -- drive a
#: known distance at a known command and solve for it. Until then every
#: velocity in m/s is only proportionally correct, not absolutely.
DEFAULT_RPM_PER_MPS = 100.0

#: Hard ceiling on commanded wheel RPM, whatever the twist asks for.
DEFAULT_MAX_RPM = 60.0


def twist_to_track_speeds(linear_x, angular_z, track_separation):
    """Skid-steer mixing: body twist -> (left, right) track speed in m/s.

    This is exactly the relation track_odometry inverts when integrating
    encoder counts, so the two stay consistent by construction.
    """
    half = track_separation / 2.0
    return linear_x - angular_z * half, linear_x + angular_z * half
