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


def scale_to_max_rpm(rpm_left, rpm_right, max_rpm):
    """Hold both tracks under max_rpm without changing their ratio.

    Clamping each track independently would silently straighten a turn: the
    outer track pins at the ceiling while the inner keeps its value, so the
    ratio between them drifts toward 1 as commanded speed rises, and the arc
    flattens out. At high enough speed both tracks pin and the machine drives
    dead straight while still being commanded to turn.

    Scaling both by the same factor keeps the commanded shape and only reduces
    how fast it is executed.
    """
    peak = max(abs(rpm_left), abs(rpm_right))
    if peak > max_rpm and peak > 0.0:
        scale = max_rpm / peak
        return rpm_left * scale, rpm_right * scale
    return rpm_left, rpm_right


def declare_float(node, name, default):
    """Declare a float parameter that also accepts an integer on the CLI.

    ROS 2 infers a parameter's type from its declared default, so a DOUBLE
    parameter rejects ``-p max_rpm:=20`` and demands ``:=20.0``. Every tunable
    in this package is a physical quantity where 20 and 20.0 mean the same
    thing, and the exception names the mismatch without naming the fix -- so
    accept either and coerce.
    """
    from rcl_interfaces.msg import ParameterDescriptor

    node.declare_parameter(name, default,
                           ParameterDescriptor(dynamic_typing=True))
    return float(node.get_parameter(name).value)


def twist_to_track_speeds(linear_x, angular_z, track_separation):
    """Skid-steer mixing: body twist -> (left, right) track speed in m/s.

    This is exactly the relation track_odometry inverts when integrating
    encoder counts, so the two stay consistent by construction.
    """
    half = track_separation / 2.0
    return linear_x - angular_z * half, linear_x + angular_z * half
