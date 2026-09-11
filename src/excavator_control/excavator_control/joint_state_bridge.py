"""Publish /joint_states for the arm and cab.

The TF chain base_link -> cab_link -> camera_link runs through swing_joint,
which is revolute, so robot_state_publisher emits *nothing* for the camera
until something publishes that joint's position. Navigation then fails in a
confusing way: the costmap has no camera transform and silently stays empty.

This node owns that responsibility, because excavator_control is what talks to
the servos. It replaces the stock joint_state_publisher, which would otherwise
be an extra apt dependency that only ever publishes zeros.

Modes:
    hardware (default)  read the swingdrive position over the LSS bus
    simulate            publish fixed angles; no serial port required

The boom, stick and bucket actuators are open-loop RC linear actuators with no
position feedback, so their angles are whatever was last commanded, tracked in
software. Until a commanding node reports back, they report their configured
defaults.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .drive_config import declare_float

JOINT_NAMES = ["swing_joint", "boom_joint", "stick_joint", "bucket_joint"]

# Mid-travel defaults, inside the limits in excavator_kinematics.calibration.
DEFAULT_ANGLES = {
    "swing_joint": 0.0,
    "boom_joint": math.radians(40.0),
    "stick_joint": math.radians(-80.0),
    "bucket_joint": math.radians(-100.0),
}

SWINGDRIVE_ID = 2


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")

        self.declare_parameter("mode", "hardware")       # hardware | simulate
        self.declare_parameter("port", "/dev/ttyUSB0")
        # LSS reports the swingdrive in tenths of a degree.

        self.mode = self.get_parameter("mode").value
        self.counts_per_deg = declare_float(self, "swing_counts_per_degree", 10.0)
        rate = declare_float(self, "publish_rate", 20.0)

        self.angles = dict(DEFAULT_ANGLES)
        self.swingdrive = None

        if self.mode == "hardware":
            self._connect(self.get_parameter("port").value)

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f"joint_state_bridge running in '{self.mode}' mode")

    def _connect(self, port):
        """Open the LSS bus, falling back to simulate mode if it is absent."""
        try:
            from . import lss
            from . import lss_const as lssc
            lss.initBus(port, lssc.LSS_DefaultBaud)
            self.swingdrive = lss.LSS(SWINGDRIVE_ID)
        except Exception as exc:
            self.get_logger().warn(
                f"LSS bus unavailable on {port} ({exc}); falling back to simulate mode. "
                "TF will publish, but the cab angle will not be real."
            )
            self.mode = "simulate"

    def _read_swing(self):
        if self.swingdrive is None:
            return None
        try:
            position = self.swingdrive.getPosition()
        except Exception as exc:
            self.get_logger().warn(f"swingdrive read failed: {exc}",
                                   throttle_duration_sec=5.0)
            return None
        if position is None:
            return None
        try:
            return math.radians(float(position) / self.counts_per_deg)
        except (TypeError, ValueError):
            return None

    def _publish(self):
        if self.mode == "hardware":
            swing = self._read_swing()
            if swing is not None:
                self.angles["swing_joint"] = swing

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [self.angles[n] for n in JOINT_NAMES]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
