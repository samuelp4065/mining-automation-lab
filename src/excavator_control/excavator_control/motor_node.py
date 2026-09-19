"""Differential (skid-steer) drive for the two tracks, from /cmd_vel.

Replaces an earlier single-servo bring-up version that drove only LSS ID 0 and
discarded angular.z entirely, so the machine could go forwards but never turn.

    ros2 run excavator_control motor_node
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Safety
------
A teleop drive node with no timeout is genuinely dangerous on a tracked
machine: if the publisher dies, the terminal closes, or the network drops, the
last command keeps executing and the excavator keeps driving. The watchdog
below stops the tracks when /cmd_vel goes quiet, and the tracks are also
stopped on shutdown. Do not disable either.

Only one process may own /dev/ttyUSB0. Stop the curses teleop scripts, and
note that track_odometry and joint_state_bridge share this bus -- running them
alongside this node is fine, but a second *teleop* is not.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from . import lss
from . import lss_const as lssc
from .drive_config import (
    DEFAULT_MAX_RPM, DEFAULT_PORT, DEFAULT_RPM_PER_MPS,
    DEFAULT_TRACK_SEPARATION, LEFT_ID, LEFT_SIGN, RIGHT_ID, RIGHT_SIGN,
    declare_float, scale_to_max_rpm, twist_to_track_speeds,
)


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        self.declare_parameter("port", DEFAULT_PORT)
        port = self.get_parameter("port").value

        self.separation = declare_float(self, "track_separation", DEFAULT_TRACK_SEPARATION)
        self.rpm_per_mps = declare_float(self, "rpm_per_mps", DEFAULT_RPM_PER_MPS)
        self.max_rpm = declare_float(self, "max_rpm", DEFAULT_MAX_RPM)
        self.cmd_timeout = declare_float(self, "cmd_timeout", 0.5)

        # Proximity interlock. require_proximity_ok=False disables it entirely
        # for bench work; leave it on whenever the machine can actually move.
        self.declare_parameter("require_proximity_ok", True)
        self.require_proximity = self.get_parameter("require_proximity_ok").value
        self.proximity_timeout = declare_float(self, "proximity_timeout", 1.0)
        # Start blocked: until proximity_monitor has said otherwise, there is
        # no evidence the path ahead is clear.
        self.proximity_ok = not self.require_proximity
        self.last_proximity_time = None

        lss.initBus(port, lssc.LSS_DefaultBaud)
        self.left = lss.LSS(LEFT_ID)
        self.right = lss.LSS(RIGHT_ID)

        self.last_cmd_time = self.get_clock().now()
        self.stopped = True
        self._stop()

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "proximity_ok", self._on_proximity, 10)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f"motor_node on {port}: separation={self.separation} m, "
            f"{self.rpm_per_mps} rpm per m/s, max {self.max_rpm} rpm, "
            f"watchdog {self.cmd_timeout} s, "
            f"proximity interlock {'ON' if self.require_proximity else 'OFF'}"
        )

    def _on_proximity(self, msg):
        if msg.data != self.proximity_ok:
            self.get_logger().info(
                f"proximity: forward motion {'permitted' if msg.data else 'BLOCKED'}")
        self.proximity_ok = bool(msg.data)
        self.last_proximity_time = self.get_clock().now()

    def _forward_permitted(self):
        """Fail safe: no interlock signal means no forward motion.

        Silence is treated as unsafe rather than ignored. If the monitor
        crashes or the camera is unplugged the topic simply stops, and an
        interlock that quietly switches itself off when it breaks is worse
        than none -- by then you are relying on it.
        """
        if not self.require_proximity:
            return True
        if self.last_proximity_time is None:
            return False
        age = (self.get_clock().now() - self.last_proximity_time).nanoseconds * 1e-9
        if age > self.proximity_timeout:
            self.get_logger().warn(
                f"no proximity signal for {age:.1f} s; blocking forward motion",
                throttle_duration_sec=2.0)
            return False
        return self.proximity_ok

    def _on_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        # Block forward travel only. Reverse and spin stay available, or the
        # operator would have no way to back away from the obstacle that
        # triggered the stop.
        if linear_x > 0.0 and not self._forward_permitted():
            linear_x = 0.0

        v_left, v_right = twist_to_track_speeds(
            linear_x, msg.angular.z, self.separation
        )

        # Convert first, limit second. Clamping each track on its own would
        # flatten the commanded turn as speed rises -- see scale_to_max_rpm.
        rpm_left, rpm_right = scale_to_max_rpm(
            v_left * self.rpm_per_mps, v_right * self.rpm_per_mps, self.max_rpm
        )

        self._drive(rpm_left, rpm_right)
        self.stopped = False

    def _drive(self, rpm_left, rpm_right):
        try:
            self.left.wheelRPM(int(round(LEFT_SIGN * rpm_left)))
            self.right.wheelRPM(int(round(RIGHT_SIGN * rpm_right)))
        except Exception as exc:
            self.get_logger().error(f"LSS write failed: {exc}",
                                    throttle_duration_sec=2.0)

    def _stop(self):
        self._drive(0.0, 0.0)
        self.stopped = True

    def _watchdog(self):
        """Stop the tracks when /cmd_vel goes quiet.

        Most teleop publishers stop sending rather than sending zeros when a
        key is released or the process dies, so without this the last non-zero
        command would run indefinitely.
        """
        if self.stopped:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.get_logger().warn(
                f"no cmd_vel for {elapsed:.2f} s; stopping tracks"
            )
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leaving the tracks driving on exit is how a machine runs away.
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
