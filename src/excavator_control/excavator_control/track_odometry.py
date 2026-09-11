"""Wheel odometry for the skid-steer tracks, from LSS servo positions.

Replaces rtabmap's rgbd_odometry as the odometry source. Visual odometry is a
poor fit for this machine: a vibrating tracked chassis, skid-steer rotation,
and a low-texture limestone face defeat feature tracking exactly where the
excavator spends its time. Track encoders and a gyro do not care about texture.

Publishes:
    /odom                     nav_msgs/Odometry
    odom -> base_footprint    TF (disable with publish_tf:=false when an EKF owns it)

Accuracy note
-------------
Skid-steer odometry is decent in translation and *bad* in rotation, because
turning requires the tracks to slip laterally by design. The yaw integrated
here will drift, and no amount of tuning fixes that -- it is a property of the
drivetrain. The intended fix is to fuse this with the D435i gyro in
robot_localization, letting wheel odometry carry translation and the IMU carry
heading. Until then, treat yaw as approximate and expect drift on every turn.

Set publish_tf to false once an EKF is running, or the two will fight over the
odom -> base_footprint transform.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from . import lss
from . import lss_const as lssc

LEFT_TRACK_ID = 0
RIGHT_TRACK_ID = 1


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class TrackOdometry(Node):
    def __init__(self):
        super().__init__("track_odometry")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate", 20.0)
        # MEASURE both of these on the machine; they set the scale of every
        # distance nav2 believes it has travelled.
        self.declare_parameter("track_separation", 0.36)   # m, centre-to-centre
        self.declare_parameter("counts_per_metre", 1000.0)  # LSS position units per metre

        port = self.get_parameter("port").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.separation = self.get_parameter("track_separation").value
        self.counts_per_m = self.get_parameter("counts_per_metre").value
        rate = self.get_parameter("publish_rate").value

        lss.initBus(port, lssc.LSS_DefaultBaud)
        self.left = lss.LSS(LEFT_TRACK_ID)
        self.right = lss.LSS(RIGHT_TRACK_ID)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        self.last_cmd = Twist()
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f"track_odometry on {port}: separation={self.separation} m, "
            f"{self.counts_per_m} counts/m, publish_tf={self.publish_tf}"
        )

    def _on_cmd_vel(self, msg):
        """Remember the commanded twist so /odom can report velocity."""
        self.last_cmd = msg

    def _read_positions(self):
        """Return (left, right) encoder positions, or None if the bus hiccups."""
        try:
            left = self.left.getPosition()
            right = self.right.getPosition()
        except Exception as exc:                       # serial bus errors are transient
            self.get_logger().warn(f"LSS read failed: {exc}", throttle_duration_sec=5.0)
            return None

        if left is None or right is None:
            return None
        try:
            return float(left), float(right)
        except (TypeError, ValueError):
            return None

    def _update(self):
        positions = self._read_positions()
        if positions is None:
            return
        left, right = positions

        now = self.get_clock().now()
        if self.last_left is None:
            self.last_left, self.last_right = left, right
            self.last_time = now
            return

        # Right track is mirrored: forward motion counts the opposite way.
        d_left = (left - self.last_left) / self.counts_per_m
        d_right = -(right - self.last_right) / self.counts_per_m
        self.last_left, self.last_right = left, right

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        d_centre = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self.separation   # the slip-prone term

        # Integrate at the midpoint heading: markedly better than using the
        # start heading when the machine is turning while driving.
        mid_yaw = self.yaw + d_yaw / 2.0
        self.x += d_centre * math.cos(mid_yaw)
        self.y += d_centre * math.sin(mid_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))

        self._publish(now, d_centre / dt, d_yaw / dt)

    def _publish(self, stamp, v_linear, v_angular):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        msg.twist.twist.linear.x = v_linear
        msg.twist.twist.angular.z = v_angular

        # Yaw is the untrustworthy term on a skid-steer; say so in the
        # covariance so an EKF weights the IMU over this.
        msg.pose.covariance[0] = 0.02     # x
        msg.pose.covariance[7] = 0.02     # y
        msg.pose.covariance[35] = 0.30    # yaw -- deliberately pessimistic
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.30
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quaternion(self.yaw)
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TrackOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
