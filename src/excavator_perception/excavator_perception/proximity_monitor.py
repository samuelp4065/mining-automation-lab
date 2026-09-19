"""Proximity interlock: stop forward motion when something is close ahead.

Watches a patch of the depth image and publishes whether the machine may
drive forward. motor_node subscribes and gates cmd_vel.

    ros2 run excavator_perception proximity_monitor
    ros2 run excavator_perception proximity_monitor --ros-args -p stop_below:=1.5

Why this is a separate node
---------------------------
Two hard constraints shape it.

/dev/ttyUSB0 admits one process, and motor_node holds it, so nothing else can
command the servos directly -- the stop has to travel over a topic.

And camera I/O must stay out of the servo control loop. A USB stall that
blocked motor_node would freeze the thing responsible for stopping, which is
the opposite of what an interlock is for.

It subscribes to a depth topic rather than opening the camera itself, for the
same reason: realsense2_camera owns the device when navigation is running,
and two processes cannot both claim it.

Fail-safe
---------
This node publishes continuously, so its messages are also a heartbeat.
motor_node treats silence as unsafe after a timeout -- if this node crashes,
the camera unplugs, or the topic stops, the machine stops. An interlock that
quietly disables itself when it breaks is worse than none, because you will
have learned to trust it.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from excavator_perception.proximity import (
    ProximityConfig, evaluate, patch_distance,
)

#: Depth encodings this node understands, mapped to their numpy dtype.
_ENCODINGS = {"16UC1": np.uint16, "mono16": np.uint16, "32FC1": np.float32}


def depth_to_metres(msg, depth_scale):
    """sensor_msgs/Image -> float32 metres, NaN where there was no return.

    Zero means "no measurement" in a 16-bit depth image, not "zero distance".
    Letting those through as 0.0 would make the interlock trip permanently on
    every dropout.
    """
    dtype = _ENCODINGS.get(msg.encoding)
    if dtype is None:
        raise ValueError(
            f"unsupported depth encoding {msg.encoding!r}; "
            f"expected one of {sorted(_ENCODINGS)}"
        )

    raw = np.frombuffer(msg.data, dtype=dtype)
    # step is bytes per row and may include padding, so reshape via step
    # rather than assuming width * itemsize.
    per_row = msg.step // np.dtype(dtype).itemsize
    raw = raw.reshape(msg.height, per_row)[:, :msg.width]

    if dtype == np.float32:
        metres = raw.astype(np.float32).copy()   # already metres
    else:
        metres = raw.astype(np.float32) * depth_scale
    metres[raw == 0] = np.nan
    return metres


class ProximityMonitor(Node):
    def __init__(self):
        super().__init__("proximity_monitor")

        self.declare_parameter("depth_topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("stop_below", 1.00)
        self.declare_parameter("release_above", 1.20)
        self.declare_parameter("patch", 9)
        self.declare_parameter("min_valid_fraction", 0.25)
        self.declare_parameter("depth_scale", 0.001)   # 16UC1 is millimetres
        # Default samples the image centre. Fractions rather than pixels so the
        # setting survives a resolution change.
        self.declare_parameter("centre_x", 0.5)
        self.declare_parameter("centre_y", 0.5)

        topic = self.get_parameter("depth_topic").value
        self.patch = int(self.get_parameter("patch").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.centre_x = float(self.get_parameter("centre_x").value)
        self.centre_y = float(self.get_parameter("centre_y").value)

        self.config = ProximityConfig(
            stop_below_m=float(self.get_parameter("stop_below").value),
            release_above_m=float(self.get_parameter("release_above").value),
            min_valid_fraction=float(
                self.get_parameter("min_valid_fraction").value),
        )

        # Start blocked. Until a frame has actually been seen and judged safe,
        # the machine has no evidence it may move.
        self.clear = False

        self.ok_pub = self.create_publisher(Bool, "proximity_ok", 10)
        self.distance_pub = self.create_publisher(Float32, "proximity_distance", 10)

        # Camera drivers publish depth best-effort; a reliable subscription
        # would silently receive nothing.
        self.create_subscription(
            Image, topic, self._on_depth,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info(
            f"proximity_monitor on {topic}: stop below "
            f"{self.config.stop_below_m:.2f} m, release above "
            f"{self.config.release_above_m:.2f} m, {self.patch}x{self.patch} patch"
        )

    def _on_depth(self, msg):
        try:
            metres = depth_to_metres(msg, self.depth_scale)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            self._publish(False, float("nan"))
            return

        centre = (self.centre_x * (msg.width - 1), self.centre_y * (msg.height - 1))
        distance, fraction = patch_distance(metres, centre, self.patch)
        state = evaluate(distance, fraction, self.config, self.clear)

        if state.clear != self.clear:
            # Two call sites, not one with a variable severity: rclpy caches
            # severity per call site and raises "Logger severity cannot be
            # changed between calls" if the same line logs at info then warn.
            if state.clear:
                self.get_logger().info(f"forward motion permitted: {state.reason}")
            else:
                self.get_logger().warn(f"forward motion BLOCKED: {state.reason}")
        self.clear = state.clear
        self._publish(state.clear, state.distance_m)

    def _publish(self, clear, distance):
        self.ok_pub.publish(Bool(data=bool(clear)))
        self.distance_pub.publish(Float32(data=float(distance)))


def main(args=None):
    rclpy.init(args=args)
    node = ProximityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
