"""Publish synthetic depth frames, for testing without a camera.

Lets the whole interlock path be exercised at a desk: fake_depth ->
proximity_monitor -> motor_node. Testing a safety stop by walking in front of
a moving excavator is a poor way to find out the thresholds are wrong.

    # hold a surface at 2 m, then drop it to 0.5 m and watch the interlock trip
    ros2 run excavator_perception fake_depth --ros-args -p distance:=2.0
    ros2 param set /fake_depth distance 0.5

    # sweep in and back out continuously
    ros2 run excavator_perception fake_depth --ros-args -p mode:=sweep

    # simulate the camera losing returns entirely
    ros2 param set /fake_depth dropout 1.0

Modes:
    fixed   hold `distance`, changeable live with ros2 param set
    sweep   ramp between sweep_near and sweep_far and back, continuously

`dropout` punches random holes in the frame, which is what a dark or glossy
surface does in reality. Push it past min_valid_fraction and the interlock
should block rather than trusting what little is left.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FakeDepth(Node):
    def __init__(self):
        super().__init__("fake_depth")

        self.declare_parameter("topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("rate", 15.0)
        self.declare_parameter("distance", 2.0)     # metres, live-settable
        self.declare_parameter("dropout", 0.0)      # 0..1 share of dead pixels
        self.declare_parameter("noise", 0.01)       # metres, gaussian
        self.declare_parameter("mode", "fixed")     # fixed | sweep
        self.declare_parameter("sweep_near", 0.5)
        self.declare_parameter("sweep_far", 2.0)
        self.declare_parameter("sweep_period", 10.0)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        rate = float(self.get_parameter("rate").value)

        self.rng = np.random.default_rng(0)
        self.t = 0.0
        self.dt = 1.0 / rate

        self.pub = self.create_publisher(
            Image, self.get_parameter("topic").value,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(self.dt, self._publish)

        self.get_logger().info(
            f"fake_depth publishing {self.width}x{self.height} at {rate:g} Hz "
            f"on {self.get_parameter('topic').value}")

    def _current_distance(self):
        if self.get_parameter("mode").value != "sweep":
            return float(self.get_parameter("distance").value)

        near = float(self.get_parameter("sweep_near").value)
        far = float(self.get_parameter("sweep_far").value)
        period = max(float(self.get_parameter("sweep_period").value), 1e-3)
        # Triangle wave: linear in and linear out, so the moment it crosses a
        # threshold is easy to reason about.
        phase = (self.t % period) / period
        ramp = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
        return near + (far - near) * ramp

    def _publish(self):
        self.t += self.dt
        distance = self._current_distance()
        noise = float(self.get_parameter("noise").value)
        dropout = float(np.clip(self.get_parameter("dropout").value, 0.0, 1.0))

        metres = np.full((self.height, self.width), distance, dtype=np.float32)
        if noise > 0.0:
            metres += self.rng.normal(0.0, noise, metres.shape).astype(np.float32)

        raw = np.clip(metres * 1000.0, 0, 65535).astype(np.uint16)
        if dropout > 0.0:
            raw[self.rng.random(raw.shape) < dropout] = 0   # 0 == no return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth_optical_frame"
        msg.height, msg.width = self.height, self.width
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = self.width * 2
        msg.data = raw.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
