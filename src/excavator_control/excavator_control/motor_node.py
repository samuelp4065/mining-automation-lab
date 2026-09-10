#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from . import lss
from . import lss_const as lssc

# =======================
# LSS CONFIG
# =======================
CST_LSS_Port = "/dev/ttyUSB0"
CST_LSS_Baud = lssc.LSS_DefaultBaud
LSS_ID = 0   # wheel servo ID

# =======================
# WHEEL PARAMETERS
# =======================
MAX_WHEEL_SPEED = 200    # LSS wheel speed limit
# CMD_TIMEOUT_SEC = 2


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # ---- Init LSS bus ----
        lss.initBus(CST_LSS_Port, CST_LSS_Baud)
        self.lss = lss.LSS(LSS_ID)

        self.get_logger().info("LSS wheel motor initialized")

        # ---- State ----
        self.last_cmd_time = self.get_clock().now()
        self.current_speed = 0

        # ---- Subscriber ----
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # ---- Watchdog timer ----
        # self.create_timer(0.1, self.watchdog)

    # =======================
    # CALLBACK
    # =======================
    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x     # forward/back
        angular = msg.angular.z  # turning

        # Simple differential-wheel logic (single wheel)
        speed = linear * MAX_WHEEL_SPEED

        # Clamp speed
        speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, speed))
        speed = int(speed)

        self.current_speed = speed
        self.lss.wheel(speed)

        self.get_logger().info(
            f"cmd_vel linear={linear:.2f} → wheel speed={speed}"
        )

    # =======================
    # SAFETY WATCHDOG
    # =======================
    # def watchdog(self):
    #     elapsed = (
    #         self.get_clock().now() - self.last_cmd_time
    #     ).nanoseconds * 1e-9

    #     if elapsed > CMD_TIMEOUT_SEC:
    #         if self.current_speed != 0:
    #             self.lss.wheel(0)
    #             self.current_speed = 0
    #             self.get_logger().warn("Wheel stopped (timeout)")

    # =======================
    # CLEAN SHUTDOWN
    # =======================
    def destroy_node(self):
        self.lss.wheel(0)
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()








# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# from . import lss
# from . import lss_const as lssc

# # =======================
# # LSS CONFIG
# # =======================
# CST_LSS_Port = "/dev/ttyUSB0"
# CST_LSS_Baud = lssc.LSS_DefaultBaud
# LSS_ID = 0

# # =======================
# # SERVO LIMITS
# # =======================
# SERVO_MIN_DEG = -90.0
# SERVO_MAX_DEG =  90.0
# SERVO_CENTER_DEG = 0.0

# # =======================
# # SAFETY
# # =======================
# CMD_TIMEOUT_SEC = 0.5


# class MotorNode(Node):
#     def __init__(self):
#         super().__init__('motor_node')

#         # ---- Init LSS bus ----
#         lss.initBus(CST_LSS_Port, CST_LSS_Baud)
#         self.lss = lss.LSS(LSS_ID)

#         self.get_logger().info("LSS initialized")

#         # ---- Servo state ----
#         self.current_angle_deg = SERVO_CENTER_DEG
#         self.last_cmd_time = self.get_clock().now()

#         # ---- Subscriber ----
#         self.sub = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_callback,
#             10
#         )

#         # ---- Watchdog timer ----
#         self.create_timer(0.1, self.watchdog)

#     # =======================
#     # CALLBACK
#     # =======================
#     def cmd_callback(self, msg: Twist):
#         self.last_cmd_time = self.get_clock().now()

#         linear = msg.linear.x   # i / ,
#         speed_scale = 60.0      # deg/sec feel

#         # Incremental joint control
#         self.current_angle_deg += linear * speed_scale * 0.1

#         # Clamp limits
#         self.current_angle_deg = max(
#             SERVO_MIN_DEG,
#             min(SERVO_MAX_DEG, self.current_angle_deg)
#         )

#         # Convert to LSS units (0.1 deg)
#         angle_lss = int(self.current_angle_deg * 10)

#         self.lss.move(angle_lss)

#         self.get_logger().info(
#             f"cmd_vel={linear:.2f} → servo={self.current_angle_deg:.1f} deg"
#         )

#     # =======================
#     # SAFETY WATCHDOG
#     # =======================
#     def watchdog(self):
#         elapsed = (
#             self.get_clock().now() - self.last_cmd_time
#         ).nanoseconds * 1e-9

#         if elapsed > CMD_TIMEOUT_SEC:
#             self.lss.move(int(self.current_angle_deg * 10))

#     # =======================
#     # CLEAN SHUTDOWN
#     # =======================
#     def destroy_node(self):
#         self.lss.move(int(SERVO_CENTER_DEG * 10))
#         super().destroy_node()


# def main():
#     rclpy.init()
#     node = MotorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()










# #!/usr/bin/env python3
# import time
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32

# from . import lss
# from . import lss_const as lssc

# CST_LSS_Port = "/dev/ttyUSB0"
# CST_LSS_Baud = lssc.LSS_DefaultBaud
# LSS_ID = 0


# class MotorNode(Node):
#     def __init__(self):
#         super().__init__('motor_node')

#         # Init LSS bus
#         lss.initBus(CST_LSS_Port, CST_LSS_Baud)
#         self.lss = lss.LSS(LSS_ID)

#         self.get_logger().info("LSS initialized")

#         # Subscriber
#         self.sub = self.create_subscription(
#             Int32,
#             'lss_position',
#             self.cmd_callback,
#             10
#         )
        

#     def cmd_callback(self, msg):
#         pos = msg.data  # LSS uses 0.1 deg units
#         self.lss.move(pos)
#         self.get_logger().info(f"Moving LSS to {pos/10.0:.1f} deg")


# def main():
#     rclpy.init()
#     motor_node = MotorNode()
#     rclpy.spin(motor_node)
#     motor_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
