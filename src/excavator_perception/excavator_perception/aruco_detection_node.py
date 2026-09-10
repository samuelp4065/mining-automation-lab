#------------------------------------------------------------------------------------------------

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped
import numpy as np

class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Adjust topic name as needed
            self.image_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        # Camera matrix and distortion coefficients (example values, replace with actual calibration)
        self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.marker_length = 0.09  # Marker size in meters

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'  # Adjust frame as needed
                pose_msg.pose.position.x = tvecs[i][0][0]
                pose_msg.pose.position.y = tvecs[i][0][1]
                pose_msg.pose.position.z = tvecs[i][0][2]
                # Convert rotation vector to quaternion (simplified, for full implementation use tf)
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                # This is a placeholder; proper quaternion conversion needed
                pose_msg.pose.orientation.w = 1.0  # Simplified
                self.publisher.publish(pose_msg)
                self.get_logger().info(f'Detected ArUco marker ID: {marker_id}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
