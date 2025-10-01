#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from image_geometry import PinholeCameraModel

from circle_pattern_pose_estimation.msg import CirclePose, CirclePoseArray

from std_msgs.msg import Bool


class CircleDetector(Node):
    def __init__(self):
        super().__init__('circle_detector')

        # Declare node parameters 
        self.declare_parameter('camera_info_topic', '/wrist_rgbd_depth_sensor/camera_info')
        self.declare_parameter('image_topic', '/wrist_rgbd_depth_sensor/image_raw')
        self.declare_parameter('depth_image_topic', '/wrist_rgbd_depth_sensor/depth/image_raw')

        # Access node parameters 
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_image_topic = self.get_parameter('depth_image_topic').value

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_image_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        self.image_pub = self.create_publisher(Image, '/circle_detector/image', 10)
        self.circles_pub = self.create_publisher(CirclePoseArray, '/detected_circles/poses', 10)

        self.depth_image = None
        self.rgb_image = None

        # For tracking circles consistently 
        self.tracked_circles = {}  # key: circle_id, value: 3D position (np.array)
        self.next_circle_id = 1

        # Setup a flag to start the detection upon a click on the webpage 
        self.detection_enabled = False
        self.start_detection_sub = self.create_subscription(
            Bool,
            '/start_circle_detection',
            self.start_detection_callback,
            10
        )

        self.get_logger().info(f"Circle Detector Node Initialised! Waiting for signal to start detection...")

    # Try to find the same circle in the successive frames
    def match_circle(self, new_position, threshold=0.03):

        for cid, old_pos in self.tracked_circles.items():
            dist = np.linalg.norm(new_position - old_pos)
            if dist < threshold:
                return cid
        return None

    def start_detection_callback(self, msg: Bool):
        self.detection_enabled = msg.data
        state = "enabled" if msg.data else "disabled"
        self.get_logger().info(f"Circle detection {state}.")


    def camera_info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_received = True

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if not self.camera_info_received or self.depth_image is None or not self.detection_enabled:
            return

        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(gray, 5)

        # Detect circles
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=20,
            param1=50,
            param2=30,
            minRadius=1, # 5
            maxRadius=50 # 13
        )

        circle_array_msg  = CirclePoseArray()
        circle_array_msg.circles = []

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for i, (x, y, r) in enumerate(circles):

                # Depth at (x, y)
                if y < self.depth_image.shape[0] and x < self.depth_image.shape[1]:
                    depth = self.depth_image[y, x] / 1000.0  # Convert from mm to meters

                    if depth > 0:
                        # Back-project to 3D
                        ray = self.camera_model.projectPixelTo3dRay((x, y))
                        point_3d = np.array(ray) * depth

                        # Track consistent circle ID
                        matched_id = self.match_circle(point_3d)
                        if matched_id is not None:
                            circle_id = matched_id
                        else:
                            circle_id = self.next_circle_id
                            self.next_circle_id += 1

                        self.tracked_circles[circle_id] = point_3d
                        label = f"c_{circle_id}"

                        # Draw label and circle
                        cv2.putText(self.rgb_image, label, (x - 10, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        cv2.circle(self.rgb_image, (x, y), r, (0, 255, 0), 2)

                        # Create PoseStamped message
                        pose_stamped = PoseStamped()
                        pose_stamped.header = msg.header
                        pose_stamped.pose.position.x = point_3d[0]
                        pose_stamped.pose.position.y = point_3d[1]
                        pose_stamped.pose.position.z = point_3d[2]

                        # Add label to pose estimation message
                        circle_msg = CirclePose()
                        circle_msg.label = label
                        circle_msg.pose = pose_stamped
                        circle_array_msg.circles.append(circle_msg)


        # Convert OpenCV image to ROS Image message
        output_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding='bgr8')
        output_msg.header = msg.header  # Preserve timestamp and frame_id

        # Publish annotated image
        self.image_pub.publish(output_msg)

        # Publish labeled poses
        self.circles_pub.publish(circle_array_msg)



def main(args=None):
    rclpy.init(args=args)
    node = CircleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
