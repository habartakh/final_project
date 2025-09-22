#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from cv2 import aruco
from image_geometry import PinholeCameraModel

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_geometry_msgs


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', 0.045)  # meters
        self.marker_size = self.get_parameter('marker_size').value

        self.declare_parameter('camera_info_topic', '/wrist_rgbd_depth_sensor/camera_info')
        self.declare_parameter('image_topic', '/wrist_rgbd_depth_sensor/image_raw')
        self.declare_parameter('camera_frame', 'D415_link')

        # Access node parameters 
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # Subscribers
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.image_pub = self.create_publisher(Image, '/aruco/image_marked', 10)
        self.marker_pub = self.create_publisher(Marker, '/aruco/rviz_marker', 10)

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)


        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info("Camera info received.")

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        # Convert image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs()
            )

            for i in range(len(ids)):
                rvec = rvecs[i]
                tvec = tvecs[i]

                # Draw detected marker and its axis
                aruco.drawDetectedMarkers(frame, corners)
                # aruco.drawAxis(frame, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs(), rvec, tvec, 0.03)
                cv2.drawFrameAxes(frame, self.camera_model.intrinsicMatrix(), self.camera_model.distortionCoeffs(), rvec, tvec, 0.03)


                # The Aruco marker pose w.r.t D415_color_optical_frame (i.e the frame in camera_info)
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose.position.x = float(tvec[0][0])
                pose_msg.pose.position.y = float(tvec[0][1])
                pose_msg.pose.position.z = float(tvec[0][2])

                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quat = self.rotation_matrix_to_quaternion(rotation_matrix)

                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                # Transform the pose into the D415_link frame 
                # Since we want to find the TF base_link -- D415_link
                try:
                    target_frame = self.camera_frame
                    transformed_pose = self.tf_buffer.transform(pose_msg, target_frame, timeout=rclpy.duration.Duration(seconds=0.5))

                    # Now publish this new pose or broadcast it as TF
                    self.pose_pub.publish(transformed_pose)

                    # Also broadcast it as TF for debugging purposes
                    t = TransformStamped()
                    t.header = transformed_pose.header
                    t.child_frame_id = f"aruco_{ids[i][0]}"
                    t.transform.translation.x = transformed_pose.pose.position.x
                    t.transform.translation.y = transformed_pose.pose.position.y
                    t.transform.translation.z = transformed_pose.pose.position.z
                    t.transform.rotation = transformed_pose.pose.orientation
                    self.tf_broadcaster.sendTransform(t)

                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {e}")


        # Publish annotated image
        output_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        output_img.header = msg.header
        self.image_pub.publish(output_img)

    def rotation_matrix_to_quaternion(self, R):
        """Convert a 3x3 rotation matrix to quaternion"""
        qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
