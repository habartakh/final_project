#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class CameraRobotFramePublisher : public rclcpp::Node {
public:
  CameraRobotFramePublisher() : Node("aruco_camera_frame_publisher") {

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize the TF base_link - aruco marker listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ =
        this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
            "/aruco/markers", 10,
            std::bind(&CameraRobotFramePublisher::handle_aruco_pose, this,
                      std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), // 1s interval
        std::bind(&CameraRobotFramePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "CameraRobotFramePublisher node initialized!");
  }

private:
  // Get the ArUco marker pose from the detection node topic
  void handle_aruco_pose(
      const std::shared_ptr<aruco_interfaces::msg::ArucoMarkers> msg) {

    estimated_aruco_pose = msg->poses[0];

    // Get the timestamp from the ArUco detection
    rclcpp::Time msg_time = msg->header.stamp;
  }

  // Broadcast the TF camera -- base_link every second
  void timer_callback() {

    //  IMPORTANT: The aruco pose is estimated relative TO the CAMERA frame
    //  We are establishing the TF FROM the base_link so we need to project the
    //  aruco coordinates in the aruco (robot) frame. To do that, invert the
    //  position and orientation vectors
    RCLCPP_INFO(this->get_logger(), "Inside timer callback!!");

    // Look up for the transformation between base_link and
    // the aruco marker frames
    std::string fromFrameRel = "rg2_gripper_aruco_link";
    std::string toFrameRel = "base_link";
    try {
      t_aruco_base = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                                 tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    // Computing the TF base_link -- camera amount to doing the following:
    // TF(base_link - cam) = TF(base_link - aruco) * TF(aruco - cam)

    // Convert both transforms to TF2 types
    tf2::Transform tf_aruco_base;
    tf2::Transform tf_cam_aruco;

    // tf from base to aruco (from TF)
    tf2::fromMsg(t_aruco_base.transform, tf_aruco_base);

    // tf from camera to aruco (from ArUco detection)
    tf2::Vector3 trans(estimated_aruco_pose.position.x,
                       estimated_aruco_pose.position.y,
                       estimated_aruco_pose.position.z);

    tf2::Quaternion rot(
        estimated_aruco_pose.orientation.x, estimated_aruco_pose.orientation.y,
        estimated_aruco_pose.orientation.z, estimated_aruco_pose.orientation.w);

    tf_cam_aruco.setOrigin(trans);
    tf_cam_aruco.setRotation(rot);

    // Invert the camera-to-aruco to get aruco-to-camera
    tf2::Transform tf_aruco_cam = tf_cam_aruco.inverse();

    // Now compute base_link -> camera
    tf2::Transform tf_base_camera = tf_aruco_base * tf_aruco_cam;

    // Convert back to ROS message

    t_base_camera_msg.header.stamp = this->get_clock()->now();
    t_base_camera_msg.header.frame_id = "base_link";
    t_base_camera_msg.child_frame_id = "wrist_rgbd_camera_link";

    t_base_camera_msg.transform.translation.x = tf_base_camera.getOrigin().x();
    t_base_camera_msg.transform.translation.y = tf_base_camera.getOrigin().y();
    t_base_camera_msg.transform.translation.z = tf_base_camera.getOrigin().z();

    tf2::Quaternion q = tf_base_camera.getRotation();
    t_base_camera_msg.transform.rotation.x = q.x();
    t_base_camera_msg.transform.rotation.y = q.y();
    t_base_camera_msg.transform.rotation.z = q.z();
    t_base_camera_msg.transform.rotation.w = q.w();

    // RCLCPP_INFO(this->get_logger(), "rotation.x: %f",
    //             t_base_camera_msg.transform.rotation.x);

    // RCLCPP_INFO(this->get_logger(), "translation.x: %f",
    //             t_base_camera_msg.transform.translation.x);
    // RCLCPP_INFO(this->get_logger(), "translation.y: %f",
    //             t_base_camera_msg.transform.translation.y);
    // RCLCPP_INFO(this->get_logger(), "translation.z: %f",
    //             t_base_camera_msg.transform.translation.z);

    // Broadcast the transform
    tf_broadcaster_->sendTransform(t_base_camera_msg);

    compile_aruco_poses_file();
  }

  // Compile all the TF values obtained during robot trajectory detection inside
  // a text file. They will be used to get the approximate to the real TF value
  void compile_aruco_poses_file() {

    // Open a file named "tf_messages.txt" in append mode
    std::ofstream outputFile("/home/user/ros2_ws/src/final_project/"
                             "camera_calibration/config/tf_messages.txt",
                             std::ios::app);

    // Check if the file was opened successfully
    if (!outputFile.is_open()) {
      std::cerr << "Error: Unable to open the file for writing." << std::endl;
    }

    else {

      // Write some data to the file
      outputFile << t_base_camera_msg.transform.translation.x << " "
                 << t_base_camera_msg.transform.translation.y << " "
                 << t_base_camera_msg.transform.translation.z << " "
                 << t_base_camera_msg.transform.rotation.x << " "
                 << t_base_camera_msg.transform.rotation.y << " "
                 << t_base_camera_msg.transform.rotation.z << " "
                 << t_base_camera_msg.transform.rotation.w << std::endl;


      RCLCPP_INFO(this->get_logger(), "########################################"
                                      "###############################");

      // Close the file
      outputFile.close();

      std::cout << "Data successfully written to tf_messages.txt" << std::endl;
    }
  }

  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr
      subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Pose estimated_aruco_pose;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  // TF between the robot -- camera frames
  geometry_msgs::msg::TransformStamped t_aruco_base;
  geometry_msgs::msg::TransformStamped t_base_camera_msg;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRobotFramePublisher>());
  rclcpp::shutdown();
  return 0;
}