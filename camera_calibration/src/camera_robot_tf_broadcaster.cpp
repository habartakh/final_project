#include <algorithm>
#include <chrono>
#include <cstddef>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

    // Parameters declaration
    yaml_file_path =
        this->declare_parameter<std::string>("yaml_file_path", "data.txt");

    // The frames are named differently in the real and simulated robots
    base_frame =
        this->declare_parameter<std::string>("base_frame", "base_link");

    camera_frame = this->declare_parameter<std::string>(
        "camera_frame", "wrist_rgbd_camera_link");

    aruco_frame = this->declare_parameter<std::string>(
        "aruco_frame", "rg2_gripper_aruco_link");

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize the TF base_link - aruco marker listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco/pose", 10,
        std::bind(&CameraRobotFramePublisher::handle_aruco_pose, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), // 1 second interval
        std::bind(&CameraRobotFramePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "CameraRobotFramePublisher node initialized!");
  }

private:
  // Get the ArUco marker pose from the detection node topic
  void handle_aruco_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    // If a new message was received
    if (msg != nullptr) {

      // Add new pose to buffer
      pose_buffer_.push_back(msg->pose);

      // Keep buffer size limited
      if (pose_buffer_.size() > pose_buffer_size_) {
        pose_buffer_.pop_front();
      }

      new_message_published = true;
      estimated_aruco_pose = msg->pose;
      current_timestamp = msg->header.stamp;
    }
  }

  // Broadcast the TF camera -- base_link every second
  void timer_callback() {

    //  IMPORTANT: The aruco pose is estimated relative TO the CAMERA frame
    //  We are establishing the TF FROM the base_link so we need to project the
    //  aruco coordinates in the aruco (robot) frame. To do that, invert the
    //  position and orientation vectors

    // Look up for the transformation between base_link and
    // the aruco marker frames
    std::string fromFrameRel = aruco_frame;
    std::string toFrameRel = base_frame;
    try {
      t_aruco_base = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel, rclcpp::Time(current_timestamp));
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
    tf2::Matrix3x3 correction;
    // correction.setRPY(0, M_PI / 2, 0); // Rotate -90° around Y axis
    correction.setRPY(0, 0, 0);

    tf2::Quaternion correction_quat;
    correction.getRotation(correction_quat);

    tf2::Transform marker_frame_fix;
    marker_frame_fix.setIdentity();
    marker_frame_fix.setRotation(correction_quat);

    // Compute the median of the detected aruco poses to use next
    // This guarentees a more robust estimation as aruco poses are easily
    // influenced by external factors
    geometry_msgs::msg::Pose filtered_pose = compute_median_pose();

    // Build tf2 transform from OpenCV pose
    tf2::Transform tf_cam_marker;
    tf_cam_marker.setOrigin(tf2::Vector3(filtered_pose.position.x,
                                         filtered_pose.position.y,
                                         filtered_pose.position.z));
    tf_cam_marker.setRotation(tf2::Quaternion(
        filtered_pose.orientation.x, filtered_pose.orientation.y,
        filtered_pose.orientation.z, filtered_pose.orientation.w));

    // Apply correction
    tf_cam_aruco = marker_frame_fix * tf_cam_marker;

    // tf2::Vector3 trans(estimated_aruco_pose.position.x,
    //                    estimated_aruco_pose.position.y,
    //                    estimated_aruco_pose.position.z);

    // tf2::Quaternion rot(
    //     estimated_aruco_pose.orientation.x,
    //     estimated_aruco_pose.orientation.y,
    //     estimated_aruco_pose.orientation.z,
    //     estimated_aruco_pose.orientation.w);

    // tf_cam_aruco.setOrigin(trans);
    // tf_cam_aruco.setRotation(rot);

    // Invert the camera-to-aruco to get aruco-to-camera
    tf2::Transform tf_aruco_cam = tf_cam_aruco.inverse();

    // Now compute base_link -> camera
    tf2::Transform tf_base_camera = tf_aruco_base * tf_aruco_cam;

    // Convert back to ROS message

    t_base_camera_msg.header.stamp = this->get_clock()->now();
    t_base_camera_msg.header.frame_id = base_frame;
    t_base_camera_msg.child_frame_id = camera_frame;

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

    // Broadcast the transform
    tf_broadcaster_->sendTransform(t_base_camera_msg);

    // Add all TF values computed to a text file
    compile_aruco_poses_file();
  }

  geometry_msgs::msg::Pose compute_median_pose() {
    geometry_msgs::msg::Pose median_pose;

    if (pose_buffer_.empty())
      return median_pose;

    // Separate components
    std::vector<double> xs, ys, zs;
    std::vector<tf2::Quaternion> quaternions;

    for (const auto &pose : pose_buffer_) {
      xs.push_back(pose.position.x);
      ys.push_back(pose.position.y);
      zs.push_back(pose.position.z);
      tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
      quaternions.push_back(q);
    }

    // Compute medians
    auto median = [](std::vector<double> &vec) -> double {
      std::sort(vec.begin(), vec.end());
      return vec[vec.size() / 2];
    };

    median_pose.position.x = median(xs);
    median_pose.position.y = median(ys);
    median_pose.position.z = median(zs);

    // Average quaternion (approximate)
    tf2::Quaternion avg_q(0, 0, 0, 0);
    for (const auto &q : quaternions) {
      avg_q += q;
    }
    avg_q.normalize();

    median_pose.orientation.x = avg_q.x();
    median_pose.orientation.y = avg_q.y();
    median_pose.orientation.z = avg_q.z();
    median_pose.orientation.w = avg_q.w();

    return median_pose;
  }

  // Compile all the TF values obtained during robot trajectory detection inside
  // a text file. They will be used to get the approximate to the real TF value
  void compile_aruco_poses_file() {

    // Open a file named "tf_messages.txt" in append mode
    std::ofstream outputFile(yaml_file_path, std::ios::app);

    // Check if the file was opened successfully
    if (!outputFile.is_open()) {
      std::cerr << "Error: Unable to open the file for writing." << std::endl;
    }

    else if (!new_message_published) {
      RCLCPP_INFO(this->get_logger(), "No ArUco detected yet. Searching...");

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

      new_message_published = false;

      // Close the file
      outputFile.close();

      std::cout << "Data successfully written to tf_messages.txt" << std::endl;
    }
  }

  // Parameters
  std::string yaml_file_path;
  std::string base_frame;
  std::string camera_frame;
  std::string aruco_frame;

  // TF listeners Broadcasters
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Pose estimated_aruco_pose;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  bool new_message_published = false;

  // TF between the robot -- camera frames
  geometry_msgs::msg::TransformStamped t_aruco_base;
  geometry_msgs::msg::TransformStamped t_base_camera_msg;

  rclcpp::Time current_timestamp;

  // Store the N last aruco poses in a buffer to compute their median
  std::deque<geometry_msgs::msg::Pose> pose_buffer_;
  const size_t pose_buffer_size_ = 20;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRobotFramePublisher>());
  rclcpp::shutdown();
  return 0;
}