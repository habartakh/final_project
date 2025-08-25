#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class ArucoCameraFramePublisher : public rclcpp::Node {
public:
  ArucoCameraFramePublisher() : Node("aruco_camera_frame_publisher") {

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ =
        this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
            "/aruco/markers", 10,
            std::bind(&ArucoCameraFramePublisher::handle_aruco_pose, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "ArucoCameraFramePublisher node initialized!");
  }

private:
  // Broadcast the TF between the camera and detected ArUco marker frames
  void handle_aruco_pose(
      const std::shared_ptr<aruco_interfaces::msg::ArucoMarkers> msg) {
    geometry_msgs::msg::TransformStamped t;

    // NOTE: The /aruco/marker topic publishes the ArUco pose relative to the
    // camera's coordinate system
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "wrist_rgbd_camera_link";
    t.child_frame_id = "aruco_frame";

    // Get the ArUco marker position and orientation via the ArUco detection
    // and pose estimation node

    t.transform.translation.x = msg->poses[0].position.x;
    t.transform.translation.y = msg->poses[0].position.y;
    t.transform.translation.z = msg->poses[0].position.z;

    t.transform.rotation.x = msg->poses[0].orientation.x;
    t.transform.rotation.y = msg->poses[0].orientation.y;
    t.transform.rotation.z = msg->poses[0].orientation.z;
    t.transform.rotation.w = msg->poses[0].orientation.w;

    RCLCPP_INFO(this->get_logger(), "t.transform.rotation.x: %f",
                t.transform.rotation.x);

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr
      subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoCameraFramePublisher>());
  rclcpp::shutdown();
  return 0;
}