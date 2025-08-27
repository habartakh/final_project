#include <functional>
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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class CameraRobotFramePublisher : public rclcpp::Node {
public:
  CameraRobotFramePublisher() : Node("aruco_camera_frame_publisher") {

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize the 

    subscription_ =
        this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
            "/aruco/markers", 10,
            std::bind(&CameraRobotFramePublisher::handle_aruco_pose, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "CameraRobotFramePublisher node initialized!");
  }

private:
  // Broadcast the TF between the camera and the robot base_link frames
  void handle_aruco_pose(
      const std::shared_ptr<aruco_interfaces::msg::ArucoMarkers> msg) {
    geometry_msgs::msg::TransformStamped t;

    // The TF is computed FROM the robot base_link TO the camera frame
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "wrist_rgbd_camera_link";

    //  IMPORTANT: The aruco pose is estimated relative TO the CAMERA frame
    //  We are establishing the TF FROM the base_link so we need to project the
    //  aruco coordinates in the aruco (robot) frame To do that, invert the
    //  position and orientation vectors
    estimated_aruco_pose = msg->poses[0];

    // Computing the TF base_link -- camera amount to doing the following:
    // TF(base_link - cam) = TF(base_link - aruco) + Coordinates(cam - aruco)

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
  geometry_msgs::msg::Pose estimated_aruco_pose;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRobotFramePublisher>());
  rclcpp::shutdown();
  return 0;
}