#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit2_services/srv/arm_joints.hpp"

static const std::string PLANNING_GROUP = "ur_manipulator";

// This service server takes an array of joints values and input and
// moves the robot so that it could achieve the requested position
// It returns a boolean indicating if the requested position was reached or not

class GoToPoseServiceServer : public rclcpp::Node {
public:
  using ArmJoints = moveit2_services::srv::ArmJoints;

  GoToPoseServiceServer() : Node("go_to_pose_service_server") {
    RCLCPP_INFO(this->get_logger(), "Starting GoToPoseServiceServer...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // Spin a MoveGroup node in background
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { executor_.spin(); }).detach();

    // Initialize MoveIt interface
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, PLANNING_GROUP);
    joint_model_group_ =
        move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Change the reference frame to the camera_frame

    // Declare parameter with default
    this->declare_parameter<std::string>("reference_frame",
                                         "D415_color_optical_frame");

    // Get parameter value
    std::string reference_frame =
        this->get_parameter("reference_frame").as_string();

    move_group_->setPoseReferenceFrame(reference_frame);
    RCLCPP_INFO(this->get_logger(), "Reference frame set to: %s",
                reference_frame.c_str());

    // Change the end-effector frame
    // Get the current end-effector link
    std::string current_ee_link = move_group_->getEndEffectorLink();
    RCLCPP_INFO(this->get_logger(), "Current end-effector link: %s",
                current_ee_link.c_str());

    // Create service server
    service_ = this->create_service<ArmJoints>(
        "go_to_pose", std::bind(&GoToPoseServiceServer::handle_service, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Go To Pose Service Server ready.");
  }

private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  const moveit::core::JointModelGroup *joint_model_group_;

  rclcpp::Service<ArmJoints>::SharedPtr service_;
  geometry_msgs::msg::Pose target_pose_robot_;

  void handle_service(const std::shared_ptr<ArmJoints::Request> request,
                      std::shared_ptr<ArmJoints::Response> response) {
    const auto &goal_pose = request->input_array.data;

    RCLCPP_INFO(this->get_logger(),
                "Received service request with %lu joint values",
                goal_pose.size());

    // If the requested joint values format is wrong
    if (goal_pose.size() != 7) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid goal pose size: %lu (expected: 7)",
                  goal_pose.size());
      response->success = false;
      return;
    }

    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = goal_pose[0];
    target_pose_robot_.position.y = goal_pose[1];
    target_pose_robot_.position.z = goal_pose[2];
    target_pose_robot_.orientation.x = goal_pose[3];
    target_pose_robot_.orientation.y = goal_pose[4];
    target_pose_robot_.orientation.z = goal_pose[5];
    target_pose_robot_.orientation.w = goal_pose[6];

    move_group_->setPoseTarget(target_pose_robot_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success =
        (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
      move_group_->execute(plan);
      response->success = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      response->success = false;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPoseServiceServer>());
  rclcpp::shutdown();
  return 0;
}
