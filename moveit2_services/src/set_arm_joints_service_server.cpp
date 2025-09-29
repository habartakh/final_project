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

class ArmJointsServiceServer : public rclcpp::Node {
public:
  using ArmJoints = moveit2_services::srv::ArmJoints;

  ArmJointsServiceServer() : Node("arm_joints_service_server") {
    RCLCPP_INFO(this->get_logger(), "Starting ArmJointsServiceServer...");

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

    // Create service server
    service_ = this->create_service<ArmJoints>(
        "set_arm_joints",
        std::bind(&ArmJointsServiceServer::handle_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Arm Joints Service Server ready.");
  }

private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  const moveit::core::JointModelGroup *joint_model_group_;

  rclcpp::Service<ArmJoints>::SharedPtr service_;

  void handle_service(const std::shared_ptr<ArmJoints::Request> request,
                      std::shared_ptr<ArmJoints::Response> response) {
    const auto &joints = request->input_array.data;

    RCLCPP_INFO(this->get_logger(),
                "Received service request with %lu joint values",
                joints.size());

    // If the requested joint values format is wrong
    if (joints.size() != joint_model_group_->getVariableCount()) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid joint value size: %lu (expected: %u)", joints.size(),
                  joint_model_group_->getVariableCount());
      response->success = false;
      return;
    }

    move_group_->setJointValueTarget(joints);

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
  rclcpp::spin(std::make_shared<ArmJointsServiceServer>());
  rclcpp::shutdown();
  return 0;
}
