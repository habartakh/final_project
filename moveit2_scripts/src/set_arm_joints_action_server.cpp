#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "moveit2_scripts/action/set_joint_values.hpp"

static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class JointTrajectoryActionServer : public rclcpp::Node {
public:
  using SetJointValues = moveit2_scripts::action::SetJointValues;
  using GoalHandleSetJointValues =
      rclcpp_action::ServerGoalHandle<SetJointValues>;

  JointTrajectoryActionServer() : Node("joint_trajectory_action_server") {
    RCLCPP_INFO(this->get_logger(), "Starting JointTrajectoryActionServer...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // Initialize move_group
    move_group_node_ = rclcpp::Node::make_shared("move_group_node");
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "ur_manipulator");
    joint_model_group_ =
        move_group_->getCurrentState()->getJointModelGroup("ur_manipulator");

    // Create the action server
    action_server_ = rclcpp_action::create_server<SetJointValues>(
        this, "set_joint_values",
        std::bind(&JointTrajectoryActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointTrajectoryActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&JointTrajectoryActionServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JointTrajectoryActionServer ready.");
  }

private:
  // MoveIt
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  const moveit::core::JointModelGroup *joint_model_group_;

  // Action server
  rclcpp_action::Server<SetJointValues>::SharedPtr action_server_;

  // Goal Handling
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const SetJointValues::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal with %ld joint values",
                goal->joint_values.size());

    if (goal->joint_values.size() != joint_model_group_->getVariableCount()) {
      RCLCPP_WARN(this->get_logger(), "Invalid joint value size. Rejecting.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleSetJointValues> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleSetJointValues> goal_handle) {
    std::thread{
        std::bind(&JointTrajectoryActionServer::execute, this, goal_handle)}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleSetJointValues> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    const auto goal = goal_handle->get_goal();

    move_group_->setJointValueTarget(goal->joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success =
        (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    auto feedback = std::make_shared<SetJointValues::Feedback>();
    feedback->progress = 0.5f;
    goal_handle->publish_feedback(feedback);

    if (success) {
      move_group_->execute(plan);
      feedback->progress = 1.0f;
      goal_handle->publish_feedback(feedback);

      auto result = std::make_shared<SetJointValues::Result>();
      result->success = true;
      result->message = "Trajectory executed successfully";
      goal_handle->succeed(result);
    } else {
      auto result = std::make_shared<SetJointValues::Result>();
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryActionServer>());
  rclcpp::shutdown();
  return 0;
}