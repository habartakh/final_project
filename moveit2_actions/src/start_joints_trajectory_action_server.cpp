#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "moveit2_actions/action/set_trajectory_joint_values.hpp"
#include "moveit2_actions/msg/joint_values.hpp"

static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class JointTrajectoryActionServer : public rclcpp::Node {
public:
  using SetTrajectoryJointValues =
      moveit2_actions::action::SetTrajectoryJointValues;
  using GoalHandleSetTrajectoryJointValues =
      rclcpp_action::ServerGoalHandle<SetTrajectoryJointValues>;

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

    // get initial state of robot
    joint_model_group_ =
        move_group_->getCurrentState()->getJointModelGroup("ur_manipulator");

    // Create the action server
    action_server_ = rclcpp_action::create_server<SetTrajectoryJointValues>(
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
  moveit::planning_interface::MoveGroupInterface::Plan
      kinematics_trajectory_plan_;
  bool plan_success_robot_ = false;

  // declare trajectory planning variables for robot
  std::vector<double> joint_group_positions_robot_;
  moveit::core::RobotStatePtr current_state_robot_;

  // Action server
  rclcpp_action::Server<SetTrajectoryJointValues>::SharedPtr action_server_;

  // Goal Handling
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const SetTrajectoryJointValues::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal with %ld waypoints",
                goal->trajectory.size());

    for (const auto &waypoint : goal->trajectory) {
      if (waypoint.positions.size() != joint_model_group_->getVariableCount()) {
        RCLCPP_WARN(this->get_logger(), "Invalid waypoint size. Rejecting.");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleSetTrajectoryJointValues> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleSetTrajectoryJointValues> goal_handle) {
    std::thread{
        std::bind(&JointTrajectoryActionServer::execute, this, goal_handle)}
        .detach();
  }

  void execute(
      const std::shared_ptr<GoalHandleSetTrajectoryJointValues> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<SetTrajectoryJointValues::Feedback>();

    size_t total = goal->trajectory.size();

    for (size_t i = 0; i < total; ++i) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<SetTrajectoryJointValues::Result>();
        result->success = false;
        result->message = "Goal canceled";
        goal_handle->canceled(result);
        return;
      }

      const auto &waypoint = goal->trajectory[i];
      start_aruco_visible_trajectory(waypoint.positions);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success =
          (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (!success) {
        auto result = std::make_shared<SetTrajectoryJointValues::Result>();
        result->success = false;
        result->message = "Planning failed at waypoint " + std::to_string(i);
        goal_handle->abort(result);
        return;
      }

      move_group_->execute(plan);

      feedback->progress = static_cast<float>(i + 1) / total;
      goal_handle->publish_feedback(feedback);
    }

    auto result = std::make_shared<SetTrajectoryJointValues::Result>();
    result->success = true;
    result->message = "Trajectory executed successfully";
    goal_handle->succeed(result);
  }

  // Actions for robot arm movement

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_->setJointValueTarget(joint_group_positions_robot_);
  }

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ = (move_group_->plan(kinematics_trajectory_plan_) ==
                           moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_->execute(kinematics_trajectory_plan_);
      // RCLCPP_INFO(this->get_logger(), "Robot Kinematics Trajectory Success
      // !");
    } else {
      RCLCPP_INFO(this->get_logger(), "Robot Kinematics Trajectory Failed !");
    }
  }
  // Move the arm in a set trajectory obtained through the yaml file
  // The trajectory parameters were finetuned using RVIZ
  void
  start_aruco_visible_trajectory(const std::vector<double> &joint_positions) {

    // Set the joints to go to each waypoint

    // Set up the joints values to the appropriate position
    move_group_->setJointValueTarget(joint_positions);

    // plan and execute the cartesian trajectory
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    //  Wait for 5 seconds before rotation
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // For each waypoint of the trajectory, rotate the marker from right to
    // left, then downwards
    rotate_gripper();

    // Stay in the same position for 2 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  // Change the gripper's orientation to a given gripper_joint_value
  void move_gripper_to(double gripper_joint_value, bool is_horizontal) {

    // Rotate the gripper either horizontally
    if (is_horizontal) {
      setup_joint_value_target(
          joint_group_positions_robot_[0], joint_group_positions_robot_[1],
          joint_group_positions_robot_[2], joint_group_positions_robot_[3],
          joint_group_positions_robot_[4], gripper_joint_value);
    }
    // Or vertically
    else {
      setup_joint_value_target(
          joint_group_positions_robot_[0], joint_group_positions_robot_[1],
          joint_group_positions_robot_[2], joint_group_positions_robot_[3],
          gripper_joint_value, joint_group_positions_robot_[5]);
    }

    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

  // Perform a rotation from right to left of the ArUco marker
  void perform_gripper_horizontal_motion(double base_gripper_value) {

    for (int i = -1; i < 2; ++i) {
      double gripper_joint_value = base_gripper_value + i * 0.4;
      move_gripper_to(gripper_joint_value, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    // Return the ArUco marker to its initial orientation
    move_gripper_to(base_gripper_value, true);
  }

  // Perform a rotation downwards
  void perform_gripper_vertical_motion(double base_gripper_value) {

    for (int i = 0; i < 3; ++i) {
      double gripper_joint_value = base_gripper_value - i * 0.3;
      move_gripper_to(gripper_joint_value, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
  }

  // Rotate the gripper 5 times to detect different marker positions
  void rotate_gripper() {
    RCLCPP_INFO(this->get_logger(),
                "Rotating the gripper horizontally for this instance...");

    // Get the robot's current joint values
    current_state_robot_ = move_group_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_,
                                                  joint_group_positions_robot_);

    double base_gripper_value = joint_group_positions_robot_[5];
    double base_wrist_value = joint_group_positions_robot_[4];

    // Perform motion from right to left and vice versa
    perform_gripper_horizontal_motion(base_gripper_value);

    // Rotate the robot vertically
    perform_gripper_vertical_motion(base_wrist_value);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryActionServer>());
  rclcpp::shutdown();
  return 0;
}