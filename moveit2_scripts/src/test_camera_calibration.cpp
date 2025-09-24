#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlaceTrajectory {
public:
  PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // Change the reference frame to the camera_frame
    std::string reference_frame = "D415_color_optical_frame";
    move_group_robot_->setPoseReferenceFrame(reference_frame);
    RCLCPP_INFO(LOGGER, "Reference frame set to: %s", reference_frame.c_str());

    // Change the end-effector frame
    // Get the current end-effector link
    std::string current_ee_link = move_group_robot_->getEndEffectorLink();
    RCLCPP_INFO(LOGGER, "Current end-effector link: %s",
                current_ee_link.c_str());

    // Set a new end-effector link
    // std::string new_ee_link =
    //     "rg2_gripper_right_finger"; // Replace with the name of your desired
    //                                 // end-effector link
    // move_group_robot_->setEndEffectorLink(new_ee_link);
    // RCLCPP_INFO(LOGGER, "New end-effector link set to: %s",
    //             new_ee_link.c_str());

    // get initial state of robot and gripper
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot and gripper
    initial_state_robot_ = move_group_robot_->getCurrentState(10);
    initial_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    initial_state_gripper_ = move_group_gripper_->getCurrentState(10);
    initial_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // Store the initial joints positions to use later to make the robot return
    // to its initial position at the end of the pick and place sequence
    initial_joint_group_positions_robot_ = joint_group_positions_robot_;
    initial_joint_group_positions_gripper_ = joint_group_positions_gripper_;

    // set start state of robot and gripper to current state
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  }

  ~PickAndPlaceTrajectory() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Going to test Position...");

    // Test the goal pose target in World frame : Successful
    // setup_goal_pose_target(-0.369, 0.120, -0.141, -1.000, 0.000, 0.000,
    // 0.000);

    // Test the pose target in Camera Frame
    // Point in camera_frame: x=-0.904, y=0.309, z=-0.159
    // setup_goal_pose_target(-0.904, 0.309, -0.159, -1.000, 0.000, 0.000,
    // 0.000);

    // Using provided Tf: x=0.183, y=-0.915, z=0.439 : Not Successful
    // setup_goal_pose_target(0.183, -0.915, 0.439, -1.000, 0.000, 0.000,
    // 0.000);

    // Successful: - label: circle_1
    //   pose:
    //     header:
    //       stamp:
    //         sec: 1758571890
    //         nanosec: 34108416
    //       frame_id: D415_color_optical_frame
    //     pose:
    //       position:
    //         x: 0.0015841652525555648
    //         y: 0.2274346302429275
    //         z: 0.8707875627193555
    // setup_goal_pose_target(0.001, 0.2274, 0.4707, 0.000, 0.000,
    // 0.000, 1.000);

    // Better results!
    // setup_goal_pose_target(0.001, 0.2274, 0.080, 0.000, 0.000, 0.000, 1.000);

    setup_goal_pose_target(-0.0965, 0.1793, 0.485, 0.000, 0.000, 0.000, 1.000);

    // test 166:
    // x: -0.06288740805348997
    //     y: 0.1704637111203776
    //     z: 0.89983959520538

    setup_goal_pose_target(-0.0628, 0.1704, 0.4998, 0.000, 0.000, 0.000, 1.000);

    // plan and execute the trajectory
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for robot and gripper
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // declare joint_model_group for robot and gripper
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for robot and gripper
  std::vector<double> joint_group_positions_robot_;
  std::vector<double> initial_joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  RobotStatePtr initial_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;
  std::vector<double> joint_group_positions_gripper_;
  std::vector<double> initial_joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  RobotStatePtr initial_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(std::string pose_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    // plan the gripper action
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    // execute the planned gripper action
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

  void go_to_test_position() {

    RCLCPP_INFO(LOGGER, "Going to Pregrasp Position...");
    // setup the goal pose target
    // Best coordinates for pregrasp, adjusted through Gazebo and RVIZ
    setup_goal_pose_target(+0.3385, -0.020, +0.250, -1.000, 0.000, 0.000,
                           0.000);

    // plan and execute the trajectory
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

  void return_to_initial_position() {

    RCLCPP_INFO(LOGGER, "Returning to the initial pose...");

    // Return to the initial state of the robot
    move_group_robot_->setJointValueTarget(
        initial_joint_group_positions_robot_);

    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    // Then set the gripper to its initial position
    move_group_gripper_->setJointValueTarget(
        initial_joint_group_positions_gripper_);
    plan_trajectory_gripper();
    execute_trajectory_gripper();
  }

}; // class PickAndPlaceTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place_trajectory");

  // instantiate class
  PickAndPlaceTrajectory pick_and_place_trajectory_node(base_node);

  // execute trajectory plan
  pick_and_place_trajectory_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}