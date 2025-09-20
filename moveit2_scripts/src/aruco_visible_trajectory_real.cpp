#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class ArucoVisibleTrajectoryReal {
public:
  ArucoVisibleTrajectoryReal(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(
        LOGGER,
        "Initializing Class: ArUco Visible Trajectory in Real Environment...");

    // Extract the arm trajectory joint values
    extract_joint_values_from_yaml("arm_joints_real.yaml");

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

    // Store the initial joints positions to use later to make the robot return
    // to its initial position at the end of the ArUco Visible Trajectory
    // sequence
    initial_joint_group_positions_robot_ = joint_group_positions_robot_;

    // set start state of robot and gripper to current state
    move_group_robot_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER,
                "Class Initialized: ArUco Visible Trajectory Trajectory");
  }

  ~ArucoVisibleTrajectoryReal() {
    // indicate termination
    RCLCPP_INFO(LOGGER,
                "Class Terminated: ArUco Visible Trajectory Trajectory");
  }

  // The main function executed: moves the ArUco marker in a circular manner
  void execute_trajectory_plan() {
    RCLCPP_INFO(
        LOGGER,
        "Planning and Executing ArUco Visible Trajectory Trajectory...");

    // Start moving the arm to the waypoints from YAML file
    start_aruco_visible_trajectory();

    // wait 2 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO(LOGGER,
                "ArUco Visible Trajectory Trajectory Execution Complete");
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
  Plan gripper_trajectory_plan_;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  // The values of the joints extracted from the YAML file
  std::vector<std::vector<double>> arm_joints_trajectory;

  void extract_joint_values_from_yaml(const std::string &yaml_file_name) {

    // Get the package directory at runtime
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("moveit2_scripts");

    std::string yaml_file_path =
        package_share_directory + "/config/" + yaml_file_name;

    // Load the YAML File
    YAML::Node config = YAML::LoadFile(yaml_file_path);
    const auto &trajectory_joints = config["trajectory_joints"];

    for (const auto waypoint : trajectory_joints) {
      const auto joint_values =
          waypoint["joint_values"].as<std::vector<double>>();
      arm_joints_trajectory.push_back(joint_values);
    }
  }

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
      // RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
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

  // Return the robot arm to its initial configuration
  void return_to_initial_position() {

    RCLCPP_INFO(LOGGER, "Returning to the initial pose...");

    // Return to the initial state of the robot
    move_group_robot_->setJointValueTarget(
        initial_joint_group_positions_robot_);

    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

  // Move the arm in a set trajectory obtained through the yaml file
  // The trajectory parameters were finetuned using RVIZ
  void start_aruco_visible_trajectory() {

    // Set the joints to go to each waypoint
    for (auto &joints_vector : arm_joints_trajectory) {

      std::cout << "#######################################" << std::endl;
      std::cout << "NEW ARUCO POSE !! " << std::endl;
      std::cout << "#######################################" << std::endl;
      // Set up the joints values to the appropriate position
      setup_joint_value_target(joints_vector[0], joints_vector[1],
                               joints_vector[2], joints_vector[3],
                               joints_vector[4], joints_vector[5]);

      // plan and execute the cartesian trajectory
      plan_trajectory_kinematics();
      execute_trajectory_kinematics();

      //  Wait for 5 seconds before rotation
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));

      // For each waypoint of the trajectory, rotate the marker from right to
      // left, then downwards
      rotate_gripper();

      // Stay in the same position for 2 seconds
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
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

    for (int i = -2; i < 3; ++i) {
      double gripper_joint_value = base_gripper_value + i * 0.3;
      move_gripper_to(gripper_joint_value, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    // Return the ArUco marker to its initial orientation
    move_gripper_to(base_gripper_value, true);
  }

  // Perform a rotation downwards
  void perform_gripper_vertical_motion(double base_gripper_value) {

    for (int i = 0; i < 4; ++i) {
      double gripper_joint_value = base_gripper_value - i * 0.2;
      move_gripper_to(gripper_joint_value, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

    // Remove this return since it generates false TF computations
    // Bacause the detection cannot update the aruco position swiftly with rapid
    // arm movements

    // Return the ArUco marker to its initial orientation
    // move_gripper_to(base_gripper_value, false);
  }

  // Rotate the gripper 5 times to detect different marker positions
  void rotate_gripper() {
    RCLCPP_INFO(LOGGER,
                "Rotating the gripper horizontally for this instance...");

    // Get the robot's current joint values
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    double base_gripper_value = joint_group_positions_robot_[5];
    double base_wrist_value = joint_group_positions_robot_[4];

    // Perform motion from right to left and vice versa
    perform_gripper_horizontal_motion(base_gripper_value);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Rotate the robot vertically
    perform_gripper_vertical_motion(base_wrist_value);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

}; // class ArucoVisibleTrajectoryReal

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("aruco_visible_trajectory_real");

  // instantiate class
  ArucoVisibleTrajectoryReal aruco_visible_trajectory_node(base_node);

  // execute trajectory plan
  aruco_visible_trajectory_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}