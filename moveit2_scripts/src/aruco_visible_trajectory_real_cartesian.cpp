#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class ArucoVisibleTrajectory {
public:
  ArucoVisibleTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: ArUco Visible Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // Initialize a subscriber node to get the detected object position
    start_signal_node =
        rclcpp::Node::make_shared("start_signal_node_real", node_options);
    start_signal_sub =
        start_signal_node->create_subscription<std_msgs::msg::Bool>(
            "start_signal_topic", 10,
            std::bind(&ArucoVisibleTrajectory::topic_callback, this, _1));

    progress_pub = start_signal_node->create_publisher<std_msgs::msg::Int16>(
        "trajectory_progress", 10);
    progress_timer = start_signal_node->create_wall_timer(
        500ms, std::bind(&ArucoVisibleTrajectory::timer_callback, this));

    start_signal_future_ = start_signal_promise_.get_future();
    executor_.add_node(start_signal_node);

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

  ~ArucoVisibleTrajectory() {
    // indicate termination
    RCLCPP_INFO(LOGGER,
                "Class Terminated: ArUco Visible Trajectory Trajectory");
  }

  // The main function executed: moves the ArUco marker in a circular manner
  void execute_trajectory_plan() {
    if (start_trajectory) {

      RCLCPP_INFO(
          LOGGER,
          "Planning and Executing ArUco Visible Trajectory Trajectory...");

      // First, start from a configuration where the ArUco marker is visible
      // setup_arm_first_configuration();

      // wait 2 seconds
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Start the circular trajectory pipeline
      start_aruco_visible_trajectory();

      // After finishing the task, return to the original position
      // return_to_initial_position();

      RCLCPP_INFO(LOGGER,
                  "ArUco Visible Trajectory Trajectory Execution Complete");

      start_trajectory = false;

      // Publish an illogical progress value to indicate the end of the movement
      current_progress = 200;

      // Wait for 5 seconds before exiting
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
  }

  void wait_for_start_signal() { start_signal_future_.wait(); }

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

  // subscriber to get the detected object position
  rclcpp::Node::SharedPtr start_signal_node;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_signal_sub;

  // Publisher to share the progress of the trajectory
  rclcpp::TimerBase::SharedPtr progress_timer;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr progress_pub;
  double current_progress = 0.0;
  bool start_trajectory = false;

  // Get the object detected position asynchronously with future object
  std::promise<std_msgs::msg::Bool> start_signal_promise_;
  std::future<std_msgs::msg::Bool> start_signal_future_;

  // declare single threaded executor for move_group node
  rclcpp::executors::MultiThreadedExecutor executor_;

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

  // Set up the joints to make the ArUco visible
  // Values were obtained from the moveit RVIZ GUI
  void setup_arm_first_configuration() {

    setup_joint_value_target(0.08, -2.7, -1.39, -0.38, 2.62, -1.38);

    //  plan and execute the trajectory
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

  // Move the arm in a circular trajectory while making sure the ArUco Marker
  // stays visible. The trajectory parameters were finetuned using RVIZ
  void start_aruco_visible_trajectory() {
    int total_steps = 10; // Number of stops during the motion
    double delta_x = 0.0;
    double delta_y = 0.0;

    for (int i = 0; i < total_steps; ++i) {

      std::cout << "####################################################"
                << std::endl;
      std::cout << "NEW WAYPOINT !! " << std::endl;

      current_progress = ((static_cast<double>(i) + 1.0) / total_steps) * 100.0;

      double angle = (2.0 * M_PI * i) / total_steps;

      delta_x = 0.02 * cos(angle);
      delta_y = -0.02 * sin(angle);
      setup_waypoints_target(delta_x, delta_y, 0.000);

      // plan and execute the cartesian trajectory
      plan_trajectory_cartesian();

      // std::this_thread::sleep_for(std::chrono::milliseconds(20000));

      execute_trajectory_cartesian();

      //  Wait for 5 seconds before rotation
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // For each waypoint of circular trajectory, rotate the marker 5 times
      rotate_gripper();

      // Stay in the same position for 2 seconds
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
  }

  // Change the gripper's orientation to a given gripper_joint_value
  void move_gripper_to(double gripper_joint_value) {
    setup_joint_value_target(
        joint_group_positions_robot_[0], joint_group_positions_robot_[1],
        joint_group_positions_robot_[2], joint_group_positions_robot_[3],
        joint_group_positions_robot_[4], gripper_joint_value);

    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

  // Perform a rotation from right to left of the ArUco marker
  void perform_gripper_motion(double base_gripper_value) {

    for (int i = -1; i < 2; ++i) {
      double gripper_joint_value = base_gripper_value + i * 0.2;
      move_gripper_to(gripper_joint_value);
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    // Return the ArUco marker to its initial orientation
    move_gripper_to(base_gripper_value);
  }

  // Rotate the gripper 5 times to detect different marker positions
  void rotate_gripper() {
    RCLCPP_INFO(LOGGER, "Rotating the gripper for this instance...");

    // Get the robot's current joint values
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    double base_gripper_value = joint_group_positions_robot_[5];

    // Perform motion in both directions
    perform_gripper_motion(base_gripper_value);
  }

  // Detected object position subscriber callback
  // returns the object's position in reference to the base_link of the arm
  void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Looking for the start signal...");

    // If The signal was sent
    if (msg != nullptr) {

      start_signal_promise_.set_value(*msg); // Fulfilled the promise

      start_trajectory = true;

      // Then unsubscribe from the topic
      start_signal_sub.reset();
    }
  }

  void timer_callback() {

    auto message = std_msgs::msg::Int16();
    message.data = (int)current_progress;
    RCLCPP_INFO(LOGGER, "Current Progress: '%d'", message.data);
    progress_pub->publish(message);
  }

}; // class ArucoVisibleTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("aruco_visible_trajectory_real_cartesian");

  // instantiate class
  ArucoVisibleTrajectory aruco_visible_trajectory_node(base_node);

  // Block until the start signal is received
  RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for start signal...");
  aruco_visible_trajectory_node.wait_for_start_signal();

  // execute trajectory plan
  aruco_visible_trajectory_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}