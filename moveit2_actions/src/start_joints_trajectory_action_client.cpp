#include <functional>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <string>

#include "moveit2_actions/action/set_trajectory_joint_values.hpp"
#include "moveit2_actions/msg/joint_values.hpp"

using namespace std::chrono_literals;

class JointTrajectoryActionClient : public rclcpp::Node {
public:
  using SetTrajectoryJointValues =
      moveit2_actions::action::SetTrajectoryJointValues;
  using GoalHandleSetTrajectoryJointValues =
      rclcpp_action::ClientGoalHandle<SetTrajectoryJointValues>;

  JointTrajectoryActionClient() : Node("joint_trajectory_action_client") {
    this->client_ptr_ = rclcpp_action::create_client<SetTrajectoryJointValues>(
        this, "set_joint_values");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&JointTrajectoryActionClient::send_goal, this));
  }

  void send_goal() {

    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      rclcpp::shutdown();
      return;
    }

    // Create goal message
    auto goal_msg = SetTrajectoryJointValues::Goal();

    moveit2_actions::msg::JointValues waypoint1;
    waypoint1.positions = {0.19, -2.6, -1.31, -0.42, 2.58, -1.48};

    moveit2_actions::msg::JointValues waypoint2;
    waypoint2.positions = {0.23, -3.02, -0.26, -0.98, 2.56, -1.41};

    moveit2_actions::msg::JointValues waypoint3;
    waypoint3.positions = {0.24, -2.70, -1.08, -0.47, 2.56, -1.4};

    goal_msg.trajectory.push_back(waypoint1);
    goal_msg.trajectory.push_back(waypoint2);
    goal_msg.trajectory.push_back(waypoint3);

    RCLCPP_INFO(this->get_logger(), "Sending goal with %zu waypoints...",
                goal_msg.trajectory.size());

    auto send_goal_options =
        rclcpp_action::Client<SetTrajectoryJointValues>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(
        &JointTrajectoryActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(
        &JointTrajectoryActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&JointTrajectoryActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<SetTrajectoryJointValues>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(
      GoalHandleSetTrajectoryJointValues::SharedPtr goal_handle) {

    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by the server, waiting for result...");
    }
  }

  void feedback_callback(
      GoalHandleSetTrajectoryJointValues::SharedPtr,
      const std::shared_ptr<const SetTrajectoryJointValues::Feedback>
          feedback) {

    double progress = feedback->progress;
    RCLCPP_INFO(this->get_logger(), "Current progress is: %f", progress);
  }

  void result_callback(
      const GoalHandleSetTrajectoryJointValues::WrappedResult &result) {

    bool trajectory_success = result.result->success;
    if (trajectory_success) {
      RCLCPP_INFO(this->get_logger(), "Trajectory is SUCCESSFUl");
    } else {
      RCLCPP_INFO(this->get_logger(), "Trajectory is NOT SUCCESSFUl");
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryActionClient>());
  rclcpp::shutdown();
  return 0;
}
