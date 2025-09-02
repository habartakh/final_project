#include "moveit2_scripts/action/set_joint_values.hpp" 
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using SetJointValues = moveit2_scripts::action::SetJointValues;
using GoalHandleSetJointValues =
    rclcpp_action::ClientGoalHandle<SetJointValues>;

class JointClient : public rclcpp::Node {
public:
  JointClient() : Node("set_arm_joints_client") {
    client_ =
        rclcpp_action::create_client<SetJointValues>(this, "set_joint_values");

    // Wait for the server
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    // Prepare the goal
    auto goal_msg = SetJointValues::Goal();
    goal_msg.joint_values = {0.245, -2.674, -1.915, -0.834, 2.821, -0.491};

    RCLCPP_INFO(this->get_logger(), "Sending goal...");
    auto send_goal_options =
        rclcpp_action::Client<SetJointValues>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [this](GoalHandleSetJointValues::SharedPtr,
               const std::shared_ptr<const SetJointValues::Feedback> feedback) {
          RCLCPP_INFO(this->get_logger(), "Feedback: progress = %.2f%%",
                      feedback->progress * 100.0f);
        };

    send_goal_options.result_callback =
        [this](const GoalHandleSetJointValues::WrappedResult &result) {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
                        "Result: success = %d, message = %s",
                        result.result->success, result.result->message.c_str());
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
          }

          // Shutdown after result
          rclcpp::shutdown();
        };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<SetJointValues>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
