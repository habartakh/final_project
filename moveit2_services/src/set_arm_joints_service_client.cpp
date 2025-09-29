#include "moveit2_services/srv/arm_joints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class ArmJointsServiceClient : public rclcpp::Node {
public:
  ArmJointsServiceClient() : Node("arm_joints_service_client") {

    client_ =
        this->create_client<moveit2_services::srv::ArmJoints>("set_arm_joints");

    // Wait for the service to become available
    while (!client_->wait_for_service(5s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for the service to be available...");
    }

    // Prepare the request
    auto request =
        std::make_shared<moveit2_services::srv::ArmJoints::Request>();

    // Example joint values: you can change this according to your robot's DoF
    std::vector<double> joint_values = {0.245,  -2.674, -1.915,
                                        -0.834, 2.821,  -0.491};
    request->input_array.data = joint_values;

    // Send the request
    auto result_future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(),
                    "Joint values successfully sent and executed!");
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to execute joint trajectory.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
    }
  }

private:
  rclcpp::Client<moveit2_services::srv::ArmJoints>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmJointsServiceClient>();
  rclcpp::shutdown();
  return 0;
}
