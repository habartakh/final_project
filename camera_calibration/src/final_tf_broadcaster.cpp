#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

#include "std_msgs/msg/bool.hpp"

// This structure stores the TF values compiled in the text file
struct Pose {
  double x, y, z;
  double qx, qy, qz, qw;
};

// Test if the same TF value is repeated
bool isDuplicate(const Pose &a, const Pose &b) {
  return (a.x == b.x) && (a.y == b.y);
}

/**********************************************************************
 This node processes all the TF values collected during the arm movement
 and averages them to get an accurate TF that is broadcasted afterwards
***********************************************************************/
class FinalTfBroadcaster : public rclcpp::Node {
public:
  FinalTfBroadcaster() : Node("final_tf_broadcaster") {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    file_path = this->declare_parameter<std::string>("file_path", "data.txt");

    // The frames are named differently in the real and simulated robots
    base_frame =
        this->declare_parameter<std::string>("base_frame", "base_link");

    camera_frame = this->declare_parameter<std::string>(
        "camera_frame", "wrist_rgbd_camera_link");

    // Only trigger the timer after receiving the start signal
    start_signal_sub = this->create_subscription<std_msgs::msg::Bool>(
        "start_tf_broadcast_topic", 10,
        std::bind(&FinalTfBroadcaster::start_signal_callback, this,
                  std::placeholders::_1));

    // Broadcast the computed base_link -- camera TF periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FinalTfBroadcaster::broadcastTransform, this));

    // Stop the timer from activating at first
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(),
                "Node Launched. Waiting for the start signal....");
  }

private:
  std::string base_frame;
  std::string camera_frame;
  std::string file_path;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_signal_sub;
  bool average_computed = false;

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped averaged_tf_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Eigen::Quaterniond> quaternions;
  std::vector<Eigen::Vector3d> positions;

  // Wait till hearing the start signal
  void start_signal_callback(const std_msgs::msg::Bool::SharedPtr msg) {

    // If The signal was sent
    if (msg != nullptr) {

      RCLCPP_INFO_ONCE(this->get_logger(),
                       "Start signal is heard! Timer will be started");

      // Then start the timer recording the TF values
      timer_->reset();
    }
  }

  // Computes the average of all the TFs collected during the robot trajectory
  bool readAndAveragePoses(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", path.c_str());
      return false;
    }

    // Each line of the file contains the position (x, y, z) & orientation
    // quaternion (x, y, z, w) separated by a space character
    std::string line;
    Pose prevPose = {};
    bool firstLine = true;

    // double sumX = 0, sumY = 0, sumZ = 0;
    // double sumQX = 0, sumQY = 0, sumQZ = 0, sumQW = 0;
    int count = 0;

    while (std::getline(file, line)) {
      std::istringstream iss(line);
      Pose pose;
      // Check if the line format is valid
      if (!(iss >> pose.x >> pose.y >> pose.z >> pose.qx >> pose.qy >>
            pose.qz >> pose.qw)) {
        RCLCPP_WARN(this->get_logger(), "Invalid line format: '%s'",
                    line.c_str());
        continue;
      }

      // If the same TF value is repeated, skip it to avoid bias
      //   if (!firstLine && isDuplicate(pose, prevPose)) {
      //     RCLCPP_WARN(this->get_logger(), "Repeating TF value: %f", pose.x);
      //     continue;
      //   }

      positions.emplace_back(pose.x, pose.y, pose.z);
      quaternions.emplace_back(pose.qw, pose.qx, pose.qy,
                               pose.qz); // Eigen uses (w, x, y, z)

      //   // Compute the sum of each component
      //   sumX += pose.x;
      //   sumY += pose.y;
      //   sumZ += pose.z;

      //   sumQX += pose.qx;
      //   sumQY += pose.qy;
      //   sumQZ += pose.qz;
      //   sumQW += pose.qw;

      prevPose = pose;
      firstLine = false;
      count++;
    }

    file.close();

    if (count == 0) {
      RCLCPP_ERROR(this->get_logger(), "No valid poses found.");
      return false;
    }

    // AVERAGE POSITION (simple arithmetic mean)
    Eigen::Vector3d pos_sum(0, 0, 0);
    for (const auto &p : positions)
      pos_sum += p;
    Eigen::Vector3d avg_pos = pos_sum / positions.size();

    // MARKLEY'S METHOD FOR QUATERNION AVERAGE
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (const auto &q : quaternions) {
      Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
      A += q_vec * q_vec.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(A);
    if (eigensolver.info() != Eigen::Success) {
      RCLCPP_ERROR(this->get_logger(), "Eigen decomposition failed.");
      return false;
    }

    Eigen::Vector4d avg_qvec =
        eigensolver.eigenvectors().col(3); // Eigenvectors sorted ascending
    Eigen::Quaterniond avg_q(avg_qvec(0), avg_qvec(1), avg_qvec(2),
                             avg_qvec(3)); // (w, x, y, z)
    avg_q.normalize();                     // Always normalize just in case

    RCLCPP_INFO(this->get_logger(), "Ended average TF computation.");

    // Set the averaged transform
    averaged_tf_.header.frame_id = base_frame;
    averaged_tf_.child_frame_id = camera_frame;
    averaged_tf_.transform.translation.x = avg_pos.x();
    averaged_tf_.transform.translation.y = avg_pos.y();
    averaged_tf_.transform.translation.z = avg_pos.z();
    averaged_tf_.transform.rotation.x = avg_q.x();
    averaged_tf_.transform.rotation.y = avg_q.y();
    averaged_tf_.transform.rotation.z = avg_q.z();
    averaged_tf_.transform.rotation.w = avg_q.w();

    return true;
  }

  void broadcastTransform() {

    if (!average_computed) {
      if (!readAndAveragePoses(file_path)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to read or process pose file.");
        return;
      }
      average_computed = true;
    }
    averaged_tf_.header.stamp = this->get_clock()->now();
    broadcaster_->sendTransform(averaged_tf_);

    RCLCPP_INFO_ONCE(this->get_logger(), "Broadcasting the final TF...");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FinalTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}