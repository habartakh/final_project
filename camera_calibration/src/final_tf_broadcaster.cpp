#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

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

    std::string file_path =
        this->declare_parameter<std::string>("file_path", "data.txt");

    if (!readAndAveragePoses(file_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read or process pose file.");
      return;
    }

    // Broadcast the computed base_link -- camera TF periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FinalTfBroadcaster::broadcastTransform, this));
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped averaged_tf_;
  rclcpp::TimerBase::SharedPtr timer_;

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

    double sumX = 0, sumY = 0, sumZ = 0;
    double sumQX = 0, sumQY = 0, sumQZ = 0, sumQW = 0;
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
      if (!firstLine && isDuplicate(pose, prevPose)) {
        RCLCPP_WARN(this->get_logger(), "Repeating TF value: %f", pose.x);
        continue;
      }

      // Compute the sum of each component
      sumX += pose.x;
      sumY += pose.y;
      sumZ += pose.z;

      sumQX += pose.qx;
      sumQY += pose.qy;
      sumQZ += pose.qz;
      sumQW += pose.qw;

      prevPose = pose;
      firstLine = false;
      count++;
    }

    file.close();

    if (count == 0) {
      RCLCPP_ERROR(this->get_logger(), "No valid poses found.");
      return false;
    }

    // Normalize the quaternion
    double avgQX = sumQX / count;
    double avgQY = sumQY / count;
    double avgQZ = sumQZ / count;
    double avgQW = sumQW / count;

    double norm = std::sqrt(avgQX * avgQX + avgQY * avgQY + avgQZ * avgQZ +
                            avgQW * avgQW);
    if (norm == 0) {
      RCLCPP_ERROR(this->get_logger(), "Quaternion normalization failed.");
      return false;
    }

    avgQX /= norm;
    avgQY /= norm;
    avgQZ /= norm;
    avgQW /= norm;

    // Compute the averaged transform
    averaged_tf_.header.frame_id = "base_link";
    averaged_tf_.child_frame_id = "wrist_rgbd_camera_link";
    averaged_tf_.transform.translation.x = sumX / count;
    averaged_tf_.transform.translation.y = sumY / count;
    averaged_tf_.transform.translation.z = sumZ / count;
    averaged_tf_.transform.rotation.x = avgQX;
    averaged_tf_.transform.rotation.y = avgQY;
    averaged_tf_.transform.rotation.z = avgQZ;
    averaged_tf_.transform.rotation.w = avgQW;

    return true;
  }

  void broadcastTransform() {
    averaged_tf_.header.stamp = this->get_clock()->now();
    broadcaster_->sendTransform(averaged_tf_);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FinalTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}