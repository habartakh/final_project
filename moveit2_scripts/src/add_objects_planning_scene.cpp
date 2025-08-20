#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class PlanningSceneExample : public rclcpp::Node {
public:
  PlanningSceneExample() : Node("planning_scene_example") {
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    addCollisionObjects(*planning_scene_interface_);
  }

private:
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;

  void
  addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &psi) {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects(2);

    collision_objects[0].id = "wall";
    collision_objects[0].header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {2.0, 0.03, 2.0};
    // IMPORTANT: The poses are RELATIVE to the position of the robot arm
    // Since regardless of its coordinates in the Gazebo world, the robot arm is
    // always paced at coordinates (0,0,0) in the planning scene
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.3;
    pose.position.y = -0.56;
    pose.position.z = -0.03;
    pose.orientation.w = 1.0;

    collision_objects[0].primitives.push_back(box);
    collision_objects[0].primitive_poses.push_back(pose);
    collision_objects[0].operation = collision_objects[0].ADD;

    /***************************** Table **************************************/
    collision_objects[1].id = "table";
    collision_objects[1].header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive box1;
    box1.type = shape_msgs::msg::SolidPrimitive::BOX;
    box1.dimensions = {0.85, 1.81, 0.05};
    // IMPORTANT: The poses are RELATIVE to the position of the robot arm
    // Since regardless of its coordinates in the Gazebo world, the robot arm is
    // always paced at coordinates (0,0,0) in the planning scene
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.3;
    pose1.position.y = 0.36;
    pose1.position.z = -0.03;
    pose1.orientation.w = 1.0;

    collision_objects[1].primitives.push_back(box1);
    collision_objects[1].primitive_poses.push_back(pose1);
    collision_objects[1].operation = collision_objects[1].ADD;

    psi.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(),
                "Added collision object to planning scene.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneExample>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
