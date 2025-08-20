#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PlanningSceneExample : public rclcpp::Node
{
public:
    PlanningSceneExample() : Node("planning_scene_example")
    {
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        addCollisionObjects(*planning_scene_interface_);
    }

private:
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& psi)
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects(1);

        collision_objects[0].id = "table";
        collision_objects[0].header.frame_id = "world";

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {0.5, 1.0, 0.8};

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = 0.0;
        pose.position.z = 0.4;
        pose.orientation.w = 1.0;

        collision_objects[0].primitives.push_back(box);
        collision_objects[0].primitive_poses.push_back(pose);
        collision_objects[0].operation = collision_objects[0].ADD;

        psi.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(this->get_logger(), "Added collision object to planning scene.");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanningSceneExample>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
