#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem> // Include the filesystem library
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

class PlanningSceneExample : public rclcpp::Node {
public:
  PlanningSceneExample() : Node("planning_scene_example") {
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Add the collision objects declared in YAML file to the planning scene
    addCollisionObjectsFromYAML(*planning_scene_interface_,
                                "collision_objects.yaml");
  }

private:
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;

  // Extract the objects info from the YAML file and add them to the
  // planning scene
  void addCollisionObjectsFromYAML(
      moveit::planning_interface::PlanningSceneInterface &psi,
      const std::string &yaml_file_name) {

    // Get the package directory at runtime
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory(
            "moveit2_scripts"); // Replace with your package name

    std::string yaml_file_path =
        package_share_directory + "/config/" + yaml_file_name;

    // Load the YAML File
    YAML::Node config = YAML::LoadFile(yaml_file_path);
    const auto &objects = config["collision_objects"];

    // Container of all the collision objects to be added to the scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    for (const auto &obj : objects) {
      moveit_msgs::msg::CollisionObject co;
      co.id = obj["id"].as<std::string>();
      co.header.frame_id = "world";

      const std::string type = obj["type"].as<std::string>();
      const auto position = obj["position"].as<std::vector<double>>();
      const auto orientation = obj["orientation"].as<std::vector<double>>();

      // IMPORTANT: The poses are RELATIVE to the position of the robot arm
      // Since regardless of its coordinates in the Gazebo world, the robot arm
      // is always paced at coordinates (0,0,0) in the planning scene
      geometry_msgs::msg::Pose pose;
      pose.position.x = position[0];
      pose.position.y = position[1];
      pose.position.z = position[2];
      pose.orientation.x = orientation[0];
      pose.orientation.y = orientation[1];
      pose.orientation.z = orientation[2];
      pose.orientation.w = orientation[3];

      // The object is rendered differently depending on its geometry
      // The wall and table are simple boxes while the other objects are loaded
      // by their meshes
      if (type == "box") {
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        std::vector<double> dims = obj["dimensions"].as<std::vector<double>>();
        box.dimensions.assign(dims.begin(), dims.end());
        co.primitives.push_back(box);
        co.primitive_poses.push_back(pose);
      }

      else if (type == "mesh") {
        const std::string mesh_path = obj["mesh_path"].as<std::string>();
        shapes::Mesh *m = shapes::createMeshFromResource(mesh_path);
        if (!m) {
          RCLCPP_WARN(rclcpp::get_logger("add_objects"),
                      "Failed to load mesh: %s", mesh_path.c_str());
          continue;
        }

        shapes::ShapeMsg mesh_msg_variant;
        shapes::constructMsgFromShape(m, mesh_msg_variant);
        shape_msgs::msg::Mesh mesh_msg =
            boost::get<shape_msgs::msg::Mesh>(mesh_msg_variant);

        co.meshes.push_back(mesh_msg);
        co.mesh_poses.push_back(pose);
        delete m;
      }

      co.operation = co.ADD;
      collision_objects.push_back(co);
    }

    psi.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(rclcpp::get_logger("add_objects"),
                "Added %zu collision objects from YAML",
                collision_objects.size());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneExample>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
