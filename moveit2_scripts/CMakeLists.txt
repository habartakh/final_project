cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SetJointValues.action"
  DEPENDENCIES builtin_interfaces
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(add_objects_planning_scene src/add_objects_planning_scene.cpp)
target_include_directories(add_objects_planning_scene PUBLIC include)
ament_target_dependencies(add_objects_planning_scene
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)
# Add the linker flag otherwise yaml library wont work
target_link_libraries(add_objects_planning_scene ${YAML_CPP_LIBRARIES})

# Pick and place pipeline executable
add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC include)
ament_target_dependencies(pick_and_place
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)


# Make the Aruco visible executable
add_executable(make_aruco_visible src/make_aruco_visible.cpp)
target_include_directories(make_aruco_visible PUBLIC include)
ament_target_dependencies(make_aruco_visible
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)

# Set Joint values action server executable 
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(set_arm_joints_action_server src/set_arm_joints_action_server.cpp)
target_include_directories(set_arm_joints_action_server PUBLIC include)
ament_target_dependencies(set_arm_joints_action_server
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)
target_link_libraries(set_arm_joints_action_server "${cpp_typesupport_target}")

# Set Joint values action client executable 

add_executable(set_arm_joints_action_client src/set_arm_joints_action_client.cpp)
target_include_directories(set_arm_joints_action_client PUBLIC include)
ament_target_dependencies(set_arm_joints_action_client
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)
target_link_libraries(set_arm_joints_action_client "${cpp_typesupport_target}")


install(TARGETS
   add_objects_planning_scene
   pick_and_place
   make_aruco_visible
   set_arm_joints_action_server
   set_arm_joints_action_client
   DESTINATION lib/${PROJECT_NAME}
)

# install the config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
