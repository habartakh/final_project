cmake_minimum_required(VERSION 3.8)
project(moveit2_actions)

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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointValues.msg"
  "action/SetTrajectoryJointValues.action"
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

# Set Trajectory Joint values action server executable 
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(start_joints_trajectory_action_server src/start_joints_trajectory_action_server.cpp)
target_include_directories(start_joints_trajectory_action_server PUBLIC include)
ament_target_dependencies(start_joints_trajectory_action_server
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  # yaml-cpp
)
target_link_libraries(start_joints_trajectory_action_server "${cpp_typesupport_target}")

# Set Trajectory Joint values action client executable 
add_executable(start_joints_trajectory_action_client src/start_joints_trajectory_action_client.cpp)
target_include_directories(start_joints_trajectory_action_client PUBLIC include)
ament_target_dependencies(start_joints_trajectory_action_client
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  rclcpp_action 
  # yaml-cpp
)
target_link_libraries(start_joints_trajectory_action_client "${cpp_typesupport_target}")

install(TARGETS
   start_joints_trajectory_action_server
   start_joints_trajectory_action_client
   DESTINATION lib/${PROJECT_NAME}
)

# install the config files
install(DIRECTORY
#   config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
