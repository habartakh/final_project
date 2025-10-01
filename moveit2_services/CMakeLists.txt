cmake_minimum_required(VERSION 3.8)
project(moveit2_services)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmJoints.srv"
  DEPENDENCIES std_msgs 
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

# For using the custom message defined in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Service Server to setup Set Joints values 
add_executable(set_arm_joints_service_server src/set_arm_joints_service_server.cpp)
target_include_directories(set_arm_joints_service_server PUBLIC include)
ament_target_dependencies(set_arm_joints_service_server
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  rclcpp_action 
  action_msgs
)
target_link_libraries(set_arm_joints_service_server "${cpp_typesupport_target}")

# Service Client to setup Set Joints values 
add_executable(set_arm_joints_service_client src/set_arm_joints_service_client.cpp)
target_include_directories(set_arm_joints_service_client PUBLIC include)
ament_target_dependencies(set_arm_joints_service_client
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  rclcpp_action 
  action_msgs
)
target_link_libraries(set_arm_joints_service_client "${cpp_typesupport_target}")

# Service Server to make end_effector go to goal pose 
add_executable(go_to_pose_service_server src/go_to_pose_service_server.cpp)
target_include_directories(go_to_pose_service_server PUBLIC include)
ament_target_dependencies(go_to_pose_service_server
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  rclcpp_action 
  action_msgs
)
target_link_libraries(go_to_pose_service_server "${cpp_typesupport_target}")

# Service Client to make end_effector go to goal pose 
add_executable(go_to_pose_service_client src/go_to_pose_service_client.cpp)
target_include_directories(go_to_pose_service_client PUBLIC include)
ament_target_dependencies(go_to_pose_service_client
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  rclcpp_action 
  action_msgs
)
target_link_libraries(go_to_pose_service_client "${cpp_typesupport_target}")


install(TARGETS
   set_arm_joints_service_server
   set_arm_joints_service_client
   go_to_pose_service_server
   go_to_pose_service_client
   DESTINATION lib/${PROJECT_NAME}
)

# install the config files
install(DIRECTORY
  srv
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
