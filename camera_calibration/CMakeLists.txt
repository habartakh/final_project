cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(aruco_interfaces REQUIRED)

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

# Add the TF broadcaster of the ArUco and Camera frames
add_executable(aruco_camera_tf_broadcaster src/aruco_camera_tf_broadcaster.cpp)
target_include_directories(aruco_camera_tf_broadcaster PUBLIC include)
ament_target_dependencies(
    aruco_camera_tf_broadcaster
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    aruco_interfaces
)

# Add the TF broadcaster of the ArUco and Camera frames
add_executable(camera_robot_tf_broadcaster src/camera_robot_tf_broadcaster.cpp)
target_include_directories(camera_robot_tf_broadcaster PUBLIC include)
ament_target_dependencies(
    camera_robot_tf_broadcaster
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
	tf2_geometry_msgs
    aruco_interfaces
)

# Add the final TF broadcaster of the base_link and Camera frames
add_executable(final_tf_broadcaster src/final_tf_broadcaster.cpp)
target_include_directories(final_tf_broadcaster PUBLIC include)
ament_target_dependencies(
    final_tf_broadcaster
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
	tf2_geometry_msgs
)

install(TARGETS
    aruco_camera_tf_broadcaster
	camera_robot_tf_broadcaster
	final_tf_broadcaster
    DESTINATION lib/${PROJECT_NAME}
)

# install the launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
