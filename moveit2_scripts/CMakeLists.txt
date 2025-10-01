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
find_package(std_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


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

#Node that adds the real robot collision objects to the planning scene 
add_executable(add_objects_planning_scene_real src/add_objects_planning_scene_real.cpp)
target_include_directories(add_objects_planning_scene_real PUBLIC include)
ament_target_dependencies(add_objects_planning_scene_real
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)
# Add the linker flag otherwise yaml library wont work
target_link_libraries(add_objects_planning_scene_real ${YAML_CPP_LIBRARIES})

# Pick and place pipeline executable
add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC include)
ament_target_dependencies(pick_and_place
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)


# Make the Aruco visible in simulation executable
add_executable(make_aruco_visible src/make_aruco_visible.cpp)
target_include_directories(make_aruco_visible PUBLIC include)
ament_target_dependencies(make_aruco_visible
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)


# The trajectory pipeline to move the ArUco marker in places visible by the camera in the simulation
add_executable(aruco_visible_trajectory src/aruco_visible_trajectory.cpp)
target_include_directories(aruco_visible_trajectory PUBLIC include)
ament_target_dependencies(aruco_visible_trajectory
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)


# Trajectory maintaining the ArUco marker visible by the camera in the real world
add_executable(aruco_visible_trajectory_real src/aruco_visible_trajectory_real.cpp)
target_include_directories(aruco_visible_trajectory_real PUBLIC include)
ament_target_dependencies(aruco_visible_trajectory_real
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  std_msgs
  yaml-cpp
)

# Add the linker flag otherwise yaml library wont work
target_link_libraries(aruco_visible_trajectory_real ${YAML_CPP_LIBRARIES})

# The trajectory pipeline to move the ArUco marker in places visible by the camera in the simulation
add_executable(aruco_visible_trajectory_real_cartesian src/aruco_visible_trajectory_real_cartesian.cpp)
target_include_directories(aruco_visible_trajectory_real_cartesian PUBLIC include)
ament_target_dependencies(aruco_visible_trajectory_real_cartesian
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  yaml-cpp
)


# Executable to test if the TF is okay or not
add_executable(test_camera_calibration src/test_camera_calibration.cpp)
target_include_directories(test_camera_calibration PUBLIC include)
ament_target_dependencies(test_camera_calibration
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
)


install(TARGETS
   add_objects_planning_scene
   add_objects_planning_scene_real
   pick_and_place
   make_aruco_visible
   aruco_visible_trajectory
   aruco_visible_trajectory_real
   aruco_visible_trajectory_real_cartesian
   test_camera_calibration
   DESTINATION lib/${PROJECT_NAME}
)

# install the config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
