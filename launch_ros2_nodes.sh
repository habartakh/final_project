#!/bin/bash

ros2 launch custom_moveit_pkg_new move_group.launch.py use_sim_time:=False&

sleep 5  # Wait for 5 seconds

ros2 launch moveit2_scripts add_objects_planning_scene_real.launch.py&

ros2 launch aruco_detection aruco_detector.launch.py&

ros2 launch camera_calibration camera_robot_tf_broadcaster.launch.py&

ros2 launch circle_pattern_pose_estimation circle_pose_estimation.launch.py&

ros2 launch camera_calibration final_tf_broadcaster.launch.py&

ros2 launch moveit2_services set_arm_joints_service_server.launch.py use_sim_time:=False&

ros2 launch moveit2_services go_to_pose_service_server.launch.py use_sim_time:=False




