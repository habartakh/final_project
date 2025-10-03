#!/bin/bash

ros2 launch camera_calibration camera_robot_tf_broadcaster.launch.py&

ros2 launch camera_calibration final_tf_broadcaster.launch.py&

ros2 launch moveit2_scripts aruco_visible_trajectory_real_cartesian.launch.py




