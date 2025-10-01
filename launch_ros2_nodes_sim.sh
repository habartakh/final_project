#!/bin/bash

ros2 launch custom_moveit_config move_group.launch.py&

sleep 5  # Wait for 5 seconds

ros2 launch moveit2_scripts add_objects_planning_scene.launch.py&

ros2 launch aruco_detection aruco_detector.launch.py \
        image_topic:=/wrist_rgbd_depth_sensor/image_raw \
        camera_info_topic:=/wrist_rgbd_depth_sensor/camera_info \
        camera_frame:=wrist_rgbd_camera_link&

ros2 launch camera_calibration camera_robot_tf_broadcaster.launch.py \
        yaml_file_path:=/home/user/ros2_ws/src/final_project/camera_calibration/config/tf_messages_sim.txt \
        base_frame:=base_link \
        camera_frame:=wrist_rgbd_camera_link \
        aruco_frame:=rg2_gripper_aruco_link&

ros2 launch circle_pattern_pose_estimation circle_pose_estimation.launch.py \
        image_topic:=/wrist_rgbd_depth_sensor/image_raw \
        depth_image_topic:=/wrist_rgbd_depth_sensor/depth/image_raw \
        camera_info_topic:=/wrist_rgbd_depth_sensor/camera_info&

ros2 launch camera_calibration final_tf_broadcaster.launch.py \
        yaml_file_path:=/home/user/ros2_ws/src/final_project/camera_calibration/config/tf_messages_sim.txt \
        base_frame:=base_link \
        camera_frame:=wrist_rgbd_camera_link&

ros2 launch moveit2_services set_arm_joints_service_server_sim.launch.py&

ros2 launch moveit2_services go_to_pose_service_server_sim.launch.py




