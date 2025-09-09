import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    final_tf_broadcaster_node = Node(
        name="final_tf_broadcaster",
        package="camera_calibration",
        executable="final_tf_broadcaster",
        output="screen", 
        parameters=[
                {'file_path': '/home/user/ros2_ws/src/final_project/camera_calibration/config/tf_messages.txt'},
            ],
        
    )


    # wait for 30 seconds to make sure the robot arm started following the aruco_visible trajectory
    delayed_tf_node = TimerAction(
        period=30.0,  
        actions=[
        final_tf_broadcaster_node,
        ])


    return LaunchDescription([
        
     final_tf_broadcaster_node,
       
    ])