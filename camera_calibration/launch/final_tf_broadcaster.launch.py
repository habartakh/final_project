import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch arguments
    yaml_file_path_arg = DeclareLaunchArgument(
        'yaml_file_path',
        default_value='/home/user/ros2_ws/src/final_project/camera_calibration/config/tf_messages_real.txt',
        description='Path to the YAML configuration file'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame of the robot'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='D415_link',
        description='Camera frame'
    )

    # LaunchConfigurations to use in Node parameters
    yaml_file_path = LaunchConfiguration('yaml_file_path')
    base_frame = LaunchConfiguration('base_frame')
    camera_frame = LaunchConfiguration('camera_frame')

    final_tf_broadcaster_node = Node(
        name="final_tf_broadcaster",
        package="camera_calibration",
        executable="final_tf_broadcaster",
        output="screen", 
        parameters=[
                {'file_path': yaml_file_path},
                {'base_frame': base_frame},
                {'camera_frame': camera_frame},
            ],
        
    )


    # wait for 30 seconds to make sure the robot arm started following the aruco_visible trajectory
    delayed_tf_node = TimerAction(
        period=10.0,  
        actions=[
        final_tf_broadcaster_node,
        ])


    return LaunchDescription([
        yaml_file_path_arg,
        base_frame_arg,
        camera_frame_arg,
        final_tf_broadcaster_node,
       
    ])