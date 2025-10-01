import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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

    aruco_frame_arg = DeclareLaunchArgument(
        'aruco_frame',
        default_value='aruco_link',
        description='ARUCO marker frame'
    )

    # LaunchConfigurations to use in Node parameters
    yaml_file_path = LaunchConfiguration('yaml_file_path')
    base_frame = LaunchConfiguration('base_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    aruco_frame = LaunchConfiguration('aruco_frame')

    # Define the node with dynamic parameters
    camera_robot_tf_broadcaster_node = Node(
        name="camera_robot_tf_broadcaster",
        package="camera_calibration",
        executable="camera_robot_tf_broadcaster",
        output="screen",
        parameters=[{
            'yaml_file_path': yaml_file_path,
            'base_frame': base_frame,
            'camera_frame': camera_frame,
            'aruco_frame': aruco_frame
        }]
    )

    # Optional: Delay the TF node
    delayed_tf_node = TimerAction(
        period=10.0,
        actions=[camera_robot_tf_broadcaster_node],
    )

    return LaunchDescription([
        yaml_file_path_arg,
        base_frame_arg,
        camera_frame_arg,
        aruco_frame_arg,
        # delayed_tf_node,  
        camera_robot_tf_broadcaster_node,
    ])
