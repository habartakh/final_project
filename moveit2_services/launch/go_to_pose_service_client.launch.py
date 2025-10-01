import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='True if working in simulated environment or else it is false.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_config = MoveItConfigsBuilder("name", package_name="custom_moveit_pkg_new").to_moveit_configs()

    go_to_pose_service_client_node = Node(
        name="go_to_pose_service_client",
        package="moveit2_services",
        executable="go_to_pose_service_client",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],                   
    )


    return LaunchDescription([
        use_sim_time_arg,
        go_to_pose_service_client_node, 
    ])