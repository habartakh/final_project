import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():


    moveit_config = MoveItConfigsBuilder("name", package_name="custom_moveit_config").to_moveit_configs()

    set_arm_joints_service_server_node = Node(
        name="set_arm_joints_service_server",
        package="moveit2_services",
        executable="set_arm_joints_service_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],                   
    )


    set_arm_joints_service_client_node = Node(
        name="set_arm_joints_service_client",
        package="moveit2_services",
        executable="set_arm_joints_service_client",
        output="screen", 
         parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],                      
    )

    delayed_client_node = TimerAction(
        period=10.0,  
        actions=[
        set_arm_joints_service_client_node,
        ])


    return LaunchDescription([
      set_arm_joints_service_server_node, 
    #   set_arm_joints_service_client_node,
      delayed_client_node,
    ])