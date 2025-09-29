import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution

def generate_launch_description():

    start_joint_trajectory_action_server_node = Node(
        name="start_joints_trajectory_action_server",
        package="moveit2_actions",
        executable="start_joints_trajectory_action_server",
        output="screen",            
           
    )


    start_joint_trajectory_action_client_node = Node(
        name="start_joints_trajectory_action_client",
        package="moveit2_actions",
        executable="start_joints_trajectory_action_client",
        output="screen",            
           
    )


    return LaunchDescription([
      start_joint_trajectory_action_server_node, 
    #   start_joint_trajectory_action_client_node
       
    ])