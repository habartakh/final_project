import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    move_group_file_path = os.path.join(
        get_package_share_directory('custom_moveit_config'),
        'launch',
        'move_group.launch.py'
    )

    move_rviz_file_path = os.path.join(
        get_package_share_directory('custom_moveit_config'),
        'launch',
        'moveit_rviz.launch.py'
    )

    add_objects_planning_scene_node = Node(
        name="add_objects_planning_scene",
        package="moveit2_scripts",
        executable="add_objects_planning_scene",
        output="screen", 
    )

    moveit_config = MoveItConfigsBuilder("name", package_name="custom_moveit_config").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )


    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(move_group_file_path)
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_rviz_file_path)
        ),
        add_objects_planning_scene_node,
        # moveit_cpp_node,
    ])