import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

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
        name="set_arm_joints_action_server",
        package="moveit2_scripts",
        executable="set_arm_joints_action_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )


    # wait for 10 seconds to make sure the collision objects were added to the planning scene
    # Then start planning and executing the trajectory 
    delayed_moveit_node = TimerAction(
        period=10.0,  
        actions=[
        moveit_cpp_node,
        ])


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_rviz_file_path)
        ),
        add_objects_planning_scene_node,
        # moveit_cpp_node,
        delayed_moveit_node,
    ])