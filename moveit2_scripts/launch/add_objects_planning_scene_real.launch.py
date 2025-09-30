import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    move_rviz_file_path = os.path.join(
        get_package_share_directory('custom_moveit_pkg_new'),
        'launch',
        'moveit_rviz.launch.py'
    )

    add_objects_planning_scene_real_node = Node(
        name="add_objects_planning_scene_real",
        package="moveit2_scripts",
        executable="add_objects_planning_scene_real",
        output="screen", 
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_rviz_file_path)
        ),
        add_objects_planning_scene_real_node,
    ])