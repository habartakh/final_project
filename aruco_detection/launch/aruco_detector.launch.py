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

    aruco_pose_estimation_file_path = os.path.join(
        get_package_share_directory('aruco_pose_estimation'),
        'launch',
        'aruco_pose_estimation.launch.py'
    )

    rviz_file = os.path.join(
        get_package_share_directory('aruco_detection'),
        'rviz',
        'aruco.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    aruco_detector_node = Node(
        name="aruco_detector",
        package="aruco_detection",
        executable="aruco_detector.py",
        output="screen", 
        parameters=[
                {'camera_info_topic': '/D415/color/camera_info'},
                {'image_topic': '/D415/color/image_raw'},
                {'camera_frame': 'D415_link'},
                
            ],
    )


    return LaunchDescription([
      rviz2_node,
      aruco_detector_node, 
       
    ])