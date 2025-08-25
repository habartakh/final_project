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

    aruco_pose_estimation_file_path = os.path.join(
        get_package_share_directory('aruco_pose_estimation'),
        'launch',
        'aruco_pose_estimation.launch.py'
    )

    aruco_camera_tf_broadcaster_node = Node(
        name="aruco_camera_tf_broadcaster",
        package="camera_calibration",
        executable="aruco_camera_tf_broadcaster",
        output="screen", 
    )


    # wait for 10 seconds to make sure the collision objects were added to the planning scene
    # Then start planning and executing the trajectory 
    delayed_tf_node = TimerAction(
        period=5.0,  
        actions=[
        aruco_camera_tf_broadcaster_node,
        ])


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aruco_pose_estimation_file_path)
        ),
        delayed_tf_node,
       
    ])