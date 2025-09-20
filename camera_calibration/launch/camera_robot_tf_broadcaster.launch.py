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

    aruco_visible_trajectory_launch_file_path =  os.path.join(
        get_package_share_directory('moveit2_scripts'),
        'launch',
        'aruco_visible_trajectory_real.launch.py'
    )

    camera_robot_tf_broadcaster_node = Node(
        name="camera_robot_tf_broadcaster",
        package="camera_calibration",
        executable="camera_robot_tf_broadcaster",
        output="screen", 
        parameters=[
                {'yaml_file_path': '/home/user/ros2_ws/src/final_project/camera_calibration/config/tf_messages_real.txt'},
                {'base_frame': 'base_link'},
                {'camera_frame': 'D415_link'},
                {'aruco_frame': 'aruco_link'},
            ],
    )


    # wait for 30 seconds to make sure the robot arm started following the aruco_visible trajectory
    delayed_tf_node = TimerAction(
        period=10.0,  
        actions=[
        camera_robot_tf_broadcaster_node,
        ])


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aruco_pose_estimation_file_path)
        ),
        #  IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(aruco_visible_trajectory_launch_file_path)
        # ),
     delayed_tf_node,
       
    ])