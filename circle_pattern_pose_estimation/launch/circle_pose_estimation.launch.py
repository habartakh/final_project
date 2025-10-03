import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution

def generate_launch_description():

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/D415/color/image_raw',
        description='Base frame of the robot'
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/D415/depth/image_rect_raw',
        description='Base frame of the robot'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/D415/color/camera_info',
        description='ARUCO marker frame'
    )

    # LaunchConfigurations to use in Node parameters
    image_topic = LaunchConfiguration('image_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    

    aruco_pose_estimation_file_path = os.path.join(
        get_package_share_directory('aruco_pose_estimation'),
        'launch',
        'aruco_pose_estimation.launch.py'
    )

    rviz_file = os.path.join(
        get_package_share_directory('circle_pattern_pose_estimation'),
        'rviz',
        'circle.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    circle_pose_estimation_node = Node(
        name="circle_pose_estimation",
        package="circle_pattern_pose_estimation",
        executable="circle_pose_estimation.py",
        output="screen", 
        parameters=[
                {'camera_info_topic': camera_info_topic},
                {'image_topic': image_topic},
                {'depth_image_topic': depth_image_topic},
            ],
    )


    return LaunchDescription([
        # rviz2_node,
        image_topic_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,


        circle_pose_estimation_node, 
       
    ])