from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare a launch argument named 'my_argument' with a default value
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Either use simulation time or not'
    )
    use_sim_time_value = LaunchConfiguration('use_sim_time')

    moveit_config = MoveItConfigsBuilder("name", package_name="custom_moveit_config").to_moveit_configs()
   
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": use_sim_time_value},
        ],
    )
    return LaunchDescription([
        use_sim_time_arg,
        move_group_node,
        ])