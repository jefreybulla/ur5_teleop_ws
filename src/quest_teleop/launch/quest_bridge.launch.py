"""Quest WebXR page + ROS bridge only (controller pose topics, no robot)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('certfile', default_value='',
                              description='PEM cert; empty = plain HTTP (localhost / adb reverse only)'),
        DeclareLaunchArgument('keyfile', default_value=''),
    ]
    bridge = Node(
        package='quest_teleop', executable='quest_bridge', output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'certfile': ParameterValue(LaunchConfiguration('certfile'), value_type=str),
            'keyfile': ParameterValue(LaunchConfiguration('keyfile'), value_type=str),
        }],
    )
    return LaunchDescription(args + [bridge])
