"""UR5 in RViz driven by the Quest controller (or fake_quest) via resolved rates."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
        DeclareLaunchArgument('hand', default_value='right'),
        DeclareLaunchArgument('scale', default_value='1.0'),
        DeclareLaunchArgument('operator_yaw_deg', default_value='0.0'),
        # Speed limits (defaults match quest_teleop). Raise them only in simulation first.
        DeclareLaunchArgument('v_max', default_value='0.25',
                              description='Max tool linear speed [m/s]'),
        DeclareLaunchArgument('w_max', default_value='1.0',
                              description='Max tool angular speed [rad/s]'),
        DeclareLaunchArgument('max_joint_speed', default_value='1.0',
                              description='Per-joint speed cap [rad/s]'),
    ]
    # Robot model + RViz + idle joint publisher from ur5_kinematics.
    robot_viz = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ur5_kinematics'), 'launch',
                              'resolved_rates_demo.launch.py']),
        launch_arguments={'launch_servo': 'false'}.items(),
    )
    bridge = Node(
        package='quest_teleop', executable='quest_bridge', output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'certfile': ParameterValue(LaunchConfiguration('certfile'), value_type=str),
            'keyfile': ParameterValue(LaunchConfiguration('keyfile'), value_type=str),
        }],
    )
    teleop = Node(
        package='quest_teleop', executable='quest_teleop', output='screen',
        parameters=[{
            'mode': 'sim',
            'hand': ParameterValue(LaunchConfiguration('hand'), value_type=str),
            'scale': ParameterValue(LaunchConfiguration('scale'), value_type=float),
            'operator_yaw_deg': ParameterValue(LaunchConfiguration('operator_yaw_deg'), value_type=float),
            'v_max': ParameterValue(LaunchConfiguration('v_max'), value_type=float),
            'w_max': ParameterValue(LaunchConfiguration('w_max'), value_type=float),
            'max_joint_speed': ParameterValue(LaunchConfiguration('max_joint_speed'),
                                              value_type=float),
        }],
    )
    return LaunchDescription(args + [robot_viz, bridge, teleop])
