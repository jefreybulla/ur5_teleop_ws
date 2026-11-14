# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type = LaunchConfiguration('ur_type')
    rviz_config = LaunchConfiguration('rviz_config_file')
    launch_servo = LaunchConfiguration('launch_servo')

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                PathJoinSubstitution(
                    [FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']
                ),
                ' ur_type:=',
                ur_type,
                ' name:=ur',
                ' safety_limits:=true',
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ur_type',
                default_value='ur5',
                description='UR robot variant passed to ur_description.',
            ),
            DeclareLaunchArgument(
                'rviz_config_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('ur_description'),
                        'rviz',
                        'view_robot.rviz',
                    ]
                ),
                description='RViz configuration (includes Path/Marker displays if added).',
            ),
            DeclareLaunchArgument(
                'launch_servo',
                default_value='false',
                description=(
                    'If true, start resolved_rates_demo in this launch. '
                    'If false, run ros2 run ur5_kinematics resolved_rates_demo '
                    'in a second terminal.'
                ),
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='ur5_kinematics',
                executable='joint_state_idle',
                output='screen',
                condition=UnlessCondition(launch_servo),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            ),
            Node(
                package='ur5_kinematics',
                executable='resolved_rates_demo',
                output='screen',
                condition=IfCondition(launch_servo),
            ),
        ]
    )