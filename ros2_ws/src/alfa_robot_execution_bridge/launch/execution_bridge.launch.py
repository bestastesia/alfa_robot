from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('alfa_robot_execution_bridge'),
                'config',
                'execution_bridge.yaml',
            ]),
            description='YAML config for the ALFA execution bridge.',
        ),
        Node(
            package='alfa_robot_execution_bridge',
            executable='execution_bridge_node',
            name='alfa_execution_bridge',
            output='screen',
            parameters=[config_file],
        ),
    ])
