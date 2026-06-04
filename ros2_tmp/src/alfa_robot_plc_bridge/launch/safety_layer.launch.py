"""PLC 急停/安全层启动入口。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('service_timeout_s', default_value='3.0'),
    ]
    return LaunchDescription(declared_arguments + [
        Node(
            package='alfa_robot_plc_bridge',
            executable='plc_safety_node',
            name='plc_safety_node',
            output='screen',
            parameters=[{
                'service_timeout_s': ParameterValue(LaunchConfiguration('service_timeout_s'), value_type=float),
            }],
        ),
    ])
