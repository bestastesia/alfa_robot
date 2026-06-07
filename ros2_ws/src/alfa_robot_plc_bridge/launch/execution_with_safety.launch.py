"""PLC 执行层 + 急停层双进程启动入口。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('alfa_robot_plc_bridge'))
    params = package_share / 'config' / 'plc_bridge.yaml'
    declared_arguments = [
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('plc_ip', default_value='192.168.1.88'),
        DeclareLaunchArgument('command_hz', default_value='20.0'),
        DeclareLaunchArgument('feedback_hz', default_value='20.0'),
        DeclareLaunchArgument('publish_joint_states', default_value='false'),
        DeclareLaunchArgument('velocity_limit_deg_s', default_value='5.0'),
        DeclareLaunchArgument('plc_execution_mode', default_value='stream'),
        DeclareLaunchArgument('acceleration_limit_deg_s2', default_value='10.0'),
        DeclareLaunchArgument('deceleration_limit_deg_s2', default_value='10.0'),
        DeclareLaunchArgument('emergency_deceleration_deg_s2', default_value='30.0'),
        DeclareLaunchArgument('service_timeout_s', default_value='3.0'),
    ]
    return LaunchDescription(declared_arguments + [
        Node(
            package='alfa_robot_plc_bridge',
            executable='plc_bridge_node',
            name='plc_bridge_node',
            output='screen',
            parameters=[
                str(params),
                {
                    'mock': ParameterValue(LaunchConfiguration('mock'), value_type=bool),
                    'plc_ip': LaunchConfiguration('plc_ip'),
                    'command_hz': ParameterValue(LaunchConfiguration('command_hz'), value_type=float),
                    'feedback_hz': ParameterValue(LaunchConfiguration('feedback_hz'), value_type=float),
                    'publish_joint_states': ParameterValue(LaunchConfiguration('publish_joint_states'), value_type=bool),
                    'velocity_limit_deg_s': ParameterValue(LaunchConfiguration('velocity_limit_deg_s'), value_type=float),
                    'plc_execution_mode': LaunchConfiguration('plc_execution_mode'),
                    'acceleration_limit_deg_s2': ParameterValue(LaunchConfiguration('acceleration_limit_deg_s2'), value_type=float),
                    'deceleration_limit_deg_s2': ParameterValue(LaunchConfiguration('deceleration_limit_deg_s2'), value_type=float),
                    'emergency_deceleration_deg_s2': ParameterValue(LaunchConfiguration('emergency_deceleration_deg_s2'), value_type=float),
                },
            ],
        ),
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
