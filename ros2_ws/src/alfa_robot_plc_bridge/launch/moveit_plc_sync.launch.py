"""
MoveIt 规划 + PLC bridge 实机同步测试启动文件。

启动后可用：
  ros2 run alfa_robot_plc_bridge moveit_plan_to_plc --group left_v5_arm \
    --delta-deg left_v5_joint1:1 --execute
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('plc_ip', default_value='192.168.1.88'),
        DeclareLaunchArgument('command_hz', default_value='20.0'),
        DeclareLaunchArgument('feedback_hz', default_value='20.0'),
    ]

    moveit_launch_dir = PathJoinSubstitution([
        FindPackageShare('alfa_robot_moveit_config'),
        'launch',
    ])
    plc_params = PathJoinSubstitution([
        FindPackageShare('alfa_robot_plc_bridge'),
        'config',
        'plc_bridge.yaml',
    ])

    return LaunchDescription(declared_arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_launch_dir, 'rsp.launch.py'])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_launch_dir, 'static_virtual_joint_tfs.launch.py'])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_launch_dir, 'move_group.launch.py'])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_launch_dir, 'moveit_rviz.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
        Node(
            package='alfa_robot_plc_bridge',
            executable='plc_bridge_node',
            name='plc_bridge_node',
            output='screen',
            parameters=[
                plc_params,
                {
                    'mock': ParameterValue(LaunchConfiguration('mock'), value_type=bool),
                    'plc_ip': LaunchConfiguration('plc_ip'),
                    'command_hz': ParameterValue(LaunchConfiguration('command_hz'), value_type=float),
                    'feedback_hz': ParameterValue(LaunchConfiguration('feedback_hz'), value_type=float),
                },
            ],
        ),
    ])
