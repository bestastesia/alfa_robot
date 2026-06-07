"""
PLC 实机 GUI 滑块控制 + RViz 可视化。

使用方式:
  ros2 launch alfa_robot_plc_bridge slider_control_plc.launch.py

启动前若电脑路由不稳定，先运行:
  tools/alfa_robot_plc_driver/scripts/plc_net_setup.sh

链路:
  joint_state_publisher_gui -> /joint_states_gui
    -> gui_joint_state_to_plc_trajectory -> /plc_joint_trajectory
    -> plc_bridge_node -> PLC Modbus TCP

说明:
  只下发 config/plc_bridge.yaml 里映射的 12 个机械臂轴；updown/底盘/夹爪等未映射关节暂时忽略。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
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
        DeclareLaunchArgument('gui_command_duration_s', default_value='0.1'),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('alfa_robot_description'),
            'urdf',
            'alfa_robot.urdf.xacro',
        ]),
        ' use_mock_hardware:=true',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('alfa_robot_description'),
        'rviz',
        'alfa_robot.rviz',
    ])

    plc_params = PathJoinSubstitution([
        FindPackageShare('alfa_robot_plc_bridge'),
        'config',
        'plc_bridge.yaml',
    ])

    return LaunchDescription(declared_arguments + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            remappings=[('joint_states', 'joint_states_gui')],
        ),
        Node(
            package='alfa_robot_plc_bridge',
            executable='gui_joint_state_to_plc_trajectory',
            name='gui_joint_state_to_plc_trajectory',
            output='screen',
            parameters=[{
                'command_duration_s': ParameterValue(LaunchConfiguration('gui_command_duration_s'), value_type=float),
                'min_publish_period_s': 0.05,
                'min_position_delta_rad': 0.0005,
            }],
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
