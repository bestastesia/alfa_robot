from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = Path(get_package_share_directory('alfa_robot_moveit_config'))
    plc_share = Path(get_package_share_directory('alfa_robot_plc_bridge'))
    bench_share = Path(get_package_share_directory('alfa_robot_benchmarks'))

    return LaunchDescription([
        DeclareLaunchArgument('plc_mock', default_value='true'),
        DeclareLaunchArgument('velocity_limit_deg_s', default_value='5.0'),
        DeclareLaunchArgument('plc_execution_mode', default_value='stream'),
        DeclareLaunchArgument('moveit_velocity_scale', default_value='0.25'),
        DeclareLaunchArgument('moveit_acceleration_scale', default_value='0.2'),
        DeclareLaunchArgument('planning_time', default_value='8.0'),
        DeclareLaunchArgument('planning_attempts', default_value='20'),
        DeclareLaunchArgument('fixed_updown', default_value='0.18'),
        DeclareLaunchArgument('ik_timeout', default_value='0.01'),
        DeclareLaunchArgument('ik_max_attempts', default_value='5'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_share / 'launch' / 'demo.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(plc_share / 'launch' / 'execution_with_safety.launch.py')),
            launch_arguments={
                'mock': LaunchConfiguration('plc_mock'),
                'velocity_limit_deg_s': LaunchConfiguration('velocity_limit_deg_s'),
                'plc_execution_mode': LaunchConfiguration('plc_execution_mode'),
                'publish_joint_states': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bench_share / 'launch' / 'fixed_platform_dual_ik_service.launch.py')),
            launch_arguments={
                'fixed_updown': LaunchConfiguration('fixed_updown'),
                'timeout': LaunchConfiguration('ik_timeout'),
                'check_collision': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bench_share / 'launch' / 'temporary_moveit_joint_planner.launch.py')),
            launch_arguments={
                'planning_time': LaunchConfiguration('planning_time'),
                'planning_attempts': LaunchConfiguration('planning_attempts'),
                'moveit_velocity_scale': LaunchConfiguration('moveit_velocity_scale'),
                'moveit_acceleration_scale': LaunchConfiguration('moveit_acceleration_scale'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bench_share / 'launch' / 'fixed_platform_task_orchestrator.launch.py')),
            launch_arguments={
                'fixed_updown': LaunchConfiguration('fixed_updown'),
                'mock_planner': 'false',
                'wait_execution_done': 'true',
                'ik_max_attempts': LaunchConfiguration('ik_max_attempts'),
            }.items(),
        ),
        Node(
            package='alfa_robot_benchmarks',
            executable='monitor_ik_planner_chain.py',
            name='ik_planner_chain_monitor',
            output='screen',
            emulate_tty=True,
        ),
    ])
