from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_share = Path(get_package_share_directory('alfa_robot_moveit_config'))
    plc_share = Path(get_package_share_directory('alfa_robot_plc_bridge'))
    bench_share = Path(get_package_share_directory('alfa_robot_benchmarks'))

    return LaunchDescription([
        DeclareLaunchArgument('plc_mock', default_value='true'),
        DeclareLaunchArgument('velocity_limit_deg_s', default_value='5.0'),
        DeclareLaunchArgument('plc_execution_mode', default_value='stream'),
        DeclareLaunchArgument('acceleration_limit_deg_s2', default_value='10.0'),
        DeclareLaunchArgument('deceleration_limit_deg_s2', default_value='10.0'),
        DeclareLaunchArgument('emergency_deceleration_deg_s2', default_value='30.0'),
        DeclareLaunchArgument('moveit_velocity_scale', default_value='0.25'),
        DeclareLaunchArgument('moveit_acceleration_scale', default_value='0.2'),
        DeclareLaunchArgument('planning_time', default_value='8.0'),
        DeclareLaunchArgument('planning_attempts', default_value='20'),
        DeclareLaunchArgument('fixed_updown', default_value='0.18'),
        DeclareLaunchArgument('ik_timeout', default_value='0.01'),
        DeclareLaunchArgument('ik_max_attempts', default_value='5'),
        DeclareLaunchArgument('demo_mode', default_value='ik'),
        DeclareLaunchArgument('enable_plc_supervisor', default_value='true'),
        DeclareLaunchArgument('enable_home_hold', default_value='false'),
        DeclareLaunchArgument('home_hold_period_s', default_value='1.0'),
        DeclareLaunchArgument('home_hold_duration_s', default_value='0.5'),
        DeclareLaunchArgument('home_hold_max_commands', default_value='0'),
        DeclareLaunchArgument('stop_home_hold_on_task', default_value='true'),
        DeclareLaunchArgument('enable_center_separation_plate', default_value='true'),
        DeclareLaunchArgument('center_plate_x_min', default_value='0.4'),
        DeclareLaunchArgument('center_plate_x_max', default_value='0.76'),
        DeclareLaunchArgument('center_plate_y_thickness', default_value='0.001'),
        DeclareLaunchArgument('center_plate_z_min', default_value='0.0'),
        DeclareLaunchArgument('center_plate_z_max', default_value='1.8'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_share / 'launch' / 'demo.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(plc_share / 'launch' / 'execution_with_safety.launch.py')),
            launch_arguments={
                'mock': LaunchConfiguration('plc_mock'),
                'velocity_limit_deg_s': LaunchConfiguration('velocity_limit_deg_s'),
                'plc_execution_mode': LaunchConfiguration('plc_execution_mode'),
                'acceleration_limit_deg_s2': LaunchConfiguration('acceleration_limit_deg_s2'),
                'deceleration_limit_deg_s2': LaunchConfiguration('deceleration_limit_deg_s2'),
                'emergency_deceleration_deg_s2': LaunchConfiguration('emergency_deceleration_deg_s2'),
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
                'enable_center_separation_plate': LaunchConfiguration('enable_center_separation_plate'),
                'center_plate_x_min': LaunchConfiguration('center_plate_x_min'),
                'center_plate_x_max': LaunchConfiguration('center_plate_x_max'),
                'center_plate_y_thickness': LaunchConfiguration('center_plate_y_thickness'),
                'center_plate_z_min': LaunchConfiguration('center_plate_z_min'),
                'center_plate_z_max': LaunchConfiguration('center_plate_z_max'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bench_share / 'launch' / 'fixed_platform_task_orchestrator.launch.py')),
            launch_arguments={
                'fixed_updown': LaunchConfiguration('fixed_updown'),
                'mock_planner': 'false',
                'wait_execution_done': 'true',
                'ik_max_attempts': LaunchConfiguration('ik_max_attempts'),
                'demo_mode': LaunchConfiguration('demo_mode'),
            }.items(),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('enable_plc_supervisor')),
            package='alfa_robot_benchmarks',
            executable='plc_acceptance_supervisor.py',
            name='plc_acceptance_supervisor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'enable_home_hold': ParameterValue(LaunchConfiguration('enable_home_hold'), value_type=bool),
                'home_period_s': ParameterValue(LaunchConfiguration('home_hold_period_s'), value_type=float),
                'home_duration_s': ParameterValue(LaunchConfiguration('home_hold_duration_s'), value_type=float),
                'max_home_commands': ParameterValue(LaunchConfiguration('home_hold_max_commands'), value_type=int),
                'stop_home_hold_on_task': ParameterValue(LaunchConfiguration('stop_home_hold_on_task'), value_type=bool),
                'fixed_updown': ParameterValue(LaunchConfiguration('fixed_updown'), value_type=float),
            }],
        ),
        Node(
            package='alfa_robot_benchmarks',
            executable='monitor_ik_planner_chain.py',
            name='ik_planner_chain_monitor',
            output='screen',
            emulate_tty=True,
        ),
    ])
