from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fixed_updown', default_value='0.18'),
        DeclareLaunchArgument('mock_planner', default_value='false'),
        DeclareLaunchArgument('wait_execution_done', default_value='true'),
        DeclareLaunchArgument('execution_timeout_ms', default_value='120000'),
        DeclareLaunchArgument('service_timeout_ms', default_value='30000'),
        DeclareLaunchArgument('ik_max_attempts', default_value='5'),
        Node(
            package='alfa_robot_benchmarks',
            executable='fixed_platform_task_orchestrator',
            name='fixed_platform_task_orchestrator',
            output='screen',
            parameters=[{
                'fixed_updown': LaunchConfiguration('fixed_updown'),
                'mock_planner': LaunchConfiguration('mock_planner'),
                'ik_service': '/alfa_dual_ik/solve',
                'planner_service': '/alfa_moveit/plan_joint_target',
                'trajectory_topic': '/plc_joint_trajectory',
                'plc_state_topic': '/plc_bridge_state',
                'wait_execution_done': LaunchConfiguration('wait_execution_done'),
                'execution_timeout_ms': LaunchConfiguration('execution_timeout_ms'),
                'service_timeout_ms': LaunchConfiguration('service_timeout_ms'),
                'ik_max_attempts': LaunchConfiguration('ik_max_attempts'),
            }],
        ),
    ])
