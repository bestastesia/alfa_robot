from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fixed_updown', default_value='0.18'),
        DeclareLaunchArgument('lock_updown', default_value='true'),
        DeclareLaunchArgument('workers', default_value='16'),
        DeclareLaunchArgument('h_candidate_count', default_value='16'),
        DeclareLaunchArgument('seed_count', default_value='32'),
        DeclareLaunchArgument('timeout', default_value='0.01'),
        DeclareLaunchArgument('check_collision', default_value='true'),
        DeclareLaunchArgument('fallback_enabled', default_value='false'),
        Node(
            package='alfa_robot_benchmarks',
            executable='fixed_platform_kdl_ik_service.py',
            name='fixed_platform_kdl_ik_service',
            output='screen',
            parameters=[{
                'fixed_updown': LaunchConfiguration('fixed_updown'),
                'lock_updown': LaunchConfiguration('lock_updown'),
                'workers': LaunchConfiguration('workers'),
                'h_candidate_count': LaunchConfiguration('h_candidate_count'),
                'seed_count': LaunchConfiguration('seed_count'),
                'timeout': LaunchConfiguration('timeout'),
                'check_collision': LaunchConfiguration('check_collision'),
                'fallback_enabled': LaunchConfiguration('fallback_enabled'),
                'h_mode': 'fixed_discrete',
                'h_search_margin': 0.2,
                'tool0_offset': 0.0,
                'try_target_orders': False,
                'use_reversed_target_order': True,
                'check_tip_error': True,
                'position_tolerance': 0.03,
                'top_suction_position_tolerance': 0.04,
                'orientation_tolerance': 0.05,
                'top_suction_orientation_tolerance': 0.0872664626,
                'enforce_arm_base_collisions': True,
                'reject_swapped_tips': True,
            }],
        ),
    ])
