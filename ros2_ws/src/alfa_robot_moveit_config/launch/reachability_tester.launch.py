from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('mode', default_value='auto'),
        DeclareLaunchArgument('side', default_value='left'),
        DeclareLaunchArgument('reference_frame', default_value='world'),
        DeclareLaunchArgument('output_csv', default_value='reachability_results.csv'),
        DeclareLaunchArgument('min_x', default_value='0.0'),
        DeclareLaunchArgument('max_x', default_value='0.8'),
        DeclareLaunchArgument('min_y', default_value='0.0'),
        DeclareLaunchArgument('max_y', default_value='0.6'),
        DeclareLaunchArgument('min_z', default_value='0.1'),
        DeclareLaunchArgument('max_z', default_value='0.8'),
        DeclareLaunchArgument('step', default_value='0.1'),
        DeclareLaunchArgument('manual_period', default_value='0.2'),
        DeclareLaunchArgument('manual_duration', default_value='0.0'),
        DeclareLaunchArgument('service_timeout', default_value='10.0'),
        DeclareLaunchArgument('ik_timeout', default_value='0.2'),
        DeclareLaunchArgument('avoid_collisions', default_value='true'),
    ]

    reachability_tester = Node(
        package='alfa_robot_moveit_config',
        executable='reachability_tester.py',
        name='reachability_tester',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'side': LaunchConfiguration('side'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'output_csv': LaunchConfiguration('output_csv'),
            'min_x': LaunchConfiguration('min_x'),
            'max_x': LaunchConfiguration('max_x'),
            'min_y': LaunchConfiguration('min_y'),
            'max_y': LaunchConfiguration('max_y'),
            'min_z': LaunchConfiguration('min_z'),
            'max_z': LaunchConfiguration('max_z'),
            'step': LaunchConfiguration('step'),
            'manual_period': LaunchConfiguration('manual_period'),
            'manual_duration': LaunchConfiguration('manual_duration'),
            'service_timeout': LaunchConfiguration('service_timeout'),
            'ik_timeout': LaunchConfiguration('ik_timeout'),
            'avoid_collisions': LaunchConfiguration('avoid_collisions'),
        }],
    )

    return LaunchDescription(declared_arguments + [reachability_tester])
