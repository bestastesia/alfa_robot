from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rerun', default_value='false'),
        DeclareLaunchArgument('rerun_connect', default_value='false'),
        DeclareLaunchArgument('frame_id', default_value='world'),
        Node(
            package='alfa_robot_benchmarks',
            executable='planning_scene_visualizer.py',
            name='planning_scene_visualizer',
            output='screen',
            parameters=[{
                'planning_scene_topic': '/planning_scene',
                'collision_object_topic': '/collision_object',
                'marker_topic': '/alfa_visualization/planning_scene_markers',
                'frame_id': LaunchConfiguration('frame_id'),
                'use_rerun': LaunchConfiguration('use_rerun'),
                'rerun_connect': LaunchConfiguration('rerun_connect'),
            }],
        ),
    ])
