from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder('alfa_robot', package_name='alfa_robot_moveit_config').to_moveit_configs()
    return LaunchDescription([
        DeclareLaunchArgument('scene_x_shift', default_value='-3.84'),
        DeclareLaunchArgument('scene_y_shift', default_value='0.0'),
        DeclareLaunchArgument('scene_z_shift', default_value='0.0'),
        DeclareLaunchArgument('max_scene_objects', default_value='20'),
        Node(
            package='alfa_robot_benchmarks',
            executable='temporary_moveit_joint_planner',
            name='temporary_moveit_joint_planner',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {
                    'group_name': 'dual_v5_arm_with_base',
                    'service_name': '/alfa_moveit/plan_joint_target',
                    'frame_id': 'world',
                    'mujoco_scene_xml': '/mnt/mydisk/ALFA/alfa_robot/simulation/mujoco/scene.xml',
                    'max_scene_objects': LaunchConfiguration('max_scene_objects'),
                    'scene_x_shift': LaunchConfiguration('scene_x_shift'),
                    'scene_y_shift': LaunchConfiguration('scene_y_shift'),
                    'scene_z_shift': LaunchConfiguration('scene_z_shift'),
                },
            ],
        ),
    ])
