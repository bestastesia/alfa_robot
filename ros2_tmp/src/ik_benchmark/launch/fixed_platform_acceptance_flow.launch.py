from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config").to_moveit_configs()
    temporary_planner_params = [
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
            'max_scene_objects': 20,
            'scene_x_shift': -3.69,
            'scene_y_shift': 0.0,
            'scene_z_shift': 0.0,
            'planning_time': LaunchConfiguration('planning_time'),
            'planning_attempts': LaunchConfiguration('planning_attempts'),
            'velocity_scale': LaunchConfiguration('moveit_velocity_scale'),
            'acceleration_scale': LaunchConfiguration('moveit_acceleration_scale'),
            'joint_goal_tolerance': 0.01,
            'validate_goal_state_collision': True,
        },
    ]

    return LaunchDescription([
        DeclareLaunchArgument('fixed_updown', default_value='0.18'),
        DeclareLaunchArgument('temporary_planner', default_value='true'),
        DeclareLaunchArgument('plc_mock', default_value='true'),
        DeclareLaunchArgument('velocity_limit_deg_s', default_value='5.0'),
        DeclareLaunchArgument('plc_execution_mode', default_value='stream'),
        DeclareLaunchArgument('moveit_velocity_scale', default_value='0.25'),
        DeclareLaunchArgument('moveit_acceleration_scale', default_value='0.2'),
        DeclareLaunchArgument('planning_time', default_value='8.0'),
        DeclareLaunchArgument('planning_attempts', default_value='20'),
        DeclareLaunchArgument('with_move_group', default_value='true'),
        DeclareLaunchArgument('with_visualization', default_value='true'),
        DeclareLaunchArgument('with_task_publisher', default_value='false'),
        DeclareLaunchArgument('workers', default_value='16'),
        DeclareLaunchArgument('h_candidate_count', default_value='16'),
        DeclareLaunchArgument('seed_count', default_value='32'),
        DeclareLaunchArgument('timeout', default_value='0.01'),
        DeclareLaunchArgument('check_collision', default_value='true'),
        DeclareLaunchArgument('left_x', default_value='0.70'),
        DeclareLaunchArgument('left_y', default_value='0.20'),
        DeclareLaunchArgument('left_z', default_value='1.40'),
        DeclareLaunchArgument('right_x', default_value='0.70'),
        DeclareLaunchArgument('right_y', default_value='-0.20'),
        DeclareLaunchArgument('right_z', default_value='1.40'),
        DeclareLaunchArgument('ik_max_attempts', default_value='5'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('alfa_robot_moveit_config')) / 'launch' / 'move_group.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('with_move_group')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('alfa_robot_plc_bridge')) / 'launch' / 'execution_with_safety.launch.py')
            ),
            launch_arguments={
                'mock': LaunchConfiguration('plc_mock'),
                'velocity_limit_deg_s': LaunchConfiguration('velocity_limit_deg_s'),
                'plc_execution_mode': LaunchConfiguration('plc_execution_mode'),
                'publish_joint_states': 'false',
            }.items(),
        ),
        Node(
            package='alfa_robot_benchmarks',
            executable='fixed_platform_kdl_ik_service.py',
            name='fixed_platform_kdl_ik_service',
            output='screen',
            parameters=[{
                'fixed_updown': LaunchConfiguration('fixed_updown'),
                'lock_updown': True,
                'workers': LaunchConfiguration('workers'),
                'h_candidate_count': LaunchConfiguration('h_candidate_count'),
                'seed_count': LaunchConfiguration('seed_count'),
                'timeout': LaunchConfiguration('timeout'),
                'check_collision': LaunchConfiguration('check_collision'),
                'fallback_enabled': False,
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
                'service_name': '/alfa_dual_ik/solve',
            }],
        ),
        Node(
            package='alfa_robot_benchmarks',
            executable='temporary_moveit_joint_planner',
            name='temporary_moveit_joint_planner',
            output='screen',
            condition=IfCondition(LaunchConfiguration('temporary_planner')),
            parameters=temporary_planner_params,
        ),
        Node(
            package='alfa_robot_benchmarks',
            executable='planning_scene_visualizer.py',
            name='planning_scene_visualizer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('with_visualization')),
            parameters=[{
                'planning_scene_topic': '/planning_scene',
                'collision_object_topic': '/collision_object',
                'marker_topic': '/alfa_visualization/planning_scene_markers',
                'frame_id': 'world',
                'use_rerun': False,
            }],
        ),
        Node(
            package='alfa_robot_benchmarks',
            executable='fixed_platform_task_orchestrator',
            name='fixed_platform_task_orchestrator',
            output='screen',
            parameters=[{
                'fixed_updown': LaunchConfiguration('fixed_updown'),
                'mock_planner': False,
                'ik_service': '/alfa_dual_ik/solve',
                'planner_service': '/alfa_moveit/plan_joint_target',
                'trajectory_topic': '/plc_joint_trajectory',
                'plc_state_topic': '/plc_bridge_state',
                'wait_execution_done': True,
                'ik_max_attempts': LaunchConfiguration('ik_max_attempts'),
            }],
        ),
        Node(
            package='alfa_robot_benchmarks',
            executable='mock_box_task_publisher',
            name='mock_box_task_publisher',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('with_task_publisher')),
            parameters=[{
                'left_x': LaunchConfiguration('left_x'),
                'left_y': LaunchConfiguration('left_y'),
                'left_z': LaunchConfiguration('left_z'),
                'right_x': LaunchConfiguration('right_x'),
                'right_y': LaunchConfiguration('right_y'),
                'right_z': LaunchConfiguration('right_z'),
                'task_topic': '/alfa_task/command',
                'status_topic': '/alfa_task/status',
            }],
        ),
    ])
