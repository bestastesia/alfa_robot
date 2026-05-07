import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('lio_2d_mapper')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # 点云转LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': '2d_body',
                'transform_tolerance': 0.05,
                'min_height': -0.9,
                'max_height': 1.0,
                'angle_min': -1.0472,
                'angle_max':  1.0472,
                'angle_increment': 0.00872,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 20.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ]
        ),

        # Nav2 各节点
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],  # 注意：你已有cmd_vel_nav
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params_file],
        ),
    ])