from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # 点云转 LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan_out', '/scan'),
            ],
            parameters=[
                {
                    'use_sim_time': False,
                    'target_frame': 'body',
                    'output_frame_id': 'body',
                    'min_height': -0.2,
                    'max_height': 2.0,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.0087,
                    'scan_time': 0.1,
                    'range_min': 0.1,
                    'range_max': 150.0,
                }
            ],
        ),

        # 启动 RViz2
        # ExecuteProcess(
        #     cmd=['rviz2', '-d', '/home/ar/FastLio/rviz/fastlio_to_scan.rviz'],
        #     output='screen'
        # )
    ])
