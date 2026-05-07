# launch/lidar_tf_node.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lio_2d_mapper',       
            executable='lidar_tf_node',
            name='lidar_tf_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'map',
                'base_frame': 'body',
                '2d_base_frame': '2d_body',
                'fastlio_odom_topic': '/Odometry',
            }]
        ),
        Node(
            package="lio_2d_mapper",
            executable="offline_map_publisher",
            name="offline_map_publisher",
            output="screen",
            parameters=[{
                "map_path": "/home/ar/fast_lio_ws/src/lio_2d_mapper/maps/0428_2/map.pgm",
                "map_yaml_path": "/home/ar/fast_lio_ws/src/lio_2d_mapper/maps/0428_2/map.yaml", 
                "map_topic": "/offline_map",
                "map_frame": "map"
            }]
        ),
        Node(
            package="lio_2d_mapper",
            executable="mapper_node",
            name="mapper_node",
            output="screen",
            parameters=[{
                'use_sim_time': False,
            }]
        ),

        # Node(
        #     package="pointcloud_to_laserscan",
        #     executable="pointcloud_to_laserscan_node",
        #     name="pointcloud_to_laserscan_node",
        #     output="screen",
        #     # 参数配置
        #     parameters=[{
        #         "target_frame": "2d_body",
        #         "min_height": -0.9,
        #         "max_height": 0.5,
        #     }],
        #     # 话题重映射
        #     remappings=[
        #         ("cloud_in", "/cloud_registered"),
        #         ("scan", "/scan"),
        #     ]
        # ),
    ])