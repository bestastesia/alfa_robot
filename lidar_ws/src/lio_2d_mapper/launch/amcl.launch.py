from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # ======================
        # 1. 地图服务器
        # ======================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': '/home/ar/fast_lio_ws/src/lio_2d_mapper/map_2d.yaml'
            }]
        ),

        # ======================
        # 2. AMCL
        # ======================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{

                'use_sim_time': False,

                # frame
                'global_frame_id': 'map',
                'odom_frame_id': 'camera_init',
                'base_frame_id': 'body',

                # scan
                'scan_topic': '/scan',
                'laser_likelihood_max_dist': 2.0,

                # particles
                'min_particles': 500,
                'max_particles': 2000,

                # laser model
                'laser_model_type': 'likelihood_field',
                'max_beams': 60,

                # update
                'update_min_d': 0.1,
                'update_min_a': 0.2,
            }]
        ),

        # ======================
        # 3. 自动初始化 AMCL（关键新增）
        # ======================
        Node(
            package='lio_2d_mapper',
            executable='lidar_init_pose_node',
            name='lidar_init_pose_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_topic': '/Odometry',
                'publish_once': True
            }]
        ),

        # ======================
        # 4. lifecycle 管理
        # ======================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),
    ])