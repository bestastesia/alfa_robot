"""
auto_grasp.launch.py
一键启动完整自动抓取流程：
  1. MID360 雷达
  2. D455 相机
  3. Fast-LIO 建图
  4. 箱体 6D Pose 感知
  5. auto_grasp_node（等第一条识别结果，转换后启动 MoveIt 实机运动）
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------- 传感器 ----------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py',
            ])
        ),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
            ])
        ),
    )

    # ---------- Fast-LIO ----------
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'false',
        }.items(),
    )

    # ---------- 箱体感知 ----------
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('box_perception'), 'launch', 'perception.launch.py',
            ])
        ),
    )

    # ---------- 自动抓取协调节点 ----------
    auto_grasp_node = Node(
        package='alfa_robot_bringup',
        executable='auto_grasp_node.py',
        name='auto_grasp_node',
        output='screen',
    )

    # ---------- 按时间顺序延迟启动 ----------
    delayed_camera = TimerAction(period=3.0, actions=[camera_launch])
    delayed_fast_lio = TimerAction(period=6.0, actions=[fast_lio_launch])
    delayed_perception = TimerAction(period=9.0, actions=[perception_launch])
    # 感知节点就绪后再启动协调节点
    delayed_auto_grasp = TimerAction(period=15.0, actions=[auto_grasp_node])

    return LaunchDescription([
        lidar_launch,
        delayed_camera,
        delayed_fast_lio,
        delayed_perception,
        delayed_auto_grasp,
    ])
