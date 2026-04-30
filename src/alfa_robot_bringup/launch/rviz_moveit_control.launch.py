"""
RViz + MoveIt 控制机械臂启动文件

组合方案：
  1. slider_control_real_hw.launch.py 的硬件通信
  2. MoveIt 的规划功能
  3. RViz 可视化

使用方法：
  ros2 launch alfa_robot_bringup rviz_moveit_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("canopen_profile_velocity", default_value="50000"),
        DeclareLaunchArgument("canopen_profile_accel", default_value="50000"),
    ]

    # 1. 硬件控制 (来自 slider_control_real_hw.launch.py)
    alfa_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("alfa_robot_bringup"),
                "launch",
                "alfa_robot.launch.py",
            ])
        ]),
        launch_arguments={
            "use_mock_hardware": "false",
            "robot_controller": "left_arm_controller",
            "runtime_config_package": "alfa_robot_bringup",
            "controllers_file": "alfa_robot_moveit_real_controllers.yaml",
            "use_joint_gui_control": "false",
            "use_rviz": "false",  # 禁用 alfa_robot 的 RViz，使用 MoveIt 的 RViz
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    # 2. MoveIt move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("alfa_robot_moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        ]),
    )

    # 3. RViz
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("alfa_robot_moveit_config"),
                "launch",
                "moveit_rviz.launch.py",
            ])
        ]),
    )

    # 4. 静态 TF
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("alfa_robot_moveit_config"),
                "launch",
                "static_virtual_joint_tfs.launch.py",
            ])
        ]),
    )

    # 启动顺序：硬件 → (延迟) → MoveIt + RViz
    delay_moveit = TimerAction(
        period=10.0,  # 等待硬件和控制器完全启动
        actions=[
            static_tf_launch,
            move_group_launch,
            moveit_rviz_launch,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            alfa_robot_launch,
            delay_moveit,
        ]
    )
