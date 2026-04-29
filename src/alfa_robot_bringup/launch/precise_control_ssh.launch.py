"""
实机精确控制（无 GUI，适合 SSH 远程操作）

使用方式:
  ros2 launch alfa_robot_bringup precise_control_ssh.launch.py

功能:
  - 启动机械臂控制系统（无 GUI、无 RViz）
  - 启动精确控制命令行工具
  - 适合 SSH 远程无图形界面操作

启动前请先建立 CAN 接口:
  sudo ip link set can0 up type can bitrate 1000000
  sudo ip link set can1 up type can bitrate 1000000
  sudo ip link set can2 up type can bitrate 1000000
  sudo ip link set can3 up type can bitrate 1000000

控制命令:
  list              - 列出所有关节当前值
  set <joint> <val> - 设置关节值 (高精度)
  move <v1> ...     - 批量设置所有关节
  home              - 归零
  save <name>       - 保存姿态
  load <name>       - 加载姿态
  quit              - 退出
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "canopen_profile_velocity",
            default_value="50000",
            description="CANopen 线性关节速度 (pulses/s)",
        ),
        DeclareLaunchArgument(
            "canopen_profile_accel",
            default_value="50000",
            description="CANopen 线性关节加速度 (pulses/s²)",
        ),
    ]

    # 启动主控制系统（无 GUI，无 RViz）
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
            "robot_controller": "all_position_controller",
            "runtime_config_package": "alfa_robot_bringup",
            "controllers_file": "alfa_robot_moveit_real_controllers.yaml",
            "use_joint_gui_control": "false",  # 不启动 GUI
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    # 精确控制工具（延迟启动，等待系统就绪）
    precise_control_node = Node(
        package="alfa_robot_bringup",
        executable="precise_joint_control.py",
        name="precise_joint_control",
        output="screen",
        parameters=[{
            "controller_name": "all_position_controller",
        }],
    )

    delayed_precise_control = TimerAction(
        period=10.0,  # 等待硬件系统完全启动
        actions=[precise_control_node],
    )

    return LaunchDescription(declared_arguments + [
        alfa_robot_launch,
        delayed_precise_control,
    ])
