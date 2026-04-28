"""
精确关节控制工具 + RViz 可视化

使用方式:
  ros2 launch alfa_robot_bringup precise_control_real_hw.launch.py

功能:
  - 命令行精确输入关节值 (支持高精度，6+ 小数位)
  - RViz 实时可视化
  - 支持保存/加载姿态

启动前请先建立 CAN 接口:
  sudo ip link set can0 up type can bitrate 1000000
  sudo ip link set can1 up type can bitrate 1000000
  sudo ip link set can2 up type can bitrate 1000000
  sudo ip link set can3 up type can bitrate 1000000
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
            description="CANopen 线性关节速度 (pulses/s)。50000≈3.8mm/s",
        ),
        DeclareLaunchArgument(
            "canopen_profile_accel",
            default_value="50000",
            description="CANopen 线性关节加速度 (pulses/s²)",
        ),
    ]

    # 启动主控制系统
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
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    # 精确控制工具 (延迟启动，等待系统就绪)
    precise_control_node = Node(
        package="alfa_robot_bringup",
        executable="precise_joint_control.py",
        name="precise_joint_control",
        output="screen",
        prefix="xterm -e" if _check_xterm() else "",
    )

    delayed_precise_control = TimerAction(
        period=8.0,  # 等待硬件系统启动
        actions=[precise_control_node],
    )

    return LaunchDescription(declared_arguments + [
        alfa_robot_launch,
        delayed_precise_control,
    ])


def _check_xterm():
    """检查 xterm 是否可用（用于在新窗口启动控制界面）。"""
    import shutil
    return shutil.which("xterm") is not None
