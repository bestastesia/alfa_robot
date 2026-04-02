"""
实机 GUI 滑块控制 + RViz 可视化

使用方式:
  ros2 launch alfa_robot_bringup slider_control_real_hw.launch.py

可选参数:
  canopen_profile_velocity:=50000   # 线性关节速度 (pulses/s)
  canopen_profile_accel:=50000      # 线性关节加速度 (pulses/s²)

启动前请先建立 CAN 接口:
  sudo ip link set can0 up type can bitrate 1000000
  sudo ip link set can1 up type can bitrate 1000000
  sudo ip link set can2 up type can bitrate 1000000
  sudo ip link set can3 up type can bitrate 1000000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            "controllers_file": "alfa_robot_controllers.yaml",
            "use_joint_gui_control": "true",
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [alfa_robot_launch])
