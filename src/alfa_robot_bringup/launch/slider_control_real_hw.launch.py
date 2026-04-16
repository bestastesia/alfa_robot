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

回零位行为:
  启动后 homing_node 会自动等待控制器就绪，然后发送一次全零位指令（自动回零位）。
  关闭时硬件插件的 on_deactivate() 会驱动所有关节回零位再断电（use_safe_shutdown=true）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
            "controllers_file": "alfa_robot_controllers.yaml",
            "use_joint_gui_control": "true",
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    # 启动后自动回零位：等待 all_position_controller 激活后发布一次全零位指令
    homing_node = Node(
        package="alfa_robot_bringup",
        executable="homing_node.py",
        name="homing_node",
        output="screen",
        parameters=[{
            "controller_name": "all_position_controller",
            # 关节顺序与 alfa_robot_controllers.yaml 中 all_position_controller 一致:
            # turn, updown, leftarmbase, leftjoint1-4, rightarmbase, rightjoint1-4
            "home_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "publish_count": 5,
            "poll_interval": 1.0,
        }],
    )

    return LaunchDescription(declared_arguments + [alfa_robot_launch, homing_node])
