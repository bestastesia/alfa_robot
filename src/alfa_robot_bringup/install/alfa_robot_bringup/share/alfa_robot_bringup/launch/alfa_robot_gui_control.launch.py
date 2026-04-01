# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
GUI 实机控制启动文件：通过 joint_state_publisher_gui 滑块控制真实硬件。
默认使用 all_position_controller + 实机硬件插件。
缺失的 CAN 总线只产生 WARN，不阻止系统启动。

用法:
  # 全部 4 条总线
  ros2 launch alfa_robot_bringup alfa_robot_gui_control.launch.py

  # 指定 CANopen 速度
  ros2 launch alfa_robot_bringup alfa_robot_gui_control.launch.py canopen_profile_velocity:=100000

  # 仅启动部分总线（缺失的自动跳过）
  ros2 launch alfa_robot_bringup alfa_robot_gui_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="alfa_robot_bringup",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="alfa_robot_controllers.yaml",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="alfa_robot_description",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="alfa_robot.urdf.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_hardware_plugin",
            default_value="alfa_robot_hardware/AlfaRobotHW",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "canopen_profile_velocity",
            default_value="50000",
            description="CANopen motor profile velocity in pulses/s. "
            "50000≈3.8mm/s, 100000≈7.6mm/s, 500000≈38mm/s.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "canopen_profile_accel",
            default_value="50000",
            description="CANopen motor profile acceleration in pulses/s².",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    real_hardware_plugin = LaunchConfiguration("real_hardware_plugin")
    canopen_profile_velocity = LaunchConfiguration("canopen_profile_velocity")
    canopen_profile_accel = LaunchConfiguration("canopen_profile_accel")

    # Get URDF via xacro — real hardware (use_mock_hardware:=false)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=false",
            " ",
            "mock_sensor_commands:=false",
            " ",
            "real_hardware_plugin:=",
            real_hardware_plugin,
            " ",
            "canopen_profile_velocity:=",
            canopen_profile_velocity,
            " ",
            "canopen_profile_accel:=",
            canopen_profile_accel,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "alfa_robot.rviz"]
    )

    # controller_manager subscribes to /robot_description topic
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["all_position_controller", "-c", "/controller_manager"],
    )

    # GUI slider control: joint_state_publisher_gui + bridge node
    joint_gui_control_group = GroupAction(
        actions=[
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                remappings=[("joint_states", "joint_states_gui")],
            ),
            Node(
                package="alfa_robot_bringup",
                executable="joint_states_to_controller_bridge.py",
                name="joint_states_to_controller_bridge",
                output="screen",
            ),
        ],
    )

    # Startup sequence: robot_state_pub → (2s) → control_node → (3s) → joint_state_broadcaster → controller
    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[control_node],
                ),
            ],
        )
    )
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            rviz_node,
            joint_gui_control_group,
            delay_control_node,
            delay_joint_state_broadcaster,
            delay_robot_controller,
        ]
    )
