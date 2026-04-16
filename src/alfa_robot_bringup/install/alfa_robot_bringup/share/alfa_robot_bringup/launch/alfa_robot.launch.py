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

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/b-robotized/ros_team_workspace) repository.
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="alfa_robot_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="alfa_robot_controllers.yaml",
            description="YAML file with the controllers configuration. "
            "Use alfa_robot_controllers_can.yaml for real CAN hardware (minimal config).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="alfa_robot_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="alfa_robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="all_position_controller",
            choices=["all_position_controller", "forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start. Use all_position_controller for GUI testing.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joint_gui_control",
            default_value="false",
            description="Enable joint_state_publisher_gui + bridge to control motors via sliders. "
            "Requires robot_controller:=forward_position_controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_hardware_plugin",
            default_value="alfa_robot_hardware/AlfaRobotHW",
            description="Plugin for real hardware control (used when use_mock_hardware:=false).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "canopen_profile_velocity",
            default_value="50000",
            description="CANopen (Leisai) motor profile velocity in pulses/s. "
            "50000≈3.8mm/s, 100000≈7.6mm/s, 500000≈38mm/s.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "canopen_profile_accel",
            default_value="50000",
            description="CANopen (Leisai) motor profile acceleration in pulses/s². "
            "Controls how fast the motor ramps up to profile velocity.",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    use_joint_gui_control = LaunchConfiguration("use_joint_gui_control")
    real_hardware_plugin = LaunchConfiguration("real_hardware_plugin")
    canopen_profile_velocity = LaunchConfiguration("canopen_profile_velocity")
    canopen_profile_accel = LaunchConfiguration("canopen_profile_accel")

    # Get URDF via xacro
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
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
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

    # controller_manager 从 /robot_description topic 获取 URDF，避免弃用警告（由 robot_state_publisher 发布）
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

    robot_controller_names = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    # No inactive controllers defined - add controller names here if needed
    inactive_robot_controller_names = []
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    # controller_manager 需在 robot_state_publisher 发布 /robot_description 后再启动
    delay_control_node_after_robot_state_publisher = RegisterEventHandler(
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
    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
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

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        robot_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        inactive_robot_controller_spawners[i - 1]
                        if i > 0
                        else robot_controller_spawners[-1]
                    ),
                    on_exit=[controller],
                )
            )
        ]

    # GUI 滑块控制：joint_state_publisher_gui + 桥接节点
    joint_gui_control_group = GroupAction(
        condition=IfCondition(use_joint_gui_control),
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

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            rviz_node,
            joint_gui_control_group,
            delay_control_node_after_robot_state_publisher,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
