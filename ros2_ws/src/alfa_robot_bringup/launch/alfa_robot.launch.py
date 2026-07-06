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
Alfa Robot 硬件启动文件

功能：
  1. 启动 robot_state_publisher 发布机器人状态
  2. 启动 ros2_control_node 进行硬件控制
  3. 启动控制器 (joint_state_broadcaster, 用户选择的控制器)
  4. 可选：启动 RViz 可视化
  5. 可选：启动 GUI 滑块控制

生命周期管理：
  - 使用事件处理器确保正确的启动和关闭顺序
  - Ctrl+C 时确保所有节点正常退出
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, TimerAction, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="alfa_robot_moveit_config",
            description='Package with the controller\'s configuration in "config" folder.',
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="alfa_robot_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="alfa_robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors. Only if use_mock_hardware=true.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="dual_arm_controller",
            choices=[
                "dual_arm_controller",
                "all_position_controller",
            ],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "use_joint_gui_control",
            default_value="false",
            description="Enable joint_state_publisher_gui + bridge to control motors via sliders. "
            "Works with robot_controller:=all_position_controller (default).",
        ),
        DeclareLaunchArgument(
            "real_hardware_plugin",
            default_value="alfa_robot_hardware/AlfaRobotHW",
            description="Plugin for real hardware control (used when use_mock_hardware:=false).",
        ),
        DeclareLaunchArgument(
            "canopen_profile_velocity",
            default_value="50000",
            description="CANopen motor profile velocity in pulses/s. 50000≈3.8mm/s, 100000≈7.6mm/s.",
        ),
        DeclareLaunchArgument(
            "canopen_profile_accel",
            default_value="50000",
            description="CANopen motor profile acceleration in pulses/s².",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz visualization.",
        ),
    ]

    # LaunchConfiguration references
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
    use_rviz = LaunchConfiguration("use_rviz")

    # URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " prefix:=", prefix,
            " use_mock_hardware:=", use_mock_hardware,
            " mock_sensor_commands:=", mock_sensor_commands,
            " real_hardware_plugin:=", real_hardware_plugin,
            " canopen_profile_velocity:=", canopen_profile_velocity,
            " canopen_profile_accel:=", canopen_profile_accel,
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "alfa_robot.rviz"]
    )

    # ===== 节点定义 =====

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    # controller_manager 需在 robot_state_publisher 发布 /robot_description 后再启动
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
    )

    # RViz 可选启动
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    # 控制器 spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="robot_controller_spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

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

    # ===== 生命周期管理 =====
    #
    # 启动顺序：
    #   robot_state_publisher → (2s) → control_node
    #   control_node → (3s) → joint_state_broadcaster_spawner
    #   joint_state_broadcaster_spawner → robot_controller_spawner
    #   robot_controller_spawner → rviz_node
    #
    # 关闭顺序（Ctrl+C）：
    #   自动按依赖关系的逆序关闭

    # 1. robot_state_publisher 启动后，延迟启动 control_node
    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[TimerAction(period=2.0, actions=[control_node])],
        )
    )

    # 2. control_node 启动后，延迟启动 joint_state_broadcaster
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    # 3. joint_state_broadcaster 启动完成后，启动 robot_controller
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # 4. robot_controller 启动完成后，启动 RViz
    #    (确保所有控制器就绪后再启动可视化)
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            # 核心节点
            robot_state_pub_node,
            joint_gui_control_group,

            # 生命周期事件处理器
            delay_control_node,
            delay_jsb,
            delay_robot_controller,
            delay_rviz,
        ]
    )
