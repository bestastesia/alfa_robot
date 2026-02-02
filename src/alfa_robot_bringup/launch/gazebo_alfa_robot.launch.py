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
# Gazebo Classic 仿真启动文件
# 依赖: sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Controller to spawn for arm control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz for visualization.",
        )
    )

    robot_controller = LaunchConfiguration("robot_controller")
    use_rviz = LaunchConfiguration("use_rviz")

    description_package = "alfa_robot_description"
    description_file = "alfa_robot.urdf.xacro"
    runtime_config_package = "alfa_robot_bringup"
    controllers_file = "alfa_robot_controllers.yaml"

    # 控制器配置文件路径 (gazebo_ros2_control 插件需要)
    simulation_controllers_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # 生成 robot_description (Gazebo 模式)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "use_mock_hardware:=false",
            " ",
            "sim_gazebo_classic:=true",
            " ",
            "simulation_controllers:=",
            simulation_controllers_path,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 启动 Gazebo 空世界 (默认 empty.world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
    )

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 将机器人 spawn 到 Gazebo
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "alfa_robot",
            "-x", "0",
            "-y", "0",
            "-z", "0.5",
        ],
        output="screen",
    )

    # 延迟 spawn，等待 Gazebo 就绪 (5秒)
    # 使用 TimerAction 而非 OnProcessStart，因为 IncludeLaunchDescription 不能作为 target_action
    delay_spawn_after_gazebo = TimerAction(
        period=5.0,
        actions=[
            robot_state_publisher_node,
            spawn_entity_node,
        ],
    )

    # 加载 joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 加载位置控制器
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

    # 延迟加载控制器 (spawn 完成后)
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                TimerAction(
                    period=2.0,
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

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "alfa_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_launch,
            delay_spawn_after_gazebo,
            delay_joint_state_broadcaster,
            delay_robot_controller,
            rviz_node,
        ]
    )
