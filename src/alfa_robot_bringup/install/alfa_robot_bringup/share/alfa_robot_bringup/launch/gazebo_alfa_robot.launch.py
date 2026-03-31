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
# Gazebo (GZ / Ignition) 仿真启动文件
# 依赖: ros-humble-ros-gz-sim, ros-humble-gz-ros2-control (Humble + Fortress)
#

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, simulation_controllers_path: str):
    """Generate URDF with sim_gazebo:=true and spawn robot_state_publisher."""
    xacro_path = os.path.join(
        get_package_share_directory("alfa_robot_description"),
        "urdf",
        "alfa_robot.urdf.xacro",
    )
    robot_description_config = xacro.process_file(
        xacro_path,
        mappings={
            "prefix": "",
            "use_mock_hardware": "false",
            "mock_sensor_commands": "false",
            "sim_gazebo_classic": "false",
            "sim_gazebo": "true",
            "simulation_controllers": simulation_controllers_path,
            "real_hardware_plugin": "alfa_robot_hardware/AlfaRobotHW",
        },
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    return [robot_state_publisher]


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization.",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the robot.",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    namespace = LaunchConfiguration("namespace")

    # Absolute path to controller yaml (required by gz_ros2_control plugin)
    bringup_share = get_package_share_directory("alfa_robot_bringup")
    simulation_controllers_path = os.path.join(
        bringup_share, "config", "alfa_robot_controllers.yaml"
    )

    robot_state_publisher = OpaqueFunction(
        function=lambda context: get_robot_description(
            context, simulation_controllers_path
        ),
    )

    # GZ resource path for meshes
    desc_share = get_package_share_directory("alfa_robot_description")
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.dirname(desc_share)

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        arguments=["-topic", "/robot_description"],
        output="screen",
    )

    # Controller loaders (chain: spawn -> joint_state_broadcaster -> base -> left_arm -> right_arm -> velocity)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_base_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "base_position_controller",
        ],
        output="screen",
    )
    load_left_arm_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "left_arm_position_controller",
        ],
        output="screen",
    )
    load_right_arm_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "right_arm_position_controller",
        ],
        output="screen",
    )
    load_forward_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "forward_velocity_controller",
        ],
        output="screen",
    )

    rviz_config_file = os.path.join(desc_share, "rviz", "alfa_robot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=namespace,
        parameters=[{"source_list": ["joint_states"], "rate": 30, "use_sim_time": True}],
    )

    return LaunchDescription([
        use_rviz_arg,
        namespace_arg,
        gazebo_empty_world,
        robot_state_publisher,
        rviz_node,
        spawn,
        joint_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_base_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_base_position_controller,
                on_exit=[load_left_arm_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_left_arm_position_controller,
                on_exit=[load_right_arm_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_right_arm_position_controller,
                on_exit=[load_forward_velocity_controller],
            )
        ),
    ])
