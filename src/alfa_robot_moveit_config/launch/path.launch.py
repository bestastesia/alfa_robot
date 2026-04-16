#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    planning_group_arg = DeclareLaunchArgument(
        "planning_group",
        default_value="left_arm",
        description="Planning group: left_arm, right_arm, or dual_arm"
    )

    plan_only_arg = DeclareLaunchArgument(
        "plan_only",
        default_value="false",
        description="Only plan without executing"
    )

    velocity_scale_arg = DeclareLaunchArgument(
        "velocity_scale",
        default_value="0.2",
        description="Max velocity scaling factor"
    )

    acceleration_scale_arg = DeclareLaunchArgument(
        "acceleration_scale",
        default_value="0.2",
        description="Max acceleration scaling factor"
    )

    path_node = Node(
        package="alfa_robot_moveit_config",
        executable="path_node.py",
        name="path_node",
        output="screen",
        parameters=[{
            "planning_group": LaunchConfiguration("planning_group"),
            "plan_only": LaunchConfiguration("plan_only"),
            "max_velocity_scaling_factor": LaunchConfiguration("velocity_scale"),
            "max_acceleration_scaling_factor": LaunchConfiguration("acceleration_scale"),
        }],
    )

    return LaunchDescription([
        planning_group_arg,
        plan_only_arg,
        velocity_scale_arg,
        acceleration_scale_arg,
        path_node,
    ])
