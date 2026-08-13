from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("service_name", default_value="/robot_motion/check_collision"),
            DeclareLaunchArgument("joint_group", default_value="dual_arm"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            Node(
                package="alfa_robot_moveit_config",
                executable="motion_collision_service_node",
                name="motion_collision_service",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    {
                        "service_name": LaunchConfiguration("service_name"),
                        "joint_group": LaunchConfiguration("joint_group"),
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                    }
                ],
            ),
        ]
    )
