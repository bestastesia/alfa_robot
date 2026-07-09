from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("service_name", default_value="/robot_motion/solve_arm_ik"),
            DeclareLaunchArgument("root_samples", default_value="720"),
            DeclareLaunchArgument("default_max_solutions", default_value="8"),
            DeclareLaunchArgument("default_position_tolerance", default_value="0.0001"),
            DeclareLaunchArgument("default_orientation_tolerance", default_value="0.0001"),
            DeclareLaunchArgument("top_suction_orientation_tolerance", default_value="0.05"),
            Node(
                package="alfa_robot_moveit_config",
                executable="analytic_arm_ik_service_node",
                name="analytic_arm_ik_service",
                output="screen",
                parameters=[
                    {
                        "service_name": LaunchConfiguration("service_name"),
                        "root_samples": LaunchConfiguration("root_samples"),
                        "default_max_solutions": LaunchConfiguration("default_max_solutions"),
                        "default_position_tolerance": LaunchConfiguration("default_position_tolerance"),
                        "default_orientation_tolerance": LaunchConfiguration("default_orientation_tolerance"),
                        "top_suction_orientation_tolerance": LaunchConfiguration(
                            "top_suction_orientation_tolerance"
                        ),
                    }
                ],
            ),
        ]
    )
