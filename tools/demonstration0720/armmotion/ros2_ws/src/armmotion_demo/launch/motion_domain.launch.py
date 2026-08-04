from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_rt_control", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("source_ws", default_value="/motion_ws"),
            DeclareLaunchArgument("output_root", default_value="/motion_data"),
            DeclareLaunchArgument("trajectory_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("execution_speed_scale", default_value="3.0"),
            DeclareLaunchArgument("allow_partial_domain_test", default_value="true"),
            Node(
                package="armmotion_demo",
                executable="mock_current_rt_control",
                condition=IfCondition(LaunchConfiguration("use_mock_rt_control")),
                output="screen",
            ),
            Node(
                package="armmotion_demo",
                executable="domain_motion_server",
                output="screen",
                parameters=[
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "source_ws": LaunchConfiguration("source_ws"),
                        "output_root": LaunchConfiguration("output_root"),
                        "trajectory_rate_hz": LaunchConfiguration("trajectory_rate_hz"),
                        "execution_speed_scale": LaunchConfiguration("execution_speed_scale"),
                        "allow_partial_domain_test": LaunchConfiguration(
                            "allow_partial_domain_test"
                        ),
                    }
                ],
            ),
        ]
    )
