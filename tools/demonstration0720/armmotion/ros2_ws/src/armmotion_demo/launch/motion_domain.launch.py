from alfa_robot_execution_bridge.joints import RT_CONTROL_ACTION_NAME
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
            DeclareLaunchArgument(
                "trajectory_action",
                default_value=RT_CONTROL_ACTION_NAME,
            ),
            DeclareLaunchArgument("source_ws", default_value="/motion_ws"),
            DeclareLaunchArgument("output_root", default_value="/motion_data"),
            DeclareLaunchArgument("trajectory_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("execution_speed_scale", default_value="3.0"),
            DeclareLaunchArgument("max_updown_speed_m_s", default_value="0.15"),
            DeclareLaunchArgument("recapture_preferred_updown_m", default_value="0.3"),
            DeclareLaunchArgument(
                "turn_zero_target_y_compensation_m", default_value="0.0"
            ),
            DeclareLaunchArgument("enable_trajectory_cache", default_value="true"),
            DeclareLaunchArgument("require_trajectory_cache_hit", default_value="false"),
            DeclareLaunchArgument(
                "trajectory_cache_fallback_on_planning_failure", default_value="true"
            ),
            DeclareLaunchArgument("allow_partial_domain_test", default_value="true"),
            Node(
                package="armmotion_demo",
                executable="mock_current_rt_control",
                condition=IfCondition(LaunchConfiguration("use_mock_rt_control")),
                output="screen",
                parameters=[
                    {
                        "trajectory_action": LaunchConfiguration("trajectory_action"),
                    }
                ],
            ),
            Node(
                package="armmotion_demo",
                executable="domain_motion_server",
                output="screen",
                parameters=[
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "trajectory_action": LaunchConfiguration("trajectory_action"),
                        "source_ws": LaunchConfiguration("source_ws"),
                        "output_root": LaunchConfiguration("output_root"),
                        "trajectory_rate_hz": LaunchConfiguration("trajectory_rate_hz"),
                        "execution_speed_scale": LaunchConfiguration("execution_speed_scale"),
                        "max_updown_speed_m_s": LaunchConfiguration(
                            "max_updown_speed_m_s"
                        ),
                        "recapture_preferred_updown_m": LaunchConfiguration(
                            "recapture_preferred_updown_m"
                        ),
                        "turn_zero_target_y_compensation_m": LaunchConfiguration(
                            "turn_zero_target_y_compensation_m"
                        ),
                        "enable_trajectory_cache": LaunchConfiguration(
                            "enable_trajectory_cache"
                        ),
                        "require_trajectory_cache_hit": LaunchConfiguration(
                            "require_trajectory_cache_hit"
                        ),
                        "trajectory_cache_fallback_on_planning_failure": LaunchConfiguration(
                            "trajectory_cache_fallback_on_planning_failure"
                        ),
                        "allow_partial_domain_test": LaunchConfiguration(
                            "allow_partial_domain_test"
                        ),
                    }
                ],
            ),
        ]
    )
