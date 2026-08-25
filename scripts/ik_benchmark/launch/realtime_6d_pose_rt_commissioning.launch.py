"""Guarded ELECTRI-102 commissioning launch; never used by the Mock preview."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_current_motion_initial_pose() -> tuple[list[float], float]:
    baseline_path = os.path.join(
        get_package_share_directory("alfa_robot_moveit_config"),
        "config",
        "motion_baselines",
        "current_motion_baseline.yaml",
    )
    with open(baseline_path, encoding="utf-8") as baseline_file:
        baseline = yaml.safe_load(baseline_file)
    arm_pose = [float(value) for value in baseline["loaded_planning"]["loaded_pose_deg"]]
    if len(arm_pose) != 6:
        raise ValueError("current Motion loaded_pose_deg must contain exactly 6 values")
    return arm_pose, float(baseline["updown"]["default_home_m"])


def generate_launch_description() -> LaunchDescription:
    initial_arm_pose_deg, default_updown = load_current_motion_initial_pose()
    arm = LaunchConfiguration("arm")
    stage = LaunchConfiguration("commissioning_stage")
    fixed_updown = LaunchConfiguration("fixed_updown")
    target_rate_hz = LaunchConfiguration("target_rate_hz")
    batch_rate_hz = LaunchConfiguration("batch_rate_hz")
    allow_provisional_limits = LaunchConfiguration("allow_provisional_limits")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_rerun = LaunchConfiguration("enable_rerun")
    spawn_viewer = LaunchConfiguration("spawn_viewer")
    enable_target_marker = LaunchConfiguration("enable_target_marker")
    allow_real_command = PythonExpression(
        ["'", stage, "' in ('hold', 'track')"]
    )
    enable_target_tracking = PythonExpression(["'", stage, "' == 'track'"])
    rviz_config = os.path.join(
        get_package_share_directory("alfa_robot_benchmarks"),
        "prototypes",
        "realtime_6d_pose",
        "realtime_6d_pose.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm", default_value="left"),
            DeclareLaunchArgument("commissioning_stage", default_value="observe"),
            DeclareLaunchArgument("fixed_updown", default_value=str(default_updown)),
            DeclareLaunchArgument("target_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("batch_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("allow_provisional_limits", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("enable_rerun", default_value="true"),
            DeclareLaunchArgument("spawn_viewer", default_value="true"),
            # Real commissioning starts without an interactive writer.  Start
            # the marker separately only after the first hold is ACKed.
            DeclareLaunchArgument("enable_target_marker", default_value="false"),
            Node(
                package="alfa_robot_benchmarks",
                executable="realtime_6d_pose_prototype",
                name="realtime_6d_pose_motion",
                output="screen",
                parameters=[
                    {
                        "arm": arm,
                        "pose_source": "topic",
                        "initial_arm_joints_deg": initial_arm_pose_deg,
                        "initial_other_arm_joints_deg": initial_arm_pose_deg,
                        "rt_control_joint_names": list(RT_CONTROL_JOINT_NAMES),
                        "fixed_updown": ParameterValue(fixed_updown, value_type=float),
                        "wait_for_feedback_init": True,
                        "target_rate_hz": ParameterValue(target_rate_hz, value_type=float),
                        "jump_threshold_deg": 10.0,
                        "collision_edge_step_deg": 2.0,
                    }
                ],
            ),
            Node(
                package="alfa_robot_benchmarks",
                executable="rolling_pose_client.py",
                name="realtime_6d_pose_rolling_client",
                output="screen",
                parameters=[
                    {
                        "backend": "real",
                        "arm": arm,
                        "batch_rate_hz": ParameterValue(batch_rate_hz, value_type=float),
                        "allow_real_command": ParameterValue(
                            allow_real_command, value_type=bool
                        ),
                        "allow_provisional_limits": ParameterValue(
                            allow_provisional_limits, value_type=bool
                        ),
                        "enable_target_tracking": ParameterValue(
                            enable_target_tracking, value_type=bool
                        ),
                        "stable_sample_count": 25,
                        "stable_rotary_span_deg": 0.05,
                        "stable_updown_span_m": 0.0002,
                    }
                ],
            ),
            Node(
                package="alfa_robot_benchmarks",
                executable="interactive_6d_target.py",
                name="interactive_6d_target",
                output="screen",
                condition=IfCondition(enable_target_marker),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="realtime_6d_pose_rviz",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(enable_rviz),
            ),
            Node(
                package="alfa_robot_benchmarks",
                executable="realtime_6d_pose_rerun.py",
                name="realtime_6d_pose_rerun_prototype",
                output="screen",
                condition=IfCondition(enable_rerun),
                parameters=[
                    {
                        "arm": arm,
                        "spawn_viewer": ParameterValue(spawn_viewer, value_type=bool),
                        "log_rate_hz": 30.0,
                    }
                ],
            ),
        ]
    )
