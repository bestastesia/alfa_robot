import os

import yaml
from ament_index_python.packages import get_package_share_directory
from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    updown = float(baseline["updown"]["default_home_m"])
    return arm_pose, updown


def generate_launch_description() -> LaunchDescription:
    initial_arm_pose_deg, default_updown = load_current_motion_initial_pose()
    arm = LaunchConfiguration("arm")
    fixed_updown = LaunchConfiguration("fixed_updown")
    target_rate_hz = LaunchConfiguration("target_rate_hz")
    jump_threshold_deg = LaunchConfiguration("jump_threshold_deg")
    collision_edge_step_deg = LaunchConfiguration("collision_edge_step_deg")
    batch_rate_hz = LaunchConfiguration("batch_rate_hz")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_rerun = LaunchConfiguration("enable_rerun")
    spawn_viewer = LaunchConfiguration("spawn_viewer")
    rviz_config = os.path.join(
        get_package_share_directory("alfa_robot_benchmarks"),
        "prototypes",
        "realtime_6d_pose",
        "realtime_6d_pose.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm", default_value="left"),
            DeclareLaunchArgument("fixed_updown", default_value=str(default_updown)),
            DeclareLaunchArgument("target_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("jump_threshold_deg", default_value="10.0"),
            DeclareLaunchArgument("collision_edge_step_deg", default_value="2.0"),
            DeclareLaunchArgument("batch_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("enable_rerun", default_value="true"),
            DeclareLaunchArgument("spawn_viewer", default_value="true"),
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
                        # Motion-TX preview owns an internal command state.  It
                        # deliberately waits for no controller feedback.
                        "wait_for_feedback_init": False,
                        "feedback_sync_enabled": False,
                        "target_rate_hz": ParameterValue(target_rate_hz, value_type=float),
                        "jump_threshold_deg": ParameterValue(
                            jump_threshold_deg, value_type=float
                        ),
                        "collision_edge_step_deg": ParameterValue(
                            collision_edge_step_deg, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="alfa_robot_benchmarks",
                executable="motion_rolling_preview.py",
                name="realtime_6d_pose_motion_tx_preview",
                output="screen",
                parameters=[
                    {
                        "arm": arm,
                        "fixed_updown": ParameterValue(fixed_updown, value_type=float),
                        "initial_arm_joints_deg": initial_arm_pose_deg,
                        "batch_rate_hz": ParameterValue(batch_rate_hz, value_type=float),
                    }
                ],
            ),
            Node(
                package="alfa_robot_benchmarks",
                executable="interactive_6d_target.py",
                name="interactive_6d_target",
                output="screen",
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
