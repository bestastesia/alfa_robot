from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package_name: str, launch_file: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8766"),
            DeclareLaunchArgument("subscribe_joint_states", default_value="true"),
            DeclareLaunchArgument("state_publish_period_s", default_value="0.2"),
            DeclareLaunchArgument("execute_forward_action", default_value="true"),
            DeclareLaunchArgument(
                "execution_action_name",
                default_value="/dual_arm_trajectory_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument("execute_wait_for_action_timeout_s", default_value="10.0"),
            DeclareLaunchArgument("execute_wait_for_goal_acceptance", default_value="false"),
            DeclareLaunchArgument("execute_wait_for_result", default_value="false"),
            DeclareLaunchArgument("execute_wait_for_result_timeout_s", default_value="120.0"),
            DeclareLaunchArgument("execute_resample_before_forward", default_value="true"),
            DeclareLaunchArgument("execute_resample_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("execute_adapt_to_hardware_joint_order", default_value="true"),
            DeclareLaunchArgument("execute_hold_missing_from_joint_states", default_value="true"),
            DeclareLaunchArgument("task_service_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("run_dual_grasp_task_service_name", default_value="/robot_motion/run_dual_grasp_task"),
            DeclareLaunchArgument("task_receipt_topic", default_value="/robot_motion/task_receipt"),
            DeclareLaunchArgument("default_fixed_updown", default_value="0.08"),
            DeclareLaunchArgument("default_candidate_limit", default_value="8"),
            DeclareLaunchArgument("default_planning_mode", default_value="shortcut"),
            DeclareLaunchArgument("plan_trajectory_duration_s", default_value="0.0"),
            DeclareLaunchArgument("plan_trajectory_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("plan_max_joint_step_deg", default_value="4.5"),
            DeclareLaunchArgument("plan_max_updown_step_m", default_value="0.01"),
            DeclareLaunchArgument("plan_check_collision", default_value="true"),
            DeclareLaunchArgument("ik_root_samples", default_value="720"),
            DeclareLaunchArgument("ik_default_max_solutions", default_value="8"),
            DeclareLaunchArgument("collision_joint_group", default_value="dual_arm_with_base"),
            DeclareLaunchArgument("model_joint_states_topic", default_value="/robot_motion/model_joint_states"),
            IncludeLaunchDescription(
                package_launch("robot_motion_runtime", "runtime_services.launch.py"),
                launch_arguments={
                    "dashboard_host": LaunchConfiguration("dashboard_host"),
                    "dashboard_port": LaunchConfiguration("dashboard_port"),
                    "subscribe_joint_states": LaunchConfiguration("subscribe_joint_states"),
                    "state_publish_period_s": LaunchConfiguration("state_publish_period_s"),
                    "execute_forward_action": LaunchConfiguration("execute_forward_action"),
                    "execution_action_name": LaunchConfiguration("execution_action_name"),
                    "execute_wait_for_action_timeout_s": LaunchConfiguration("execute_wait_for_action_timeout_s"),
                    "execute_wait_for_goal_acceptance": LaunchConfiguration("execute_wait_for_goal_acceptance"),
                    "execute_wait_for_result": LaunchConfiguration("execute_wait_for_result"),
                    "execute_wait_for_result_timeout_s": LaunchConfiguration("execute_wait_for_result_timeout_s"),
                    "execute_resample_before_forward": LaunchConfiguration("execute_resample_before_forward"),
                    "execute_resample_rate_hz": LaunchConfiguration("execute_resample_rate_hz"),
                    "execute_adapt_to_hardware_joint_order": LaunchConfiguration("execute_adapt_to_hardware_joint_order"),
                    "execute_hold_missing_from_joint_states": LaunchConfiguration("execute_hold_missing_from_joint_states"),
                    "task_service_timeout_s": LaunchConfiguration("task_service_timeout_s"),
                    "run_dual_grasp_task_service_name": LaunchConfiguration("run_dual_grasp_task_service_name"),
                    "task_receipt_topic": LaunchConfiguration("task_receipt_topic"),
                    "default_fixed_updown": LaunchConfiguration("default_fixed_updown"),
                    "default_candidate_limit": LaunchConfiguration("default_candidate_limit"),
                    "default_planning_mode": LaunchConfiguration("default_planning_mode"),
                    "plan_trajectory_duration_s": LaunchConfiguration("plan_trajectory_duration_s"),
                    "plan_trajectory_rate_hz": LaunchConfiguration("plan_trajectory_rate_hz"),
                    "plan_max_joint_step_deg": LaunchConfiguration("plan_max_joint_step_deg"),
                    "plan_max_updown_step_m": LaunchConfiguration("plan_max_updown_step_m"),
                    "plan_check_collision": LaunchConfiguration("plan_check_collision"),
                    "collision_service_name": "/robot_motion/check_collision",
                    "model_joint_states_topic": LaunchConfiguration("model_joint_states_topic"),
                }.items(),
            ),
            IncludeLaunchDescription(
                package_launch("alfa_robot_moveit_config", "analytic_arm_ik_service.launch.py"),
                launch_arguments={
                    "service_name": "/robot_motion/solve_arm_ik",
                    "root_samples": LaunchConfiguration("ik_root_samples"),
                    "default_max_solutions": LaunchConfiguration("ik_default_max_solutions"),
                }.items(),
            ),
            IncludeLaunchDescription(
                package_launch("alfa_robot_moveit_config", "motion_collision_service.launch.py"),
                launch_arguments={
                    "service_name": "/robot_motion/check_collision",
                    "joint_group": LaunchConfiguration("collision_joint_group"),
                    "joint_state_topic": LaunchConfiguration("model_joint_states_topic"),
                }.items(),
            ),
        ]
    )
