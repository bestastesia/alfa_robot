from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8766"),
            DeclareLaunchArgument("subscribe_joint_states", default_value="true"),
            DeclareLaunchArgument("model_joint_states_topic", default_value="/robot_motion/model_joint_states"),
            DeclareLaunchArgument("state_publish_period_s", default_value="0.2"),
            DeclareLaunchArgument("execute_forward_action", default_value="true"),
            DeclareLaunchArgument(
                "execution_action_name",
                default_value="/dual_arm_jtc/follow_joint_trajectory",
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
            DeclareLaunchArgument("solve_arm_ik_service_name", default_value="/robot_motion/solve_arm_ik"),
            DeclareLaunchArgument("run_dual_arm_pose_task_service_name", default_value="/robot_motion/run_dual_arm_pose_task"),
            DeclareLaunchArgument("run_dual_grasp_task_service_name", default_value="/robot_motion/run_dual_grasp_task"),
            DeclareLaunchArgument("task_receipt_topic", default_value="/robot_motion/task_receipt"),
            DeclareLaunchArgument("default_fixed_updown", default_value="0.0"),
            DeclareLaunchArgument("default_candidate_limit", default_value="8"),
            DeclareLaunchArgument("default_planning_mode", default_value="shortcut"),
            DeclareLaunchArgument("plan_trajectory_duration_s", default_value="0.0"),
            DeclareLaunchArgument("plan_trajectory_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("plan_max_joint_step_deg", default_value="4.5"),
            DeclareLaunchArgument("plan_max_updown_step_m", default_value="0.01"),
            DeclareLaunchArgument("publish_empty_scene_on_start", default_value="true"),
            DeclareLaunchArgument("plan_check_collision", default_value="false"),
            DeclareLaunchArgument("collision_service_name", default_value="/robot_motion/check_collision"),
            Node(
                package="robot_motion_runtime",
                executable="motion_state_source_node",
                name="motion_state_source",
                output="screen",
                parameters=[
                    {
                        "subscribe_joint_states": ParameterValue(
                            LaunchConfiguration("subscribe_joint_states"), value_type=bool
                        ),
                        "model_joint_states_topic": LaunchConfiguration("model_joint_states_topic"),
                        "publish_model_joint_states": True,
                        "publish_period_s": ParameterValue(
                            LaunchConfiguration("state_publish_period_s"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="motion_scene_source_node",
                name="motion_scene_source",
                output="screen",
                parameters=[
                    {
                        "publish_empty_scene_on_start": ParameterValue(
                            LaunchConfiguration("publish_empty_scene_on_start"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="dual_arm_ik_candidate_service_node",
                name="dual_arm_ik_candidate_service",
                output="screen",
                parameters=[
                    {
                        "solve_arm_ik_service": LaunchConfiguration("solve_arm_ik_service_name"),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="box_pair_task_adapter_node",
                name="box_pair_task_adapter",
                output="screen",
                parameters=[
                    {
                        "pose_task_service": LaunchConfiguration("run_dual_arm_pose_task_service_name"),
                        "service_timeout_s": ParameterValue(
                            LaunchConfiguration("task_service_timeout_s"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="dual_grasp_task_adapter_node",
                name="dual_grasp_task_adapter",
                output="screen",
                parameters=[
                    {
                        "service_name": LaunchConfiguration("run_dual_grasp_task_service_name"),
                        "pose_task_service": LaunchConfiguration("run_dual_arm_pose_task_service_name"),
                        "receipt_topic": LaunchConfiguration("task_receipt_topic"),
                        "service_timeout_s": ParameterValue(
                            LaunchConfiguration("task_service_timeout_s"), value_type=float
                        ),
                        "default_fixed_updown": ParameterValue(
                            LaunchConfiguration("default_fixed_updown"), value_type=float
                        ),
                        "default_candidate_limit": ParameterValue(
                            LaunchConfiguration("default_candidate_limit"), value_type=int
                        ),
                        "default_planning_mode": LaunchConfiguration("default_planning_mode"),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="plan_extract_service_node",
                name="plan_extract_service",
                output="screen",
                parameters=[
                    {
                        "check_collision": ParameterValue(
                            LaunchConfiguration("plan_check_collision"), value_type=bool
                        ),
                        "collision_service_name": LaunchConfiguration("collision_service_name"),
                        "trajectory_duration_s": ParameterValue(
                            LaunchConfiguration("plan_trajectory_duration_s"), value_type=float
                        ),
                        "trajectory_rate_hz": ParameterValue(
                            LaunchConfiguration("plan_trajectory_rate_hz"), value_type=float
                        ),
                        "max_joint_step_deg": ParameterValue(
                            LaunchConfiguration("plan_max_joint_step_deg"), value_type=float
                        ),
                        "max_updown_step_m": ParameterValue(
                            LaunchConfiguration("plan_max_updown_step_m"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="plan_loaded_service_node",
                name="plan_loaded_service",
                output="screen",
                parameters=[
                    {
                        "check_collision": ParameterValue(
                            LaunchConfiguration("plan_check_collision"), value_type=bool
                        ),
                        "collision_service_name": LaunchConfiguration("collision_service_name"),
                        "trajectory_duration_s": ParameterValue(
                            LaunchConfiguration("plan_trajectory_duration_s"), value_type=float
                        ),
                        "trajectory_rate_hz": ParameterValue(
                            LaunchConfiguration("plan_trajectory_rate_hz"), value_type=float
                        ),
                        "max_joint_step_deg": ParameterValue(
                            LaunchConfiguration("plan_max_joint_step_deg"), value_type=float
                        ),
                        "max_updown_step_m": ParameterValue(
                            LaunchConfiguration("plan_max_updown_step_m"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="execute_trajectory_service_node",
                name="execute_trajectory_service",
                output="screen",
                parameters=[
                    {
                        "action_name": LaunchConfiguration("execution_action_name"),
                        "forward_action": ParameterValue(
                            LaunchConfiguration("execute_forward_action"), value_type=bool
                        ),
                        "wait_for_action_timeout_s": ParameterValue(
                            LaunchConfiguration("execute_wait_for_action_timeout_s"), value_type=float
                        ),
                        "wait_for_goal_acceptance": ParameterValue(
                            LaunchConfiguration("execute_wait_for_goal_acceptance"), value_type=bool
                        ),
                        "wait_for_result": ParameterValue(
                            LaunchConfiguration("execute_wait_for_result"), value_type=bool
                        ),
                        "wait_for_result_timeout_s": ParameterValue(
                            LaunchConfiguration("execute_wait_for_result_timeout_s"), value_type=float
                        ),
                        "resample_before_forward": ParameterValue(
                            LaunchConfiguration("execute_resample_before_forward"), value_type=bool
                        ),
                        "resample_rate_hz": ParameterValue(
                            LaunchConfiguration("execute_resample_rate_hz"), value_type=float
                        ),
                        "adapt_to_hardware_joint_order": ParameterValue(
                            LaunchConfiguration("execute_adapt_to_hardware_joint_order"), value_type=bool
                        ),
                        "hold_missing_from_joint_states": ParameterValue(
                            LaunchConfiguration("execute_hold_missing_from_joint_states"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="motion_task_orchestrator_node",
                name="motion_task_orchestrator",
                output="screen",
                parameters=[
                    {
                        "service_timeout_s": ParameterValue(
                            LaunchConfiguration("task_service_timeout_s"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="robot_motion_runtime",
                executable="motion_runtime_dashboard_node",
                name="motion_runtime_dashboard",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("dashboard_host"),
                        "port": ParameterValue(
                            LaunchConfiguration("dashboard_port"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
