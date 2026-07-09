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
            DeclareLaunchArgument("execution_action_name", default_value="/alfa_execution/execute_joint_trajectory"),
            DeclareLaunchArgument("execute_wait_for_action_timeout_s", default_value="10.0"),
            DeclareLaunchArgument("execute_wait_for_goal_acceptance", default_value="false"),
            DeclareLaunchArgument("execute_resample_before_forward", default_value="true"),
            DeclareLaunchArgument("execute_resample_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("task_service_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("plan_check_collision", default_value="true"),
            DeclareLaunchArgument("ik_root_samples", default_value="720"),
            DeclareLaunchArgument("ik_default_max_solutions", default_value="8"),
            DeclareLaunchArgument("collision_joint_group", default_value="dual_arm_with_base"),
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
                    "execute_resample_before_forward": LaunchConfiguration("execute_resample_before_forward"),
                    "execute_resample_rate_hz": LaunchConfiguration("execute_resample_rate_hz"),
                    "task_service_timeout_s": LaunchConfiguration("task_service_timeout_s"),
                    "plan_check_collision": LaunchConfiguration("plan_check_collision"),
                    "collision_service_name": "/robot_motion/check_collision",
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
                }.items(),
            ),
        ]
    )
