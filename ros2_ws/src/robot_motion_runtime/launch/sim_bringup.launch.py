from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_launch(package_name: str, launch_file: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument("execution_action_name", default_value="/alfa_execution/execute_joint_trajectory"),
            DeclareLaunchArgument("initial_updown", default_value="0.0"),
            DeclareLaunchArgument("joint_state_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("start_rerun", default_value="true"),
            DeclareLaunchArgument("rerun_rate_hz", default_value="15.0"),
            IncludeLaunchDescription(
                package_launch("alfa_robot_moveit_config", "static_virtual_joint_tfs.launch.py")
            ),
            IncludeLaunchDescription(
                package_launch("alfa_robot_moveit_config", "rsp.launch.py")
            ),
            Node(
                package="robot_motion_runtime",
                executable="kinematic_sim_executor_node",
                name="kinematic_sim_executor",
                output="screen",
                parameters=[
                    {
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "action_name": LaunchConfiguration("execution_action_name"),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("joint_state_rate_hz"), value_type=float
                        ),
                        "initial_updown": ParameterValue(
                            LaunchConfiguration("initial_updown"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="alfa_robot_rerun",
                executable="rerun_joint_state_viewer_node",
                name="rerun_joint_state_viewer",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rerun")),
                parameters=[
                    {
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "log_rate_hz": ParameterValue(
                            LaunchConfiguration("rerun_rate_hz"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
