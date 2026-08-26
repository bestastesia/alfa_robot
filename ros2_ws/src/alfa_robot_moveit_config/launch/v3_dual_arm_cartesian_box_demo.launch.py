from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "alfa_robot", package_name="alfa_robot_moveit_config"
    ).to_moveit_configs()

    start_rviz = LaunchConfiguration("start_rviz")
    start_rerun = LaunchConfiguration("start_rerun")
    auto_run_once = LaunchConfiguration("auto_run_once")
    recording_path = LaunchConfiguration("rerun_recording_path")
    motion_mode = LaunchConfiguration("motion_mode")
    box_size = LaunchConfiguration("box_size")
    initial_target_roll_deg = LaunchConfiguration("initial_target_roll_deg")
    angular_step_deg = LaunchConfiguration("angular_step_deg")
    maximum_joint_step_deg = LaunchConfiguration("maximum_joint_step_deg")
    rviz_config = LaunchConfiguration("rviz_config")
    initial_box_x = LaunchConfiguration("initial_box_x")
    initial_box_y = LaunchConfiguration("initial_box_y")
    initial_box_z = LaunchConfiguration("initial_box_z")
    target_offset_x = LaunchConfiguration("target_offset_x")
    target_offset_y = LaunchConfiguration("target_offset_y")
    target_offset_z = LaunchConfiguration("target_offset_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_rerun", default_value="true"),
            DeclareLaunchArgument("auto_run_once", default_value="false"),
            DeclareLaunchArgument("rerun_recording_path", default_value=""),
            DeclareLaunchArgument("motion_mode", default_value="translate"),
            DeclareLaunchArgument("box_size", default_value="0.40"),
            DeclareLaunchArgument("initial_target_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("angular_step_deg", default_value="2.0"),
            DeclareLaunchArgument("maximum_joint_step_deg", default_value="12.0"),
            DeclareLaunchArgument("initial_box_x", default_value="0.73"),
            DeclareLaunchArgument("initial_box_y", default_value="0.0"),
            DeclareLaunchArgument("initial_box_z", default_value="0.55"),
            DeclareLaunchArgument("target_offset_x", default_value="-0.12"),
            DeclareLaunchArgument("target_offset_y", default_value="0.0"),
            DeclareLaunchArgument("target_offset_z", default_value="0.0"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    moveit_config.package_path
                    / "config"
                    / "v3_dual_arm_cartesian_box_demo.rviz"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
                remappings=[
                    (
                        "/joint_states",
                        "/v3_dual_arm_cartesian_box_demo/joint_states",
                    )
                ],
            ),
            Node(
                package="alfa_robot_moveit_config",
                executable="v3_dual_arm_cartesian_box_demo",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "auto_run_once": auto_run_once,
                        "motion_mode": motion_mode,
                        "box_size": ParameterValue(box_size, value_type=float),
                        "initial_target_roll_deg": ParameterValue(
                            initial_target_roll_deg, value_type=float
                        ),
                        "angular_step_deg": ParameterValue(
                            angular_step_deg, value_type=float
                        ),
                        "maximum_joint_step_deg": ParameterValue(
                            maximum_joint_step_deg, value_type=float
                        ),
                        "initial_box_x": ParameterValue(initial_box_x, value_type=float),
                        "initial_box_y": ParameterValue(initial_box_y, value_type=float),
                        "initial_box_z": ParameterValue(initial_box_z, value_type=float),
                        "initial_target_offset_x": ParameterValue(
                            target_offset_x, value_type=float
                        ),
                        "initial_target_offset_y": ParameterValue(
                            target_offset_y, value_type=float
                        ),
                        "initial_target_offset_z": ParameterValue(
                            target_offset_z, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="alfa_robot_rerun",
                executable="v3_dual_arm_cartesian_box_viewer",
                output="screen",
                parameters=[
                    {
                        "recording_path": recording_path,
                        "spawn_viewer": True,
                    }
                ],
                condition=IfCondition(start_rerun),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
