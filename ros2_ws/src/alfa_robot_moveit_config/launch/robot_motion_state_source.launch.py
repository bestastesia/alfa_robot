from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_joint_states", default_value="/joint_states"),
            DeclareLaunchArgument("output_state_topic", default_value="/robot_motion/state"),
            DeclareLaunchArgument("source", default_value="joint_states"),
            DeclareLaunchArgument("authoritative", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="world"),
            DeclareLaunchArgument("scene_id", default_value=""),
            DeclareLaunchArgument("state_id_prefix", default_value=""),
            DeclareLaunchArgument("publish_period_s", default_value="0.0"),
            Node(
                package="alfa_robot_moveit_config",
                executable="robot_motion_state_source.py",
                name="robot_motion_state_source",
                output="screen",
                parameters=[
                    {
                        "input_joint_states": LaunchConfiguration("input_joint_states"),
                        "output_state_topic": LaunchConfiguration("output_state_topic"),
                        "source": LaunchConfiguration("source"),
                        "authoritative": ParameterValue(
                            LaunchConfiguration("authoritative"), value_type=bool
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "scene_id": LaunchConfiguration("scene_id"),
                        "state_id_prefix": LaunchConfiguration("state_id_prefix"),
                        "publish_period_s": ParameterValue(
                            LaunchConfiguration("publish_period_s"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
