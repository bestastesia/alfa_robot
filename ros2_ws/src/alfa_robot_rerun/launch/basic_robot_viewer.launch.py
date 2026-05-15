from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    description_package = LaunchConfiguration("description_package")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    spawn = LaunchConfiguration("spawn")
    recording_path = LaunchConfiguration("recording_path")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "alfa_robot.urdf.xacro"]),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("description_package", default_value="alfa_robot_description"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("spawn", default_value="true"),
            DeclareLaunchArgument("recording_path", default_value=""),
            Node(
                package="alfa_robot_rerun",
                executable="basic_robot_viewer",
                name="alfa_rerun_basic_robot_viewer",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description_content,
                        "joint_states_topic": joint_states_topic,
                        "spawn": spawn,
                        "recording_path": recording_path,
                    }
                ],
            ),
        ]
    )
