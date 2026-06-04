from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "feedback_topic_1",
            default_value="/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
            description="Primary RViz MoveIt interactive marker feedback topic.",
        ),
        DeclareLaunchArgument(
            "feedback_topic_2",
            default_value="/rviz2_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
            description="Secondary RViz MoveIt interactive marker feedback topic.",
        ),
        DeclareLaunchArgument(
            "feedback_topic_3",
            default_value="/robot_interaction_interactive_marker_topic/feedback",
            description="Fallback interactive marker feedback topic.",
        ),
        DeclareLaunchArgument(
            "feedback_topic_4",
            default_value="/move_marker/feedback",
            description="MoveIt RobotInteraction default feedback topic.",
        ),
        DeclareLaunchArgument("target_frame", default_value=""),
        DeclareLaunchArgument("log_every_update", default_value="true"),
        DeclareLaunchArgument("publish_markers", default_value="true"),
        DeclareLaunchArgument("marker_topic", default_value="/rviz_dual_goal_pose_monitor/markers"),
    ]

    monitor = Node(
        package="alfa_robot_moveit_config",
        executable="rviz_dual_goal_pose_monitor.py",
        name="rviz_dual_goal_pose_monitor",
        output="screen",
        parameters=[{
            "feedback_topics": [
                LaunchConfiguration("feedback_topic_1"),
                LaunchConfiguration("feedback_topic_2"),
                LaunchConfiguration("feedback_topic_3"),
                LaunchConfiguration("feedback_topic_4"),
            ],
            "target_frame": LaunchConfiguration("target_frame"),
            "log_every_update": LaunchConfiguration("log_every_update"),
            "publish_markers": LaunchConfiguration("publish_markers"),
            "marker_topic": LaunchConfiguration("marker_topic"),
        }],
    )

    return LaunchDescription(declared_arguments + [monitor])
