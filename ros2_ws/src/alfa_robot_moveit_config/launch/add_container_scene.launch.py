from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("mode", default_value="add"),
        DeclareLaunchArgument("scene", default_value="container"),
        DeclareLaunchArgument("object_prefix", default_value="container"),
        DeclareLaunchArgument("origin_xyz", default_value="1.4,0.0,0.0"),
        DeclareLaunchArgument("container_size_xyz", default_value="1.6,1.2,1.2"),
        DeclareLaunchArgument("wall_thickness", default_value="0.04"),
        DeclareLaunchArgument("floor_thickness", default_value="0.04"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("include_ceiling", default_value="false"),
        DeclareLaunchArgument("box_id", default_value="box_obstacle"),
        DeclareLaunchArgument("box_center_xyz", default_value="1.0,0.0,0.4"),
        DeclareLaunchArgument("box_size_xyz", default_value="0.4,0.4,0.8"),
        DeclareLaunchArgument("service_name", default_value="/apply_planning_scene"),
        DeclareLaunchArgument("timeout_sec", default_value="5.0"),
    ]

    scene_node = Node(
        package="alfa_robot_moveit_config",
        executable="add_container_scene.py",
        name="add_container_scene",
        output="screen",
        parameters=[{
            "frame_id": LaunchConfiguration("frame_id"),
            "mode": LaunchConfiguration("mode"),
            "scene": LaunchConfiguration("scene"),
            "object_prefix": LaunchConfiguration("object_prefix"),
            "origin_xyz": LaunchConfiguration("origin_xyz"),
            "container_size_xyz": LaunchConfiguration("container_size_xyz"),
            "wall_thickness": LaunchConfiguration("wall_thickness"),
            "floor_thickness": LaunchConfiguration("floor_thickness"),
            "yaw": LaunchConfiguration("yaw"),
            "include_ceiling": LaunchConfiguration("include_ceiling"),
            "box_id": LaunchConfiguration("box_id"),
            "box_center_xyz": LaunchConfiguration("box_center_xyz"),
            "box_size_xyz": LaunchConfiguration("box_size_xyz"),
            "service_name": LaunchConfiguration("service_name"),
            "timeout_sec": LaunchConfiguration("timeout_sec"),
        }],
    )

    return LaunchDescription(declared_arguments + [scene_node])
