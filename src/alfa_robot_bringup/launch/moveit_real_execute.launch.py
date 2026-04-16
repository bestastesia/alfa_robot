from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package", default_value="alfa_robot_description"),
        DeclareLaunchArgument("description_file", default_value="alfa_robot.urdf.xacro"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("real_hardware_plugin", default_value="alfa_robot_hardware/AlfaRobotHW"),
        DeclareLaunchArgument("canopen_profile_velocity", default_value="50000"),
        DeclareLaunchArgument("canopen_profile_accel", default_value="50000"),
        DeclareLaunchArgument("run_rviz", default_value="false"),
        DeclareLaunchArgument("auto_execute", default_value="true"),
        DeclareLaunchArgument("planning_group", default_value="left_arm"),
        DeclareLaunchArgument("velocity_scale", default_value="0.2"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.2"),
        DeclareLaunchArgument("left_x", default_value="0.357"),
        DeclareLaunchArgument("left_y", default_value="-0.705"),
        DeclareLaunchArgument("left_z", default_value="0.684"),
        DeclareLaunchArgument("left_qx", default_value="-0.502"),
        DeclareLaunchArgument("left_qy", default_value="0.502"),
        DeclareLaunchArgument("left_qz", default_value="-0.498"),
        DeclareLaunchArgument("left_qw", default_value="0.498"),
        DeclareLaunchArgument("right_x", default_value="0.357"),
        DeclareLaunchArgument("right_y", default_value="0.705"),
        DeclareLaunchArgument("right_z", default_value="0.684"),
        DeclareLaunchArgument("right_qx", default_value="-0.498"),
        DeclareLaunchArgument("right_qy", default_value="-0.502"),
        DeclareLaunchArgument("right_qz", default_value="0.502"),
        DeclareLaunchArgument("right_qw", default_value="0.498"),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    real_hardware_plugin = LaunchConfiguration("real_hardware_plugin")
    canopen_profile_velocity = LaunchConfiguration("canopen_profile_velocity")
    canopen_profile_accel = LaunchConfiguration("canopen_profile_accel")
    run_rviz = LaunchConfiguration("run_rviz")
    auto_execute = LaunchConfiguration("auto_execute")
    planning_group = LaunchConfiguration("planning_group")
    velocity_scale = LaunchConfiguration("velocity_scale")
    acceleration_scale = LaunchConfiguration("acceleration_scale")
    left_x = LaunchConfiguration("left_x")
    left_y = LaunchConfiguration("left_y")
    left_z = LaunchConfiguration("left_z")
    left_qx = LaunchConfiguration("left_qx")
    left_qy = LaunchConfiguration("left_qy")
    left_qz = LaunchConfiguration("left_qz")
    left_qw = LaunchConfiguration("left_qw")
    right_x = LaunchConfiguration("right_x")
    right_y = LaunchConfiguration("right_y")
    right_z = LaunchConfiguration("right_z")
    right_qx = LaunchConfiguration("right_qx")
    right_qy = LaunchConfiguration("right_qy")
    right_qz = LaunchConfiguration("right_qz")
    right_qw = LaunchConfiguration("right_qw")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=false ",
            "mock_sensor_commands:=false ",
            "real_hardware_plugin:=",
            real_hardware_plugin,
            " ",
            "canopen_profile_velocity:=",
            canopen_profile_velocity,
            " ",
            "canopen_profile_accel:=",
            canopen_profile_accel,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("alfa_robot_bringup"),
            "config",
            "alfa_robot_moveit_real_controllers.yaml",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controllers_yaml],
        remappings=[("~/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    torso_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["torso_group_controller", "-c", "/controller_manager"],
    )
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_moveit_config"), "launch", "move_group.launch.py"]
            )
        )
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        condition=IfCondition(run_rviz),
    )
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_moveit_config"), "launch", "static_virtual_joint_tfs.launch.py"]
            )
        )
    )

    path_execute_node = Node(
        package="alfa_robot_moveit_config",
        executable="path_node.py",
        output="screen",
        arguments=[
            "--planning-group",
            planning_group,
            "--pose-left",
            left_x,
            left_y,
            left_z,
            left_qx,
            left_qy,
            left_qz,
            left_qw,
            "--pose-right",
            right_x,
            right_y,
            right_z,
            right_qx,
            right_qy,
            right_qz,
            right_qw,
            "--velocity-scale",
            velocity_scale,
            "--acceleration-scale",
            acceleration_scale,
        ],
        condition=IfCondition(auto_execute),
    )

    delayed_control_node = TimerAction(period=2.0, actions=[control_node])
    delayed_jsb = TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])
    delayed_torso_controller = TimerAction(period=8.0, actions=[torso_group_controller_spawner])
    delayed_left_controller = TimerAction(period=11.0, actions=[left_arm_controller_spawner])
    delayed_right_controller = TimerAction(period=14.0, actions=[right_arm_controller_spawner])
    delayed_moveit = TimerAction(
        period=18.0,
        actions=[static_tf_launch, move_group_launch, moveit_rviz_launch],
    )
    delayed_path_execute = TimerAction(
        period=26.0,
        actions=[path_execute_node],
        condition=IfCondition(auto_execute),
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            delayed_control_node,
            delayed_jsb,
            delayed_torso_controller,
            delayed_left_controller,
            delayed_right_controller,
            delayed_moveit,
            delayed_path_execute,
        ]
    )
