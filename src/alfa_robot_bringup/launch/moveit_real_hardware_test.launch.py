from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package", default_value="alfa_robot_description"),
        DeclareLaunchArgument("description_file", default_value="alfa_robot.urdf.xacro"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("real_hardware_plugin", default_value="alfa_robot_hardware/AlfaRobotHW"),
        DeclareLaunchArgument("canopen_profile_velocity", default_value="50000"),
        DeclareLaunchArgument("canopen_profile_accel", default_value="50000"),
        DeclareLaunchArgument("auto_run_test", default_value="false"),
        DeclareLaunchArgument("group_name", default_value="right_arm"),
        DeclareLaunchArgument("target_x", default_value="0.357"),
        DeclareLaunchArgument("target_y", default_value="-0.705"),
        DeclareLaunchArgument("target_z", default_value="0.684"),
        DeclareLaunchArgument("target_qx", default_value="-0.502"),
        DeclareLaunchArgument("target_qy", default_value="0.502"),
        DeclareLaunchArgument("target_qz", default_value="-0.498"),
        DeclareLaunchArgument("target_qw", default_value="0.498"),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    real_hardware_plugin = LaunchConfiguration("real_hardware_plugin")
    canopen_profile_velocity = LaunchConfiguration("canopen_profile_velocity")
    canopen_profile_accel = LaunchConfiguration("canopen_profile_accel")
    auto_run_test = LaunchConfiguration("auto_run_test")
    group_name = LaunchConfiguration("group_name")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    target_qx = LaunchConfiguration("target_qx")
    target_qy = LaunchConfiguration("target_qy")
    target_qz = LaunchConfiguration("target_qz")
    target_qw = LaunchConfiguration("target_qw")

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
        [FindPackageShare("alfa_robot_moveit_config"), "config", "ros2_controllers.yaml"]
    )

    # 加载 MoveIt 配置（与 test_path.launch.py 相同的方式）
    moveit_config = MoveItConfigsBuilder(
        "alfa_robot", package_name="alfa_robot_moveit_config"
    ).to_moveit_configs()

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
        )
    )
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_moveit_config"), "launch", "static_virtual_joint_tfs.launch.py"]
            )
        )
    )

    # ===== 仿照 test_path.launch.py 的逻辑 =====
    # trajectory_executor: 订阅 /joint_trajectory，转发到控制器的 FollowJointTrajectory action
    trajectory_executor_node = Node(
        package="alfa_robot_moveit_config",
        executable="trajectory_executor",
        output="screen",
        condition=IfCondition(auto_run_test),
    )

    # path 规划节点: 使用 MoveGroupInterface 规划轨迹，发布到 /joint_trajectory
    # 参数通过 MoveItConfigsBuilder 加载（与 test_path.launch.py 一致）
    # 目标位姿通过命令行参数传入: <group_name> <x> <y> <z> <qx> <qy> <qz> <qw>
    path_planning_node = Node(
        package="alfa_robot_moveit_config",
        executable="path",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=[
            group_name,
            target_x, target_y, target_z,
            target_qx, target_qy, target_qz, target_qw,
        ],
        condition=IfCondition(auto_run_test),
    )

    delayed_control_node = TimerAction(period=2.0, actions=[control_node])
    delayed_jsb = TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])
    delayed_torso_controller = TimerAction(period=8.0, actions=[torso_group_controller_spawner])
    delayed_left_controller = TimerAction(period=11.0, actions=[left_arm_controller_spawner])
    delayed_right_controller = TimerAction(period=14.0, actions=[right_arm_controller_spawner])
    delayed_moveit = TimerAction(
        period=20.0,
        actions=[static_tf_launch, move_group_launch, moveit_rviz_launch],
    )
    # trajectory_executor 先启动，等待接收轨迹
    delayed_trajectory_executor = TimerAction(
        period=26.0,
        actions=[trajectory_executor_node],
        condition=IfCondition(auto_run_test),
    )
    # path 规划节点稍后启动，确保 move_group 和 trajectory_executor 都已就绪
    delayed_path_node = TimerAction(
        period=30.0,
        actions=[path_planning_node],
        condition=IfCondition(auto_run_test),
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
            delayed_trajectory_executor,
            delayed_path_node,
        ]
    )
