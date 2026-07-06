"""
MoveIt 实机测试启动文件

功能：
  1. 启动 ros2_control_node 进行硬件控制
  2. 启动 MoveIt move_group 进行规划
  3. 启动控制器 (joint_state_broadcaster, dual arm controller)
  4. 可选：自动运行测试轨迹

注意：
  - move_group.launch.py 内部会启动 robot_state_publisher
  - 此文件不再重复启动 robot_state_publisher，避免节点冲突
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
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
        DeclareLaunchArgument("group_name", default_value="left_arm"),
        # 目标位姿
        DeclareLaunchArgument("target_x", default_value="0.357"),
        DeclareLaunchArgument("target_y", default_value="-0.705"),
        DeclareLaunchArgument("target_z", default_value="0.684"),
        DeclareLaunchArgument("target_qx", default_value="-0.502"),
        DeclareLaunchArgument("target_qy", default_value="0.502"),
        DeclareLaunchArgument("target_qz", default_value="-0.498"),
        DeclareLaunchArgument("target_qw", default_value="0.498"),
    ]

    # LaunchConfiguration references
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

    # URDF via xacro (实机模式)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "prefix:=", prefix,
            " ",
            "use_mock_hardware:=false ",
            "mock_sensor_commands:=false ",
            "real_hardware_plugin:=", real_hardware_plugin,
            " ",
            "canopen_profile_velocity:=", canopen_profile_velocity,
            " ",
            "canopen_profile_accel:=", canopen_profile_accel,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # 实机控制器配置
    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("alfa_robot_bringup"),
            "config",
            "alfa_robot_moveit_real_controllers.yaml",
        ]
    )

    # 加载 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        "alfa_robot", package_name="alfa_robot_moveit_config"
    ).to_moveit_configs()

    # ===== 节点定义 =====

    # ros2_control_node - 硬件控制
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control_node",
        output="both",
        parameters=[controllers_yaml],
        remappings=[("~/robot_description", "/robot_description")],
    )

    # 控制器 spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="both",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    dual_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="dual_arm_controller_spawner",
        output="both",
        arguments=["dual_arm_controller", "-c", "/controller_manager"],
    )

    # ===== MoveIt Launch =====
    # 注意：move_group.launch.py 内部会启动 robot_state_publisher

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments={
            "robot_description": robot_description_content,
        }.items(),
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

    # ===== 测试节点 =====
    # trajectory_executor: 订阅 /joint_trajectory，转发到控制器的 FollowJointTrajectory action
    trajectory_executor_node = Node(
        package="alfa_robot_moveit_config",
        executable="trajectory_executor",
        name="trajectory_executor",
        output="screen",
        condition=IfCondition(auto_run_test),
    )

    # path 规划节点: 使用 MoveGroupInterface 规划轨迹，发布到 /joint_trajectory
    path_planning_node = Node(
        package="alfa_robot_moveit_config",
        executable="path",
        name="path_planning_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=[
            group_name,
            target_x, target_y, target_z,
            target_qx, target_qy, target_qz, target_qw,
        ],
        condition=IfCondition(auto_run_test),
    )

    # ===== 生命周期管理 =====
    #
    # 启动顺序：
    #   control_node → (3s) → joint_state_broadcaster
    #   joint_state_broadcaster → dual_arm_controller
    #   dual_arm_controller → MoveIt (move_group + static_tf + rviz)
    #   MoveIt → trajectory_executor → path_planning_node

    # 1. control_node 启动后，延迟启动 joint_state_broadcaster
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    # 2. joint_state_broadcaster 完成后，启动 dual_arm_controller
    delay_dual_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dual_arm_controller_spawner],
        )
    )

    # 3. dual_arm_controller 完成后，启动 MoveIt
    delay_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dual_arm_controller_spawner,
            on_exit=[
                static_tf_launch,
                move_group_launch,
                moveit_rviz_launch,
            ],
        )
    )

    # 4. MoveIt 启动后，延迟启动测试节点
    delay_trajectory_executor = TimerAction(
        period=8.0,
        actions=[trajectory_executor_node],
        condition=IfCondition(auto_run_test),
    )

    delay_path_node = TimerAction(
        period=12.0,
        actions=[path_planning_node],
        condition=IfCondition(auto_run_test),
    )

    return LaunchDescription(
        declared_arguments
        + [
            # 核心节点
            control_node,

            # 生命周期事件处理器
            delay_jsb,
            delay_dual_arm,
            delay_moveit,
            delay_trajectory_executor,
            delay_path_node,
        ]
    )
