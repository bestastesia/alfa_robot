"""
MoveIt 实机执行启动文件

功能：
  1. 启动 ros2_control_node 进行硬件控制
  2. 启动 MoveIt move_group 进行规划
  3. 启动控制器 (joint_state_broadcaster, dual arm controller)
  4. 可选：启动 RViz 可视化
  5. 可选：自动执行预设轨迹

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
        DeclareLaunchArgument("run_rviz", default_value="false"),
        DeclareLaunchArgument("auto_execute", default_value="false"),
        DeclareLaunchArgument("planning_group", default_value="dual_arm_with_base"),
        DeclareLaunchArgument("velocity_scale", default_value="0.2"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.2"),
        # 左臂目标位姿
        DeclareLaunchArgument("left_x", default_value="0.357"),
        DeclareLaunchArgument("left_y", default_value="-0.705"),
        DeclareLaunchArgument("left_z", default_value="0.684"),
        DeclareLaunchArgument("left_qx", default_value="-0.502"),
        DeclareLaunchArgument("left_qy", default_value="0.502"),
        DeclareLaunchArgument("left_qz", default_value="-0.498"),
        DeclareLaunchArgument("left_qw", default_value="0.498"),
        # 右臂目标位姿
        DeclareLaunchArgument("right_x", default_value="0.357"),
        DeclareLaunchArgument("right_y", default_value="0.705"),
        DeclareLaunchArgument("right_z", default_value="0.684"),
        DeclareLaunchArgument("right_qx", default_value="-0.498"),
        DeclareLaunchArgument("right_qy", default_value="-0.502"),
        DeclareLaunchArgument("right_qz", default_value="0.502"),
        DeclareLaunchArgument("right_qw", default_value="0.498"),
    ]

    # LaunchConfiguration references
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
    # 我们不需要在顶层重复启动

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

    # 自动执行节点
    path_execute_node = Node(
        package="alfa_robot_moveit_config",
        executable="path_node.py",
        name="path_execute_node",
        output="screen",
        arguments=[
            "--planning-group", planning_group,
            "--pose-left", left_x, left_y, left_z, left_qx, left_qy, left_qz, left_qw,
            "--pose-right", right_x, right_y, right_z, right_qx, right_qy, right_qz, right_qw,
            "--velocity-scale", velocity_scale,
            "--acceleration-scale", acceleration_scale,
        ],
        condition=IfCondition(auto_execute),
    )

    # ===== 生命周期管理 =====
    #
    # 启动顺序：
    #   control_node → (3s) → joint_state_broadcaster
    #   joint_state_broadcaster → dual_arm_controller
    #   dual_arm_controller → MoveIt (move_group + static_tf + rviz)
    #   MoveIt → path_execute_node

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

    # 4. MoveIt 启动后，延迟启动 path_execute_node
    delay_path_execute = TimerAction(
        period=8.0,  # MoveIt 启动需要时间
        actions=[path_execute_node],
        condition=IfCondition(auto_execute),
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
            delay_path_execute,
        ]
    )
