# alfa_robot_bringup 分层重构 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 alfa_robot_bringup 重构为 franka_bringup 风格的三层架构：原子基座 / 场景编排 / 配置工具层，同时修复已确认的 bug。

**Architecture:**
- `base.launch.py` — 单机器人原子基座（robot_description / RSP / ros2_control_node / JSB / 可选默认控制器），事件驱动启动顺序
- `launch_utils.py` — 配置装载与校验工具（load_yaml / resolve_controller_file / validate_prefix_unique）
- 场景层（slider / moveit_test）只做 `IncludeLaunchDescription(base)` + 增量附加节点
- 三套命名清晰的 controllers yaml（default / slider / moveit）

**Tech Stack:** ROS 2 Humble, ros2_control, MoveIt 2, Python 3.10, launch / launch_ros

---

## 文件结构

| 操作 | 路径 | 职责 |
|------|------|------|
| 新建 | `alfa_robot_bringup/alfa_robot_bringup/__init__.py` | Python 包入口 |
| 新建 | `alfa_robot_bringup/alfa_robot_bringup/launch_utils.py` | load_yaml / resolve_controller_file / validate_prefix_unique |
| 新建 | `alfa_robot_bringup/config/controllers.default.yaml` | 4 个 JTC 控制器（torso/left_arm/right_arm/plate）基础配置 |
| 新建 | `alfa_robot_bringup/config/controllers.slider.yaml` | slider 场景：同上 + update_rate |
| 新建 | `alfa_robot_bringup/config/controllers.moveit.yaml` | MoveIt 场景控制器（与 moveit_config 对齐） |
| 新建 | `alfa_robot_bringup/launch/base.launch.py` | 原子基座，替代现 alfa_robot.launch.py 的核心逻辑 |
| 修改 | `alfa_robot_bringup/launch/alfa_robot.launch.py` | 精简为 base.launch.py 的薄包装（向后兼容入口） |
| 修改 | `alfa_robot_bringup/launch/slider_control_real_hw.launch.py` | include base + 启动全部 4 个控制器 + homing_node |
| 修改 | `alfa_robot_bringup/launch/moveit_real_hardware_test.launch.py` | include base + 事件驱动 + MoveIt |
| 修改 | `alfa_robot_bringup/scripts/joint_states_to_controller_bridge.py` | 修复 return→continue bug |
| 修改 | `alfa_robot_bringup/scripts/homing_node.py` | 修复连发逻辑（加间隔定时器） |
| 修改 | `alfa_robot_bringup/CMakeLists.txt` | 安装 Python 模块 |
| 删除 | `alfa_robot_bringup/scripts/debug_mesh_path.py` | Agent 遗留调试文件 |
| 删除 | `alfa_robot_bringup/config/alfa_robot_controllers.yaml` | 旧 ForwardCommandController，已废弃 |
| 删除 | `alfa_robot_bringup/config/alfa_robot_controllers_can.yaml` | 旧 CAN 配置，已废弃 |
| 修改 | `alfa_robot_bringup/config/test_goal_publishers_config.yaml` | 更新控制器名称 |

---

## Task 1: 修复 joint_states_to_controller_bridge.py 的 return → continue bug

**Files:**
- Modify: `alfa_robot_bringup/scripts/joint_states_to_controller_bridge.py:69`

- [ ] **Step 1: 修复 bug**

将第 69 行的 `return` 改为 `continue`：

```python
    def _joint_states_callback(self, msg: JointState):
        for controller, (pub, joints) in self._publishers.items():
            positions = []
            for name in joints:
                try:
                    idx = msg.name.index(name)
                    positions.append(msg.position[idx])
                except ValueError:
                    continue  # joint not yet available, skip this controller
```

Wait — the `continue` needs to skip the *outer* `for controller` loop, not the inner one. The fix is: set a flag or use a helper:

```python
    def _joint_states_callback(self, msg: JointState):
        for controller, (pub, joints) in self._publishers.items():
            positions = []
            missing = False
            for name in joints:
                try:
                    idx = msg.name.index(name)
                    positions.append(msg.position[idx])
                except ValueError:
                    missing = True
                    break  # skip this controller
            if missing:
                continue

            traj = JointTrajectory()
            traj.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 100ms
            traj.points = [point]
            pub.publish(traj)
```

- [ ] **Step 2: 验证语法**

```bash
cd /home/kzoia/alfa_robot_ws/src/alfa_robot_bringup
python3 -c "import ast; ast.parse(open('scripts/joint_states_to_controller_bridge.py').read()); print('OK')"
```

Expected: `OK`

---

## Task 2: 修复 homing_node.py 连发逻辑

**Files:**
- Modify: `alfa_robot_bringup/scripts/homing_node.py`

现有代码在微秒内连发 N 条相同轨迹，改为用定时器间隔发送。

- [ ] **Step 1: 重写 _send_home，用定时器间隔重发**

```python
def _send_home(self) -> None:
    self._done = True
    self._timer.cancel()
    self._home_remaining = self._publish_count
    self._home_timer = self.create_timer(0.2, self._publish_home_once)
    self.get_logger().info('HomingNode: sending home positions...')

def _publish_home_once(self) -> None:
    if self._home_remaining <= 0:
        self._home_timer.cancel()
        self.get_logger().info('HomingNode: all home commands sent. Shutting down.')
        raise SystemExit(0)

    for controller, (pub, joints) in self._publishers.items():
        traj = JointTrajectory()
        traj.joint_names = list(joints)
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(joints)
        point.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [point]
        pub.publish(traj)

    self._home_remaining -= 1
```

在 `__init__` 中添加成员初始化：

```python
self._home_remaining: int = 0
self._home_timer = None
```

- [ ] **Step 2: 验证语法**

```bash
python3 -c "import ast; ast.parse(open('scripts/homing_node.py').read()); print('OK')"
```

Expected: `OK`

---

## Task 3: 删除遗留文件，清理废弃配置

**Files:**
- Delete: `alfa_robot_bringup/scripts/debug_mesh_path.py`
- Delete: `alfa_robot_bringup/config/alfa_robot_controllers.yaml`
- Delete: `alfa_robot_bringup/config/alfa_robot_controllers_can.yaml`

- [ ] **Step 1: 删除文件**

```bash
cd /home/kzoia/alfa_robot_ws/src/alfa_robot_bringup
rm scripts/debug_mesh_path.py
rm config/alfa_robot_controllers.yaml
rm config/alfa_robot_controllers_can.yaml
```

- [ ] **Step 2: 确认无其他引用**

```bash
grep -r "debug_mesh_path\|alfa_robot_controllers\.yaml\|alfa_robot_controllers_can" \
  /home/kzoia/alfa_robot_ws/src --include="*.py" --include="*.yaml" --include="*.launch.py" --include="CMakeLists.txt"
```

Expected: 无输出（或只在已知过时文件中出现）

---

## Task 4: 新建 launch_utils.py

**Files:**
- Create: `alfa_robot_bringup/alfa_robot_bringup/__init__.py`
- Create: `alfa_robot_bringup/alfa_robot_bringup/launch_utils.py`

- [ ] **Step 1: 创建 Python 包**

`alfa_robot_bringup/alfa_robot_bringup/__init__.py`：
```python
```
（空文件）

- [ ] **Step 2: 编写 launch_utils.py**

```python
# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Launch utility helpers for alfa_robot_bringup."""

import os
import sys
from typing import List

import yaml


def load_yaml(file_path: str) -> dict:
    """Load a YAML file and return its contents as a dict."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Config file not found: {file_path}")
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def resolve_controller_file(package_share: str, scenario: str) -> str:
    """
    Return absolute path to the controllers yaml for a given scenario.

    Scenarios: 'default', 'slider', 'moveit'
    Falls back to 'default' if the scenario file is missing.
    """
    name = f"controllers.{scenario}.yaml"
    path = os.path.join(package_share, "config", name)
    if not os.path.exists(path):
        fallback = os.path.join(package_share, "config", "controllers.default.yaml")
        print(
            f"Warning: {name} not found, falling back to controllers.default.yaml",
            file=sys.stderr,
        )
        return fallback
    return path


def validate_prefix_unique(prefixes: List[str]) -> None:
    """Exit with error if any prefix appears more than once."""
    if len(prefixes) != len(set(prefixes)):
        duplicates = [p for p in prefixes if prefixes.count(p) > 1]
        print(
            f"Error: robot prefixes must be unique.\n"
            f"  prefixes: {prefixes}\n"
            f"  duplicates: {list(set(duplicates))}",
            file=sys.stderr,
        )
        sys.exit(1)
```

- [ ] **Step 3: 验证**

```bash
cd /home/kzoia/alfa_robot_ws/src/alfa_robot_bringup
python3 -c "
import sys; sys.path.insert(0, '.')
from alfa_robot_bringup.launch_utils import load_yaml, resolve_controller_file, validate_prefix_unique
print('import OK')
validate_prefix_unique(['a', 'b'])
print('validate_prefix_unique OK')
"
```

Expected:
```
import OK
validate_prefix_unique OK
```

---

## Task 5: 新建三套 controllers yaml

**Files:**
- Create: `alfa_robot_bringup/config/controllers.default.yaml`
- Create: `alfa_robot_bringup/config/controllers.slider.yaml`
- Create: `alfa_robot_bringup/config/controllers.moveit.yaml`

- [ ] **Step 1: controllers.default.yaml**

```yaml
# Default controllers: joint_state_broadcaster + 4x JointTrajectoryController
# Used by base.launch.py for mock hardware and general bringup.

controller_manager:
  ros__parameters:
    update_rate: 200  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    torso_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    plate_controller:
      type: joint_trajectory_controller/JointTrajectoryController

torso_group_controller:
  ros__parameters:
    joints:
      - turn
      - updown
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

left_arm_controller:
  ros__parameters:
    joints:
      - leftarmbase
      - leftjoint1
      - leftjoint2
      - leftjoint3
      - leftjoint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

right_arm_controller:
  ros__parameters:
    joints:
      - rightarmbase
      - rightjoint1
      - rightjoint2
      - rightjoint3
      - rightjoint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

plate_controller:
  ros__parameters:
    joints:
      - plate
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

- [ ] **Step 2: controllers.slider.yaml**

（与 default 相同内容，作为独立文件保留，便于后续调参差异化）

```yaml
# Slider scenario controllers: all 4 JTCs active for GUI slider control.
# Used by slider_control_real_hw.launch.py.

controller_manager:
  ros__parameters:
    update_rate: 200  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    torso_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    plate_controller:
      type: joint_trajectory_controller/JointTrajectoryController

torso_group_controller:
  ros__parameters:
    joints:
      - turn
      - updown
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

left_arm_controller:
  ros__parameters:
    joints:
      - leftarmbase
      - leftjoint1
      - leftjoint2
      - leftjoint3
      - leftjoint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

right_arm_controller:
  ros__parameters:
    joints:
      - rightarmbase
      - rightjoint1
      - rightjoint2
      - rightjoint3
      - rightjoint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

plate_controller:
  ros__parameters:
    joints:
      - plate
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

- [ ] **Step 3: controllers.moveit.yaml**

```yaml
# MoveIt scenario controllers: 4 JTCs for FollowJointTrajectory execution.
# Used by moveit_real_hardware_test.launch.py.
# Mirrors alfa_robot_moveit_config/config/ros2_controllers.yaml but owned here.

controller_manager:
  ros__parameters:
    update_rate: 200  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    torso_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    plate_controller:
      type: joint_trajectory_controller/JointTrajectoryController

torso_group_controller:
  ros__parameters:
    joints:
      - turn
      - updown
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

left_arm_controller:
  ros__parameters:
    joints:
      - leftarmbase
      - leftjoint1
      - leftjoint2
      - leftjoint3
      - leftjoint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

right_arm_controller:
  ros__parameters:
    joints:
      - rightarmbase
      - rightjoint1
      - rightjoint2
      - rightjoint3
      - rightjoint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

plate_controller:
  ros__parameters:
    joints:
      - plate
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

---

## Task 6: 新建 base.launch.py（原子基座）

**Files:**
- Create: `alfa_robot_bringup/launch/base.launch.py`

原子基座：只做 robot_description / robot_state_publisher / ros2_control_node / joint_state_broadcaster。
启动顺序：RSP →(2s)→ control_node →(3s)→ joint_state_broadcaster →(exit)→ 默认控制器（可选）。

- [ ] **Step 1: 编写 base.launch.py**

```python
# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

"""
alfa_robot 单机器人原子基座。

只负责：
  robot_state_publisher
  ros2_control_node
  joint_state_broadcaster (spawner)
  可选默认控制器 spawner

不包含：RViz、MoveIt、homing_node、GUI 控制节点。
上层 launch 文件通过 IncludeLaunchDescription 引入本文件，再附加场景节点。

参数:
  controllers_file        -- 完整路径或相对于 runtime_config_package/config 的文件名
  runtime_config_package  -- 含 controllers yaml 的包名（默认 alfa_robot_bringup）
  description_package     -- 含 URDF/xacro 的包名
  description_file        -- xacro 文件名
  prefix                  -- 关节名前缀
  use_mock_hardware       -- 是否使用 mock 硬件
  mock_sensor_commands    -- mock 传感器命令
  real_hardware_plugin    -- 实机硬件插件
  canopen_profile_velocity
  canopen_profile_accel
  default_controller      -- 基座层默认启动的控制器名，空字符串表示不启动
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="alfa_robot_bringup",
            description="Package containing the controllers yaml in its config/ directory.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.default.yaml",
            description="Controllers yaml filename (relative to runtime_config_package/config).",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="alfa_robot_description",
            description="Package containing the robot URDF/xacro.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="alfa_robot.urdf.xacro",
            description="URDF/xacro filename.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint name prefix for multi-robot setups.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware (mirrors commands to states).",
        ),
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock sensor command interfaces.",
        ),
        DeclareLaunchArgument(
            "real_hardware_plugin",
            default_value="alfa_robot_hardware/AlfaRobotHW",
            description="Hardware plugin class (used when use_mock_hardware:=false).",
        ),
        DeclareLaunchArgument(
            "canopen_profile_velocity",
            default_value="50000",
            description="CANopen profile velocity in pulses/s.",
        ),
        DeclareLaunchArgument(
            "canopen_profile_accel",
            default_value="50000",
            description="CANopen profile acceleration in pulses/s².",
        ),
        DeclareLaunchArgument(
            "default_controller",
            default_value="",
            description="Controller to spawn after JSB. Empty string = do not spawn any.",
        ),
    ]

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    real_hardware_plugin = LaunchConfiguration("real_hardware_plugin")
    canopen_profile_velocity = LaunchConfiguration("canopen_profile_velocity")
    canopen_profile_accel = LaunchConfiguration("canopen_profile_accel")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " prefix:=", prefix,
            " use_mock_hardware:=", use_mock_hardware,
            " mock_sensor_commands:=", mock_sensor_commands,
            " real_hardware_plugin:=", real_hardware_plugin,
            " canopen_profile_velocity:=", canopen_profile_velocity,
            " canopen_profile_accel:=", canopen_profile_accel,
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 事件驱动启动链：RSP → (2s) → control_node → (3s) → JSB
    delay_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[TimerAction(period=2.0, actions=[control_node])],
        )
    )
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            delay_control_node,
            delay_jsb,
        ]
    )
```

- [ ] **Step 2: 语法检查**

```bash
python3 -c "
import ast
src = open('/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/launch/base.launch.py').read()
ast.parse(src)
print('base.launch.py syntax OK')
"
```

---

## Task 7: 重写 alfa_robot.launch.py 为 base.launch.py 的薄包装

**Files:**
- Modify: `alfa_robot_bringup/launch/alfa_robot.launch.py`

保留向后兼容（`robot_controller` 参数、RViz、GUI 滑块组），但核心 bringup 改为 include base.launch.py。

- [ ] **Step 1: 重写 alfa_robot.launch.py**

```python
# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

"""
alfa_robot 通用入口 launch 文件。

向后兼容包装：包含 base.launch.py，并在其上附加：
  - RViz 可视化
  - GUI 滑块控制（use_joint_gui_control:=true 时）
  - 指定控制器 spawner（robot_controller 参数）

直接使用建议：
  仿真/开发:  ros2 launch alfa_robot_bringup alfa_robot.launch.py
  滑块实机:   ros2 launch alfa_robot_bringup slider_control_real_hw.launch.py
  MoveIt测试: ros2 launch alfa_robot_bringup moveit_real_hardware_test.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="alfa_robot_bringup",
            description="Package with controllers yaml.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.default.yaml",
            description="Controllers yaml filename.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="alfa_robot_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="alfa_robot.urdf.xacro",
        ),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
        DeclareLaunchArgument(
            "real_hardware_plugin",
            default_value="alfa_robot_hardware/AlfaRobotHW",
        ),
        DeclareLaunchArgument("canopen_profile_velocity", default_value="50000"),
        DeclareLaunchArgument("canopen_profile_accel", default_value="50000"),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="torso_group_controller",
            choices=[
                "torso_group_controller",
                "left_arm_controller",
                "right_arm_controller",
                "plate_controller",
            ],
            description="Single controller to spawn after JSB.",
        ),
        DeclareLaunchArgument(
            "use_joint_gui_control",
            default_value="false",
            description="Enable joint_state_publisher_gui + bridge node for slider control.",
        ),
    ]

    robot_controller = LaunchConfiguration("robot_controller")
    use_joint_gui_control = LaunchConfiguration("use_joint_gui_control")
    description_package = LaunchConfiguration("description_package")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_bringup"), "launch", "base.launch.py"]
            )
        ),
        launch_arguments={
            "runtime_config_package": LaunchConfiguration("runtime_config_package"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "prefix": LaunchConfiguration("prefix"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "mock_sensor_commands": LaunchConfiguration("mock_sensor_commands"),
            "real_hardware_plugin": LaunchConfiguration("real_hardware_plugin"),
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "alfa_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    joint_gui_control_group = GroupAction(
        condition=IfCondition(use_joint_gui_control),
        actions=[
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                remappings=[("joint_states", "joint_states_gui")],
            ),
            Node(
                package="alfa_robot_bringup",
                executable="joint_states_to_controller_bridge.py",
                name="joint_states_to_controller_bridge",
                output="screen",
            ),
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            base_launch,
            rviz_node,
            delay_robot_controller,
            joint_gui_control_group,
        ]
    )
```

- [ ] **Step 2: 语法检查**

```bash
python3 -c "import ast; ast.parse(open('/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/launch/alfa_robot.launch.py').read()); print('OK')"
```

---

## Task 8: 重写 slider_control_real_hw.launch.py（全部 4 个控制器）

**Files:**
- Modify: `alfa_robot_bringup/launch/slider_control_real_hw.launch.py`

场景契约明确：**控全部 4 个控制器 + GUI 滑块 + homing**。

- [ ] **Step 1: 重写文件**

```python
"""
实机 GUI 滑块控制（全关节）+ RViz 可视化

启动全部 4 个 JTC 控制器（torso/left_arm/right_arm/plate），
joint_states_to_controller_bridge 向 4 个控制器分发滑块指令，
homing_node 等待全部 4 个控制器 active 后发送一次全零位。

使用方式:
  ros2 launch alfa_robot_bringup slider_control_real_hw.launch.py

可选参数:
  canopen_profile_velocity:=50000
  canopen_profile_accel:=50000

启动前建立 CAN 接口:
  sudo ip link set can0 up type can bitrate 1000000
  sudo ip link set can1 up type can bitrate 1000000
  sudo ip link set can2 up type can bitrate 1000000
  sudo ip link set can3 up type can bitrate 1000000
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "canopen_profile_velocity",
            default_value="50000",
            description="CANopen 线性关节速度 (pulses/s)。50000≈3.8mm/s",
        ),
        DeclareLaunchArgument(
            "canopen_profile_accel",
            default_value="50000",
            description="CANopen 线性关节加速度 (pulses/s²)",
        ),
    ]

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_bringup"), "launch", "base.launch.py"]
            )
        ),
        launch_arguments={
            "use_mock_hardware": "false",
            "controllers_file": "controllers.slider.yaml",
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    torso_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["torso_group_controller", "-c", "/controller_manager"],
    )
    left_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )
    right_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )
    plate_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["plate_controller", "-c", "/controller_manager"],
    )

    # 事件驱动：JSB 退出后依次 spawn 4 个控制器
    spawn_torso = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[torso_spawner],
        )
    )
    spawn_left = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=torso_spawner,
            on_exit=[left_arm_spawner],
        )
    )
    spawn_right = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_spawner,
            on_exit=[right_arm_spawner],
        )
    )
    spawn_plate = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_spawner,
            on_exit=[plate_spawner],
        )
    )

    bridge_node = Node(
        package="alfa_robot_bringup",
        executable="joint_states_to_controller_bridge.py",
        name="joint_states_to_controller_bridge",
        output="screen",
    )

    gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        remappings=[("joint_states", "joint_states_gui")],
    )

    homing_node = Node(
        package="alfa_robot_bringup",
        executable="homing_node.py",
        name="homing_node",
        output="screen",
        parameters=[{
            "publish_count": 5,
            "poll_interval": 1.0,
        }],
    )

    return LaunchDescription(
        declared_arguments + [
            base_launch,
            spawn_torso,
            spawn_left,
            spawn_right,
            spawn_plate,
            bridge_node,
            gui_node,
            homing_node,
        ]
    )
```

- [ ] **Step 2: 语法检查**

```bash
python3 -c "import ast; ast.parse(open('/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/launch/slider_control_real_hw.launch.py').read()); print('OK')"
```

---

## Task 9: 重写 moveit_real_hardware_test.launch.py（事件驱动）

**Files:**
- Modify: `alfa_robot_bringup/launch/moveit_real_hardware_test.launch.py`

去掉全部 TimerAction 硬等待，改为 include base.launch.py + 事件驱动 spawn 4 个控制器 + 附加 MoveIt 节点。

- [ ] **Step 1: 重写文件**

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        DeclareLaunchArgument("target_x", default_value="0.357"),
        DeclareLaunchArgument("target_y", default_value="-0.705"),
        DeclareLaunchArgument("target_z", default_value="0.684"),
        DeclareLaunchArgument("target_qx", default_value="-0.502"),
        DeclareLaunchArgument("target_qy", default_value="0.502"),
        DeclareLaunchArgument("target_qz", default_value="-0.498"),
        DeclareLaunchArgument("target_qw", default_value="0.498"),
    ]

    auto_run_test = LaunchConfiguration("auto_run_test")
    group_name = LaunchConfiguration("group_name")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    target_qx = LaunchConfiguration("target_qx")
    target_qy = LaunchConfiguration("target_qy")
    target_qz = LaunchConfiguration("target_qz")
    target_qw = LaunchConfiguration("target_qw")

    # MoveIt 配置（在 evaluate 时加载，不依赖 Timer）
    moveit_config = MoveItConfigsBuilder(
        "alfa_robot", package_name="alfa_robot_moveit_config"
    ).to_moveit_configs()

    # 原子基座（实机、moveit 场景 controllers）
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("alfa_robot_bringup"), "launch", "base.launch.py"]
            )
        ),
        launch_arguments={
            "use_mock_hardware": "false",
            "controllers_file": "controllers.moveit.yaml",
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "prefix": LaunchConfiguration("prefix"),
            "real_hardware_plugin": LaunchConfiguration("real_hardware_plugin"),
            "canopen_profile_velocity": LaunchConfiguration("canopen_profile_velocity"),
            "canopen_profile_accel": LaunchConfiguration("canopen_profile_accel"),
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    torso_spawner = Node(
        package="controller_manager", executable="spawner", output="both",
        arguments=["torso_group_controller", "-c", "/controller_manager"],
    )
    left_arm_spawner = Node(
        package="controller_manager", executable="spawner", output="both",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )
    right_arm_spawner = Node(
        package="controller_manager", executable="spawner", output="both",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )
    plate_spawner = Node(
        package="controller_manager", executable="spawner", output="both",
        arguments=["plate_controller", "-c", "/controller_manager"],
    )

    # 事件驱动 controller spawn 链
    spawn_torso = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[torso_spawner],
        )
    )
    spawn_left = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=torso_spawner, on_exit=[left_arm_spawner])
    )
    spawn_right = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=left_arm_spawner, on_exit=[right_arm_spawner])
    )
    spawn_plate = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=right_arm_spawner, on_exit=[plate_spawner])
    )

    # MoveIt 节点在最后一个控制器 spawn 完成后启动（5s 宽裕时间）
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

    start_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=plate_spawner,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[static_tf_launch, move_group_launch, moveit_rviz_launch],
                )
            ],
        )
    )

    trajectory_executor_node = Node(
        package="alfa_robot_moveit_config",
        executable="trajectory_executor",
        output="screen",
        condition=IfCondition(auto_run_test),
    )
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

    # 测试节点在 MoveIt 启动后 6s 再起
    start_test_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=plate_spawner,
            on_exit=[
                TimerAction(
                    period=11.0,
                    actions=[trajectory_executor_node],
                    condition=IfCondition(auto_run_test),
                ),
                TimerAction(
                    period=15.0,
                    actions=[path_planning_node],
                    condition=IfCondition(auto_run_test),
                ),
            ],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            base_launch,
            spawn_torso,
            spawn_left,
            spawn_right,
            spawn_plate,
            start_moveit,
            start_test_nodes,
        ]
    )
```

- [ ] **Step 2: 语法检查**

```bash
python3 -c "import ast; ast.parse(open('/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/launch/moveit_real_hardware_test.launch.py').read()); print('OK')"
```

---

## Task 10: 更新 CMakeLists.txt，安装 Python 模块

**Files:**
- Modify: `alfa_robot_bringup/CMakeLists.txt`

- [ ] **Step 1: 更新 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(alfa_robot_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files and configs
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts (executables)
install(
  PROGRAMS
    scripts/joint_states_to_controller_bridge.py
    scripts/homing_node.py
    scripts/test_moveit_pose_goal.py
    scripts/setup_can.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module (launch_utils)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
endif()

ament_package()
```

- [ ] **Step 2: 确认 package.xml 有 ament_cmake_python 依赖**

```bash
grep -c "ament_cmake_python" /home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/package.xml
```

如果输出 `0`，需在 `package.xml` 的 `<buildtool_depend>` 区块添加：
```xml
<buildtool_depend>ament_cmake_python</buildtool_depend>
```

---

## Task 11: 更新 test_goal_publishers_config.yaml 控制器名称

**Files:**
- Modify: `alfa_robot_bringup/config/test_goal_publishers_config.yaml`

- [ ] **Step 1: 更新 controller_name 字段**

```yaml
# Copyright (c) 2025, b»robotized
# ...

# 测试用：左臂 leftjoint2, leftjoint3, leftjoint4
# 对应控制器：left_arm_controller（JointTrajectoryController）

publisher_joint_trajectory_controller:
  ros__parameters:
    controller_name: "left_arm_controller"
    wait_sec_between_publish: 6
    repeat_the_same_goal: 1

    goal_time_from_start: 3.0
    goal_names: ["pos1", "pos2", "pos3", "pos4"]
    pos1:
      positions: [0.0, 0.785, 0.785, 0.785, 0.0]
    pos2:
      positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    pos3:
      positions: [0.0, -0.785, -0.785, -0.785, 0.0]
    pos4:
      positions: [0.0, 0.0, 0.0, 0.0, 0.0]

    joints:
      - leftarmbase
      - leftjoint1
      - leftjoint2
      - leftjoint3
      - leftjoint4
```

---

## Task 12: 构建验证

- [ ] **Step 1: 构建包**

```bash
cd /home/kzoia/alfa_robot_ws
colcon build --packages-select alfa_robot_bringup --symlink-install 2>&1 | tail -20
```

Expected: `Summary: 1 package finished`（无 ERROR）

- [ ] **Step 2: 验证��装文件**

```bash
ls /home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/install/alfa_robot_bringup/share/alfa_robot_bringup/launch/
```

Expected: 包含 `base.launch.py`, `alfa_robot.launch.py`, `slider_control_real_hw.launch.py`, `moveit_real_hardware_test.launch.py`

- [ ] **Step 3: 验证 Python 模块可导入**

```bash
source /home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/install/local_setup.bash 2>/dev/null || true
python3 -c "from alfa_robot_bringup.launch_utils import load_yaml, resolve_controller_file, validate_prefix_unique; print('launch_utils import OK')"
```

Expected: `launch_utils import OK`

- [ ] **Step 4: Commit**

```bash
cd /home/kzoia/alfa_robot_ws/src/alfa_robot_bringup
git add -A
git commit -m "refactor: restructure bringup into layered base/scene/utils architecture

- Add base.launch.py: atomic single-robot base (RSP/control_node/JSB, event-driven)
- Add launch_utils.py: load_yaml / resolve_controller_file / validate_prefix_unique
- Add controllers.{default,slider,moveit}.yaml: named scenario configs
- Rewrite alfa_robot.launch.py as thin base.launch.py wrapper
- Rewrite slider_control_real_hw.launch.py: all 4 JTCs + GUI + homing
- Rewrite moveit_real_hardware_test.launch.py: event-driven, no TimerAction chains
- Fix joint_states_to_controller_bridge.py: return->continue bug (per-controller skip)
- Fix homing_node.py: replace burst-publish with timer-spaced republish
- Remove debug_mesh_path.py (agent artifact with wrong hardcoded path)
- Remove legacy ForwardCommandController yaml files"
```
