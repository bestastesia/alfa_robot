# MoveIt2 Real Hardware Test

本文档说明 `alfa_robot_bringup` 中新增的 MoveIt2 实机轨迹测试链路，以及 `test_moveit_pose_goal.py` 的使用方式。

## 1. 目的

这个测试链路用于验证下面这条路径是否配置正确：

`test_moveit_pose_goal.py`
-> `move_group` (`move_action`)
-> MoveIt 控制器选择
-> `left_arm_controller` / `right_arm_controller`
-> `follow_joint_trajectory`
-> `ros2_control`
-> `AlfaRobotHW::write()`
-> `CanBus::writeOnce()`
-> 实际 CAN / CANopen 总线

未连接真实硬件时，最多只能验证到软件链路是否完整，不能证明 CAN 帧已经真正发到电机。

## 2. 当前链路组成

### 2.1 启动文件

`launch/moveit_real_hardware_test.launch.py` 会按顺序启动：

1. `robot_state_publisher`
2. `ros2_control_node`
3. `joint_state_broadcaster`
4. `left_arm_controller`
5. `right_arm_controller`
6. `static_virtual_joint_tfs`
7. `move_group`
8. `moveit_rviz`
9. 可选 `test_moveit_pose_goal.py`

其中控制器启动顺序改成了定时串行拉起，避免原先基于 `spawner` 退出事件时序不稳定的问题。

### 2.2 控制器配置

实机测试使用的控制器配置文件是：

- `config/alfa_robot_moveit_real_controllers.yaml`

它定义了两个 `JointTrajectoryController`：

- `left_arm_controller`
- `right_arm_controller`

两个控制器都导出 `position` 命令接口，供 MoveIt 执行关节轨迹。

### 2.3 MoveIt 控制器映射

MoveIt 使用下面的配置把规划结果映射到执行控制器：

- `../alfa_robot_moveit_config/config/moveit_controllers.yaml`

当前已确认：

- `left_arm_controller` 的 `action_ns` 为 `follow_joint_trajectory`
- `right_arm_controller` 的 `action_ns` 也已补齐为 `follow_joint_trajectory`

如果这里漏掉 `action_ns`，MoveIt 只能识别到部分控制器，轨迹无法完整下发。

### 2.4 硬件写入位置

实机命令最终在下面这个函数汇总并写出：

- `../alfa_robot_hardware/src/alfa_robot_hardware.cpp`

关键流程：

1. `JointTrajectoryController` 把目标位置写入硬件命令接口。
2. `AlfaRobotHW::write()` 读取 `hw_position_commands_` 和 `canopen_position_commands_`。
3. RMD 关节和 CANopen 关节分别整理成命令映射。
4. 调用 `can_bus_->writeOnce(rmd_cmds, canopen_cmds, dt)`。

## 3. 已修正的问题

为了让这条链路能正常拉起，当前已经修正这几个问题：

### 3.1 轮子关节名不一致

URDF 中的轮子关节是：

- `left_back`
- `left_forward`
- `right_back`
- `right_forward`

硬件与控制器配置中曾经误写成带空格的名字，导致 `ros2_control` 报：

- `Wrong state or command interface configuration`

现在已统一成下划线命名。

### 3.2 右臂 MoveIt 控制器缺少 action namespace

`right_arm_controller` 原先未声明：

- `action_ns: follow_joint_trajectory`

这个问题会导致 MoveIt 只接入一部分控制器。现在已补齐。

### 3.3 bringup 启动顺序不稳定

原先 launch 依赖 `spawner` 退出事件继续拉后续节点，实际运行时不稳定。现在改成显式延时串行启动：

- 2s: `ros2_control_node`
- 5s: `joint_state_broadcaster`
- 8s: `left_arm_controller`
- 11s: `right_arm_controller`
- 14s: MoveIt + RViz
- 20s: 自动测试节点

## 4. 测试脚本说明

测试脚本路径：

- `scripts/test_moveit_pose_goal.py`

这个脚本不是直接往控制器 topic 发消息，而是给 MoveIt 的 `move_action` 发送一个目标位姿约束，然后由 MoveIt 完成：

1. 规划
2. 发布显示轨迹给 RViz
3. 选择执行控制器
4. 调用 `follow_joint_trajectory` action 执行

### 4.1 默认参数

默认参数针对左臂：

- `group_name:=left_arm`
- `ee_link:=left_ee_link`
- `reference_frame:=base_link`
- 默认目标位姿:
  - `x=0.613`
  - `y=-0.900`
  - `z=1.083`
  - `qx=0.507`
  - `qy=-0.377`
  - `qz=0.623`
  - `qw=-0.462`

### 4.2 关键参数

- `group_name`: MoveIt 规划组，通常是 `left_arm` 或 `right_arm`
- `ee_link`: 末端 link，例如 `left_ee_link`
- `reference_frame`: 目标约束参考系，默认 `base_link`
- `position_tolerance`: 位置容差
- `orientation_tolerance`: 姿态容差
- `allowed_planning_time`: 规划超时时间
- `num_planning_attempts`: 规划尝试次数
- `max_velocity_scaling_factor`: 最大速度缩放
- `max_acceleration_scaling_factor`: 最大加速度缩放
- `plan_only`: 仅规划不执行
- `wait_for_server_sec`: 等待 `move_action` 的超时时间

## 5. 使用方法

### 5.1 编译

```bash
cd ~/alfa_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select alfa_robot_bringup alfa_robot_moveit_config alfa_robot_hardware
source install/setup.bash
```

### 5.2 只启动链路，不自动发测试

```bash
ros2 launch alfa_robot_bringup moveit_real_hardware_test.launch.py auto_run_test:=false
```

适合先确认下面这些内容是否都起来：

- `joint_state_broadcaster`
- `left_arm_controller`
- `right_arm_controller`
- `move_group`
- RViz

### 5.3 启动后自动发一次测试轨迹

```bash
ros2 launch alfa_robot_bringup moveit_real_hardware_test.launch.py auto_run_test:=true
```

默认会给左臂发送一次位姿目标。

### 5.4 手动指定测试目标

左臂示例：

```bash
ros2 launch alfa_robot_bringup moveit_real_hardware_test.launch.py \
  auto_run_test:=true \
  group_name:=left_arm \
  ee_link:=left_ee_link \
  target_x:=0.357 \
  target_y:=-0.705 \
  target_z:=0.684 \
  target_qx:=-0.502 \
  target_qy:=0.502 \
  target_qz:=-0.498 \
  target_qw:=0.498
```
- Translation: [0.357, -0.705, 0.684]
- Rotation: in Quaternion (xyzw) [-0.502, 0.502, -0.498, 0.498]
右臂示例：

```bash
ros2 launch alfa_robot_bringup moveit_real_hardware_test.launch.py \
  auto_run_test:=true \
  group_name:=right_arm \
  ee_link:=right_ee_link
```

### 5.5 单独运行测试脚本

前提是 `move_group` 已经在运行：

```bash
ros2 run alfa_robot_bringup test_moveit_pose_goal.py \
  --ros-args \
  -p group_name:=left_arm \
  -p ee_link:=left_ee_link
```

如果只想验证规划，不执行控制器：

```bash
ros2 run alfa_robot_bringup test_moveit_pose_goal.py \
  --ros-args \
  -p group_name:=left_arm \
  -p ee_link:=left_ee_link \
  -p plan_only:=true
```

## 6. 预期现象

### 6.1 软件链路正常时

启动日志中应能看到：

- `joint_state_broadcaster` loaded and activated
- `left_arm_controller` loaded and activated
- `right_arm_controller` loaded and activated
- `move_group` 输出 `You can start planning now!`

测试脚本正常时应输出类似：

- `MoveIt goal accepted, waiting for result`
- `MoveIt finished: error_code=1 planned_points=... executed_points=...`

### 6.2 RViz 正常时

应能看到：

1. 机器人模型
2. MoveIt 规划出的路径
3. 执行时的轨迹显示

如果当前环境没有图形显示，`rviz2` 可能因 `could not connect to display` 退出。这不影响命令链路本身，但会失去可视化。

### 6.3 未连接硬件时

如果 CAN 口不可用，硬件层通常会打印类似：

- `Failed to create CAN socket`
- `CAN bus ... not available`

这说明：

- 软件链路可能已经通了
- 但物理总线写入没有发生

## 7. 建议检查项

接上真实硬件后，建议按这个顺序验证：

1. `can0` / `can1` / `can2` / `can3` 已经正确 `UP`
2. `ros2_control_node` 成功加载 `AlfaRobotHW`
3. `left_arm_controller` 和 `right_arm_controller` 均为 active
4. `move_group` 已启动
5. `test_moveit_pose_goal.py` 返回 `error_code=1`
6. 电机侧或总线抓包能看到实际命令发送

## 8. 相关文件

- `launch/moveit_real_hardware_test.launch.py`
- `scripts/test_moveit_pose_goal.py`
- `config/alfa_robot_moveit_real_controllers.yaml`
- `../alfa_robot_moveit_config/config/moveit_controllers.yaml`
- `../alfa_robot_hardware/src/alfa_robot_hardware.cpp`
