# 固定平台双臂临时验收运行手册

本手册用于 `ros2_tmp` 临时环境。当前链路：任务发布 → 任务编排 → KDL IK → MoveIt 关节规划 → PLC/mock 执行 → RViz/monitor 观察。

## 0. 通用准备

每个新终端先执行：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果刚改过 C++：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
source /opt/ros/humble/setup.bash
colcon build --packages-select alfa_robot_plc_bridge alfa_robot_benchmarks --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 1. 推荐方式：一键启动除任务发布外的全部线程

### mock PLC 联调

终端 1：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py \
  plc_mock:=true
```

### 真实 PLC 联调

确认机械臂和急停安全后，终端 1 改为：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py \
  plc_mock:=false \
  velocity_limit_deg_s:=5.0 \
  plc_execution_mode:=stream
```

如果真实 PLC 出现轨迹流式覆盖卡顿，可临时切成更接近 CLI 的最终点绝对运动模式做诊断：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py \
  plc_mock:=false \
  velocity_limit_deg_s:=3.0 \
  plc_execution_mode:=final_abs
```

注意：`final_abs` 只用于排查“PLC 单次绝对运动是否正常”，不是正式轨迹执行方案。正式避障/轨迹运动仍使用 `plc_execution_mode:=stream`。

这个 launch 会启动：

- `demo.launch.py`：URDF、robot_state_publisher、ros2_control fake/demo、move_group、RViz。
- `plc_bridge_node`：PLC/mock PLC 执行层，接收 `/plc_joint_trajectory`，默认不发布 `/joint_states`。
- `plc_safety_node`：急停/安全接口。
- `fixed_platform_kdl_ik_service.py`：固定平台双臂 IK 服务 `/alfa_dual_ik/solve`。
- `temporary_moveit_joint_planner`：临时 MoveIt 关节规划服务 `/alfa_moveit/plan_joint_target`。
- `fixed_platform_task_orchestrator`：任务编排层。
- `monitor_ik_planner_chain.py`：链路诊断输出。

关闭时在终端 1 按 `Ctrl-C`，由 ROS launch 统一关闭子进程。若异常残留，可重启终端或手动清理相关 ROS 进程。

## 2. 单独启动任务发布线程

终端 2：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run alfa_robot_benchmarks mock_box_task_publisher
```

启动后按回车发布当前任务。现在任务发布层不再依赖箱子编号，而是直接发布左右末端坐标。

默认任务语义：

- 任务：`manual_front_pair`
- 左末端目标：`(0.70, 0.20, 1.40)`
- 右末端目标：`(0.70, -0.20, 1.40)`
- 坐标系：`world`
- 朝向：保持 `+X`；默认不额外 roll（`left_roll_deg=0`、`right_roll_deg=0`）。
- 机器人固定 `updown=0.18m`

临时改目标坐标示例：

```bash
ros2 run alfa_robot_benchmarks mock_box_task_publisher --ros-args \
  -p task_id:=manual_front_pair \
  -p left_x:=0.70 -p left_y:=0.20 -p left_z:=1.40 \
  -p right_x:=0.70 -p right_y:=-0.20 -p right_z:=1.40 \
  -p left_roll_deg:=0.0 -p right_roll_deg:=0.0
```

## 3. 关键可调参数

为了避免启动命令过长，只把当前最重要的旋钮放到一键栈：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `plc_mock` | `true` | `true` 走 mock PLC，`false` 连接真实 PLC。 |
| `velocity_limit_deg_s` | `5.0` | PLC 层写入每轴速度上限，类似 CLI 的 `--vel`。 |
| `plc_execution_mode` | `stream` | 正式执行用 `stream` 按 MoveIt 轨迹连续覆盖目标；`final_abs` 只用于诊断，会丢掉中间轨迹点。 |
| `moveit_velocity_scale` | `0.25` | MoveIt 规划轨迹速度缩放。 |
| `moveit_acceleration_scale` | `0.2` | MoveIt 规划轨迹加速度缩放。 |
| `planning_time` | `8.0` | MoveIt 单次规划时间上限。 |
| `planning_attempts` | `20` | MoveIt 规划尝试次数。 |
| `ik_max_attempts` | `5` | 编排层 IK 失败后的重试次数。 |

常用真实 PLC 启动例子：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py \
  plc_mock:=false \
  velocity_limit_deg_s:=3.0 \
  moveit_velocity_scale:=0.15 \
  planning_time:=10.0 \
  planning_attempts:=30
```

## 4. PLC CLI 与 ROS bridge 的本质区别

你手动执行的命令：

```bash
python3 -m alfa_robot_plc_driver.cli move-abs --targets 1:0,2:0,...,12:0 --vel 3 --yes-write
```

底层写寄存器方式和 ROS bridge 使用的是同一套协议核心：都写每轴 `VELOCITY/ACC/DEC/TARGET/COMMAND_ID/CONTROL_WORD=ENABLE_MOVE_ABS`。

本质区别不是寄存器协议，而是运动命令形态：

- CLI `move-abs`：一次性给 12 轴一个最终绝对目标，PLC 自己按速度执行到位。
- ROS bridge `stream`：先接收 MoveIt 的整条轨迹，再按 `command_hz` 周期不断覆盖每轴绝对目标，相当于“位置流式跟随”。真实 PLC 卡顿时，优先怀疑这个周期覆盖模式与 PLC 侧运动队列/刷新节奏不匹配。
- ROS bridge `final_abs`：只取 MoveIt 轨迹最后一个点，调用同类 `move_abs` 逻辑写 PLC；它会丢掉中间轨迹点，所以只适合诊断，不适合正式避障轨迹执行。


## 5. 真实 PLC 不动时的最小断点检查

如果 CLI `move-abs` 能动，但本栈发任务不动，不要反复点任务，先按顺序看断点：

1. 看 PLC bridge 是否真是实机模式、参数是否像 CLI：

```bash
ros2 param get /plc_bridge_node mock
ros2 param get /plc_bridge_node plc_ip
ros2 param get /plc_bridge_node velocity_limit_deg_s
ros2 param get /plc_bridge_node acceleration_limit_deg_s2
ros2 param get /plc_bridge_node deceleration_limit_deg_s2
ros2 param get /plc_bridge_node emergency_deceleration_deg_s2
ros2 param get /plc_bridge_node plc_execution_mode
```

期望：`mock=false`，`vel` 与启动值一致，`acc/dec/emergency_dec` 默认分别是 `10/10/30`，和 CLI 默认一致。

2. 看 bridge 有没有收到编排层发来的轨迹：

```bash
ros2 topic echo /plc_bridge_state
```

同时看启动终端是否出现：

```text
accepted trajectory for PLC from topic: mode=stream, points=..., duration=..., preview=Axis1: ...
trajectory finished: commands=..., duration=..., final_targets={Axis1: ...}
```

先看编排层是否出现：

```text
Publishing PLC trajectory task=... topic=/plc_joint_trajectory points=... joints=... subscribers=...
Published PLC trajectory task=...
```

如果这里 `subscribers=0`，说明 PLC bridge 没订阅到 `/plc_joint_trajectory`，通常是 bridge 没启动、域不一致或没有 source 当前工作区。
如果没有 `Publishing PLC trajectory`，问题在 IK/MoveIt 到编排层发布之前。
如果有 `Publishing PLC trajectory` 但没有 `accepted trajectory for PLC from topic`，问题在 `/plc_joint_trajectory` topic 连接。
如果有 `accepted trajectory for PLC` 但 `trajectory execution failed`，问题在 PLC 通讯或寄存器写入。
如果有 `trajectory finished` 但机械不动，优先检查轴映射/轴反向/PLC 当前模式/是否被急停或未使能。

3. 对照 CLI 后读 PLC 状态：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_ec/tools/alfa_robot_plc_driver
python3 -m alfa_robot_plc_driver.cli status --max-axes 12
```

重点看每轴 `target/vel/ack/err/status_word` 是否变化。


## 6. 当前几何与障碍口径

- 箱垛：2 列 × 4 行。
- 两列中心对称在机器人中线两侧：`y=+0.20` 和 `y=-0.20`。
- 箱子尺寸：`0.3m × 0.4m × 0.4m`。
- 箱子前侧面：`x=0.76m`。
- 箱子中心：`x=0.91m`。
- 末端规划点：`x=0.70m`，比箱子前侧面安全退让约 `6cm`。
- 障碍不启动 MuJoCo，只按 MuJoCo/仿真参数静态注入 MoveIt PlanningScene。

## 7. 当前初始姿态

- `updown=0.18m`。
- 左右臂 home：`0°, 0°, 0°, 0°, 0°, 0°`。
- PLC bridge 不再发布 `/joint_states`；`/joint_states` 只采用 demo/ros2_control 的状态。

检查 `/joint_states` 发布者：

```bash
ros2 topic info /joint_states -v
```

期望：`Publisher count: 1`，发布者为 `joint_state_broadcaster`。

## 8. 各线程数据流

### demo / MoveIt / RViz 层

- 输入：URDF、SRDF、controller 配置、PlanningScene。
- 输出：`/joint_states`、TF、`/move_group` action、RViz 可视化。
- 作用：作为当前联调的“真实机器人状态”和规划后端。

### IK 服务层

- 服务：`/alfa_dual_ik/solve`。
- 输入：左右末端 pose、抓取模式、当前 joint seed、固定 `updown`。
- 输出：12 轴 joint target + `updown`。
- 当前策略：左右臂分别 KDL `/compute_ik`，合并后用 `/check_state_validity` 做双臂/全身碰撞校验。

### MoveIt 临时规划层

- 服务：`/alfa_moveit/plan_joint_target`。
- 输入：IK 产出的 joint target。
- 输出：`trajectory_msgs/JointTrajectory`。
- 当前参数：`planning_time=8s`，`planning_attempts=20`，比早期 `3s/5 attempts` 更保守。
- 额外检查：目标 state 越界/碰撞拒绝、轨迹保持 `updown=0.18`、末点接近 IK target。

### 任务编排层

- 订阅：`/alfa_task/command`。
- 发布：`/alfa_task/status`、`/plc_joint_trajectory`、调试 topic。
- 流程：收到任务 → 回 accepted → 调 IK → 调 planner → 发轨迹 → 等 PLC/mock 执行真正开始并结束 → 回 done。

### PLC/mock 执行层

- 订阅：`/plc_joint_trajectory`。
- 发布：`/plc_bridge_state`。
- 当前不发布 `/joint_states`。
- 真实 PLC 时速度上限：`velocity_limit_deg_s=5.0`。

### 急停层

接口：

```bash
ros2 service call /alfa_safety/soft_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/emergency_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/reset std_srvs/srv/Trigger {}
ros2 topic echo /alfa_safety/state
```

### monitor 层

- 监听 `/alfa_debug/orchestrator_ik_result`。
- 监听 `/alfa_debug/planner_received_joint_target`。
- 监听 `/alfa_debug/planner_output_trajectory`。
- 监听 `/alfa_debug/orchestrator_plc_trajectory`。
- 用于确认 IK → planner → PLC 的 joint 数据没有被改坏。

## 9. 验收时重点看

终端 1 应出现类似日志：

```text
Applied 8 static 2-column x 4-row acceptance boxes: front_x=0.76 center_x=0.91
Temporary MoveIt joint planner ready: group=dual_v5_arm_with_base ... updown=0.180
KDL IK success h=0.180 ... left=SUCCESS right=SUCCESS
Plan success task=manual_front_pair ...
accepted trajectory for PLC: mode=stream, points=..., duration=...
trajectory finished: commands=... duration=...
Task manual_front_pair: done - task completed
```

monitor 里重点看：

```text
IK -> planner request: 全部 0
planner trajectory last -> PLC trajectory last: 全部 0
planner target -> planner trajectory last: 小于 1deg
```

## 10. 已验证结果

本机在 `2026-06-05` 已完成一次端到端验证：

- `/joint_states` 发布者只有 `joint_state_broadcaster`。
- `manual_front_pair` 任务 accepted。
- KDL IK 成功，约 `9ms`。
- MoveIt 规划成功，约 `0.11s`。
- 轨迹发送到 PLC mock。
- `stream` 模式下，编排层等待 mock PLC 约 `19–20s` 执行完成后才发布 done。
- `final_abs` 模式下，PLC/mock 命令数为 `1`，性质更接近 CLI `move-abs`，仅作为诊断对照，不作为正式轨迹执行方案。
- 任务发布层收到 `task completed`。


## 11. PLC 监听与 home 保持

一键栈默认启动 `plc_acceptance_supervisor.py`，它会订阅 `/plc_bridge_state` 和 `/alfa_task/status`，每秒汇总 PLC 状态，不需要盯多个终端。

关键日志：

```text
PLC monitor: mode=real state=normal executing=false last_commands=... home_commands=...
PLC state changed: mode=real state=executing executing=true ...
home hold command published #...
home hold stopped because task started: ...
```

默认只监听，不主动发 home。若需要启动后持续把机械臂校准到当前 home：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py \
  plc_mock:=false \
  velocity_limit_deg_s:=3.0 \
  plc_execution_mode:=final_abs \
  enable_home_hold:=true \
  home_hold_period_s:=1.0
```

安全规则：

- home hold 只在 PLC `state=normal` 且 `executing=false` 时发。
- 收到任务 `accepted/running` 后默认自动停止 home hold，避免和任务轨迹抢 `/plc_joint_trajectory`。
- `home_hold_max_commands:=N` 可限制最多发 N 次；默认 `0` 表示不限制，直到任务开始。
- 当前 home 是 `updown=0.18m`，双臂 12 轴全 `0°`。

只想单独开监听：

```bash
ros2 run alfa_robot_benchmarks plc_acceptance_supervisor.py
```

只想单独开监听 + home 校准：

```bash
ros2 run alfa_robot_benchmarks plc_acceptance_supervisor.py --ros-args \
  -p enable_home_hold:=true \
  -p home_period_s:=1.0 \
  -p max_home_commands:=5
```


中间隔板可调参数示例：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py \
  enable_center_separation_plate:=true \
  center_plate_x_min:=0.4 \
  center_plate_x_max:=0.76 \
  center_plate_y_thickness:=0.001
```

## 12. 保险演示模式

### A. MoveIt 规划固定 12 轴目标

一键栈保持启动，但让编排层跳过 IK，直接把固定 joint target 交给 MoveIt 规划，再发给 PLC bridge：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py \
  plc_mock:=false \
  velocity_limit_deg_s:=5.0 \
  plc_execution_mode:=stream \
  demo_mode:=fixed_joint_target \
  enable_center_separation_plate:=true
```

固定目标是 ROS/MoveIt 方向：

```text
left:  -28, 51, -38, 30, -81, 83 deg
right:  28, 49, -35, -28, -79, -85 deg
```

然后仍然用任务发布层回车触发：

```bash
ros2 run alfa_robot_benchmarks mock_box_task_publisher
```

### B. 直接 PLC CLI 伪任务

完全绕过 IK/MoveIt/ROS 执行层，回车后直接调用 PLC CLI。现在每按一次回车会在 `HOME(全 0)` 和 `TARGET(演示姿态)` 之间切换，第一次回车去 TARGET，第二次回车回 HOME：

```bash
ros2 run alfa_robot_benchmarks fake_plc_cli_task_publisher.py --ros-args \
  -p velocity:=5.0
```

实际 TARGET 命令等价于：

```bash
python3 -m alfa_robot_plc_driver.cli move-abs \
  --targets 1:-28,2:51,3:38,4:30,5:81,6:83,7:28,8:-49,9:-35,10:-28,11:-79,12:-85 \
  --vel 3 --yes-write
```

注意：PLC CLI 目标使用 PLC/电控方向；MoveIt 固定目标使用 ROS/MoveIt 方向，所以第 3、5、8 轴符号不同。

### C. PLC CLI 01020102 循环演示

`fake_plc_cli_task_publisher.py` 当前会在启动时先发一次 HOME，然后每次回车按以下顺序循环：

```text
TARGET1 -> HOME -> TARGET2 -> HOME -> TARGET1 -> HOME -> TARGET2 -> HOME ...
```

默认速度：

```text
HOME:    15
TARGET1: 60
TARGET2: 15
```

运行：

```bash
ros2 run alfa_robot_benchmarks fake_plc_cli_task_publisher.py
```

临时改速度：

```bash
ros2 run alfa_robot_benchmarks fake_plc_cli_task_publisher.py --ros-args \
  -p target1_velocity:=60.0 \
  -p target2_velocity:=15.0 \
  -p home_velocity:=15.0
```

TARGET2 默认值：

```text
1:16,2:15,3:-119,4:-160,5:46,6:-104,7:33,8:-7,9:111,10:56,11:46,12:40
```
