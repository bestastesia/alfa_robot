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
- 朝向：`+X`
- 机器人固定 `updown=0.18m`

临时改目标坐标示例：

```bash
ros2 run alfa_robot_benchmarks mock_box_task_publisher --ros-args \
  -p task_id:=manual_front_pair \
  -p left_x:=0.70 -p left_y:=0.20 -p left_z:=1.40 \
  -p right_x:=0.70 -p right_y:=-0.20 -p right_z:=1.40
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


## 5. 当前几何与障碍口径

- 箱垛：2 列 × 4 行。
- 两列中心对称在机器人中线两侧：`y=+0.20` 和 `y=-0.20`。
- 箱子尺寸：`0.3m × 0.4m × 0.4m`。
- 箱子前侧面：`x=0.76m`。
- 箱子中心：`x=0.91m`。
- 末端规划点：`x=0.70m`，比箱子前侧面安全退让约 `6cm`。
- 障碍不启动 MuJoCo，只按 MuJoCo/仿真参数静态注入 MoveIt PlanningScene。

## 6. 当前初始姿态

- `updown=0.18m`。
- 左右臂 home：`0°, 5°, 145°, 0°, 120°, 0°`。
- PLC bridge 不再发布 `/joint_states`；`/joint_states` 只采用 demo/ros2_control 的状态。

检查 `/joint_states` 发布者：

```bash
ros2 topic info /joint_states -v
```

期望：`Publisher count: 1`，发布者为 `joint_state_broadcaster`。

## 7. 各线程数据流

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

## 8. 验收时重点看

终端 1 应出现类似日志：

```text
Applied 8 static 2-column x 4-row acceptance boxes: front_x=0.76 center_x=0.91
Temporary MoveIt joint planner ready: group=dual_v5_arm_with_base ... updown=0.180
KDL IK success h=0.180 ... left=SUCCESS right=SUCCESS
Plan success task=manual_front_pair ...
trajectory finished: commands=... duration=...
Task manual_front_pair: done - task completed
```

monitor 里重点看：

```text
IK -> planner request: 全部 0
planner trajectory last -> PLC trajectory last: 全部 0
planner target -> planner trajectory last: 小于 1deg
```

## 9. 已验证结果

本机在 `2026-06-05` 已完成一次端到端验证：

- `/joint_states` 发布者只有 `joint_state_broadcaster`。
- `manual_front_pair` 任务 accepted。
- KDL IK 成功，约 `9ms`。
- MoveIt 规划成功，约 `0.11s`。
- 轨迹发送到 PLC mock。
- `stream` 模式下，编排层等待 mock PLC 约 `19–20s` 执行完成后才发布 done。
- `final_abs` 模式下，PLC/mock 命令数为 `1`，性质更接近 CLI `move-abs`，仅作为诊断对照，不作为正式轨迹执行方案。
- 任务发布层收到 `task completed`。

(base) li@li-Legion-R9000P-ARX8:/mnt/mydisk/ALFA/alfa_robot_ec/tools/alfa_robot_plc_driver$ python3 -m alfa_robot_plc_driver.cli move-abs --targets 1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,10:0,11:0,12:0 --vel 3 --yes-write