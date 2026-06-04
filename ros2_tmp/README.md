# ros2_tmp 固定平台双臂验收临时工作区

这个 workspace 只保留明天固定平台双臂验收需要的最小 ROS2 包集：

- `alfa_robot_description`：当前 v6 双臂模型和必要 mesh，仅保留 `alfa_robot_v2_arm_v6`。
- `alfa_robot_moveit_config`：MoveIt/RViz 层使用的配置和规划入口。
- `bio_ik`：当前 IK 服务依赖的双末端 BioIK plugin。
- `ik_benchmark` / `alfa_robot_benchmarks`：临时 IK 服务、任务发布层、任务编排层。
- `alfa_robot_plc_bridge`：电控执行层和急停层，默认 mock PLC。

不包含雷达、导航、感知、twist mux、硬件包和历史 benchmark 数据。

## 编译

建议避免 conda 库污染链接：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONPATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 启动临时流程

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_flow.launch.py
```

这个命令只启动系统底座，默认不自动发任务，便于联调时确认每层状态后再手动放行。

当前总 launch 会启动：

- `move_group`。
- 临时 MoveIt joint planner：`/alfa_moveit/plan_joint_target`。
- MuJoCo cargo box 参考障碍，写入 MoveIt PlanningScene。
- IK 服务：`/alfa_dual_ik/solve`。
- 任务编排层；模拟任务发布层默认不启动。
- 电控执行层和急停层，默认 `plc_mock:=true`。

确认系统状态正常后，单独启动任务发布层：

```bash
ros2 run alfa_robot_benchmarks mock_box_task_publisher --ros-args -p x_offset:=0.76
```

然后在任务发布层终端按回车：

1. 发布箱子 `5/6` 的双目标任务。
2. 收到 `accepted` 后停止重复发送。
3. 收到 `done` 后自动发布箱子 `7/8` 的双目标任务。

如果确实想一键启动并自动带上任务发布层，可以加：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_flow.launch.py with_task_publisher:=true
```

## 层间接口

- 任务发布 topic：`/alfa_task/command`，类型 `alfa_robot_benchmarks/msg/TaskCommand`。
- 任务状态 topic：`/alfa_task/status`，类型 `alfa_robot_benchmarks/msg/TaskStatus`。
- IK 服务：`/alfa_dual_ik/solve`，类型 `alfa_robot_benchmarks/srv/SolveDualIk`。
- MoveIt 规划服务：`/alfa_moveit/plan_joint_target`，类型 `alfa_robot_benchmarks/srv/PlanJointTarget`。当前由临时 planner 提供，后续同事正式 MoveIt 层接管时可用 `temporary_planner:=false` 关闭。
- 执行层输入：`/plc_joint_trajectory`，类型 `trajectory_msgs/msg/JointTrajectory`，由 `alfa_robot_plc_bridge` 执行。
- 急停层服务：`/alfa_safety/soft_stop`、`/alfa_safety/emergency_stop`、`/alfa_safety/reset`、`/alfa_safety/clear_commands`。
- 急停层状态：`/alfa_safety/state`。

## 单独启动执行 + 急停层

```bash
ros2 launch alfa_robot_plc_bridge execution_with_safety.launch.py mock:=true
```

常用急停命令：

```bash
ros2 service call /alfa_safety/soft_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/emergency_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/reset std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/clear_commands std_srvs/srv/Trigger {}
ros2 topic echo /alfa_safety/state
```

## 障碍可视化

RViz 方案：

```bash
ros2 launch alfa_robot_benchmarks planning_scene_visualization.launch.py
```

然后在 RViz 添加 `MarkerArray`，topic 选择：

```text
/alfa_visualization/planning_scene_markers
```

Rerun 方案：

```bash
ros2 launch alfa_robot_benchmarks planning_scene_visualization.launch.py use_rerun:=true
```

该节点订阅 MoveIt 的 `/planning_scene` 和 `/collision_object`，用于显示 MoveIt 里建立的障碍物；MoveIt 同事接入后无需改可视化层。

当前 launch 默认 `temporary_planner:=true`、`plc_mock:=true`，因此正式 MoveIt 层未接入时也能用临时 planner 验证任务链路，执行层默认连接 mock PLC。正式链路中，编排层会把 MoveIt 返回的 `JointTrajectory` 发布到 `/plc_joint_trajectory`，电控执行层负责按时间/控制周期写 PLC 或 mock PLC。

## 关键参数

- `fixed_updown:=0.18`：固定平台验收默认锁死 updown。
- `lock_updown:=true`：IK 内部也锁死 updown，保证返回的 12 轴 joint target 与固定平台真实状态一致。
- `workers:=16 h_candidate_count:=16 seed_count:=32 timeout:=0.01`：IK 候选求解默认配置。
- `x_offset:=0.76`：箱子前表面 x 坐标当前直接取该值，默认 `0.76m`。
- `temporary_planner:=false`：关闭临时 MoveIt planner，等待同事正式 MoveIt 服务提供 `/alfa_moveit/plan_joint_target`。
- `with_task_publisher:=true`：随总 launch 一起启动模拟任务发布层；默认 `false`，建议联调时单独启动。
- `plc_mock:=false`：执行层切到真实 PLC。

## 分层逐个启动方案

联调推荐不用总 launch，而是按层逐个开，确认一个层正常后再开下一层。

每个终端都先执行：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_tmp
source /opt/ros/humble/setup.bash
source install/setup.bash
```

终端 1：执行层 + 急停层，默认 mock PLC：

```bash
ros2 launch alfa_robot_plc_bridge execution_with_safety.launch.py mock:=true
```

终端 2：MoveIt / RViz 层：

```bash
ros2 launch alfa_robot_moveit_config demo.launch.py
```

如果只要 move_group、不需要 RViz，也可以用：

```bash
ros2 launch alfa_robot_moveit_config move_group.launch.py
```

终端 3：临时 MoveIt joint planner。等正式 MoveIt 层接管 `/alfa_moveit/plan_joint_target` 后，这个终端不要开：

```bash
ros2 launch alfa_robot_benchmarks temporary_moveit_joint_planner.launch.py
```

终端 4：IK 服务层：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_dual_ik_service.launch.py fixed_updown:=0.18 lock_updown:=true workers:=16 h_candidate_count:=16 seed_count:=32 timeout:=0.01 check_collision:=true fallback_enabled:=false
```

终端 5：任务编排层：

```bash
ros2 launch alfa_robot_benchmarks fixed_platform_task_orchestrator.launch.py fixed_updown:=0.18 mock_planner:=false wait_execution_done:=true
```

终端 6：任务发布层。这个终端由人手动控制，启动后按回车才发布任务：

```bash
ros2 run alfa_robot_benchmarks mock_box_task_publisher --ros-args -p x_offset:=0.76
```

终端 7：监控/急停：

```bash
ros2 topic echo /alfa_task/status
ros2 topic echo /plc_bridge_state
ros2 topic echo /alfa_safety/state
```

急停命令：

```bash
ros2 service call /alfa_safety/soft_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/emergency_stop std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/reset std_srvs/srv/Trigger {}
ros2 service call /alfa_safety/clear_commands std_srvs/srv/Trigger {}
```

推荐启动顺序：执行层/急停层 → MoveIt/RViz → 临时 planner 或正式 planner → IK 服务 → 任务编排层 → 任务发布层。这样任务不会在系统未准备好时自动发出。
