# demonstration0720 双线程机械臂演示

该目录是独立覆盖层，不修改 `/home/ar/lhy_dev` 中同事维护的代码。它只提供两个用户线程：

- `algorithm_thread`：按当前已验证的五任务规划器计算完整轨迹，并按七步门控执行。
- `task_thread`：确认侧吸/顶吸距离，发送 `A1..A5` 或 `B1..B5`，每次回车执行下一步。

`A` 是横向偏移 5cm 布局，`B` 是居中布局；数字 `1..5` 对应从上到下：

| 编号 | 箱子 | 吸附 | 抽离 |
|---|---|---|---|
| 1 | L1/R3 | 侧吸 | 双臂箱体位姿 RRT，updown 固定 |
| 2 | L4/R6 | 侧吸 | 双臂箱体位姿 RRT，updown 固定 |
| 3 | L7/R9 | 侧吸 | 双臂固定，updown 直升 0.4m |
| 4 | L10/R12 | 顶吸 | 双臂固定，updown 直升 0.4m |
| 5 | L13/R15 | 顶吸 | 双臂固定，updown 直升 0.4m |

## 前置条件

工控机的驱动、双臂 FollowJointTrajectory 控制器和 PLC 节点已经启动。以下接口必须存在：

- `/dual_arm_trajectory_controller/follow_joint_trajectory`
- `/canopen/updown_position_controller/commands`
- `/plc/left_solenoid`
- `/plc/right_solenoid`
- `/joint_states`

第2步到达吸附位后打开左右电磁阀；第6步关闭左右电磁阀。本程序不读取也不修改真空泵状态。

## 启动

终端一：

```bash
cd /home/ar/demostration0720/src/armmotion
./run_algorithm_thread.sh
```

终端二：

```bash
cd /home/ar/demostration0720/src/armmotion
./run_task_thread.sh
```

任务线程启动后，第一次回车先把机器人从当前反馈姿态运动到负重初始位：双臂均为 `[0,-45,120,-75,0,0]deg`，updown 为 `0.3m`。`turn` 始终保持启动时的真实角度，不参与规划、不发生运动。初始化完成后再确认侧吸和顶吸距离。

之后输入一个任务，例如 `A1`。规划完成后，连续七次直接回车，分别执行：

1. 负重位（updown=0.3m）到 IK 前 5cm。
2. 笛卡尔前进 5cm，到位后打开左右电磁阀。
3. 抽离，然后回负重姿态；抽离终态高于 `0.45m` 时降到 `0.45m`，低于或等于 `0.45m` 时保持当前 updown，不额外上升。
4. updown 单独下降到 0.1m。
5. 双臂到放置位。
6. 关闭左右电磁阀。
7. 双臂和 updown 同步回负重位（updown=0.3m）。

### 轨迹速度合同

- 双臂轨迹按 `30Hz` 固定频率发送，每个 `JointTrajectoryPoint` 同时包含完整的 `positions` 和 `velocities`。
- 轨迹整形只合并同一直线上的冗余采样点；整段轨迹统一完成起步和停车，RRT 中间点不再拆成独立停车段。每个关节只在运动方向正负翻转时归零，同向转折保持连续速度并交由控制器插值。
- 当前双臂有效速度上限为 `30deg/s`，关节加速度上限为 `60deg/s²`；必要时自动延长该轨迹段，不再通过相邻位置差硬切速度。
- 关节方向转换同时作用于位置和速度，但 J6 的零位偏置只作用于位置，绝不会叠加到速度。
- updown 仍使用独立 PP 位置控制，只发送阶段终点及四字段 profile；其阶段时长继续与双臂轨迹同步计算。

输入 `D` 可修改两种距离，输入 `Q` 退出任务线程。算法线程启动时会一次性启动并预热长驻 planner；后续每个任务只重置 monitor 状态，并按任务原子更新距离、横向布局、抽离模式和动态碰撞场景，不再重复支付 MoveIt 冷启动成本。算法线程 `Ctrl-C` 时会一并关闭 planner 子进程。

## 真实控制器插值回放

工控机 EtherCAT 的 `joint_trajectory_controller` 以 `250Hz` 执行。当前轨迹点包含位置和速度，因此控制器在相邻点之间采用三次 Hermite 样条，而不是 Rerun 旧回放中的线性连接。以下命令用当前算法计算代表任务 `B1/A3/B5`，按同样的 `250Hz` 样条执行，再从真实控制 tick 中就近抽取约 `90Hz` 写入 RRD：

```bash
./build.sh
./run_controller_interpolated_rerun.sh
```

自选任务和输出文件：

```bash
./run_controller_interpolated_rerun.sh \
  --tasks B1,A3,B5 \
  --save /tmp/controller_interpolated_selected_tasks.rrd
```

RRD 中双臂显示的是控制器样条后的轨迹；updown 不属于 EtherCAT `joint_trajectory_controller`，仍按当前独立 PP 规划时序显示。

## 安全干运行

算法、七步门控和 ROS 接口联调时，不向控制器或 PLC 写命令：

```bash
./run_algorithm_thread.sh --dry-run
```

双臂轨迹按受限速度曲线重新分布为 30Hz 采样，以原 10Hz 轨迹的 3 倍目标速度执行；双臂有效速度上限为 30°/s、加速度上限为 60°/s²。程序只允许通过 `alfa_robot_execution_bridge/joints.py` 做实机关节顺序、方向和 updown 零点转换。updown 使用四字段生产命令，随时间缩放同步从原 0.05m/s 上限提高到 0.15m/s。

真实执行前，每个轨迹段都会用 `/joint_states` 检查实际起点是否与规划起点一致。默认最大关节偏差 5 度、updown 偏差 15mm；不满足时拒绝执行，不会把当前位置强行跳到轨迹首帧。
