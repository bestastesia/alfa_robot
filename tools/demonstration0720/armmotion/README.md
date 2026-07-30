# demonstration0720 双线程机械臂演示

该目录是独立覆盖层，不修改 `/home/ar/lhy_dev` 中同事维护的代码。它只提供两个用户线程：

- `algorithm_thread`：按当前已验证的五任务规划器计算完整轨迹，并按六步门控执行。
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

先使用已验收的 rt-control 不可变发布副本启动真实控制域：

```bash
cd ~/rt-control-current
./tools/rt_control_ipc.sh
```

按现场要求确认并输入 `ENABLE_RT_CONTROL`。只有看到
`READY: rt-control 已启动并完成 /rt/enable。` 后，才允许启动本目录程序。
本目录不负责启动容器、切换控制器或调用 `/rt/enable`。
本目录默认使用与已验收控制容器一致的 `ROS_DOMAIN_ID=42`；如调用方已经显式设置
`ROS_DOMAIN_ID`，则保留调用方的值。
宿主机算法进程固定使用 Fast DDS UDPv4 数据面，避免 root 容器与普通用户进程之间的
共享内存文件权限导致“能发现 action、但 goal 无法送达”的假连通状态。算法规划器仅
启动 MoveIt `move_group` 和规划节点，禁止在真实控制域中再启动本地 `ros2_control`、
`robot_state_publisher` 或第二个 `/controller_manager`。

以下接口必须存在：

- `/dual_arm_jtc/follow_joint_trajectory`
- `/plc/left_solenoid`
- `/plc/right_solenoid`
- `/plc/vacuum_pump`
- `/joint_states`

第2步到达吸附位后同步打开左右电磁阀和真空泵；第5步同步关闭三路输出。

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

之后输入一个任务，例如 `A1`。规划完成后，连续六次直接回车，分别执行：

1. 负重位（updown=0.3m）到 IK 前 5cm。
2. 笛卡尔前进 5cm，到位后打开左右电磁阀和真空泵。
3. 抽离，然后回负重姿态；抽离终态高于 `0.45m` 时降到 `0.45m`，低于或等于 `0.45m` 时保持当前 updown，不额外上升。
4. 双臂和 updown 使用完整 `shortcut + 局部 RRT` 轨迹同步到放置位（updown=0.1m）。
5. 关闭左右电磁阀和真空泵。
6. 双臂和 updown 同步回负重位（updown=0.3m）。

### 轨迹速度合同

- 轨迹按 `30Hz` 固定频率发送，每个 `JointTrajectoryPoint` 都包含完整 14 轴：右臂6轴、左臂6轴、`turn`、`updown`，并同时携带完整的 `positions`、`velocities` 和 `accelerations`。
- 轨迹整形只合并同一直线上的冗余采样点；整段轨迹统一完成起步和停车，RRT 中间点不再拆成独立停车段。每个关节只在运动方向正负翻转时归零，同向转折保持连续速度并交由控制器插值。
- 当前双臂有效速度上限为 `30deg/s`，关节加速度上限为 `60deg/s²`；必要时自动延长该轨迹段，不再通过相邻位置差硬切速度。
- `/dual_arm_jtc` 命令和 `/joint_states` 反馈必须经过统一 `joint.py` 合同；四个实机反向轴在该边界补偿，编码器零点和 J6 偏置仍由 rt-control 硬件配置负责。
- `updown` 不再使用独立 PP 话题，而是和其他 13 轴一起进入同一个 FJT 目标；速度硬限制为 `0.15m/s`，加速度默认限制为 `0.05m/s²`。
- `turn` 始终读取并保持当前反馈值，但仍作为完整 14 轴目标的一部分发送，满足控制器禁止 partial goal 的合同。

输入 `D` 可修改两种距离，输入 `Q` 退出任务线程。算法线程启动时会一次性启动并预热长驻 planner；后续每个任务只重置 monitor 状态，并按任务原子更新距离、横向布局、抽离模式和动态碰撞场景，不再重复支付 MoveIt 冷启动成本。算法线程 `Ctrl-C` 时会一并关闭 planner 子进程。

## 真实控制器插值回放

工控机 EtherCAT 的 `joint_trajectory_controller` 以 `250Hz` 执行。当前轨迹点包含位置、速度和加速度，因此控制器在相邻点之间采用五次样条，而不是 Rerun 旧回放中的线性连接。以下命令用当前算法计算代表任务 `B1/A3/B5`，按同样的 `250Hz` 样条执行，再从真实控制 tick 中就近抽取约 `90Hz` 写入 RRD：

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

RRD 中全部 14 轴均显示 `dual_arm_jtc` 五次样条后的轨迹。

## 安全干运行

算法、六步门控和 ROS 接口联调时，不向控制器或 PLC 写命令：

```bash
./run_algorithm_thread.sh --dry-run
```

轨迹按受限速度曲线重新分布为 30Hz 采样，以原 10Hz 轨迹的 3 倍目标速度执行；双臂有效速度上限为 30°/s、加速度上限为 60°/s²，updown 有效速度上限为 0.15m/s。程序只允许通过 `alfa_robot_execution_bridge/joints.py` 获取 rt-control 的固定 14 轴顺序，不做原始电机语义转换。

真实执行前，每个轨迹段都会用 `/joint_states` 检查实际起点是否与规划起点一致。默认最大关节偏差 5 度、updown 偏差 15mm；不满足时拒绝执行，不会把当前位置强行跳到轨迹首帧。

## 手动示教

使用本目录入口，确保沿用 Domain 42、UDP 通信配置、`/dual_arm_jtc` 和统一 `joint.py` 合同：

```bash
./run_jog_to_pose.sh \
  --right-joint2-deg 10 \
  --left-joint2-deg 10 \
  --updown-m 0.20 \
  --duration-s 4
```

不加 `--send` 时只计算和打印轨迹；确认目标无误后再加 `--send`，程序仍会要求现场按回车确认。不要使用 `/home/ar/lhy_dev/run_rt_jog_to_pose.sh` 验证本目录的方向合同，该脚本在 rt-control 容器内直接发送，不读取本目录的 `joint.py`。
