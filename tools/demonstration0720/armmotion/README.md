# demonstration0720 Motion 联调工具

> 本目录根部的 `algorithm_thread` / `task_thread` 是历史实机双线程演示，会直接操作 PLC，仅用于旧实验复现。正式 Motion 域入口是 `/motion/execute_stage`，不拥有电磁阀、真空泵或真空阈值控制权；正式边界见 `docs/运控/Motion域Docker开发联调.md`。

正式入口只有一个 Motion 服务：`./run_motion_domain.sh --mock|--hardware`。`./run_manual_motion_task.sh` 只是开发阶段模拟 Autonomy 的 Action 客户端，不属于生产运行进程。

正式 Action 的侧吸目标是实际正面吸附面中心，顶吸目标是实际顶面吸附面中心。若只发送
一侧目标并将另一侧标为 `NO_MOVE`，Motion 会把有效 6D Pose 的 Y 取负镜像到另一臂，
观察阶段和抓取阶段优先使用修正后的左右独立 6D 目标在线解算；左右同时 `NO_MOVE` 会被拒绝。
正式流程只有在线规划失败后才降级查找轨迹缓存。Turn=0 换算默认不再叠加固定 Y 向补偿，`turn_zero_target_y_compensation_m=0.0`。
正式抓取目标保留感知 XYZ，但按吸附模式把姿态强制投影到标准侧吸或顶吸姿态；重拍目标姿态不做该处理。

该目录是独立覆盖层，不修改 `/home/ar/lhy_dev` 中同事维护的代码。它只提供两个用户线程：

- `algorithm_thread`：只消费左右箱体正面中心 6D 位姿，内部识别排数、吸附方式和策略，计算完整轨迹并按六步门控执行。
- `task_thread`：测试用位姿生产者。确认侧吸/顶吸距离，用 `A1..A5` 或 `B1..B5` 生成左右正面中心 6D 位姿，每次回车执行下一步；任务编号和箱号不会发给算法线程。

`A` 是横向偏移 5cm 布局，`B` 是居中布局；数字 `1..5` 对应从上到下：

| 编号 | 箱子 | 吸附 | 抽离 |
|---|---|---|---|
| 1 | L1/R3 | 侧吸 | 双臂箱体位姿 RRT，updown 固定 |
| 2 | L4/R6 | 侧吸 | 双臂箱体位姿 RRT，updown 固定 |
| 3 | L7/R9 | 侧吸 | 双臂固定，updown 直升 0.4m |
| 4 | L10/R12 | 顶吸 | 箱体位姿 RRT，优先上抬并允许双臂与 updown 联动 |
| 5 | L13/R15 | 顶吸 | 箱体位姿 RRT，优先上抬并允许双臂与 updown 联动 |

## 前置条件

当前实机使用 Native rt-control。先由控制域负责人启动并使能，再只读确认状态：

```bash
cd ~/rt-control-dev/robot
./tools/rt_control_native.sh status
```

只有确认控制器已使能且 `/whole_body_jtc/follow_joint_trajectory` 存在服务端后，才允许启动 Motion。
本目录不负责启动 Native rt-control、切换控制器或调用 `/rt/enable`。
本目录默认使用 Native rt-control 的 `ROS_DOMAIN_ID=0`；如调用方已经显式设置
`ROS_DOMAIN_ID`，则保留调用方的值。
宿主机算法进程固定使用 Fast DDS UDPv4 数据面。算法规划器仅
启动 MoveIt `move_group` 和规划节点，禁止在真实控制域中再启动本地 `ros2_control`、
`robot_state_publisher` 或第二个 `/controller_manager`。

以下接口必须存在：

- `/whole_body_jtc/follow_joint_trajectory`
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

任务线程启动后，第一次回车先把机器人从当前反馈姿态运动到负重初始位：双臂均为 `[0,-45,120,-75,0,0]deg`，updown 为 `0.3m`。Motion 不检测、规划或改变 `turn`。初始化完成后再确认侧吸和顶吸距离。

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
- `/whole_body_jtc` 命令和 `/joint_states` 反馈必须经过统一 `joints.py` 合同；所有方向符号均为正，电机方向与编码器零点由 rt-control 负责，Motion 不重复校准。
- `updown` 不再使用独立 PP 话题，而是和其他 13 轴一起进入同一个 FJT 目标；速度硬限制为 `0.15m/s`，加速度默认限制为 `0.05m/s²`。
- Motion 不拥有 `turn`。由于控制器禁止 partial goal，执行适配器仅在完整14轴消息中复制最新 `turn` 反馈并发送零速度、零加速度；不存在主动命令 Turn 的接口。

输入 `D` 可修改两种距离，输入 `Q` 退出任务线程。算法线程启动时会一次性启动并预热长驻 planner；后续每个任务只重置 monitor 状态，并按任务原子更新距离、横向布局、抽离模式和动态碰撞场景，不再重复支付 MoveIt 冷启动成本。算法线程 `Ctrl-C` 时会一并关闭 planner 子进程。

### 任务消息合同

`/armmotion/task_request` 的业务字段固定为：

```yaml
request_id: string
left:
  pose_6d: {x, y, z, roll, pitch, yaw}
right:
  pose_6d: {x, y, z, roll, pitch, yaw}
```

两个 `pose_6d` 都是 `base_link` 下箱体朝向机器人的正面中心位姿，不是吸盘接触位姿。
历史双线程算法不接收任务编号或箱号。正式 `/motion/execute_stage` 的 6D 目标是实际吸附面
中心，并通过 `grasp_mode` 区分侧吸和顶吸。Motion 收到左右目标后，先按实时 Turn 换算到 Turn=0，再按各自侧吸/顶吸模式
归一末端朝向；若左右目标高度不同，则保留各自 `x/y`，并把两者 `z` 对齐到较低目标。
当前五排箱高为 `0.4m`，算法使用最近排中心匹配，
默认允许高度残差 `±0.12m`；落在排间模糊区的输入会明确拒绝。相机和停车造成的合法
`x/y` 及双臂分别的深度偏差会保留到 IK、附着箱和动态箱墙计算中，不会吸附到标称网格位置。

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

RRD 中全部 14 轴均显示 `whole_body_jtc` 五次样条后的轨迹。

## 安全干运行

算法、六步门控和 ROS 接口联调时，不向控制器或 PLC 写命令：

```bash
./run_algorithm_thread.sh --dry-run
```

轨迹按受限速度曲线重新分布为 30Hz 采样，以原 10Hz 轨迹的 3 倍目标速度执行；双臂有效速度上限为 30°/s、加速度上限为 60°/s²，updown 有效速度上限为 0.15m/s。程序只允许通过 `alfa_robot_execution_bridge/joints.py` 获取 rt-control 的固定 14 轴顺序，不做原始电机语义转换。

真实执行前，每个轨迹段都会用 `/joint_states` 检查实际起点是否与规划起点一致。默认最大关节偏差 5 度、updown 偏差 15mm；不满足时拒绝执行，不会把当前位置强行跳到轨迹首帧。

## 手动示教

使用本目录入口，确保沿用 Domain 0、UDP 通信配置、`/whole_body_jtc` 和统一 `joints.py` 合同：

```bash
./run_jog_to_pose.sh \
  --right-joint2-deg 10 \
  --left-joint2-deg 10 \
  --updown-m 0.20 \
  --duration-s 4
```

不加 `--send` 时只计算和打印轨迹；确认目标无误后再加 `--send`，程序仍会要求现场按回车确认。不要使用旧 `/home/ar/lhy_dev/run_rt_jog_to_pose.sh` 验证本目录的方向合同，该脚本不读取当前 Motion 的 `joints.py`。

## 30组 Action 诊断样例

输出全部真实 Goal 到 JSON，不发送轨迹：

```bash
./run_dump_action_examples.sh --output data/motion_action_examples_30.json
```

输出单组可直接检查或手工发送的 Action 命令：

```bash
./run_dump_action_examples.sh --distance-cm 72 --row 3 --format commands
```

单臂真实输入样例：

```bash
./run_dump_action_examples.sh \
  --distance-cm 72 --row 3 --single-arm left --format commands
```
