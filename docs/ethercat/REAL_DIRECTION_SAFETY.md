# L6/R8 实机方向与运行时安全基准

日期：2026-07-16（重写，反映当前实际架构；上一版日期 2026-06-30）

## 当前实机执行路径（重要：不是"服务编排"那一套）

L6/R8 任务序列的默认真实执行路径是：

```text
scripts/lhy_dev/send_dual_grasp_sequence.py  （--execute-backend planner-live，默认值）
  -> subprocess 调用 execute_l6_r8_real_live.py（工控机上的转发脚本）
    -> execute_l6_r8_mock_live.py --executor-mode real --ros-domain-id inherit ...
      -> 直接向 /dual_arm_trajectory_controller/follow_joint_trajectory 发 FollowJointTrajectory action
```

这条路径**绕开**了服务编排层（`RunDualGraspTask` → `dual_grasp_task_adapter_node` →
`RunDualArmPoseTask` → `motion_task_orchestrator_node` → `PlanExtract`/`PlanLoaded`/
`ExecuteTrajectory` 服务链 → `execute_trajectory_service_node` → action）。
`--execute-backend service` 是另一条路径，走上面这条服务编排链，但**当前不是默认值**，
且线上实际从未切换到过它。

**结论：这份文档描述的所有硬上限、方向映射、负重姿态锁，都要在 `execute_l6_r8_mock_live.py`
本身里强制生效，不能指望编排层或某个 wrapper shell 脚本来兜底。** 旧版文档提到的
`/home/ar/lhy_dev/run_l6_r8_real.sh` 现在不是唯一入口；真正的入口点是
`send_dual_grasp_sequence.py` 的 `run_planner_live_task()` 拼出的 `execute_l6_r8_real_live.py`
命令行。

## 运行时硬上限（写在 execute_l6_r8_mock_live.py 里，real direct 模式生效）

以下常量定义在 `execute_l6_r8_mock_live.py` 顶部，只在 `real_direct`
（即 `executor_mode == "real" and not start_execution_bridge`）时生效：

| 参数 | 硬上限 | 常量名 | 覆盖用环境变量 |
| --- | --- | --- | --- |
| `--max-joint-speed-deg-s` | 20.0 | `MAX_SAFE_REAL_JOINT_SPEED_DEG_S` | `ALFA_ALLOW_UNSAFE_SPEED_OVERRIDE=I_UNDERSTAND_SPEED_RISK` |
| `--hz` | 10.0 | `MAX_SAFE_REAL_HZ` | `ALFA_ALLOW_UNSAFE_HZ_OVERRIDE=I_UNDERSTAND_HZ_RISK` |
| `--max-updown-speed-m-s`（仅 `--send-updown` 时检查） | 0.05 | `MAX_SAFE_REAL_UPDOWN_SPEED_M_S` | `ALFA_ALLOW_UNSAFE_UPDOWN_SPEED_OVERRIDE=I_UNDERSTAND_UPDOWN_SPEED_RISK` |
| `--loaded-preferred-pose-index` | 必须为 0 | 无（直接比较 `!= 0`） | `ALFA_ALLOW_UNSAFE_LOADED_POSE_OVERRIDE=I_UNDERSTAND_LOADED_POSE_RISK` |
| 关闭方向映射（`--no-real-apply-direction-signs`） | 禁止 | 无 | `ALFA_ALLOW_UNSAFE_DIRECTION_OVERRIDE=I_UNDERSTAND_DIRECTION_RISK` |

超过上限或触发禁止项时，脚本用 `SystemExit` 直接拒绝启动，不会静默放行。

`--max-joint-speed-deg-s` 的实机默认值为 10.0，轨迹仍按 10Hz 下发。20.0 只保留为必须显式指定的
硬上限，不再是默认测试速度。

`--hz` 硬上限 10.0 与 `execute_trajectory_service_node.py` 里 `resample_rate_hz` 参数默认值
（同为 10.0）保持同一口径，见下节。这两处如果要联动修改，必须同时改。

## 人工确认关卡

`execute_l6_r8_mock_live.py` 的 `wait_for_enter_confirmation()`：

- 只在 `executor_mode == "real"` 时触发（mock 模式不驱动硬件，不强制人工确认）。
- 非 tty 环境（被 subprocess/cron 调起、没有交互终端）直接拒绝执行，不会让 `EOFError`
  静默穿透变成"当作已确认"。
- 交互环境下直接按回车确认，不再要求输入 `YES`。
- 触发点：进入初始化负重姿态前、预接触且人工确认 updown 到位后、IK 吸附点确认后、放货姿态确认箱子释放后。
- 初始化负重姿态的 `updown` 目标独立固定为逻辑/URDF `0.30m`。程序读取当前物理反馈并经
  `physical_to_logical_updown()` 转换后，与双臂轨迹同步运动到 `0.30m`；不能再把当前 IK
  参考高度 `fixed_updown` 当作负重初始化高度。
- 2026-07-19 实机复测确认 `UPDOWN_PHYSICAL_ZERO_OFFSET_M=0.0`，因此 logical 与 physical
  数值相同、范围均为 `[0,0.7]m`。转换函数和范围检查仍是强制合同，不允许旁路。
- 2026-07-19 电控合同升级后，`/canopen/updown_position_controller/commands` 必须固定发送
  `[position_m, velocity_mps, acceleration_mps2, deceleration_mps2]`。旧的单元素位置命令会被
  runtime 拒绝。全流程使用 `updown.py` 构造原子四字段命令，每个阶段只下发一次最终 PP 目标，
  速度按阶段位移/时长计算并受 `--max-updown-speed-m-s` 限制。
- 抽离回到负重姿态后，默认追加“负重→放货→负重”循环。放货姿态采用 ROS/URDF 语义：
  双臂均为 `[0,-55,-50,-60,0,0]°`，`turn=0°`，`updown=0.10m`；去程保留两只附着箱，
  回程按箱子已释放处理。两段均先做全场景直连碰撞校验，仅碰撞区间才调用局部 RRT 修补。
- 外层 `run_13_dual_grasp_tasks.sh --execute --yes-execute` 的命令行安全开关仍保留；
  `--yes-execute` 不是运行过程中的交互 token。

## 服务编排路径（`--execute-backend service`）里 velocity_scale/acceleration_scale 的现状

如果切到 `--execute-backend service`：`RunDualGraspTask`/`RunBoxPairTask` 等 srv 里的
`velocity_scale`/`acceleration_scale` 字段，经过 `dual_grasp_task_adapter_node.py`/
`box_pair_task_adapter_node.py`（用 `common.py` 里的 `clamp_motion_scale()` 统一 clamp 到
`[0.0, 1.0]`）、`motion_task_orchestrator_node.py` 逐层转发后，最终到达
`execute_trajectory_service_node.py` 的 `on_execute()`：

- 会做范围校验（拒绝 `<0.0` 或 `>1.0` 的值），并在非零时打印 warning。
- **但目前仍然不会真正改变发给 `FollowJointTrajectory` action 的轨迹时序**——唯一实际影响
  执行节奏的是 `resample_rate_hz`（10Hz 契约）。也就是说，调这两个 scale 目前不会让机器人跑
  更快或更慢，只是被接受和记录，不生效。

这条路径当前不是默认执行路径，风险敞口有限，但如果以后要切换默认 backend 或有人直接调用
service 路径，必须先解决这个"参数被接受但不生效"的问题，否则调用方会误以为已经生效。

## 方向映射唯一真相源（未变）

仓库内方向映射唯一真相源仍是：

```text
ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge/joints.py
```

其中 `ROS_TO_ETHERCAT_SIGN_BY_JOINT` 定义 `ROS/Rerun 语义角度 -> 实机 EtherCAT 指令角度`。

规则：

- 不要在执行脚本、launch、YAML 或临时测试脚本里复制方向表。
- `execute_l6_r8_mock_live.py` 必须从 `alfa_robot_execution_bridge.joints` 导入 joint 顺序和方向转换函数。
- `alfa_robot_execution_bridge/config/*.yaml` 只允许配置是否应用方向映射，不允许复制 `direction_signs`。
- 如果实机方向重新标定，只改 `joints.py`，然后运行 `scripts/safety/check_l6_r8_real_safety.py`。

当前已验证正确的 direct-real 方向规则：

| 关节 | 软件方向处理 |
| --- | --- |
| left_joint3 | 翻转 |
| left_joint5 | 翻转 |
| right_joint2 | 翻转 |
| right_joint4 | 翻转 |
| 其它 9 个关节 | 不翻转 |

负重姿态族：`loaded_preferred_pose_index=1` 对应 `[-75, 135, 60]` 肘型（历史上出现过方向反的
问题源头之一）；当前实机唯一确认方向正确的是 `loaded_preferred_pose_index=0`。

## 禁止事项

不要在实机任务流程（无论走 `send_dual_grasp_sequence.py` 还是直接调
`execute_l6_r8_mock_live.py`）中临时追加或恢复：

```bash
--no-real-apply-direction-signs
--loaded-preferred-pose-index 1
--hz <更高频率，不设 ALFA_ALLOW_UNSAFE_HZ_OVERRIDE>
--max-joint-speed-deg-s <超过 20，不设 ALFA_ALLOW_UNSAFE_SPEED_OVERRIDE>
--max-updown-speed-m-s <超过 0.05，不设 ALFA_ALLOW_UNSAFE_UPDOWN_SPEED_OVERRIDE>
```

正确、安全的取值始终是 `--loaded-preferred-pose-index 0`（配合 `--real-apply-direction-signs`，
即不加 `--no-real-apply-direction-signs`），这是当前唯一在实机上确认过方向的组合。

如需诊断方向，使用独立小角度单轴测试脚本，不要改 L6/R8 实机任务脚本。

## 当前安全校验脚本

```bash
scripts/safety/check_l6_r8_real_safety.py
```

这是仓库内的静态 AST 扫描脚本，**不会运动机器人**，只读文件内容，检查：

- `joints.py` 里的方向符号表 `ROS_TO_ETHERCAT_SIGN_BY_JOINT` 没有被改。
- `execute_l6_r8_mock_live.py` 没有自建方向表，而是从 `joints.py` 导入。
- `execute_l6_r8_mock_live.py` 负重姿态族 index 0 的角度值、`loaded_joint_map` 默认 index 没有被改。
- `execute_l6_r8_mock_live.py` 保留了 `--loaded-preferred-pose-index`、
  `--no-real-apply-direction-signs` 及对应的 `ALFA_ALLOW_UNSAFE_*_OVERRIDE` 拒绝护栏。
- `dual_arm_planner_node.cpp`/launch 文件里的 `loaded_preferred_pose_index` 默认值仍是 0。
- 本文档本身列出的关键关节/参数没有从文档里消失。
- **（新增）** `send_dual_grasp_sequence.py` 的 `--execute-backend` 默认值仍是 `planner-live`，
  `--yes-execute` 确认护栏还在，`--max-joint-speed-deg-s`/`--max-updown-speed-m-s`/`--hz`
  的默认值没有被静默改动，且 `run_planner_live_task()` 确实把 `args.hz`/
  `args.max_joint_speed_deg_s`/`args.max_updown_speed_m_s`/升降加减速度转发给了下游 real-direct 脚本
  （即上限校验不会因为参数没传下去而被绕过）。

工控机上原来提到的 `/home/ar/lhy_dev/verify_l6_r8_direction_safety.sh` 如果还存在，可以继续
作为补充只读校验跑一遍，但仓库侧的唯一权威校验入口是上面这个 Python 脚本。

## 复测建议

如果未来改了 controller/hardware 方向处理、负重姿态族、`send_dual_grasp_sequence.py` 的
默认参数，或 `execute_trajectory_service_node.py` 的 `resample_rate_hz`，先运行：

```bash
python3 scripts/safety/check_l6_r8_real_safety.py
```

再用小角度单轴测试确认方向；不要直接跑 L6/R8 全流程验证方向。
