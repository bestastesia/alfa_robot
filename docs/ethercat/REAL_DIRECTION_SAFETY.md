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

**关于 `--max-joint-speed-deg-s` 上限取值 20.0（而不是更保守的 10.0）**：脚本当前默认值本身就是
20.0（`send_dual_grasp_sequence.py` 第 258 行 `default=20.0`），这是经项目负责人明确确认保留的值，
不是本次修复引入的放宽。若之后要收紧到更低的值，需要同时改脚本默认值和这里的硬上限常量，
并重新走一次仿真/mock验证。

`--hz` 硬上限 10.0 与 `execute_trajectory_service_node.py` 里 `resample_rate_hz` 参数默认值
（同为 10.0）保持同一口径，见下节。这两处如果要联动修改，必须同时改。

## 人工确认关卡

`execute_l6_r8_mock_live.py` 的 `require_explicit_confirmation()`：

- 只在 `executor_mode == "real"` 时触发（mock 模式不驱动硬件，不强制人工确认）。
- 非 tty 环境（被 subprocess/cron 调起、没有交互终端）直接拒绝执行，不会让 `EOFError`
  静默穿透变成"当作已确认"。
- 交互环境下必须手动输入指定 token（默认 `"YES"`），单纯回车不算确认。
- 触发点：进入负重姿态前一次，到达负重姿态、正式开始 L6/R8 任务执行前再一次。

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
| 其它 10 个关节 | 不翻转 |

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
  `args.max_joint_speed_deg_s`/`args.max_updown_speed_m_s` 转发给了下游 real-direct 脚本
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
