# Motion 域 Docker 开发联调

## 1. 职责边界

Motion 只负责：接收运动阶段、计算/缓存计划、下发完整十四轴轨迹、发布阶段结果和就绪状态。

Motion 不负责：打开/关闭电磁阀、控制真空泵、判定真空阈值、启动或使能 rt-control。上述吸附流程由 Autonomy 编排 RT-Control。

## 2. 数据流

```text
Autonomy/测试客户端
        |
        | /motion/execute_stage
        v
Motion 域服务
  - 阶段顺序校验
  - 长驻 Planner / PlanningScene
  - 轨迹生成与执行
        |
        | /dual_arm_jtc/follow_joint_trajectory
        v
rt-control Docker
        |
        +---- /joint_states ----> Motion

Autonomy -----------------------> RT-Control 吸附接口
```

## 3. 对外接口

| ROS 名称 | 类型 | 生产者 → 消费者 |
|---|---|---|
| `/motion/execute_stage` | `alfa_motion_interfaces/action/ExecuteMotionStage` | Autonomy/测试客户端 → Motion |
| `/motion/readiness` | `alfa_motion_interfaces/msg/MotionReadiness` | Motion → Autonomy/观测工具 |
| `/dual_arm_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion → RT-Control |
| `/joint_states` | `sensor_msgs/msg/JointState` | RT-Control → Motion |

阶段固定为：

1. `MOVE_TO_RECAPTURE`
2. `MOVE_TO_PREGRASP`
3. `APPROACH_SUCTION`
4. `MOVE_TO_PLACE`
5. `RETURN_INITIAL`

`MOVE_TO_PREGRASP` 必须提供左右箱体正面中心 `base_link` 位姿；算法内部判断箱体排数、吸附方式和抽离策略。后续阶段必须使用同一 `task_id` 且严格按顺序调用。吸附和释放的确认不混入 Motion Action，由 Autonomy 在阶段之间等待 RT-Control 回执。

### 阶段语义

| 阶段 | Motion 执行内容 | 完成后的外部动作 |
|---|---|---|
| `MOVE_TO_RECAPTURE` | 从当前状态运动到第一批左右重拍末端 6D 位姿 | Autonomy 触发感知精定位 |
| `MOVE_TO_PREGRASP` | 根据左右正面中心 6D 位姿计算完整计划，并执行到预抓取位 | Autonomy 请求下一阶段 |
| `APPROACH_SUCTION` | 从预抓取位沿接触方向靠近吸附位；Motion 不打开气路 | Autonomy 命令 RT-Control 打开吸附通路并等待真空条件 |
| `MOVE_TO_PLACE` | 执行抽离、负重过渡、预放置和放置轨迹；Motion 不关闭气路 | Autonomy 命令 RT-Control 释放并等待释放条件 |
| `RETURN_INITIAL` | 从放置位返回初始/负重待机位并清除本任务计划 | Autonomy 进入下一任务或结束 |

### Action 数据合同

- Goal 公共字段：`MotionTaskContext(request_id, task_id, sequence_id)` 和 `stage`。
- 一次任务分两次发送左右目标对，并保持同一 `task_id`：`MOVE_TO_RECAPTURE` 发送左右重拍末端 6D 位姿，`MOVE_TO_PREGRASP` 发送左右精定位箱体正面中心 6D 位姿。
- 两个阶段都使用 `left_target`、`right_target`，类型为 `MotionPoseTarget`；`pose` 必须是 `PoseStamped(base_link)`，`grasp_mode` 可取 `NO_MOVE`、`SIDE_SUCTION`、`TOP_SUCTION`。本版仅校验该字段，暂不改变既有策略。
- 第二次完整规划必须使用第一次重拍执行后的真实 `/joint_states` 作为起点，禁止回退为固定负重位。
- Goal 不包含箱号、排号、预计算策略、PLC 指令或 `execute/dry_run` 开关；每个 6D 目标携带的 `grasp_mode` 仅作为正式接口字段保留。
- Result 返回阶段成功、计划 ID、规划/执行耗时和 `MotionErrorInfo`；阶段真正完成后才返回 ROS Action `SUCCEEDED`。
- Feedback 只报告当前阶段、内部状态和进度，不作为吸附、释放或安全判据。
- `dry_run` 是 Motion 进程级测试配置，不能由单个任务临时切换。

`/motion/readiness` 使用 `MotionReadiness`，只发布 Motion 是否可接收任务、当前状态和模型/标定/接口版本摘要；它不代替 Action Result 或 RT-Control 安全状态。

## 4. 接口包

- `alfa_motion_interfaces`：只保存 Motion 对外公开的阶段 Action、任务上下文、错误和就绪状态。
- `robot_motion_interfaces`：当前仓库内部规划服务合同，尚未完成去 ROS 化，不能作为五域公共接口。

Motion 不再维护 `alfa_system_interfaces`、`alfa_control_interfaces` 或 `robot_interfaces` 的副本。

## 5. Docker

- ROS 2 Humble + MoveIt 基础镜像。
- 源码只读挂载 `/repo`。
- Release 构建产物写入 `docker/motion/.workspace`。
- 实机使用 `ROS_DOMAIN_ID=42`、host network、Fast DDS UDPv4。
- Motion 不启动、使能、复位或停止 rt-control。

启动命令与 Mock 示例见 `docker/motion/README.md`。

## 6. 未完成项

- `grasp_mode` 对算法策略的正式驱动逻辑；当前只校验和透传该字段。
- Gate、SafetyState、模型版本和标定版本准入。
- 生产级故障恢复、任务取消和通信未知状态验收。
- 将 `robot_motion_interfaces` 的内部服务图进一步收回进程内算法接口。
- 外部五域规范仍保留旧 M-01/M-02/M-03 与 Motion 控真空描述，需要由整机架构文档同步到本阶段合同。
