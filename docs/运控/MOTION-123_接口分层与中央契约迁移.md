# MOTION-123 接口分层与中央契约迁移

## 固定结论

- Motion 域内接口唯一包：`motion_internal_interfaces`。
- 域间公共接口唯一来源：完整仓库 `https://github.com/SevenovaHangzhou/robot_interfaces.git`。
- 中央仓库锁定提交：`f699f45972ad15bbbbbb3da1a4894faf209144c9`。
- 中央契约版本：`contract/endpoints.yaml` 的 `0.7.0`；ROS 包版本 `0.6.0`。
- 依赖入口：`ros2_ws/src/dependencies.repos`；锁记录：`ros2_ws/src/dependencies.lock.yaml`。
- 本次不修改中央 IDL，不复制、改名或包装公共 IDL。
- 变更规则引用中央仓库 `docs/robot_interfaces_ai_change_playbook.md`。

## 公共接口迁移清单

| 接口 | 生产者 | 消费者 | 旧来源 | 新来源 | 动作 |
|---|---|---|---|---|---|
| `ExecuteMotionStage.action` | `motion_domain_server` | Autonomy、`manual_motion_domain_task` | `alfa_motion_interfaces` | `robot_motion_interfaces` | 改为中央 Action，补齐 `ok/error/diagnostic` |
| `DualArmPoseTargets.msg` | Autonomy、测试客户端 | `motion_domain_server` | `alfa_motion_interfaces` | `robot_motion_interfaces` | 字段改为 `left/right_grasp_mode`，使用中央枚举 |
| `MotionReadiness.msg` | `motion_domain_server` | Autonomy | `alfa_motion_interfaces` | `robot_system_interfaces/DomainReadiness` | 改为统一准入状态、阻塞原因和结构化错误 |
| `MotionErrorInfo.msg` | `motion_domain_server` | Autonomy、测试客户端 | `alfa_motion_interfaces` | `robot_system_interfaces/ErrorInfo` + `ErrorCode` | 改用 DREE 错误码、严重级别和来源字段 |
| `/motion/readiness` QoS | `motion_domain_server` | Autonomy | 本地手写 `QoSProfile` | `robot_interfaces_qos.latched()` | 删除本地重复 QoS 真值 |

## 域内接口清单

下表全部保留在 `motion_internal_interfaces`，只允许 Motion 工作空间内的软件包依赖。

| IDL | 生产者 | 消费者 | 旧包 | 目标包 | 动作 |
|---|---|---|---|---|---|
| `msg/AttachedBox` | 任务适配、场景源 | 抽离/负重规划、碰撞服务 | `robot_motion_internal_interfaces` | `motion_internal_interfaces` | 包名迁移 |
| `msg/ArmExtractPolicy` | 抓取策略适配 | 抽离规划 | 同上 | 同上 | 包名迁移 |
| `msg/DualGraspStrategy` | 抓取策略适配 | Planner monitor | 同上 | 同上 | 包名迁移 |
| `msg/GraspTarget` | 任务适配 | 抓取策略、规划编排 | 同上 | 同上 | 包名迁移 |
| `msg/MotionContext` | 任务入口 | 域内规划/回执 | 同上 | 同上 | 包名迁移 |
| `msg/MotionPlanCandidate` | 抽离/负重规划 | 编排与执行选择 | 同上 | 同上 | 包名迁移 |
| `msg/Pose6D` | 域内任务客户端 | IK/任务适配 | 同上 | 同上 | 包名迁移 |
| `msg/RobotMotionScene` | `motion_scene_source_node` | 抽离/负重/碰撞规划 | 同上 | 同上 | 包名迁移 |
| `msg/RobotMotionState` | `motion_state_source_node` | IK、规划、Dashboard | 同上 | 同上 | 包名迁移 |
| `msg/TaskReceipt` | 任务编排 | Dashboard、域内调用方 | 同上 | 同上 | 包名迁移 |
| `srv/CheckCollision` | `motion_collision_service_node` | 抽离/负重规划 | 同上 | 同上 | 包名迁移 |
| `srv/ExecuteTrajectory` | `execute_trajectory_service_node` | 域内任务编排 | 同上 | 同上 | 包名迁移 |
| `srv/PlanDualArmIk` | `dual_arm_ik_candidate_service_node` | 位姿任务适配 | 同上 | 同上 | 包名迁移 |
| `srv/PlanExtract` | `plan_extract_service_node` | 域内任务编排 | 同上 | 同上 | 包名迁移 |
| `srv/PlanLoaded` | `plan_loaded_service_node` | 域内任务编排 | 同上 | 同上 | 包名迁移 |
| `srv/RunBoxPairTask` | `box_pair_task_adapter_node` | 旧实验工具 | 同上 | 同上 | 包名迁移；不得作为域间入口 |
| `srv/RunDualArmPoseTask` | `dual_grasp_task_adapter_node` | 箱号/位姿适配 | 同上 | 同上 | 包名迁移；不得作为域间入口 |
| `srv/RunDualGraspTask` | `dual_grasp_task_adapter_node` | 旧实验工具 | 同上 | 同上 | 包名迁移；不得作为域间入口 |
| `srv/RunMotionTask` | `motion_task_orchestrator_node` | 域内任务适配 | 同上 | 同上 | 包名迁移 |
| `srv/SetRobotMotionScene` | `motion_scene_source_node` | 仿真/测试工具 | 同上 | 同上 | 包名迁移 |
| `srv/SetRobotMotionState` | `motion_state_source_node` | 仿真/测试工具 | 同上 | 同上 | 包名迁移 |
| `srv/SolveArmIk` | `analytic_arm_ik_service_node` | 双臂 IK 组合服务 | 同上 | 同上 | 包名迁移 |

## 依赖边界

- `armmotion_demo` 直接依赖 `robot_motion_interfaces`、`robot_system_interfaces`、`robot_interfaces_qos`。
- `robot_motion_runtime`、`alfa_robot_moveit_config` 直接依赖 `motion_internal_interfaces`。
- 仓库内不得再次出现 `alfa_motion_interfaces` 或 `robot_motion_internal_interfaces`。
- `motion_internal_interfaces` 不得被 Autonomy、Perception、RT-Control 或其他仓库消费。

检查命令：

```bash
rg -n 'alfa_motion_interfaces|robot_motion_internal_interfaces' .
rg -l 'motion_internal_interfaces' ros2_ws/src tools scripts
```

## 升级与回滚

旧状态：

- Motion 代码基线：`19f5b2aaa0a87742fba64e0a6480294877ca834e`。
- 原中央依赖 SHA：`1a60d83d52aa97952c8dbb3baafb50b6a95b9e86`。
- 公共运行时实际仍使用本地 `alfa_motion_interfaces`。

部署必须原子进行：

1. Motion 与 Autonomy 都以中央 SHA `f699f45972ad15bbbbbb3da1a4894faf209144c9` 完成离线构建。
2. 停止旧 Motion/Autonomy 进程，禁止新旧 `ExecuteMotionStage` 同时在线。
3. 先部署中央接口构建产物，再部署 Motion 生产者和 Autonomy 消费者。
4. 核对 `/motion/execute_stage` 类型、`/motion/readiness` 类型与 QoS 后恢复任务流。

回滚必须同样原子：停止两端，Motion 回到代码基线 `19f5b2a`，中央仓库回到
`1a60d83d52aa97952c8dbb3baafb50b6a95b9e86`，重建并同时恢复生产者和消费者。

## 验证门槛

```bash
cd ros2_ws
vcs import src < src/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to motion_internal_interfaces robot_motion_interfaces armmotion_demo
colcon test --packages-select motion_internal_interfaces robot_motion_interfaces armmotion_demo
colcon test-result --verbose
```

2026-08-17 本机验证：

- `motion_internal_interfaces`、中央三个依赖包及 13 个 Motion 相关包 Release 构建通过。
- 域内 IDL 生成测试：2 tests，0 failure；合同单测：29 passed。
- `/motion/execute_stage` 类型为 `robot_motion_interfaces/action/ExecuteMotionStage`。
- 成功烟测：`CAMERA_VIEW` 返回 `SUCCEEDED`、`ok=true`、`error.code=SUCCESS`。
- 失败烟测：不可达 `PREGRASP` 返回 `ABORTED`、`ok=false`、
  `error.code=MOTION_IK_NO_SOLUTION(3152)`、`retryable=true`。
- `/motion/readiness` 类型为 `robot_system_interfaces/msg/DomainReadiness`，发布端
  QoS 为 `RELIABLE + TRANSIENT_LOCAL`；失败后状态为 `DEGRADED` 并携带同一结构化错误。

部署时必须使用干净 overlay，避免旧 `alfa_motion_interfaces` 和
`robot_motion_internal_interfaces` 的历史构建产物继续被 ROS 环境发现。
