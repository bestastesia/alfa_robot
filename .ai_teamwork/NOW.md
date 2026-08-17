# 当前项目状态

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目；当前仓库只保留运控、电控、机器人模型及其必要运行适配，历史感知、导航和仿真实现已退出主线。

## 开工入口

- 先读 `.ai_teamwork/START.md`。
- 再读本文件和 `.ai_teamwork/TASKS.md`。
- `.ai_teamwork/archive/` 默认不读；只有追溯旧方案、验收证据、责任边界时再查。

## 当前推进重点

- 当前主线：`v5_dev` 已收口左右箱体正面中心 6D 位姿任务合同。
- Motion 五域联调入口为中央 `robot_motion_interfaces` 提供的 `/motion/execute_stage` 阶段 Action；正式入口只规划和执行轨迹，吸附通路由 Autonomy 编排 RT-Control。
- 域间接口完整依赖中央 `robot_interfaces` 仓库，固定 SHA 记录于 `ros2_ws/src/dependencies.lock.yaml`；域内规划接口统一由 `motion_internal_interfaces` 提供，禁止其他域依赖。
- 正式阶段输入只包含执行阶段和一对左右 `base_link` 6D 目标，不再包含任务号、箱号、frame 或规划内部字段；Action UUID 作为请求身份。同一逻辑任务先发送重拍目标对，再发送精定位抓取目标对。每个目标携带 `NO_MOVE/SIDE_SUCTION/TOP_SUCTION` 模式字段，本版只校验该字段，既有策略仍由位姿分类逻辑决定。
- Motion 对外只有 `/motion/execute_stage` 一个阶段 Action；`CAMERA_VIEW→PREGRASP→APPROACH→PLACE→HOME` 必须顺序调用。Motion 不再检测、规划或主动改变实体 `turn`；规划内部仅使用固定虚拟值。因为 Native rt-control 的完整14轴 Action 禁止 partial goal，发送轨迹时只复制最新 `turn` 反馈并保持零速度、零加速度，Turn 所有权归其他域。
- 旧 `/robot_motion/run_box_pair_task` 仅保留为显式兼容入口，默认完整栈不启动 `box_pair_task_adapter_node`。
- 当前任务表只保留未完成/需确认事项：T-0030/T-0031/T-0032/T-0037。
- 已完成/已同步 Linear 的长过程已归档到 `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/`。
- PM 必须持续把完成任务移出当前表，避免后续 AI 误认为仍需处理。
- 架构目标、包职责和调试代码准入规则见 `docs/运控/系统架构与包职责边界.md`。
- 可视化架构入口见 `docs/system_portal/architecture.html`。

## 当前仍需注意

- 不要删除 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_v2_arm_v5/`，当前 URDF 仍依赖 description 包内 mesh。
- Linear/Git 关联提交标题优先使用 `Refs TIM-xx: ...`；只写 `TIM-xx:` 不稳定。
- 一个 issue 只对创建时的验收目标负责；后续探索/测试应拆新 issue 或放 Backlog，不要让已达标 issue 永远开着。
- `alfa_robot_moveit_config` 已不再编译或包含 `scripts/ik_benchmark/` 的头文件；公共 IK 候选类型已迁入 `robot_motion_core`，Rerun 公共实现已迁入 `alfa_robot_rerun`。
- `dual_arm_planner_node` 仍承载完整候选排序、抽离和负重规划适配；这些实现尚未全部迁入独立 core/planning service。
- 历史 `bio_ik`、仓库内 `alfa_robot_hardware` 和旧 `alfa_robot_bringup` 已退出主线；实机硬件与生命周期由外部 `rt-control` 域负责。

## 当前主要模块速查

- Motion 公开阶段契约：中央依赖 `ros2_ws/src/robot_interfaces/robot_motion_interfaces/`
- 仓库内部规划服务契约：`ros2_ws/src/motion_internal_interfaces/`
- 纯算法公共核心：`ros2_ws/src/robot_motion_core/`
- 核心运行时：`ros2_ws/src/robot_motion_runtime/`
- 场景能力：`ros2_ws/src/robot_motion_scene_service/`
- 运动算法与 MoveIt 适配：`ros2_ws/src/alfa_robot_analytic_ik/`、`ros2_ws/src/alfa_robot_moveit_config/`
- 执行适配：`ros2_ws/src/alfa_robot_execution_bridge/`；真实硬件由外部 `rt-control` 域负责
- 模型与启动编排：`ros2_ws/src/alfa_robot_description/`、`ros2_ws/src/robot_motion_runtime/launch/`
- 可视化：`ros2_ws/src/alfa_robot_rerun/`
- 实验资产：`scripts/ik_benchmark/`，仅用于验证和迁移，不应反向成为运行时职责来源。

## 常用追溯入口

- 本次归档摘要：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/COMPLETED_SUMMARY.md`
- 本次归档前完整日志：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/LOG.before_archive.md`
- 控制层硬编码：`docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- 控制架构梳理：`docs/REFACTOR_ARCHITECTURE_NOTES.md`
- IK 服务说明：`docs/运控/IK/ik_service.md`
