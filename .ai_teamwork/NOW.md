# 当前项目状态

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目；当前仓库只保留运控、电控、机器人模型及其必要运行适配，历史感知、导航和仿真实现已退出主线。

## 开工入口

- 先读 `.ai_teamwork/START.md`。
- 再读本文件和 `.ai_teamwork/TASKS.md`。
- `.ai_teamwork/archive/` 默认不读；只有追溯旧方案、验收证据、责任边界时再查。

## 当前推进重点

- 当前主线：`v5_dev` 已收口左右箱体正面中心 6D 位姿任务合同；功能分支正在接入 V3 七轴双臂模型。
- V3 模型工作跟踪：Linear `MOTION-94`。外观使用 `robot_v3.0.1_visual` 高精度网格，碰撞继续使用 `robot_v3.0.1` 低面数网格。
- 正式任务输入只包含 `request_id`、左右正面中心 `pose_6d` 和 `execute`；算法内部按高度容差识别排数、吸附方式和抽离策略，禁止从箱号或外部吸附模式获取帮助。
- 旧 `/robot_motion/run_box_pair_task` 仅保留为显式兼容入口，默认完整栈不启动 `box_pair_task_adapter_node`。
- 当前任务表只保留未完成/需确认事项：T-0030/T-0031/T-0032/T-0037。
- 已完成/已同步 Linear 的长过程已归档到 `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/`。
- PM 必须持续把完成任务移出当前表，避免后续 AI 误认为仍需处理。
- 架构目标、包职责和调试代码准入规则见 `docs/运控/系统架构与包职责边界.md`。
- 可视化架构入口见 `docs/system_portal/architecture.html`。

## 当前仍需注意

- V3 模型的外观与碰撞网格职责分离，均由 `ros2_ws/src/alfa_robot_description/meshes/robot_v3_0_1/` 管理，不能只保留其中一套。
- Linear/Git 关联提交标题优先使用 `Refs MOTION-xx: ...`；只写 `MOTION-xx:` 不稳定。
- 一个 issue 只对创建时的验收目标负责；后续探索/测试应拆新 issue 或放 Backlog，不要让已达标 issue 永远开着。
- `alfa_robot_moveit_config` 已不再编译或包含 `scripts/ik_benchmark/` 的头文件；公共 IK 候选类型已迁入 `robot_motion_core`，Rerun 公共实现已迁入 `alfa_robot_rerun`。
- `dual_arm_planner_node` 仍承载完整候选排序、抽离和负重规划适配；这些实现尚未全部迁入独立 core/planning service。
- 历史 `bio_ik`、仓库内 `alfa_robot_hardware` 和旧 `alfa_robot_bringup` 已退出主线；实机硬件与生命周期由外部 `rt-control` 域负责。
- 当前 V3 七轴模型只完成 description、ros2_control、MoveIt Demo 和 KDL 数值 IK 适配；旧结构解析 IK 与完整抓取算法尚未迁移，禁止将本轮模型展示验证描述成全流程验收。

## 当前主要模块速查

- 接口契约：`ros2_ws/src/robot_motion_interfaces/`
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
