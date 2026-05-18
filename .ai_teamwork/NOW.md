# 当前项目状态

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目；当前协作重点是让新 AI 用最少上下文接住 v5 机械臂运控/仿真任务。

## 开工入口

- 先读 `.ai_teamwork/START.md`。
- 再读本文件和 `.ai_teamwork/TASKS.md`。
- `.ai_teamwork/archive/` 默认不读；只有追溯旧方案、验收证据、责任边界时再查。

## 当前推进重点

- 当前分支：`v5_dev`。
- 当前任务表只保留未完成/需确认事项：T-0030/T-0031/T-0032/T-0036/T-0037。
- 已完成/已同步 Linear 的长过程已归档到 `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/`。
- PM 必须持续把完成任务移出当前表，避免后续 AI 误认为仍需处理。

## 当前仍需注意

- `alfa_robot_v2_arm_v5/` 根目录包疑似历史包；删除前由 Git 操作工程师再次查引用。
- 不要删除 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_v2_arm_v5/`，当前 URDF 仍依赖 description 包内 mesh。
- Linear/Git 关联提交标题优先使用 `Refs TIM-xx: ...`；只写 `TIM-xx:` 不稳定。
- 一个 issue 只对创建时的验收目标负责；后续探索/测试应拆新 issue 或放 Backlog，不要让已达标 issue 永远开着。

## 当前主要模块速查

- 运控：`ros2_ws/src/alfa_robot_hardware/`、`ros2_ws/src/alfa_robot_bringup/`、`ros2_ws/src/alfa_robot_moveit_config/`
- 机械模型：`ros2_ws/src/alfa_robot_description/`
- 感知抓取：`ros2_ws/src/box_perception/`、`ros2_ws/src/box_perception_msgs/`
- 雷达导航：`lidar_ws/src/`、`ros2_ws/src/fast_lio/`、`ros2_ws/src/livox_ros_driver2/`
- 仿真/测试：`simulation/`、`scripts/`

## 常用追溯入口

- 本次归档摘要：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/COMPLETED_SUMMARY.md`
- 本次归档前完整日志：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/LOG.before_archive.md`
- 控制层硬编码：`docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- 控制架构梳理：`docs/REFACTOR_ARCHITECTURE_NOTES.md`
- IK 服务说明：`docs/运控/IK/ik_service.md`
