# 当前项目状态

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目；用户主导方向，AI 团队负责高效执行、清楚交接、避免重复劳动。

## 当前入口原则

- 新窗口先读 `.ai_teamwork/START.md`，再看本文件和 `.ai_teamwork/TASKS.md`。
- `.ai_teamwork/archive/` 是历史备份区，默认不读；只有追溯原因、恢复旧方案、查责任边界时再看。
- PM 必须把已完成任务及时从当前任务表移到“最近完成/归档”，避免后续 AI 误认为仍需处理。

## 当前推进重点（2026-05-16）

- PM 已重新接手，并把旧任务/长日志备份到 `.ai_teamwork/archive/2026-05-16_pm_handoff/`。
- 当前任务表已收敛为少量仍需推进项：
  - T-0030：当前机械臂单臂可达空间批量仿真。
  - T-0031：可达空间点云 + 当前机器人实体同场景可视化。
  - T-0032：可达性测试可信度校验。
  - T-0036：判断并清理根目录 `alfa_robot_v2_arm_v5/`。

## 最近完成

- T-0025/T-0026/T-0027：实时力学分析语义、MVP 和物理可信度校验已完成。
- T-0029：九朝向可达性脚本已完成，核心文件是 `scripts/nine_orient_reachability.py`。
- T-0033：v1 底盘迁移进 v5 方案已完成。
- T-0034：MoveIt 碰撞规则放宽已完成。
- T-0035：v5_5 机械臂集成记录在 `v5_dev_beta`/历史分支中，当前工作分支是否已合入需按 Git 状态确认。

## v5 模型目录注意

- 根目录 `alfa_robot_v2_arm_v5/` 是独立/历史 ROS 包，当前初查未发现主工程直接依赖它。
- 当前主 URDF 依赖的是 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_v2_arm_v5/`，不要删除这个 mesh 目录。
- 删除根目录 `alfa_robot_v2_arm_v5/` 前，应由 Git 操作工程师做最后一次引用检查和构建/启动验证。

## 当前主要模块速查

- 运控：`ros2_ws/src/alfa_robot_hardware/`、`ros2_ws/src/alfa_robot_bringup/`、`ros2_ws/src/alfa_robot_moveit_config/`
- 机械模型：`ros2_ws/src/alfa_robot_description/`
- 感知抓取：`ros2_ws/src/box_perception/`、`ros2_ws/src/box_perception_msgs/`
- 雷达导航：`lidar_ws/src/`、`ros2_ws/src/fast_lio/`、`ros2_ws/src/livox_ros_driver2/`
- 仿真/测试：`simulation/`、`scripts/`

## 有用文档

- 控制层硬编码：`docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- 控制架构梳理：`docs/REFACTOR_ARCHITECTURE_NOTES.md`
- 雷达进度：`docs/雷达/LIDAR_SLAM_PROGRESS.md`
- 导航底盘接口：`docs/navigation_chassis_integration.md`
- IK 服务说明：`docs/运控/IK/ik_service.md`
