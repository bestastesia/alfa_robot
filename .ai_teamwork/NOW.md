# 当前项目状态

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目，当前由用户主导方向，AI 团队负责高效执行、记录交接和避免重复劳动。

## 当前分支和注意点

- 当前主分支/工作分支以用户实际 Git 状态为准。
- 主 ROS2 工作区：`ros2_ws/`
- 雷达/导航工作区：`lidar_ws/`
- 协作区：`.ai_teamwork/`
- 历史归档：`.ai_teamwork/archive/`，默认不读，必要时再查。

## 当前方向重置

- 日期：2026-05-13
- 用户决定：后续直接采用机械侧调好的不同比例机械臂模型进行测试。
- 已撤回：Pinocchio 参数化仿真路线。
- 已撤回：T 电机 STL 单体标定/自动拼装路线。
- 当前任务表已清空，等待用户/PM 下发新任务。
- 旧任务/长日志迁移到 `.ai_teamwork/archive/2026-05-13_direction_reset/`。
- 默认不要继续阅读或沿用旧方向文档，除非用户明确要求追溯。

## 最近完成 (2026-05-14)

- **RViz 双机器人显示 bug 已修复**：根本原因是 MoveIt MotionPlanning 插件的 Query Goal State 渲染了一个不跟踪 joint_states 的"目标机器人"。已禁用 Query Goal State 和 Planned Path 可视，只保留 Scene Robot。
- **pick_place_demo.py IK group 已修正**：从 `dual_v5_arm` 改为 `dual_v5_arm_with_base`，updown 关节参与 IK 求解。
- **当前工作树有大量未提交改动**：左右臂差异化限位、SRDF 新 group/collision 规则、bio_ik 配置、四次抓取脚本、控制器配置等，详见 git diff。
- **待修：IK 偶尔 360° 旋转**，优先级不高。

## 当前主要模块速查

- 运控：`ros2_ws/src/alfa_robot_hardware/`、`ros2_ws/src/alfa_robot_bringup/`、`ros2_ws/src/alfa_robot_moveit_config/`
- 机械模型：`ros2_ws/src/alfa_robot_description/`
- 感知抓取：`ros2_ws/src/box_perception/`、`ros2_ws/src/box_perception_msgs/`
- 雷达导航：`lidar_ws/src/`、`ros2_ws/src/fast_lio/`、`ros2_ws/src/livox_ros_driver2/`
- 仿真：`simulation/`

## 有用文档

- 控制层硬编码：`docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- 控制架构梳理：`docs/REFACTOR_ARCHITECTURE_NOTES.md`
- 雷达进度：`docs/雷达/LIDAR_SLAM_PROGRESS.md`
- 导航底盘接口：`docs/navigation_chassis_integration.md`
- IK 服务说明：`docs/运控/IK/ik_service.md`
- 感知 MoveIt 设计：`docs/superpowers/specs/2026-04-06-perception-moveit-integration-design.md`
