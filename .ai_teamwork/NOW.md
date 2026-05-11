# 当前项目状态

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目，当前重点是让运控、机械模型、MoveIt/IK、感知抓取、雷达 SLAM 导航和仿真逐步形成可演示闭环。

## 当前分支和注意点

- 当前分支：`v5_dev`
- 主 ROS2 工作区：`ros2_ws/`
- 雷达/导航工作区：`lidar_ws/`
- 当前已有本地改动：`.gitignore`，未确认归属前不要覆盖。
- `ros2_ws` 和 `lidar_ws` 有重复包，改动前最好确认当前任务以哪个为准。

## 当前主要模块理解

- 运控：`ros2_ws/src/alfa_robot_hardware/`、`ros2_ws/src/alfa_robot_bringup/`
- 机械模型：`ros2_ws/src/alfa_robot_description/`、`alfa_robot_v2_arm_v4_new/`
- MoveIt/IK：`ros2_ws/src/alfa_robot_moveit_config/`、`scripts/ik_benchmark/`
- 感知抓取：`ros2_ws/src/box_perception/`、`ros2_ws/src/box_perception_msgs/`
- 雷达导航：`lidar_ws/src/`、`ros2_ws/src/fast_lio/`、`ros2_ws/src/livox_ros_driver2/`
- 仿真：`simulation/`

## 近期协作目标

1. 让每个 AI 新窗口快速知道项目当前状态。
2. 根据用户/PM 指令领取小任务。
3. 每轮结束留下简短交接，方便下个 AI 接上。
4. 避免多个 AI 在不知情的情况下改同一块核心文件。

## 有用文档

- 控制层硬编码：`docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- 控制架构梳理：`docs/REFACTOR_ARCHITECTURE_NOTES.md`
- 雷达进度：`docs/雷达/LIDAR_SLAM_PROGRESS.md`
- 导航底盘接口：`docs/navigation_chassis_integration.md`
- IK 服务说明：`docs/运控/IK/ik_service.md`
- 感知 MoveIt 设计：`docs/superpowers/specs/2026-04-06-perception-moveit-integration-design.md`
