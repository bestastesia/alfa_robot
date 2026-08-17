# ALFA Robot

基于 ROS2 的双臂工业机器人运控、电控与模型仓库。

## Repository Structure

```
alfa_robot/
├── ros2_ws/         ROS2 workspace (colcon build here)
│   └── src/
│       ├── motion_internal_interfaces/  Motion 域内稳定契约
│       ├── robot_interfaces/        中央域间契约（由 dependencies.repos 导入）
│       ├── robot_motion_core/        无 ROS 副作用的运控核心
│       ├── robot_motion_runtime/     权威状态、场景与任务编排
│       └── alfa_robot_rerun/         公共只读可视化 Adapter
├── scripts/         Motion-control experiments and validation tools
│   └── ik_benchmark/  IK benchmark 与可达性实验入口
└── docs/            Architecture, interfaces and validation records
```

## Build

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

包职责和依赖方向见 `docs/运控/系统架构与包职责边界.md`。
