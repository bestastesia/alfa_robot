# robot_motion_scene_service

`robot_motion_scene_service` 管理运控流程中的动态世界模型和碰撞场景辅助逻辑。

当前阶段先从旧 `dual_arm_planner` 中独立出两层能力：

- 纯几何层：集装箱面板、箱墙开洞障碍、末端携带箱体、AABB 检查。
- MoveIt 适配层：把上述几何对象写入 `PlanningSceneInterface` 或临时 `PlanningScene` 快照。

它不负责业务状态机、不负责 IK、不负责轨迹规划；这些仍由 task / IK / planning 包调用。

## 导出库

- `robot_motion_scene_service::robot_motion_scene_core`：只依赖基础消息和 Eigen，提供箱垛、集装箱、箱墙、附着箱和 AABB 几何。
- `robot_motion_scene_service::robot_motion_scene_adapter`：依赖 MoveIt，把 core 里的几何对象同步到 PlanningScene。

```cmake
find_package(robot_motion_scene_service REQUIRED)

target_link_libraries(your_target
  robot_motion_scene_service::robot_motion_scene_core
  robot_motion_scene_service::robot_motion_scene_adapter
)
```

## 兼容边界

旧 `alfa_robot_moveit_config/include/alfa_robot_moveit_config/motion_core/scene_geometry.hpp`、
`task_geometry.hpp`、`motion_scene_adapter.hpp` 现在只是转发头；真实实现只保留在本包内。
后续迁移到 `robot_motion_control` 时，应优先迁移本包，而不是复制
`dual_arm_planner_node.cpp` 里的场景调用代码。
