# robot_motion_scene_service

`robot_motion_scene_service` 管理运控流程中的动态世界模型和碰撞场景辅助逻辑。

当前阶段先从旧 `dual_arm_planner` 中独立出两层能力：

- 纯几何层：集装箱面板、箱墙开洞障碍、末端携带箱体、AABB 检查。
- MoveIt 适配层：把上述几何对象写入 `PlanningSceneInterface` 或临时 `PlanningScene` 快照。

它不负责业务状态机、不负责 IK、不负责轨迹规划；这些仍由 task / IK / planning 包调用。
