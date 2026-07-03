# robot_motion_scene_service 职责

## 负责

- 生成集装箱碰撞面板。
- 根据当前抓取箱号生成箱墙开洞障碍。
- 生成末端附着箱体规格。
- 管理 MoveIt PlanningScene 中的动态障碍和附着物。
- 提供后续碰撞检查模块复用的基础几何结构。

## 不负责

- 不决定抓取顺序。
- 不决定抽离策略。
- 不求 IK。
- 不运行 RRT 或其他轨迹规划。
- 不执行轨迹。

## 当前迁移说明

该包先保持旧仓库命名空间 `alfa_robot::motion`，降低从旧 `dual_arm_planner` 拆分时的风险。
后续迁移到 `robot_motion_control` 主线时，再统一评估是否改为 `robot_motion::scene`。

当前包名里带 `service`，但它不是 ROS service 节点；它是 C++ 几何库和 MoveIt 场景适配库。
如果后续需要运行时场景服务，应在上层另建 node/action/service 包装层，并继续复用这里的
`robot_motion_scene_core` 与 `robot_motion_scene_adapter`。
