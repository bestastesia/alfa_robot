# 运控工程师长期注意事项

## 核心目标

维护任务、规划、场景、执行到硬件驱动的完整运控链路，优先保证唯一事实源、实机安全和 joint command/state 一致。

## 主要关注路径

- `ros2_ws/src/robot_motion_interfaces/`
- `ros2_ws/src/robot_motion_runtime/`
- `ros2_ws/src/robot_motion_scene_service/`
- `ros2_ws/src/alfa_robot_analytic_ik/`
- `ros2_ws/src/alfa_robot_moveit_config/`
- `ros2_ws/src/alfa_robot_execution_bridge/`
- `ros2_ws/src/alfa_robot_hardware/`
- `ros2_ws/src/alfa_robot_bringup/`
- `ros2_ws/src/alfa_robot_rerun/`
- `docs/运控/系统架构与包职责边界.md`
- `docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- `docs/REFACTOR_ARCHITECTURE_NOTES.md`

## 长期注意

- 改控制链路前先确认 joint 名称、controller 名称、topic 和硬件接口是否一致。
- 涉及实机动作、限位、方向、减速比、电机 ID、node id 时要特别谨慎。
- 不要只改 launch/config 的一端，忘记对应 controller 或 MoveIt 配置。
- MoveIt/RViz 调试工具链也归运控侧统筹；涉及末端 frame 或模型结构时找机械工程师确认。
- 实机相关改动尽量先 mock、静态检查和可重复系统测试，再低速实机验证。
- 发现硬编码时先记录影响范围，不要急着大重构。
- benchmark 和诊断工具只能调用公开接口，生产包禁止反向依赖实验脚本。
- `robot_motion_runtime` 负责权威状态和任务状态机；禁止 MoveIt、执行桥或可视化发布第二份事实。
