# 运控工程师长期注意事项

## 核心目标

维护 ROS2 controller 到硬件驱动的控制链路，优先保证实机安全和 joint command/state 一致。

## 主要关注路径

- `ros2_ws/src/alfa_robot_hardware/`
- `ros2_ws/src/alfa_robot_bringup/`
- `docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- `docs/REFACTOR_ARCHITECTURE_NOTES.md`

## 长期注意

- 改控制链路前先确认 joint 名称、controller 名称、topic 和硬件接口是否一致。
- 涉及实机动作、限位、方向、减速比、电机 ID、node id 时要特别谨慎。
- 不要只改 launch/config 的一端，忘记对应 controller 或 MoveIt 配置。
- 实机相关改动尽量先 mock/仿真/静态检查，再低速实机验证。
- 发现硬编码时先记录影响范围，不要急着大重构。
