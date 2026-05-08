# alfa_robot_v2_arm_v4 Joint Limits

当前 description 已按新 URDF 原样接入。以下限位来自 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 中的新 URDF 结构；continuous 关节没有 `<limit>` 标签，后续可视化确认后再改成实际上下限。

| Joint | Type | URDF lower | URDF upper | Effort | Velocity | 待确认 |
| --- | --- | --- | --- | --- | --- | --- |
| pitch | revolute | 0 | 0.26 | 0 | 0 | 否 |
| turn | continuous | - | - | - | - | 是 |
| updown | prismatic | 0 | 0.99 | 0 | 0 | 否 |
| leftjoint1 | prismatic | 0 | 0.3 | 0 | 0 | 否 |
| leftjoint2 | continuous | - | - | - | - | 是 |
| leftjoint3 | continuous | - | - | - | - | 是 |
| leftjoint4 | continuous | - | - | - | - | 是 |
| leftjoint5 | continuous | - | - | - | - | 是 |
| leftjoint6 | prismatic | 0 | 0.15 | 0 | 0 | 否 |
| rightjoint1 | prismatic | 0 | 0.3 | 0 | 0 | 否 |
| rightjoint2 | continuous | - | - | - | - | 是 |
| rightjoint3 | continuous | - | - | - | - | 是 |
| rightjoint4 | continuous | - | - | - | - | 是 |
| rightjoint5 | continuous | - | - | - | - | 是 |
| rightjoint6 | prismatic | 0 | 0.15 | 0 | 0 | 否 |

## 涉及限位的文件

- `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`: URDF 位置限位源头；continuous 关节当前无 `<limit>`。
- `ros2_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro`: ros2_control command interface 行程；当前只给 URDF 有限位的 `pitch/updown/*joint1/*joint6` 写 min/max。
- `ros2_ws/src/alfa_robot_moveit_config/config/joint_limits.yaml`: MoveIt 速度/加速度限位；新增 `pitch` 后需要按实测继续修正。
- `ros2_ws/src/alfa_robot_bringup/config/alfa_robot_controllers.yaml`: mock/bringup controller joint 列表，包含 `pitch`。
- `ros2_ws/src/alfa_robot_bringup/config/alfa_robot_moveit_real_controllers.yaml`: 实机 MoveIt controller joint 列表，包含 `pitch`，但未修改 hardware 包。
- `ros2_ws/src/alfa_robot_bringup/scripts/auto_grasp_node.py`: 业务层对 `updown` 有 `0.0..1.0` clamp，后续应改成 `0.99` 或真实行程。
- `ros2_ws/src/alfa_robot_moveit_config/scripts/move_group_arm_full.py`: 交互测试脚本对 `turn`、`updown` 有硬编码 clamp，后续应按真实限位更新。
- `simulation/mujoco/alfa_robot.xml`: MuJoCo joint range / actuator ctrlrange；已按新 URDF 当前限位和 continuous 关节临时控制范围同步。
