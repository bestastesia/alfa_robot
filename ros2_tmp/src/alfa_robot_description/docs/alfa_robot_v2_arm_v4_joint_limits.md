# alfa_robot_v2_arm_v4_new Joint Limits

当前 description 已同步 `alfa_robot_v2_arm_v4_new/urdf/alfa_robot_v2_arm_v4_new.urdf`。以下限位按新版 URDF 当前写法记录；注意部分关节类型仍是 `continuous`，但 SolidWorks 导出的 URDF 里附带了 `lower=0 upper=0` 的 `<limit>` 标签，后续可视化确认后再改真实上下限。

| Joint | Type | URDF lower | URDF upper | Effort | Velocity | 待确认 |
| --- | --- | --- | --- | --- | --- | --- |
| pitch | revolute | 0 | 0.26 | 0 | 0 | 否 |
| turn | continuous | - | - | - | - | 是 |
| updown | prismatic | 0 | 0.99 | 0 | 0 | 否 |
| leftjoint1 | prismatic | 0 | 0.3 | 0 | 0 | 否 |
| leftjoint2 | continuous | 0 | 0 | 0 | 0 | 是 |
| leftjoint3 | continuous | 0 | 0 | 0 | 0 | 是 |
| leftjoint4 | continuous | 0 | 0 | 0 | 0 | 是 |
| leftjoint5 | continuous | 0 | 0 | 0 | 0 | 是 |
| leftjoint6 | prismatic | 0 | 0.15 | 0 | 0 | 否 |
| rightjoint1 | prismatic | 0 | 0.3 | 0 | 0 | 否 |
| rightjoint2 | continuous | 0 | 0 | 0 | 0 | 是 |
| rightjoint3 | continuous | 0 | 0 | 0 | 0 | 是 |
| rightjoint4 | continuous | 0 | 0 | 0 | 0 | 是 |
| rightjoint5 | continuous | 0 | 0 | 0 | 0 | 是 |
| rightjoint6 | prismatic | 0 | 0.15 | 0 | 0 | 否 |

## 涉及限位的文件

- `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`: URDF 位置限位源头，已同步 `alfa_robot_v2_arm_v4_new`。
- `ros2_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro`: ros2_control command interface 行程；已按新版 URDF 当前限位同步，`turn` 仍无 min/max。
- `ros2_ws/src/alfa_robot_moveit_config/config/joint_limits.yaml`: MoveIt 速度/加速度限位；位置限位仍主要来自 URDF。
- `ros2_ws/src/alfa_robot_bringup/config/alfa_robot_controllers.yaml`: mock/bringup controller joint 列表。
- `ros2_ws/src/alfa_robot_bringup/config/alfa_robot_moveit_real_controllers.yaml`: 实机 MoveIt controller joint 列表；未修改 hardware 包。
- `simulation/mujoco/alfa_robot.xml`: MuJoCo joint range / actuator ctrlrange；已按新版 URDF 当前限位和 `turn` 临时控制范围同步。
- `ros2_ws/src/alfa_robot_bringup/scripts/auto_grasp_node.py`: 业务层对 `updown` 有 `0.0..1.0` clamp，后续应改成 `0.99` 或真实行程。
- `ros2_ws/src/alfa_robot_moveit_config/scripts/test_left_arm_full_planning.py`: 交互测试脚本对 `turn`、`updown` 有硬编码 clamp，后续应按真实限位更新。
