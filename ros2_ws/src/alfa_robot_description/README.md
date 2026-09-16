# ALFA Robot V3 描述模型

当前默认模型为 V3.1.1 双臂、双吸盘和 Kkozia 主动悬挂底盘组合版。

## 来源

- 仓库：`SevenovaHangzhou/robot_description`
- 分支：`robot_v3_suction_chassis`
- 导入提交：`30313fb54999f9e09a1245c9f77d8e0243e1d11c`
- 导入日期：2026-09-15

V3.1.1 提供双七轴关节链和 J1～J6 连杆；Kkozia 分支提供双自由度头部、
主动悬挂四舵轮底盘、Tool0 以及双吸盘/双夹爪末端。为兼容现有 MoveIt 与 Demo，
消费者保留 `world` 根链接，并通过固定关节连接来源模型的 `base_footprint`。

## 模型入口

- 默认双吸盘：`urdf/alfa_robot.urdf.xacro`
- 显式双吸盘：`urdf/alfa_robot_dual_suction.urdf.xacro`
- 显式双夹爪：`urdf/alfa_robot_dual_gripper.urdf.xacro`

默认吸盘版包含 26 个可动关节；夹爪版包含 28 个。`config/initial_positions.yaml` 与 `config/joint_limits.yaml` 默认对应吸盘版。

## 关键坐标与关节合同

- `base_footprint -> base_link`：`[0.1900000028, -0.0000442724, 0.4000025060] m`
- `base_footprint`：四轮零位轴心平均位置向下偏移 `0.1 m` 的地面旋转中心
- `updown`：`[-1.0, 0.0] m`，`0 m` 为最高点
- 默认初始（`home`）：`updown=-0.3m`，左臂
  `[155, -105, 20, 90, -90, -40, 0] deg`，右臂
  `[25, -105, -20, 90, -90, 40, 0] deg`
- 卸货（`unloading`）：`updown=-0.3m`，左臂
  `[-50, 90, -50, 50, 20, -40, -60] deg`，右臂
  `[-130, 90, 50, 50, -20, -40, 60] deg`
- 未列出的头部、悬挂、舵轮及夹爪关节均为0；完整合同见
  `config/named_poses*.yaml`
- `left_tool0`、`right_tool0` 沿用来源模型坐标，不等同于重新标定后的吸附面 TCP

## 实机安全边界

本次只更新 description、mock ros2_control 和 MoveIt 模型配置。V3.1.1 的
`updown=[-1.0, 0.0] m` 与 Tool0 坐标在完成实体零位、方向、行程和 TCP 标定前，
只用于模型查看、mock 与离线规划验证，不得直接替代真实执行域合同。
