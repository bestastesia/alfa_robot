# ALFA Robot V3 描述模型

当前默认模型为双吸盘、主动悬挂底盘版 V3.0.9。

## 来源

- 仓库：`SevenovaHangzhou/robot_description`
- 分支：`robot_v3_suction_chassis`
- 导入提交：`17f5bdc46b8f2580ee81aed919da7b404da3bdaf`
- 导入日期：2026-09-15

导入资产包括 V3 双七轴机械臂语义、双自由度头部、主动悬挂四舵轮底盘，以及双吸盘/双夹爪两套末端。来源模型的 46 个机械臂 STL 与仓库已有 `robot_v3_0_8` 资产逐字节一致，因此直接复用，避免重复提交约 20 MB。为兼容本仓库现有 MoveIt 与 Demo，模型保留 `world` 根链接，并通过固定关节连接来源模型的 `base_footprint`。

## 模型入口

- 默认双吸盘：`urdf/alfa_robot.urdf.xacro`
- 显式双吸盘：`urdf/alfa_robot_dual_suction.urdf.xacro`
- 显式双夹爪：`urdf/alfa_robot_dual_gripper.urdf.xacro`

默认吸盘版包含 26 个可动关节；夹爪版包含 28 个。`config/initial_positions.yaml` 与 `config/joint_limits.yaml` 默认对应吸盘版。

## 关键坐标与关节合同

- `base_footprint -> base_link`：`[0.195, 0.015, 0.400] m`
- `updown`：`[-0.5, 0.5] m`，`0 m` 为来源模型行程中点
- 左臂默认姿态：`[150, 90, -5, 120, 0, 0, 0] deg`
- 右臂默认姿态：`[-150, -90, 5, -120, 0, 0, 0] deg`
- `left_tool0`、`right_tool0` 沿用来源模型坐标，不等同于重新标定后的吸附面 TCP

## 实机安全边界

本次只更新 description、mock ros2_control 和 MoveIt 模型配置。当前真实执行桥仍保护既有 `updown=[0.0, 0.7] m` 实机合同；来源模型的 `[-0.5, 0.5] m` 坐标**不得直接用于实机执行**。在完成升降零位、方向、行程和 TCP 标定前，只用于模型查看、mock 与离线规划验证。
