# v5 六轴机械臂 proxy 运控接口需求

## 背景

当前新目标线切换为第五代机械臂开发。机械完整 STL 尚未完成，但运控、MoveIt、RViz、MuJoCo 不应等待机械最终模型。

本阶段采用 **v5 proxy model**：使用圆柱体、长方体、胶囊体等 primitive geometry 先构建运动学和碰撞近似正确的第五代机械臂模型。后续机械正式 STL 到位后，只替换 visual mesh 和必要的 collision 包络，不改变 joint/link/frame/controller 接口。

## 已知机械方向

- 机械臂类型：传统 6 轴机械臂。
- 结构：`2 + 1 + 3`。
  - 肩部 2 自由度。
  - 肘部 1 自由度。
  - 腕部 3 自由度。
- 第 1 自由度旋转轴：`Z` 轴。
- 两个主连杆长度：均大于 `0.6 m`，需要机械工程师给准确值。
- 电机方案：全部采用 T 型电机。
- 目标安装位置：替换原机械臂安装位置，安装到当前底盘/升降/转台体系上。

## proxy model 原则

proxy model 不是最终机械外观模型，而是运控和仿真的先行代理模型。

必须优先保证：

1. joint 数量、joint 顺序正确。
2. joint axis 正确。
3. joint origin / link frame 正确。
4. 零位姿定义清楚。
5. 主连杆长度接近真实设计。
6. joint limits、速度、加速度可用于 MoveIt 规划。
7. `tool0` / 末端执行器 frame 正确。
8. 左右臂安装到底盘的 mount frame 正确。
9. collision primitive 保守、稳定、适合 MoveIt 避障。

不要求：

1. 外观与最终 STL 完全一致。
2. 电机壳、线束、结构板细节完整。
3. 复杂 mesh collision。

## 推荐技术栈

### ROS2 / MoveIt 侧

- 使用 `URDF + Xacro` 构建 v5 proxy 模型。
- 使用 primitive geometry：
  - T 型电机：`cylinder`。
  - 主连杆：`box` 或 `capsule` 风格近似；URDF 无 capsule 时用 cylinder/box 组合。
  - 法兰/腕部：短 `cylinder`。
  - 末端工具：简化 `box`/`cylinder`。
- 使用 MoveIt2 验证：
  - FK / IK。
  - 单臂规划。
  - 双臂规划。
  - 自碰撞。
  - 障碍物避让。

### MuJoCo 侧

- 使用 MJCF primitive geometry：`box`、`cylinder`、`capsule`、`sphere`。
- MuJoCo joint 名、link/body 名应尽量与 ROS2 侧保持可映射。
- proxy 阶段先实现位置控制和 `/joint_states` 回传，不等待最终 STL。

### 参数事实源

建议先维护 YAML 参数源，再由 Xacro/MJCF 消费或人工同步：

- `v5_kinematic_params.yaml`
- `v5_mounting_params.yaml`
- `v5_joint_limits.yaml`
- `v5_proxy_geometry.yaml`

## 命名建议

早期开发建议带 `v5` 前缀，避免和 v4 当前模型冲突。

### 左臂

- joints：
  - `left_v5_joint1`
  - `left_v5_joint2`
  - `left_v5_joint3`
  - `left_v5_joint4`
  - `left_v5_joint5`
  - `left_v5_joint6`
- links：
  - `left_v5_link0`
  - `left_v5_link1`
  - `left_v5_link2`
  - `left_v5_link3`
  - `left_v5_link4`
  - `left_v5_link5`
  - `left_v5_link6`
- tool frame：
  - `left_v5_tool0`

### 右臂

- joints：
  - `right_v5_joint1`
  - `right_v5_joint2`
  - `right_v5_joint3`
  - `right_v5_joint4`
  - `right_v5_joint5`
  - `right_v5_joint6`
- links：
  - `right_v5_link0`
  - `right_v5_link1`
  - `right_v5_link2`
  - `right_v5_link3`
  - `right_v5_link4`
  - `right_v5_link5`
  - `right_v5_link6`
- tool frame：
  - `right_v5_tool0`

### MoveIt groups

- `left_v5_arm`
- `right_v5_arm`
- `dual_v5_arm`
- `left_v5_arm_with_base`
- `right_v5_arm_with_base`
- `dual_v5_arm_with_base`

如果后续决定 v5 完全替换旧臂，也可以把接口收敛回现有 `leftjoint1~6/rightjoint1~6`。但在 proxy 并行开发阶段，优先使用 `v5` 前缀降低误用风险。

## ros2_control / controller 约束

- 每条 v5 机械臂按 6 个主动关节处理。
- proxy 阶段至少暴露 `position` command interface。
- state interface 至少包含：
  - `position`
  - `velocity`
- controller joint 顺序必须固定为：
  - `joint1 -> joint2 -> joint3 -> joint4 -> joint5 -> joint6`
- 实机电控未定前，不要把 T 型电机 CAN ID、减速比、方向等写死到长期配置。
- 电控信息后续应单独进入 hardware/electrical 配置层。

## 机械工程师交付清单

机械工程师优先交付参数事实源，不要等待完整外观建模。

### 1. 运动学参数

交付文件建议：`v5_kinematic_params.yaml`

每个关节需要提供：

- joint name。
- joint type：revolute / continuous。
- parent link。
- child link。
- origin xyz。
- origin rpy。
- axis xyz。
- 零位姿定义。
- 正方向定义。
- 软限位和硬限位。

### 2. 连杆参数

交付文件建议：`v5_proxy_geometry.yaml`

每个 link 需要提供：

- link name。
- 近似几何类型：box / cylinder / capsule-like。
- 尺寸。
- 相对 link frame 的 visual origin。
- 相对 link frame 的 collision origin。
- 质量/惯量可先给粗略值，缺失时仿真工程师可用估算值。

### 3. T 型电机参数

每个电机需要提供：

- 电机外径。
- 电机厚度。
- 输出轴/法兰位置。
- 电机坐标系与对应 joint frame 的关系。
- 是否有减速器或额外传动。

### 4. 安装到底盘参数

交付文件建议：`v5_mounting_params.yaml`

左右臂分别提供：

- mount frame 名称。
- parent frame，暂定从当前底盘/升降/转台体系中选择。
- mount origin xyz。
- mount origin rpy。
- 左右臂是否镜像。
- 镜像平面和轴方向规则。

### 5. 末端参数

交付文件建议：`v5_tool_frame.md`

需要提供：

- `tool0` 相对 `joint6/link6` 的 xyz/rpy。
- 法兰尺寸。
- 吸盘/夹爪安装偏置。
- 末端碰撞包络尺寸。

## 机械工程师当前任务安排

### M-01：确认 v5 六轴运动链

- 输出：6 个 joint 的 origin、axis、parent/child link。
- 验收：运控可据此生成一条单臂 proxy URDF。

### M-02：确认左右臂安装基准

- 输出：左/右 mount frame 相对当前底盘的 xyz/rpy。
- 验收：proxy 双臂能安装到当前机器人底盘上，RViz 中位置合理。

### M-03：确认 proxy collision 几何

- 输出：每个 link 的 box/cylinder/capsule-like 尺寸。
- 验收：MoveIt 中能做自碰撞和障碍物避让，不明显穿模。

### M-04：确认 joint limits

- 输出：每个 joint 的硬限位、推荐软限位、最大速度、最大加速度。
- 验收：MoveIt `joint_limits.yaml` 可直接配置。

### M-05：准备 STL 替换规范

- 输出：后续每个 STL 对应 link、mesh origin、单位、坐标方向。
- 验收：STL 到位后可替换 visual，不破坏已验证的 joint/link/frame。

## 后续落地顺序

1. 机械工程师先交付 M-01/M-02 的最小参数。
2. 运控/机械共同生成单臂 v5 proxy Xacro。
3. 接入当前底盘，生成双臂 proxy URDF。
4. MoveIt 配置 v5 planning groups 和 joint limits。
5. RViz 验证 FK、IK、障碍物避让。
6. MuJoCo 用同名 joint/body 搭 primitive 模型。
7. ROS2 -> MuJoCo 轨迹执行闭环。
8. STL 到位后替换 visual mesh，保留简化 collision。

## 当前禁止事项

- 不要删除原 v4 STL/mesh。
- 不要把临时 proxy 参数当作最终机械事实。
- 不要在没有电控确认前写死 T 型电机 CAN ID、方向、减速比。
- 不要改动已有 v4 控制链路作为 v5 试验入口，v5 应先并行隔离开发。
