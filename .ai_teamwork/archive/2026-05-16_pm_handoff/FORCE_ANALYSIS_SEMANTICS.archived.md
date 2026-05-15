# 实时力学分析关节/电机受力语义规范

任务：T-0025 机械工程师  
范围：当前 ROS URDF、后续机械侧不同比例模型、末端吸盘载荷 frame。  
目标：给 T-0026/T-0027 明确“各电机旋转方向受力/力矩”和“法向连接件受力”的坐标、命名和显示语义。

## 结论

- 当前模型的真实事实源是 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 展开的 URDF。
- 实时工具必须从 URDF/Pinocchio 当前关节位姿动态计算 joint placement 和 axis，不要长期硬编码零位世界坐标。
- 末端吸盘力当前作用点定义为 `left_v5_tool0` / `right_v5_tool0` 原点；如果后续机械模型提供真实吸盘中心，应新增固定 frame `left_suction_center` / `right_suction_center` 并把本规范的作用点迁移过去。
- 用户已确认：“旋转方向受力”就是 revolute joint 绕 joint axis 的力矩 `tau_axis_Nm`，单位 N·m；不要把它误叫成线性力 N。
- “法向受力”不应被理解成唯一指标；后续界面应以“连接件/部件受力”为主，至少显示连接反力在轴向、主法向、副法向以及径向合力上的分解，帮助判断不同结构件是否合理。

## 坐标系约定

### 全局显示坐标

- `world`：RViz/MeshCat 全局显示 frame，Z 轴向上。
- 当前 `world -> base_link` 固定偏移为 `[0, 0, 0.09]`。
- 实时可视化箭头统一画在 `world` 下。

### 关节局部坐标

对每个 `*_v5_jointN`：

- `joint_origin`：URDF joint `<origin>` 对应点；在 Pinocchio 中使用该 joint placement。
- `axis_local`：URDF `<axis xyz="...">`，表达在 joint frame 中。
- `axis_world(q)`：实时姿态下 `joint_frame_world_rotation(q) * axis_local`。
- 正方向：右手定则；`tau_axis > 0` 表示绕 `axis_world` 的正向力矩。

### 末端吸盘载荷

当前没有独立吸盘 frame，因此先定义：

| 侧 | 载荷作用 frame | 作用点 | 默认力方向 |
| --- | --- | --- | --- |
| left | `left_v5_tool0` | frame 原点 | `world` 下 `[0, 0, -m*g]` |
| right | `right_v5_tool0` | frame 原点 | `world` 下 `[0, 0, -m*g]` |

如果吸盘真实受力点不在 `tool0` 原点，机械侧需要提供固定偏移：

- `left_suction_center` 相对 `left_v5_tool0` 的 `xyz/rpy`
- `right_suction_center` 相对 `right_v5_tool0` 的 `xyz/rpy`

在未提供前，T-0026 以 `*_v5_tool0` 原点作为吸盘重力作用点验收。

## 当前 v5 关节语义

以下 local 数据来自当前主 URDF；后续比例模型如保持同名 joint，可沿用同一语义。`child_connection_local` 是该电机/link 向下一关节或 tool0 的观察方向，用来定义主法向。

### 左臂

| joint | parent -> child | axis_local | child_connection_local | normal_local | 语义 |
| --- | --- | --- | --- | --- | --- |
| `left_v5_joint1` | `left_v5_link0 -> left_v5_link1` | `[0, 0, 1]` | `[-0.0595, 0, 0.0735]` | `[-1, 0, 0]` | 底部 Z 轴旋转，主法向指向下一关节横向偏置 |
| `left_v5_joint2` | `left_v5_link1 -> left_v5_link2` | `[0, 0, -1]` | `[0, 0.600, -0.148]` | `[0, 1, 0]` | 肩部横轴等效旋转，大臂向 `+Y` 展开 |
| `left_v5_joint3` | `left_v5_link2 -> left_v5_link3` | `[0, 0, 1]` | `[0, -0.412, 0.009]` | `[0, -1, 0]` | 肘部旋转，小臂向 `-Y` 回折 |
| `left_v5_joint4` | `left_v5_link3 -> left_v5_link4` | `[-1, 0, 0]` | `[0.0595, -0.074, 0]` | `[0, -1, 0]` | 腕 1，主法向沿腕部连接偏置 |
| `left_v5_joint5` | `left_v5_link4 -> left_v5_link5` | `[1, 0, 0]` | `[-0.060, -0.074, 0]` | `[0, -1, 0]` | 腕 2，主法向沿腕部连接偏置 |
| `left_v5_joint6` | `left_v5_link5 -> left_v5_link6` | `[0, 0, -1]` | `[0, 0, 0.080]` | `[1, 0, 0]` | 腕 3/法兰，tool0 沿轴向，主法向取局部 `+X` |

### 右臂

右臂是左臂关于机器人中线的镜像，joint 名称和 axis_local 保持一致，连接方向按 Y 镜像。

| joint | parent -> child | axis_local | child_connection_local | normal_local | 语义 |
| --- | --- | --- | --- | --- | --- |
| `right_v5_joint1` | `right_v5_link0 -> right_v5_link1` | `[0, 0, 1]` | `[-0.0595, 0, 0.0735]` | `[-1, 0, 0]` | 底部 Z 轴旋转 |
| `right_v5_joint2` | `right_v5_link1 -> right_v5_link2` | `[0, 0, -1]` | `[0, -0.600, -0.148]` | `[0, -1, 0]` | 右侧大臂向 `-Y` 展开 |
| `right_v5_joint3` | `right_v5_link2 -> right_v5_link3` | `[0, 0, 1]` | `[0, 0.412, 0.009]` | `[0, 1, 0]` | 右侧小臂向 `+Y` 回折 |
| `right_v5_joint4` | `right_v5_link3 -> right_v5_link4` | `[-1, 0, 0]` | `[0.0595, 0.074, 0]` | `[0, 1, 0]` | 腕 1 |
| `right_v5_joint5` | `right_v5_link4 -> right_v5_link5` | `[1, 0, 0]` | `[-0.060, 0.074, 0]` | `[0, 1, 0]` | 腕 2 |
| `right_v5_joint6` | `right_v5_link5 -> right_v5_link6` | `[0, 0, -1]` | `[0, 0, 0.080]` | `[1, 0, 0]` | 腕 3/法兰 |

## 零位世界坐标参考

此表只用于调试显示方向是否大致正确；实时工具必须使用 FK 动态更新。

| joint | world origin @ zero | axis_world @ zero |
| --- | --- | --- |
| `left_v5_joint1` | `[-0.050000, 0.200000, 0.496000]` | `[0, 0, 1]` |
| `left_v5_joint2` | `[-0.109500, 0.200000, 0.569500]` | `[-1, 0, 0]` |
| `left_v5_joint3` | `[-0.257498, 0.199999, 1.169500]` | `[1, 0, 0]` |
| `left_v5_joint4` | `[-0.248496, 0.199996, 1.581500]` | `[-1, 0, 0]` |
| `left_v5_joint5` | `[-0.188996, 0.125997, 1.581500]` | `[0, 1, 0]` |
| `left_v5_joint6` | `[-0.188995, 0.065996, 1.655500]` | `[0, 0, -1]` |
| `right_v5_joint1` | `[-0.050000, -0.200000, 0.496000]` | `[0, 0, 1]` |
| `right_v5_joint2` | `[-0.109500, -0.200000, 0.569500]` | `[-1, 0, 0]` |
| `right_v5_joint3` | `[-0.257498, -0.199999, 1.169500]` | `[1, 0, 0]` |
| `right_v5_joint4` | `[-0.248496, -0.199996, 1.581500]` | `[-1, 0, 0]` |
| `right_v5_joint5` | `[-0.188996, -0.125997, 1.581500]` | `[0, -1, 0]` |
| `right_v5_joint6` | `[-0.188995, -0.065996, 1.655500]` | `[0, 0, -1]` |

## 受力/力矩分解定义

对每个关节 `i`，实时工具应输出以下字段。

### 旋转轴方向力矩

字段：`tau_axis_Nm`

- 来源：Pinocchio generalized torque 或 `J_tool(q)^T * external_wrench` 对应 joint 的分量。
- 单位：N·m。
- 正方向：绕 `axis_world(q)` 的右手正方向。
- 显示：在 joint origin 画沿 `axis_world` 的力矩/旋转箭头；颜色区分正负。

不要把 `tau_axis_Nm` 写成 N，也不要叫“轴向力”。它是电机需要承受/输出的关节力矩。

### 轴向连接力

字段：`force_axis_N`

- 若 T-0026/T-0027 能从逆动力学得到 joint 处连接反力 `F_joint_world`，则：

```text
force_axis_N = dot(F_joint_world, axis_world)
```

- 单位：N。
- 含义：沿电机转轴方向的拉/压力，不等于电机扭矩。

### 连接件/部件受力分解

字段：`force_normal_N`、`force_side_N`、`force_radial_magnitude_N`

定义：

```text
normal_world = normalize(R_joint_world * normal_local)
side_world   = normalize(cross(axis_world, normal_world))
force_normal_N = dot(F_joint_world, normal_world)
force_side_N   = dot(F_joint_world, side_world)
force_radial_magnitude_N = sqrt(force_normal_N^2 + force_side_N^2)
```

含义：

- `force_normal_N`：沿机械连接主偏置方向的连接反力分量，可作为连接件/支架受力分析的主方向指标。
- `force_side_N`：垂直于 axis 和 normal 的另一径向分量。
- `force_radial_magnitude_N`：连接件径向合力，适合做结构风险排序。

因此 T-0026 的界面不必只叫“法向受力”，建议显示为“连接件受力”，并展开：

- `axis`：沿电机转轴方向的拉/压力；
- `normal`：沿下一段连接/偏置方向的主要剪切/弯曲相关分量；
- `side`：另一径向方向的剪切分量；
- `radial magnitude`：径向合力，用于快速比较哪个连接件更危险。

如果后续要判断“不同部件受力是否合理”，建议 T-0026/T-0027 在每个关节处至少同时显示 `tau_axis_Nm`、`force_axis_N`、`force_normal_N`、`force_side_N`、`force_radial_magnitude_N`。若能进一步拿到 joint wrench，也应保留连接处弯矩分量用于判断连接板/支架弯曲风险。

如果某个比例模型改变了连杆方向，`normal_local` 应由该模型的 `child_connection_local` 重新投影得到，而不是继续使用旧数值。

## 连接件观察点

当前 ROS URDF 没有独立的连接件 frame，因此先统一使用以下观察点：

| 观察量 | frame/点 | 用途 |
| --- | --- | --- |
| 电机轴承/关节受力 | `*_v5_jointN` origin | 显示 `tau_axis_Nm`、`force_axis_N`、`force_radial_magnitude_N` |
| 电机/link 出口连接 | `child_connection_local` 对应点 | 画主法向方向、检查下一段连杆受力方向 |
| 末端吸盘载荷 | `*_v5_tool0` origin | 施加可调吸盘重力 |

后续机械侧如果提供真实螺栓孔/法兰面/吸盘中心，建议新增固定 frame，而不是让仿真工程师靠 mesh 猜点位：

- `left_v5_jointN_bearing_center`
- `left_v5_jointN_connector_out`
- `left_suction_center`
- 右臂同名 `right_*`

## 给 T-0026 的接口要求

实时可视化工具至少需要显示：

1. 吸盘重力滑块：输入质量 `m_kg` 或力 `F_N`，默认力方向 `world [0,0,-1]`。
2. 每个 joint 的 `axis_world` 箭头和 `tau_axis_Nm` 数值，作为电机旋转方向受力/力矩指标。
3. 每个 joint 的连接件受力分解：`force_axis_N`、`force_normal_N`、`force_side_N`。
4. 每个 joint 的 `force_radial_magnitude_N`，用于快速看哪一处连接件/结构件受力最大。
5. 左右臂分开显示，命名必须保留 `left_v5_joint1..6` / `right_v5_joint1..6`。

建议输出字段：

```yaml
joint_name:
  origin_world: [x, y, z]
  axis_world: [x, y, z]
  normal_world: [x, y, z]
  side_world: [x, y, z]
  tau_axis_Nm: 0.0
  force_axis_N: 0.0
  force_normal_N: 0.0
  force_side_N: 0.0
  force_radial_magnitude_N: 0.0
```

## 给 T-0027 的校验重点

- 校验 `tau_axis_Nm` 与 Pinocchio generalized torque 的符号是否一致。
- 校验外力 wrench 的 frame 表达，不要把 world wrench 误当 local wrench。
- 校验 `force_normal_N` 的来源是否真的是连接反力；如果 MVP 阶段只能算 torque，必须在界面标注法向力暂不可用或为近似值。
- 校验正负方向：正力矩为绕 `axis_world` 右手方向，正法向为 `normal_world` 方向。

## 后续比例模型要求

机械侧提供不同比例模型时必须满足至少一项：

1. 保持 `left_v5_joint1..6`、`right_v5_joint1..6`、`left_v5_tool0`、`right_v5_tool0` 命名不变；或
2. 提供一个映射表，把新模型 joint/tool frame 映射回本规范字段。

如果比例模型改变大臂/小臂长度，只要 joint origin 和 axis 在 URDF 中正确，T-0026 应自动跟随；不要再用旧零位坐标硬编码。
