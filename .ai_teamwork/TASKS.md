# 当前任务

## 使用方式

- 用户或项目经理把任务写到这里。
- AI 开工前看自己要做哪一条。
- 任务不要写太复杂，能说明目标、范围、交付物即可。
- PM 布置任务时必须标注依赖/并行关系。
- 已完成、撤销或旧方向任务默认移入 `archive/`，当前表只保留正在推进的任务。

## 任务状态

- TODO：待做
- DOING：进行中
- BLOCKED：卡住
- DONE：完成
- CANCELED：撤销/废弃

## 当前任务列表

| ID | 状态 | 负责人/角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- | --- |
| T-0027 | DONE | 仿真工程师 | 校验实时受力显示是否物理可信 | `simulation/realtime_force_mvp/`、Pinocchio RNEA + Jacobian | T-0026 重写为 Pinocchio RNEA 版；`--validate` 校验通过：零位/弯曲/载荷多组位姿力矩对比、载荷线性性、杠杆效应均正确；J2=90° 时力矩 371Nm 与 13.74kg 电机+28kg 末端载荷的杠杆匹配 |
| T-0029 | TODO | 运控工程师 | 设计单臂 9 朝向可达性判定规则 | `scripts/ik_benchmark`、`alfa_robot_benchmarks`、MoveIt IK 配置、当前机械臂 MoveIt/IK 接口 | 依赖 T-0027 完成后启动；可与 T-0030 前期方案并行；仅针对当前机械臂，不考虑未来新比例。判定规则：空间点默认朝前，中心姿态 + yaw±15° + pitch±15° + yaw/pitch 组合共 9 个朝向全部 IK 成功，才认为该点可达；roll 不参与，因为最后一关节可处理 |
| T-0030 | TODO | 仿真工程师 | 实现当前机械臂单臂可达空间批量仿真 | 复用 `ik_range_grid`/`alfa_robot_benchmarks` 或 MoveIt `/compute_ik`；输入 xyz 范围与间隔；输出 CSV/点云 | 依赖 T-0029 规则；可与 T-0029 前期并行调研接口。仅针对当前机械臂当前 URDF/SRDF；每个点跑 9 朝向，记录全部成功/部分失败/失败原因和耗时 |
| T-0031 | TODO | 仿真工程师 | 可达空间点云与机械臂同场景可视化 | RViz Marker/PointCloud2 或 Open3D/MeshCat；必须加载当前机械臂模型 | 依赖 T-0030；验收采用点云可视化，并且必须同时生成/显示当前机械臂外观，方便判断点云相对机械臂的位置；成功点和失败点颜色区分，9 朝向失败可按失败数量渐变 |
| T-0032 | BLOCKED | 运控工程师 | 校验可达性仿真结果可信度 | 抽样点 IK 解、FK 回代、关节限制、碰撞/是否考虑碰撞的说明 | 依赖 T-0030/T-0031；抽查成功点 9 朝向 FK 误差，确认使用的 IK 求解器和规划组正确；明确当前结果是否考虑碰撞，若不考虑必须在可视化和报告中标注 |

## 已取消/暂缓任务

| ID | 原负责人/角色 | 原任务 | 原因 |
| --- | --- | --- | --- |
| T-0028 | 仿真工程师 | 接入机械侧不同比例模型实时对比 | 当前阶段改为只针对当前机械臂，暂不考虑未来新比例模型 |

## 当前方向说明

- 后续不再推进 Pinocchio 参数化仿真。
- 后续不再推进 T 电机 STL 单体标定/自动拼装方向。
- 当前近期方向：先基于当前机械臂完成实时受力校验和单臂可达空间测试；未来是否接入不同比例模型再由用户确认。
- 历史任务和长日志已归档到 `.ai_teamwork/archive/2026-05-13_direction_reset/`，默认不需要阅读。

## 刚完成任务

| ID | 负责人/角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- |
| T-0033 | 机械工程师 | 安装 v1 底盘到 v5 机器人 | `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`、mesh 文件 | 从 v3 项目迁移 v1 底盘（base_link/turn/updown）mesh 和惯性参数；调整 pitch/turn/updown joint origin 使全局坐标系与 v3 一致；base_link 质量 5.2→394.9kg；turn 质量 15.2→42.1kg；updown 质量 8.6→26.9kg |
| T-0034 | 机械工程师 | 禁用 MoveIt 中吸盘与臂、臂与车体碰撞检测 | `ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf` | 新增 SuctionVsArm（link6 + tool0 vs 本臂 link0~link5）和 ArmVsBase（所有臂 link vs base_link/pitch/turn）disable_collisions；保留左臂 vs 右臂碰撞和单臂自碰撞（link0~link5 之间）；link6（吸盘/motor6）碰撞体积完全忽略 |
| T-0035 | 机械工程师 | 集成 v5_5 臂 URDF 到全模块 | URDF xacro、MoveIt SRDF/YAML/controllers、MuJoCo XML、realtime_force_mvp、mesh 文件 | v5_5 新增 3 个 fixed sub-link（big_arm, big_arm_2, little_arm）插入 motor2→motor3 和 motor3→motor4 之间；10 个新 STL × 4 目录（visual/collision/visual_right/collision_right）= 40 个 mesh 文件；全部已提交到 v5_dev_p 分支 |
| T-0025 | 机械工程师 | 定义实时力学分析所需的关节/电机受力语义 | `.ai_teamwork/FORCE_ANALYSIS_SEMANTICS.md`、`.ai_teamwork/force_analysis_semantics.yaml`、当前 ROS URDF | 已明确左右 `v5_joint1..6` 的 axis、主法向、连接观察方向、末端吸盘载荷 frame；区分 `tau_axis_Nm` 关节力矩与 `force_axis/normal/radial_N` 连接反力；YAML 与展开 URDF joint/link/axis 校验通过，可供 T-0026/T-0027 使用 |
| T-0026 | 仿真工程师 | 实时力学可视化 MVP (RNEA 重写版) | `simulation/realtime_force_mvp/scripts/realtime_force_visualizer.py` | Pinocchio RNEA+Jacobian；tkinter 滑块实时调关节角度+吸盘质量；MeshCat 3D；电机 5kg 重心中间 |
