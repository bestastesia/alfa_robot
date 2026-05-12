# 当前任务

## 使用方式

- 用户或项目经理把任务写到这里。
- AI 开工前看自己要做哪一条。
- 任务不要写太复杂，能说明目标、范围、交付物即可。
- PM 更新任务时，DONE 任务必须从“当前任务列表”移到“已完成任务”，不要长期留在当前列表里。

## 任务状态

- TODO：待做
- DOING：进行中
- BLOCKED：卡住
- DONE：完成
- CANCELED：撤销/废弃

## 当前任务列表

| ID | 状态 | 负责人/角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- | --- |

## 已完成任务

| ID | 负责人/角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- |
| T-0001 | 项目经理 | 建立轻量 AI 协作区 | `AGENTS.md`, `CLAUDE.md`, `.ai_teamwork/` | 已从重流程改为轻量协作 |
| T-0002 | Git 操作工程师 | 提交 AI 轻量协作机制更改 | `AGENTS.md`, `CLAUDE.md`, `.ai_teamwork/`（含 `engineers/`） | 仅提交协作相关文件 |
| T-0004 | 运控工程师 | 定义原机械臂位置安装六轴机械臂的接口需求 | `.ai_teamwork/V5_PROXY_INTERFACE.md` | 已明确 v5 传统 6 轴、2+1+3、Z 轴 joint1、T 型电机、proxy 技术栈与机械交付清单 |
| T-0003 | 机械工程师 | 拆卸/移除当前 4 代机械臂配置 | `ros2_ws/src/alfa_robot_description/`、MoveIt/bringup 控制配置 | 已从主 URDF/MoveIt/RViz 路径真实移除左右机械臂 link/joint；未删除 STL/mesh；simulation 未改 |
| T-0005 | 机械工程师 | 按运控接口需求替换为六轴机械臂模型 | `ros2_ws/src/alfa_robot_description/`、MoveIt/bringup 控制配置 | 已实现 v5 proxy 双六轴机械臂；左右安装到 `updown` 两侧，挂点为 `x=0, y=±0.32, z=0.18`；simulation 未改 |
| T-0006 | MoveIt/运控工程师 | 修正 `dual_arm_with_base` 规划组关节组成 | `alfa_robot.srdf`、MoveIt/controller 配置、双臂规划入口 | 已切到 `dual_v5_arm_with_base`；移除 `pitch/turn`，保留 `updown + 12` 关节；MoveItConfigsBuilder 和包构建通过 |
| T-0007 | MoveIt/机械工程师 | 修复右手末端位姿控制球不显示 | MoveIt SRDF end effector / planning group | 已按旧双臂可用配置模式，为 `dual_v5_arm_with_base` 注册左右两个 end_effector；单臂/with_base/dual 组均有对应末端；MoveItConfigsBuilder 和包构建通过 |
| T-0008 | 机械工程师 | 将右手模型对称到左手 | `ros2_ws/src/alfa_robot_description/` | 已完成真实 Y 镜像修正；右臂 origin/rpy/axis 按左臂镜像；xacro 和 check_urdf 通过 |
| T-0009 | 运控工程师 | 实时显示 RViz 中双末端目标位姿 | MoveIt RViz interactive marker feedback、Pose/Marker 显示 | 新增 `rviz_dual_goal_pose_monitor.py` 和 launch；监听 RViz 交互球 feedback，发布左右目标 PoseStamped 和 MarkerArray；包构建通过 |
| T-0010 | 机械工程师 | 替换为新的机械臂 URDF/模型文件 | `ros2_ws/src/alfa_robot_description/` | 已接入 `alfa_robot_arm_v5` 新 mesh/URDF 参数；保留当前 v5 proxy 备份；主 URDF 继续使用既有 `left/right_v5_*` 接口以兼容 MoveIt/ros2_control；xacro/check_urdf/description 构建通过 |
| T-0011 | 运控工程师 | BioIK IK 解过滤碰撞状态 | `dual_arm_planner_node.cpp`、MoveIt 依赖配置 | 已给直接 `setFromIK` 路径接入 PlanningScene 碰撞有效性回调；构建通过 |
| T-0012 | 机械工程师 | 移除 v5 机械臂安装连接件碰撞 | `ros2_ws/src/alfa_robot_description/` | 最终确认需隐藏/禁碰的是旧 `updown` STL；已移除 `updown` visual/collision、清理 motor1 内部小连接块，并保留全部 `left/right_v5_joint1..6`、`link0..tool0` 接口；xacro/check_urdf/description 构建通过 |
| T-0013 | 运控工程师 | 设计并实现机械臂可达范围测试核心 | `reachability_tester.py`、`reachability_tester.launch.py` | 已实现手动 TF 位姿记录和自动区域 IK 采样，输出 CSV；包构建通过；无 move_group 环境下启动可正常报 `/compute_ik` 不可用 |
| T-0016 | 仿真工程师 | 搭建 Pinocchio 参数化模型生成器骨架 | `simulation/pinocchio_parametric/` | 已完成独立工具目录、YAML baseline stub、Jinja2 URDF 模板、生成脚本、可选 Pinocchio/MeshCat 加载、机械交接文件；baseline 待 T-0015 校准；【已撤销】Pinocchio 仿真路线无法达到用户期望，不再作为后续方案推进 |
| T-0015 | 机械工程师 | 提取当前机械臂参数化 baseline | `simulation/pinocchio_parametric/configs/v5_baseline_stub.yaml`、`MECHANICAL_BASELINE.md` | 已基于当前 ROS2 v5_1 URDF/Xacro 校准 mount、joint origin/rpy/axis、link mass/COM/inertia、大臂/小臂长度和 wrist/flange offset；生成器/check_urdf 通过，T-0017 可继续语义校验；【已撤销】Pinocchio 仿真路线无法达到用户期望，不再作为后续方案推进 |
| T-0017 | 运控工程师 | 校验参数化模型的运动学语义 | `simulation/pinocchio_parametric/tools/validate_kinematics.py`、`generated/kinematic_semantics_report.md` | 已完成 URDF/YAML 静态语义、轻量 FK/Jacobian/position IK 校验；当前环境未安装 Pinocchio，Pinocchio 数值校验路径已实现但未运行；【已撤销】Pinocchio 仿真路线无法达到用户期望，不再作为后续方案推进 |
| T-0018 | 机械工程师 | 为 Pinocchio 参数化模型补齐真实外观 mesh 映射 | `simulation/pinocchio_parametric/configs/v5_baseline_stub.yaml`、`simulation/pinocchio_parametric/MESH_MAPPING.md` | 已补齐当前真实 mesh 映射，但用户不验收：这只是复用当前机械臂 mesh，不是“自动生成的不同参数机械臂仍具真实外观”的参数化生成方案；需 T-0021/T-0022 返工；【已撤销】Pinocchio 仿真路线无法达到用户期望，不再作为后续方案推进 |
| T-0019 | 仿真工程师 | 升级 Pinocchio/MeshCat 可视化为真实机械臂外观 | `simulation/pinocchio_parametric/templates/`、`tools/generate_model.py`、MeshCat 加载路径 | 已接入 T-0018 `mesh_mapping`，默认 `visual.use_primitives=false` 生成当前 v5 SolidWorks STL visual/collision；支持 package/file/relative mesh URI 模式，MeshCat 加载时传入 ROS package 搜索路径；保留 `--visual-mode primitives` fallback；生成器/check_urdf/语义校验通过；【已撤销】Pinocchio 仿真路线无法达到用户期望，不再作为后续方案推进 |

## 已撤销任务

| ID | 原负责人/角色 | 原任务 | 撤销原因 | 后续处理 |
| --- | --- | --- | --- | --- |
| T-0015 | 机械工程师 | 提取当前机械臂参数化 baseline | Pinocchio 参数化仿真路线整体撤销 | 已删除 `simulation/pinocchio_parametric/`，仅保留任务历史记录 |
| T-0016 | 仿真工程师 | 搭建 Pinocchio 参数化模型生成器骨架 | Pinocchio 参数化仿真路线整体撤销 | 已删除 `simulation/pinocchio_parametric/`，仅保留任务历史记录 |
| T-0017 | 运控工程师 | 校验参数化模型的运动学语义 | Pinocchio 参数化仿真路线整体撤销 | 已删除 `simulation/pinocchio_parametric/`，仅保留任务历史记录 |
| T-0018 | 机械工程师 | 为 Pinocchio 参数化模型补齐真实外观 mesh 映射 | Pinocchio 参数化仿真路线整体撤销 | 已删除相关实现，停止返工 |
| T-0019 | 仿真工程师 | 升级 Pinocchio/MeshCat 可视化为真实机械臂外观 | Pinocchio 参数化仿真路线整体撤销 | 已删除相关实现，停止返工 |
| T-0020 | Git 操作工程师 | 清理参数化生成物提交边界 | Pinocchio 参数化仿真路线整体撤销 | 用户已要求清理/回滚；`simulation/pinocchio_parametric/` 与 `.ai_teamwork/PINOCCHIO_STACK.md` 已删除；外部 `universal_robot-noetic-devel/` 已加入忽略避免误提交；等待提交 |
| T-0021 | 机械工程师 | 定义参数化机械臂真实外观生成规范 | Pinocchio 参数化仿真路线整体撤销 | 已取消，相关后续不再推进 |
| T-0022 | 仿真工程师 | 返工 Pinocchio 参数化视觉生成器 | Pinocchio 参数化仿真路线整体撤销 | 已取消，相关后续不再推进 |
| T-0023 | 仿真工程师 | 参数化外观验收 demo | Pinocchio 参数化仿真路线整体撤销 | 已取消，相关后续不再推进 |

## 待用户/PM 补充

| ID | 建议角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- |
