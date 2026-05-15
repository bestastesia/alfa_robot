# AI 协作日志

这里记录当前方向下的最新协作信息。旧方向的长日志已归档到：

- `.ai_teamwork/archive/2026-05-13_direction_reset/LOG.full_history.before_reset.md`

## 2026-05-14 Git 操作工程师 / Claude / 分支重命名 + v5_dev 工作提交

- 做了什么：
  1. **v4_dev 本地与远程合并**：v4_dev 本地和远程仅 .gitignore 差异（本地版本更详细，保留 scripts 白名单），用 `git merge -X ours` 合并，保留本地 .gitignore 版本。
  2. **分支重命名**：v3/v4 版本进入不再优化阶段，分支正式更名：
     - `v3_dev` → `alfa_v3`（6DOF 对称臂，冻结）
     - `v4_dev`（合并后）→ `alfa_v4`（v2_arm_v4 正式版，冻结）
     - `v4_dev_1` → `alfa_v4_beta`（v4 实验变体）
     - `v5_dev_p` → `v5_dev_beta`（v5_5 臂实验分支）
  3. **v5_dev 工作提交**：将当前工作树改动提交到 v5_dev，包含 RViz 双机器人显示修复、IK group 修正、v1 底盘恢复、吸盘碰撞忽略、左右臂差异化限位、SRDF/kinematics/controllers 配置更新、updown.STL mesh 优化、get_end_effector_pose.py 清理等。
- 改了哪里：分支重命名（本地）；工作区 17 个文件提交到 v5_dev。
- 待远程操作：推送新分支名 `alfa_v3`/`alfa_v4`/`alfa_v4_beta`/`v5_dev_beta`，删除远程旧名 `v3_dev`/`v4_dev`/`v4_dev_1`/`v5_dev_p`。

## 2026-05-14 运控工程师 / Claude / RViz 双机器人显示修复 + IK group 修正 + 抓取演示脚本完善

- 做了什么：
  1. **定位并修复 RViz 双机器人显示 bug**：根本原因是 MoveIt MotionPlanning 插件的 `Query Goal State`（目标位姿机器人，橙色）渲染了一个不跟踪 `/joint_states` 的机器人。当前 Planning Group 为 `base_group`（只含 updown），所以这个"目标机器人"只在 updown 上与 Scene Robot 不同步，表现为"updown 不动的第二个机器人"。修复：`Query Goal State: false` + `Planned Path Show Robot Visual: false`，只保留 Scene Robot（跟踪 `/joint_states`，updown 正确）。
  2. **修复 pick_place_demo.py IK group 错误**：`DUAL_ARM_GROUP` 从 `"dual_v5_arm"`（不含 updown）改为 `"dual_v5_arm_with_base"`（含 updown），使 IK 求解纳入升降关节。
  3. **前同事已完成的改动（当前工作树未提交）**：左右臂差异化关节限位（URDF xacro + ros2_control + joint_limits.yaml）；SRDF 新增 `dual_v5_arm`、`dual_v5_arm_with_base`、`empty_group`（含 virtual joint）等 group 及大量 disable_collisions 规则；kinematics.yaml 新增 bio_ik solver 配置；initial_positions.yaml 更新；moveit_controllers.yaml / ros2_controllers.yaml 更新为双控制器（torso + dual_v5_arm）；pick_place_demo.py 完整四次抓取脚本（25KB）；updown.STL mesh 优化（19MB→5MB）；rviz 配置大幅更新。
- 改了哪里：`moveit.rviz`（Query Goal State=false, Planned Path 隐藏, Scene Robot 保留）；`pick_place_demo.py`（DUAL_ARM_GROUP 改为 dual_v5_arm_with_base）；前同事改动的完整文件列表见 git diff。
- 验证结果：RViz 重启后只显示一个机器人，updown 跟随指令正确运动；IK 求解包含 updown 关节。
- 留给下个 AI：IK 求解偶尔出现 360° 旋转问题（归一化逻辑已加但部分关节仍绕远路），优先级不高但待修；empty_group（含 virtual joint world_to_base）在 SRDF 中存在但实际未被使用，可清理或保留；pick_place_demo.py 功能完整但尚未提交。

## 2026-05-14 机械工程师 / Claude / v1 底盘恢复 + 吸盘碰撞完全忽略

- 做了什么：
  1. **恢复 v1 底盘**：之前提交 v5_dev_p 后回退 v5_dev 时把底盘修改也一起回退了，现在重新将 v1 底盘修改应用到 v5_dev 工作树。修改内容同 T-0033：base_link mesh 从 `alfa_robot_v2_arm_v4_new` 改为 `alfa_robot`（质量 5.2→394.9kg），turn mesh 和惯性更新（质量 15.2→42.1kg），updown mesh 和惯性恢复可见（质量 8.6→26.9kg），pitch/turn/updown joint origin 重新调整使全局坐标与 v3 一致。
  2. **吸盘碰撞体积完全忽略**：用户反馈吸盘（link6）仍与自身臂碰撞导致规划失败。在 SRDF SuctionVsArm 中新增 `left_v5_link6` vs `left_v5_link0~link5`、`right_v5_link6` vs `right_v5_link0~link5` 的 disable_collisions，与之前已有的 tool0 规则合并。现在 link6（motor6/吸盘）和 tool0 的碰撞体积在 MoveIt 规划中完全忽略，仅保留 link0~link5 之间的单臂自碰撞和左右臂互碰撞。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`（底盘恢复）、`ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf`（SuctionVsArm 扩展到包含 link6）。
- 验证结果：xacro 展开成功；底盘 mesh 路径和质量值正确；SRDF 中 link6 + tool0 与本臂所有 link 的碰撞均已禁用。
- 留给下个 AI：当前底盘和碰撞配置均为未提交的 v5_dev 工作树修改；v5_5 臂变更仍在 v5_dev_p 分支（未合并）；`simulation/realtime_force_mvp/` 为 untracked 目录。

## 2026-05-13 项目经理 / Codex / 方向重置与协作区打扫
- 做了什么：按用户要求撤回 T 电机 STL 标定/自动拼装方向，并将此前已完成、撤销和旧方向任务迁移到归档区。
- 改了哪里：重写 `.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`；归档旧 `TASKS/LOG/MOTOR_MODEL_CALIBRATION/motor_calibration` 到 `.ai_teamwork/archive/2026-05-13_direction_reset/`；删除当前区的 T 电机标定文档和预览目录。
- 验证结果：当前任务列表为空；当前方向说明改为“直接采用机械侧调好的不同比例机械臂模型进行测试”。
- 留给下个 AI：默认只读当前 `START.md`、`NOW.md`、`TASKS.md`、`LOG.md`；旧任务只有必要追溯时再查 archive。

## 2026-05-13 项目经理 / Codex / 安排实时力学可视化任务
- 做了什么：根据用户要求，下发“实时调整末端吸盘重力并实时查看各电机旋转方向/法向受力”的任务链。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0025 定义力学语义；T-0026 做 Pinocchio+MeshCat 实时 MVP；T-0027 运控校验公式和语义；T-0028 后续接入机械侧不同比例模型实时对比。
- 留给下个 AI：核心验收是实时交互可视化，不允许只输出实验后 CSV/图表；吸盘重力变化后，各电机旋转轴方向受力/力矩和法向连接件受力必须实时刷新。

## 2026-05-13 机械工程师 / Codex / T-0025 实时力学受力语义
- 做了什么：基于当前 ROS `alfa_robot.urdf.xacro` 展开模型，定义左右 v5 六轴关节的旋转轴、主法向/副法向、连接件观察点和末端吸盘载荷作用 frame；明确“旋转方向”输出为关节力矩 `tau_axis_Nm`，法向/径向为连接反力分解。
- 改了哪里：新增 `.ai_teamwork/FORCE_ANALYSIS_SEMANTICS.md` 和机器可读 `.ai_teamwork/force_analysis_semantics.yaml`；更新 `.ai_teamwork/TASKS.md`。
- 验证结果：`xacro` 展开当前 ROS URDF 成功；`check_urdf` 通过；脚本校验 YAML 中所有 joint/link/axis 与展开 URDF 一致。
- 留给仿真/运控：T-0026 读取 YAML 做实时 MeshCat 显示，吸盘载荷先作用在 `left_v5_tool0/right_v5_tool0`；T-0027 重点校验 `tau_axis_Nm` 符号、world/local wrench 表达和法向反力是否真来自连接反力。

## 2026-05-13 项目经理 / Codex / 安排可达空间仿真任务
- 做了什么：根据用户规划，安排 T-0028 完成后的单臂可达空间仿真任务链，面向机械侧不同比例机械臂模型。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0029 定义 9 朝向判定；T-0030 批量 IK 采样；T-0031 点云+机械臂同场景可视化；T-0032 运控可信度校验。全部标注依赖 T-0028 后启动。
- 留给下个 AI：可以优先调研 `scripts/ik_benchmark/scripts/ik_range_grid.py` 和 `alfa_robot_benchmarks/ik_range_grid`，但正式实现应等 T-0025~T-0028 力学实时工具链完成后再推进。

## 2026-05-13 仿真工程师 / Codex / T-0026 实时力学可视化 MVP
- 做了什么：新建 `simulation/realtime_force_mvp/`，实现读取当前 ROS xacro、生成临时 URDF、dry-run 校验、Pinocchio+MeshCat 实时吸盘重力可视化框架。
- 改了哪里：`README.md`、`FORCE_SEMANTICS_MVP.md`、`config/force_semantics_mvp.yaml`、`scripts/realtime_force_visualizer.py`、`generated/.gitignore`。
- 验证结果：`py_compile` 通过；`--dry-run` 成功展开当前 `alfa_robot.urdf.xacro`，检查左右 `*_v5_tool0` 和 `updown + 6` 关节；ROS `check_urdf` 通过。当前环境未安装 Pinocchio/MeshCat，所以实时浏览器路径已实现但未实际打开。
- 留给机械/运控：当前力学语义是 provisional：`axis_torque = axis dot ((tip-joint) x F)`，`normal = F - axis*(axis dot F)`；T-0025/T-0027 需要确认各电机轴、连接件法向和吸盘作用点 frame 后再定版。

## 2026-05-13 机械工程师 / Codex / T-0025 用户语义确认补充
- 做了什么：记录用户确认“旋转方向受力”就是关节轴力矩 `tau_axis_Nm`；同时将“法向受力”表述扩展为连接件/部件受力分析，不限定只看单一法向分量。
- 改了哪里：更新 `.ai_teamwork/FORCE_ANALYSIS_SEMANTICS.md` 和 `.ai_teamwork/force_analysis_semantics.yaml`。
- 验证结果：文档层补充，无需重新展开 URDF；T-0026 应在界面中显示连接件受力分解和径向合力，而不是只显示一个“法向力”。
- 留给仿真/运控：实时工具至少显示 `tau_axis_Nm`、`force_axis_N`、`force_normal_N`、`force_side_N`、`force_radial_magnitude_N`；如果能拿到完整 joint wrench，后续再加入连接处弯矩分量评估支架/连接板风险。

## 2026-05-13 仿真工程师 / Codex / T-0026 接入 T-0025 机械语义
- 做了什么：将实时力学可视化 MVP 从 provisional 语义切到优先读取 `.ai_teamwork/force_analysis_semantics.yaml`；左右臂 joint 列表、tool frame、`axis_local`、`normal_local` 均来自 T-0025，缺失时才回退本地配置。
- 改了哪里：更新 `simulation/realtime_force_mvp/config/force_semantics_mvp.yaml`、`simulation/realtime_force_mvp/scripts/realtime_force_visualizer.py`、`simulation/realtime_force_mvp/README.md`、`simulation/realtime_force_mvp/FORCE_SEMANTICS_MVP.md` 和 `.ai_teamwork/TASKS.md`。
- 验证结果：`/usr/bin/python3 -m py_compile` 通过；`--dry-run` 成功展开当前 ROS xacro，并打印 `[OK] loaded T-0025 semantics`，左右 `left/right_v5_joint1..6` 的 axis 与 normal 均识别；生成 URDF 已清理且由 `generated/.gitignore` 忽略。
- 留给运控/机械：当前 MeshCat 仍是静态吸盘重力投影 MVP；`tau_axis_Nm`、`force_axis_N`、`force_normal_N`、`force_side_N`、`force_radial_magnitude_N` 的正式物理意义需要 T-0027 用 Pinocchio Jacobian/RNEA 或 joint wrench 再校验。

## 2026-05-13 仿真工程师 / Claude / T-0026 重写 + T-0027 完成
- 做了什么：用 Pinocchio RNEA 递归牛顿-欧拉逆动力学完全重写实时力学可视化工具。核心改变：1) 所有连杆重力由 RNEA 天然包含（旧 MVP 忽略）；2) 末端吸盘载荷用 Jacobian 转置精确映射（旧 MVP 用手动叉乘近似）；3) 各电机 mass 默认 5kg、com=[0,0,0]（用户要求）；4) tkinter 滑块 GUI 实时调整 15 关节角度+吸盘质量；5) 实时力矩信息面板显示 τ_axis/F_axis/normal/side 和 effort%；6) `--validate` 模式输出 T-0027 校验报告（5 组位姿力矩对比、载荷线性性、杠杆效应验证均通过）。
- 改了哪里：新建 `simulation/realtime_force_mvp/scripts/realtime_force_visualizer.py`（约 600 行，4 个类：AlfaPinocchioModel/MeshCatVisualizerBridge/SliderGUI/T0027Validator）；重写 README.md 和 FORCE_SEMANTICS_MVP.md；更新 TASKS.md（T-0026→DONE, T-0027→DONE）。
- 验证结果：`--validate` 校验报告正常；零位力矩物理合理；J2 弯曲 90° 时杠杆效应正确；载荷力矩与质量成正比（ratio=2.0000）。
- 留给下个 AI：RNEA 已取代手动叉乘；连杆重力不再被忽略；力分解中的 normal/side 仍使用简化投影（y 方向），后续如需更精确可用 T-0025 完整语义轴；GUI 滑块已可用，MeshCat 3D 需在有浏览器环境下测试。

## 2026-05-13 仿真工程师 / Claude / T-0026 修复 + 底盘同步 + 关节质量更新

- 做了什么：
  1. **MeshCat FK 畸变修复**：发现 Pinocchio 4.0.0 的 `JointModelRUBZ`（continuous 关节）在 FK 计算中产生非正交旋转矩阵（det=1.25，含缩放/剪切），导致所有子关节 mesh 畸变和连接脱离。修复：将 URDF 中 `turn` 从 `continuous` 改为 `revolute`（限位 ±2π），Pinocchio 改用 `JointModelRZ`，nq 从 16 降为 15，FK 恢复正交。同时修复 `viewerUrl`→`viewer.url()` API 变更和 `Frame.parent`→`Frame.parentJoint` deprecation。
  2. **底盘同步**：从最新 xacro 重新生成 URDF，同步机械工程师的底盘变更——base_link mass 5→395kg，turn 15→42kg，updown 8.5→27kg，mesh 路径更新为 `alfa_robot/`，新增 updown visual mesh（15→16 个 mesh）。
  3. **关节质量精细化**：按用户指定将各关节电机从统一 5kg 改为独立质量：J1=2.84, J2=13.74, J3=2.84, J4=2.84, J5=2.84, J6=28.0kg（含夹取货物）。
- 改了哪里：`simulation/realtime_force_mvp/generated/current_alfa_robot.urdf`（turn continuous→revolute + 底盘同步重新生成）；`simulation/realtime_force_mvp/scripts/realtime_force_visualizer.py`（viewerUrl 修复、deprecation 修复、JOINT_MASSES 常量替代统一 5kg、`_override_motor_inertials` 支持每关节独立质量）；TASKS.md 和 LOG.md 更新。
- 验证结果：`--validate` 校验全部通过；MeshCat 旋转关节不再畸变；J2=90° 时力矩 371Nm 与杠杆匹配；各关节质量正确（J1=2.84, J2=13.74, J6=28.0）。
- 留给下个 AI：URDF 中 turn 的 continuous→revolute 是对 Pinocchio 4.0.0 RUBZ bug 的 workaround，每次从 xacro 重新生成 URDF 后需手动将 turn 改回 revolute。如果 Pinocchio 升级后修复了此 bug 可恢复 continuous。

## 2026-05-13 机械工程师 / Claude / T-0033 安装 v1 底盘到 v5 机器人
- 做了什么：从 `/mnt/mydisk/ALFA/alfa_robot_v3` 迁移 v1 底盘（base_link/turn/updown）到当前 v5 机器人。替换 mesh 文件路径从 `alfa_robot_v2_arm_v4_new` 到 `alfa_robot`；更新惯性参数（base_link 5.2→394.9kg, turn 15.2→42.1kg, updown 8.6→26.9kg）；重新计算 pitch/turn/updown joint origin 使全局坐标与 v3 一致（base_link 0,0,0.09; turn -0.008,0,0.219; updown -0.244,0,0.797）；updown link 的 visual/collision 从隐藏恢复为可见。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`；mesh 文件已在 `alfa_robot` 目录中。
- 验证结果：xacro 展开成功；全局坐标校验通过（与 v3 一致）。
- 留给下个 AI：v1 底盘已就位，但 v2 底盘就绪后需再次替换。

## 2026-05-13 机械工程师 / Claude / T-0034 禁用 MoveIt 碰撞检测
- 做了什么：按用户要求在 MoveIt SRDF 中禁用两类碰撞：1) 吸盘（link6）与同臂所有 link 的碰撞（SuctionVsArm）；2) 所有臂 link 与 base_link/pitch/turn/updown 的碰撞（ArmVsBase）。保留：左臂 vs 右臂碰撞、单臂自碰撞（不含吸盘）。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf`。
- 验证结果：MoveIt 可正常加载 SRDF；碰撞矩阵符合用户要求。
- 留给下个 AI：当前碰撞配置为调试/开发便利而放宽，正式使用前需恢复完整碰撞检测。

## 2026-05-13 机械工程师 / Claude / T-0035 集成 v5_5 臂 URDF 到全模块
- 做了什么：将 `alfa_robot_arm_v5_5` 新 URDF 集成到所有模块。v5_5 相比 v5 新增 3 个 fixed sub-link（big_arm, big_arm_2, little_arm），插入在 motor2→motor3 和 motor3→motor4 之间；更新 URDF xacro 宏定义、MoveIt SRDF（新 sub-link 的 Adjacent disable_collisions + SuctionVsArm + ArmVsBase）、MoveIt YAML 配置和控制器、MuJoCo XML 模型、realtime_force_mvp 脚本和配置；复制 10 个新 STL × 4 目录 = 40 个 mesh 文件到 `alfa_robot_arm_v5/`。所有变更已提交到 `v5_dev_p` 分支，`v5_dev` 工作树已回退到原始状态。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`、`ros2_ws/src/alfa_robot_moveit_config/config/`（SRDF + YAML + controllers）、`simulation/mujoco/alfa_robot.xml`、`simulation/realtime_force_mvp/`（visualizer + yaml）、40 个 mesh STL 文件。
- 验证结果：colcon build 通过（`--packages-up-to alfa_robot_moveit_config`）；v5_dev_p 分支提交 `b77fe23` 包含完整变更；v5_dev 工作树干净回退（仅 `.ai_teamwork/` 文件和 updown.STL 有非本次修改的 diff）。
- 留给下个 AI：v5_5 变更在 `v5_dev_p` 分支，`v5_dev` 仍为旧 v5 臂；`simulation/realtime_force_mvp/` 为 untracked 目录（从未 git add），磁盘上已有 v5_5 修改但不在任何分支中；正式合并前需 `git add` 该目录并补充提交。

## 2026-05-13 项目经理 / Codex / 调整 T-0027 与可达性任务范围
- 做了什么：根据用户反馈，说明 T-0027 的意义，并将后续可达空间任务改为只针对当前机械臂，不再等待或支持未来新比例模型。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0026 移入刚完成任务；T-0027 改为 TODO，定义为“校验实时受力显示是否物理可信”；T-0028 暂缓；T-0029/T-0030/T-0031/T-0032 均改为当前机械臂范围。
- 留给下个 AI：T-0027 不是做新界面，而是检查 T-0026 的力/力矩计算有没有物理含义错误；可达性测试只跑当前机械臂当前 URDF/SRDF。
