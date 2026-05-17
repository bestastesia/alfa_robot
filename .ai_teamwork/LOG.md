# AI 协作日志

这里记录当前方向下的最新协作信息。长日志已归档：

- `.ai_teamwork/archive/2026-05-13_direction_reset/LOG.full_history.before_reset.md`
- `.ai_teamwork/archive/2026-05-16_pm_handoff/LOG.before_cleanup.md`

## 2026-05-16 项目经理 / Codex / PM 重新接手与任务清理
- 做了什么：重新梳理当前项目状态，将已完成任务和长日志归档，当前任务表只保留待推进事项。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/NOW.md`、`.ai_teamwork/LOG.md`，新增 `.ai_teamwork/archive/2026-05-16_pm_handoff/`。
- 验证结果：当前待推进任务为 T-0030/T-0031/T-0032/T-0036；已完成 T-0025/T-0026/T-0027/T-0029/T-0033/T-0034/T-0035 只保留摘要；力学语义和 v5 proxy 历史文档已备份进本次归档。
- 留给下个 AI：默认不要读归档；需要追溯旧细节再查 archive。`alfa_robot_v2_arm_v5/` 根目录包疑似可删，但 description 包内 `meshes/alfa_robot_v2_arm_v5/` 仍被当前 URDF 使用，不能删。

## 2026-05-16 仿真工程师 / Codex / 最小 Rerun 机器人可视化案例
- 做了什么：按用户要求先做最基础 Rerun 案例，新建 `alfa_robot_rerun` ROS2 Python 包；读取当前 `alfa_robot_description` xacro，加载 visual mesh，订阅 `/joint_states`，用轻量 URDF FK 更新 Rerun 中各 link 位姿。
- 改了哪里：新增 `ros2_ws/src/alfa_robot_rerun/`，包含 `basic_robot_viewer` 节点、`basic_robot_viewer.launch.py`、`README.md`、`package.xml`、`setup.py/setup.cfg`。
- 验证结果：确认 `/usr/bin/python3` 已有 `rerun-sdk 0.31.4`；`py_compile` 通过；`colcon build --packages-select alfa_robot_rerun --symlink-install` 通过；`ros2 pkg executables` 可见 `basic_robot_viewer`；`spawn:=false` 短时启动成功并订阅 `/joint_states`。
- 留给下个 AI：第一版刻意不接 MoveIt/RViz 交互、不做规划和记录管理；后续若扩展，优先加 planned path/末端 frame/可达点云 logger，保持只读观察层。

## 2026-05-16 仿真工程师 / Codex / Rerun SDK 0.31 时间 API 修复
- 做了什么：修复 `basic_robot_viewer` 在收到 `/joint_states` 后崩溃的问题；Rerun 0.31 没有 `set_time_seconds`，改为 `rr.set_time("ros_time", timestamp=stamp)`。
- 改了哪里：`ros2_ws/src/alfa_robot_rerun/alfa_robot_rerun/basic_robot_viewer.py`。
- 验证结果：`py_compile` 通过；`colcon build --packages-select alfa_robot_rerun --symlink-install` 通过；用 `spawn:=false` 启动并 `ros2 topic pub --once /joint_states` 模拟一条关节状态，节点未再崩溃。
- 留给下个 AI：继续保持最小案例定位；若增加功能，优先加末端坐标系/轨迹显示，不要急着接管 MoveIt 交互。

## 2026-05-16 运控工程师 / Codex / V5 单圈关节限位调整
- 做了什么：按用户提供的 description 可视化角度映射，将左右 v5 六轴全部调整为单圈 `revolute` 限位；未使用 `continuous`。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`、`ros2_ws/src/alfa_robot_moveit_config/config/joint_limits.yaml`、`ros2_ws/src/alfa_robot_moveit_config/config/initial_positions.yaml`，新增 `docs/V5_JOINT_LIMIT_POLICY.md`。
- 换算规则：按旋转方向不变处理，即 `actual_angle = program_angle + offset`；因此 `left_v5_joint3/5` 写为 `180~540°` 初始 `360°`，`right_v5_joint4` 写为 `0~360°` 初始 `180°`，其余按用户映射得到对应单圈区间。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给机械/电控工程师：请重点确认非零程序偏移关节的驱动零点与实际零点关系：`left_v5_joint2/3/5`、`right_v5_joint1/2/4`；当前 MoveIt 和 ros2_control 初始角已按程序侧偏移写入。

## 2026-05-16 运控工程师 / Codex / description 预览初始角同步
- 做了什么：排查 `ros2 launch alfa_robot_description view_alfa_robot.launch.py` 看不到限位调整后初始姿态变化的问题。
- 判断原因：该 launch 实时加载的 xacro 限位已经是新的，但只启动 `joint_state_publisher_gui`，不会读取 MoveIt 的 `initial_positions.yaml`；GUI 对不跨 0 的关节会默认放在区间中点，对跨 0 的关节会默认 0，因此用户期望的 remapped 初始角不会全部生效。
- 改了哪里：`ros2_ws/src/alfa_robot_description/launch/view_alfa_robot.launch.py`，新增 `use_v5_initial_positions` 参数，默认把 v5 remap 后的初始角通过 `zeros.*` 传给 `joint_state_publisher_gui`。
- 验证结果：`/usr/bin/python3 -m py_compile` 通过；`colcon build --packages-select alfa_robot_description` 通过；短启动 launch 成功。
- 使用说明：重新 source `ros2_ws/install/setup.bash` 后启动即可；如需看纯 URDF 默认行为，可加 `use_v5_initial_positions:=false`。

## 2026-05-16 运控工程师 / Codex / 修正 right_v5_joint1 单圈限位
- 做了什么：根据用户在 description 预览中复核结果，修正 `right_v5_joint1` 初始化位姿和滑条范围。
- 改了哪里：`right_v5_joint1` 从此前按偏移换算的 `[-90°, 270°] / init 90°` 改为程序侧直观 `[-180°, 180°] / init 0°`；同步 URDF、MoveIt joint_limits、initial_positions、description preview `zeros.*` 和 `docs/V5_JOINT_LIMIT_POLICY.md`。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给机械/电控工程师：`right_v5_joint1` 现在以程序/GUI 0 位作为初始直观姿态，不再使用 +90° 程序偏移；不要按上一版 `-90:-180` 映射自动改回。

## 2026-05-16 运控工程师 / Codex / 修正 right_v5_joint4 单圈限位
- 做了什么：根据用户指出 rightjoint4 与 rightjoint1 同类问题，将 `right_v5_joint4` 初始化位姿和滑条范围改为 description/GUI 直观语义。
- 改了哪里：`right_v5_joint4` 从此前按偏移换算的 `[0°, 360°] / init 180°` 改为程序侧直观 `[-180°, 180°] / init 0°`；同步 URDF、MoveIt joint_limits、initial_positions、description preview `zeros.*` 和 `docs/V5_JOINT_LIMIT_POLICY.md`。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给机械/电控工程师：`right_v5_joint4` 现在以程序/GUI 0 位作为初始直观姿态，不再使用 +180° 程序偏移；不要按上一版 `0:-180` 映射自动改回。

## 2026-05-16 运控工程师 / Codex / Pick-Place IK baseline 初版
- 做了什么：在 `alfa_robot_benchmarks` 中新增确定性 `pick_place_baseline` C++ 可执行，作为 BioIK baseline 的第一版。
- 流程来源：复用 `pick_place_demo.py` 当前启用的 `PICK_POINTS` 顺序；每轮执行 `安全位 → 接近位 → 抓取位 → 后退位`。
- 关键语义：每一步 IK seed 使用上一步成功求解后的关节角，模拟连续执行，不再每个点从 home 重新开始。
- 改了哪里：新增 `src/pick_place_baseline_main.cpp`；`CMakeLists.txt` 安装该可执行；`IkSolver` 增加 `dual_v5_arm_with_base` / `dual_v5_arm` 的双臂识别；README 增加用法。
- 留给后续 AI：当前 baseline 只做 IK 求解与末端误差记录，尚未接入 PlanningScene 碰撞检测、关节限位 margin、轨迹级代价和 BioIK 自定义 cost；这些应作为下一阶段扩展。
- 追加说明：`pick_place_baseline` 默认补偿 v5 URDF 中 `link6 -> tool0` 的局部 `+Z 0.1m` 固定偏移；公开 `result.pos_error` 记录期望 tool0 与实际 tool0 的误差，原始补偿 IK 误差保存在 `ik_result_raw`。

## 2026-05-16 运控工程师 / Codex / 修复 benchmark Rerun 时间轴 API
- 做了什么：修复 `visualize_rerun.py` 在 Rerun SDK 0.31.4 下 `rr.set_time_sequence` 不存在导致崩溃的问题。
- 改了哪里：新增 `set_sample_time()` 兼容包装；旧 SDK 继续用 `set_time_sequence`，新 SDK 使用 `rr.set_time("sample", sequence=i)`。
- 验证建议：重新运行 `python3 ros2_ws/src/alfa_robot_benchmarks/scripts/visualize_rerun.py /tmp/pick_place_baseline.jsonl`。

## 2026-05-16 运控工程师 / Codex / 修复 benchmark Rerun 时间轴 API
- 做了什么：修复 `visualize_rerun.py` 在 Rerun SDK 0.31.4 下 `rr.set_time_sequence` 不存在导致崩溃的问题。
- 改了哪里：新增 `set_sample_time()` 兼容包装；旧 SDK 继续用 `set_time_sequence`，新 SDK 使用 `rr.set_time("sample", sequence=i)`。
- 验证建议：重新运行 `python3 ros2_ws/src/alfa_robot_benchmarks/scripts/visualize_rerun.py /tmp/pick_place_baseline.jsonl`。

## 2026-05-16 运控工程师 / Codex / benchmark Rerun 显示机器人模型
- 做了什么：升级 `alfa_robot_benchmarks/scripts/visualize_rerun.py`，不再只显示末端点位；默认加载当前 `alfa_robot_description` xacro/URDF visual mesh，并按 JSONL 中每帧 `result.joint_values` 做离线 FK 回放整机姿态。
- 改了哪里：脚本内新增轻量 URDF parser、mesh 路径解析、关节 FK、机器人静态 mesh 日志和逐帧 link transform 日志；保留 `--no-robot` 只看点位，新增 `--no-meshes` 和 `--robot-path`。
- 验证结果：`py_compile` 通过；`/usr/bin/python3 .../visualize_rerun.py /tmp/pick_place_baseline.jsonl --save /tmp/pick_place_baseline_robot.rrd` 成功，输出 root=world、links=21、meshes=16、12 samples。
- 留给后续 AI：当前机器人姿态来自 JSONL 的 IK 关节解，不包含连续插值轨迹；下一步若要看运动过程残影/轨迹质量，需要 baseline 输出 stage 间插值或规划轨迹点。
- 追加修复：脚本在未手动 source 工作空间时，也会自动通过 `ros2_ws/install/setup.bash` 调用 xacro，避免 `$(find alfa_robot_description)` 失败。

## 2026-05-16 运控工程师 / Codex / benchmark IK 接入碰撞过滤
- 问题确认：`alfa_robot_benchmarks::IkSolver::solveDual()` 原先传给 `searchPositionIK()` 的 `IKCallbackFn` 是空的，BioIK 只满足末端位姿，不会否定碰撞解；用户在 Rerun 中看到的碰撞是合理反馈。
- 修复内容：给 `IkSolver` 创建 `planning_scene::PlanningScene`，在单臂/双臂 IK callback 中调用 PlanningScene 碰撞检查；默认 `reject_collisions=true`，碰撞候选解返回 `GOAL_IN_COLLISION`，不再作为成功解。
- 结果字段：`IkResult` / JSONL 新增 `collision_checked`、`collision_free`、`collision_rejection_count`、`collision_pairs`。
- 诊断模式：`pick_place_baseline --allow-collision-solutions` 可复现旧行为但仍记录碰撞对，用于定位建模问题。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks` 通过；默认拒绝碰撞模式下第一个 `round_1/safe` 被拒绝；诊断旧行为显示 `round_1/safe` 确实包含 `left_v5_link6 <-> updown`、`right_v5_link6 <-> updown`。
- 重要备注：SRDF 中 arm-vs-base/pitch/turn 大量被 disable，但 `left/right_v5_link6 <-> updown` 没有 disable，因此 PlanningScene 能抓到这类碰撞；后续应由机械/建模继续确认 `updown.STL` 与手臂连杆的 collision mesh 是否过大或安装关系是否需要调整。

## 2026-05-16 运控工程师 / Codex / baseline 碰撞过滤改为多 seed 搜索
- 背景：用户指出“默认碰撞候选解会被拒绝”如果只做一次 IK 就会变成看运气；这个判断正确。碰撞 callback 只能作为安全门，不能替代搜索/优化。
- 改动：`pick_place_baseline` 默认先用上一阶段成功关节姿态作为 attempt 0；如果碰撞/失败，再用确定性扰动 seed 多次重试，直到找到无碰撞解或尝试次数耗尽。
- 新参数：`--seed-attempts`、`--seed-noise`、`--updown-seed-noise`；保留 `--allow-collision-solutions` 作为诊断旧行为。
- 设计语义：baseline 仍保持“每阶段从上一次结束姿态开始”，但不再把第一个碰撞候选解等同于目标不可达。

## 2026-05-16 运控工程师 / Codex / baseline 碰撞过滤改为多 seed 搜索
- 背景：用户指出“默认碰撞候选解会被拒绝”如果只做一次 IK 就会变成看运气；这个判断正确。碰撞 callback 只能作为安全门，不能替代搜索/优化。
- 改动：`pick_place_baseline` 默认先用上一阶段成功关节姿态作为 attempt 0；如果碰撞/失败，再用确定性扰动 seed 多次重试，直到找到无碰撞解或尝试次数耗尽。
- 新参数：`--seed-attempts`、`--seed-noise`、`--updown-seed-noise`；保留 `--allow-collision-solutions` 作为诊断旧行为。
- 设计语义：baseline 仍保持“每阶段从上一次结束姿态开始”，但不再把第一个碰撞候选解等同于目标不可达。
- 验证补充：使用 `--seed-attempts 48 --seed-noise 1.2 --updown-seed-noise 0.2 --rounds 1` 复跑后，`round_1/safe` 仍未找到无碰撞解；高频碰撞对包括左右臂互碰，以及 `left_v5_link2 <-> updown`、`right_v5_link2/right_v5_link3 <-> updown`。这说明当前 safe 目标/模型 collision mesh 组合本身高度可疑，不能简单归因于第一个候选解运气差。

## 2026-05-16 运控工程师 / Codex / RViz description 预览增加碰撞监控入口
- 问题说明：`ros2 launch alfa_robot_description view_alfa_robot.launch.py` 只显示 `RobotModel` visual，不加载 MoveIt PlanningScene，所以碰撞不会自动变红。
- 新增工具：`alfa_robot_benchmarks/collision_state_monitor`，订阅 `/joint_states`，使用当前 URDF + SRDF + MoveIt PlanningScene 做碰撞检查，发布 `/alfa_collision_markers`。
- RViz 改动：`alfa_robot_description/rviz/alfa_robot.rviz` 增加 `CollisionStatus` MarkerArray 显示，订阅 `/alfa_collision_markers`。
- 显示语义：无碰撞显示绿色 `collision free`；有碰撞显示红色 `COLLISION`、碰撞 link 对和接触点红球。注意它是叠加 marker，不会直接把 RobotModel mesh 改色。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks alfa_robot_description` 中 `collision_state_monitor` 构建通过；短时启动节点并发布 `/joint_states` 后，可 echo 到 `/alfa_collision_markers`。
- 追加修复：修正 `alfa_robot_description/rviz/alfa_robot.rviz` 中 `Displays` 缩进，确保 RViz 能正常加载新增 `CollisionStatus` MarkerArray。
## 2026-05-16 运控工程师 / Codex / baseline失败候选导出
- 做了什么：增强 `pick_place_baseline` 的 attempts JSONL 输出，失败候选也完整记录 seed、joint_values、碰撞对和误差。
- 改了哪里：`scripts/ik_benchmark/src/pick_place_baseline_main.cpp`；并在 `scripts/ik_benchmark/src/ik_solver.cpp` 保存首个被碰撞拒绝候选解。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks` 通过；测试输出中失败 attempt 已含 13 维关节值和 collision_pairs。
- 留给下个 AI：后续可直接用 JSONL 中 `attempts[*].joint_values` 复现/可视化失败姿态，定位碰撞来源。
## 2026-05-16 运控工程师 / Codex / 修复 dual_v5 BioIK 左右目标反绑
- 做了什么：确认 `dual_v5_arm_with_base` 在 BioIK 多 tip 求解中目标数组与实际 tip 绑定相反，导致左手追右目标、右手追左目标。
- 改了哪里：`scripts/ik_benchmark/src/ik_solver.cpp` 在 `solveDual()` 内保持 public API 为 left/right，但传给 BioIK 时交换 targets 顺序；`pick_place_baseline` 增加 direct/swapped tip match 诊断字段。
- 验证结果：修复前 `direct_pos_error≈0.6`、`swapped_pos_error≈0`；修复后 `direct_pos_error≈1e-6`、`swapped_pos_error≈0.6/1.0`，完整 3 轮无 swapped_better。
- 留给下个 AI：JSONL 中 `tip_target_match` 可继续用于防止后续改 SRDF/BioIK 配置时再次出现左右末端反绑。
## 2026-05-16 运控工程师 / Codex / baseline 增加放置后安全位
- 做了什么：在 `pick_place_baseline` 每轮 `safe→approach→grasp→retreat` 后增加 `place_safe`，用于前往下一个目标点前先回到放置后安全位置。
- 改了哪里：`scripts/ik_benchmark/src/pick_place_baseline_main.cpp` 的 `makeRoundStages()` 和 JSONL header flow。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks` 通过；`--rounds 1` 输出第 5 阶段 `round_1/place_safe`，目标为 left `(0.6,0.2,0.6)`、right `(0.6,-0.2,0.6)`。
- 留给下个 AI：当前新增 `place_safe` 阶段会实际参与 IK/碰撞过滤；本次验证里该阶段失败，可用 Rerun/JSONL 继续检查姿态与碰撞原因。

## 2026-05-16 项目经理 / Codex / Linear 议题补充任务改为本地文档指派
- 做了什么：用户要求不是由 PM 直接在 Linear 创建议题，而是写入本地协作文档，让运控工程师 Codex 自行阅读并补充 Linear 议题。
- 改了哪里：`.ai_teamwork/TASKS.md` 新增 T-0037，要求运控工程师在 Linear 项目 `v5机械臂水管版运控全流程项目推进` 的 `运动学算法闭环` 里程碑下补充开始中/未开始议题。
- Linear 处理：误创建的 `TIM-30 PM-0001 运控工程师补充运动学算法闭环议题清单` 已改为 `Canceled`，保留为误操作记录。
- 留给运控工程师：不要重复创建已完成工作；围绕 `T-0001 baseline 标准流程` 判断是否需要新 issue，能放在 T-0001 内部步骤的不要单独拆。
## 2026-05-16 运控工程师 / Codex / Linear 运动学算法闭环议题补充
- 做了什么：按 T-0037 到 Linear 项目 `v5机械臂水管版运控全流程项目推进` / 里程碑 `运动学算法闭环` 补充后续未开始议题。
- 新增 Linear：`TIM-31/T-0006` 候选解评分器与代价分解；`TIM-32/T-0007` 双臂安全距离与近碰撞惩罚；`TIM-33/T-0008` Rerun 候选/失败/代价对比可视化。
- 关联关系：`TIM-31`、`TIM-32` blocked by `TIM-25/T-0005`；`TIM-33` blocked by `TIM-31`；并在 `TIM-25` 留评论说明边界。
- 未新增原因：V5 限位、碰撞状态可视化、失败候选导出、BioIK 左右反绑、Rerun 最小机器人显示已有 Done issues；`place_safe` 失败归入 `TIM-25` 主线，不单独拆。
- 追加修正：按用户反馈重写 `TIM-31/32/33` 标题和描述，改为面向人类协作的高信息密度说明，减少抽象术语。
## 2026-05-17 运控工程师 / Codex / Linear 完成 T-0005 并拆分 T-0006 三路线
- 做了什么：按用户确认将 Linear `TIM-25/T-0005` 标记 Done；完成评论说明单轮 baseline 已跑通，多轮失败接受为碰撞/不可达，不阻塞主线。
- 更新 Linear：重写 `TIM-31/T-0006` 为三路线探索总任务，目标是让 baseline 从“能到”升级为“选得更好”。
- 新增 Linear：`TIM-34/T-0006A` 规则差分式 updown 搜索；`TIM-35/T-0006B` 代价注入；`TIM-36/T-0006C` 多次求解后外部评分。
- 关联关系：`TIM-31` blocks `TIM-34/35/36`；三条路线与 `TIM-32` 双臂安全距离、`TIM-33` Rerun 对比可视化保持相关。
- 追加修正：按用户新定义更新 A 方案。`TIM-34/T-0006A` 不再是左右臂分开 IK，也不做差分搜索；改为“可达范围查表选最小 updown 移动 + 固定 updown 后双臂 BioIK 联合求解”。
