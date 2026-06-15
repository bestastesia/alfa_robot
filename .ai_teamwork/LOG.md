# AI 协作日志

这里仅保留当前分支仍需让新 AI 立刻看到的最新交接。长过程和已完成事项已归档。

## 归档索引

- `.ai_teamwork/archive/2026-05-13_direction_reset/LOG.full_history.before_reset.md`
- `.ai_teamwork/archive/2026-05-16_pm_handoff/LOG.before_cleanup.md`
- `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/LOG.before_archive.md`
- `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/COMPLETED_SUMMARY.md`

默认不要读归档；只有追溯历史原因、验收证据、责任边界或恢复旧方案时再查。

## 2026-05-18 项目经理 / Codex / v5_dev 协作文件归档

- 做了什么：切换到 `v5_dev`，将 2026-05-16～2026-05-18 的长日志、已完成摘要和归档前状态备份到 `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/`。
- 改了哪里：精简 `.ai_teamwork/LOG.md`、`.ai_teamwork/TASKS.md`、`.ai_teamwork/NOW.md`；新增本次 archive 目录和 `COMPLETED_SUMMARY.md`。
- 当前保留任务：只保留 T-0030/T-0031/T-0032/T-0036/T-0037 这类仍可能需要处理的任务；已完成的 T-0025/T-0026/T-0027/T-0029/T-0033/T-0034/T-0035/TIM-37 移入归档摘要。
- 留给下个 AI：开工仍先读 `.ai_teamwork/START.md`、`.ai_teamwork/NOW.md`、`.ai_teamwork/TASKS.md`；不要把 archive 里的旧任务当成当前待办。

## 2026-05-22 运控 / Codex / IK 选优 Linear 结构调整

- 做了什么：根据用户确认，将原 `MOTION-4` 回到 Backlog；它保留为 baseline 多 seed 候选评分方向，不再承接 MOTION-6 后续优化。
- 新建任务：`MOTION-29` 作为 `MOTION-15` 下的新父任务：`IK 选优探索 / Updown 查表后续：多 h × 多 seed 候选选优`，related 到 `MOTION-6` 和 `MOTION-4`。
- 子任务：`MOTION-30` 验证多 h × 多 seed 并行 IK 求解耗时；`MOTION-31` 设计多 h × 多 seed 候选评分代价函数。
- 留给下个 AI：`MOTION-6` 保持 Done；MOTION-6 的耗时/最近 h 不一定最优问题不要重开原任务，转到 MOTION-29/30/31 推进。

## 2026-05-22 运控 / Codex / MOTION-30 并行 IK benchmark 第一版

- 做了什么：新增 `scripts/ik_benchmark/src/parallel_ik_benchmark_main.cpp` 并注册 `parallel_ik_benchmark`，用于测试多 h × 多 seed 的串行/并行 IK wall time。
- 测试数据：success=原 `pick_place_demo.py` PICK_POINTS；mixed=仅预抓取/抓取/后退 z-0.4；mostly_fail=仅预抓取/抓取/后退 z-0.8；unreachable=上述数据整体 x+0.5；safe/place_safe 高度保持原始设计。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks` 通过；smoke test `--limit-cases 1 --h-candidates 2 --seed-attempts 2 --workers-list 1,2 --timeout 0.05` 可运行，产物 `/tmp/parallel_ik_benchmark_smoke4.jsonl`。
- 重要发现：拆分 `init_ms` 和 `solve_wall_ms` 后，并行在 timeout 多的样本上有 solve wall time 收益；但线程并行下成功/碰撞/左右绑定结果与串行存在不一致，后续必须重点验证 BioIK/MoveIt 线程安全，或改进为进程级并行对照。
## 2026-05-22 运控 / Codex / MOTION-30 测试数据校准

- 做了什么：校准 `parallel_ik_benchmark` 数据集，把 `place_safe` 默认高度从 0.6 提到 0.85，避免 success/mixed 被明显范围外点污染。
- 改了哪里：默认扩展 3 组 PICK_POINTS 为 15 组轻微 xyz 扰动点；dataset JSON 记录每个阶段的 h 区间和 `h_reachable`。
- 验证结果：expanded 下 success=75/75 在查表范围内，mixed=74/75，mostly_fail=57/75，unreachable=0/225；构建通过。
- 留给下个 AI：做并行耗时统计时不要加 `--no-unreachable-grid`，否则 unreachable 没候选，无法测失败超时成本。

## 2026-05-22 运控 / Codex / MOTION-30 staged 并行 IK 验证

- 做了什么：重写 `parallel_ik_benchmark` 为按 pick/place 顺序推进的 staged episode；每阶段继承上一阶段选中的 `h` 和 seed。
- h 策略：先把 `current_h` clamp 到双臂共同可达区间，得到最小运动 `h`，再围绕该 `h` 生成多个候选。
- 左右绑定：调用 `IkSolver::solveDual(left, right)`，并用 FK 做 direct/swapped 误差检查；当前 smoke test 未接受反绑解。
- 验证结果：`--h-candidates 5 --seed-attempts 4 --workers-list 1,2,4 --allow-collision-solutions` 单 episode 全阶段成功；workers=4 wall time 约为串行 1/3。

## 2026-05-22 运控 / Codex / MOTION-30 数据集重构

- 做了什么：重构 `parallel_ik_benchmark` 数据集生成：先生成原始/z-0.4/z-0.8/轻微扰动/极端偏移候选池，再按可达与不可达池拼 `success/mixed/mostly_fail/unreachable`。
- 默认策略：`--dataset-prefilter sphere`，用可达球快速分池；保留 `--dataset-prefilter ik` 作为慢速精筛。
- 验证结果：构建通过；sphere smoke 可运行，header 会记录候选池、可达池、不可达池和各 profile 数量。

## 2026-05-23 运控 / Codex / ParallelUpdownAwareIkSolver 初版

- 做了什么：新增集中式 C++ core `ParallelUpdownAwareIkSolver`，把 h 规划、候选生成、并行 BioIK、FK 校验、fallback、基础 cost 放入一个求解器类。
- 文件：`include/ik_benchmark/parallel_updown_aware_ik_solver.h`、`src/parallel_updown_aware_ik_solver.cpp`、`src/parallel_updown_ik_demo_main.cpp`、`config/parallel_updown_aware_ik.yaml`。
- 验证：构建通过；`parallel_updown_ik_demo` 的 fixed_discrete 与 continuous_range smoke 都可运行。
- 注意：continuous_range 当前用 seed 限制 h 小范围，若 BioIK 跳出范围会被 validator 拒绝，然后可进入 release_updown_fallback；后续需验证/增强临时 joint bound 或 consistency limit。

## 2026-05-23 运控 / Codex / Updown IK 默认代价函数更新

- 做了什么：将 `ParallelUpdownAwareIkSolver` 默认 cost 改为 updown 分段奖励/惩罚 + joint2/3 力矩 proxy。
- 参数：新增 updown 静止奖励、0.1m 内运动奖励、0.1m 外距离惩罚、joint2/3 torque 权重、水平角零点和连杆 proxy 参数；模板见 `config/parallel_updown_aware_ik.yaml`。
- 验证：构建通过；`parallel_updown_ik_demo --h-mode fixed_discrete` 可运行。
- 注意：该 solver 当前仍在 `alfa_robot_benchmarks` 包内；后续应迁出为正式 ROS IK 求解包，再做 service/action 与 MoveIt 插件/adapter 接入。

## 2026-05-24 运控 / Codex / Updown solver 对比 benchmark

- 做了什么：新增 `updown_solver_comparison`，对比三组：完全不限 updown 的 BioIK 随机 seed、lookup-like fixed-h+fallback、新 `ParallelUpdownAwareIkSolver`。
- 输出位置：默认 `data/ik_benchmark/updown_solver_comparison.jsonl`，不再写 `/tmp`；已支持 `--max-stages`、`--fallback-timeout` 方便 smoke。
- 验证：构建通过；`--max-stages 1 --timeout 0.02 --fallback-timeout 0.05` smoke 可生成 header/stage/summary。
- 可达球边缘精扫脚本：`ros2_ws/src/alfa_robot_moveit_config/scripts/x_edge_refine_reachability.py`；全量九向脚本是 `nine_orient_reachability.py`。

## 2026-05-24 运控 / Codex / 可达球参数更新与 cost 同步

- 做了什么：按新模型将默认可达球更新为左 `(0.015, 0.3125, 0.6625)`、右 `(0.015, -0.3125, 0.6625)`、半径 `0.815`。
- 影响范围：`ParallelUpdownAwareIkSolver`、`parallel_updown_aware_ik.yaml`、`pick_place_updown_lookup`、`parallel_ik_benchmark`。
- 验证：`colcon build --packages-select alfa_robot_benchmarks` 通过。
- Linear：已回复 MOTION-30 新评论；已在 MOTION-31 写入当前默认代价函数表达式。

## 2026-05-24 运控 / Codex / Updown 对比实验图表
- 做了什么：把本次 updown solver comparison 结果生成中文图表和汇总表。
- 改了哪里：新增 `scripts/ik_benchmark/scripts/plot_updown_solver_comparison.py`；补充 comparison JSONL 后续记录 selected joints 与 joint2/3 力臂字段。
- 验证结果：`alfa_robot_benchmarks` 构建通过；图表输出到 `data/ik_benchmark/updown_solver_comparison/charts/`。
- 留给下个 AI：当前历史 JSONL 未含关节角，因此 joint2/3 力臂图只有说明；重新跑实验后会生成真实力臂曲线。

## 2026-05-24 电控顾问 / Codex / 六个关节电机自搭机械臂方案咨询
- 做了什么：围绕用户计划用公司闲置的 6 个相同关节电机自搭机械臂学习电气/电控，提供区别于常规 2+1+3 六轴机械臂的结构创意方向。
- 改了哪里：仅追加本协作日志；未改代码与工程文件。
- 验证结果：不涉及构建/测试。
- 留给下个 AI：用户希望用低成本实物项目学习机械臂电控、伺服、线束、安全、控制，不急于全面系统学习。

## 2026-05-24 电控顾问 / Codex / 流行多轴机器人结构头脑风暴
- 做了什么：基于用户觉得常规 6 轴、双 3 轴方案不够有心意，补充当前更流行/更有展示感的多轴机器人结构方向。
- 改了哪里：仅追加本协作日志；未改代码与工程文件。
- 验证结果：不涉及构建/测试；答复结合 2025-2026 humanoid/mobile manipulation/dual-arm robotics 趋势资料。
- 留给下个 AI：用户倾向用 6 个相同关节电机做低成本实物学习平台，偏好“有心意”和能体现多轴能力的结构，而不是普通工业六轴臂。

## 2026-05-24 运控 / Codex / benchmark 超参数 YAML 化
- 做了什么：`updown_solver_comparison` 已支持从 `parallel_updown_aware_ik.yaml` 读取实验组、lookup-like、baseline 和流程超参数。
- 改了哪里：`scripts/ik_benchmark/src/updown_solver_comparison_benchmark.cpp`、`scripts/ik_benchmark/config/parallel_updown_aware_ik.yaml`。
- 验证结果：`alfa_robot_benchmarks` 构建通过；`yaml_config_smoke.jsonl` header 正确记录 YAML 参数。
- 留给下个 AI：后续调 h 数、seed 数、workers、joint2 权重都优先改 YAML；命令行只用于临时覆盖 output/rounds/timeout/workers 等运行项。

## 2026-05-24 电控顾问 / Codex / 12 轴大小扭矩关节混合机械臂方案咨询
- 做了什么：用户补充现有 6 个百牛米级大扭矩关节电机和 6 个小扭矩关节电机，电压不同，希望组合成单台 12 轴机械臂；提供结构分配、电气分压、安全与控制架构建议。
- 改了哪里：仅追加本协作日志；未改代码与工程文件。
- 验证结果：不涉及构建/测试。
- 留给下个 AI：重点原则是大扭矩关节放近端承重，小扭矩关节放远端做灵巧腕/末端/微动，不建议让小扭矩关节承受大臂重量。

## 2026-05-24 运控 / Codex / 实验组 fallback 增强
- 做了什么：将实验组 fallback 改为 global free-h 多 seed family 并行求解；每轮找到合法解即停止，再按现有 cost 排序。
- 改了哪里：`parallel_updown_aware_ik_solver`、`updown_solver_comparison_benchmark.cpp`、`parallel_updown_aware_ik.yaml`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；smoke 与强制 fallback 测试通过，强制场景选中 `global_free_h_fallback`。
- 留给下个 AI：完整实验需重跑并重新生成图表，关注第三轮实验组是否还缺失。

## 2026-05-24 运控 / Codex / 恢复对照组2 lookup baseline
- 做了什么：将对照组2从复用实验组全量候选池改回旧 lookup 语义：按 h/seed 顺序收集少量合法解后早停，再按最小 updown 运动选解。
- 改了哪里：`scripts/ik_benchmark/src/updown_solver_comparison_benchmark.cpp`。
- 验证结果：编译通过；`lookup_restored_smoke.jsonl` 中对照组2前三阶段 61 次 IK，实验组 400 次 IK，已恢复差异。
- 留给下个 AI：完整实验需重跑并重新生成 charts；不要再把对照组2改成实验组全量候选池。

## 2026-05-24 运控 / Codex / continuous_range seed 预算与去重
- 做了什么：新增 `continuous_seed_multiplier`，continuous 模式保留固定当前 h seed 组，并用连续 range seed 倍数补齐预算。
- 改了哪里：`parallel_updown_aware_ik_solver`、`updown_solver_comparison_benchmark.cpp`、`parallel_updown_aware_ik.yaml`。
- 验证结果：编译通过；`continuous_multiplier_dedup_smoke.jsonl` 可正常生成，header 记录 multiplier=10。
- 留给下个 AI：完整实验需重跑；同一阶段 trial 生成已加去重保护，避免完全相同 IK 输入重复求解。

## 2026-05-28 运控 / Codex / IK 候选审计表格与分页可视化
- 做了什么：扩展候选表输出，新增按流程节点拆分的候选 CSV；新增 `visualize_candidate_rerun.py` 支持按 stage/rank 分页查看候选机器人姿态。
- 改了哪里：`scripts/ik_benchmark/scripts/plot_updown_solver_comparison.py`，新增 `scripts/ik_benchmark/scripts/visualize_candidate_rerun.py`；`ros2_ws/src/alfa_robot_benchmarks` 路径为同一映射。
- 验证结果：用 `smoke_experiment_only.jsonl` 生成 `候选按任务拆分/01_round_1_safe_候选明细.csv`，96 行按合法/rank/score 排序；当前环境缺 `rerun` 包，Rerun 脚本仅验证 `--help`。
- 留给下个 AI：提交时记得包含新增脚本；若要运行 Rerun 可视化，需在安装 `rerun-sdk` 的 Python 环境执行。

## 2026-05-28 机械工程师 / Codex / 修正 v5 双臂 joint1 初始 0 位
- 做了什么：按用户确认，将当前需要的机械 0 位同步为左右 `joint1=30°`；demo/mock ros2_control 初始状态和 MoveIt SRDF `home` 均改为 `0.52359878 rad`。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/config/initial_positions.yaml`、`ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf`、`ros2_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro`。
- 验证结果：`xacro` 展开 description 和 MoveIt wrapper 均通过；`check_urdf` 均通过；展开后的 ros2_control 中 `left_v5_joint1/right_v5_joint1` 的 `initial_value` 均为 `0.52359878`；SRDF home 同步为 `0.52359878`。
- 留给下个 AI：这是初始姿态/演示 0 位修正，不是改 URDF 关节轴或 mesh；如果实机编码器也要把 30° 当逻辑 0，需要另做硬件侧 offset/标定任务，不能只靠 demo 初始值。

## 2026-05-28 机械工程师 / Codex / 补偿 joint1 初始偏置后的 tool0 姿态
- 做了什么：用户确认机械臂在 `joint1=30°` 时才是正的，但末端姿态不应随这 30° 一起偏转；因此在左右 `tool0_fixed` 上加入反向固定姿态补偿。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 中 `left_v5_tool0_fixed` 改为 `rpy="0 0 0.52359878"`，`right_v5_tool0_fixed` 改为 `rpy="0 0 -0.52359878"`。
- 验证结果：description 与 MoveIt wrapper 的 `xacro` 展开通过，`check_urdf` 均通过；FK 复算显示 `joint1=30°` 时左右 `tool0` 姿态与补偿前 `joint1=0°` 目标姿态误差约 `8.9e-7`。
- 留给下个 AI：这是末端 frame 姿态补偿，不改变 link6 mesh 或 joint1 轴；如果后续改变 joint1 初始偏置角，tool0_fixed 的 yaw 补偿也要同步更新。

## 2026-05-29 运控工程师 / Codex / 固定64次IK候选benchmark
- 做了什么：实验组 benchmark 改为固定正确 target order，可配置 8线程/10ms/64次主候选池；关闭 fallback 以保证每阶段固定预算。
- 改了哪里：`parallel_updown_aware_ik_solver` 增加 `use_reversed_target_order`，`parallel_updown_aware_ik.yaml` 改为 `h_candidate_count=8`、`seed_count=8`、`workers=8`、`timeout_per_trial=0.01`、`try_target_orders=false`、`fallback.enabled=false`。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks` 通过；单阶段 smoke 确认只生成 64 个候选且全部为已验证正确的 `swapped` 顺序。
- 留给下个 AI：当前 smoke 的 `round_1/safe` 在当前模型/配置下无合法解；即使用旧预算 160次/0.5s 也无合法解，说明需要先确认当前机械模型/目标姿态/姿态容差变化，而不是单纯调线程或 timeout。

## 2026-05-30 机械工程师 / Codex / 反向调整 tool0 姿态补偿
- 做了什么：根据用户反馈“方向反了”，将 `tool0_fixed` 的 yaw 补偿方向对调：左末端改为 `-30°`，右末端改为 `+30°`。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 中 `left_v5_tool0_fixed` 和 `right_v5_tool0_fixed` 的 `rpy`。
- 验证结果：description 和 MoveIt wrapper 的 `xacro` 展开通过；两份 `check_urdf` 通过。
- 留给下个 AI：这次是按 RViz/视觉语义反馈调整末端 frame 方向；若仍不对，下一步应在 RViz 中看 `left_v5_tool0/right_v5_tool0` 的 TF 轴，确认需要绕 tool 局部 Z 轴还是绕 world/updown Z 轴补偿。

## 2026-05-30 机械工程师 / Codex / 恢复 tool0 并隐藏重复 updown STL
- 做了什么：按用户要求，仅恢复本包中刚才的 `tool0_fixed` 姿态补偿试改，保留此前已确认的左右 `joint1=30°` 初始位；同时定位 updown/joint1 连接处重复 STL 来源。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`：`left/right_v5_tool0_fixed` 恢复 `rpy="0 0 0"`；移除 `updown` 的 visual/collision mesh，仅保留其 inertial/link/joint 作为升降和安装 frame。
- 验证结果：description 与 MoveIt wrapper 的 `xacro` 展开通过；两份 `check_urdf` 通过；展开后 `updown` 无 visual/collision，`left/right_v5_link0` 和 `link1` mesh 保留。
- 留给下个 AI：重复外观来源是旧 `alfa_robot_v2_arm_v5/visual/updown.STL` 与新 v5_6 机械臂 `link0/base.STL` 在安装区域叠加；不要删除 v5 `link0/base.STL`，否则会丢机械臂第一个基座/电机外观。

## 2026-05-30 机械工程师 / Codex / 保留 updown 并移除 v5 link0 base mesh
- 做了什么：根据用户纠正，恢复 `updown.STL` 的 visual/collision；移除左右 `left/right_v5_link0` 上的 v5_6 `base.STL` visual/collision，让 `joint1` 通过无外观的 link0 frame 直接接到 updown 的 STL 安装结构。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`。
- 验证结果：description 和 MoveIt wrapper 的 `xacro` 展开通过；两份 `check_urdf` 通过；展开后 `updown` 有 visual/collision，`left/right_v5_link0` 无 visual/collision，`left/right_v5_link1` 的电机 mesh 保留。
- 留给下个 AI：用户希望看到 updown 安装结构，不希望 v5_6 根部 `base.STL` 在 joint1 连接处重复叠加；不要再隐藏 updown。

## 2026-05-30 机械工程师 / Codex / 下移 joint1 去除 link0 base 悬空距离
- 做了什么：在 `alfa_robot.urdf.xacro` 中把左右 `*_v5_joint1` 的 origin 从 `xyz="0 0 0.056"` 改为 `xyz="0 0 0"`，因为 `link0` 的 `base.STL` 已经被去掉，不能再保留一个 base 厚度的安装高度。
- 保持不变：`updown.STL` 仍作为升降关节的可视/碰撞模型；`left/right_v5_link0` 仍作为无可视/无碰撞的安装坐标系；左右 joint1 初始值仍为 30°；`tool0_fixed` 仍为未补偿状态。
- 验证：已展开 description 与 MoveIt wrapper xacro，并通过 `check_urdf`；展开后的 `left_v5_joint1` 和 `right_v5_joint1` origin 均为 `xyz="0 0 0"`。

## 2026-05-30 机械工程师 / Codex / 禁用机械臂根部与 updown 自碰撞
- 做了什么：在 MoveIt SRDF 中仅新增 `updown` 与左右机械臂根部电机碰撞体 `left_v5_link1`、`right_v5_link1` 的禁碰，避免规划误认为根部 base 与 updown 自碰撞。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf`。
- 保持不变：没有禁用 `updown` 与其他机械臂 link 的碰撞，也没有改 URDF 几何安装位置。
- 验证：MoveIt wrapper xacro 展开与 `check_urdf` 通过；脚本确认 SRDF 中存在两对禁碰关系。

## 2026-05-30 机械工程师 / Codex / 将 joint1 的 30° 姿态烘为 0 位
- 做了什么：把用户认可的左右 `joint1=+30°` 姿态烘进 URDF joint origin，使控制/MoveIt 中 `left_v5_joint1=0`、`right_v5_joint1=0` 时呈现原来 `+30°` 的机械臂姿态。
- 改了哪里：`alfa_robot.urdf.xacro` 中 `left_v5_joint1` origin yaw 设为 `-0.52359878`，`right_v5_joint1` origin yaw 设为 `+0.52359878`；`initial_positions.yaml`、SRDF home、description ros2_control joint1 初值均改回 `0`。
- 末端姿态处理：没有额外旋转 `tool0_fixed`；通过 FK 对比确认新模型 `joint1=0` 时左右 `tool0` 的位置和姿态都与旧模型 `joint1=+30°` 完全一致。
- 验证：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本对比左右 `tool0` rotation/position error 均为 `0.000e+00`。

## 2026-05-30 机械工程师 / Codex / 最终：tool0 姿态与 SRDF 语义末端对齐
- 正确方法：先按 live `demo.launch.py` 全零关节状态计算左右 `tool0` 的平均姿态，再把 MoveIt SRDF 的 `end_effector parent_link` 从 `left/right_v5_link6` 改为 `left/right_v5_tool0`。
- 保留改动：`left_v5_tool0_fixed` rpy 为 `0.00000188634799516 0.00363531907027 0.523587766715`，`right_v5_tool0_fixed` rpy 为 `-0.00000188633751216 0.00363531853545 -0.523587767584`；末端 `xyz` 保持 `0 0 0.1`。
- 保留改动：SRDF 中 6 个 `end_effector parent_link` 全部指向 `*_v5_tool0`，因为 RViz/MoveIt 末端交互球之前使用 `link6`，而左右 `link6` 本身仍差约 60°。
- 不要恢复：不要使用早期只按 `joint2=1.57079633` 离线姿态算出的 `tool0_fixed` 补偿；不要把 SRDF 末端 parent 改回 `*_v5_link6`。
- 验证：用户在重启/刷新 MoveIt demo 后确认末端朝向正确；脚本验证 `left_v5_tool0 -> right_v5_tool0` 姿态为 0°，`check_urdf` 通过。

## 2026-05-30 运控工程师 / Codex / 固定64次IK smoke复测
- 做了什么：机械模型修正后复测固定64次实验组；定位到失败主因是 benchmark 仍按旧 `tool0_offset=0.1` 做补偿。
- 改了哪里：`parallel_updown_aware_ik.yaml` 中 `tool0_offset` 调整为 `0.0`，因为当前 IK tip 已是 `left_v5_tool0/right_v5_tool0`，继续减 0.1 会造成末端位置稳定偏差 0.1m。
- 验证结果：单阶段 smoke 在 `tool0_offset=0.0` 下恢复成功，`round_1/safe` 为 64 候选、46 合法、选中 rank1。
- 留给下个 AI：后续完整 benchmark 应使用固定64次配置：8线程、10ms、单一正确 target order、fallback关闭、tool0_offset=0.0。

## 2026-05-30 机械工程师 / Codex / 调整 pitch 关节安装偏移
- 做了什么：按用户要求调整 `pitch` revolute joint 相对 `base_link` 的安装位置，目标为 `x=0.3`、`y=-0.05`，保留原 `z=0.129` 和 `rpy=0 0 0`。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 中 `joint name="pitch"` 的 `<origin xyz>` 从 `-0.007937 0 0.129` 改为 `0.3 -0.05 0.129`。
- 验证：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；展开后两份 URDF 的 `pitch` origin 均为 `xyz="0.3 -0.05 0.129"`。

## 2026-06-01 Git 操作工程师 / Codex / 将 MOTION-10 仿真同步改动集成到当前分支
- 做了什么：确认 `b2ffbc56d87fdff607958d2ed1c0089c28d0b091` 对应的 MOTION-10 改动已在当前分支完成等价集成，并保留当前分支 v5_6 机械臂/关节限位状态。
- 改了哪里：重新生成 `simulation/mujoco/alfa_robot.xml`，使 MuJoCo XML 使用当前 description 的 v5_6 mesh、pitch 安装位姿和当前关节限位；未提交旁路存在的 `scripts/ik_benchmark` 未归档改动。
- 验证结果：`alfa_robot_description`、`alfa_robot_moveit_config` 编译通过；`SemanticScene` 消息可见；MuJoCo 三份 XML 加载通过。
- 留给下个 AI：如继续处理 IK benchmark 速度脚本，请单独确认并提交 `scripts/ik_benchmark` 的未提交改动，避免和 MOTION-10 集成混在一起。

## 2026-06-03 机械工程师 / Codex / 迁移 backpack v6 URDF 到主 ROS2 包
- 做了什么：检查 `/mnt/mydisk/ALFA/backpack/alfa_robot_v2_arm_v6` 后确认导出 URDF 只有 `left/rightjoint1-6`，没有工具末端 link 或固定 tool 偏移；已将 v6 meshes 复制到 `alfa_robot_description/meshes/alfa_robot_v2_arm_v6`，并保存 raw URDF 到 `urdf/vendor/alfa_robot_v2_arm_v6_raw.urdf`。
- 改了哪里：`alfa_robot.urdf.xacro` 已替换为 v6 几何/惯量/关节原点，并映射到现有 `left_v5_*`/`right_v5_*` 命名以兼容 MoveIt/controller；新增 `left/right_v5_tool0`，末端固定偏移暂按现有约定 `xyz="0 0 0.1" rpy="0 0 0"`；SRDF 保持 end-effector 指向 `*_v5_tool0`；joint2 MoveIt/ros2_control 范围同步为 v6 的 `[-3.14, 3.14]`。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本确认 v6 mesh 均走 `package://alfa_robot_description/...`、左右 `tool0` 姿态平行；干净环境下 `colcon build --packages-select alfa_robot_description alfa_robot_moveit_config --symlink-install --allow-overriding ...` 通过。
- 留给下个 AI：v6 导出限位 effort/velocity 为 0，当前沿用旧工程的 effort/velocity；`pitch/turn/updown` 的控制速度/effort 仍需机械/运控确认；tool0 的 `0.1m` 偏移是沿用旧吸盘/工具约定，不是 v6 CAD 导出结果，必须由机械确认真实 TCP；所有可达性/IK benchmark 需要基于新模型重跑。

## 2026-06-03 机械工程师 / Codex / 修正 v6 joint5 旋转方向
- 做了什么：用户确认 v6 迁移后 `joint5` 默认姿态/运动方向反了；采用最小语义修正，把左右 `left/right_v5_joint5` 的 URDF 轴从 `0 0 1` 改为 `0 0 -1`。
- 改了哪里：只改 `alfa_robot.urdf.xacro` 中左右 joint5 axis；`initial_positions.yaml`、SRDF home、description ros2_control 的左右 joint5 默认值保持 `+1.04719755`，让默认姿态随轴修正后变为用户看到的正确方向。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；展开后左右 joint5 axis 均为 `0 0 -1`，默认/home/ros2_control 仍为 `+1.04719755`。
- 留给下个 AI：这是关节语义方向修正，不改 STL 和 joint origin；如果硬件驱动侧已有 joint5 符号补偿，需避免重复取反。

## 2026-06-03 机械工程师 / Codex / 整体抬升机器人地面对齐
- 做了什么：用户用 Rerun Z 扫描确认车体最低齐平约在 `z=-0.195`，因此将整体模型相对 world 抬升 `+0.195m`。
- 改了哪里：`alfa_robot.urdf.xacro` 的 `world_to_base` fixed joint origin 从 `xyz="0 0 0.09"` 改为 `xyz="0 0 0.285"`。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；展开后两份 URDF 的 `world_to_base` origin 均为 `0 0 0.285`。
- 留给下个 AI：这是全局可视/TF 抬升，不改 base_link mesh 本身或内部机械关节；若仿真地面/导航地图另有 base frame 约定，需要同步确认。

## 2026-06-03 机械工程师 / Codex / 修正 Rerun Z 扫描后整体高度
- 做了什么：确认此前 Rerun Z 扫描未在扫描帧继承机器人静态 link transform，导致误判仍在 `z=-0.195` 相切；修正扫描脚本后，按 STL 包围盒最低点约 `+0.082906m` 将整体高度回调。
- 改了哪里：`scripts/tmp_rerun_x_front_scan.py` 在扫描前把当前 URDF 的 link FK 作为 static transforms 记录；`alfa_robot.urdf.xacro` 的 `world_to_base` fixed joint origin 从 `xyz="0 0 0.285"` 改为 `xyz="0 0 0.202094"`。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；展开后两份 URDF 的 `world_to_base` origin 均为 `0 0 0.202094`。
- 留给下个 AI：如果用户再次用 Rerun 验收，应运行修正后的 `scripts/tmp_rerun_x_front_scan.py --mode z-ground`；旧脚本或旧 `.rrd` 文件会继续显示错误的 `-0.195` 相切结果。

## 2026-06-03 运控工程师 / Codex / 修正箱垛 IK 顶吸顺序与 Rerun 目标高度
- 做了什么：将箱垛 demo 最后两轮顶吸顺序改为 `17+19`、`18+20`；排查侧吸看起来沿世界 Z 偏差较大的问题，确认主要是 Rerun 可视化目标/箱子仍用旧 `world_to_base z=0.09` 手动偏移，而当前 URDF 已是 `0.202094`。
- 改了哪里：`box_stack_dual_ik_benchmark_main.cpp` 更新顶吸配对；`visualize_box_stack_dual_ik.py` 改为从当前 URDF FK 自动读取 `base_link` 到 `world` 的变换，再转换箱子和目标点。
- 验证结果：`alfa_robot_benchmarks` 编译通过；新 10 轮数据保存到 `data/ik_benchmark/box_stack_dual_ik/box_stack_x0375_offset03_top_suction_17_19_18_20.jsonl`，Rerun 保存到同名 `.rrd`；前 8 轮侧吸仍为 6/8 成功，顶吸进入 IK 求解但仍无合法解。
- 留给下个 AI：侧吸成功轮 IK 本身误差约 2~20mm，若用户继续觉得姿态不贴合，应优先看 `selected_direct_pos_error` 与目标/FK误差向量，而不是旧 Rerun 目标高度；顶吸失败原因主要仍是 `tip_error_too_large/tip_order_error`。

## 2026-06-03 运控工程师 / Codex / 校准箱垛 IK 世界坐标与 base_link 坐标
- 做了什么：确认箱子/抓取点是 world 地面坐标，但 MoveIt IK 请求使用 `base_link` 坐标；此前直接把 world z 送进 IK，导致吸点整体沿 Z 偏移。
- 改了哪里：`box_stack_dual_ik_benchmark_main.cpp` 中送入 IK 的目标 z 统一减去当前 `world_to_base z=0.202094`，同时 JSON 额外保留 `*_target_world` 方便审计；Rerun 中箱子保持 world 地面，目标点由 base 坐标通过当前 URDF FK 转回 world 显示。
- 验证结果：新数据 `box_stack_x0375_offset03_z_calibrated.jsonl` 中 round1 base z `1.597906` 转回 world z `1.8`，round9 顶吸 base z `0.197906` 转回 world z `0.4`；前 8 轮侧吸本轮全部成功，顶吸仍无合法解。
- 留给下个 AI：后续如果 `world_to_base` 再改，必须同步这个 benchmark 的坐标转换；更长期应改成从 URDF/TF 自动读，而不是保留常量。

## 2026-06-03 运控工程师 / Codex / 修正 v6 tool0 相对 link6 的真实吸盘延伸
- 做了什么：用户从 Rerun 发现 `link6` STL 本体贴到箱子；复查 v6 `link6` STL 包围盒发现其局部 Z 已延伸到约 `+0.159m`，原 `tool0_fixed xyz=0 0 0.1` 只是从 link6 原点延伸，不是从 link6 物理末端再延伸 0.1m。
- 改了哪里：`alfa_robot.urdf.xacro` 中左右 `*_v5_tool0_fixed` 从 `xyz=0 0 0.1` 改为 `xyz=0 0 0.259`；Rerun 箱垛可视化增加 `link6 -> tool0` 延伸箭头和 tool0 marker。
- 验证结果：`alfa_robot_description`、`alfa_robot_moveit_config`、`alfa_robot_benchmarks` 编译通过；新箱垛数据 `box_stack_x0375_offset03_tool0_0259.jsonl` 中第1轮 link6 到目标约 `0.259m`，tool0 到目标约 `0.012m`。
- 留给下个 AI：`0.259m = link6 STL max local z 0.159m + 工具延伸 0.1m` 是基于当前 v6 STL 的工程近似；若机械确认 TCP/吸盘长度不同，应同时更新 URDF 和所有依赖 tool0 的 benchmark/MoveIt 验收。

## 2026-06-03 机械工程师 / Codex / 同步双臂 6 轴关节限位
- 做了什么：按用户给定机械限位同步左右双臂 6 轴：joint1 ±135°，joint2/3/4/6 ±180°，joint5 ±145°。
- 改了哪里：`alfa_robot.urdf.xacro` 的左右 `left/right_v5_joint1-6` `<limit>`；`alfa_robot_macro.ros2_control.xacro` 的 mock/控制接口 command min/max；`joint_limits.yaml` 的 MoveIt position limits。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本核对展开后左右 12 个关节限位均为目标弧度值。
- 留给下个 AI：本次只改位置限位，未改 velocity/acceleration/effort；用户原文重复写了两次 joint3，按常规理解处理为 joint3 与 joint4 都是 ±180°。
## 2026-06-07 运控 / Codex / motion-5 吸收 PLC bridge 并丢弃 ros2_tmp 临时框架
- 做了什么：在 `emoji-father/motion-5-updown-lookup-bioik` 上以 merge 方式接入 `fixed-platform-dual-arm-acceptance-20260605` 历史，但只吸收 `ros2_tmp` 中可复用的 PLC 执行/安全包。
- 改了哪里：新增 `ros2_ws/src/alfa_robot_plc_bridge/`；未保留 `ros2_tmp/` 临时全流程框架；外部备份目录 `/mnt/mydisk/ALFA/alfa_robot_ec` 已按用户要求删除。
- 验证结果：`source /opt/ros/humble/setup.bash && colcon build --packages-select alfa_robot_plc_bridge --symlink-install` 通过。
- 留给下个 AI：后续主线只从 `ros2_ws/src/alfa_robot_plc_bridge` 继续整理 PLC 接口；不要再依赖 `ros2_tmp` 的临时任务编排/IK/MoveIt glue。
## 2026-06-08 运控 / Codex / MoveIt 4x5 箱垛全流程复现服务
- 做了什么：把 `dual_arm_planner_node` 从随机 demo 改成可请求的 MoveIt 全流程复现节点：预抓取固定关节姿态 → 双末端 IK 抓取位 → 负载固定姿态 → 放置固定姿态；支持 4x5 箱垛前 8 轮侧吸和后 2 轮顶吸。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`、`ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py --show-args` 参数正常。
- 留给下个 AI：RViz 弹回/多臂显示的核心风险是多路 `/joint_states` 或 mock controller 状态未持续更新；该节点默认 `prefer_commanded_state:=true`，用于连续规划时以最后命令状态作为下一段起点，但 RViz 是否弹回仍取决于 `/joint_states` 发布源是否唯一且正确。

## 2026-06-08 运控 / Codex / MoveIt 全流程改用自研 IK 选优并记录 Rerun 回放
- 做了什么：`dual_arm_planner_node` 的抓取 IK 从 MoveIt 原生 `setFromIK` 改为旧 benchmark 的“离散 h × 多 seed × cost scorer”方案，MoveIt 只负责关节轨迹规划/执行。
- 改了哪里：`dual_arm_planner_node.cpp` 接入 `ParallelUpdownAwareIkSolver`，新增 JSONL 轨迹记录；`dual_arm_planner.launch.py` 暴露 h/seed/线程/timeout/记录路径参数；新增 `visualize_moveit_box_stack_flow.py` 将规划轨迹转成 Rerun。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`dual_arm_planner.launch.py --show-args` 参数正常。
- 留给下个 AI：IK 输入沿用旧 benchmark 的 base_link 语义，默认 `world_to_base_z=0.202094`；若底盘/URDF 基准变化，要同步该参数或改成 TF 自动读取。

## 2026-06-08 运控 / Codex / 修复 dual_arm_planner 启动缺 bio_ik/move_group
- 做了什么：用户启动 `dual_arm_planner.launch.py` 报 `bio_ik/BioIKKinematicsPlugin` 不存在；确认 `bio_ik` 源码在工作区但此前未进入 overlay，补构建并把 `bio_ik` 写入 MoveIt config 运行依赖。
- 改了哪里：`package.xml` 增加 `bio_ik` exec 依赖；`dual_arm_planner.launch.py` 不再把 kinematics.yaml 注入 planner 节点，并默认同时启动 `move_group`，自研 IK 只由 benchmark solver 内部加载 `bio_ik`。
- 验证结果：`colcon build --packages-select bio_ik alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动确认 `move_group` 和 `dual_arm_planner` ready，`bio_ik` 插件错误消失。
- 留给下个 AI：运行该 planner 前必须 source 当前 `ros2_ws/install/setup.bash`；如果换机器，要先构建 `bio_ik`，否则插件列表只有 KDL/LMA/pick_ik。

## 2026-06-09 运控 / Codex / 修复 dual_arm_planner 执行控制器缺失
- 做了什么：用户调用 `/dual_arm_planner/run_box_stack_flow` 返回 `MoveIt execute failed, code=-4`；确认 `-4=CONTROL_FAILED`，原因是原 launch 只启动 `move_group` 和 planner，没有启动 demo 里的 ros2_control/mock 控制器。
- 改了哪里：`dual_arm_planner.launch.py` 增加 `static_virtual_joint_tfs`、`robot_state_publisher`、`ros2_control_node`、`spawn_controllers.launch.py`，对齐 demo 的执行环境。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动确认 `dual_v5_arm_controller`、`torso_controller`、`joint_state_broadcaster` 均 active，`/dual_arm_planner/run_box_stack_flow` service 可见。
- 留给下个 AI：若只想生成轨迹不执行，可 `execute:=false`；正式执行前必须等待 controller spawner 输出 `Configured and activated`。

## 2026-06-09 运控 / Codex / 修复 dual_arm_planner 服务执行链路连续状态
- 做了什么：定位 `/dual_arm_planner/run_box_stack_flow` 看似卡住/执行失败的根因；服务实际能进入回调，主要失败点是 MoveIt 执行起点容差过严、长 service callback 阻塞 `/joint_states` 更新、固定放置姿态 joint6 180° 略超 URDF 上限。
- 改了哪里：`dual_arm_planner.launch.py` 增加 `allowed_start_tolerance:=0.05` 并默认 `prefer_commanded_state:=false`；`dual_arm_planner_node.cpp` 将 `/joint_states` 放入独立 Reentrant callback group，规划起点优先取实时状态，并对目标状态执行 `enforceBounds`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`execute:=true max_rounds:=1 include_top_suction:=false` 调用 `/dual_arm_planner/run_box_stack_flow` 返回 `success=True`。
- 留给下个 AI：Ctrl-C 关闭时 `move_group` 仍可能在析构阶段 segfault，但不影响运行验证；如 CLI 显示 `waiting for service`，先等 `DualArmPlannerNode ready` 和 `Received /dual_arm_planner/run_box_stack_flow request` 日志。

## 2026-06-09 运控 / Codex / 修复箱垛全流程顶吸 IK 并完成 10 轮执行验证
- 做了什么：完整复测 `dual_arm_planner` 10 轮流程；确认第 2 轮 loaded 失败不是稳定复现点，稳定问题在第 9/10 轮顶吸 IK 被姿态误差拒绝。
- 改了哪里：顶吸 IK 姿态误差改为 tool Z 轴方向误差；顶吸默认姿态容差从 5° 放宽到 7°，并在 launch 暴露 `ik_top_orientation_tolerance_deg`。
- 验证结果：纯 IK 箱垛 benchmark 从 8/10 提升到 10/10；`execute:=true max_rounds:=10` 服务返回成功，记录 `data/ik_benchmark/moveit_box_stack_flow/full_top7_axis.jsonl` 和 `.rrd`。
- 留给下个 AI：当前 7° 是基于 v6 模型和现有 tool0/吸盘近似的工程容差；若机械侧确认 TCP 或吸盘姿态变化，需要重新跑顶吸可达性和箱垛全流程。

## 2026-06-09 运控 / Codex / 加速 MoveIt 测试并加入集装箱障碍
- 做了什么：将 `dual_arm_planner` 默认执行速度比例拉满到 `velocity_scale=1.0`、`acceleration_scale=1.0`；新增集装箱左右侧壁和顶板障碍。
- 改了哪里：`dual_arm_planner.launch.py` 暴露集装箱参数；`dual_arm_planner_node.cpp` 向 MoveIt PlanningScene 注入 3 个 collision object；Rerun 可视化脚本同步绘制透明集装箱。
- 验证结果：`alfa_robot_moveit_config` 编译通过；带集装箱 `execute=false max_rounds=1` 服务返回成功；生成 `container_speed_smoke_normalized.rrd` 和 `full_top7_axis_with_container.rrd`。
- 留给下个 AI：当前集装箱只建左右墙与顶板，未加地板/前后端墙，避免车体和地面自碰撞；Rerun 对 `velocity_scale=1.0` 默认做 4x 视觉插值，避免回放跟执行一起变快。

## 2026-06-09 运控 / Codex / MoveIt 箱垛流程加入末端附着箱碰撞
- 做了什么：抓取 IK 到位后，将左右抓到的箱子作为 `AttachedCollisionObject` 挂到对应 `*_v5_tool0`，`loaded/place` 规划期间参与 MoveIt 碰撞，放置后删除；Rerun 同步显示附着箱。
- 改了哪里：`dual_arm_planner_node.cpp`、`dual_arm_planner.launch.py`、`visualize_moveit_box_stack_flow.py`、`CMakeLists.txt`。
- 验证结果：`alfa_robot_moveit_config` 编译通过；带集装箱和附着箱 `execute=false max_rounds=10` 完整通过，生成 `data/ik_benchmark/moveit_box_stack_flow/attached_box_full.jsonl` 和 `.rrd`。
- 留给下个 AI：附着箱尺寸默认 `0.3×0.4×0.4m`；当前只建“已抓起的两个箱子”，不建剩余箱子作为障碍；完整验证为避免 OMPL 随机自碰路径，使用 `planning_time:=20.0 planning_attempts:=80`。
- 2026-06-09 运控：修复 MoveIt 箱垛流程的阶段连续性。`prefer_commanded_state` 现在优先使用上一阶段目标状态，Rerun 补齐每段 start_state；验证 goal->next start 断裂从数 rad 降为 0。开启附着箱后 round2/loaded 暴露真实自碰撞失败，关闭附着箱对照可完整跑完 40 stage。
- 2026-06-10 运控：MoveIt 箱垛 benchmark 改为 5×5 编号体系，机器人对准中间列；抓取顺序更新为 L/R=(2,4),(7,9),(12,14),(17,19),(22,24)。保持集装箱障碍和末端附着箱碰撞，完整 5 轮 / 20 stage 规划成功，Rerun 保存为 `data/ik_benchmark/moveit_box_stack_flow/box_stack_5x5_pairs_2_4_attached.rrd`。
- 2026-06-10 运控：MoveIt 箱垛 benchmark 增加 5×5 中第 1/3/5 列静态箱子障碍，障碍箱体按 0.002m inward inset 缩小；保持集装箱障碍和末端附着箱碰撞。测试到第 5 轮 L22/R24 顶吸 grasp_ik 规划失败后停止，Rerun 保存为 `data/ik_benchmark/moveit_box_stack_flow/box_stack_5x5_pairs_2_4_static_cols_135_attached_partial.rrd`。

## 2026-06-08 机械工程师 / Codex / joint4 固定连接件化并验证三平行轴
- 做了什么：按用户要求将左右臂原 `joint4.STL/link4` 从可动件改为固定连接件，并在其后新增空的 `link4_axis` 作为真正可动 `joint4` 的 child，使 `joint2/joint3/joint4` 在零位下三轴平行；后续 `joint5/joint6/tool0` 链路保持语义不变。
- 改了哪里：`alfa_robot.urdf.xacro` 中新增左右 `left/right_v5_joint4_connector_fixed` 与 `left/right_v5_link4_axis`，保留可控关节名 `left/right_v5_joint4`；`alfa_robot.srdf` 中将 `link4-link5` 相邻禁碰拆成 `link4-link4_axis` 与 `link4_axis-link5`。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本计算零位下左右 `joint2/joint3/joint4` 世界轴向点积均为 1.0，确认三者平行。
- 留给下个 AI：这是 URDF 运动学/可视化验证版；`link4_axis` 是无可视/无碰撞的小惯量空 link，真实 CAD 仍需补一个明确的 joint4 轴承/电机安装结构，否则外观只会显示旧 joint4 固定件和后段直接从轴点接出。

## 2026-06-08 机械工程师 / Codex / joint4 可视化改为 1234556 代理结构
- 做了什么：用户指出三平行改造不能只是空 link，结构可视化也应体现 `1234456/1234556`；因此把新增 `link4_axis` 从空 link 改为带 `joint5.STL` visual/collision 的代理电机，并将原 `joint5` 按原 `joint5->joint6` 方向错开，形成可见的固定 4 + 可动 5 + 原 5 结构。
- 改了哪里：`alfa_robot.urdf.xacro` 的左右 `left/right_v5_link4_axis` 增加 `left/rightjoint5.STL` visual/collision；左右 `left/right_v5_joint5` origin 从 `0 0 0` 改为参考原 5-6 间距的偏移。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本确认左右 `joint2/joint3/joint4` 零位世界轴向仍完全平行。`alfa_robot_description` 已编译成功；`alfa_robot_moveit_config` 因本地 `pick_ik` install 缺失/不完整未编过。
- 留给下个 AI：当前是外观代理方案，不是真实 CAD；如果要编译 MoveIt config，需要先修复/完整构建 `pick_ik`，或清理半截 install 后重建依赖。

## 2026-06-08 机械工程师 / Codex / 修正 1234556 后段 T 型电机正交接续
- 做了什么：用户指出上一版只平移第二个 `5`，没有让后续 T 型电机随新 joint4 坐标系正交接续；已将左右 `joint5` origin 的姿态改为参考原 `joint5->joint6` 的完整 `rpy="1.5708 -0.013602 0"`，使后段关节坐标系跟随前一 T 型输出端旋转。
- 改了哪里：`alfa_robot.urdf.xacro` 中左右 `left/right_v5_joint5` 的 origin rpy 从 `0 0 0` 改为 `1.5708 -0.013602 0`，保留前一版可见 `link4_axis` 代理 STL 和错开位置。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本确认左右 `joint2/joint3/joint4` 世界轴向完全平行，`joint4-joint5` 与 `joint5-joint6` 点积约 `3.67e-06`，即近似严格正交；`alfa_robot_description` 编译通过。
- 留给下个 AI：当前仍是 URDF 代理模型，不是真实 CAD；可用 `ros2 launch alfa_robot_description view_alfa_robot.launch.py` 验收外观和轴向，MoveIt config 编译仍依赖本地 `pick_ik` install 状态。

## 2026-06-08 机械工程师 / Codex / 统一默认初始化为零位
- 做了什么：按用户要求将 description 预览、MoveIt 启动、SRDF home、ros2_control mock 默认值、MuJoCo seed 中的机器人关节初始化统一改为零位。
- 改了哪里：`view_alfa_robot.launch.py` 的 `joint_state_publisher_gui` zeros；`initial_positions.yaml`；`mujoco_initial_positions.yaml`；`alfa_robot.srdf` 的 `home` group_state；`alfa_robot_macro.ros2_control.xacro` 的 `*_initial` 默认参数。
- 验证结果：旧非零启动姿态 `updown=0.45`、`joint2=0.26179939`、`joint3=2.35619449`、`joint5=1.04719755` 已从启动/初始化入口清除；description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；`alfa_robot_description` 编译通过。
- 留给下个 AI：本次只改默认/初始化值，不改 URDF 几何、关节限位或控制速度；`mujoco_initial_positions.yaml` 的移动底盘 `base_x/base_y/base_yaw` 保持原场景摆放值。

## 2026-06-08 机械工程师 / Codex / 修正右臂 55 代理件重合
- 做了什么：用户反馈右臂 `55` 位置两个 T 型件重合；检查发现右臂新增 `joint5` 的 y 偏移沿用了原始 `5->6` 方向，导致两个 `rightjoint5.STL` 拉回同侧。
- 改了哪里：`alfa_robot.urdf.xacro` 中 `right_v5_joint5` origin 从 `xyz="0.00078891 -0.083 -0.057995"` 改为 `xyz="0.00078891 0.083 -0.057995"`，姿态 `rpy="1.5708 -0.013602 0"` 保持不变。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；左右 `joint4->joint5` 距离均为约 `0.101257m`，`joint2/3/4` 仍平行且 `joint4-5/5-6` 仍正交；`alfa_robot_description` 编译通过。
- 留给下个 AI：这是右臂可视代理件位置修正，不改关节名、控制配置、初始化或限位。

## 2026-06-08 机械工程师 / Codex / 修正右臂 5 上下方向并重定义 joint5 零位
- 做了什么：按用户反馈修正右臂新增 `5` 代理件上下装反的问题，并将左右 `joint5` 的逻辑零位重定义为旧姿态的 `-90°`，初始化仍保持 `0`。
- 改了哪里：`alfa_robot.urdf.xacro` 中右臂 `right_v5_link4_axis` 的 `rightjoint5.STL` visual/collision origin 加 `rpy="3.14159265 0 0"`；左右 `left/right_v5_joint5` origin rpy 改为 `-1.57106636 -1.55719433 -3.14132260` 以烘入旧 `-90°` 零偏；左右 joint5 URDF limit 改为 `[-3.14159265, 3.14159265]`；同步 `joint_limits.yaml` 与 `alfa_robot_macro.ros2_control.xacro` 的 joint5 限位。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；展开后左右 joint5 limit 均为 ±π，初始化文件仍为 joint5=0；脚本确认 `joint2/3/4` 仍三平行，`joint4-5/5-6` 仍正交；`alfa_robot_description` 编译通过。
- 留给下个 AI：joint5 的用户/MoveIt 数值语义已改变，`joint5=0` 现在表示旧模型的 `joint5=-90°` 姿态；若运控或硬件侧有 joint5 零点标定，需要同步这一零偏，避免重复偏置。

## 2026-06-08 机械工程师 / Codex / 改正右臂第二个 5 的外观翻转目标
- 做了什么：用户指出右臂问题在第二个 `5`，不是第一个 `5` 代理件；已恢复 `right_v5_link4_axis` 的 `rightjoint5.STL` visual/collision 为 `rpy="0 0 0"`，并将第二个 `right_v5_link5` 的 visual/collision 改为 `rpy="3.14159265 0 0"`。
- 改了哪里：`alfa_robot.urdf.xacro` 中只调整右臂两个 `rightjoint5.STL` 的 visual/collision origin；不改关节 origin、axis、limit、初始化或控制配置。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本确认 `right_v5_link4_axis` visual rpy 为 `0 0 0`、`right_v5_link5` visual rpy 为 `3.14159265 0 0`，左右三平行与后段正交轴系不变；`alfa_robot_description` 编译通过。
- 留给下个 AI：若用户仍认为右侧方向不对，下一步应继续只调整 `right_v5_link5` 的 visual/collision origin，不要再动 `right_v5_link4_axis` 或 joint5 运动学零偏。

## 2026-06-08 机械工程师 / Codex / 右臂 5 外观按左臂模板复制
- 做了什么：用户决定不再单独排查右臂第二个 `5` 的翻转方向，直接按左臂可视策略复制到右臂；已将右臂两个 `rightjoint5.STL` 的 visual/collision origin 都恢复为 `rpy="0 0 0"`，保留右臂关节位置/零偏/限位。
- 改了哪里：`alfa_robot.urdf.xacro` 中 `right_v5_link5` 的 visual/collision origin 从 `rpy="3.14159265 0 0"` 改回 `rpy="0 0 0"`；`right_v5_link4_axis` 也保持 `rpy="0 0 0"`。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本确认右臂两个 `5` visual rpy 均为 `0 0 0`，左右 `joint2/3/4` 三平行与后段正交关系不变；`alfa_robot_description` 编译通过。
- 留给下个 AI：当前采用“左臂正确模板复制到右臂”的外观策略，后续若仍有右臂 CAD 口子问题，应优先由 CAD mesh 原始镜像关系确认，而不是继续在 URDF 里反复加 visual 翻转。

## 2026-06-08 机械工程师 / Codex / 右臂 4-6 严格镜像左臂
- 做了什么：按用户明确要求，不再采用“看起来相似”的右臂修补，而是将左臂 `joint4_connector/link4_axis/joint4/link5/joint5/link6/joint6` 的零位世界位姿关于机器人中线严格镜像到右臂，并反解右臂局部 joint origin。
- 改了哪里：`alfa_robot.urdf.xacro` 中右臂 `right_v5_joint4_connector_fixed`、`right_v5_joint4`、`right_v5_joint5`、`right_v5_joint6` 的 origin；右臂 `right_v5_link4_axis/right_v5_link5/right_v5_link6` 的 visual/collision origin 与左臂一致保持 `rpy="0 0 0"`，mesh 仍使用右臂 STL。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本逐项检查 `link4/joint4/link4_axis/joint5/link5/joint6/link6` 的右臂位姿与左臂镜像位姿，最大位置误差约 `6.1e-09m`、旋转误差约 0；visual/collision origin 也左右一致；`alfa_robot_description` 编译通过。
- 留给下个 AI：这是严格中线镜像版本；如果右臂仍与用户 CAD 预期不一致，优先检查 rightjoint*.STL 本身是否已经预镜像或导出坐标系不一致，而不是再局部翻转 URDF visual。

## 2026-06-08 机械工程师 / Codex / 右臂 joint5 STL 可视/碰撞几何严格镜像补偿
- 做了什么：用户指出右臂倒数 2/3 关节仍重合，说明之前只镜像了 link/joint frame，没有同步验证 STL 几何；重新用 STL 顶点包围盒检查发现 `rightjoint5.STL` 本地几何与左侧镜像存在额外翻转/偏置。
- 改了哪里：`alfa_robot.urdf.xacro` 中右臂两个 `rightjoint5.STL`（`right_v5_link4_axis` 和 `right_v5_link5`）的 visual/collision origin 统一增加 `rpy="0 3.14159265 0"`，并分别加局部平移补偿 `xyz="0 0.01704726 0.00000011"` 与 `xyz="-0.00010367 0.01779720 0"`；visual 与 collision 使用完全相同补偿。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；STL 世界包围盒检查显示右臂 `link4_axis/link5/link6` 相对左臂镜像的 bbox 误差均小于 `2e-5m`，右臂 `4axis-5` 与 `5-6` 的 overlap 尺寸/体积与左臂一致；`alfa_robot_description` 编译通过。
- 留给下个 AI：这里补偿的是 rightjoint5.STL 的 visual/collision 几何坐标，不改关节运动学；若后续替换 CAD，应优先清除此类 STL 局部补偿并使用导出坐标一致的右臂 mesh。

## 2026-06-08 机械工程师 / Codex / 修正右臂 joint5 安装朝向 180° 问题
- 做了什么：用户指出右臂两个 `joint5` 结构位置对了，但安装朝向像是相对左臂镜像多转了 180°；复查 visual frame 后确认此前 `rpy="0 3.14159265 0"` 会让右臂两个 `rightjoint5.STL` 的安装朝向相对左臂镜像差 180°。
- 改了哪里：`alfa_robot.urdf.xacro` 中右臂 `right_v5_link4_axis` 与 `right_v5_link5` 的 visual/collision origin 均改回 `rpy="0 0 0"`，并分别使用局部平移补偿 `xyz="-0.00163222 0.01704702 0.11485305"`、`xyz="-0.00174631 0.01780364 0.11485294"` 保持外包络与左臂镜像对齐；visual 与 collision 同步。
- 验证结果：description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；脚本确认两个右臂 `joint5` visual frame 相对左臂镜像的旋转误差约 `2e-06°`，bbox 误差不超过 `1e-08m`；`alfa_robot_description` 编译通过。
- 留给下个 AI：这里保留安装朝向镜像正确，靠局部 xyz 补偿对齐 rightjoint5.STL 外包络；质心仍有约 5.7cm 差异，来自 STL 内部非对称细节，不应再用 180° 翻转修正。

## 2026-06-08 机械工程师 / Codex / 直接生成右臂 joint5 对称 STL
- 做了什么：按用户要求不再依赖 URDF visual/collision 补偿，而是直接把右臂 `rightjoint5.STL` 改成左臂 `leftjoint5.STL` 的局部镜像版；先备份原导出文件为 `rightjoint5.original_export.STL`。
- 改了哪里：`meshes/alfa_robot_v2_arm_v6/visual/rightjoint5.STL` 与 `collision/rightjoint5.STL` 由对应 `leftjoint5.STL` 通过局部 `z` 取反生成，并反转三角面顶点顺序保持法向；`alfa_robot.urdf.xacro` 中右臂两个 `rightjoint5.STL` 的 visual/collision origin 清回 `xyz="0 0 0" rpy="0 0 0"`。
- 验证结果：脚本确认 visual/collision 的右侧 STL 本地 bbox 等于左侧 STL 的 z 镜像；description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；`alfa_robot_description` 编译通过。
- 留给下个 AI：原始 CAD 导出的右臂 joint5 STL 已保留为 `rightjoint5.original_export.STL`；当前包内实际使用的是由左臂生成的镜像 STL，如后续重新导 CAD，需要注意不要被覆盖。

## 2026-06-08 机械工程师 / Codex / 修正 rightjoint5 STL 镜像轴为局部 Y
- 做了什么：用户反馈右臂看起来仍不是真对称；重新枚举 `leftjoint5.STL` 生成右臂 STL 的局部 X/Y/Z 三种镜像，发现正确镜像轴是局部 `Y`，不是上一版局部 `Z`。
- 改了哪里：重新生成 `meshes/alfa_robot_v2_arm_v6/visual/rightjoint5.STL` 与 `collision/rightjoint5.STL`，由对应 `leftjoint5.STL` 进行局部 `y` 取反并反转三角面顶点顺序保持法向；URDF 中右臂两个 `rightjoint5.STL` 的 visual/collision origin 继续保持 `xyz="0 0 0" rpy="0 0 0"`。
- 验证结果：本地 STL bbox 检查确认 rightjoint5 等于 leftjoint5 的局部 Y 镜像；展开后世界坐标中 `right_v5_link4_axis/right_v5_link5` 的 visual 几何与左臂对应几何关于中线镜像，bbox 与质心误差约 `1e-08`；description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；`alfa_robot_description` 编译通过。
- 留给下个 AI：当前 rightjoint5.STL 是左侧 STL 的局部 Y 镜像生成版，原 CAD 导出右侧文件仍保存在 `rightjoint5.original_export.STL`；不要再用局部 Z 镜像版本。

## 2026-06-08 机械工程师 / Codex / 修复 rightjoint5 镜像 STL 法向导致黑色显示
- 做了什么：用户反馈右臂两个 `joint5` 虽已对称但显示为黑色；检查发现镜像生成的 `rightjoint5.STL` 存储法向与三角面绕序相反，RViz 光照下呈黑色。
- 改了哪里：重新生成 `meshes/alfa_robot_v2_arm_v6/visual/rightjoint5.STL` 与 `collision/rightjoint5.STL`：仍采用局部 `Y` 镜像，但修正三角面绕序和法向一致性；URDF 无需改变。
- 验证结果：脚本检查 visual/collision `rightjoint5.STL` 的法向与面片绕序点积均为正（negative%=0），本地 bbox 仍为 `leftjoint5.STL` 的局部 Y 镜像；description 与 MoveIt wrapper xacro 展开通过，`check_urdf` 通过；`alfa_robot_description` 编译通过。
- 留给下个 AI：如 RViz 仍黑，先重启 RViz/MoveIt 清 mesh 缓存；文件层面 rightjoint5 法向已经修正。
- 2026-06-10 运控：已将 `feature/mechanical-structure-characteristics-research-20260608` rebase 到最新 `v5_dev`（含 MOTION-50）；本地删除旧 `feature/full-flow-ik-grasp-benchmark-20260609-motion-50`。基于机械结构分支复跑 5×5 静态箱列 benchmark，第一段 `round_1_L2_R4/pregrasp` 即失败，MoveIt 报目标采样区无有效状态，碰撞对为 `updown <-> left_v5_link6`。诊断 Rerun：`mechanical_branch_static_5x5_preview.rrd`、`mechanical_branch_pregrasp_5x5_preview.rrd`。
- 2026-06-10 运控：按用户新机械结构调整箱垛 benchmark 固定关键帧：pre 改为左 `0,-90,135,-45,0,0` / 右 `0,-90,135,45,0,0`，loaded 改为双侧 `0,-60,120,-90,0,0`；流程改为 `pregrasp -> grasp_ik -> loaded -> detach -> return_pregrasp`，去掉 place。复跑机械结构分支时 pregrasp 与 grasp_ik 已通过，但 `round_1_L2_R4/loaded` 失败，MoveIt 报 `right_v5_link6 <-> right_v5_link2` 自碰；Rerun：`mechanical_branch_5x5_static_cols_new_keyposes_partial.rrd`、`mechanical_branch_loaded_new_keypose_preview.rrd`。
- 2026-06-10 运控：确认机械结构分支 loaded 右臂 joint4 应镜像为 `+90°`；代码改为 left loaded `0,-60,120,-90,0,0` / right loaded `0,-60,120,90,0,0`。复跑 5×5 静态障碍箱垛 benchmark 后前两轮完整通过，`loaded` 自碰问题消失；当前失败前移到第 3 轮 `round_3_L12_R14/grasp_ik`，自研 IK `512` 次无合法解。Rerun：`mechanical_branch_5x5_static_cols_new_keyposes_right_loaded_j4p90_partial.rrd`。
- 2026-06-10 运控：用户在 Rerun 发现末端附着箱会穿过 1/3/5 静态箱列；确认原因不是障碍未注入，而是 MoveIt/自研 IK 没有显式复核“附着箱 vs 静态箱列”的轨迹几何重叠。已新增附着箱 AABB 与静态箱列 AABB 的路径级硬校验；复跑后在 `round_1_L2_R4/loaded` 被正确拦截，原因 `trajectory point 2: carried_left_box_2 overlaps static_box_obstacle_1`。Rerun：`mechanical_branch_5x5_static_cols_carried_collision_guard_partial.rrd`。

## 2026-06-11 运控 / Codex / 左臂抽箱 primitive demo
- 做了什么：在 `dual_arm_planner_node` 增加 `run_left_extract_demo`，从第一抓 L2/R4 的双臂 IK 到位后，只针对左臂执行后退/上升/仰角上抬候选搜索，检查机器人碰撞、attached box 静态箱/集装箱碰撞，并用邻箱 AABB overlap 判断脱离。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`、`ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py`。
- 验证结果：`left_extract_demo_final.jsonl` 成功；左箱在第 12 步后退 0.36m 时与左右邻箱脱离，已生成 `data/ik_benchmark/left_extract_primitive/left_extract_demo_final.rrd`。
- 留给下个 AI：当前是贪心单臂 demo，不是完整图搜索；抽箱阶段每个候选仍调用 512 次 BioIK，速度很慢，后续应改为解析/KDL 多解或缓存候选。

## 2026-06-11 运控 / Codex / 修复左臂抽箱吸附跳变与下压问题
- 做了什么：定位到吸附跳变并非 attached box 建模本身，而是抽箱阶段重新 IK 导致构型突变；新增 `attach_hold` 记录帧证明吸附瞬间关节保持不变。
- 改了哪里：抽箱 primitive 从双臂 BioIK 改为固定 updown 的左臂 KDL 小步候选；修正 pitch-up 符号；加入末端 z 不下降、工具法向不下压、碰撞和邻箱 overlap 检查。
- 验证结果：`left_extract_demo_kdl_fixed.jsonl/.rrd` 成功；第 12 步后退 0.36m 脱离邻箱，抽箱阶段 `updown` 固定，末端高度不再低于吸附后高度。
- 留给下个 AI：当前仍是贪心候选，不是全局图搜索；第 8/9 步 KDL 无解时允许跳过继续搜索，后续可加层图/插值碰撞检查让路径连续性更强。


## 2026-06-13 运控 / Codex / 抽离后负重姿态规划验证
- 做了什么：在左臂抽箱 benchmark 中增加“抽离成功后继续规划到负重姿态”的验证；负重段固定 updown，只规划双臂 12 轴，并将左侧末端箱真正作为 AttachedCollisionObject 加入 MoveIt，同时继续做末端箱 vs 静态箱墙/集装箱 AABB 逐点审计。
- 改了哪里：`dual_arm_planner_node.cpp` 新增负重段专用 `dual_v5_arm` MoveGroup、规划时间/次数/候选上限参数、负重段 CSV/JSONL 记录；`dual_arm_planner.launch.py` 暴露相关参数。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。基于当前 link3 加长结构跑 top64：四组 L2/R4、L7/R9、L12/R14、L17/R19 共 150 个抽离成功候选尝试负重段规划，0 个通过；主要失败为末端箱撞静态箱墙，其次为撞集装箱顶板/侧壁或 MoveIt 无有效轨迹。Rerun：`data/ik_benchmark/motion51_extract_replay/link3_longer_extract_then_loaded_top64_failure_paths.rrd`。
- 留给下个 AI：当前“抽离后直接到固定负重姿态”在完整箱墙/集装箱碰撞下不可行；下一步应先设计中间过渡姿态/更长安全退出距离/去除或动态更新已抽出箱邻近障碍，再进入负重姿态规划。

## 2026-06-13 运控 / Codex / 更新加长结构负重姿态并复测
- 做了什么：按用户判断，将抽离后负重姿态由旧姿态改为左右双臂 `0,-75,135,0,60,0`，并在相同 top64 条件下重跑“抽离成功后到负重姿态”验证。
- 改了哪里：`dual_arm_planner_node.cpp` 中 `left_loaded_arm_`、`right_loaded_arm_` 固定姿态。
- 验证结果：编译通过；四组 L2/R4、L7/R9、L12/R14、L17/R19 共 136 个抽离成功候选尝试负重规划，16 个通过，整体成功率 11.8%。其中 L7/R9 成功 6/45，L12/R14 成功 10/29，L2/R4 与 L17/R19 仍为 0。Rerun：`data/ik_benchmark/motion51_extract_replay/link3_longer_extract_then_loaded_top64_loaded_0_-75_135_0_60_0.rrd`。
- 留给下个 AI：新负重姿态明显优于旧姿态，但仍不是全局稳定方案；主要剩余失败仍是末端箱撞静态箱墙，其次是集装箱顶板/MoveIt 无有效轨迹。后续应优先搜索“抽离后过渡姿态/负重姿态族”，而不是只用单一固定负重姿态。

## 2026-06-13 运控 / Codex / 负重姿态族先验加入 IK 评分与抽离后规划
- 做了什么：将抽离后的负重姿态从单一固定姿态扩展为左右各 3 组可配置姿态族；IK 评分新增“靠近任意负重姿态”和“额外靠近首选负重姿态”的代价项，抽离成功后会按当前姿态选择最近的负重姿态再规划。
- 改了哪里：`ParallelUpdownAwareIkSolver` 增加 loaded pose family 代价；`dual_arm_planner_node`/launch 增加姿态族参数、权重参数、最近负重姿态选择与 JSON/CSV 记录；benchmark yaml 同步新增姿态族先验。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。top64 direct benchmark 成功跑完四组，Rerun：`data/ik_benchmark/motion51_extract_replay/loaded_pose_family_prior_top64_direct.rrd`；L7/R9 负重规划 15/33 成功，L12/R14 13/28 成功，L2/R4 与 L17/R19 仍主要受箱墙/集装箱碰撞限制。
- 留给下个 AI：默认姿态族已去掉 joint4 ±180 的勉强重复解；后续可直接通过 `loaded_left_pose_family_deg`、`loaded_right_pose_family_deg`、`loaded_preferred_pose_index` 和两个 `ik_loaded_*_weight` 参数调参，无需重编译。

## 2026-06-13 运控 / Codex / MoveIt 静态箱障碍改为动态挖洞箱墙
- 做了什么：按当前抓取对动态生成箱墙障碍：集装箱墙/顶始终存在；静态箱障碍不再是 1/3/5 列整箱，而是当前 L/R 箱位置处挖洞的箱墙，包含左侧墙段、右侧墙段、中间墙段和下方支撑墙段。
- 改了哪里：`dual_arm_planner_node.cpp` 中静态障碍生成/MoveIt PlanningScene 更新/碰撞审计改为 pair-specific；JSONL 每个 stage 记录当前箱墙；`visualize_moveit_box_stack_flow.py` 支持按 stage 动态显示箱墙。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。top64 direct 四组抽离+负重规划跑通：L2/R4 2/16，L7/R9 25/36，L12/R14 9/34，L17/R19 13/16；Rerun：`data/ik_benchmark/motion51_dynamic_box_wall/dynamic_box_wall_top64_direct.rrd`。
- 留给下个 AI：当前箱墙是“按当前抓取对局部放宽”的保守模型；如果后续引入真实抓取顺序，需要根据已移除箱子进一步更新箱墙范围，而不是一次性固定全局箱垛。

## 2026-06-13 运控 / Codex / 抽离后负重规划加入 updown 归零与成功可视化
- 做了什么：抽离成功后的负重规划不再只规划 12 个机械臂关节，改为使用 `dual_v5_arm_with_base`，目标状态同时要求 `updown=0`；Rerun 中系统判定成功的负重规划段末端箱显示为绿色。
- 改了哪里：`dual_arm_planner_node.cpp` 中负重目标状态写入 `updown=0`，负重规划组默认改为 `dual_v5_arm_with_base`；`dual_arm_planner.launch.py` 同步默认参数；`visualize_moveit_box_stack_flow.py` 按 stage 成功状态给 attached box 着色。
- 验证结果：编译通过；top64 direct 四组跑通且所有负重段 `target_updown=0`。结果：L2/R4 3/11、L7/R9 16/31、L12/R14 11/31、L17/R19 15/18，总计负重段 45/87 成功。Rerun：`data/ik_benchmark/motion51_dynamic_box_wall/dynamic_box_wall_updown0_top64_direct.rrd`。
- 留给下个 AI：`updown=0` 是更真实但更强的约束；若后续某层成功率不足，优先考虑抽离后中间过渡姿态，而不是只放宽碰撞。

## 2026-06-13 运控 / Codex / 修正负重段 updown 目标为 0.45m
- 做了什么：用户更正负重段目标高度不是 `updown=0`，而是回到 `0.45m`；已将目标高度做成 `extract_loaded_target_updown` 参数，默认 `0.45`。
- 改了哪里：`dual_arm_planner_node.cpp` 的负重目标状态写入 `extract_loaded_target_updown_`；`dual_arm_planner.launch.py` 暴露同名参数。
- 验证结果：编译通过；L7/R9 smoke 中负重段 `target_updown=0.45`，轨迹末端 updown 到 0.44~0.45m，4/4 有效。数据：`data/ik_benchmark/motion51_dynamic_box_wall/loaded_updown045_L7_R9_smoke.jsonl`。
- 留给下个 AI：后续如果要临时测试其他负重高度，直接 launch 传 `extract_loaded_target_updown:=...`，无需改代码。

## 2026-06-13 运控 / Codex / 抽箱链路默认 updown 改为 0.3m
- 做了什么：按用户最新要求，抽箱 direct grasp 的候选 h 计算以上一次 `updown=0.3m` 为基准；抽离后负重规划目标高度也改为 `updown=0.3m`。
- 改了哪里：`dual_arm_planner_node.cpp` 默认 `extract_grasp_ik_home_updown`、`extract_loaded_target_updown` 改为 0.3；`dual_arm_planner.launch.py` 同步默认值。
- 验证结果：编译通过；L7/R9 smoke 中 first IK h=0.3，负重段 `target_updown=0.3`，4/4 有效。数据：`data/ik_benchmark/motion51_dynamic_box_wall/loaded_updown03_L7_R9_smoke.jsonl`。
- 留给下个 AI：这两个参数仍可通过 launch 覆盖；当前默认值已不是 0.45。

## 2026-06-13 运控 / Codex / 0.3m 抽离到负重计算耗时统计
- 做了什么：基于当前动态挖洞箱墙、抓取/负重 `updown=0.3m`、top64 direct 条件，统计从双臂 IK、左臂抽离 primitive 到抽离后负重 MoveIt 规划的计算耗时。
- 改了哪里：无代码改动；新增统计摘要 `data/ik_benchmark/motion51_dynamic_box_wall/updown03_timing_summary.md`。
- 验证结果：完整 benchmark 跑通，完整成功样本 82 个。全链路平均 103.46ms，中位 100.52ms，P90 141.23ms，最大 206.95ms；其中 IK 平均 6.46ms、抽离搜索平均 50.48ms、负重 MoveIt 规划平均 46.52ms。
- 留给下个 AI：该耗时只代表求解/规划计算时间，不包含真实硬件执行时间；若线上只取第一个成功候选，实际任务延迟可能低于离线 top64 全量 benchmark 的总运行时间。

## 2026-06-13 运控 / Codex / 修正 0.3m 方案阶段总 wall 耗时口径
- 做了什么：用户指出需要的是系统进入阶段到算完全部结果的总耗时，而不是单个成功候选耗时；已重新用 launch log 时间戳和 CSV 统计 top64 候选池完整 wall 时间。
- 改了哪里：新增 `data/ik_benchmark/motion51_dynamic_box_wall/updown03_total_stage_wall_timing.md` 和 `.csv`；原 `updown03_timing_summary.md` 仍代表单候选成功本体耗时，不代表全候选池 wall。
- 验证结果：4 组 top64 总 wall 约 101.08s，平均每组 25.27s；其中 IK 候选池平均每组约 564.93ms，IK 后抽离+负重+记录平均每组约 24.70s。
- 留给下个 AI：若线上策略改成“找到第一个合格候选就停”，实际 wall 会显著小于 top64 全量 benchmark；当前 25s/组是离线全候选审计模式，不适合作为实时执行预期。

## 2026-06-14 运控 / Codex / 负重规划成功率与关节差距量化
- 做了什么：基于已有 `dynamic_box_wall_updown03_top64_direct.jsonl`，量化抽离后关节角度到负重目标姿态的差距，与负重规划成功/失败、成功轨迹实际关节运动量之间的关系。
- 改了哪里：新增统计文件 `data/ik_benchmark/motion51_dynamic_box_wall/loaded_plan_distance_motion_analysis.md`、`.csv` 和 `loaded_plan_distance_motion_summary.csv`。
- 验证结果：126 个负重规划 stage 中成功 82、失败 44。成功样本 12轴 L2 差距均值 2.847rad，失败均值 3.769rad；最大单关节目标差距成功均值 115.2°，失败均值 138.4°。成功样本中目标 L1/L2 差距与实际轨迹总运动量高度相关（corr≈0.96/0.94）。
- 留给下个 AI：关节目标差距对成功率和实际运动量都有明显解释力，但 L12/R14 成功/失败差距相近，说明仍有环境/路径几何因素；后续候选评分可加入“到负重姿态族距离”和“最大单关节差距”硬/软约束。

## 2026-06-14 运控 / Codex / 负重规划按最近负重姿态 Top10 筛选
- 做了什么：基于“抽离后关节角度越接近负重姿态族，MoveIt 负重规划成功率越高且轨迹运动量越小”的结论，新增负重规划前筛选：64 个低代价 IK 候选先完成抽离；对抽离成功候选计算到最近负重姿态族的 12 轴关节角度总差；按总差升序只取前 10 进入 MoveIt 负重规划。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `extract_loaded_sort_by_pose_distance`、负重姿态距离指标、CSV/JSONL 记录；`dual_arm_planner.launch.py` 暴露同名参数。
- 验证结果：编译通过；四组 L2/R4、L7/R9、L12/R14、L17/R19 跑通，Top10 负重规划总成功 32/40。Rerun：`data/ik_benchmark/motion51_loaded_pose_top10/loaded_pose_top10_sorted_success_ordered.rrd`；统计：`data/ik_benchmark/motion51_loaded_pose_top10/loaded_pose_top10_timing_summary.md`。
- 留给下个 AI：当前仍是离线全候选审计模式，整组 wall 含 64 个候选全部抽离；线上可进一步改成“排序后遇到第一个成功负重规划即停”，预计实际延迟会明显低于当前审计口径。

## 2026-06-14 运控 / Codex / 双臂同步抽离与 Top10 负重规划验证
- 做了什么：将左臂抽离 primitive 扩展为双臂同步抽离：同一双臂 IK 候选下，左右臂分别用固定 updown 的 KDL 小步抽离，并组合成同一个 RobotState 做双臂自碰、动态箱墙、集装箱和左右末端附着箱碰撞审计；两臂都脱离邻箱后，再按最近负重姿态族 12 轴角度总差排序，取前 10 做 MoveIt 双臂负重规划。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `extract_benchmark_dual_arm`、右臂/双臂抽离候选、双附着箱负重规划和 CSV/Rerun 记录；`dual_arm_planner.launch.py` 暴露双臂 benchmark 参数。
- 验证结果：编译通过；四组 L2/R4、L7/R9、L12/R14、L17/R19 完整跑通。双臂抽离成功分别为 13/64、12/64、51/64、17/64；Top10 负重规划总成功 40/40。Rerun：`data/ik_benchmark/motion51_dual_extract/dual_extract_top10_sorted_success_ordered.rrd`；统计：`data/ik_benchmark/motion51_dual_extract/dual_extract_top10_timing_summary.md`。
- 留给下个 AI：当前双臂抽离仍是离线全候选审计，L2/R4 与 L7/R9 双臂抽离耗时较大；后续线上化应做 early-stop、候选预筛、或更强的双臂局部路径搜索，避免每组固定跑满 64 个候选。

## 2026-06-14 运控 / Codex / 双臂异步抽离验证
- 做了什么：将双臂抽离从“每一步左右同步组合”新增为“左右臂各自独立抽离，再合成全过程检查双臂/附着箱/环境碰撞”的异步模式。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp` 新增 `extract_benchmark_dual_async` 流程；`dual_arm_planner.launch.py` 暴露启动参数。
- 验证结果：去重 Top64 下，异步抽离成功率为 L2/R4 13/64、L7/R9 25/64、L12/R14 12/37、L17/R19 5/55；相比同步去重版，L7/R9、L12/R14 提升，L2/R4 持平，L17/R19 小幅提升。
- 留给下个 AI：当前失败原因已细化到 KDL 无解、集装箱顶碰撞、robot state colliding/out of bounds；后者还需要进一步拆成自碰撞/限位/场景碰撞。

## 2026-06-14 运控 / Codex / 抽离判定改为侧面投影脱离
- 做了什么：将抽离成功判定从“附着箱与邻箱 3D AABB 完全不重叠”改为“附着箱左右侧面在 x-z 投影上与左右邻箱侧面不再重合”。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp` 的 `carried_box_detached_from_neighbors`。
- 验证结果：异步去重 Top64 下，宽松判定成功率为 L2/R4 15/64、L7/R9 24/64、L12/R14 15/38、L17/R19 5/53；已生成成功/失败 Rerun。
- 留给下个 AI：宽松判定后主要失败仍集中在 KDL 无解、集装箱顶碰撞、robot state colliding/out of bounds；下一步应细分碰撞对并优化抽离动作模板。

## 2026-06-14 运控 / Codex / 抽离早停策略与耗时验证
- 做了什么：抽离阶段改为失败早停（当前步所有候选失败即结束该 IK 候选），成功早停（检测到侧面脱离即认为成功，默认最多额外走 3 步，额外步失败不取消成功）。
- 改了哪里：`dual_arm_planner_node.cpp` 的单臂抽离路径 rollout；`dual_arm_planner.launch.py` 新增 `extract_success_extra_steps`。
- 验证结果：宽松侧面判定 + 异步去重 + 负重 Top10 下，抽离总耗时从约 179.27s 降到约 24.52s，负重规划约 1.93s，IK 约 2.23s。
- 留给下个 AI：成功率保持同量级（L2/R4 14/64、L7/R9 22/64、L12/R14 11/33、L17/R19 7/54），后续重点仍是细分碰撞和优化动作模板。

## 2026-06-14 运控 / Codex / 16线程抽离与负重首成功即停计时
- 做了什么：将双臂抽离 benchmark 增加候选级并行执行，支持 `extract_benchmark_extract_workers`；负重规划增加 `extract_loaded_stop_on_first_success`，按排序顺序首个负重规划成功即停止后续尝试，并记录全链路 wall 耗时。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`、`ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。16线程测试数据在 `data/ik_benchmark/motion51_dual_extract_parallel16_loaded_stop_v3/`；4组总 wall 约 8.62s，完整链路 3/4 成功，L17/R19 失败在抽离阶段。
- 留给下个 AI：并行模式不记录逐步 Rerun，避免并发写记录流；需要可视化时用串行/记录模式复跑特定候选。当前耗时口径为 `IK wall + 去重 + 抽离 wall + 负重规划 wall`。

## 2026-06-14 运控 / Codex / KDL 加锁 A/B 验证
- 做了什么：验证用户怀疑的 `RobotState::setFromIK()` 调 KDL 不适合直接多线程并发；在抽离 KDL 调用外加 `extract_kdl_mutex_`，仅串行化 KDL 本体，候选调度/碰撞后处理仍保持并行外壳。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`。
- 验证结果：16线程未加锁 v3 为 3/4 成功，L17/R19 抽离失败；KDL 加锁 A 方案恢复为 4/4 成功。数据在 `data/ik_benchmark/motion51_dual_extract_parallel16_kdl_locked/`，摘要 `kdl_locked_ab_timing_summary.md`。代价是抽离 wall 约 18.57s，明显慢于未加锁并行。
- 留给下个 AI：当前结论支持“MoveIt 共享 KDL solver 不能直接并发调用”。若要同时保成功率和速度，下一步应做每线程独立 KDL solver/解析 IK，而不是共享 `JointModelGroup::setFromIK()`。

## 2026-06-14 运控 / Codex / 绕开 MoveIt setFromIK 的独立 KDL 并行验证
- 做了什么：在 `dual_arm_planner_node` 增加 `extract_use_independent_kdl` 实验开关，抽离阶段可绕开 `RobotState::setFromIK()`，直接基于 URDF 构建左右臂独立 Orocos KDL chain；每次候选求解用本地 solver 与多 seed 扰动，再写回 `RobotState` 做原有碰撞/误差/抽离规则检查。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`、`ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py`、`CMakeLists.txt`、`package.xml`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。小样本 A/B：16 候选/16 worker 下，旧 MoveIt setFromIK 抽离 wall≈1860ms、成功 7/16；独立 KDL seed4 抽离 wall≈500ms、成功 3/16。说明真并行速度方向成立，但第一版独立 KDL 的求解鲁棒性低于 MoveIt 插件，需要继续调 solver/seed/误差策略。
- 留给下个 AI：如果继续推进，优先对齐 MoveIt KDL 插件的随机重启/搜索策略或改用每线程持有独立 `KDLKinematicsPlugin` 实例；当前直接 Orocos KDL NR_JL 已证明不会被共享插件锁拖慢。

## 2026-06-14 运控 / Codex / 独立 KDL seed8 完整四组流程测试
- 做了什么：继续优化绕开 MoveIt `setFromIK()` 的独立 Orocos KDL 抽离路径，采用 `extract_independent_kdl_seed_attempts=8`、`jitter=15deg`、`max_iterations=120`，跑完四组 `L2/R4、L7/R9、L12/R14、L17/R19` 完整链路：IK 候选池 → 去重 → 16 worker 抽离 → 负重规划首成功即停。
- 改了哪里：同上一条，新增的独立 KDL 路径继续通过 launch 参数控制；结果在 `data/ik_benchmark/motion51_independent_kdl_full_seed8/`。
- 验证结果：完整链路 `4/4` 成功。总任务 wall `9283.5ms`，其中 IK `2272.3ms`，去重 `4.3ms`，抽离 `6008.7ms`，负重规划 `998.2ms`。抽离成功候选 `33/224`，负重规划实际尝试 `4/4` 成功。
- 留给下个 AI：独立 KDL seed8 相比加锁 setFromIK 方案总耗时从约 `21.75s` 降到约 `9.28s`，但抽离成功候选数下降；后续若追求更高候选成功率，可尝试每线程独立 `KDLKinematicsPlugin` 或继续调 seed/jitter/姿态约束。

## 2026-06-14 运控 / Codex / 侧吸高度窗 0.9~1.3 与新四组抽离测试
- 做了什么：按用户要求将侧吸 IK 的 updown 可达高度窗改为 `updown+0.9 ~ updown+1.3`，并把抽离 benchmark 的默认四组改为可配置序列，当前默认 `2,3;7,4;8,9;12,13`。
- 改了哪里：`dual_arm_planner_node.cpp`、`dual_arm_planner.launch.py`；同时修复相邻开口箱墙会生成 4mm `between` 幽灵障碍的问题，相邻两箱开洞时不再生成中间墙。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。新四组离线测试结果在 `data/ik_benchmark/motion51_front_window_09_13_pairs_2_3_7_4_8_9_12_13_clean/`；`L7/R4`、`L12/R13` 有完整成功样本，`L2/R3`、`L8/R9` 仍失败在抽离阶段，主因是 `left_kdl_no_solution`。
- 留给下个 AI：底层/相邻箱 pair 的抽离失败不是负重规划问题；下一步应针对左臂抽离 KDL/动作模板继续优化，或对低层改走顶吸策略。

## 2026-06-14 运控 / Codex / 原四组抓取任务固定版复跑
- 做了什么：将抽离 benchmark 默认抓取序列固定回原任务 `2,4;7,9;12,14;17,19`，其余侧吸高度窗、动态箱墙、独立 KDL 抽离、负重姿态筛选等保持当前方案。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py`、`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。复跑结果保存在 `data/ik_benchmark/motion51_original_pairs_current/`，Rerun 为 `original_pairs_current.rrd`。当前结果：L2/R4 抽离 20/64、负重 1/64；L7/R9 抽离 16/64、负重 1/64；L12/R14 抽离 5/58、负重 1/58；L17/R19 因当前侧吸高度窗 `updown+0.9~1.3` 判定 `h_interval_unreachable`，符合“不再侧吸最低排”的现阶段设定。
- 留给下个 AI：如果要给其它部门 AI 解释完整流程，优先给 `dual_arm_planner_node.cpp`、`dual_arm_planner.launch.py`、`visualize_moveit_box_stack_flow.py`、当前 URDF/SRDF/MoveIt config，以及一份 JSONL/RRD 结果；不要只给 Rerun，Rerun 缺少算法入口和参数语义。

## 2026-06-15 运控 / Codex / 运控流程封装第一阶段
- 做了什么：开始把 `dual_arm_planner_node.cpp` 中的全流程算法按责任拆分；第一阶段只抽出不依赖 MoveIt RobotState 的纯数据、姿态数学和场景几何工具，避免行为变化。
- 改了哪里：新增 `motion_core/task_geometry`、`motion_core/pose_math`、`motion_core/scene_geometry`，并导出 `alfa_robot_motion_core` 库；`dual_arm_planner_node.cpp` 改为引用这些模块；新增跨仓库对接文档 `docs/运控/MOTION_PIPELINE_REFACTOR.md`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；轻量 launch 烟测已到 `DualArmPlannerNode ready`，能正常加载集装箱和动态箱墙几何。
- 留给下个 AI：后续按文档顺序继续拆 `SceneAdapter`、`LoadedPosePlanner`、`ExtractPlanner`、`IKSelector`；每一步都要保持 JSONL/CSV/Rerun 记录不丢。

## 2026-06-15 运控 / Codex / MoveIt 场景适配层拆分
- 做了什么：继续把 `dual_arm_planner_node.cpp` 的 MoveIt PlanningScene 副作用拆出，新增 `MotionSceneAdapter` 管理集装箱障碍、动态箱墙、末端附着箱的 ADD/REMOVE 与当前场景状态。
- 改了哪里：新增 `include/alfa_robot_moveit_config/motion_scene_adapter.hpp`、`src/motion_scene_adapter.cpp`，并导出 `alfa_robot_motion_scene_adapter`；`dual_arm_planner_node.cpp` 改为通过 adapter 设置箱墙和附着箱，记录时仍能读取当前场景状态。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。
- 留给下个 AI：后续可以继续拆 `LoadedPosePlanner`、`ExtractPlanner`、`IKSelector`；注意 `MotionSceneAdapter` 只负责 MoveIt 场景适配，不负责 RobotState 碰撞判断或路径搜索。

## 2026-06-15 - 运控 planner 重构：负重姿态选择模块拆分

- 新增 `LoadedPoseSelector`，负责从抽离后的关节状态选择最近的负重姿态族，并生成负重目标 joint state。
- `DualArmPlannerNode` 不再内联维护负重姿态距离计算、最近姿态选择和目标状态生成，MoveIt 负重规划调用仍留在节点内。
- 更新 `MOTION_PIPELINE_REFACTOR.md`，补充当前模块划分和后续拆分顺序。
- 验证：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动烟测达到 `DualArmPlannerNode ready`。

## 2026-06-15 - 运控 planner 重构：负重规划模块拆分

- 新增 `LoadedPosePlanner`，负责抽离后到负重姿态的 MoveIt 规划、临时附着箱状态和规划记录回调。
- `DualArmPlannerNode` 的 `plan_loaded_from_extract_state()` 缩减为调用模块并回填 timing，原有 JSONL 字段保持。
- 更新 `MOTION_PIPELINE_REFACTOR.md`，标记 `LoadedPosePlanner` 已完成，下一步转向 `ExtractPlanner`。
- 验证：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动烟测达到 `DualArmPlannerNode ready`。

## 2026-06-15 - 运控 planner 重构：抽离动作模板拆分

- 新增 `ExtractMotionPlanner`，负责抽离动作模板、pitch 调整层、retreat/lift 候选目标盒心生成。
- `DualArmPlannerNode` 的抽离候选生成仍负责 KDL 求解和碰撞判定，但动作组合硬编码已迁出。
- 更新 `MOTION_PIPELINE_REFACTOR.md`，下一步 `ExtractPlanner` 继续拆 KDL 候选求解、早停规则和失败原因统计。
- 验证：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动烟测达到 `DualArmPlannerNode ready`。

## 2026-06-15 - 运控 planner 重构：抽离候选求解/评分/记录模块拆分

- 做了什么：继续将 `dual_arm_planner_node.cpp` 中的抽离相关职责下沉为可复用模块，新增 IK 候选选择器、抽离共享类型、抽离候选评分器、抽离候选 KDL 求解器和 JSONL 记录器。
- 改了哪里：新增 `IkCandidateSelector`、`ExtractCandidateScorer`、`ExtractCandidateSolver`、`MotionFlowRecorder`、`extract_planner_types`；`DualArmPlannerNode` 现在主要负责 ROS 参数、流程编排、场景碰撞判定和 rollout 策略。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动烟测达到 `DualArmPlannerNode ready`，集装箱与动态箱墙正常应用，JSONL recorder 正常打开。
- 留给下个 AI：下一步若继续拆，应聚焦 `ExtractRolloutPlanner`（早停规则、失败原因统计、双臂同步/异步 rollout）和 `IKSolverService`，不要再把新算法塞回节点文件。

## 2026-06-15 - 运控 planner 重构：抽离输出与负重批处理继续拆分

- 做了什么：继续收敛抽离相关模块，新增 CSV 输出、timing 汇总和负重规划批处理接口，并删除未使用的旧 BioIK 抽离候选死路径。
- 改了哪里：新增 `ExtractBenchmarkCsvWriter`、`ExtractBenchmarkSummary`；`LoadedPosePlanner` 增加 `planBatch()`，负责成功抽离候选的排序、limit 和首成功即停；`DualArmPlannerNode` 不再手写这些重复调度。
- 验证结果：干净 ROS 环境下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动烟测达到 `DualArmPlannerNode ready`，JSONL recorder 正常打开。
- 留给下个 AI：剩余最大块是 `ExtractRolloutPlanner`，但它和场景碰撞判定、双臂组合、Rerun step 记录耦合很深；继续拆时应先设计 callback seam，避免丢失失败原因和记录字段。

## 2026-06-15 - 运控 planner 重构：抽离 rollout 状态机拆出

- 做了什么：新增 `ExtractRolloutPlanner`，把单臂/双臂抽离 rollout、早停、双臂异步组合、逐步记录字段生成从 `DualArmPlannerNode` 拆出。
- 改了哪里：新增 `include/alfa_robot_moveit_config/extract_rollout_planner.hpp`、`src/extract_rollout_planner.cpp`；节点通过 callback seam 复用原有场景碰撞判定，避免改变碰撞语义。
- 验证结果：干净 ROS 环境下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过。
- 留给下个 AI：当前大节点已降到约 2500 行，后续主要剩 `IKSolverService` 和 `FlowOrchestrator`；不要重新把 rollout 逻辑塞回节点。

## 2026-06-15 - 运控 planner 重构：自研双臂 IK 适配层拆分

- 做了什么：新增 `OptimizedDualIkSolver`，把 fixed h × multi seed × cost scorer 的请求构造、selected joint 写回和 IK 审计 JSON 从 `DualArmPlannerNode` 拆出。
- 改了哪里：新增 `include/alfa_robot_moveit_config/optimized_dual_ik_solver.hpp`、`src/optimized_dual_ik_solver.cpp`；主节点只保留碰撞/边界验收和 MoveIt 规划调用。
- 验证结果：干净 ROS 环境下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动烟测达到 `DualArmPlannerNode ready`。
- 留给下个 AI：IK 算法本体仍来自 benchmark solver；现在已经有清晰 seam，后续若要改成 ROS IK 服务，可以优先替换 `OptimizedDualIkSolver::solve()` 的后端。

## 2026-06-15 - 运控 planner 重构：流程编排与 benchmark runner 拆分

- 做了什么：新增 `BoxStackFlowOrchestrator`、`ExtractDemoOrchestrator` 和 `ExtractBenchmarkRunner`，把传统箱垛流程、多 pair 抽离 demo 外层循环、IK 候选筛选/去重/抽离并行/负重批处理/summary 写入从主节点拆出。
- 改了哪里：新增 `box_stack_flow_orchestrator.*`、`extract_demo_orchestrator.*`、`extract_benchmark_runner.*`；`DualArmPlannerNode` 通过 callback 提供 MoveIt/IK/场景/记录能力。
- 验证结果：干净 ROS 环境下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；短启动烟测达到 `DualArmPlannerNode ready`，JSONL header 正常写入。
- 留给下个 AI：主节点已基本降为 ROS 参数、MoveIt 后端、场景碰撞判定和 callback 装配层；后续不要再把流程循环和统计字段写回节点。

## 2026-06-15 - 运控 planner 重构：复用边界 review 与复现实验

- 做了什么：review 拆分后的 CMake/安装边界，修复 public header 暴露 benchmark IK 类型但实现只编进节点的问题；`OptimizedDualIkSolver` 和 benchmark solver 实现现在随 `alfa_robot_motion_scene_adapter` 一起编译，benchmark IK headers 也随包安装。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/CMakeLists.txt`；复现实验数据保存在 `data/ik_benchmark/refactor_replay_review/`。
- 验证结果：干净 ROS 环境下 `alfa_robot_moveit_config` 编译通过；launch 烟测达到 `DualArmPlannerNode ready`；小规模 L2/R4 复现成功并生成 JSONL/CSV；原四组复现前三组进入抽离+负重并成功，L17/R19 仍按当前侧吸高度窗预期失败在 `h_interval_unreachable`，已生成 `refactor_replay_original_pairs.rrd`。
- 留给下个 AI：如果其它包要复用 IK/抽离/负重规划模块，优先链接 `alfa_robot_motion_scene_adapter`；复现实验请使用独立 `ROS_DOMAIN_ID` 或先清理旧 launch，避免 service 请求打到残留节点。
