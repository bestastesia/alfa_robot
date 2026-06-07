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

## 2026-06-04 运控工程师 / Codex / 补齐 Linear milestone 归属
- 做了什么：检查李昊洋负责的 V1 Project issues，补齐明显漏挂 milestone 的 MOTION-37 全流程 demo 及其子任务。
- 改了哪里：MOTION-37、MOTION-43 归入 `M6 连续 20 箱验收`；MOTION-38、MOTION-39、MOTION-40 归入 `M2 目标选择与可达性`；MOTION-41、MOTION-42 归入 `M6 连续 20 箱验收`。
- 验证结果：重新拉取 V1 Project issues 后，上述关键 open/demo 任务均已有 milestone。
- 留给下个 AI：Backlog/已取消/非 V1 项目任务未强行归属；后续新建 MOTION-37 子任务时记得同步 milestone。

## 2026-06-04 运控工程师 / Codex / 修正 MOTION-37 验收 issue 拆分
- 做了什么：用户确认明天固定平台双臂验收本质就是 MOTION-37，不应另开临时重复 issue；已取消 MOTION-43，并把验收口径并回 MOTION-37。
- 改了哪里：MOTION-37 改为固定平台双臂验收 Demo；MOTION-39/40/41/42 改为固定平台场景、IK、MoveIt、执行/可视化子任务；新增 MOTION-44 作为独立急停/安全层子任务。
- 验证结果：MOTION-37 及子 issue 均归入 `M6 连续 20 箱验收`，MOTION-43 已 Canceled。
- 留给下个 AI：后续不要继续使用 MOTION-43；明天验收推进全部写 MOTION-37 及其子 issue。

## 2026-06-04 仿真工程师 / Codex / 修复 MuJoCo 数字孪生重复障碍与残留容器
- 做了什么：定位用户同时运行旧静态桥 `mujoco_planning_scene_bridge.py` 和新数字孪生 launch 时，MoveIt 会同时收到 `mj_*` 静态障碍和 `semantic_*` 语义障碍，导致两排箱子；当前固定平台 scene 已不生成 container/table，但语义转换此前会用默认零位继续生成地下容器。
- 改了哪里：`semantic_scene_utils.py` 在 container/table pose 为零时跳过对应对象；`semantic_scene_to_planning_scene.py` 启动后清理历史 `mj_*`、container、platform 残留对象；`mujoco_planning_scene_bridge.py` 默认禁用，需 `--allow-legacy` 才能作为旧静态调试桥运行。
- 验证结果：`alfa_robot_moveit_config` 编译通过；短启动 `mujoco_digital_twin.launch.py` 后 `/collision_object` 只有 `semantic_scene_to_planning_scene` 发布，语义样本中 `container_front_pose` 为零且不再生成 `semantic_container_*`，日志显示清理 573 个历史对象。
- 留给下个 AI：数字孪生模式只运行 `ros2 launch alfa_robot_moveit_config mujoco_digital_twin.launch.py`；不要再并行运行 `mujoco_planning_scene_bridge.py`，否则旧版本环境仍可能重复写入 PlanningScene。

## 2026-06-04 运控工程师 / Codex / 完成固定平台双臂 IK 服务骨架
- 做了什么：按 MOTION-40 实现固定平台验收用 IK 服务层，提供 `/alfa_dual_ik/solve`，输入左右目标 Pose 与当前关节状态，输出选中的双臂 joint target、耗时、候选统计和诊断 JSON。
- 改了哪里：新增 `SolveDualIk.srv`、`fixed_platform_dual_ik_service.cpp`、`fixed_platform_dual_ik_service.launch.py`；更新 `CMakeLists.txt` 与 `package.xml` 以生成 ROS2 service 接口并安装 launch。
- 验证结果：`alfa_robot_benchmarks` 用系统 Python 重新编译通过；launch 可启动服务并在 `/alfa_dual_ik/solve` 可见；低预算 smoke call 能返回结构化结果和诊断。
- 留给下个 AI：当前服务假定目标已在 `base_link` 坐标系；MoveIt/RViz 层若发 world 坐标，需要在调用方或后续服务版本加入 TF 转换。固定验收默认 `fixed_updown=0.18`，PLC 仍走 mock。

## 2026-06-04 运控工程师 / Codex / 搭建固定平台验收 ros2_tmp 最小流程
- 做了什么：按用户要求新建 `ros2_tmp` 临时 ROS2 workspace，只保留固定平台双臂验收需要的 description、MoveIt config、bio_ik、IK/流程包；未迁移雷达、导航、感知、twist mux、硬件包和历史数据。
- 改了哪里：`ros2_tmp/src/ik_benchmark` 新增任务发布层 `mock_box_task_publisher`、任务编排层 `fixed_platform_task_orchestrator`、任务/状态消息和 MoveIt/执行层预留 service；IK 服务输出改为 12 个双臂 joint target，full joint 解写入诊断 JSON。
- 验证结果：干净环境下 `ros2_tmp` 四包编译通过；launch 可启动 `/alfa_dual_ik/solve`、`/alfa_task/command`、`/alfa_task/status`；手动发布 5/6 任务后编排层完成 accepted → IK → mock planner → mock executor → done，并触发发布层准备 7/8。
- 留给下个 AI：运行说明见 `ros2_tmp/README.md`；MoveIt 层接入时把 `mock_planner:=false` 并实现 `/alfa_moveit/plan_joint_target`，执行层接入时把 `mock_executor:=false` 并实现 `/alfa_execution/execute_joint_trajectory`。

## 2026-06-04 运控工程师 / Codex / 修正执行层接口为逐点 joint target
- 做了什么：按用户确认的分工，执行层只接 joint target，不接完整 trajectory；编排层负责把 MoveIt 返回的 trajectory points 拆成逐点 joint target 发给执行层。
- 改了哪里：`ros2_tmp/src/ik_benchmark` 新增 `ExecuteJointTarget.srv`；`fixed_platform_task_orchestrator` 改为调用 `/alfa_execution/execute_joint_target`；launch/README 同步新接口。
- 验证结果：`alfa_robot_benchmarks` 重新编译通过；`ExecuteJointTarget` 接口可见；服务启动正常。IK 服务新增 `lock_updown:=true`，smoke 诊断显示 `h_candidates=[0.18]`，保证固定平台只输出 12 轴目标时不会暗中选其他 updown。
- 留给下个 AI：固定 `updown=0.18` 后，部分箱子点可能 IK 失败；这是固定平台可达性问题，不是任务链路问题。若临时验收点位要可达，需要调整箱子高度/目标点或固定平台安装高度。

## 2026-06-04 运控工程师 / Codex / 接入电控执行层与临时 MoveIt 障碍规划
- 做了什么：将电控同事的 `alfa_robot_plc_bridge` 筛选迁入 `ros2_tmp`，保留执行层 `plc_bridge_node` 和急停层 `plc_safety_node`；编排层改为发布 MoveIt 返回的 `JointTrajectory` 到 `/plc_joint_trajectory`，并等待 `/plc_bridge_state` 回 normal 后再通知任务完成。
- 改了哪里：`ros2_tmp/src/alfa_robot_plc_bridge` 新增执行/急停包；`fixed_platform_task_orchestrator` 对齐 `/plc_joint_trajectory`；新增 `temporary_moveit_joint_planner` 提供 `/alfa_moveit/plan_joint_target`，启动时从 MuJoCo `scene.xml` 解析 20 个 cargo box 写入 MoveIt PlanningScene；新增 `planning_scene_visualizer.py` 发布 `/alfa_visualization/planning_scene_markers` 并可选 Rerun。
- 验证结果：`ros2_tmp` 五包编译通过；总 launch 可启动 `/alfa_dual_ik/solve`、`/alfa_moveit/plan_joint_target`、`/plc_joint_trajectory`、`/plc_bridge_state`、`/alfa_safety/*` 和障碍 marker；临时 MoveIt planner 日志显示已应用 20 个 MuJoCo cargo 障碍。
- 留给下个 AI：当前临时 planner 默认开启 `temporary_planner:=true`，同事正式 MoveIt 服务接管后可设为 false；执行层默认 `plc_mock:=true`，接真实 PLC 时设为 false。障碍 x 偏移当前为 `scene_x_shift=-3.925`，用于把 MuJoCo 远处 cargo 映射到固定平台箱垛附近，联调时可按实际箱子位置微调。

## 2026-06-04 运控工程师 / Codex / ros2_tmp 最新接口口径修正
- 做了什么：前两条 ros2_tmp 日志里提到的 `mock_executor`、`/alfa_execution/execute_joint_target`、`ExecuteJointTarget.srv` 已被电控真实包替代，避免误用。
- 当前口径：编排层发布 `trajectory_msgs/JointTrajectory` 到 `/plc_joint_trajectory`；电控 `plc_bridge_node` 负责轨迹执行和 PLC/mock PLC；急停由 `plc_safety_node` 对外提供 `/alfa_safety/*`。
- 留给下个 AI：以本条和上一条“接入电控执行层与临时 MoveIt 障碍规划”为准。

## 2026-06-04 临时固定平台验收：箱子 x 与初始臂姿态更新

- ros2_tmp 任务目标箱子前表面默认改为 `x=0.76m`，任务发布层参数 `x_offset` 现在直接表示箱子前表面 x，不再使用 `0.375 + x_offset`。
- 临时 MoveIt 障碍箱同步到 `x=0.76m`：MuJoCo cargo 解析使用 `scene_x_shift=-3.84`，fallback box stack 也使用 `front_face_x=0.76`。
- 双臂初始/home 姿态统一为 `0°, 5°, 145°, 0°, 120°, 0°`：MoveIt initial_positions、SRDF home、description view zeros、ros2_control initial、任务编排 home seed 均已同步。
- mock PLC 执行层新增 `mock_initial_positions_deg`，默认发布同一套 12 轴初始角度，避免 `/joint_states` 仍从 0 位开始影响 MoveIt 起点。
- 已构建验证：`alfa_robot_description`、`alfa_robot_moveit_config`、`alfa_robot_plc_bridge`、`alfa_robot_benchmarks` 通过。

## 2026-06-04 固定 h IK 线程语义修正

- 原因：固定平台 `updown=0.18` 时，期望 IK 不是少量候选，而是同一个固定 h 下充分并行试 seed。
- 修改：fixed_discrete 分支改为 `workers × seed_count` 次 trial；默认 `workers=16 seed_count=32` 即同一 h 下 512 次不同 seed，并由 16 线程并行消费。
- 影响：`h_candidate_count` 在单一固定 h 场景下不再决定总次数；多个 h 候选时会在这些 h 间轮询，但总次数仍以 `workers × seed_count` 为准。
- 已构建验证：`colcon build --packages-select alfa_robot_benchmarks` 通过。

## 2026-06-05 运控工程师 / Codex / 固定平台临时任务切到 11/10 并增加 IK 重试
- 做了什么：临时任务发布层改为只发布 `box_pair_11_10`，左手抓 11、右手抓 10；目标点为 world 下 L11 `(0.76, 0.2, 1.0)`、R10 `(0.76, -0.2, 1.0)`。
- 改了哪里：`ros2_tmp/src/ik_benchmark/src/mock_box_task_publisher.cpp`；`fixed_platform_task_orchestrator` 新增 `ik_max_attempts`，默认 5 次，避免 BioIK 单次随机失败直接终止任务；IK launch 默认侧吸 `position_tolerance` 从 2cm 放宽到 3cm。
- 验证结果：`colcon build --packages-select alfa_robot_benchmarks` 通过；当前运行环境下 11/10 第一次 IK 失败、第二次 IK 成功，随后 MoveIt planner 和 PLC mock 执行链路完成，发布层收到 `task completed`。
- 留给下个 AI：若重启系统，需要重启编排层以加载新二进制；测试命令可用 `ros2 launch alfa_robot_benchmarks fixed_platform_task_orchestrator.launch.py fixed_updown:=0.18 mock_planner:=false wait_execution_done:=true ik_max_attempts:=5`，再运行 `ros2 run alfa_robot_benchmarks mock_box_task_publisher`。

## 2026-06-05 运控工程师 / Codex / 临时验收 IK 后端切为单臂 KDL
- 做了什么：确认 BioIK 双臂 fixed h 在高位 front 目标上会失败，但单臂 KDL 对同一点可稳定求解；临时验收 IK 服务改为左右臂分别通过 MoveIt `/compute_ik` 的 KDL 求解，再合并 12 轴目标。
- 改了哪里：`ros2_tmp/src/ik_benchmark/scripts/fixed_platform_kdl_ik_service.py`；`fixed_platform_dual_ik_service.launch.py` 和 `fixed_platform_acceptance_flow.launch.py` 默认启动该 KDL 服务；`mock_box_task_publisher` 当前测试目标改为 `box_pair_7_6`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；直接调用 7/6 `z=1.4` 成功，KDL IK wall 约 4.9ms；完整任务发布层 7/6 → 编排层 → KDL IK → MoveIt planner → PLC mock 链路完成。
- 留给下个 AI：当前 KDL 服务 `check_collision:=false` 运行；如果要启用碰撞，需要解决单臂 KDL 和全身碰撞的判定策略。服务接口仍是 `/alfa_dual_ik/solve`，编排层无需改。

## 2026-06-05 运控工程师 / Codex / 固定平台 KDL IK 临时参数口径修正
- 做了什么：复现用户 7/6 任务失败后确认根因不是点位不可达，而是临时 KDL 服务继承了 launch 的 `check_collision:=true` 和 `timeout:=0.01`，导致 `/compute_ik` 在 IK 阶段做碰撞拒绝/时间过短。
- 改了哪里：`ros2_tmp/src/ik_benchmark/scripts/fixed_platform_kdl_ik_service.py`；KDL `/compute_ik` 固定 `avoid_collisions=false`，碰撞交给 MoveIt 规划层；内部 timeout 最小钳到 0.05s。
- 验证结果：用 `fixed_updown:=0.18 timeout:=0.01 check_collision:=true` 启动 KDL 服务时日志显示实际 `timeout=0.050s compute_ik_avoid_collisions=False`；发布 `box_pair_7_6` 后任务层收到 accepted 并返回 done。
- 留给下个 AI：临时验收链路不要在 IK 阶段启用碰撞拒绝；如需碰撞，应在 MoveIt 规划层或合并双臂 joint target 后做全身校验。

## 2026-06-05 运控工程师 / Codex / 固定平台临时验收一键栈收口
- 做了什么：`ros2_tmp` 临时验收链路改为一键启动除任务发布外的全部线程；任务发布层单独按回车发布 `box_pair_7_6`。
- 改了哪里：PLC bridge 默认不发布 `/joint_states`；demo 初始 `updown=0.18`；临时 planner 静态注入 2列×4行箱子，箱子前侧 `x=0.76`、中心 `x=0.91`、末端目标 `x=0.70`；KDL IK 合并左右臂后调用 `/check_state_validity` 做全身碰撞校验；编排层等待 PLC/mock 真正执行开始并结束后才回 done。
- 验证结果：`alfa_robot_benchmarks` 编译通过；本机完整跑通 `mock_box_task_publisher` → KDL IK → MoveIt 规划 → PLC mock 执行；`/joint_states` 确认为仅 `joint_state_broadcaster` 发布；任务约等待 20.4s mock 执行完成后返回 done。
- 留给下个 AI：运行说明见 `ros2_tmp/docs/fixed_platform_acceptance_runbook.md`；推荐用 `ros2 launch alfa_robot_benchmarks fixed_platform_acceptance_stack.launch.py plc_mock:=true` 启动栈，再单独运行任务发布层。

## 2026-06-05 运控工程师 / Codex / PLC 发送模式与验收参数化
- 做了什么：对比 PLC CLI `move-abs` 和 ROS bridge，确认底层寄存器协议一致，主要差异是 CLI 一次性发最终绝对点，ROS 默认按 MoveIt 轨迹流式覆盖目标。
- 改了哪里：PLC bridge 新增 `plc_execution_mode=stream|final_abs`；任务发布层改为直接给左右末端坐标，不再依赖箱号；一键栈暴露 `velocity_limit_deg_s`、`moveit_velocity_scale`、`moveit_acceleration_scale`、`planning_time`、`planning_attempts` 等关键参数。
- 验证结果：`alfa_robot_plc_bridge`、`alfa_robot_benchmarks` 编译通过；mock 下 `stream` 模式完整任务成功并等待约 19–20s；`final_abs` 模式成功时 PLC/mock 命令数为 1，性质更接近 CLI `move-abs`。
- 留给下个 AI：真实 PLC 若出现卡顿，先用 `plc_execution_mode:=final_abs velocity_limit_deg_s:=3.0` 区分是 PLC/机械执行问题还是 stream 轨迹覆盖问题；运行说明见 `ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。

## 2026-06-05 运控 / Codex / 固定平台验收 PLC 执行断点
- 做了什么：针对“CLI move-abs 能动但 acceptance stack 不动”补充了 PLC 执行链路断点日志；编排层发 `/plc_joint_trajectory` 前会打印 topic、点数、joint 数、订阅者数，PLC bridge 收到 topic/action 轨迹后会打印执行模式、点数、时长和前 6 轴 first->last 预览，开始写 PLC 前打印 `starting PLC write loop`，结束打印命令数和 final targets。
- 改了哪里：`ros2_tmp/src/ik_benchmark/src/fixed_platform_task_orchestrator.cpp`、`ros2_tmp/src/alfa_robot_plc_bridge/alfa_robot_plc_bridge/plc_bridge_node.py`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：`colcon build --packages-select alfa_robot_plc_bridge alfa_robot_benchmarks --symlink-install` 通过。
- 留给下个 AI：如果实机仍不动，先看启动终端是否依次出现 `Publishing PLC trajectory ... subscribers>0`、`accepted trajectory for PLC from topic`、`starting PLC write loop`、`trajectory finished`；用这些日志判断问题在编排/Topic/PLC 写入/PLC 侧执行哪一段。

## 2026-06-05 运控 / Codex / 固定平台验收全零初始姿态
- 做了什么：按用户要求把 fixed-platform acceptance 的初始化姿态改为 `updown=0.18m`，双臂 12 个关节全 0°；IK 服务 home seed、编排层 fallback home、MoveIt 初始位置、SRDF home、PLC mock 初始值同步为全 0。
- 改了哪里：`ros2_tmp/src/alfa_robot_moveit_config/config/initial_positions.yaml`、`ros2_tmp/src/alfa_robot_moveit_config/config/mujoco_initial_positions.yaml`、`ros2_tmp/src/alfa_robot_moveit_config/config/alfa_robot.srdf`、`ros2_tmp/src/ik_benchmark/src/fixed_platform_dual_ik_service.cpp`、`ros2_tmp/src/ik_benchmark/src/fixed_platform_task_orchestrator.cpp`、`ros2_tmp/src/alfa_robot_plc_bridge/config/plc_bridge.yaml`、`ros2_tmp/src/alfa_robot_plc_bridge/alfa_robot_plc_bridge/plc_bridge_node.py`。
- 验证结果：相关包编译通过。
- 留给下个 AI：实机 PLC 不发布 `/joint_states`，当前 MoveIt/RViz 状态仍来自 demo/ros2_control；启动后检查 `/joint_states` 初始应为 updown 0.18、双臂全 0。

## 2026-06-05 运控 / Codex / PLC 验收监听与 home 保持
- 做了什么：新增 `plc_acceptance_supervisor.py`，默认随 `fixed_platform_acceptance_stack.launch.py` 启动，集中监听 `/plc_bridge_state` 和 `/alfa_task/status`，每秒输出 PLC 状态摘要；可选 `enable_home_hold:=true` 时在 PLC 空闲状态下周期发送全零 home 轨迹用于启动后自动校准。
- 改了哪里：`ros2_tmp/src/ik_benchmark/scripts/plc_acceptance_supervisor.py`、`ros2_tmp/src/ik_benchmark/launch/fixed_platform_acceptance_stack.launch.py`、`ros2_tmp/src/ik_benchmark/CMakeLists.txt`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；单独运行 supervisor 可启动并正常等待 `/plc_bridge_state`。
- 留给下个 AI：home hold 默认关闭；若开启，收到任务 `accepted/running` 后默认自动停止，避免和任务轨迹抢 `/plc_joint_trajectory`。实机联调建议先用 `home_hold_max_commands:=3` 限制校准次数。

## 2026-06-05 运控 / Codex / 固定平台中线防撞隔板
- 做了什么：在临时 MoveIt 规划层启动时默认注入一块 `y=0` 中线隔板，范围 `x=0.4~0.76`、`y` 厚度 `0.1m`、`z=0~1.8`，用于阻止左右臂规划时穿过中间区域导致互撞。
- 改了哪里：`ros2_tmp/src/ik_benchmark/src/temporary_moveit_joint_planner.cpp`、`ros2_tmp/src/ik_benchmark/launch/temporary_moveit_joint_planner.launch.py`、`ros2_tmp/src/ik_benchmark/launch/fixed_platform_acceptance_stack.launch.py`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；`fixed_platform_acceptance_stack.launch.py --show-args` 可看到 `enable_center_separation_plate` 和 `center_plate_*` 参数。
- 留给下个 AI：如果隔板太保守导致规划失败，可先减小 `center_plate_y_thickness` 或缩短 `center_plate_x_max`；默认开启，启动日志会显示 `center_plate=on`。

## 2026-06-05 运控 / Codex / 固定平台末端 roll 朝向
- 做了什么：任务发布层保持末端朝 `+X`，并默认左手绕工具 `+X` roll `+90°`、右手 roll `-90°`，满足左手逆时针、右手顺时针的验收姿态需求。
- 改了哪里：`ros2_tmp/src/ik_benchmark/src/mock_box_task_publisher.cpp`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；发布层启动日志显示 `L ... roll=90.0`、`R ... roll=-90.0`。
- 留给下个 AI：如现场视觉判断方向相反，可通过任务发布层参数 `left_roll_deg` / `right_roll_deg` 直接交换符号，无需改 IK/MoveIt。

## 2026-06-05 运控 / Codex / 撤回末端默认 roll
- 做了什么：用户确认新增左右末端 roll 后 KDL IK 无解，已将任务发布层默认 `left_roll_deg/right_roll_deg` 撤回为 `0/0`，保持末端朝 `+X` 且不额外 roll。
- 改了哪里：`ros2_tmp/src/ik_benchmark/src/mock_box_task_publisher.cpp`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；发布层启动日志显示 `roll=0.0`。
- 留给下个 AI：roll 参数仍保留，可现场临时通过 `-p left_roll_deg:=... -p right_roll_deg:=...` 测试，但默认不要加，避免 KDL 无解。

## 2026-06-05 运控 / Codex / 保险演示模式
- 做了什么：新增两套保险演示入口。A：`fixed_platform_task_orchestrator` 支持 `demo_mode:=fixed_joint_target`，跳过 IK，直接把固定 12 轴 ROS/MoveIt 目标交给 MoveIt 规划再发 PLC bridge。B：新增 `fake_plc_cli_task_publisher.py`，回车后直接调用 `alfa_robot_plc_driver.cli move-abs` 写 PLC。
- 改了哪里：`ros2_tmp/src/ik_benchmark/src/fixed_platform_task_orchestrator.cpp`、`ros2_tmp/src/ik_benchmark/launch/fixed_platform_task_orchestrator.launch.py`、`ros2_tmp/src/ik_benchmark/launch/fixed_platform_acceptance_stack.launch.py`、`ros2_tmp/src/ik_benchmark/scripts/fake_plc_cli_task_publisher.py`、`ros2_tmp/src/ik_benchmark/CMakeLists.txt`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；launch 参数可见 `demo_mode`；CLI 伪任务入口可启动。
- 留给下个 AI：MoveIt 固定目标使用 ROS/MoveIt 方向 `left=-28,51,-38,30,-81,83; right=28,49,-35,-28,-79,-85`；PLC CLI 直发使用电控方向 `1:-28,2:51,3:38,4:30,5:81,6:83,7:28,8:-49,9:-35,10:-28,11:-79,12:-85`。

## 2026-06-05 运控 / Codex / PLC CLI 伪任务回车切换
- 做了什么：`fake_plc_cli_task_publisher.py` 从单次执行改为循环交互，每次回车在 `HOME(12轴全0)` 与 `TARGET(演示姿态)` 之间切换；第一次回车去 TARGET，第二次回车回 HOME。
- 改了哪里：`ros2_tmp/src/ik_benchmark/scripts/fake_plc_cli_task_publisher.py`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：`alfa_robot_benchmarks` 编译通过；入口可启动并显示 HOME/TARGET targets 与 velocity。
- 留给下个 AI：速度参数仍是 `-p velocity:=...`；可用 `-p home_targets:=...` 和 `-p targets:=...` 临时改两个端点。

## 2026-06-05 运控 / Codex / PLC CLI 01020102 循环演示
- 做了什么：`fake_plc_cli_task_publisher.py` 改为启动立即发 HOME，然后每次回车按 `TARGET1 -> HOME -> TARGET2 -> HOME` 循环；TARGET1 默认速度 60，TARGET2 默认速度 15，HOME 默认速度 15。
- 改了哪里：`ros2_tmp/src/ik_benchmark/scripts/fake_plc_cli_task_publisher.py`、`ros2_tmp/docs/fixed_platform_acceptance_runbook.md`。
- 验证结果：脚本语法检查通过，`alfa_robot_benchmarks` 编译通过；未在本机实际执行节点，避免启动即发 HOME 误动。
- 留给下个 AI：旧参数 `velocity` 现在忽略；需要改速度用 `home_velocity`、`target1_velocity`、`target2_velocity`。
