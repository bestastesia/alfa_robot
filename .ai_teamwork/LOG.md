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
