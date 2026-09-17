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

## 2026-06-16 运控 / Codex / 抽离阶段补充吸附箱自碰撞校验
- 做了什么：修复抽离阶段只检查裸机器人、未把吸附箱并入 RobotState 碰撞检测的问题；抽离单臂/双臂校验现在会将 carried box attach 到状态后再用 MoveIt collision checker 判断箱子与机器人自身、另一臂和场景是否碰撞。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；原四组回归中 L2/R4、L7/R9、L12/R14 均完成抽离并带箱规划到负重，L17/R19 仍按当前侧吸高度窗失败在 `h_interval_unreachable`。
- 留给下个 AI：抽离阶段的合法性现在包含“吸附箱 vs 机器人自身/场景”真实碰撞；若后续看到负重规划起点碰撞，优先检查动态箱墙/箱体尺寸或 touch link 设置，而不是再怀疑抽离漏掉 carried box。

## 2026-06-16 运控 / Codex / L2-R3 中间箱抽离后侧向让位验证
- 做了什么：在抽离后、负重规划前增加可选“中心列箱体向对应手臂侧向让位”阶段；目标最多侧移 0.4m，但允许部分成功，避免已走通的让位因为后续一步碰撞而被整体丢弃。
- 改了哪里：`loaded_pose_planning.*` 增加侧移规划；`extract_planning_pipeline.*` 记录侧移耗时/距离；`dual_arm_planner.launch.py` 和节点参数增加 `extract_loaded_lateral_shift_*`。
- 验证结果：L2/R3 快速测试成功，侧移后负重规划 `1/5` 成功；成功样本侧移约 0.12m。带 rollout 记录测试也成功，侧移约 0.08m，生成回放 `data/ik_benchmark/lateral_shift_after_extract/L2_R3_partial_rerun/L2_R3_partial_rerun.rrd`。
- 留给下个 AI：这个阶段默认关闭，需显式传 `extract_loaded_lateral_shift_enabled:=true`；目前证明“尽可能侧移”比“必须走满 40cm”更稳，后续可以继续优化侧移步长/规划超时。

## 2026-06-16 运控 / Codex / L7-R4、L8-R9、L12-R13 侧移方案回归
- 做了什么：按 `L7/R4;L8/R9;L12/R13` 跑同一套“抽离后尽可能侧移再负重规划”流程，并生成 Rerun。
- 结果：L7/R4 抽离成功且负重规划 `1/1` 成功，不需要侧移；L8/R9 抽离阶段 `0/64`，主要失败为左臂 carried box 与 `turn` 自碰；L12/R13 抽离阶段 `0/64`，主要失败为右臂未能与邻箱侧面脱离或与 `turn` 自碰。
- 数据：`data/ik_benchmark/lateral_shift_after_extract/L7R4_L8R9_L12R13_rerun/`，回放文件 `pairs.rrd`。
- 结论：侧移方案只解决“抽离已成功但负重规划卡住”的问题；L8/R9、L12/R13 当前瓶颈仍在抽离阶段本身，需要继续优化抽离动作/抓取 IK 起点。

## 2026-06-16 运控 / Codex / 货墙远 10cm 后三组抽离侧移回归
- 做了什么：将 `box_front_x` 从 0.825 调整到 0.925，重跑 `L7/R4;L8/R9;L12/R13`，其它算法参数不变。
- 结果：三组整体成功。L7/R4 抽离和负重规划成功且无需侧移；L8/R9 抽离成功 54/64，侧移 0.4m 后负重规划成功；L12/R13 抽离成功 64/64，侧移 0.4m 后负重规划成功。
- 数据：`data/ik_benchmark/lateral_shift_after_extract/L7R4_L8R9_L12R13_far10cm/`，回放文件 `pairs.rrd`。
- 结论：前一轮失败核心不是侧移策略本身，而是车/货墙距离不足导致抽离阶段过早贴近中心柱/箱墙；远 10cm 后抽离空间显著改善。

## 2026-06-16 运控 / Codex / 七组远距抽离成功样本筛选
- 做了什么：按 `L2/R3;L7/R4;L8/R9;L12/R13;L17/R14;L18/R19;L22/R23` 在 `box_front_x=0.925` 下快速筛选，并只保留成功 candidate 的 Rerun。
- 结果：成功组为 L7/R4、L8/R9、L12/R13；L2/R3 抽离有解但负重规划失败；L17/R14、L18/R19、L22/R23 在当前侧吸高度窗下 `h_interval_unreachable`。
- 数据：`data/ik_benchmark/lateral_shift_after_extract/seven_pairs_far10cm_success_only/summary.json`；精简回放 `success_candidates_only.rrd`。

## 2026-06-16 运控 / Codex / 负重规划碰撞校验口径修正
- 做了什么：排查“MoveIt 已规划但后验又说碰撞”的问题，确认原硬失败来自自定义 AABB 后验检查，与 MoveIt/FCL 规划场景不是同一套碰撞模型。
- 改了哪里：`dual_arm_planner_node.cpp` 的负重轨迹后验校验改为优先使用 MoveIt PlanningScene 同源碰撞检查；保守 AABB 检查默认只告警，不再作为硬失败。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；L2/R3 复跑 `extract_success=28/64`、`loaded_success=1/2`，数据在 `data/ik_benchmark/lateral_shift_after_extract/L2_R3_moveit_clearance_check/`。
- 留给下个 AI：如果后续看到 `Computed path is not valid`，这是 MoveIt 自己在规划管线里拒绝碰撞路径；如果只看到 AABB 告警，则说明真实 FCL 场景通过但保守包围盒过严。

## 2026-06-16 运控 / Codex / PLC 轨迹队列直连测试工具
- 做了什么：按 SEV-7 `Modbus双臂轨迹接口 V0.2-工程对齐版` 新增非 ROS 直连 PLC 测试工具，用于验证 `prepare/write/commit/start`、5/20/240 点上传和 12 轴小幅同步轨迹执行。
- 改了哪里：新增 `scripts/plc_trajectory_queue_test/plc_queue_client.py` 和 `scripts/plc_trajectory_queue_test/README.md`；协议常量使用 `Unit ID=1`、状态区 `100`、包头区 `200`、点数据区 `240~359`，`packetCrc32=0`。
- 验证结果：`python3 -m py_compile scripts/plc_trajectory_queue_test/plc_queue_client.py` 通过；本地生成 `mixed-delta` 20 点 CSV 成功，未连接/写入 PLC。
- 留给下个 AI：正式接桥层前先用该脚本在实机上按 README 顺序测试：只读状态、5 点保持 commit-only、5 点保持 start、20 点单轴小幅、40 点 `sync-wave`、40/240 点 `mixed-delta`；测试时必须传入真实 12 轴当前角度作为 `--base`，未运动轴不能填 0。

## 2026-06-25 运控 / Codex / PLC 执行链路退场与 EtherCAT 主栈切换
- 做了什么：Linear 已取消 PLC/Modbus 主线 `MOTION-27`、PLC 轨迹队列协作 `SEV-7`、PLC 插值测试 `MOTION-54`，新建 `MOTION-55` 作为 EtherCAT 主栈封装主线；本地清理旧 PLC bridge 与 PLC 队列测试工具。
- 改了哪里：删除 `ros2_ws/src/alfa_robot_plc_bridge/`、`scripts/plc_trajectory_queue_test/`；新增方向标定文档 `docs/ethercat/joint_direction_calibration.md`。
- 验证结果：源码层全局搜索旧 `alfa_robot_plc_bridge` / `plc_joint_trajectory` / PLC service / PLC 队列测试工具引用无残留；`colcon list` 不再发现 `alfa_robot_plc_bridge`；`source ros2_ws/install/setup.bash` 通过。
- 留给下个 AI：后续执行层优先围绕 `MOTION-55` 封装电控侧 EtherCAT 主栈 / ros2_control；常态开发应支持无电机 mock/仿真与实机主栈简单切换，不要恢复 PLC bridge 主线。

## 2026-06-26 运控 / Codex / 统一执行接口 mock 包
- 做了什么：新增 `alfa_robot_execution_bridge` ROS2 包，先提供统一 `FollowJointTrajectory` action 接口和 mock 后端；用于无电机/无 EtherCAT 主栈时打通上层任务编排、MoveIt 与执行层。
- 改了哪里：新增 `ros2_ws/src/alfa_robot_execution_bridge/`；默认 action 为 `/alfa_execution/execute_joint_trajectory`，mock 发布完整 13 轴 `/joint_states`。
- 验证结果：`python3 -m py_compile` 通过；`colcon build --packages-select alfa_robot_execution_bridge --symlink-install` 通过；本地启动 mock 节点并用测试客户端发送 13 轴轨迹成功返回。
- 留给下个 AI：真实 EtherCAT 后端应复用同一个 action 和 joint state 语义，只替换执行后端；mock 后端不使用 `direction_signs`，实机后端需要参考 `docs/ethercat/joint_direction_calibration.md` 做 ROS 方向到电机方向转换。

## 2026-06-26 运控 / Codex / execution bridge ros2_control 转发后端
- 做了什么：`alfa_robot_execution_bridge` 增加 `ros2_control` 后端，可把统一 action 转发到 `/dual_arm_trajectory_controller/follow_joint_trajectory`；默认关节顺序改为左臂 6 轴、右臂 6 轴、`turn`。
- 改了哪里：`execution_bridge_node.py` 现在支持 `mode=mock|ros2_control`；新增 `config/ros2_control_bridge.yaml`；launch 改为通用 `execution_bridge_node`；方向文档同步为左臂优先顺序。
- 验证结果：`python3 -m py_compile` 通过；`colcon build --packages-select alfa_robot_execution_bridge --symlink-install` 通过；mock action 烟测成功；ros2_control 模式在无下游控制器时会清晰返回 `downstream action server unavailable`。
- 留给下个 AI：实机测试前先启动电控侧 EtherCAT 主栈和 ros2_control；若下游已经处理方向，保持 `apply_direction_signs=false`，避免双重翻转。

## 2026-06-26 运控 / Codex / dual_arm_planner 丐版接 execution bridge
- 做了什么：`dual_arm_planner_node` 增加 `execution_backend:=alfa_execution_bridge` 路径，规划成功后不走 MoveIt execute，而是把 `JointTrajectory` 发到 `/alfa_execution/execute_joint_trajectory`。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 FollowJointTrajectory action client、MoveIt 关节名到执行接口关节名映射（`left_v5_joint*`→`left_joint*`，`right_v5_joint*`→`right_joint*`），并默认带 `turn` 保持值；`dual_arm_planner.launch.py` 暴露 execution 参数。
- 验证结果：`colcon build --packages-select alfa_robot_execution_bridge alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`dual_arm_planner.launch.py start_move_group:=false execute:=false execution_backend:=alfa_execution_bridge` 可启动到加载机器人模型。
- 留给下个 AI：这是“丐版接线”不是最终控制器；默认会拒绝规划里发生变化但未映射到执行接口的轴（如 `updown`），防止静默丢轴。若未来真实执行层支持更多轴，再扩展 `alfa_execution_joint_names()` 和映射表。

## 2026-06-26 Codex / 运控 / L6-R8 mock 执行闭环
- 做了什么：新增 L6/R8 单任务实时执行程序，流程为启动 mock 执行桥、计算 IK/抽离/负重规划、先执行全 0 到负重姿态、回车后按 10Hz 轨迹点执行任务，并用执行器反馈同步写入 Rerun。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py`；安装入口 `ros2_ws/src/alfa_robot_moveit_config/CMakeLists.txt`；执行桥配置 `ros2_ws/src/alfa_robot_execution_bridge/config/execution_bridge.yaml`；执行桥优雅退出 `ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge/execution_bridge_node.py`；`.gitignore` 放行新脚本。
- 验证结果：`colcon build --packages-select alfa_robot_execution_bridge alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；smoke 生成 `/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/live_mock_execution/L6_R8_mock_live_smoke.rrd`，计算成功，mock 执行全 0→负重 3.022s，L6/R8 任务轨迹 113 点、10Hz、执行 11.279s。
- 留给下个 AI：当前执行桥接口是 13 轴 `left_joint*`/`right_joint*`/`turn`，不含 `updown`；脚本按 `fixed_updown=0.3` 进行 Rerun 显示与执行语义，若后续实机需要升降轴运动，必须扩展执行接口或锁死 IK 的 h。

## 2026-06-26 运控 / Codex / L6-R8 实机流程迁移到工控机 lhy_dev
- 做了什么：将 L6/R8 全流程测试从 mock 版本扩展为实机直连版本，并把最小运行集同步到工控机 `~/lhy_dev`，不依赖旧运行目录。
- 改了哪里：`execute_l6_r8_mock_live.py` 支持 `executor-mode=mock|real`、自动探测仓库根目录、实机直连默认发 `/dual_arm_trajectory_controller/follow_joint_trajectory`；新增入口 `execute_l6_r8_real_live.py`；`extract_stage_monitor_console.py` 去除本机硬编码路径。
- 工控机内容：同步 `alfa_robot_description`、`alfa_robot_moveit_config`、`alfa_robot_execution_bridge`、`bio_ik`、`scripts/ik_benchmark` 到 `~/lhy_dev`；`pick_ik` 暂留但加 `COLCON_IGNORE`，当前流程只用 `bio_ik`。
- 验证结果：工控机 `~/lhy_dev/ros2_ws` 中 `alfa_robot_description/bio_ik/alfa_robot_execution_bridge/alfa_robot_moveit_config` 编译通过；`ros2 run alfa_robot_moveit_config execute_l6_r8_real_live.py --help` 可用；运行脚本中无 `/mnt/mydisk/ALFA/alfa_robot` 硬编码残留。
- 留给下个 AI：工控机真实控制器当前 joint order 是 `right_joint1..6,left_joint1..6,turn`，实机脚本默认按该顺序发送；内部/Rerun 仍按左臂优先整理。脚本只发送 12 个手臂轴 + `turn=0`，不发送 `updown`。Rerun 已安装到用户环境，`numpy` 保持 ROS 兼容的 `1.24.2`。

## 2026-06-28 运控 / Codex / robot_motion_scene_service 场景包独立化原型
- 做了什么：在 `v5_dev` 上新增 `robot_motion_scene_service` ROS2 包，把 `dual_arm_planner` 原本直接拥有的动态场景几何与 MoveIt PlanningScene 适配逻辑独立成包。
- 改了哪里：新增 `ros2_ws/src/robot_motion_scene_service/`，包含 `motion_core/task_geometry`、`motion_core/scene_geometry`、`motion_scene_adapter`；`alfa_robot_moveit_config` 改为依赖该包，旧同名头文件保留为兼容转发壳。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`robot_motion_scene_service` 自带 `test_scene_geometry` 通过。
- 留给下个 AI：当前命名空间仍保持 `alfa_robot::motion` 以降低旧 planner 拆分风险；迁移到 `robot_motion_control` 时可再统一命名。该包只负责动态世界/场景适配，不负责 IK、抽离策略、RRT 或任务状态机。

## 2026-06-28 运控 / Codex / 执行轨迹适配层拆分
- 做了什么：从 `DualArmPlannerNode` 中拆出 `ExecutionTrajectoryAdapter`，集中管理 MoveIt 轨迹到统一执行层 `FollowJointTrajectory` 的关节名映射、缺失轴 hold 位和未映射运动轴拒绝规则。
- 改了哪里：新增 `execution_trajectory_adapter.*` 与 `test_execution_trajectory_adapter.cpp`；`dual_arm_planner_node.cpp` 只保留配置装配和 action 发送逻辑；`alfa_robot_moveit_config` 增加 `trajectory_msgs` 依赖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，`test_execution_trajectory_adapter` 通过。
- 留给下个 AI：执行接口 seam 已集中，后续真实 EtherCAT/不同执行后端只应优先改 adapter 或 action client 装配，不要再把 joint name 映射规则散回 planner 节点。

## 2026-06-28 运控 / Codex / MoveIt 规划失败诊断拆分
- 做了什么：从 `DualArmPlannerNode` 中拆出 `planning_diagnostics`，集中管理场景碰撞原因、关节越界原因和 direct planning 失败诊断字符串。
- 改了哪里：新增 `planning_diagnostics.*`；`dual_arm_planner_node.cpp` 不再内联 `scene_collision_reason`、`group_bounds_reason`、`direct_pipeline_failure_diagnostic`；CMake 将诊断模块编入 `alfa_robot_motion_scene_adapter`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过。
- 留给下个 AI：后续排查 `direct_pipeline_planning_failed_code_*`、起点/终点碰撞、插值中越界时，优先看 `planning_diagnostics`，不要把诊断字符串散落回主节点。

## 2026-06-28 运控 / Codex / 重构分支纠偏
- 做了什么：按用户要求修正分支策略，将可读性重构提交从 `v5_dev` 独立到 `feature/motion-flow-readability-refactor-20260628`，并把本地 `v5_dev` 回退到 `origin/v5_dev`。
- 改了哪里：分支关系调整；当前 feature 包含 `robot_motion_scene_service` 场景包独立、`ExecutionTrajectoryAdapter` 执行轨迹适配层、`planning_diagnostics` 规划失败诊断模块三次重构提交。
- 验证结果：`v5_dev` 指向 `9c1e72d`，与 `origin/v5_dev` 一致；当前工作分支为 `feature/motion-flow-readability-refactor-20260628`，工作树干净后追加本日志。
- 留给下个 AI：后续所有“流程可读性/模块化”工作必须继续在该 feature 分支上小步提交，不要直接提交到 `v5_dev`。

## 2026-06-28 运控 / Codex / extract monitor 快照写入拆分
- 做了什么：从 `DualArmPlannerNode` 中拆出 `ExtractMonitorSnapshotWriter`，集中管理 extract monitor 阶段快照 JSON 的目录创建、写入和读取。
- 改了哪里：新增 `extract_monitor_snapshot_writer.*` 和单元测试；主节点只保留 `write_extract_monitor_snapshot()` 这一层日志包装，不再直接操作文件系统。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，现有 2 个测试均通过。
- 留给下个 AI：下一步若继续拆 extract monitor，优先抽出 snapshot JSON 构造/阶段状态机；文件写入已集中，不要再在节点里新增直接 `ofstream` 写快照。

## 2026-06-28 运控 / Codex / extract monitor JSON 格式层拆分
- 做了什么：从 `DualArmPlannerNode` 中拆出 `extract_monitor_json`，集中管理 RobotState、轨迹、候选 IK、单阶段回放的 JSON 格式。
- 改了哪里：新增 `extract_monitor_json.*` 和 `test_extract_monitor_json.cpp`；主节点保留少量包装函数，只负责补充当前场景的静态障碍上下文。
- 验证结果：在 `ros2_ws/` 下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，现有 3 个测试均通过。
- 留给下个 AI：后续 monitor snapshot 字段格式优先改 `extract_monitor_json`，不要继续把 JSON 拼装散在主节点里；更大的 `monitor_timing_json` 仍留在节点，之后可继续拆。

## 2026-06-28 运控 / Codex / extract monitor timing JSON 拆分
- 做了什么：继续收敛 extract monitor 审计格式，把 `monitor_timing_json` 的候选 rollout/侧移/负重尝试 JSON 打包逻辑移入 `extract_monitor_json`。
- 改了哪里：`extract_monitor_json.*` 增加 `extract_monitor_timing_json()`；`dual_arm_planner_node.cpp` 只负责传入当前 pair、目标关节名、携带箱和静态障碍上下文。
- 验证结果：在 `ros2_ws/` 下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，3 个测试均通过。
- 留给下个 AI：monitor 的 JSON 字段已经基本集中；剩余大块主要是 `run_extract_monitor_*` 阶段状态机和最终方案 replay 组装。

## 2026-06-28 运控 / Codex / monitor 失败原因统计 helper
- 做了什么：把 extract monitor 中抽离阶段和负重阶段重复的失败原因计数 JSON 构造收敛到 `failure_counts_json()`。
- 改了哪里：`extract_monitor_json.*` 新增失败统计 helper，`dual_arm_planner_node.cpp` 两处 snapshot 构造改为调用 helper，`test_extract_monitor_json` 增加覆盖。
- 验证结果：在 `ros2_ws/` 下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，3 个测试均通过。
- 留给下个 AI：阶段函数内仍有较多 snapshot 组装字段；后续可进一步把“extract_successes / loaded_plan_successes snapshot 构造”拆成命名函数。

## 2026-06-28 运控 / Codex / monitor snapshot 基础字段收敛
- 做了什么：把 extract monitor 各阶段 snapshot 重复的 `type/phase/phase_label/elapsed/box ids/box_front_x/scene_y_shift` 基础字段收敛到 `extract_monitor_snapshot_base()`。
- 改了哪里：`extract_monitor_json.*` 新增 snapshot base helper，`dual_arm_planner_node.cpp` 四个 monitor 阶段改为先创建 base 再追加阶段特有字段，`test_extract_monitor_json` 增加覆盖。
- 验证结果：在 `ros2_ws/` 下 `colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，3 个测试均通过。
- 留给下个 AI：snapshot 字段骨架已集中；后续可继续拆 IK/extract/loaded/final 四个阶段函数本身，优先从 `run_extract_monitor_final_stage()` 的 replay 组装开始。

## 2026-06-28 运控 / Codex / 可读性重构整体 review 与耗时回归
- 做了什么：整体 review `feature/motion-flow-readability-refactor-20260628` 相对 `origin/v5_dev` 的重构范围，确认场景包、执行轨迹适配、规划诊断、extract monitor JSON/快照写入拆分未改变主流程语义。
- 改了哪里：本轮只追加协作日志，无代码修改；重点复核 `robot_motion_scene_service`、`execution_trajectory_adapter`、`planning_diagnostics`、`extract_monitor_json`、`extract_monitor_snapshot_writer`。
- 验证结果：`robot_motion_scene_service` 与 `alfa_robot_moveit_config` 构建通过；4 个单测全部通过；L6/R8 代表性全流程 3 次内部耗时分别为 2779.43ms、2740.87ms、2698.89ms，平均 2739.73ms，和重构前约 2.7–2.8s 基准一致，未见明显耗时回退。
- 留给下个 AI：当前主节点仍有约 3889 行，剩余可读性优化应继续在该 feature 分支小步提交；优先拆 `run_extract_monitor_*` 阶段状态机和最终 replay 组装，避免再直接提交到 `v5_dev`。

## 2026-06-28 运控 / Codex / monitor 阶段快照构造收敛
- 做了什么：继续降低 `dual_arm_planner_node.cpp` 线性噪音，把 extract monitor 的 IK/抽离/负重阶段 snapshot JSON 构造收敛到 `extract_monitor_json`。
- 改了哪里：`extract_monitor_json.*` 新增 `extract_monitor_ik_snapshot()`、`extract_monitor_extract_snapshot()`、`extract_monitor_loaded_snapshot()`；主节点阶段函数只保留阶段流程与数据传入；`test_extract_monitor_json` 增加快照字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2836.01ms，快照 phase 为 `full_selected`，records=1，replay_stages=18。
- 留给下个 AI：主节点剩余最大线性块是 `run_extract_monitor_final_stage()` 的最终 replay 组装，可继续拆成“最终方案选择”和“replay 构造”两个更深的 Module。

## 2026-06-28 运控 / Codex / monitor 最终候选选择逻辑命名化
- 做了什么：把 `run_extract_monitor_final_stage()` 里“优先选择负重规划成功且吸附前过渡平滑的候选，否则退回第一个负重规划成功候选”的规则拆成命名函数。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `extract_monitor_pre_attach_transition_is_smooth()` 与 `select_extract_monitor_final_timing()`，最终阶段主流程不再内联候选选择循环。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2789.48ms。
- 留给下个 AI：下一步若继续提升可读性，优先把最终阶段的 replay 构造拆成独立 Module；当前这一步只改变代码组织，不改变选择规则。

## 2026-06-28 运控 / Codex / monitor 最终 replay 补录拆分
- 做了什么：继续拆 `run_extract_monitor_final_stage()`，把单状态 Plan 构造和“最终候选抽离 replay 缺失时补录”的细节从最终阶段主流程中拿出来。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `single_state_plan()` 与 `ensure_selected_extract_replay_records()`；extract 阶段和 final 阶段复用同一个单点状态 Plan 构造函数。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2901.97ms，快照 `replay_stages=21`，首段为 pre_attach，末段为 selected_loaded_plan。
- 留给下个 AI：最终阶段剩余可拆点是 pre_attach transition replay 构造、lateral shift replay 构造、loaded plan replay 构造；建议继续按“一个语义块一刀”的节奏，不要一次性搬大段。

## 2026-06-28 运控 / Codex / monitor 最终 replay 拼装分段命名化
- 做了什么：把 `run_extract_monitor_final_stage()` 中剩余的最终 replay 拼装按语义拆成 pre-attach 过渡、横向让位 replay、负重规划 replay 三个命名函数，主流程变成按阶段 append。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `append_pre_attach_replay_stage()`、`append_lateral_shift_replay_stages()`、`append_loaded_plan_replay_stage()`；最终阶段只保留选择候选、计算 goal、确保抽离 replay、组合四段 replay 和写 snapshot。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2798.15ms，快照 `replay_stages=17`，首段为 pre_attach，末段为 selected_loaded_plan。
- 留给下个 AI：final stage 的 replay 拼装已经具备清晰 seam；后续更大的收益来自把 monitor 状态机整体抽成类，或把 `ExtractMonitorState`/阶段函数从节点里移出。

## 2026-06-28 运控 / Codex / monitor 最终回放构建收束
- 做了什么：把最终阶段中“四段 replay 如何组合”的顺序收束到 `build_final_replay_stages()`，让 `run_extract_monitor_final_stage()` 只保留最终方案选择、goal 状态计算、snapshot 写入和状态推进。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `build_final_replay_stages()`，复用已有 pre_attach、抽离、横向让位、负重规划 replay append helper。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2894.90ms，快照 `replay_stages=18`，首段为 pre_attach，末段为 selected_loaded_plan。
- 留给下个 AI：monitor final 阶段已接近摘要式流程；下一步更值得做的是把 `ExtractMonitorState` 和 `run_extract_monitor_*` 阶段状态机整体从节点中独立出来。

## 2026-06-28 运控 / Codex / monitor 状态类型独立头文件
- 做了什么：为后续把 extract monitor 状态机从 `DualArmPlannerNode` 中移出做准备，先把 `ExtractMonitorPhase` 与 `ExtractMonitorState` 从节点私有定义抽到独立头文件。
- 改了哪里：新增 `include/alfa_robot_moveit_config/extract_monitor_state.hpp`；`dual_arm_planner_node.cpp` 改为 include 并使用该类型，删除节点底部内嵌 enum/struct。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过；L6/R8 `--once --no-rerun` 重跑成功，内部耗时 2999.16ms，快照 `replay_stages=19`。首次回归为 4306.51ms，重跑回到约 3s，判断为 IK/RRT 随机波动而非本次类型搬迁导致。
- 留给下个 AI：下一步可以把 `run_extract_monitor_next/full/ik/extract/loaded/final` 周围的状态机接口抽成独立 Module；当前已先把状态数据类型放到可引用的 seam。

## 2026-06-28 运控 / Codex / monitor 阶段调度 seam 拆分
- 做了什么：把 extract monitor 的 phase→stage 映射、next/full 调度顺序、阶段失败中文标签、full 阶段耗时汇总抽到 `extract_monitor_state` 模块中；节点只提供 IK/抽离/负重/最终四个阶段 callback。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 增加 `ExtractMonitorStage`、`ExtractMonitorStageCallbacks`、`run_extract_monitor_stage()`、`run_extract_monitor_full_sequence()` 等；`dual_arm_planner_node.cpp` 的 `run_extract_monitor_next/full_selected` 改为调用调度 seam；新增 `test_extract_monitor_state.cpp` 覆盖 phase 映射、Done 重跑语义、full 顺序和失败消息。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，4 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 3075.67ms，快照 phase=`full_selected`，records=1，replay_stages=17。
- 留给下个 AI：monitor 阶段调度已经有独立 seam；下一步可把阶段实现本身逐个移入一个 `ExtractMonitorRunner` 类，节点保留 ROS service 与依赖装配。

## 2026-06-28 运控 / Codex / monitor 控制器与场景重复 apply 修复
- 做了什么：把 extract monitor 的 next/full phase 持有逻辑收口为 `ExtractMonitorController`，节点不再直接写 `extract_monitor_phase_`；同时发现并修复同一箱墙 opening 重复 apply 到 MoveIt PlanningScene 时偶发长时间阻塞的问题。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 增加 Controller；`dual_arm_planner_node.cpp` 改为通过 Controller 调度四阶段 callback；`robot_motion_scene_service/src/motion_scene_adapter.cpp` 对相同 left/right opening 增加幂等跳过。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select robot_motion_scene_service alfa_robot_moveit_config` 通过，共 5 个测试通过；L6/R8 `--once --no-rerun` 复跑 3 次内部耗时为 2764.48ms、2929.44ms、2778.21ms，平均 2824.04ms，回到重构前 2.7–3.0s 量级。
- 留给下个 AI：这次异常慢的根因不是 Controller，而是旧的 MoveIt 场景重复 apply/同步偶发抖动；后续继续拆 `run_extract_monitor_*` 阶段实现时，注意不要新增无必要的 PlanningSceneInterface apply。

## 2026-06-28 运控 / Codex / IK 候选选择入口收口
- 做了什么：把“从 BioIK 全部候选中过滤合法解、按代价/h/seed 排序、按关节相似度去重、按数量裁剪”的入口收口到 `IkCandidateSelector`，让主节点不再掌握候选排序和去重细节。
- 改了哪里：`optimized_ik_pipeline.hpp/.cpp` 新增 `selectLegalFromResult()`；`dual_arm_planner_node.cpp` 的 monitor IK 阶段改为调用该深接口；新增 `test_ik_candidate_selector.cpp` 覆盖合法过滤、排序、去重和统计。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2728.45ms。
- 留给下个 AI：IK 候选去重/排序已经和 BioIK 优选模块放在一起；后续不要在 `dual_arm_planner_node.cpp` 里再写候选排序逻辑，应该继续扩展 `IkCandidateSelector` 或 IK pipeline。

## 2026-06-28 运控 / Codex / monitor 初始状态装配收口
- 做了什么：把 extract monitor 的初始状态装配规则收进 `extract_monitor_state` 模块，集中生成 prefix、箱号、左右携带箱 spec、seed state 和 loaded start state。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `extract_monitor_prefix()` 与 `make_extract_monitor_initial_state()`；`dual_arm_planner_node.cpp` 的 IK 阶段不再逐字段拼装 `ExtractMonitorState`；`test_extract_monitor_state.cpp` 增加初始状态字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2768.85ms。
- 留给下个 AI：monitor 状态生命周期已经更集中；后续如果继续拆阶段实现，优先让阶段函数读写 `ExtractMonitorState` 的位置更少、更集中。

## 2026-06-28 运控 / Codex / monitor 候选状态填充收口
- 做了什么：把 extract monitor 中“根据已选 IK 候选批量生成 RobotState 缓存”的容器操作收进 `extract_monitor_state` 模块，减少主节点里手写 candidate state 清空/预留/填充循环。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `populate_extract_monitor_candidate_states()`；`dual_arm_planner_node.cpp` 的 IK 阶段改为用该函数生成 candidate states；`test_extract_monitor_state.cpp` 增加填充调用计数和空 builder 覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2816.53ms。
- 留给下个 AI：monitor IK 阶段现在剩余主要是业务动作顺序；候选状态缓存已经有独立 seam，后续不要在节点里重复维护 `candidate_states` 容器规则。

## 2026-06-28 运控 / Codex / monitor 抽离阶段汇总收口
- 做了什么：把 extract monitor 抽离阶段中“timings 统计成功数、成功候选索引、失败原因计数”的规则收进 `extract_monitor_state`，让主节点不再手写审计统计规则。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `ExtractMonitorTimingSummary` 与 `summarize_extract_monitor_timings()`；`dual_arm_planner_node.cpp` 的抽离阶段 snapshot records 生成改为基于 summary；`test_extract_monitor_state.cpp` 覆盖成功、失败原因和 unknown 统计。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2769.82ms。
- 留给下个 AI：抽离阶段的统计口径已经集中；如果后续改“什么算抽离成功/失败分类”，优先改 `summarize_extract_monitor_timings()`，不要在节点里散写。

## 2026-06-28 运控 / Codex / monitor 负重规划汇总收口
- 做了什么：把 extract monitor 负重规划阶段中“按 batch plan indices 统计 attempted/success/failure_counts”的规则收进 `extract_monitor_state`，降低主节点对审计统计细节的认知负担。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `ExtractMonitorLoadedPlanSummary` 与 `summarize_loaded_plan_timings()`；`dual_arm_planner_node.cpp` 的 loaded 阶段 snapshot 使用 summary；`test_extract_monitor_state.cpp` 覆盖 attempted、success、unknown 和越界 index 忽略。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2908.60ms。
- 留给下个 AI：负重阶段统计口径已经集中；后续如果改并行规划候选统计，应优先改 `summarize_loaded_plan_timings()`。

## 2026-06-29 运控 / Codex / monitor 最终选择规则收口
- 做了什么：把 extract monitor 最终阶段“优先选负重成功且吸附前过渡平滑，否则回退第一个负重成功”的选择规则从主节点收口到 `extract_monitor_state` Module。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `select_extract_monitor_final_timing()`；`dual_arm_planner_node.cpp` 只保留平滑性 predicate；`test_extract_monitor_state.cpp` 补充优先选择、回退选择、无可用状态三类覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2800.52ms。
- 留给下个 AI：最终选择规则已有独立 seam；后续继续降低 `dual_arm_planner_node.cpp` 复杂度时，可优先把 pre-attach 过渡 plan 构造、最终 replay 构造进一步迁到更深 Module。

## 2026-06-29 运控 / Codex / monitor replay 公共上下文收口
- 做了什么：把 extract monitor replay 阶段里反复手写的 `candidate_order/loaded_plan_rank/left_box_id/right_box_id` 公共上下文字段收口到 `extract_monitor_json` Module。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_replay_context_json()`；`dual_arm_planner_node.cpp` 的 pre-attach、横向让位、负重 replay 统一复用该 helper；`test_extract_monitor_json.cpp` 增加字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2824.84ms，最终 replay 首尾阶段均保留 candidate/rank/box id。
- 留给下个 AI：replay JSON 公共字段已有单一 seam；后续改 replay 审计字段优先改 `extract_monitor_json`，不要在主节点里继续散写重复字段。

## 2026-06-29 运控 / Codex / monitor 候选查找规则收口
- 做了什么：把 extract monitor 中 `candidate_order` 到 IK 候选的越界检查和取值规则收口到 `ExtractMonitorState` Module，避免主节点在平滑性检查、抽离 replay 补录、最终 IK goal 构造中各自手写索引逻辑。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `extract_monitor_candidate_for_timing()`；`dual_arm_planner_node.cpp` 三处改为调用该 helper；`test_extract_monitor_state.cpp` 增加合法索引和越界返回空指针覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2831.58ms。
- 留给下个 AI：candidate_order 解释权已集中；后续如果改候选排序/筛选后索引语义，优先检查 `extract_monitor_candidate_for_timing()` 和 `IkCandidateSelector`，不要在主节点散写数组访问。

## 2026-06-29 运控 / Codex / monitor final snapshot 收口
- 做了什么：把 extract monitor 最终阶段 `final_selected` 快照 JSON 拼装从 `DualArmPlannerNode` 收口到 `extract_monitor_json` Module，和 IK/抽离/负重阶段 snapshot helper 保持同一风格。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_final_snapshot()`；`dual_arm_planner_node.cpp` final 阶段改为只传入 record 和 replay stages；`test_extract_monitor_json.cpp` 增加 final snapshot 字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2768.47ms，snapshot phase=`full_selected`，records=1，replay_stages=18。
- 留给下个 AI：四类 monitor snapshot 已全部有命名 helper；后续如改 monitor 快照格式，优先集中在 `extract_monitor_json`，主节点只负责提供业务数据。

## 2026-06-29 运控 / Codex / monitor 完整流程快照收口与耗时复核
- 做了什么：把 `run_extract_monitor_full_selected()` 中 `full_selected` 汇总快照字段收口到 `extract_monitor_json` Module，并对当前重构后的运行耗时做代表流程复核。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_full_selected_snapshot()`；`dual_arm_planner_node.cpp` 改为调用 helper；`test_extract_monitor_json.cpp` 增加字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 三次全流程内部耗时为 2762.59ms、2772.08ms、2790.77ms，仍在重构前约 2.7–3.0s 区间。
- 留给下个 AI：本轮收口没有引入耗时回退；后续如果继续降低 `dual_arm_planner_node.cpp` 复杂度，优先整体迁出 monitor 阶段 Implementation，而不是再抽浅 helper。

## 2026-06-29 运控 / Codex / monitor 候选任务调度收口
- 做了什么：把 extract monitor 抽离阶段里手写的候选多线程调度、worker 数量裁剪、timings 写回规则收口到 `extract_monitor_state` Module。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `extract_monitor_worker_count()` 与 `run_extract_monitor_candidate_tasks()`；`dual_arm_planner_node.cpp` 抽离阶段改为只描述单个候选如何 rollout；`test_extract_monitor_state.cpp` 增加 worker 规则和任务写回覆盖。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select robot_motion_scene_service alfa_robot_moveit_config` 通过，robot_motion_scene_service 1 个测试、alfa_robot_moveit_config 5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2942.44ms，仍在约 2.7–3.0s 波动区间。
- 留给下个 AI：抽离阶段并行调度已经集中；后续如果改“候选怎么分配给线程/是否早停/是否保留失败样本”，优先改 `run_extract_monitor_candidate_tasks()`，不要在主节点恢复手写线程循环。

## 2026-06-29 运控 / Codex / monitor 抽离 replay 记录收口
- 做了什么：把 extract monitor 抽离 replay step 的 JSON 记录规则从两个调用点收口成单一 helper，避免 `stage_kind/candidate_order/box_id/stage_name` 等字段双写。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `record_monitor_extract_replay_step()`；抽离阶段实时记录和最终阶段缺失 replay 补录都改为复用该 helper。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2853.29ms，snapshot phase=`full_selected`，replay_stages=18。
- 留给下个 AI：这步是局部 Locality 改善；更大的下一步仍是把 pre-attach replay 规划和最终 replay 组装整体移出主节点。

## 2026-06-29 运控 / Codex / monitor pre-attach 回放字段收口
- 做了什么：把最终回放里 pre-attach 过渡段的 `stage_kind/valid/method/transition_ms/failure_reason` 字段 schema 收口到 `extract_monitor_json` Module。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_pre_attach_replay_extra()`；`dual_arm_planner_node.cpp` 改为只传过渡结果；`test_extract_monitor_json.cpp` 覆盖成功和失败两类字段语义。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，5 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2809.06ms，pre_attach extra 中 method=`rrt`、valid=true。
- 留给下个 AI：pre-attach replay 的字段 Interface 已集中；下一步可把“插值失败再 RRT，再 shortcut/densify/验碰”的规划 Implementation 抽成更深 Module。

## 2026-06-29 运控 / Codex / monitor 预吸附过渡规划收口与耗时复核
- 做了什么：把最终回放中“负重位到 IK 吸附位”的过渡规划顺序收口到 `ExtractMonitorTransitionPlanner` Module，内部统一执行插值规划、densify、碰撞验证、失败后 RRT、shortcut、再次 densify 和验证。
- 改了哪里：新增 `extract_monitor_transition_planning.hpp/.cpp` 与 `test_extract_monitor_transition_planning.cpp`；`dual_arm_planner_node.cpp` 改为用 Adapter callback 装配具体 MoveIt/碰撞/shortcut 实现；`CMakeLists.txt` 增加源文件和单测。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，6 个测试全部通过；L6/R8 `--once --no-rerun` 三次内部耗时为 2828.94ms、2754.66ms、2772.92ms，平均约 2785.51ms，仍在重构前约 2.7–3.0s 区间。
- 留给下个 AI：pre-attach 过渡规划已有独立 Interface；后续若继续降低 `dual_arm_planner_node.cpp` 复杂度，可把 final replay 构造整体移到更深 Module，或把 `make_interpolated_joint_plan/densify/shortcut` 沉入通用轨迹 Module。

## 2026-06-29 运控 / Codex / monitor 最终回放 extra 字段收口
- 做了什么：继续把最终回放阶段的 JSON schema 从 `dual_arm_planner_node.cpp` 收口到 `extract_monitor_json` Module，主节点不再手写横向让位和负重规划回放的 extra 字段。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_selected_lateral_shift_replay_extra()` 与 `extract_monitor_selected_loaded_plan_replay_extra()`；`dual_arm_planner_node.cpp` 改为调用这两个 helper；`test_extract_monitor_json.cpp` 增加字段语义覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，6 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2761.31ms，pre_attach 和 selected_loaded_plan 回放字段保留 candidate/rank/valid。
- 留给下个 AI：monitor replay 的字段 schema 更集中；后续若继续重构，优先把 `build_final_replay_stages()` 的阶段组合 Interface 从主节点移出，而不是继续在主节点散写 JSON 字段。

## 2026-06-29 运控 / Codex / monitor 重构代码地图补全
- 做了什么：补充 `MOTION_PIPELINE_REFACTOR.md`，把本轮新增的交互式 monitor 相关 Module、数据流、测试入口和迁移建议写入文档，降低后续迁移到新仓库时的阅读门槛。
- 改了哪里：`docs/运控/MOTION_PIPELINE_REFACTOR.md` 增加 `extract_monitor_state`、`extract_monitor_json`、`ExtractMonitorSnapshotWriter`、`ExtractMonitorTransitionPlanner` 的责任说明，以及 monitor 分阶段数据流说明。
- 验证结果：检查文档中引用的头文件、实现文件和测试文件均存在；本提交为文档-only，沿用上一轮 `alfa_robot_moveit_config` 构建/测试和 L6/R8 代表流程验证结果。
- 留给下个 AI：后续迁移时先读该文档第 2、3.7、4 节，不要直接复制 `dual_arm_planner_node.cpp` 的线性流程。

## 2026-06-29 运控 / Codex / monitor 最终回放 stage 生成收口
- 做了什么：继续把最终回放阶段中“横向让位 replay stages”和“负重规划 replay stage”的生成规则从 `dual_arm_planner_node.cpp` 收口到 `extract_monitor_json` Module，节点只负责提供 target names、携带箱和静态障碍 Adapter。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_selected_lateral_shift_replay_stages()` 与 `extract_monitor_selected_loaded_plan_replay_stage()`；`dual_arm_planner_node.cpp` 删除对应循环和空指针/轨迹空判断；`test_extract_monitor_json.cpp` 用极简 RobotModel 覆盖 replay stage 生成。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，6 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2726.10ms，最终 replay 首段为 pre_attach、末段为 selected_loaded_plan。
- 留给下个 AI：最终 replay 生成细节进一步集中到 `extract_monitor_json`；主节点剩余主要是 ROS/MoveIt Adapter、IK 阶段装配和抽离 rollout Adapter。

## 2026-06-29 运控 / Codex / monitor 起始关节状态构造收口
- 做了什么：把 monitor 的 seed state / loaded start state 中“左右六轴 + updown 如何写入 RobotState”的规则从 `dual_arm_planner_node.cpp` 收口到 `extract_monitor_state` Module。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `ExtractMonitorArmSeed` 与 `make_extract_monitor_joint_state()`；`dual_arm_planner_node.cpp` 的 `make_extract_monitor_seed_state()` 和 `make_extract_monitor_loaded_start_state()` 改为只传配置；`test_extract_monitor_state.cpp` 用极简 RobotModel 覆盖左右关节和 updown 写入。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，6 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2791.84ms，pre-attach 起点 updown=0.3、left_v5_joint2=-1.3089969389957472。
- 留给下个 AI：monitor 初始状态装配规则进一步集中；主节点剩余主要是 ROS/MoveIt Adapter 和具体阶段调用顺序。

## 2026-06-29 运控 / Codex / monitor 阶段完成消息收口
- 做了什么：把 monitor IK/抽离/负重/最终阶段的中文完成消息格式从 `dual_arm_planner_node.cpp` 收口到 `extract_monitor_state` Module，减少主节点阶段实现中的样板输出拼接。
- 改了哪里：`extract_monitor_state.hpp/.cpp` 新增 `extract_monitor_*_stage_message()` 四个 helper；`dual_arm_planner_node.cpp` 改为直接调用；`test_extract_monitor_state.cpp` 增加消息格式断言。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，6 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2773.82ms，服务返回仍包含 `完整流程完成:`。
- 留给下个 AI：monitor 阶段输出文本已有集中 seam；后续如调整控制台/服务返回文案，优先改 `extract_monitor_state`，不要在主节点里散写。

## 2026-06-29 运控 / Codex / monitor 阶段 records 生成收口与耗时复核
- 做了什么：把 monitor IK 候选 records、抽离成功 records、负重尝试 records 的列表生成规则从 `dual_arm_planner_node.cpp` 收口到 `extract_monitor_json` Module，主节点不再手写 records 循环。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_candidate_records_json()` 与 `extract_monitor_timing_records_json()`；`dual_arm_planner_node.cpp` 改为调用 records helper；`test_extract_monitor_json.cpp` 增加候选/时序 records 覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，6 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2775.53ms，仍在重构前约 2.7–3.0s 区间。
- 留给下个 AI：这次只移动 JSON records 组装 Implementation，不改变 IK/抽离/负重规划算法；后续如果继续拆 `dual_arm_planner_node.cpp`，优先拆 ROS/MoveIt Adapter 或 monitor 阶段编排，不要再抽浅 pass-through helper。

## 2026-06-29 运控 / Codex / monitor 最终回放组装收口
- 做了什么：把 monitor 最终采用方案的 replay stages 组装顺序从 `dual_arm_planner_node.cpp` 收口到 `ExtractMonitorReplayBuilder` Module，主节点只保留 MoveIt/碰撞/补录抽离记录 Adapter。
- 改了哪里：新增 `extract_monitor_replay_builder.hpp/.cpp` 与 `test_extract_monitor_replay_builder.cpp`；`dual_arm_planner_node.cpp` 删除 pre-attach、横向让位、负重 replay append 样板函数；`MOTION_PIPELINE_REFACTOR.md` 补充模块地图。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，7 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2762.63ms，最终 replay_stages=18，首段为 selected_pre_attach，末段为 selected_loaded_plan。
- 留给下个 AI：最终回放阶段顺序已有独立 Interface；如果继续瘦身主节点，下一步可考虑把 `ensure_selected_extract_replay_records()` 的抽离补录 Adapter 继续下沉，或把 monitor IK 阶段的 solver 装配从节点迁出。

## 2026-06-29 运控 / Codex / monitor 抽离单步回放字段收口
- 做了什么：把 monitor 抽离 replay step 的 stage 名称和 extra 字段生成从 `dual_arm_planner_node.cpp` 收口到 `extract_monitor_json` Module，节点只保留单帧轨迹 Adapter。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `extract_monitor_selected_extract_replay_stage()`；`dual_arm_planner_node.cpp` 删除本地 `monitor_stage_json()` wrapper；`test_extract_monitor_json.cpp` 增加抽离单步回放字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，7 个测试全部通过；L6/R8 `--once --no-rerun` 两次全流程成功，内部耗时 3190.08ms 与 2724.17ms，后者回到近期约 2.7s 区间，快照中抽离 replay 字段保持 candidate_order/box_id/stage_kind。
- 留给下个 AI：抽离回放 JSON schema 已集中；后续如果继续拆 monitor，优先处理 `run_extract_monitor_ik_stage()` 的 solver/快照装配，或把 `ensure_selected_extract_replay_records()` 的补录 rollout Adapter 从主节点进一步下沉。

## 2026-06-29 运控 / Codex / 负重姿态距离指标收口
- 做了什么：把抽离候选到负重姿态族的距离指标写回逻辑从 `dual_arm_planner_node.cpp` 迁到 `LoadedPoseSelector`，让“如何计算/写入负重距离指标”归属负重姿态算法 Module。
- 改了哪里：`loaded_pose_planning.hpp/.cpp` 新增 `LoadedPoseSelector::fillTimingDistanceMetrics()`；`dual_arm_planner_node.cpp` 两个调用点改为调用 selector；新增 `test_loaded_pose_selector.cpp` 并接入 CMake；`MOTION_PIPELINE_REFACTOR.md` 补充职责说明。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，8 个测试全部通过；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2842.32ms，快照仍有 `loaded_pose_distance_sum/l2/max_joint_delta`。
- 留给下个 AI：负重距离指标不再散落在节点；后续如改候选排序或负重姿态族代价，优先看 `LoadedPoseSelector` 与 `LoadedPosePlanner`，不要把逻辑写回 ROS 节点。

## 2026-06-29 运控 / Codex / 重构整体 review 与耗时复核
- 做了什么：整体 review 了 motion flow 重构后的模块边界、测试覆盖和代表流程耗时；发现并修正 `front_z_reach_lower/upper` 在节点兜底默认值、launch 默认值和文档之间不一致的问题。
- 改了哪里：`dual_arm_planner_node.cpp` 的 front 侧吸高度窗兜底默认值对齐为 `0.45~1.25`；`docs/运控/MOTION_PIPELINE_REFACTOR.md` 同步参数表。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select robot_motion_scene_service alfa_robot_moveit_config` 通过，1+8 个测试全绿；L6/R8 `--once --no-rerun` 三次内部耗时为 2852.87ms、2809.78ms、2766.23ms，均值 2809.63ms，修正后复跑 2777.04ms，未见相对原 2.7~3.0s 基线的性能回退。
- 留给下个 AI：当前重构后的模块划分基本稳定；若继续瘦身，应优先迁出 `dual_arm_planner_node.cpp` 中剩余 ROS/MoveIt Adapter，而不是再抽浅 helper。

## 2026-06-29 运控 / Codex / IK 候选状态还原收口
- 做了什么：把“IK 候选 full_joint_names/full_joint_values 如何还原为 MoveIt RobotState”的规则从 `DualArmPlannerNode` 收口到 `optimized_ik_pipeline`，让节点不再掌握候选解写关节值的细节。
- 改了哪里：`optimized_ik_pipeline.hpp/.cpp` 新增 `robot_state_from_ik_candidate()`；`dual_arm_planner_node.cpp` 删除本地 `state_from_ik_candidate()` 并统一调用 IK 模块 helper；`test_ik_candidate_selector.cpp` 增加 RobotState 还原、未知变量忽略和 seed 保留覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，8 个测试全绿；L6/R8 `--once --no-rerun` 全流程成功，内部耗时 2845.96ms，仍在近期 2.7~3.0s 基线内。
- 留给下个 AI：IK candidate -> RobotState 的解释权已归入 IK pipeline；后续不要在 ROS 节点里重新散写 full_joint_names/full_joint_values 写回逻辑。

## 2026-06-29 运控 / Codex / IK 候选拒绝统计收口
- 做了什么：把 IK 候选 rejection reason 统计从 `DualArmPlannerNode` 收口为 `optimized_ik_pipeline` 的自由函数，同时整理 `optimized_ik_pipeline.cpp` 中重复 include/namespace 结构，让 IK 模块更像一个连续可读的实现文件。
- 改了哪里：`optimized_ik_pipeline.hpp/.cpp` 新增 `ik_candidate_rejection_counts_json()` 并由 `resultJson()` 复用；`dual_arm_planner_node.cpp` 删除本地 fallback 统计函数；`test_ik_candidate_selector.cpp` 增加 legal、指定失败原因和 unknown 统计覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，8 个测试全绿。本轮为 schema/可读性收口，不改变 IK/抽离/负重算法路径。
- 留给下个 AI：候选 rejection 统计已归 IK pipeline；monitor 和 recorder 若需要该字段，应继续调用 `ik_candidate_rejection_counts_json()`，不要在节点或 JSON 层重复实现统计。

## 2026-06-29 运控 / Codex / 目标关节顺序收口
- 做了什么：把 `updown + 双臂 12 轴` 的标准目标关节顺序从 `DualArmPlannerNode` 收口到 motion core，避免节点、monitor、recorder 各自持有关节顺序知识。
- 改了哪里：`robot_motion_scene_service/motion_core/task_geometry` 新增 `dual_arm_with_updown_joint_names()`；`dual_arm_planner_node.cpp` 删除本地 `arm_joint_target_names()` 并统一调用 motion core；`test_scene_geometry.cpp` 增加顺序断言；`MOTION_PIPELINE_REFACTOR.md` 更新模块职责。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select robot_motion_scene_service alfa_robot_moveit_config` 通过，1+8 个测试全绿。本轮只迁移常量规则，不改变运行路径或耗时。
- 留给下个 AI：关节目标顺序已归 motion core；后续新增执行器、回放、CSV 或 planner Adapter 时复用 `dual_arm_with_updown_joint_names()`，不要在节点里再手写 13 个名字。

## 2026-06-29 运控 / Codex / 场景记录 JSON 收口
- 做了什么：把集装箱板、动态箱墙、附着箱配置等记录/回放用 JSON schema 从 `DualArmPlannerNode` 收口到 `extract_monitor_json`，节点只保留从参数和 SceneAdapter 取值的 Adapter 角色。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 `container_panels_json()`、`container_obstacle_json()`、`static_box_obstacles_json()`、`attached_box_config_json()`；`dual_arm_planner_node.cpp` 删除对应手写 JSON 循环；`test_extract_monitor_json.cpp` 增加场景 JSON 字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，8 个测试全绿。本轮只迁移记录 schema，不改变场景建模、碰撞、IK、抽离或负重规划算法路径。
- 留给下个 AI：场景记录/回放 JSON 字段已集中到 `extract_monitor_json`；后续改 Rerun/JSONL 场景字段时优先改该模块，不要在节点里重新拼数组。

## 2026-06-29 运控 / Codex / 执行到位匹配规则收口
- 做了什么：把执行阶段“RobotState 是否到目标”和 `/joint_states` 是否到目标的匹配规则收口到 `ExecutionTrajectoryAdapter`，包括 MoveIt joint 名与 Alfa 执行 joint 名的兼容查找。
- 改了哪里：`execution_trajectory_adapter.hpp/.cpp` 新增 `ExecutionStateMatchRequest`、`ExecutionJointStateMatchRequest`、`robotStateMatches()`、`jointStateMatches()`；`dual_arm_planner_node.cpp` 删除本地循环判断细节，改为构造 Adapter request；`test_execution_trajectory_adapter.cpp` 增加误差阈值、忽略非机器人变量、Alfa/MoveIt 名称兼容覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 通过，8 个测试全绿。本轮不改变执行发送协议，只迁移到位判断职责。
- 留给下个 AI：执行层轨迹转换和到位判断都已归 `ExecutionTrajectoryAdapter`；后续 PLC/mock 执行对接优先扩展该 Adapter，不要在主节点继续写 joint 名映射循环。

## 2026-06-29 运控 / Codex / 抓取目标位姿规则收口
- 做了什么：把侧吸/顶吸抓取目标 Pose 的纯几何规则从 `dual_arm_planner_node.cpp` 收口到 `motion_core/pose_math`，让 node 只负责参数装配和流程编排。
- 改了哪里：`pose_math.hpp/cpp` 新增 `make_front_grasp_pose`、`make_top_suction_pose`；`dual_arm_planner_node.cpp` 删除局部 `front_grasp_pose/top_suction_pose`；新增 `test_pose_math` 锁定坐标偏移和朝向。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 9/9 通过；L6/R8 冒烟成功，内部耗时约 2982ms，仍在同一档。
- 留给下个 AI：下一步可继续收口 `dual_arm_planner_node.cpp` 中的 monitor replay/snapshot 胶水，但不要改变算法语义。

## 2026-06-29 运控 / Codex / monitor 抽离回放单步收口
- 做了什么：把 monitor 最终回放里“RobotState 单步抽离记录 -> replay stage”的轨迹构造和 JSON schema 从 `dual_arm_planner_node.cpp` 收口到 `extract_monitor_json`。
- 改了哪里：`extract_monitor_json.hpp/cpp` 新增 `extract_monitor_selected_extract_replay_state_stage()`；`dual_arm_planner_node.cpp` 的 `record_monitor_extract_replay_step()` 不再手工构造单点 plan；`test_extract_monitor_json` 增加单关节模型用例覆盖 state-stage 轨迹点生成。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 9/9 通过；L6/R8 冒烟成功，内部耗时约 2945ms。
- 留给下个 AI：`single_state_plan()` 仍被普通 keyframe 记录使用，暂不删除；后续可继续收口 snapshot 写入和 replay builder 装配。

## 2026-06-29 运控 / Codex / 单点轨迹构造工具收口
- 做了什么：把 `RobotState -> 单点 MoveIt Plan` 的重复实现收口到 `trajectory_plan_utils`，避免 node 与 monitor JSON 各自维护一份轨迹构造逻辑。
- 改了哪里：新增 `trajectory_plan_utils.hpp/cpp` 和 `test_trajectory_plan_utils`；`dual_arm_planner_node.cpp` 删除局部 `single_state_plan()`；`extract_monitor_json.cpp` 改为复用统一工具。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 10/10 通过；L6/R8 冒烟成功，内部耗时约 2897ms。
- 留给下个 AI：后续可继续把 `write_extract_monitor_snapshot()` 和 `build_final_replay_stages()` 这类 node 内装配胶水下沉，但要保持 ROS/MoveIt adapter 语义不变。

## 2026-06-29 运控 / Codex / monitor 快照写入错误语义收口
- 做了什么：把 monitor snapshot 写入失败的错误文案归到 `ExtractMonitorSnapshotWriter`，减少 `dual_arm_planner_node.cpp` 对快照路径/错误格式的了解。
- 改了哪里：`ExtractMonitorSnapshotWriter` 新增 `writeError()`；node 的 `write_extract_monitor_snapshot()` 改为调用 writer 格式化错误；测试补充路径和错误内容断言。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 10/10 通过。
- 留给下个 AI：后续更大的收益点仍是把 `build_final_replay_stages()` 和 `extract_monitor_transition_planner()` 的装配职责继续下沉。

## 2026-06-29 运控 / Codex / final replay request 构造收口
- 做了什么：把 monitor final replay 的 `ExtractMonitorReplayBuildRequest` 字段拼装从 `dual_arm_planner_node.cpp` 收口到 `ExtractMonitorReplayBuilder` 模块。
- 改了哪里：新增 `make_extract_monitor_replay_request()`，由 `ExtractMonitorState`、目标关节名、静态障碍 JSON 和 IK 目标状态生成 replay request；node 不再逐字段拼 request；`test_extract_monitor_replay_builder` 增加 factory 断言。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 10/10 通过；L6/R8 冒烟成功，内部耗时约 2813ms。
- 留给下个 AI：下一步可继续收口 `extract_monitor_transition_planner()` 的 adapter 构造，或把 final stage 的 snapshot/record 组合再下沉一点。

## 2026-06-29 运控 / Codex / 流程重构整体 review
- 做了什么：整体复核 `feature/motion-flow-readability-refactor-20260628` 的模块拆分、构建测试和运行耗时；确认重构主要是把 IK、抽离、负重规划、monitor 状态/JSON/replay 等从 `dual_arm_planner_node.cpp` 下沉为职责模块。
- 改了哪里：修正 `docs/运控/MOTION_PIPELINE_REFACTOR.md` 中抓取姿态函数归属和主节点行数说明。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；两包单测 11/11 通过；L6/R8 smoke 三次成功，阶段内部耗时约 2.78~2.83s；六任务序列跑通，L6/R3、L7/R8、L11/R12、L16/R13 成功，L1/R2 与 L17/R18 仍失败在最终选择阶段，符合近期已知难点。
- 留给下个 AI：性能看不出因重构回退；六任务脚本 wall_time 包含每个任务重新启动 MoveIt 的开销，评估算法耗时应看 snapshot/service 内部 `total_ms` 而不是总 wall。

## 2026-06-29 运控 / Codex / monitor候选状态映射收口
- 做了什么：把 `ExtractMonitorState` 中 `timing.candidate_order -> candidate RobotState` 的映射收口为 `extract_monitor_candidate_state_for_timing()`，减少 `DualArmPlannerNode` 反复理解候选索引和状态缓存细节。
- 改了哪里：`extract_monitor_state.hpp/cpp` 新增候选状态查询接口；`dual_arm_planner_node.cpp` 的最终选择平滑检查、抽离回放补录、最终 replay 起点改用该接口；`test_extract_monitor_state` 增加映射断言。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 10/10 通过；L6/R8 smoke 成功，内部耗时约 2876ms。
- 留给下个 AI：下一步可继续检查 `DualArmPlannerNode` 中纯 MoveIt 后端函数是否还能聚合为更明确的 planning/collision Adapter，但不要把 callback seam 拆成过浅文件。

## 2026-06-29 运控 / Codex / 去除monitor回放状态重建fallback
- 做了什么：继续收口 monitor 候选状态语义，删除 `ensure_selected_extract_replay_records()` 中从 IK candidate 现场重建 RobotState 的 fallback；回放补录统一使用 `ExtractMonitorState::candidate_states`，避免节点再次知道 candidate state 的构造细节。
- 改了哪里：`dual_arm_planner_node.cpp` 的 selected extract replay 补录逻辑。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 10/10 通过；L6/R8 smoke 成功，内部耗时约 2897ms。
- 留给下个 AI：`robot_state_from_ik_candidate()` 在节点内仍用于 IK stage 建候选缓存和 benchmark callback，这是当前合理 Adapter seam；不要为了删 using 而把清晰职责重新打散。

## 2026-06-29 运控 / Codex / monitor初始状态构造下沉
- 做了什么：把 monitor 的抓取 seed state 与负重起点 state 构造下沉到 `ExtractMonitorInitialStateRequest` / `make_extract_monitor_initial_state(request)`，节点只描述输入参数，不再保留两个手写 RobotState helper。
- 改了哪里：`extract_monitor_state.hpp/cpp` 新增 request 工厂；`dual_arm_planner_node.cpp` 删除 `make_extract_monitor_seed_state()` 和 `make_extract_monitor_loaded_start_state()`；`test_extract_monitor_state` 增加初始化请求断言。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 10/10 通过；L6/R8 smoke 成功，内部耗时约 2809.8ms。
- 留给下个 AI：monitor 初始化语义现在集中在 state 模块；后续若继续瘦主节点，可以优先处理 extract/loaded/final stage 的 snapshot request 构造，而不是拆 MoveIt callback 本身。

## 2026-06-29 运控 / Codex / 记录文件header构造收口
- 做了什么：把 `open_record_file()` 中手拼的大段 JSON header 收口为 `MotionFlowHeaderRequest` / `motion_flow_header_json()`，让节点只填运行参数，记录格式语义集中到 `MotionFlowRecorder` 模块。
- 改了哪里：`motion_flow_recorder.hpp/cpp` 新增 header request 与 builder；`dual_arm_planner_node.cpp` 改为调用 builder；新增 `test_motion_flow_recorder` 并接入 CMake。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，内部耗时约 2835.8ms。
- 留给下个 AI：记录 JSONL header schema 已集中；如果后续增加字段，优先改 `MotionFlowHeaderRequest`，不要再在 `DualArmPlannerNode::open_record_file()` 里堆 JSON。

## 2026-06-29 运控 / Codex / 抽离回放state阶段请求收口
- 做了什么：把 monitor 抽离回放 state 阶段的长参数调用收口为 `ExtractMonitorSelectedExtractReplayStateRequest`，节点侧只传一个请求对象，降低后续字段增减时漏传风险。
- 改了哪里：`extract_monitor_json.hpp/.cpp` 新增 request overload；`dual_arm_planner_node.cpp` 改用 request；`test_extract_monitor_json.cpp` 补 request 入口覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 三次 smoke 平均阶段内部耗时 2826.27ms，和历史 2778~2942ms 同级，无可见劣化。
- 留给下个 AI：继续重构时优先保持 request/schema 在 `extract_monitor_json` 内收口，不要把 replay 字段拼装重新散回节点。

## 2026-06-29 运控 / Codex / monitor完整快照写回收口
- 做了什么：把 `run_extract_monitor_full_selected()` 中读取旧快照、包裹 `full_selected` 快照、写回文件和拼接 snapshot 路径的逻辑收口到 `ExtractMonitorSnapshotWriter`，节点侧只描述本次完整流程的耗时输入。
- 改了哪里：`extract_monitor_snapshot_writer.hpp/cpp` 新增 `ExtractMonitorFullSelectedSnapshotRequest`、`writeFullSelectedSnapshot()` 和 `appendSnapshotPath()`；`dual_arm_planner_node.cpp` 改用 writer 接口；`test_extract_monitor_snapshot_writer.cpp` 补 full snapshot 写回覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2875.68ms，快照 phase 为 `full_selected`。
- 留给下个 AI：monitor 快照文件相关行为优先放在 `ExtractMonitorSnapshotWriter`，节点不要重新直接读写和拼装快照文件语义。

## 2026-06-29 运控 / Codex / monitor阶段快照写入语义收口
- 做了什么：把 monitor 普通阶段快照写入失败消息的构造集中到 `ExtractMonitorSnapshotWriter`，并保留节点侧 `fail()` 的记录副作用；IK/抽离/负重/final 四个阶段不再各自手写失败字符串。
- 改了哪里：`extract_monitor_snapshot_writer.hpp/cpp` 新增 `writeFailureMessage()`；`dual_arm_planner_node.cpp` 用 `write_extract_monitor_stage_snapshot()` 统一日志与 fail；`test_extract_monitor_snapshot_writer.cpp` 补失败消息覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2882.69ms。
- 留给下个 AI：如果继续拆 monitor 阶段，可继续把“生成快照 + 写快照 + 生成 stage message”的重复模式往专门的 monitor stage 结果模块里收，不要散回 callback 主流程。

## 2026-06-29 运控 / Codex / 抽离阶段快照请求收口
- 做了什么：把 monitor 抽离阶段快照的长位置参数收口为 `ExtractMonitorExtractSnapshotRequest`，让调用点显式描述 elapsed、box、candidate、worker、failure、records 等字段，降低读代码时猜参数含义的成本。
- 改了哪里：`extract_monitor_json.hpp/cpp` 新增 request overload；`dual_arm_planner_node.cpp` 的抽离阶段改用 request；`test_extract_monitor_json.cpp` 补 request 快照字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2890.69ms。
- 留给下个 AI：IK/loaded/final 快照仍可按同样方式逐步 request 化；每次只动一个 stage，降低 JSON schema 行为漂移风险。

## 2026-06-29 运控 / Codex / 负重阶段快照请求收口
- 做了什么：把 monitor 负重规划阶段快照的长位置参数收口为 `ExtractMonitorLoadedSnapshotRequest`，让 batch wall time、parallel workers、candidate limit、attempted/success 等字段在调用点有明确名字。
- 改了哪里：`extract_monitor_json.hpp/cpp` 新增 loaded request overload；`dual_arm_planner_node.cpp` 的 loaded 阶段改用 request；`test_extract_monitor_json.cpp` 补 request 快照字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2937.48ms。
- 留给下个 AI：monitor 快照剩余 IK/final 两个长参数入口可继续 request 化；建议先 final，字段更少、风险更低。

## 2026-06-29 运控 / Codex / 最终阶段快照请求收口
- 做了什么：把 monitor final 阶段快照的长位置参数收口为 `ExtractMonitorFinalSnapshotRequest`，让最终 record 与 replay stages 的含义在调用点显式化。
- 改了哪里：`extract_monitor_json.hpp/cpp` 新增 final request overload；`dual_arm_planner_node.cpp` 的 final 阶段改用 request；`test_extract_monitor_json.cpp` 补 request 快照字段覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2852.44ms。
- 留给下个 AI：monitor 快照长参数入口目前只剩 IK 阶段最明显；可继续做 `ExtractMonitorIkSnapshotRequest`，但字段包含 IK result/dedup/rejection，建议同样小步提交。

## 2026-06-29 运控 / Codex / IK阶段快照请求收口
- 做了什么：把 monitor IK 阶段快照的长位置参数收口为 `ExtractMonitorIkSnapshotRequest`，四个 monitor 阶段的快照入口现在都已有 request 化调用方式。
- 改了哪里：`extract_monitor_json.hpp/cpp` 新增 IK request overload；`dual_arm_planner_node.cpp` 的 IK 阶段改用 request；`test_extract_monitor_json.cpp` 补 IK request 与空指针保护覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2930.64ms。
- 留给下个 AI：monitor 快照入口已基本统一；下一步可以考虑把每个阶段的“生成快照 + 写入 + stage message”进一步收成 stage result helper，而不是继续扩展节点主流程。

## 2026-06-29 运控 / Codex / 抽离阶段消息请求收口
- 做了什么：把 monitor 抽离阶段完成消息的长参数收口为 `ExtractMonitorExtractStageMessageRequest`，让 success/total/workers/elapsed/snapshot_path 在调用点具名。
- 改了哪里：`extract_monitor_state.hpp/cpp` 新增 extract message request overload；`dual_arm_planner_node.cpp` 的抽离阶段 message 改用 request；`test_extract_monitor_state.cpp` 补 request 输出一致性覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2793.05ms。
- 留给下个 AI：阶段消息还剩 IK/loaded/final 可按相同模式 request 化；建议继续小步，避免一次性动所有 service 文案。

## 2026-06-29 运控 / Codex / 负重阶段消息请求收口
- 做了什么：把 monitor 负重规划阶段完成消息的长参数收口为 `ExtractMonitorLoadedStageMessageRequest`，让 success/attempted/candidates/elapsed/snapshot_path 在调用点具名。
- 改了哪里：`extract_monitor_state.hpp/cpp` 新增 loaded message request overload；`dual_arm_planner_node.cpp` 的 loaded 阶段 message 改用 request；`test_extract_monitor_state.cpp` 补 request 输出一致性覆盖。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2940.95ms。
- 留给下个 AI：阶段消息 request 化还剩 IK/final；继续保持小步提交和 smoke 验证。

## 2026-06-29 运控 / Codex / 重构整体复核与耗时回归确认
- 做了什么：整体 review 当前 `feature/motion-flow-readability-refactor-20260628` 的拆分状态、构建测试和 L6/R8 代表流程耗时；确认 `robot_motion_scene_service` 已承接场景几何/MoveIt 场景适配，`dual_arm_planner_node.cpp` 主要保留 ROS/MoveIt 装配和阶段 callback。
- 改了哪里：本轮只追加协作日志，没有改运行代码；复核范围包含 `robot_motion_scene_service`、`alfa_robot_moveit_config`、monitor snapshot/message/replay 模块和 `DualArmPlannerNode` 调用点。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；两包单测 1+11 全绿；L6/R8 三次有效 smoke 内部耗时 2838.06ms、2953.68ms、2911.89ms，平均 2901.21ms，仍在近期约 2.7~3.0s 波动区间，未见重构导致的明显耗时回退。
- 留给下个 AI：复核时一次 `ROS_DOMAIN_ID=233` 失败是 FastDDS 端口超限，不是代码问题；后续多轮 smoke 建议使用 0~120 这类安全 domain。代码层后续可继续收 IK/final stage message request，或把阶段 callback 的“快照+消息”组合成更深的 monitor stage helper。

## 2026-06-29 运控 / Codex / IK与最终阶段消息请求收口
- 做了什么：把 monitor IK 阶段和最终方案阶段完成消息的长参数收口为 `ExtractMonitorIkStageMessageRequest` 与 `ExtractMonitorFinalStageMessageRequest`，四个 monitor 阶段消息现在都支持具名 request 调用。
- 改了哪里：`extract_monitor_state.hpp/cpp` 新增 IK/final message request overload；`dual_arm_planner_node.cpp` 的 IK/final 阶段 message 改用 request；`test_extract_monitor_state.cpp` 补输出一致性覆盖。
- 验证结果：`git diff --check` 通过；`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2808.24ms。
- 留给下个 AI：monitor 阶段快照和消息的长参数已基本收口；下一步更有价值的是把“生成快照 + 写快照 + 生成消息”的阶段结果模式做成更深的 Module，而不是继续抽浅 helper。

## 2026-06-29 运控 / Codex / monitor阶段完成写入收口
- 做了什么：把 monitor 普通阶段的“写 snapshot + 返回成功消息 / 失败消息”的共同语义收口到 `ExtractMonitorSnapshotWriter::writeStageSnapshot()`，节点侧只保留 ROS 日志和 `fail()` 副作用，四个阶段末尾统一调用 `finish_extract_monitor_stage()`。
- 改了哪里：`extract_monitor_snapshot_writer.hpp/cpp` 新增 stage snapshot write request/result；`dual_arm_planner_node.cpp` 删除旧的 `write_extract_monitor_stage_snapshot()`，IK/抽离/负重/final 阶段改用统一完成入口；`test_extract_monitor_snapshot_writer.cpp` 补成功和空 snapshot 失败覆盖。
- 验证结果：`git diff --check` 通过；`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 11/11 通过；L6/R8 smoke 成功，阶段内部耗时 2859.15ms。
- 留给下个 AI：monitor 阶段末尾的重复模式已收口；如果继续降 `dual_arm_planner_node.cpp` 阅读成本，下一步应优先把每个阶段的 records/snapshot request 组装迁到更靠近 `extract_monitor_json/state` 的 Module。

## 2026-06-29 Codex / 运控 / motion pipeline 可读化重构收尾
- 做了什么：在 `feature/motion-flow-readability-refactor-20260628` 上继续收尾，提交并推送 monitor timing request、节点调用收口、场景迁移文档修正、携带箱 AABB 障碍检查下沉到 `robot_motion_scene_service`。
- 改了哪里：核心新增/调整包括 `ros2_ws/src/robot_motion_scene_service/`、`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`、`ros2_ws/src/alfa_robot_moveit_config/include/alfa_robot_moveit_config/*`、`docs/运控/MOTION_PIPELINE_REFACTOR.md`。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`robot_motion_scene_service` 1/1 单测通过；`alfa_robot_moveit_config` 11/11 单测通过；L6/R8 monitor full-selected 烟测成功，功能链路可跑通。
- 留给下个 AI：`DualArmPlannerNode` 仍约 3578 行，保留 ROS 参数、MoveIt 后端、碰撞判定和 callback 装配；不要继续往节点堆算法。L6/R8 烟测负重 RRT 阶段仍有随机耗时波动，曾出现约 3.8s 和约 14s 两类样本，属于 MoveIt/RRT 候选规划波动，不是本轮重构的确定性接口失败。迁移到新仓库时优先迁移 `robot_motion_scene_service`，再迁移 IK/抽离/负重模块，`dual_arm_planner_node.cpp` 只作包装参考。

## 2026-06-29 Codex / 运控 / motion pipeline 迁移残留清理
- 做了什么：基于最新 `v5_dev` 新建 `feature/motion-flow-followup-cleanup-20260629`，继续检查旧可读化重构分支遗留的半成品；删除已经迁到 `robot_motion_scene_service` 后仍滞留在 `alfa_robot_moveit_config` 的三份未编译旧实现，避免后续工程师误改僵尸代码。
- 改了哪里：删除 `ros2_ws/src/alfa_robot_moveit_config/src/motion_core/scene_geometry.cpp`、`src/motion_core/task_geometry.cpp`、`src/motion_scene_adapter.cpp`；更新 `docs/运控/MOTION_PIPELINE_REFACTOR.md` 说明真实实现位置和转发头兼容边界；补充 `robot_motion_scene_service` README/职责文档；新增 `test_task_geometry` 覆盖箱垛坐标、pair 解析、顶吸追加和 joint 顺序。
- 验证结果：`git diff --check` 通过；`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`robot_motion_scene_service` 2/2 单测通过；`alfa_robot_moveit_config` 11/11 单测通过。
- 留给下个 AI：当前场景几何和 MoveIt 场景适配的真实实现只在 `robot_motion_scene_service`；`alfa_robot_moveit_config/include/alfa_robot_moveit_config/motion_core/*` 和 `motion_scene_adapter.hpp` 只是兼容旧 include 的转发头。后续若继续规范项目，优先减少 `DualArmPlannerNode` 的 ROS/MoveIt 装配复杂度，不要把已迁出的场景实现拷回 MoveIt 包。

## 2026-06-30 Codex / 运控 / planner 启动稳定性修复
- 做了什么：针对“关闭后再次启动易连到旧 ROS 图、AI smoke 经常被残留进程绊住”的问题，新增进程组级清理、默认 ROS_DOMAIN_ID 隔离、fixed-h IK 懒初始化，并固化一条启动稳定性 smoke gate。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/scripts/process_lifecycle.py` 统一清理/域配置；`extract_stage_monitor_console.py`、`extract_sequence_rerun.py`、`run_extract_live_benchmark.py`、`extract_failed_attempts_rerun.py`、`execute_l6_r8_mock_live.py` 接入隔离与清理；`parallel_updown_aware_ik_solver` 避免 fixed 流程误触 free-h 求解池；新增 `extract_startup_stability_smoke.py` 与文档 `docs/运控/MOTION_PIPELINE_REFACTOR.md`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config` 12/12 通过；`ros2 run alfa_robot_moveit_config extract_startup_stability_smoke.py --rounds 2 --ros-domain-id auto` 通过，两轮服务就绪约 2.0s、旧 joint 名称污染为 0、退出后无 `/dual_arm_planner` 服务/进程残留。
- 留给下个 AI：后续启动/复启稳定性优先跑 `extract_startup_stability_smoke.py`，不要手工拼散命令；`--ros-domain-id auto` 会选 FastDDS 安全范围，显式传 233 以上会被拒绝，避免 robot_state_publisher 无限重启刷屏。算法完整流程偶发 `flow_success=false` 不等于启动稳定性失败，如需把算法成功率也作为门槛再加 `--require-flow-success`。

## 2026-06-30 Codex / 运控 / planner复用与IK预热边界修正
- 做了什么：针对“全流程 IK 阶段从约 0.5~0.8s 异常变成约 4s”的误判，确认根因是每组箱子重复启动 `dual_arm_planner_node`，把 16 个 BioIK solver 首次初始化算进单任务；新增 `/dual_arm_planner/configure_extract_monitor`，将 IK solver 预热归入启动期，并让一整段 pair sequence 复用同一个 planner。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 configure service 和任务切换/预热逻辑；`srv/ConfigureExtractMonitor.srv` 新增任务配置接口；`extract_sequence_rerun.py`、`extract_stage_monitor_console.py`、`extract_failed_attempts_rerun.py`、`execute_l6_r8_mock_live.py`、`extract_startup_stability_smoke.py` 接入“启动预热一次、每任务 configure、再 trigger compute”；`.gitignore` 放行该 srv 文件；`docs/运控/MOTION_PIPELINE_REFACTOR.md` 补充服务复用与耗时口径。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`extract_startup_stability_smoke.py --rounds 1 --ros-domain-id auto` 中 startup+prewarm 约 5971ms，单次完整流程内部 total 约 2654ms、IK 约 766ms；三组短序列共享 planner 复测中 L6/R3、L7/R8、L11/R12 的 IK 分别约 719ms、742ms、619ms，没有再把 4s 初始化混入 IK。
- 留给下个 AI：新增入口不要每个 box pair 重启 planner；正确顺序是等待 `/dual_arm_planner/configure_extract_monitor`、先 configure/prewarm，再调用 `/dual_arm_planner/run_extract_monitor_full_selected`。如果直接触发 full selected 而不 configure，首次调用仍可能把懒初始化算进任务耗时。

## 2026-06-30 Codex / 运控 / extract sequence服务客户端复用
- 做了什么：在 `extract_sequence_rerun.py` 的长序列入口中复用同一个 rclpy service client，避免每个任务都重新创建/销毁 ROS node；这不是核心算法优化，但能去掉 Python/DDS 客户端层面的数百毫秒外部墙钟开销。
- 改了哪里：`extract_stage_monitor_console.py` 新增 `ExtractMonitorServiceClient`；`extract_sequence_rerun.py` 的启动预热、每任务 configure 和 trigger 都改用同一个 client；`run_extract_live_benchmark.py` 也纳入跟踪并接入 configure/prewarm，避免被安装的旧入口继续走落后口径。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；三组短序列复测 `L6/R3;L7/R8;L11/R12` 共享 planner 成功，启动预热约 6100ms，任务 configure 外部墙钟分别约 0.4ms、25.6ms、34.3ms，IK 分别约 674ms、717ms、592ms。
- 留给下个 AI：长序列、多任务 benchmark 应优先使用可复用 client；一次性手动工具可以继续用 one-shot service helper，但不要用它作为性能口径。

## 2026-06-30 Codex / 运控 / 全流程复用边界与负重耗时口径复核
- 做了什么：按用户反馈重新从完整 `IK → 抽离 → 横向让位 → 负重规划 → 最终回放` 生命周期审计，而不是只盯 IK；确认 MoveIt backend、场景适配器、monitor 状态机、BioIK solver 池、长序列 service client 都可在同一 planner 生命周期内复用，任务级只切 box pair、箱墙开洞和 snapshot 路径。
- 改了哪里：`ExtractMonitorState` 记录负重规划 batch wall time、candidate/attempt/success/workers 等字段；最终 full-selected snapshot 保留这些字段；`extract_sequence_rerun.py` summary 同步输出 `loaded_plan_batch_wall_ms` 与负重候选统计；`MOTION_PIPELINE_REFACTOR.md` 明确 `loaded_elapsed_ms` 是负重阶段总耗时、`loaded_plan_batch_wall_ms` 才是并行规划批次耗时。
- 验证结果：待本轮最终构建/测试与短序列复跑补充；本轮目标是避免再次把启动预热、Python service client、snapshot 记录或 Rerun 回放误算成单任务核心算法耗时。
- 留给下个 AI：后续分析慢点时优先看 snapshot 中的分层字段：`startup_ms/configure_ms/ik_elapsed_ms/extract_elapsed_ms/loaded_elapsed_ms/loaded_plan_batch_wall_ms/final_elapsed_ms`，不要只看外部 `wall_ms`。

## 2026-06-30 Codex / 运控 / L6-R8实机方向安全锁
- 做了什么：针对 L6/R8 实机流程“方向又反”的高风险问题，确认风险不只在 EtherCAT sign，也在上层负重姿态族索引；将工控机 `/home/ar/lhy_dev/run_l6_r8_real.sh` 锁定为方向映射开启、`loaded_preferred_pose_index=0`、`hz=10`、`max_joint_speed=10deg/s`。
- 改了哪里：工控机 `/home/ar/lhy_dev/ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py` 支持负重姿态索引并禁止 real direct 关闭方向映射；`extract_stage_monitor_console.py` 将索引传入 planner；新增 `/home/ar/lhy_dev/verify_l6_r8_direction_safety.sh`；本地新增 `docs/ethercat/REAL_DIRECTION_SAFETY.md`。
- 验证结果：未发实机运动；`/home/ar/lhy_dev/verify_l6_r8_direction_safety.sh` 通过；`run_l6_r8_real.sh --no-real-apply-direction-signs` 和 `--loaded-preferred-pose-index=1` 均在运动前以 exit 2 拒绝。
- 留给下个 AI：不要再把 `--no-real-apply-direction-signs` 或 `loaded_preferred_pose_index=1` 加回 L6/R8 实机入口；如要改方向/速度/姿态索引，先用小角度单轴验证并同步更新安全文档。

## 2026-06-30 Codex / 运控 / L6-R8方向安全门入仓
- 做了什么：把 L6/R8 实机方向防线从工控机临时脚本扩展到仓库源码；防止后续从本仓库重新部署时把旧的 `loaded_preferred_pose_index=1` 或缺失 EtherCAT sign 映射带回实机。
- 改了哪里：`execute_l6_r8_mock_live.py` 固化 EtherCAT sign 表、默认负重姿态索引 0、real direct 禁止关闭方向映射、发送/feedback 同步应用 sign；`extract_stage_monitor_console.py` 将姿态索引传入 planner；`dual_arm_planner_node.cpp` 和 `dual_arm_planner.launch.py` 默认索引改为 0；新增 `scripts/safety/check_l6_r8_real_safety.py` 并接入 `alfa_robot_moveit_config` CTest；新增 `docs/ethercat/REAL_DIRECTION_SAFETY.md`。
- 验证结果：未发实机运动；`python3 -m py_compile` 通过；`scripts/safety/check_l6_r8_real_safety.py` 通过；`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`ctest -R check_l6_r8_real_safety` 通过；工控机 `/home/ar/lhy_dev/verify_l6_r8_direction_safety.sh` 通过。
- 留给下个 AI：任何修改 L6/R8 实机执行、负重姿态族、方向 sign、planner 默认索引前，先跑 `scripts/safety/check_l6_r8_real_safety.py`；若实机验证方向发生变化，必须同步改 safety doc、检查脚本和工控机 wrapper，不能只改一处。

## 2026-07-01 Codex / 运控工程化护栏
- 做了什么：针对当前运控代码风险，落地低风险工程护栏：统一负重阶段 AABB 为诊断默认、增加 motion baseline manifest/生成脚本、增加执行关节命名与方向契约检查、补充工程化护栏文档；Linear 已创建 MOTION-58 跟踪 URDF/tool0/坐标系/限位/实验结果版本化。
- 改了哪里：`docs/运控/工程化护栏/MOTION_ENGINEERING_GUARDS.md`、`docs/运控/MOTION_PIPELINE_REFACTOR.md`、`ros2_ws/src/alfa_robot_moveit_config/config/motion_baselines/current_motion_baseline.yaml`、`ros2_ws/src/alfa_robot_moveit_config/scripts/motion_contracts/`、`dual_arm_planner_node.cpp`、`dual_arm_planner.launch.py`、`.gitignore`、`CMakeLists.txt`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_moveit_config --return-code-on-test-failure` 13/13 通过；`ros2 run alfa_robot_moveit_config check_joint_contract.py` 通过；`generate_motion_baseline.py --print-id` 输出 `motion-baseline-2fb1335d1044bcca`。
- 留给下个 AI：当前未改高风险项：关节重命名、硬件协议/总线行为、生产生命周期、安全硬门槛、替换 BioIK/MoveIt/FCL。`scripts/ik_benchmark/scripts/preview_initial_yaw_turn_rerun.py` 是本轮前已有未提交改动，未触碰。

## 2026-07-01 Codex / 机械模型 / v9机器人URDF迁移
- 做了什么：将 `/mnt/mydisk/ALFA/backpack/alfa_robot_v2_arm_v9` 的 SolidWorks 导出模型迁入当前分支；保留项目既有 `left_v5_*`/`right_v5_*`/`tool0` 对外接口名，避免 MoveIt、运控、测试代码大面积改名。
- 改了哪里：新增 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_v2_arm_v9/{visual,collision}` 42 个 STL；重写 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 使用 v9 惯量、mesh、关节原点；同步 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro` 的 updown 控制上限为 v9 的 0.92m。
- 验证结果：`xacro` + `check_urdf` 对 description 和 MoveIt wrapper 均通过；`colcon build --packages-select alfa_robot_description` 通过；`colcon build --packages-select alfa_robot_moveit_config` 通过。
- 留给下个 AI：v9 原始 URDF 没有 `tool0`，当前沿用旧项目 `tool0_fixed xyz="0 0 0.209"`，需要机械/末端工具确认 TCP 是否仍正确；MoveIt SRDF 仍保留旧 v5 命名和碰撞矩阵，建议 RViz/MoveIt 可视化后再按 v9 外形重采样自碰撞禁用矩阵。

## 2026-07-01 Codex / 机械模型 / v9默认姿态归零
- 做了什么：将 v9 迁移后的默认启动姿态从负重姿态改为全 0；包括 description-only 预览、MoveIt home/initial positions、MuJoCo seed、ros2_control mock 初值。
- 改了哪里：`view_alfa_robot.launch.py`、`initial_positions.yaml`、`mujoco_initial_positions.yaml`、`alfa_robot.srdf`、`alfa_robot_macro.ros2_control.xacro`。
- 验证结果：启动姿态残留搜索只剩 joint limit 中的合法限位值；description 和 MoveIt wrapper 的 `xacro`/`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给下个 AI：当前 home/preview/mock 都是全 0；如果后续需要负重姿态，应新增命名 group_state 或配置项，不要覆盖默认 home。

## 2026-07-01 Codex / 机械模型 / joint4-5零位重映射
- 做了什么：将左右臂 joint4、joint5 的新 0 位重映射到旧模型的 +180° 位；joint5 限位同步改为 ±180°，joint4 已保持 ±180°。
- 改了哪里：`alfa_robot.urdf.xacro` 中左右 joint4/5 的 origin rpy 烘入 π 偏置；`joint_limits.yaml` 与 `alfa_robot_macro.ros2_control.xacro` 中左右 joint5 限位改为 ±π。
- 验证结果：description 和 MoveIt wrapper 的 `xacro`/`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给下个 AI：当前初始姿态仍为全 0；若实机编码器零位未同步，需要运控侧确认硬件零点/方向映射是否也要跟随此次语义重映射。

## 2026-07-05 Codex / 机械模型 / v10机器人URDF迁移
- 做了什么：按用户要求切换到 `feature/new-arm-iteration-motion-59-20260701`（HEAD `33cae9e`），将 `/mnt/mydisk/ALFA/backpack/alfa_robot_v2_arm_v10` 作为最新机械结构来源迁入；保留项目内 `left_v5_*`/`right_v5_*`/`tool0` 接口名。
- 改了哪里：新增待跟踪的 `ros2_ws/src/alfa_robot_description/meshes/current_robot/{visual,collision}` 42 个 STL；重生成 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 指向包内 `current_robot` mesh，并保留 joint4/joint5 的 +180° 零位重映射与 ±180° 限位；同步 `initial_positions.yaml`、`mujoco_initial_positions.yaml`。
- 验证结果：description 和 MoveIt wrapper 的 `xacro`/`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config --symlink-install` 通过。
- 留给下个 AI：v10 原始模型把原 `lidar_updown` 替换为 `front_sensor`，SRDF 未新增该 sensor 的碰撞禁用矩阵；当前继续沿用旧 `tool0_fixed xyz="0 0 0.209"`，等用户 RViz 查看后再确认 TCP 与碰撞矩阵是否要重采样。

## 2026-07-06 Codex / 机械模型 / v10 MoveIt全0碰撞矩阵修正
- 做了什么：复现 MoveIt demo 全 0 姿态碰撞；确认机械臂组本身无碰撞，碰撞来自 v10 新固定传感器/雷达与父 link 的 SRDF 禁碰矩阵未同步。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf` 新增 `updown-front_sensor`、`turn-lidar_front`、`turn-lidar_rear`、`turn-cam_front`、`turn-cam_rear` 的 Adjacent 禁碰项。
- 验证结果：临时 MoveIt checker 检查 all/left/right/dual/with_base 全 0 均 `collision=false`；`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给下个 AI：这次只禁用了固定相邻传感器与父 link，不影响机械臂-底座、机械臂-传感器等其他碰撞关系；后续如继续换 CAD，SRDF 仍建议按新几何重采样。

## 2026-07-06 Codex / 机械模型 / 新机械臂去版本命名适配
- 做了什么：按用户要求停止沿用 `v5` 命名，把当前 ROS2 核心包从“新机械结构 + v5 对外命名”调整为“新机械结构 + 无版本语义命名”；同步思考全流程从 demo runner 走向稳定仿真/孪生服务的架构问题。
- 改了哪里：`alfa_robot_description` 主 URDF 改为 `leftjoint1..6/rightjoint1..6`、`left_arm_base/right_arm_base`、`left_tool0/right_tool0`；当前 mesh 资产改为 `meshes/current_robot`，并移除 description 包内旧代际 mesh/vendor/raw 资产，安装规则只安装当前 mesh；`alfa_robot_moveit_config` 的 SRDF、kinematics、joint_limits、controller、planner、IK、抽离、负重、执行 Adapter 与相关测试同步改为 `left_arm/right_arm/dual_arm/dual_arm_with_base` 等无版本名；`robot_motion_scene_service` 的 joint list、attached box link、touch links 同步改名；`alfa_robot_bringup` 控制器配置和 launch 同步改名；删除未引用的 `alfa_robot_v5_proxy_backup.urdf.xacro`；新增文档 `docs/运控/工程化护栏/新机械臂命名与仿真服务化说明.md`。
- 验证结果：ROS2 核心包内 `left_v5/right_v5/dual_v5/_v5_` 搜索无残留；active/runtime 范围版本命名扫描无残留；`xacro` + `check_urdf` 通过；SRDF 引用检查 0 个坏引用；`current_robot` mesh 引用缺失数为 0；清理重建 `alfa_robot_description` 后，install/share 下 mesh 目录只剩 `current_robot`；`colcon build --packages-select alfa_robot_description robot_motion_scene_service alfa_robot_moveit_config alfa_robot_bringup alfa_robot_execution_bridge --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select robot_motion_scene_service alfa_robot_moveit_config --return-code-on-test-failure` 通过，15/15 单测成功；`move_group.launch.py` 烟测可加载模型、规划管线和 `dual_arm_controller`，仅保留未配置 Octomap 传感器的既有提示。
- 留给下个 AI：`simulation/mujoco/` 已清理为无版本命名，但还只是实验仿真资产；如果要把 MuJoCo 正式纳入孪生服务，必须做模型版本、场景状态和执行状态的 Adapter 验证。当前执行接口仍保留 `left_joint*/right_joint*` 作为对外易读名，由 `ExecutionTrajectoryAdapter` 转换到 MoveIt 内部 `leftjoint*/rightjoint*`，不要把这两个 Interface 混在一起。

## 2026-07-06 Codex / 仓库清理 / 旧IK包与实验资产瘦身
- 做了什么：按用户要求清理当前不再使用的 IK 包、旧 STL 资产和非最新 Rerun 结果；保留当前机械臂 `current_robot` mesh 与 BioIK/KDL 主线能力。
- 改了哪里：删除 `ros2_ws/src/pick_ik`、`ros2_ws/src/trac_ik`、`ros2_ws/src/dependencies.repos`、`scripts/setup_pickik.sh`；`scripts/ik_benchmark` 默认和快捷入口改为仅保留 `kdl`/`bio_ik`；`docs/运控/IK/ik_service.md` 更新为当前 IK 口径；删除 description 包旧 `meshes/alfa_robot` 与旧 `alfa_robot_macro.xacro`，删除 MuJoCo 旧生成 mesh 缓存；`data/**/*.rrd` 按“同目录同任务/朝向保留最新”策略删除 159 个旧文件，manifest 写入 `data/cleanup_manifests/rrd_cleanup_20260706_180429.txt`。
- 验证结果：当前 `alfa_robot_description` 源码和安装目录均只剩 `meshes/current_robot`；`colcon list` 中无 `pick_ik`/`trac_ik` 包；URDF mesh 引用 42 个、缺失 0 个；`xacro` + `check_urdf` 通过；`git diff --check` 通过；`colcon build --packages-select alfa_robot_description robot_motion_scene_service alfa_robot_moveit_config alfa_robot_bringup alfa_robot_execution_bridge alfa_robot_benchmarks --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`colcon test --packages-select robot_motion_scene_service alfa_robot_moveit_config --return-code-on-test-failure` 通过，15/15 单测成功。`data` 从约 14.6GiB 降到约 7.5GiB，RRD 从 278 个/11.96GiB 降到 119 个/4.86GiB。
- 留给下个 AI：`scripts/dh_workspace/configs/alfa_v*_urdf_left_arm_with_base.yaml` 仍保留旧 `alfa_robot_macro.xacro` 路径作为历史来源说明，不应用作当前 URDF 加载入口；历史报告中提到 pick/trac 只是历史对比，不代表当前仓库仍能运行这些插件。提交时注意把 `meshes/current_robot` 的 42 个 STL 纳入版本控制。

## 2026-07-03 Codex / 运控 / 混合侧吸顶吸全流程实验
- 做了什么：基于 `v5_dev` 新建 `feature/mixed-grasp-full-flow-research-20260703`，让 6 组抽箱序列支持按任务切换吸附模式；本次按 `front;front;front;top_suction;top_suction;top_suction` 运行。
- 改了哪里：`extract_sequence_rerun.py` 支持 `--grasp-mode-sequence`，失败任务也回放最佳失败候选；`extract_stage_monitor_console.py`/`dual_arm_planner.launch.py` 透传顶吸距离、顶吸 IK 容差和 IK 参数；`extract_monitor_state.cpp` 修正负重规划失败统计，横向让位失败也计入 failure_counts；`alfa_robot.srdf` 恢复 `lidar_front/rear <-> turn` 零位接触忽略。
- 验证结果：已生成 Rerun：`data/ik_benchmark/extract_sequence_mixed_grasp/mixed_front_top_failure_replay_20260703.rrd`；summary：`data/ik_benchmark/extract_sequence_mixed_grasp/sequence_20260703_164019/summary.json`。L6/R3、L7/R8、L11/R12、L16/R13 成功；L1/R2、L17/R18 失败但已写入失败回放轨迹。
- 留给下个 AI：L1/R2 主要失败在右臂 joint4 越界或 `right_v5_link5 <-> turn` 路径碰撞；L17/R18 失败在保守 AABB 判定 `carried_left_box_17 overlaps box_wall_L17_R18_left_side`。若继续优化，先针对这两个端点/路径碰撞诊断，不要先动顶吸 IK 主链路。

## 2026-07-03 Codex / 混合侧吸顶吸全流程验证
- 做了什么：修复 `extract_sequence_rerun.py`/`extract_stage_monitor_console.py` 到 `dual_arm_planner` 的顶吸模式传递；此前外层任务序列知道 `top_suction`，但 monitor planner 没收到 `extract_monitor_top_suction`，导致 Rerun 看起来全是侧吸。
- 改了哪里：`extract_stage_monitor_console.py` 新增 `--grasp-mode` 并传 `extract_monitor_top_suction:=true/false`；`dual_arm_planner_node.cpp` monitor IK/附着箱/目标 pose 使用 `extract_monitor_top_suction_`；`extract_sequence_rerun.py` 在失败且无 snapshot 时不中断，并在 Rerun 中标出失败目标点。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；新 Rerun 为 `data/ik_benchmark/extract_sequence_mixed_grasp/mixed_front_top_true_top_replay_20260703_c.rrd`，summary 为 `data/ik_benchmark/extract_sequence_mixed_grasp/sequence_20260703_171633/summary.json`。
- 留给下个 AI：新结果中 L11/R12、L16/R13、L17/R18 已真实进入 top_suction IK，但 512 trials legal=0；这不是“Rerun 全侧吸”问题，而是当前顶吸目标/窗口/姿态约束下 IK 无合法解。

## 2026-07-03 Codex / 顶吸车距修正
- 做了什么：修正混合侧吸/顶吸序列中顶吸车距语义；顶吸时车辆应向箱墙前进 0.30m，因此机器人坐标系下顶吸 `box_front_x` 默认使用侧吸 `box_front_x - 0.30`。
- 改了哪里：`extract_sequence_rerun.py` 新增 `--top-approach-forward` 和 `--top-box-front-x`，每个任务按吸附模式生成独立 planner 参数；顶吸失败标记和 Rerun 箱堆也使用顶吸有效 `box_front_x`。
- 验证结果：重新运行 6 组混合流程，Rerun 为 `data/ik_benchmark/extract_sequence_mixed_grasp/mixed_front_top_top_forward30_skip_extract_20260703.rrd`，summary 为 `data/ik_benchmark/extract_sequence_mixed_grasp/sequence_20260703_173034/summary.json`。顶吸目标从 `x=1.075` 修正到 `x=0.775`，顶吸 IK 从 `legal=0` 变为 `legal=458/468/470`。
- 留给下个 AI：当前后 3 组顶吸已不再卡 IK；失败转移到负重规划阶段，主要原因是附着箱体与 link3/link4 起点/目标碰撞，需要继续检查顶吸附着箱体姿态/几何或顶吸后的负重过渡策略。

## 2026-07-03 Codex / 顶吸附着箱体位置修正
- 做了什么：修复顶吸时箱体附着方向；经用户 Rerun 肉眼确认，当前 `left/right_v5_tool0` 的正 z 侧才对应吸盘外侧，顶吸箱体应位于 tool 正 z 方向。
- 改了哪里：`robot_motion_scene_service/src/motion_core/scene_geometry.cpp` 中 `make_attached_box_spec(top_suction=true)` 使用 `center_in_link.z=+carried_box_height/2`；`test_scene_geometry.cpp` 增加顶吸箱体位于 tool 正 z 方向的断言。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`robot_motion_scene_service` 测试构建和 `colcon test --packages-select robot_motion_scene_service --ctest-args -R test_scene_geometry --output-on-failure` 通过；用户确认顶吸 Rerun 中箱体附着方向正确。
- 留给下个 AI：不要再把顶吸 `center_in_link.z` 改回负值；后续顶吸失败应优先看顶吸抽离/负重过渡，而不是怀疑箱体“上飘”。

## 2026-07-04 Codex / 运控 / 混合侧吸顶吸六组全流程跑通
- 做了什么：完成 `1/2、6/3、7/8、11/12、16/13、17/18` 六组任务混合流程；前三组侧吸、后三组顶吸。顶吸抽离改为升降轴竖直抬升并用顶吸专用脱离判定，顶吸负重阶段改为“抬升后承载保持态”，不再强行套侧吸负重姿态。
- 改了哪里：`dual_arm_planner_node.cpp` 新增顶吸 lift extract 和 top-loaded hold 分支；`extract_planning_pipeline.cpp` 修正独立 KDL 链 joint 名到 MoveIt 变量名映射；`extract_sequence_rerun.py` 支持顶吸有效车距、顶吸专用负重姿态参数和失败回放；`scene_geometry.cpp` 保持顶吸箱体 `center_in_link.z=+height/2`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`python3 -m py_compile` 通过；`colcon test --packages-select robot_motion_scene_service --ctest-args -R test_scene_geometry --output-on-failure` 通过；完整六组全成功 Rerun：`data/ik_benchmark/extract_sequence_mixed_grasp/mixed_front3_top3_full_loaded3s_20260704.rrd`；summary：`data/ik_benchmark/extract_sequence_rerun/sequence_20260704_194914/summary.json`。
- 留给下个 AI：L1/R2 在默认 1s 负重规划时间下不稳定，3s 可通过；本轮成功命令使用 `--loaded-planning-time 3.0`。顶吸三组负重阶段是保持承载态，不等价于侧吸负重 RRT。

## 2026-07-07 Codex / 运控 / 三平行机械臂解析IK替换主链路
- 做了什么：新增 `alfa_robot_analytic_ik` 解析 IK 包，并把当前全流程主链路中的抓取 IK 与抽离单步 IK 从 BioIK/KDL 切到三平行解析解；MoveIt kinematics 配置改为空插件，MoveIt 仅保留 joint-space planning / scene checking；负重位姿默认改为左右 6 轴全 0，负重 updown 默认 0.0。
- 改了哪里：`ros2_ws/src/alfa_robot_analytic_ik/` 新增解析 IK 库与单测；`optimized_ik_pipeline.cpp` 使用 `analytic_three_parallel_fixed_h_cost_scorer` 生成/排序/去重候选，并用 MoveIt FK 做回代校验；`extract_planning_pipeline.cpp` 使用解析 IK 做固定 updown 抽离小步；`dual_arm_planner.launch.py`、`current_motion_baseline.yaml`、相关 monitor/rerun 脚本同步默认参数；`robot_motion_scene_service` 保留顶吸箱体位于 tool 正 z 方向。
- 验证结果：`git diff --check` 通过；`colcon build --packages-select alfa_robot_analytic_ik robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_analytic_ik robot_motion_scene_service alfa_robot_moveit_config --return-code-on-test-failure` 通过，解析 IK 1/1、scene service 2/2、MoveIt config 14/14 全绿。`test_analytic_moveit_fk` 覆盖解析 FK 与 MoveIt FK 一致。
- 留给下个 AI：运行烟测 L7/R8 可选出解析 IK 候选（`path=analytic_fixed_h`，180/180 legal），但完整流程仍失败在抽离阶段，`success=0/64`，主要碰撞为 `cam_front/lidar_front/front_sensor/updown` 与 arm link；L6/R8 当前左臂高位外侧目标解析 IK 无合法解。当前未证明完整抽箱任务成功，后续应先处理新机械臂场景/自碰撞策略或任务几何，而不是回退 BioIK/KDL。

## 2026-07-07 Codex / 运控 / 解析IK全流程Release验证
- 做了什么：基于三平行解析 IK 替换后的主链路，使用 Release 构建复跑六组混合侧吸/顶吸抽箱流程，并补充验证解析 IK 在任务链路中的实际耗时；同时修复 C++ 测试在 Release 下把 `initString()` 放进 `assert()` 导致测试 RobotModel 为空的问题。
- 改了哪里：`alfa_robot_analytic_ik` 继续作为抓取 IK 与抽离小步 IK 后端；`alfa_robot_moveit_config/test/*` 中测试 URDF/SRDF 初始化改为先执行再 assert，避免 `NDEBUG` 下副作用消失。
- 验证结果：`CMAKE_BUILD_TYPE=Release` 已确认；单任务 L17/R18 顶吸完整链路成功，内部总耗时 69.07ms，其中 IK 13.87ms、抽离 24.47ms、负重 2.79ms。六组 Rerun 已生成：`data/ik_benchmark/extract_sequence_rerun/analytic_ik_full_sequence_release_20260707.rrd`，summary：`data/ik_benchmark/extract_sequence_rerun/sequence_20260707_135723/summary.json`。六组中 L6/R3、L17/R18 成功；L1/R2、L7/R8、L11/R12 失败在最终选择/抽离结果为空，L16/R13 失败在 `h_interval_unreachable`。
- 留给下个 AI：解析 IK 的抓取 IK 耗时从旧 BioIK 链路约 586~722ms 降到约 12~15ms，优化约 45~55 倍；当前瓶颈已不在 IK，而在任务几何/抽离成功率与 L6/R3 负重 MoveIt 规划（本轮 L6/R3 负重批量规划约 3.32s）。当前测试门槛通过：`git diff --check` 通过；`colcon build --packages-select alfa_robot_analytic_ik robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON` 通过；`colcon test --packages-select alfa_robot_analytic_ik robot_motion_scene_service alfa_robot_moveit_config --return-code-on-test-failure` 通过，1/1、2/2、14/14 全绿。

## 2026-07-08 运控 / Codex / RRT*四组稳定性统计与解析IK测速
- 做了什么：为 `extract_sequence_rerun.py` 增加无 Rerun 批量统计能力，支持 `--no-rerun`、`--repeat`、`--stats-csv`、`--startup-retries`，并从 snapshot 计算全流程 12 旋转关节累计运动量。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/scripts/extract_sequence_rerun.py`；沿用本分支已有的 `extract_loaded_planner_id`/RRT* 配置。
- 验证结果：RRT* 前四组侧吸 60 轮测试完成，原始数据在 `data/ik_benchmark/extract_sequence_stats_rrtstar60/sequence_20260708_185748/stats.csv`，聚合在同目录 `stats_aggregate.csv/json`；解析 IK 500 次微基准：单臂约 357us，双臂约 714us。
- 留给下个 AI：RRT* 60轮中仅 `L6/R3` 第20轮出现一次最终选择失败，原因是 8 个负重候选均 `direct_pipeline_planning_failed_code_99999`；统计口径中 `motion_total_joint_rad` 不含 `updown`，`motion_total_axis_mixed` 含 `updown`。

## 2026-07-08 运控 / Codex / shortcut负重规划对比与IK单线程默认
- 做了什么：按四组侧吸任务 `L1/R3;L1/R8;L6/R3;L6/R8` 复测 RRTConnect、RRT*、shortcut 三种负重规划模式；shortcut 四组均成功且负重 batch 约 206ms，明显快于 RRT/RRT* 的约 1.2s。
- 改了哪里：`dual_arm_planner.launch.py` 和 `dual_arm_planner_node.cpp` 将 `ik_workers` 默认从 16 降到 1；`extract_sequence_rerun.py` 和 `extract_stage_monitor_console.py` 新增显式 `--ik-workers` 透传；新增 `robot_motion_interfaces` 轻量接口包，定义 `SolveArmIk`、`PlanExtract`、`PlanLoaded`、`CheckCollision`、`ExecuteTrajectory` 和 `RobotMotionState`；新增 `analytic_arm_ik_service_node` 和 `analytic_arm_ik_service.launch.py`，实现 `/robot_motion/solve_arm_ik` 单臂解析 IK 服务；`docs/运控/MOTION_PIPELINE_REFACTOR.md` 更新解析 IK、shortcut 对比和多阶段 interface/seam 说明；工程护栏同步解析 IK 口径。
- 验证结果：干净环境下 `colcon build --packages-select robot_motion_interfaces alfa_robot_moveit_config --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF` 通过；`analytic_arm_ik_service_node` 自检通过；真实 `ros2 service call /robot_motion/solve_arm_ik robot_motion_interfaces/srv/SolveArmIk` 返回 3 个左臂解析解；脚本 `py_compile` 通过；Release 复核对比 CSV 为 `data/ik_benchmark/loaded_planning_mode_compare_front4_release_current/summary_compare.csv`，shortcut 对四组平均总耗时约 586ms，RRT/RRT* 约 1.63s。旧六组当前第一组 `L1/R2` 未进入负重规划阶段，不能用于负重规划模式公平对比。
- 留给下个 AI：当前碰撞检查已支持 arbitrary requested state，通过 `make_full_scene_snapshot(start_state, attached_boxes)` 克隆 PlanningScene 后设置请求态；后续若拆独立 ROS 服务，要保留这种显式 `RobotState + scene + attached_boxes` 的接口，不要退回只检查当前 `/joint_states`。注意 Conda 会污染 ROS interface 的 Python typesupport 和 OpenSSL 链接；构建接口包时优先使用系统 Python 和干净环境。
- 补充验证：新增 `motion_collision_service_node` 和 `motion_collision_service.launch.py`，实现 `/robot_motion/check_collision`；输入显式 `start_state`、trajectory、scene objects、attached boxes，返回 valid/reason/contacts，不会静默替换成 live `/joint_states`。已通过 Release 编译，并用空场景显式 13 轴状态调用服务，返回 `valid=true`。
- 状态事实源补充：新增 `robot_motion_state_source.py` 和 `robot_motion_state_source.launch.py`，把指定 `/joint_states` 包装成 `/robot_motion/state`，消息类型为 `RobotMotionState`，包含 source、authoritative、context.state_id。已用隔离 `ROS_DOMAIN_ID` 烟测：发布一次 `/joint_states` 后 `/robot_motion/state` 正确输出 `authoritative=true` 和 state_id，Ctrl-C 退出干净。

## 2026-07-09 运控 / Codex / shortcut 局部 RRT 修补
- 做了什么：将抽离回放 transition planner 从“直连失败后整段 RRT”调整为“分段直连，局部被挡时只对局部窗口调用 RRT 修补”；loaded 阶段 shortcut 碰撞失败时也复用同一局部修补逻辑。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/extract_monitor_transition_planning.cpp`、`ros2_ws/src/alfa_robot_moveit_config/src/loaded_pose_planning.cpp`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；四组前侧吸 shortcut 回放成功生成 `/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/shortcut_local_rrt_patch/shortcut_local_rrt_front4.rrd`。
- 留给下个 AI：本次四组样例直连均未被挡，局部 RRT 分支未触发；后续若要专门验证局部修补，需要构造或选择 shortcut 直连碰撞但局部绕行可行的样例。

## 2026-07-09 运控 / Codex / 系统包职责 HTML 导航站
- 做了什么：新增 `docs/system_portal/` 静态多页导航站，用卡片和流程页展示系统外部交互、内部流程、包输入输出、状态和风险；数据源集中在 `assets/data.js`，后续项目更新时优先维护该文件。
- 改了哪里：`docs/system_portal/index.html`、`flows.html`、`packages.html`、`package.html`、`status.html`、`assets/data.js`、`assets/app.js`、`assets/site.css`、`README.md`。
- 验证结果：`node --check docs/system_portal/assets/data.js`、`node --check docs/system_portal/assets/app.js` 通过；本地脚本检查 HTML 的 CSS/JS 链接存在，核心包数据覆盖通过；`colcon test-result --verbose --test-result-base build/alfa_robot_moveit_config` 显示 14 tests, 0 failures。
- 留给下个 AI：新增/删除包或职责变化时，不要在 HTML 里手改卡片，优先更新 `docs/system_portal/assets/data.js`；如果后续迁移到 `robot_motion_control`，可复制此目录作为业务流程导航站初版。

## 2026-07-09 运控 / Codex / runtime 服务图与运行时前端初版
- 做了什么：新增 `robot_motion_runtime` 包，形成第一版可启动的运控运行时服务图；支持先通过 `/robot_motion/set_state` 固定仿真/Mock 的权威机器人状态，再通过 `/robot_motion/run_task` 启动最小任务链。新增运行时前端 `http://127.0.0.1:8766`，用于查看服务是否启动、各服务当前状态、请求成功/失败计数、最新 `RobotMotionState` 和 ROS graph。
- 改了哪里：`robot_motion_interfaces` 新增 `SetRobotMotionState.srv`、`RunMotionTask.srv`；`robot_motion_runtime` 新增 `motion_state_source_node`、`plan_extract_service_node`、`plan_loaded_service_node`、`execute_trajectory_service_node`、`motion_task_orchestrator_node`、`motion_runtime_dashboard_node` 和 `runtime_services.launch.py`；`docs/system_portal/assets/data.js` 与 `docs/运控/MOTION_PIPELINE_REFACTOR.md` 同步记录新运行时包职责。
- 验证结果：`/usr/bin/python3 -m py_compile ros2_ws/src/robot_motion_runtime/robot_motion_runtime/*.py` 通过；`node --check docs/system_portal/assets/data.js` 通过；`colcon build --packages-select robot_motion_interfaces robot_motion_runtime --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；隔离 `ROS_DOMAIN_ID=231` 启动 `runtime_services.launch.py subscribe_joint_states:=false execute_forward_action:=false dashboard_port:=8767`，确认 `/robot_motion/set_state`、`/robot_motion/plan_extract`、`/robot_motion/plan_loaded`、`/robot_motion/execute_trajectory`、`/robot_motion/run_task` 均存在；`set_state` 成功固定 `state_id=smoke:zero`；`run_task` dry-run 成功返回 `task chain complete in 5.06ms`；`/api/status` 显示各服务 request/success 计数和最新事实状态。
- 留给下个 AI：当前 `PlanExtract`/`PlanLoaded` 是独立服务门面，但内部仍是 shortcut baseline，不是最终 C++ 抽离 rollout / collision-aware loaded planner；`RunMotionTask` 当前输入仍是 IK candidate states，还没有把箱子编号/感知目标 -> IK candidate adapter 从 `dual_arm_planner_node` 拆出。下一步应把现有 `extract_planning_pipeline`、`loaded_pose_planning` 和 scene/collision adapter 挂到这些服务背后，而不是继续扩大 `dual_arm_planner_node`。

## 2026-07-09 运控 / Codex / runtime 完整服务图与 C++ 状态上报
- 做了什么：在 `robot_motion_runtime` 初版基础上新增完整栈启动入口，把运行时骨架、解析 IK 服务和碰撞服务一次性拉起；同时让 C++ 的 `/robot_motion/solve_arm_ik`、`/robot_motion/check_collision` 也发布 `/robot_motion/runtime_status`，前端能看到服务是否启动、最近一次请求详情、请求次数、成功/失败次数。
- 改了哪里：新增 `ros2_ws/src/robot_motion_runtime/launch/runtime_full_stack.launch.py`；新增 `alfa_robot_moveit_config/runtime_status_publisher.hpp`；修改 `analytic_arm_ik_service_node.cpp`、`motion_collision_service_node.cpp` 接入状态发布；`robot_motion_runtime` README、`docs/system_portal/assets/data.js`、`docs/运控/MOTION_PIPELINE_REFACTOR.md` 同步完整服务图说明。
- 验证结果：`/usr/bin/python3 -m py_compile ros2_ws/src/robot_motion_runtime/launch/*.py ros2_ws/src/robot_motion_runtime/robot_motion_runtime/*.py` 通过；`node --check docs/system_portal/assets/data.js` 通过；`colcon build --packages-select alfa_robot_moveit_config robot_motion_runtime --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；隔离 `ROS_DOMAIN_ID=229` 启动 `runtime_full_stack.launch.py subscribe_joint_states:=false execute_forward_action:=false dashboard_port:=8770`，确认 `/robot_motion/set_state`、`/robot_motion/solve_arm_ik`、`/robot_motion/plan_extract`、`/robot_motion/plan_loaded`、`/robot_motion/check_collision`、`/robot_motion/execute_trajectory`、`/robot_motion/run_task` 全部存在；调用 set_state、SolveArmIk、CheckCollision、RunMotionTask 均成功；`/api/status` 中 7 个 `/robot_motion/*` 服务均 available 且有 runtime_status；Ctrl-C 后所有进程干净退出。
- 留给下个 AI：`runtime_full_stack.launch.py` 已能展示完整服务图，但 `PlanExtract`/`PlanLoaded` 背后仍是 Python shortcut baseline；完整 C++ 抽离 rollout、负重规划、scene/collision 口径还没真正迁到独立服务背后。不要把当前 runtime 门面误认为已替代 `dual_arm_planner_node` 的完整算法。

## 2026-07-09 运控 / Codex / Plan 服务接入独立碰撞检查
- 做了什么：把 `PlanExtract` 和 `PlanLoaded` 从纯 shortcut 门面推进为可选碰撞过滤服务；完整栈默认开启 `plan_check_collision:=true`，两阶段生成候选轨迹后会调用 `/robot_motion/check_collision`，失败候选保留 failure_reason，成功候选优先排序。
- 改了哪里：`plan_extract_service_node.py`、`plan_loaded_service_node.py` 新增 `CheckCollision` client 与候选过滤；`runtime_services.launch.py` 新增 `plan_check_collision`/`collision_service_name` 参数；`runtime_full_stack.launch.py` 默认打开碰撞过滤；README、系统 portal、`MOTION_PIPELINE_REFACTOR.md` 同步说明。
- 验证结果：`/usr/bin/python3 -m py_compile ros2_ws/src/robot_motion_runtime/launch/*.py ros2_ws/src/robot_motion_runtime/robot_motion_runtime/*.py` 通过；`node --check docs/system_portal/assets/data.js` 通过；`colcon build --packages-select robot_motion_runtime --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；隔离 `ROS_DOMAIN_ID=230` 启动 `runtime_full_stack.launch.py subscribe_joint_states:=false execute_forward_action:=false dashboard_port:=8771`，调用 `set_state` 与 `run_task` 成功；dashboard API 显示 `plan_extract` 为 `collision_checked_shortcut`、`plan_loaded` 为 `shortcut+collision`，`check_collision.request_count=2`，说明两个 Plan 阶段均实际调用独立碰撞服务；Ctrl-C 后所有进程干净退出。
- 留给下个 AI：这一步统一了完整栈服务链的碰撞调用口径，但仍未把 C++ 抽离 rollout、带载 RRT/local-RRT 和箱子编号到 IK candidate adapter 迁入 runtime；后续继续削薄 `dual_arm_planner_node` 时应优先迁移这三块。

## 2026-07-09 Codex / MOTION-64 目标位姿服务入口
- 做了什么：在 `robot_motion_runtime` 增加目标位姿任务入口 `/robot_motion/run_dual_arm_pose_task`，链路为 `PlanDualArmIk -> PlanExtract -> PlanLoaded -> ExecuteTrajectory`；`PlanDualArmIk` 会调用左右两次 `/robot_motion/solve_arm_ik` 并组合双臂候选。
- 改了哪里：新增 `PlanDualArmIk.srv`、`RunDualArmPoseTask.srv`、`dual_arm_ik_candidate_service_node.py`；更新 `motion_task_orchestrator_node.py`、`runtime_services.launch.py`、dashboard 服务列表、README 和 `docs/运控/MOTION_PIPELINE_REFACTOR.md`。
- 验证结果：`colcon build --packages-select robot_motion_interfaces robot_motion_runtime --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`/usr/bin/python3 -m py_compile src/robot_motion_runtime/launch/*.py src/robot_motion_runtime/robot_motion_runtime/*.py` 通过；`node --check docs/system_portal/assets/data.js` 通过；提权 ROS smoke 通过：先 `/robot_motion/set_state` 固定事实状态，再调用 `/robot_motion/run_dual_arm_pose_task`，dashboard 显示 `set_state`、`solve_arm_ik`、`plan_dual_arm_ik`、`run_dual_arm_pose_task`、`plan_extract`、`plan_loaded`、`execute_trajectory` 均有请求计数且成功。
- 留给下个 AI：当前 `PlanExtract/PlanLoaded` 服务仍是 shortcut baseline，可调用碰撞服务但还没迁入完整 C++ 抽离 rollout、横向让位和 RRT/local-RRT；箱号/感知目标到左右目标 Pose 的 adapter 仍在旧 `dual_arm_planner_node` 体系里，需要继续外迁。

## 2026-07-09 Codex / MOTION-64 箱号任务入口服务
- 做了什么：新增 `/robot_motion/run_box_pair_task`，把左右箱号、侧吸/顶吸模式和箱墙几何转换成左右目标 Pose 与附着箱体，再转发 `/robot_motion/run_dual_arm_pose_task`；完整链路变为 `set_state -> run_box_pair_task -> run_dual_arm_pose_task -> plan_dual_arm_ik -> plan_extract -> plan_loaded -> execute_trajectory`。
- 改了哪里：新增 `robot_motion_interfaces/srv/RunBoxPairTask.srv`、`robot_motion_runtime/box_pair_task_adapter_node.py`；更新 `runtime_services.launch.py`、dashboard 服务列表、README、系统 portal 和 `docs/运控/MOTION_PIPELINE_REFACTOR.md`。
- 验证结果：`colcon build --packages-select robot_motion_interfaces robot_motion_runtime --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`/usr/bin/python3 -m py_compile src/robot_motion_runtime/launch/*.py src/robot_motion_runtime/robot_motion_runtime/*.py` 通过；`node --check ../docs/system_portal/assets/data.js` 通过；隔离 `ROS_DOMAIN_ID=228` 启动 `runtime_full_stack.launch.py`，先 `/robot_motion/set_state` 固定 h=0.55，再调用 `/robot_motion/run_box_pair_task` 的 L1/R3 侧吸任务成功，返回 IK/extract/loaded 各 4 个候选，dashboard `/api/status` 看到 8 个关键服务均有请求计数。
- 留给下个 AI：第一版 box adapter 固定 5×5 箱墙几何，只证明箱号语义入口已从 `dual_arm_planner_node` 外迁；真实感知箱体、动态箱墙 scene update、完整 C++ 抽离 rollout、横向让位和 RRT/local-RRT 仍需继续迁入独立服务。

## 2026-07-09 Codex / MOTION-64 场景事实源与运行时前端收口
- 做了什么：保留并完成 runtime 前端方向；新增 `/robot_motion/set_scene` 与 `/robot_motion/scene` 场景事实源，让仿真/mock 启动时可以像固定机器人状态一样固定碰撞场景。`PlanExtract` 和 `PlanLoaded` 现在优先使用请求里的 scene objects，否则使用最新权威 `/robot_motion/scene`，再统一传给 `/robot_motion/check_collision`。
- 改了哪里：新增 `RobotMotionScene.msg`、`SetRobotMotionScene.srv`、`motion_scene_source_node.py`；`RunMotionTask`、`RunDualArmPoseTask`、`RunBoxPairTask`、`PlanExtract`、`PlanLoaded` 增加请求末尾的 scene fields；runtime dashboard 增加场景事实状态面板和 `/robot_motion/set_scene` 服务；README、系统 portal 和 `docs/运控/MOTION_PIPELINE_REFACTOR.md` 同步。
- 验证结果：用去 conda 的干净环境删除并重建 `robot_motion_interfaces`、`alfa_robot_moveit_config`、`robot_motion_runtime`，构建通过；`py_compile` 和 `node --check docs/system_portal/assets/data.js` 通过；隔离 `ROS_DOMAIN_ID=228` 启动 `runtime_full_stack.launch.py subscribe_joint_states:=false execute_forward_action:=false dashboard_port:=8774 ik_root_samples:=360 ik_default_max_solutions:=4 plan_check_collision:=true`，依次调用 `set_state`、`set_scene`、`run_box_pair_task`，L1/R3 dry-run 成功，返回 `ik=4 extract=4 loaded=4`；dashboard `/api/status` 显示 `latest_scene.scene_object_count=1`，10 个关键服务均 available 且有 request_count。
- 留给下个 AI：前端已经值得保留，不需要回滚；但它仍是观测面板，不是控制台。`PlanExtract/PlanLoaded` 仍是 Python shortcut baseline + collision check，完整 C++ 抽离 rollout、横向让位、RRT/local-RRT 和动态箱墙感知更新还没有迁入 runtime 服务背后。修改接口后必须干净重编译，避免旧 typesupport 造成字段错位。

## 2026-07-09 Codex / MOTION-64 外部任务接口与任务回执
- 做了什么：新增面向整机/上游调度的简化任务入口 `/robot_motion/run_dual_grasp_task`，输入固定为左右末端位置、左右抓取模式、执行开关和速度比例；新增 `/robot_motion/task_receipt` 回执话题，发布 `accepted/running/succeeded/failed` 等整机任务状态。
- 改了哪里：`robot_motion_interfaces` 新增 `RunDualGraspTask.srv` 和 `TaskReceipt.msg`；`robot_motion_runtime` 新增 `dual_grasp_task_adapter_node.py`；`runtime_services.launch.py`、`runtime_full_stack.launch.py` 接入新节点；`execute_trajectory_service_node.py` 支持等待 FollowJointTrajectory action result，确保回执可以代表执行完成而不是仅代表命令已发送；README 增加简化任务调用和回执监听命令。
- 验证结果：`colcon build --packages-select robot_motion_interfaces robot_motion_runtime --symlink-install` 通过；`/usr/bin/python3 -m py_compile src/robot_motion_runtime/robot_motion_runtime/*.py src/robot_motion_runtime/launch/*.py` 通过；隔离 `ROS_DOMAIN_ID=131` 启动 `sim_bringup.launch.py` 与 `runtime_full_stack.launch.py execute_wait_for_goal_acceptance:=true execute_wait_for_result:=true`，调用 `/robot_motion/run_dual_grasp_task` 的 L6/R8 等价末端任务成功，返回 `state=succeeded`，回执话题依次输出 `accepted`、`running`、`succeeded`，仿真执行器确认执行 2s 轨迹。
- 留给下个 AI：外部接口已收敛为“左右末端点 + 侧吸/顶吸字段”；箱号入口 `/robot_motion/run_box_pair_task` 仍保留用于本地复现实验。若接真实执行层，需要确保执行后端能通过 action result 或等价完成信号让 `/robot_motion/task_receipt` 的 `succeeded/failed` 代表真实完成状态。

## 2026-07-10 Codex / 主仓库职责收口与历史代码清理
- 做了什么：主仓库只保留运控、电控、机器人模型和必要适配；删除感知、雷达导航、底盘 twist mux、旧 MuJoCo/受力仿真、DH/CuRobo 演示及其启动脚本。移除 MoveIt 配置包中的 MuJoCo/SemanticScene 链路和 Gazebo ros2_control 分支，保留 mock 与真实硬件两种后端。
- 架构调整：新增 `docs/运控/系统架构与包职责边界.md` 和 `docs/system_portal/architecture.html`，明确 interfaces → core → capability services → runtime → adapters → bringup 的单向依赖；`robot_motion_runtime` 负责唯一机器人/场景/任务事实，工具只能调用公开接口。移除 `.gitignore` 对 MoveIt scripts 整目录隐藏的规则，避免未跟踪调试脚本悄悄进入运行链；仍有价值的 Open3D CSV 工具迁入 `scripts/ik_benchmark`，旧 joint/group 调试脚本直接删除。
- 协作同步：当前角色与任务入口移除已退出仓库的仿真、导航、感知责任；可达性验证改归运控；归档历史不改。ROS 包索引只剩 11 个生产/兼容包和显式构建的 `alfa_robot_benchmarks`，已删除包不再出现在 overlay 中。
- 验证结果：干净 ROS 环境下保留包和 benchmark 全部构建通过；Xacro 展开和 `check_urdf` 通过；MoveIt 配置只导出 `ConfigureExtractMonitor`，已删除 Semantic 消息不再存在；解析 IK 1/1、场景 2/2、MoveIt 适配 14/14、执行桥 2/2 测试通过；Portal JavaScript 语法和 Python 工具编译检查通过。
- 构建注意：Conda `base` 会注入自己的 OpenSSL，造成 MoveIt C++ 链接失败；验证时已使用系统 PATH、清空 Conda/CMake/LD 环境后 source ROS。删除 rosidl 接口或移动包路径后必须清理对应 `build/<pkg>` 和 `install/<pkg>`，否则旧生成物会伪造失败。
- 留给下个 AI：`alfa_robot_moveit_config` 仍直接编译 `scripts/ik_benchmark` 的 IK 源码，运行时/Rerun 也仍有少量实验工具路径依赖；应按架构文档迁入 `robot_motion_core`/`robot_motion_tools`。`robot_motion_runtime` 当前没有自动测试，唯一事实源和任务状态机需要补服务级集成测试后才算生产门槛闭环。

## 2026-07-10 运控 / Codex / 核心与实验边界收口
- 做了什么：新建纯 C++ `robot_motion_core`，把 IK solver 配置、updown-aware 请求、候选、结果和代价函数 Interface 从 benchmark 迁入正式包；`alfa_robot_moveit_config` 与 `alfa_robot_benchmarks` 共同依赖该核心，不再互相复制类型或从生产 CMake 引用 `scripts/ik_benchmark`。
- 改了哪里：MoveIt 适配头文件和实现统一使用 `robot_motion::core` 类型；新增全体一方生产包的 benchmark 反向依赖护栏；把完整 URDF/FK/Rerun 实现迁入 `alfa_robot_rerun`，benchmark 旧脚本只保留兼容包装，实时 joint-state viewer 也从 runtime 迁到 Rerun 包；补齐 runtime 轨迹工具、唯一状态/场景源和 Rerun FK 单测，并修复显式 `set_state` 丢失请求 `frame_id` 的问题。
- 验证结果：干净 ROS 环境下 `robot_motion_core`、`alfa_robot_rerun`、`robot_motion_runtime`、`alfa_robot_moveit_config`、`alfa_robot_benchmarks` 构建通过；本轮 23 个相关测试全部通过，工作区累计 28 个测试零失败；隔离 ROS domain 的完整 runtime dry-run 链成功，12 个进程均可干净退出；安装态 Rerun 实时节点可加载当前 URDF（26 links）并正常启动；Portal JavaScript、Python 编译和 `git diff --check` 通过。
- 留给下个 AI：`robot_motion_core` 当前先承接公共数据模型，候选排序/去重、抽离 rollout 和轨迹评分仍在 MoveIt adapter 中；`dual_arm_planner_node` 与独立 planning service 的职责迁移尚未完成。`scripts/ik_benchmark` 仍是显式构建的实验包，但生产包已不能包含其头文件或动态导入其 helper。

## 2026-07-10 运控 / Codex / 箱体位姿 RRT 抽离实验分支
- 分支准备：将上一架构边界分支 squash merge 到 `v5_dev`，形成提交 `96a7600`；新建 `feature/extract-box-pose-rrt-20260710` 开发箱体位姿 RRT。
- 核心调整：在 `robot_motion_core` 新增纯 C++ 箱体位姿 RRT Module，支持侧吸 `retreat + pitch`、顶吸 `retreat + lift`、解析可达边回调、shortcut 和按关节累计运动量排序；MoveIt 包新增解析 IK Adapter，最后才调用统一双臂/附着箱/箱墙/集装箱碰撞检查。
- 场景调整：箱墙开洞几何后方增加 `_rear_guard`，防止搜索把箱子向货墙深处推进；该障碍由统一场景几何 Module 生成，MoveIt 场景和自定义检查共用。
- 诊断修复：修正侧吸附着箱局部轴误用和旋转方向选择；RRT 同时保留原始树路径与 shortcut 路径，定期从不同树节点主动连终点，并按完整状态序列去重；补充左右路径数、组合数、首个碰撞步和可选单臂隔离诊断；IK 快照新增场景过滤输入、通过数与拒绝原因；修复最终阶段失败回执为空；测试工具新增 `--extract-rollout-mode` 显式 A/B 参数。
- 验证结果：相关构建通过，累计 29 项测试零失败。L2/R3、L7/R4 均生成左右各 8 条路径并检查 64 组组合；隔离检查证明每只单臂自身已分别碰撞雷达/中心柱，并非双臂同步造成。已知可达顶吸 L22/R24 有 256 个合法解析解，但前 8 个候选在附着箱统一场景过滤时全部因箱墙或中心柱碰撞被拒绝，尚不能进入顶吸 RRT。
- 约束结论：当前严格侧吸 `retreat + pitch` 与顶吸现有附着场景下没有真实合法样本；未通过关闭碰撞、偷偷增加 lift 或改变箱墙来制造成功。稳定默认继续保持 `greedy`，新策略仅通过 `box_pose_rrt` 显式启用。
- 后续方向：继续推进前需明确评审新的允许自由度、机械避让或场景几何修正；当前实验已把失败边界定位到单臂碰撞和顶吸起点碰撞，而不是 RRT 路径数量不足。

## 2026-07-10 运控 / Codex / 十三组混合抓取与顶吸脱离口径修复
- 做了什么：固定 `2+3+3+3+2` 共十三组左右箱组合及逐臂侧吸/顶吸模式；序列工具默认显式启用 `box_pose_rrt`。IK 候选改为先对全部合法解排序去重，再做附着箱完整场景过滤并截取前 8 个，避免先截断造成假性无解。
- 改了哪里：在 `robot_motion_scene_service` 增加箱体相对原箱位的 X-Z 投影脱离判断；顶吸最终验收与箱体 RRT 核心统一。抽离/负重阶段 `0/N` 时立即失败并回传主导原因，失败快照保留真实阶段耗时；补十三组序列自动测试和固定传感器基线 SRDF 允许碰撞对。
- 验证结果：三包构建通过，30 项测试零失败。完整十三组真实流程中 `L11/R13`、`L16/R18` 成功；侧吸/混合任务主要在 RRT 第 4～8 步发生 `joint2 <-> updown`，底部任务主要没有附着场景合法 IK 起点。Rerun：`data/ik_benchmark/extract_sequence_rerun/box_pose_rrt_13_pairs_final_20260710.rrd`；统计：`data/ik_benchmark/extract_sequence_rerun/sequence_20260710_234401/stats.csv`。
- 留给下个 AI：顶吸箱体位姿 RRT 已证明几何与集成可行，但约 20 秒/任务；侧吸需要增加合理自由度或重设终点，底部任务需要先解决附着起点自碰撞/箱墙重叠。稳定默认不要直接改成 RRT。

## 2026-07-11 运控 / Codex / 抽离首碰撞帧诊断
- 做了什么：箱体位姿 RRT 双臂路径组合被碰撞拒绝时，保留首个碰撞前状态和碰撞状态；抽离全失败时优先选择 `joint2 <-> updown` 候选写入失败快照，避免只留下文字原因。
- 验证结果：`L1/R3` 稳定复现第 8 步 `leftjoint2 <-> updown`，Rerun 共两帧；两帧 `leftjoint2` 仅变化约 `1.685°`，说明碰撞来自连续路径逐步进入中心柱，而非关节突变。证据：`data/ik_benchmark/extract_sequence_rerun/L1_R3_joint2_updown_collision_frames.rrd`。
- 留给下个 AI：诊断帧只用于失败分析，不进入成功轨迹选择；黄色为碰撞前一帧，红色为首次碰撞帧。

## 2026-07-11 运控 / Codex / 代价函数前 IK 全量回放
- 做了什么：在优化 IK 管线中增加可选诊断捕获点，保存通过解析求解与 FK 误差校验、但尚未计算代价的全部解；序列工具新增 `--ik-only-raw`，可把十三组任务的全部原始合法解写入同一个 Rerun。
- 口径：原始解未评分、未排序去重、未做附着箱完整场景过滤；快照明确记录三项 false 标志，单条记录 `score=null`，按生成顺序编号。
- 验证结果：两包构建通过，30 项测试零失败；十三组共捕获 6684 个原始合法解，Rerun 为 `data/ik_benchmark/extract_sequence_rerun/all_tasks_pre_cost_ik_solutions.rrd`，大小约 74 MB；统计目录为 `data/ik_benchmark/extract_sequence_rerun/sequence_20260711_025903/`。

## 2026-07-11 运控 / Codex / IK 关节限位裕量代价实验
- 做了什么：解析 IK 默认关闭 h 移动代价；新增从 MoveIt RobotModel 真实关节上下限计算的非线性限位裕量代价，左右权重为 `[0.5,3.0,0.7,0.5,1.5,1.2]`，并增加参数与日志诊断。
- 验证结果：相关构建通过，30 项测试零失败；按基线相同的前 16 候选口径复跑十三组，成功率仍为 2/13，成功任务仍是 L11/R13、L16/R18。侧吸主因仍为 joint2 与 updown 的路径碰撞，证明单纯机械角限位代价不能描述中心柱几何净空。
- 证据：`data/ik_benchmark/extract_sequence_rerun/box_pose_rrt_13_pairs_joint_limit_cost_limit16_20260711.rrd`；`data/ik_benchmark/extract_sequence_rerun/sequence_20260711_050757/stats.csv`。
- 留给下个 AI：下一步应离线验证 `(h,joint1,joint2,joint3)->中心柱净空` 查表与抽离成功率相关性，不要继续盲目放大 joint2 机械限位权重。

## 2026-07-11 运控 / Codex / 系统架构驾驶舱表达升级
- 做了什么：在不新建生产前端的前提下，重构 `docs/system_portal/architecture.html` 的静态表达；按甲方三分钟阅读顺序增加能力宣言、控制闭环、能力指标、唯一事实链、工程保障区，并保留工程职责矩阵和迁移路径。
- 迭代：完成 10 轮有记录的内容/视觉/交互优化，包括交付与工程双视图、工业化视觉、响应式布局、打印版和静态资源验证。
- 验证结果：六个页面和三项静态资源经本地 HTTP 服务全部返回 200；`app.js`、`data.js` 通过 Node 语法检查；门户静态断言与 `git diff --check` 通过。
- 附带诊断结论：`all_tasks_pre_cost_ik_solutions.rrd` 是附着前运动学合法解，不代表附着箱场景合法。L11/R18 单独 IK 阶段有 296 个运动学合法解、去重后 136 个，但附着场景过滤全部拒绝：rightjoint2/updown 94、leftjoint2/updown 34、leftjoint2/turn 8。
- 留给下个 AI：门户仍是只读项目说明，不应扩成任务控制前端；碰撞性能下一步优先验证中心柱净空查表，其后再评估自有碰撞后端，不要把机械角限位代价误当几何净空。

## 2026-07-12 运控 / Codex / 箱体位姿 RRT 扩树碰撞检查
- 做了什么：将机器人自碰撞、附着箱与机器人/底座/场景碰撞直接接入箱体位姿 RRT 的边扩展；任一插值点失败时该边不进入搜索树，并保留开关复现旧口径。
- 改了哪里：`box_pose_rrt_extract_planner.cpp`、`dual_arm_planner_node.cpp`、抽离序列脚本、场景几何与 IK 候选诊断；基线说明见 `docs/运控/抽离策略实验/2026-07-12_RRT扩树碰撞口径基线.md`。
- 验证结果：`robot_motion_core` 2/2、`robot_motion_scene_service` 2/2、`alfa_robot_moveit_config` 16/16 测试通过。
- 留给下个 AI：下一实验方向应解除侧吸 `retreat`/`pitch` 单调和 `lift=0` 限制，目标改为箱体脱离区域，不再强制 90 度终态。

## 2026-07-13 运控 / Codex / 顶吸严格脱离与真实负重规划修复
- 做了什么：修复顶吸抽离成功口径，附着箱统一按末端真实旋转后的世界 AABB 判断；原箱位统一使用 world 坐标，消除重复减去 `world_to_base_z=0.202094m`；最终脱离余量统一为 3cm；删除顶吸负重阶段原地保持假成功，改为真实 `shortcut + 局部 RRT` 规划到负重位。
- 改了哪里：`alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`；新增验收工具 `alfa_robot_moveit_config/scripts/verify_extract_sequence_snapshot.py` 并纳入安装；场景几何旋转 AABB 测试补在 `robot_motion_scene_service/test/test_scene_geometry.cpp`。
- 验证结果：四包构建通过，30 项测试零失败；新 13 组运行中 6 个成功样本全部通过“真实箱体 X-Z 投影完全脱离 + 非零负重轨迹”自动验证；旧基线准确检出 7 个顶吸假成功。最终 Rerun：`data/ik_benchmark/free_space_front_rrt/strict_detach_loaded_full_13_selected.rrd`。
- 留给下个 AI：严格口径后 13 组成功率为 6/13；混合吸附任务主要失败在抽离，部分顶吸任务失败在解析 IK 与 MoveIt FK 校验，不要为了恢复旧成功率放松脱离验收或恢复原地负重假成功。

## 2026-07-13 运控 / Codex / 抽离相邻帧十度硬约束
- 做了什么：将抽离 RRT 每条边的连续性定义为“左右任一机械臂单关节真实指令差不得超过 10°”；超过时该边在扩树阶段直接拒绝。评分仍保留原六关节 L2 运动量，硬门槛单独计算，且不对有限位关节做 ±π 环绕短路，避免 360° 假连续。
- 改了哪里：`extract_planning_pipeline` 增加单关节最大差检查；`dual_arm_planner` 与 launch 默认 `extract_max_joint_delta=10°`；`pose_math` 增加向量最大绝对差函数和回归测试。
- 验证结果：定向构建和测试通过；十三组复跑 12/13 成功，所有成功任务相邻帧最大关节变化均不超过 10°，全局最大约 9.973°。L6/R13 失败在抽离阶段，64 个候选均无合法双臂 rollout，主因仍是左臂自碰撞/附着箱碰撞，另有少量 18.4° 的右臂边被新门槛拒绝。Rerun：`data/ik_benchmark/free_space_front_rrt/all_13_joint_delta_10deg_absolute_selected.rrd`；结果目录：`data/ik_benchmark/free_space_front_rrt/all_13_joint_delta_10deg_absolute/sequence_20260713_194221/`。
- 留给下个 AI：若要恢复 L6/R13，不应放宽 10°门槛；优先增加 RRT 中间节点/缩短单边步幅，或让解析 IK 分支选择显式延续上一状态。

## 2026-07-13 运控 / Codex / 负重 Shortcut 吸附箱后挡墙硬约束
- 做了什么：修复负重阶段 `shortcut + 局部 RRT` 对吸附箱后挡墙约束不完整的问题；后挡墙继续作为 MoveIt 场景障碍，同时增加吸附箱旋转世界 AABB 与 `_rear_guard` 的逐状态硬约束，任何相交轨迹点都不得被选为成功方案。其他保守 AABB 规则仍保持可选，避免扩大历史误判。
- 改了哪里：`robot_motion_scene_service/motion_core/scene_geometry` 新增后挡墙专用检查；`dual_arm_planner_node` 的完整场景状态检查无条件调用；`verify_extract_sequence_snapshot.py` 增加成功快照逐帧后挡墙验收；场景几何单测覆盖相交、分离和非后挡墙忽略。
- 验证结果：两包构建通过，30 项测试零失败。旧基线准确检出 5 组穿墙成功（L11/R18、L16/R13、L16/R23、L21/R18、L21/R23）；新 13 组复跑保留 7 组合法成功，成功负重轨迹共 200 帧，后挡墙相交 0。Rerun：`data/ik_benchmark/free_space_front_rrt/all_13_joint_delta_10deg_rear_guard_hard_selected.rrd`；结果目录：`data/ik_benchmark/free_space_front_rrt/all_13_joint_delta_10deg_rear_guard_hard/sequence_20260713_201011/`。
- 留给下个 AI：成功率从旧基线 12/13 降到 7/13 是剔除错误穿墙成功后的真实结果，不应通过关闭后挡墙硬约束恢复；后续应优化负重目标姿态或局部 RRT 搜索，而不是放松碰撞口径。

## 2026-07-13 运控 / Codex / 后挡墙局部修补与顶吸上抬中间态
- 做了什么：修正上一轮“后挡墙硬约束后直接失败”的处理方式；`shortcut` 失败后继续进入局部修补链路，同时把 `_rear_guard` 从整高墙改为只覆盖当前箱洞高度，避免顶吸箱体抬升后仍被整高后墙永久封死；顶吸负重规划在后挡墙碰撞时先尝试上抬中间态，再进入原有 `shortcut + 局部 RRT`。
- 改了哪里：`robot_motion_scene_service/motion_core/scene_geometry` 的 rear guard 高度与箱墙开洞高度统一；`loaded_pose_planning` 增加顶吸后挡墙上抬中间态；`extract_monitor_transition_planning` 增加自研局部 joint-space RRT 兜底，但当前主要成功来自上抬中间态而非该兜底。
- 验证结果：两包定向构建通过，`robot_motion_scene_service` 2/2 与 `alfa_robot_moveit_config` 16/16 测试通过。完整 13 组按 `loaded_candidate_limit=3` 复跑为 12/13 成功，12 个成功快照全部通过后挡墙验收。Rerun：`data/ik_benchmark/rear_guard_top_lift_all13_limit3.rrd`；结果目录：`data/ik_benchmark/rear_guard_top_lift_all13_limit3/sequence_20260713_225159/`。
- 留给下个 AI：唯一失败仍是 L6/R13，失败在抽离阶段 `box_pose_rrt_left_no_reachable_path`，不是负重后挡墙问题。L16/R23 和 L21/R18 虽恢复成功，但负重阶段仍很慢，分别约 18.9s 和 55.6s；后续应优化顶吸上抬后到负重位的中间姿态/目标姿态，而不是再放宽后挡墙碰撞。

## 2026-07-13 运控 / Codex / 整面后挡墙与 PP 轴约束修正
- 做了什么：按新评审意见把 `_rear_guard` 从“当前抓取箱洞后方”改成“整面箱墙后方”，x 固定在箱体后侧，y 覆盖集装箱内宽，z 覆盖完整 5 排箱堆高度；负重阶段顶吸后挡墙修补不再搜索 `updown`，改为确定性 PP 分段：`updown` 单调上抬、左右臂分别过渡、`updown` 单调回目标高度。
- 改了哪里：`robot_motion_scene_service/src/motion_core/scene_geometry.cpp` 与单测更新整面后挡墙几何；`loaded_pose_planning.cpp` 改顶吸后挡墙修补流程；`extract_monitor_transition_planning.cpp` 的自研局部 RRT 只允许单臂 6 轴进入采样，拒绝 `updown/pitch/turn` 和双臂 13 轴混合搜索，并在找到路径后用合法 shortcut 压缩平滑。
- 验证结果：两包构建通过，`robot_motion_scene_service` 与 `alfa_robot_moveit_config` 共 30 项测试零失败。13 组按 `--loaded-planning-mode shortcut --loaded-candidate-limit 3 --ik-full-h-range-scan` 复跑：8/13 成功，所有成功快照通过后验验证；Rerun：`data/ik_benchmark/fix_rearwall_pp_rrt_all13_rerun/full_sequence.rrd`；结果目录：`data/ik_benchmark/fix_rearwall_pp_rrt_all13_rerun/sequence_20260713_233042/`。
- 留给下个 AI：成功率低于上一版 12/13 是因为后挡墙按整面箱墙恢复后，混合/底部任务主要在抽离阶段无合法路径；当前失败为 L6/R13、L11/R8、L16/R23、L21/R18、L21/R23，主因是对应单臂 `box_pose_rrt_*_no_reachable_path`，不是负重后墙穿透。负重阶段慢任务仍集中在顶吸后 PP 上抬后的单臂过渡，需要继续优化中间姿态或目标负重姿态。

## 2026-07-14 运控 / Codex / PP 轴分段与双臂并行负重规划
- 做了什么：按 PP 运动约束重新定义负重阶段执行顺序：从初始/负重到 IK 仍允许 12 轴与 `updown` 同步；抽离阶段 `updown` 固定；抽离完成后先固定 `updown`，左右臂分别独立规划到负重姿态并合成为同一条并行轨迹执行，最后才单独移动 `updown` 到目标高度。默认负重姿态改为双臂 `[0,-45,120,-75,0,0]`。
- 改了哪里：`loaded_pose_planning.cpp` 新增单臂计划按时间合并函数和固定 h 的并行负重规划主路径；`extract_monitor_transition_planning.cpp` 保持局部 RRT 只处理单臂 6 轴；`dual_arm_planner.launch.py`、`extract_sequence_rerun.py`、`extract_stage_monitor_console.py`、`execute_l6_r8_mock_live.py` 同步新默认负重姿态。
- 验证结果：两包构建通过，30 项测试零失败。13 组按 `--loaded-planning-mode shortcut --loaded-candidate-limit 3 --ik-full-h-range-scan` 复跑：8/13 成功，成功快照全部通过；Rerun：`data/ik_benchmark/pp_split_loaded_parallel_all13_rerun/full_sequence.rrd`；统计目录：`data/ik_benchmark/pp_split_loaded_parallel_all13_rerun/sequence_20260713_235719/`。
- 留给下个 AI：失败任务为 L6/R13、L11/R8、L16/R23、L21/R18、L21/R23，均失败在抽离阶段 `box_pose_rrt_*_no_reachable_path`，负重阶段没有新增失败。慢任务主要是 L11/R13、L16/R18 的负重局部修补仍在秒级，需要优化单臂局部 RRT 或负重姿态族。

## 2026-07-14 运控 / Codex / MOTION-52 抽离RRT并行稳定与候选早停
- 做了什么：修复抽离阶段16线程候选并行时共享 PlanningScene/FCL 缓存导致的假失败；新增抽离成功候选 quorum，达到指定成功数后停止分发后续候选。
- 改了哪里：`dual_arm_planner_node.cpp` 增加每线程 PlanningScene 快照和场景 epoch；`extract_monitor_state.*` 支持 success quorum；`extract_sequence_rerun.py`、`extract_stage_monitor_console.py` 和 launch 暴露 `extract_success_quorum` / `extract_benchmark_extract_success_quorum` 参数。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config robot_motion_core --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`colcon test --packages-select robot_motion_core` 通过；L6/R13 16线程复现成功，总耗时 2748.3ms；13组全流程成功 13/13，Rerun 为 `data/ik_benchmark/threadlocal_scene_full13_20260714/full13.rrd`。
- 留给下个 AI：当前 RRT 已有 parent_candidates + best-first fallback；候选不足主要由带箱场景过滤和低位顶吸可达性决定。后续若继续优化，应优先记录每阶段 accepted/filtered 统计到最终 snapshot，并评估 loaded 阶段 2.3s 案例。

## 2026-07-14 运控 / Codex / MOTION-52 扩展抽离入口 IK 候选
- 做了什么：修正 IK 阶段候选过滤口径；抓取起点只检查机器人/携带箱与场景碰撞，不再要求携带箱已经完成抽离，从而避免过早删除可用于后续 RRT 的姿态。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `state_clear_for_dual_grasp_start`，IK 候选场景过滤改用抓取起点过滤；真正抽离阶段仍使用 `state_clear_for_dual_extract` 判断 detachment。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config robot_motion_core --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`colcon test --packages-select robot_motion_core` 通过；13组全流程 13/13 成功，Rerun 为 `data/ik_benchmark/grasp_start_filter_full13_retry_20260714/full13.rrd`。
- 留给下个 AI：本轮困难任务实际参与抽离的候选数已提高到 18～25 个量级；速度瓶颈仍集中在部分顶吸任务的 loaded/最终阶段，而不是 IK 或抓取起点过滤。

## 2026-07-14 Codex / MOTION-52 / 抽离RRT分界点与IK后备候选优化
- 做了什么：为箱体位姿 RRT 增加 endpoint 质量评分，使 parent/best-first 搜索不只按低维距离选点；为抽离入口 IK 增加“主候选 + h 分层后备候选”机制，默认保留低代价主候选并追加后备候选，配合抽离成功 quorum 早停。
- 改了哪里：`robot_motion_core` 的 `box_pose_extract_rrt`；`alfa_robot_moveit_config` 的 `box_pose_rrt_extract_planner`、`dual_arm_planner_node`、`dual_arm_planner.launch.py`、`extract_sequence_rerun.py`、`extract_stage_monitor_console.py`。
- 验证结果：`colcon build --packages-select robot_motion_core alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`colcon test --packages-select robot_motion_core` 通过；13组全流程在全 h 扫描 + shortcut 负重规划下全部成功，RRD：`data/ik_benchmark/rrt_endpoint_reserve_rerun_20260714/full13_endpoint_reserve.rrd`，统计：`data/ik_benchmark/rrt_endpoint_reserve_rerun_20260714/stats.csv`。
- 留给下个 AI：后备候选会提升鲁棒性，但 L6/R13、L11/R8 等任务抽离耗时仍在 2.8～3.1s，后续可继续优化 RRT 采样/goal bias/任务特化先验。

## 2026-07-14 Codex / MOTION-52 / 抽离候选后备池交错调度
- 做了什么：在“主候选 + h 分层后备候选”的基础上新增交错派发顺序，默认每 4 个低代价主候选插入 1 个后备候选，避免困难任务必须等前 25 个候选全部消耗后才尝试多样化 h 候选。
- 改了哪里：`dual_arm_planner_node.cpp` 的 `apply_extract_ik_candidate_limit`；`dual_arm_planner.launch.py`、`extract_sequence_rerun.py`、`extract_stage_monitor_console.py` 增加 `extract_ik_candidate_reserve_interleave_stride` 参数。
- 验证结果：编译通过；13 组全流程 `13/13` 成功。对比 `rrt_endpoint_reserve_rerun_20260714`，交错 stride=4 的总耗时从 `33100.2ms` 降到 `31520.1ms`，平均从 `2546.2ms` 降到 `2424.6ms`；证据：`data/ik_benchmark/rrt_reserve_interleave_full13_20260714/stats.csv`。
- 留给下个 AI：stride=2 也 `13/13` 成功且最大单任务略低，但总耗时 `32299.5ms`，默认暂不采用；后续若更关注最坏耗时而不是总耗时，可重新评估默认值。

## 2026-07-14 Codex / MOTION-52 / 最终回放构建与算法计时拆分
- 做了什么：将 `extract_monitor` 最终阶段的 Rerun 回放轨迹构建从算法耗时中拆出；无 Rerun 统计时不再重算最终回放记录，避免把可视化重建时间误计为任务规划时间。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `extract_monitor_build_final_replay` 开关；`dual_arm_planner.launch.py`、`extract_sequence_rerun.py`、`extract_stage_monitor_console.py` 同步参数，`extract_sequence_rerun.py --no-rerun` 默认关闭最终回放构建。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；13 组全流程 `13/13` 成功。交错候选方案在关闭回放构建后总耗时 `28083.3ms`、平均 `2160.3ms`，`final_ms` 总和从 `4483.5ms` 降到 `44.0ms`；证据：`data/ik_benchmark/no_replay_interleave_full13_20260714/stats.csv`。
- 留给下个 AI：该修改只影响无 Rerun 统计口径，不影响需要生成 Rerun 时的最终方案回放；剩余瓶颈仍是抽离 RRT（L6/R13、L11/R8 约 2.8～3.1s）和部分 loaded 规划（最高约 2.38s）。

## 2026-07-14 Codex / MOTION-52 / 抽离RRT候选扩展实验参数化
- 做了什么：尝试将箱体 RRT parent 选择从纯 nearest 扩展为 nearest + 低密度/低代价父节点补充，并尝试按“IK 代价 + 负重姿态距离”重排抽离入口候选；两类策略均保留为可调参数，但实测不作为默认启用。同步复测抽离成功 quorum=1/2/3 的速度与成功率。
- 改了哪里：`robot_motion_core/box_pose_extract_rrt` 增加 `parent_diverse_candidate_count`、`parent_node_score_weight`、`parent_density_weight`；`dual_arm_planner_node` 增加 `extract_ik_loaded_distance_order_weight` 并透传到 launch 与两个实验脚本。默认仍为保守 q=3，新增策略默认关闭。
- 验证结果：`colcon build --packages-select robot_motion_core alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`colcon test --packages-select robot_motion_core` 通过。13 组全流程均为 13/13 成功。当前 q=3 复测统计：`data/ik_benchmark/current_q3_full13_20260714/stats.csv`，总耗时 `27589.6ms`、平均 `2122.3ms`。q=1 可降低抽离耗时和最大任务耗时，但会提高 loaded 阶段耗时，默认暂不采用；证据：`data/ik_benchmark/default_q1_full13_20260714/stats.csv`。
- 留给下个 AI：RRT parent density/diverse 与 loaded-distance IK 重排在本轮权重下未带来整体收益，不能默认开启。真正有效但有副作用的是降低 extract success quorum；后续若继续优化，应做“抽离候选早停 + loaded 质量阈值”的闭环，而不是单独改 parent 选择或单独改 IK 排序。

## 2026-07-14 Codex / MOTION-52 / 抽离候选质量阈值早停
- 做了什么：在抽离候选并行阶段增加“成功数量 + 负重距离质量”双条件早停；达到最小成功数后，如果该候选到负重姿态的关节距离足够小则提前停止，否则继续到原成功 quorum，避免固定 q=1 带来的负重规划变慢风险。
- 改了哪里：`extract_monitor_state.*` 支持自定义候选停止条件；`dual_arm_planner_node.cpp` 增加 `extract_benchmark_extract_quality_success_quorum` 与 `extract_benchmark_extract_quality_loaded_distance_sum`；launch 与 `extract_sequence_rerun.py`、`extract_stage_monitor_console.py` 同步参数。默认关闭，不改变既有行为。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；13 组全流程按 `quality_success_quorum=1`、`quality_loaded_distance_sum=5.0` 复跑 `13/13` 成功，总耗时 `26086.8ms`、平均 `2006.7ms`，优于当前 q=3 基线 `27589.6ms`、平均 `2122.3ms`；证据：`data/ik_benchmark/quality_stop_1of3_d5_full13_20260714/stats.csv`。
- 留给下个 AI：`distance<=4.0` 过保守，总耗时 `29817.5ms`，不应采用；`distance<=5.0` 有整体收益但仍受 RRT 随机波动影响，建议作为实验/验收参数显式打开，而不是直接替代默认 q=3。

## 2026-07-14 Codex / MOTION-52 / 箱体RRT边验证缓存与启发式实验
- 做了什么：定位抽离慢点为箱体 RRT 大量重复边验证；在单臂箱体 RRT evaluator 层增加 from/to 边缓存，避免 shortcut、path_cost 和重复候选边反复执行解析 IK + 碰撞检测。同时增加 `best_first_first` / `top_best_first_first` 实验开关验证启发式格点搜索先行策略。
- 改了哪里：`box_pose_rrt_extract_planner.cpp` 增加边验证缓存；`robot_motion_core/box_pose_extract_rrt` 增加 best-first 先行配置；`dual_arm_planner.launch.py`、`extract_sequence_rerun.py`、`extract_stage_monitor_console.py` 暴露 `paths_per_arm/path_pair_limit/best_first_first/top_best_first_first` 参数。
- 验证结果：两包构建与定向测试通过。13 组全流程在边缓存 + `quality_success_quorum=1` + `quality_loaded_distance_sum=5.0` 下 `13/13` 成功，总耗时 `22869.4ms`、平均 `1759.2ms`，优于无缓存 d5 的 `26086.8ms`、当前 q=3 基线的 `27589.6ms`；证据：`data/ik_benchmark/edge_cache_q1d5_full13_20260714/stats.csv`。
- 留给下个 AI：`paths_per_arm=2`、全局 best-first 先行、top-only best-first 先行均实测变慢，不要默认启用；保留为实验参数。当前慢项转移到 loaded 阶段，L21/R23 仍约 `3.36s`，其中 loaded 约 `2.36s`。

## 2026-07-14 Codex / MOTION-52 / 优先使用自研局部RRT修补负重段
- 做了什么：将负重段 shortcut 局部修补顺序改为先尝试自研关节空间局部 RRT，再回退 MoveIt/direct local planner；同时把 loaded 候选排序与“首个成功即停”参数从实验脚本透传到 planner，避免脚本参数被硬编码吞掉。
- 改了哪里：`extract_monitor_transition_planning.cpp` 的 `repair_with_local_rrt`；`extract_sequence_rerun.py`、`extract_stage_monitor_console.py` 的 loaded 策略参数。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config robot_motion_core --symlink-install --cmake-args -DBUILD_TESTING=OFF` 通过；`colcon test --packages-select robot_motion_core alfa_robot_moveit_config --event-handlers console_direct+ --return-code-on-test-failure` 通过；三轮 13 组全流程共 `39/39` 成功，统计：`data/ik_benchmark/custom_first_limit8_repeat3_20260714/stats.csv`。平均任务耗时 `1138.5ms`，最大 `2742.8ms`；loaded 阶段平均 `263.1ms`、最大 `362.4ms`，按 13 组折算约 `3420.4ms`，相比 `edge_cache_q1d5_full13_20260714` 的 `11631.5ms` 明显下降，尾部慢点已从 loaded 转移回抽离阶段。
- 留给下个 AI：当前主要慢项是 L6/R13、L11/R8 的抽离阶段，三轮平均分别约 `2297.2ms`、`2133.5ms`；继续优化应聚焦箱体抽离 RRT 的采样/目标偏置/任务先验，而不是 loaded 规划。

## 2026-07-14 运控 / Codex / 顶吸抽离允许并优先微旋转
- 做了什么：修复顶吸箱体位姿 RRT 在 pitch=0 仅 lift 脱离后提前成功的问题，新增 top_goal_min_pitch 软阈值，默认 5°；顶吸目标姿态现在会随 box-state pitch 旋转，并在候选评分中优先更接近侧吸姿态的 top 路径。
- 改了哪里：`robot_motion_core/box_pose_extract_rrt.*`、`alfa_robot_moveit_config/src/box_pose_rrt_extract_planner.cpp`、`dual_arm_planner.launch.py`、`extract_sequence_rerun.py`。
- 验证结果：`colcon build --packages-select robot_motion_core alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select robot_motion_core` 通过；13 组任务复跑 13/13 成功，Rerun 保存到 `data/ik_benchmark/top_pitch_min5_20260714/full13_top_pitch_min5.rrd`。
- 留给下个 AI：如需要更激进顶吸旋转，可通过 `extract_box_pose_rrt_top_goal_min_pitch_deg` 或脚本参数 `--extract-box-pose-rrt-top-goal-min-pitch-deg` 调整；默认 5°只是防止 0°提前收敛，不是要求转到 90°。

## 2026-07-14 运控 / Codex / 吸附前预接触与抽离平滑
- 做了什么：吸附前回放从“初始/负重→吸附 IK”拆成“初始/负重→预接触→吸附 IK”；预接触点按左右末端本地 -Z 方向后退 5cm 重新求 IK。抽离 RRT 输出后增加 shortcut 平滑，平滑段按 5° 密采样并复用双臂附着箱/场景碰撞检查。
- 改了哪里：`extract_monitor_replay_builder.*`、`dual_arm_planner_node.cpp`、`box_pose_rrt_extract_planner.cpp`，并同步 `check_l6_r8_real_safety.py` 的当前负重姿态期望。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test --packages-select robot_motion_core alfa_robot_moveit_config --event-handlers console_direct+ --return-code-on-test-failure` 通过；13 组全流程 `13/13` 成功，Rerun：`data/ik_benchmark/extract_pre_contact_full13_v2/full13_pre_contact_20260714.rrd`，统计：`data/ik_benchmark/extract_pre_contact_full13_v2/stats_20260714.csv`。
- 留给下个 AI：当前顶吸任务尾部耗时波动仍主要来自 loaded 规划和 final replay 构建；若继续优化，优先看 `L11/R13`、`L16/R18` 的 loaded/final 阶段，而不是 IK。

## 2026-07-15 运控 / Codex / 生产执行接口与孪生消费者对齐
- 做了什么：将本地仿真孪生执行入口对齐工控机生产接口：机械臂/turn 使用 `/dual_arm_trajectory_controller/follow_joint_trajectory` action 与 `/dual_arm_trajectory_controller/joint_trajectory` topic，updown 使用 `/canopen/updown_position_controller/commands`；孪生内部按 250Hz 插值、`/joint_states` 默认 50Hz 发布，并保留旧 `/alfa_execution/execute_joint_trajectory` 兼容 action。
- 改了哪里：`robot_motion_runtime/kinematic_sim_executor_node.py` 新增真实接口消费、真实/模型关节名 alias、250Hz 控制循环；`execute_trajectory_service_node.py` 默认转发生产 action、10Hz 重采样，并把 `rightjoint*/leftjoint*` 转为生产 13 轴顺序 `right_joint1..6,left_joint1..6,turn`；`sim_bringup.launch.py`、`runtime_services.launch.py`、`runtime_full_stack.launch.py`、`dual_arm_planner` 默认 action 同步；`execution_trajectory_adapter.cpp` 与测试同步修正右臂在前顺序。
- 验证结果：`python3 -m py_compile` 通过；`colcon build --packages-select robot_motion_runtime alfa_robot_moveit_config --symlink-install` 通过；`alfa_robot_moveit_config` 16/16 测试通过；`robot_motion_runtime` 6/6 测试在 `ROS_LOG_DIR=/tmp/alfa_robot_ros_logs` 下通过。
- 留给下个 AI：真实 13 轴 action 不能承载 `updown` 轨迹；当前执行 service 若发现输入轨迹里 `updown` 变化会拒绝转发，避免静默丢掉 updown。后续若要同步执行 updown，需要单独设计“13轴 action + updown topic”的多执行器协调层。

## 2026-07-15 运控 / Codex / 工控机13任务正式接口测试入口
- 做了什么：将工控机 `/home/ar/lhy_dev` 清理为最小测试工作区并同步当前算法/运行时包；修正测试入口，不再使用旧 `run_l6_r8_task.sh` 单任务 wrapper 或旧 box-id smoke 作为主入口，改为完整 13 组 `RunDualGraspTask` 端点任务序列。
- 改了哪里：远端 `/home/ar/lhy_dev/run_13_dual_grasp_tasks.sh` 调用 `/robot_motion/run_dual_grasp_task`，发送左右末端 `position` 与 `grasp_mode` 字段；远端 `/home/ar/lhy_dev/run_l6_r8_task.sh` 改为弃用提示；远端 README 已说明算法进程、任务进程和 13 组序列。同步 `dual_grasp_task_adapter_node.py`，使 `fixed_updown` 从当前 `/joint_states` 读取，不再硬编码 0.3。
- 验证结果：远端 `/home/ar/lhy_dev/build_lhy_dev.sh` 构建 9 个包通过；`/home/ar/lhy_dev/run_13_dual_grasp_tasks.sh --list` 正确打印 13 组端点任务；`run_l6_r8_task.sh` 退出并提示改用 13 任务入口。
- 留给下个 AI：测试时先启动 `/home/ar/robot_driver` 硬件 bringup，再开 `/home/ar/lhy_dev/run_algorithm_stack.sh`，最后用 `/home/ar/lhy_dev/run_13_dual_grasp_tasks.sh --dry-run` 或 `--execute --yes-execute`。当前没有独立障碍发布者，默认场景为空；若要真实碰撞环境，需要补 `/robot_motion/set_scene` 或 world-model 发布。

## 2026-07-15 电控/运控接口 / Codex / 13任务实机执行入口修正
- 做了什么：定位 `/home/ar/lhy_dev/run_13_dual_grasp_tasks.sh --execute` 实际仍走 `robot_motion_runtime` 的 Python placeholder service，导致 `PlanExtract collision_checked_shortcut` 失败且不是 C++ 全流程算法；将工控机执行入口改为 planner-live 后端，执行时直接调用 `execute_l6_r8_real_live.py` 启动 C++ `dual_arm_planner` 生成 snapshot 再发真实控制器 action。
- 改了哪里：本地 `ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py`；远端 `/home/ar/lhy_dev/scripts/send_dual_grasp_sequence.py`、`/home/ar/lhy_dev/ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py`；远端补齐 `/home/ar/lhy_dev/ros2_ws/src/alfa_robot_execution_bridge/` 作为方向映射来源。
- 验证结果：本地 py_compile 与 L6/R8 快照离线 flatten 通过，保留 C++ 时间戳并对零时长抽离 keyframe 按 10Hz、20deg/s、0.05m/s 补时间；远端 py_compile、`--help`、`--only 4 --list` 和假 planner execute 分支通过，未触发真实运动。
- 留给下个 AI：真实执行入口现在默认保留 planner 的 `time_from_start`，不再二次压成固定短时长；updown 可通过 `/canopen/updown_position_controller/commands` 同步发布，但仍是”13轴 action + updown topic”的协调执行，不是单一硬实时多轴控制器。

## 2026-07-16 运控/电控 / Claude / planner-live 实机执行路径安全加固（Refs MOTION-52，commit d962212）
- 做了什么：接替前运控/电控工程师后系统性核查 `execute_l6_r8_mock_live.py`/`send_dual_grasp_sequence.py` 的实机安全护栏，发现硬上限（`--max-joint-speed-deg-s`/`--hz`/`--max-updown-speed-m-s`/`--loaded-preferred-pose-index`/方向映射）此前依赖 wrapper 层把关，脚本本身可被绕过；`require_explicit_confirmation()` 在非 tty 环境下可能被 `EOFError` 静默穿透当作已确认；`execute_trajectory_service_node.py` 的 `velocity_scale`/`acceleration_scale` 越界值被静默接受、不生效但调用方无感知。
- 改了哪里：`execute_l6_r8_mock_live.py` 五项硬上限改为脚本级 `SystemExit` 拒绝启动 + `ALFA_ALLOW_UNSAFE_*_OVERRIDE` 环境变量护栏；`require_explicit_confirmation()` 非 tty 直接拒绝；`execute_trajectory_service_node.py` 增加 `velocity_scale`/`acceleration_scale` 范围校验及不生效告警、`resample_rate_hz` 校验及与 10Hz 契约不一致告警；`box_pair_task_adapter_node.py`/`dual_grasp_task_adapter_node.py` 改用新增的 `common.py::clamp_motion_scale()`；`check_l6_r8_real_safety.py` 补充对 `send_dual_grasp_sequence.py` 的静态扫描；重写 `docs/ethercat/REAL_DIRECTION_SAFETY.md` 反映 planner-live 是当前默认实机执行路径。
- 验证结果：`python3 scripts/safety/check_l6_r8_real_safety.py` 通过；`py_compile` 全部改动文件通过；`colcon build --packages-select robot_motion_runtime alfa_robot_moveit_config robot_motion_interfaces robot_motion_core alfa_robot_execution_bridge` 通过。全程只改本地仓库，未做任何真机操作。
- 留给下个 AI：这些硬上限只在本地仓库 `execute_l6_r8_mock_live.py` 里生效；工控机 `~/lhy_dev` 上的副本若要同步这批改动，需要单独确认后再做，不要自动同步。

## 2026-07-16 运控 / Claude / model<->hardware joint 命名转换收敛到 common.py（Refs MOTION-52，commit deb30d0）
- 做了什么：全仓库扫描发现 `execute_trajectory_service_node.py`、`kinematic_sim_executor_node.py` 各自维护了一份 model 命名（`leftjoint1`...）↔硬件命名（`left_joint1`...）转换字典，写法不完全一致，存在”改一处漏改另一处”的隐患。
- 改了哪里：`common.py` 新增 `HARDWARE_TO_MODEL_JOINT_ALIASES`/`MODEL_TO_HARDWARE_JOINT_ALIASES`/`REAL_ARM_JOINT_NAMES`/`canonical_joint_name()`/`hardware_joint_name()` 作为唯一实现；两个下游节点删除各自的重复字典，改为从 `common.py` 导入。方向/顺序的唯一权威源仍是 `alfa_robot_execution_bridge.joints`，本次不涉及方向值改动。
- 验证结果：`py_compile` 通过；`colcon build --packages-select robot_motion_runtime alfa_robot_moveit_config` 通过；手动 import 烟雾测试确认新增符号可被两个下游节点正确导入使用。
- 留给下个 AI：以后如果还有节点需要 model/hardware joint 命名转换，直接从 `common.py` 导入，不要再新建本地字典。

## 2026-07-16 运控 / Claude / 收紧附着箱侧壁校验并稳定候选选择（Refs MOTION-52，commit 9a016fd）
- 做了什么：确认此前遗留在工作区、尚未写入日志的 L6/R13、L11/R8 末端诡异旋转/侧壁碰撞修复（详见交接文档记录的排查过程），补写日志条目并正式提交。根因是负重轨迹完整校验里集装箱侧壁/顶板 AABB 只走可选开关，默认未强制失败，而动态箱墙默认强制，两者口径不一致。
- 改了哪里：`dual_arm_planner_node.cpp` 新增 `carried_box_clear_container_obstacles()`，`state_clear_in_full_scene()` 无条件检查所有 `attached_boxes` 的集装箱侧壁/顶/底 AABB；`loaded_pose_planning.cpp` 候选排序与最终样本选择改为综合 `ik_score + distance`；`extract_monitor_state.cpp` 并行抽离早停增加 `best_ik_score` 跟踪，只有代价接近最优的成功样本才能触发 quorum 早停。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --cmake-args -DBUILD_TESTING=ON` 通过；`colcon test` 16/16 通过；`L6/R13`/`L11/R8` 复跑（`data/ik_benchmark/sidewall_fix_20260715/L6R13_L11R8_final/`）均成功选中低腕分支（约 4.7°/0°，不再是 175°/180°），container collisions = 0。
- 留给下个 AI：这批改动在写入本条日志之前已经在工作区停留过一段时间，是通过交接文档 `/tmp/handoff-alfa-motion-20260716-*.md` 补记的，不是当天新做的修复；今后修复完成后应尽量当场写日志，避免依赖临时交接文档补记。

## 2026-07-16 运控 / Claude / model joint state 归一化与固定频率轨迹重采样（来源不明确，commit fd2e2d7）
- 做了什么：提交前系统性核查工作区未提交改动时，发现 `common.py`/`motion_state_source_node.py`/`plan_extract_service_node.py`/`plan_loaded_service_node.py`/两个 launch 文件/`motion_collision_service_node.cpp` 等一批改动，既不属于当天的安全加固/命名收敛工作，也不属于最近一次侧壁修复，日志里也没有对应条目，来源不明确。已重新验证通过后按既定原则一并提交，不再单独溯源。
- 改了哪里：`common.py` 新增 `model_joint_name()`/`normalize_joint_state_for_model()`（硬件 joint state 归一化为 model 命名+固定顺序）、`make_fixed_rate_interpolated_trajectory()`（按固定频率而非固定总时长插值）；`motion_state_source_node.py` 发布归一化后的 `/robot_motion/model_joint_states`；`plan_extract_service_node.py`/`plan_loaded_service_node.py` 默认路径切到固定频率插值（`trajectory_rate_hz` 默认 10.0，与执行链路节奏一致）；`motion_collision_service_node.cpp` 的 `joint_state_topic` 改为可配置参数。
- 验证结果：`colcon build`/`colcon test` 全部通过（moveit_config 16/16，robot_motion_runtime 8/8）；手动 `rclpy` 实例化 `MotionStateSourceNode`/`PlanExtractServiceNode`/`PlanLoadedServiceNode`/`ExecuteTrajectoryServiceNode` 四个节点均正常启动。
- 留给下个 AI：这批改动来源不明确，如果后续发现行为异常，优先怀疑这里；`trajectory_duration_s`/`trajectory_rate_hz` 两套插值路径同时存在（`duration_s > 0` 走旧路径，否则走新路径），注意不要重复实现第三套。

## 2026-07-17 运控 / Claude / 集装箱相对车体动态位姿建模，支持集装箱壳 OBB（Refs MOTION-72）
- 做了什么：MOTION-72 目标2/3（碰撞几何改为全局系写入车体当时位姿转换、车体移动后碰撞场景刷新机制）此前 3 次尝试（`d273471` 占位节点、`9776d6f` 可选全局系写入、`93a65b8` 改为漂移检测告警）都停在"只告警不刷新"，`container_center_x/y` 仍是硬编码相对偏移，从未随车体位姿变化。这次实现"给定车体在 map 下的位姿 + 集装箱在 map 下的位姿（两者仍是模拟/参数化输入，不接入真实导航），计算集装箱相对车体的动态位姿"，替代硬编码静态值；同时发现车体停靠角度不可能总是对准集装箱，只做平移量映射会把朝向误差藏起来，因此把碰撞几何管线从只支持轴对齐 box 升级为支持绕 Z 轴旋转（yaw）的 OBB。范围排查确认：机械臂末端抓的箱子（`AttachedBoxSpec`）已经是由末端 FK 现算的真动态几何，不需要改；箱堆背景表（`make_boxes()`/`StaticBoxObstacle`/`BoxWallGeometryConfig`，5x5 固定编号）按用户确认不参与旋转，只改集装箱壳（`ContainerPanel`，3 块墙板）。
- 改了哪里：`task_geometry.hpp/.cpp` 新增 `OrientedBox` + `aabb_overlaps_oriented_box()`（Z 轴独立分离判断 + X-Y 平面 2D 分离轴测试，退化到 yaw=0 时与原 `aabb_overlaps` 数值一致）；`scene_geometry.hpp/.cpp` 给 `ContainerGeometryConfig`/`ContainerPanel` 加 `yaw` 字段，`make_container_panels()` 按 yaw 旋转墙板中心，`carried_box_clear_obstacles()` 按 `panel.yaw` 选择 AABB 或 OBB 判定路径，新增 `compute_container_pose_relative_to_vehicle()`（车体 map 位姿 + 集装箱 map 绝对位姿 -> 集装箱相对车体的 x/y/yaw）；`motion_scene_adapter.hpp/.cpp` 的 `makeCollisionObject()` 支持 yaw 写入 MoveIt Pose 四元数，新增 `updateContainerGeometry()` 供运行期刷新；`dual_arm_planner_node.cpp` 新增 `container_pose_dynamic_`/`container_pose_map_x/y/yaw_` 参数、`refresh_dynamic_container_geometry()`（查询 TF 有节流日志副作用，非 const）+ `container_pose_dynamic_cache_` 缓存（`container_geometry_config()` 只读缓存，保持 const，不牵连 `carried_box_clear_scene_obstacles` 等一大批既有 const 碰撞检测热路径函数），`check_static_obstacles_drift()` 在动态模式下从"只告警"改为自动重新 `apply_container_obstacles()`；`dual_arm_planner.launch.py` 暴露对应新参数。
- 验证结果：`colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config`（`-DBUILD_TESTING=ON`）通过；`test_scene_geometry`/`test_task_geometry` 新增用例（yaw 旋转墙板中心正确性、`aabb_overlaps_oriented_box` 旋转 45°典型重叠/不重叠/Z轴分离场景、带 yaw 的 `carried_box_clear_obstacles`、`compute_container_pose_relative_to_vehicle` 两组手算验证含车体 yaw=90° 场景）全部通过；`alfa_robot_moveit_config` 既有 15 个测试（除与本次改动无关、历史遗留失败的 `check_l6_r8_real_safety` 方向表快照哨兵）全部通过，确认无回归。完整 `dual_arm_planner.launch.py` 端到端 launch 烟测未能跑通——本机环境下 `ros2_control` controller spawner 反复超时失败（`torso_controller`/`dual_arm_controller` 均报 `Failed loading`/`already loaded`），60 秒内未到达 `DualArmPlannerNode ready`，确认与本次代码改动无关（构造该节点前置的 MoveIt/controller_manager 链路本身在这台机器上不稳定），未继续深挖。
- 留给下个 AI：功能默认关闭（`container_pose_dynamic` 默认 `false`），不影响任何现有行为；启用后集装箱位姿来自 `container_pose_map_x/y/yaw` 参数（模拟"已知的 map 绝对位姿"）和现有 `lookup_vehicle_pose()`（`vehicle_drift_global_frame_ -> container_frame_`，即 `map -> world`，语义上就是车体在map下的位姿，本次未新增 TF 查询代码）。真实导航接口接入时只需替换 `vehicle_pose_source_node.py` 的数据源和/或把 `container_pose_map_x/y/yaw` 换成真实感知输出，下游 `compute_container_pose_relative_to_vehicle()`/OBB 碰撞判定逻辑不需要变。完整 launch 烟测因环境 controller_manager 不稳定未跑通，建议下个 AI 在更干净的环境下补一次，或者用不依赖完整 MoveIt 栈的更轻量方式验证 `apply_container_obstacles()` 实际写入 PlanningScene 的 pose。

## 2026-07-18 运控 / Codex / 实机任务分段确认执行
- 做了什么：将 planner-live 执行从整条轨迹一次发送改为人工确认的分段流程：当前位置先回初始化负重姿态，自动到 IK 前 5cm 预接触点；人工确认 `updown` 到位后前进 5cm 到吸附点；再次确认后执行抽离并回到负重姿态。交互确认从输入 `YES` 改为直接按回车，命令行 `--yes-execute` 总安全护栏保留。
- 改了哪里：`execute_l6_r8_mock_live.py` 增加阶段切分、阶段边界连续处理、分段 action 发送、updown 到位读回提示和回车确认；同步安全检查、测试及 `docs/ethercat/REAL_DIRECTION_SAFETY.md`。
- 验证结果：本地构建与定向测试通过；本地完整 mock 通过。工控机 `/home/ar/lhy_dev` 已同步并完成完整 mock：算法 `2143.27ms`，三段执行实际耗时分别 `3.448s`、`0.616s`、`12.745s`，Rerun 为 `/home/ar/lhy_dev/data/ik_benchmark/codex_segmented_mock_remote/L6_R8_segmented.rrd`。未发送任何真实硬件轨迹。
- 留给下个 AI：实机仍使用 `run_13_dual_grasp_tasks.sh --only 4 --execute --yes-execute` 启动；运行过程中只需按回车确认。`updown` 仍是独立 PP 位置命令，预接触处必须以人工确认/读回误差作为进入 5cm 吸附动作的门槛。

## 2026-07-19 运控 / Codex / 全流程追加放货往返
- 做了什么：在“IK→抽离→负重”之后追加“负重→放货→负重初始姿态”。放货目标严格复用 `jog_to_pose.py` 的 ROS/URDF 语义：双臂 `[0,-55,-50,-60,0,0]°`、`updown=0.20m`；去程携带两个附着箱，放货后回程移除附着箱。两段复用 `ExtractMonitorTransitionPlanner`，先验证直连 shortcut，仅碰撞区间调用局部 RRT 修补。初始化首段同时读取当前 updown 物理反馈、按 `joints.py` 转为逻辑值，并与双臂同步运动到独立负重高度 `0.30m`；不再复用当前 IK 参考高度。
- 改了哪里：`dual_arm_planner_node.cpp` 增加可参数化放货循环及快照阶段；`dual_arm_planner.launch.py`、`extract_stage_monitor_console.py` 透传参数；`execute_l6_r8_mock_live.py` 将执行拆为抽离回负重、去放货、确认释放、回负重；同步安全哨兵、测试和文档。
- 验证结果：本地构建通过，`alfa_robot_moveit_config` 17/17 测试通过，完整 mock 成功；本地 Rerun：`data/ik_benchmark/place_cycle_mock/L6_R8_place_cycle.rrd`。工控机 `/home/ar/lhy_dev` clean build 和两轮完整 mock 成功，最新 Rerun：`/home/ar/lhy_dev/data/ik_benchmark/home_updown_place_cycle_mock/L6_R8.rrd`；放货去程/回程均为无碰撞直连 `joint_interpolation`，各 8.5s，去程附着箱数量 2、回程 0。未发送真实硬件轨迹。
- 留给下个 AI：实机到达放货姿态后会等待回车确认箱子已经释放，再返回负重姿态；当前尚未自动调用真空阀服务。若接入电磁阀，应在该确认边界插入关闭阀门并等待反馈，不要把阀动作埋进轨迹插值循环。

## 2026-07-19 运控/电控接口 / Codex / updown 零偏移合同实机校正
- 做了什么：根据实机复测“logical 0.28 经旧合同下发 physical 0.20，整机高度比 URDF/FK 低约 8cm”，将 `joints.py` 的 updown 零点偏移从 `0.08m` 校正为 `0.0m`；合同转换函数继续强制使用，但现在 logical 与 physical 数值相同。逻辑/物理行程同步统一为 `[0,0.7]m`。
- 改了哪里：统一更新 execution bridge 合同和测试、description URDF、MoveIt joint limits、IK h 默认范围、loaded 抬升上限、runtime adapter/launch、实验脚本、基线与标定文档；历史 `/home/ar/lhy_dev/run_move_all_joints_abs.sh` 改为 canonical `jog_to_pose.py` 兼容入口，彻底停止调用会绕过 `joints.py` 的旧重复实现。
- 验证结果：本地四个受影响包构建通过，execution bridge 5/5、motion core 2/2、moveit config 17/17、runtime 8/8 测试通过；本地完整 L6/R8 mock 成功。工控机五包 clean build、合同测试 5/5、完整 mock 成功，Rerun：`/home/ar/lhy_dev/data/ik_benchmark/updown_zero_offset_mock/L6_R8.rrd`。使用现场历史命令入口做不带 `--send` 的 dry-run，明确输出 `logical=0.2800m -> physical=0.2800m`。全程未发送真实硬件命令。
- 留给下个 AI：不要删除 `logical_to_physical_updown()` / `physical_to_logical_updown()`；零偏移不是“无合同”。工控机 description clean build出现一次 xacro underlay 依赖扫描警告，但运行时 `ros2 pkg prefix alfa_robot_description` 指向 `/home/ar/lhy_dev`，实际展开的 updown limit 已确认是 `[0,0.7]`。
## 2026-07-19 运控 / Codex / 负重候选首成功协作早停
- 做了什么：修复并行负重规划虽配置 `stop_on_first_success` 仍等待全部 worker `join` 的问题；现在以最先完成的成功候选为胜出者，并向其余 shortcut、自研局部 RRT、后挡墙修补流程传播协作取消信号。默认开启，仍可用 `--no-loaded-stop-on-first-success` 恢复“全部完成后选最低代价”。
- 改了哪里：`extract_monitor_transition_planning.*` 增加取消检查；`loaded_pose_planning.*` 增加首成功 winner 与协作取消；planner launch、13组序列、阶段监控和实况执行入口默认开启首成功早停。
- 验证结果：`alfa_robot_moveit_config` 构建通过，17/17 定向测试通过。`h=[0,0.7]` 的13组复测保持 10/13 成功；10个成功任务负重阶段平均从 695.3ms 降到 298.1ms，最大从 1629.3ms 降到 504.9ms。诊断 Rerun：`data/ik_benchmark/updown_0_0p7_full13_20260719/diagnostics/rear_guard_conflicts_L16_R23_L21_R18.rrd`、`data/ik_benchmark/updown_0_0p7_full13_20260719/diagnostics/loaded_wall_conflicts_L11_R8.rrd`。
- 留给下个 AI：无成功候选的任务（当前 L11/R8）无法触发早停，仍会跑完8个候选；首成功模式优化计算延迟，但不再保证从全部成功轨迹中选择总代价最低者。

## 2026-07-20 运控 / Codex / 不等高双侧吸矮侧双层脱离判据
- 做了什么：将双侧吸细分为等高、左高、右高；混合侧吸/顶吸先换算为双侧吸接触位姿，再按有效高度分类。不等高任务要求矮侧附着箱除原侧邻箱判据外，还必须离开原箱位和正上方一层箱位的 x-z 侧面投影；等高任务保持原规则。
- 改了哪里：`robot_motion_interfaces` 增加策略类型与 `front_clearance_levels`；`robot_motion_runtime/dual_grasp_strategy.py` 和适配节点负责分类；`robot_motion_scene_service` 提供同源双层几何判据；`dual_arm_planner_node.cpp` 将判据接入箱体位姿 RRT 目标检查和最终校验；策略文档与测试同步更新。
- 验证结果：隔离构建通过，`robot_motion_scene_service` 2/2、`alfa_robot_moveit_config` 17/17、`robot_motion_runtime` 15/15，共 34/34 测试通过。分类烟测确认 L1/R3=`(1,1)`、L1/R8=`(1,2)`、L6/R3=`(2,1)`。代表性全流程中 L6/R3 成功，总计 17.02s（抽离 16.40s）；L11/R8 抽离成功但负重规划失败，总计 135.35s（抽离 132.82s）。证据：`/tmp/alfa_unequal_front_clearance_targeted_v3_20260720/sequence_20260720_121552/`。
- 留给下个 AI：抽离早停只停止后续候选派发，不能中断已在运行的箱体 RRT；质量阈值不满足时还需累计 3 个成功。新双层目标显著提高困难任务搜索量，下一步性能优化应给抽离 RRT 增加协作取消或针对矮侧双层目标加入更强目标偏置，不能放宽本次脱离安全语义。

## 2026-07-19 运控 / Codex / MOTION-64 双抓取6D合同与任务策略
- 做了什么：把整机双抓取入口收紧为 `request_id + 左右6D接触位姿/吸附模式 + execute`；按模式和目标高度分类双侧吸、等高双顶吸、左高双顶吸、右高双顶吸、混合降级五个内部策略；不等高双顶吸仅要求低侧完全抽离，高侧允许无碰撞的最佳努力路径。
- 改了哪里：新增 `Pose6D`、`GraspTarget`、`DualGraspStrategy`、`ArmExtractPolicy` 合同；分类与混合坐标换算位于 `robot_motion_runtime/dual_grasp_strategy.py`；完整 C++ 箱体位姿 RRT 接入逐臂脱离要求；13任务工具和工控机发送器改用显式目标；详细说明见 `docs/运控/双抓取6D任务合同与策略.md`。
- 验证结果：核心、场景、运行时和完整规划共 39 项测试零失败；ROS interface 与 13任务发送器烟测通过；顶吸7组在全 h 扫描下 7/7 成功，不等高日志确认高侧进入 `collision-free best-effort path`。与前6组侧吸/混合结果合并为 11/13；L6/R3、L11/R8 仍失败在负重规划。结果：`/tmp/alfa_dual_strategy_top_fullh_20260719/sequence_20260719_220512/`、`/tmp/alfa_dual_strategy_full13_20260719_1400/sequence_20260719_215757/`。
- 留给下个 AI：箱号只应继续作为固定实验场景开洞/标签，不得参与策略分类。独立运行时 `PlanExtract` 仍是 shortcut 过渡实现，完整 C++ rollout 尚未迁入该服务；算法验收仍应走 `dual_arm_planner_node` 完整流程。当前抽离仍有明显长尾，L1/R8、L6/R13 分别约 64.9s、85.1s，原因是大量低序候选单臂 RRT 失败后才找到可用候选，不属于本轮线程阻塞。

## 2026-07-20 运控/电控接口 / Codex / updown 四字段动态速度与 PLC 联调恢复
- 做了什么：适配电控侧新的 updown 原子命令合同 `[position_m, velocity_mps, acceleration_mps2, deceleration_mps2]`；示教工具和 planner-live 全流程不再发送已失效的单元素位置命令。全流程按阶段位移/时长动态计算 PP 速度并受 `--max-updown-speed-m-s` 上限约束，每阶段只发送一次最终位置与 profile。恢复 `run_jog_to_pose.sh` 的左右电磁阀、真空泵、运动前后切换和 PLC-only 联调参数。
- 改了哪里：新增 `alfa_robot_execution_bridge/updown.py` 作为四字段构造/校验唯一入口；更新 `jog_to_pose.py`、`execute_l6_r8_mock_live.py`、13任务发送器、运动学孪生消费者、安全哨兵及执行文档。工控机 `/home/ar/lhy_dev` 已备份旧文件到 `/home/ar/lhy_dev/backups/updown_plc_20260720_000049`，同步源码并把 execution bridge 纳入 `build_lhy_dev.sh`。
- 验证结果：本地 execution bridge 单测 7/7、静态实机安全检查、9包构建通过；隔离 ROS smoke 明确拒绝 `[0.2]` 并接受 `[0.2,0.05,0.05,0.05]`。工控机 10 包构建通过，`run_jog_to_pose.sh --plc-only` dry-run、四字段构造、全流程参数转发、13任务列表通过；未写 PLC、未发送真实机械臂命令。上轮 13任务失败状态回放保存到 `data/ik_benchmark/dual_grasp_strategy_contract_20260719/loaded_failures_L6_R3_L11_R8.rrd`。
- 留给下个 AI：电控真实硬件尚未验证新四字段运动；首次实机必须先单独小行程 updown，确认位置/速度/加减速度，再运行全流程。PLC 恢复只接服务调用边界，尚未自动嵌入“到吸附点开阀、到放货点关阀”的任务状态机。

## 2026-07-20 运控 / Codex / 1.5 米集装箱十二箱十任务切换
- 做了什么：将当前固定实验场景由 2.2m 宽、5列5层箱垛切换为 1.5m 宽、3列4层箱垛；`base_link` 对齐净宽中心，箱体尺寸改为深0.30m、宽0.40m、高0.50m；任务序列切换为十组 `1/3、1/6、4/3、4/6、4/9、7/6、7/9、7/12、10/9、10/12`，上两层侧吸、下两层顶吸，混合任务仍降级双侧吸。
- 改了哪里：场景和箱号单一事实源在 `robot_motion_scene_service/motion_core/task_geometry.*`、`scene_geometry.*`；MoveIt 默认容器/附着箱参数、运行时箱号适配、十任务 Rerun 与工控机发送脚本、任务文档同步更新。另修复箱高改为0.5m后暴露的侧吸附着箱局部轴映射错误。
- 验证结果：Release 构建通过，场景、规划器、运行时共34/34测试通过。十组完整流程实测3/10成功；累计算法耗时39.857s，IK仅0.125s，主要耗时在抽离34.655s。Rerun：`data/ik_benchmark/container_1p5m_12box_flow/container_1p5m_12box_10tasks_v2.rrd`；详细表见 `docs/运控/抽离策略实验/2026-07-20_1.5米集装箱十二箱十任务验证.md`。
- 留给下个 AI：当前失败不是 IK 场景过滤误杀；主要为不等高/低位任务的箱体位姿 RRT 无可达抽离路径，以及 L1/R6、L4/R3 抽离后8个负重候选均规划失败。后续优化不能回退已统一的附着箱尺寸或放宽容器/箱墙碰撞口径。

## 2026-07-20 运控 / Codex / 侧吸 0.80m 严格碰撞复测
- 做了什么：撤回未提交的 AABB 碰撞放宽，保持原严格箱墙、集装箱和附着箱碰撞口径，把六组侧吸任务统一改为 `box_front_x=0.80m` 复跑。
- 改了哪里：算法代码未改；纠正实验文档中的已删除 worktree 安装路径，并记录本轮参数和结果。
- 验证结果：Release 全流程 `0/6`；五组失败在抽离，L4/R6 仅有 1 个抽离候选并在严格右侧壁碰撞下负重失败。Rerun：`data/ik_benchmark/container_1p5m_12box_flow/front_x080_strict/front_six_tasks_x080_strict.rrd`。
- 留给下个 AI：`0.80m` 比 `0.925m` 明显更差，主要增加解析 IK 无解、单步 10° 限制和自碰撞；下一轮如继续调距离，应在两者之间做小范围扫描，不要再次放宽碰撞口径。

## 2026-07-20 运控 / Codex / 侧吸 1.00m 严格碰撞复测
- 做了什么：保持严格碰撞与六组侧吸策略不变，仅将 `box_front_x` 增大到 `1.00m` 复跑。
- 改了哪里：算法代码未改；实验数据和文档追加 1.00m 对比结果。
- 验证结果：六组抽离全部成功，全流程 2/6；L1/R3 0.517s、L4/R6 0.747s 成功，其余四组均在负重阶段被动态箱墙/集装箱侧壁约束拒绝。Rerun：`data/ik_benchmark/container_1p5m_12box_flow/front_x100_strict/front_six_tasks_x100_strict.rrd`。
- 留给下个 AI：1.00m 已证明增大距离能解决抽离空间问题；下一步应保持该距离，专门优化不等高侧吸抽离末态到负重姿态的局部路径，不应继续靠放宽碰撞解决。

## 2026-07-20 运控 / Codex / 1.8 米集装箱十五箱五组等高抽离
- 做了什么：取消尚未完成的错位高低箱分阶段策略，切换为 1.8m 宽集装箱、3列5层箱垛和五组等高任务；上两层双侧吸、下三层双顶吸，任务在抽离完成后结束。
- 改了哪里：箱体统一为深0.30m、横宽0.50m、高0.40m；左右末端目标改为 `y=±0.45m`，附着箱保留相对箱面中心5cm横向偏置；场景、MoveIt、runtime 箱号适配、五任务脚本和测试同步更新。顶吸高度窗上限调到0.60m，最底层任务单独使用400次抽离RRT迭代。
- 验证结果：Release 构建通过；场景2/2、策略7/7及五任务定义测试通过。五组 `L1/R3、L4/R6、L7/R9、L10/R12、L13/R15` 抽离成功率5/5，算法耗时分别313.9/314.5/904.7/1630.8/329.8ms。Rerun：`data/ik_benchmark/container_1p8m_15box_flow/five_equal_height_extract_only_final.rrd`。
- 留给下个 AI：本条是中间抽离验收记录；入口默认行为已被下一条完整放置循环更新，旧抽离数据仅用于对比。

## 2026-07-20 运控 / Codex / 十五箱五任务完整放置循环
- 做了什么：将五任务入口从只测 `IK→抽离` 恢复为 `负重初始位→预接触→IK吸附位→抽离→负重位→放置位→回负重位` 完整循环；负重规划默认恢复既定的 `shortcut+碰撞区间局部RRT`，不再让直连无碰撞任务进入纯RRT等待。
- 改了哪里：`extract_sequence_rerun.py` 默认关闭 extract-only、开启放置循环并补齐放置阶段统计；`dual_arm_planner_node.cpp` 将双顶吸放置拆为 `updown降至0.10m→双臂到放置姿态→updown升至最终0.20m→释放`，侧吸维持原0.20m放置过渡；launch和阶段监控透传安全过渡高度。
- 验证结果：Release构建和五任务定义测试通过。五组完整循环5/5成功，算法耗时分别631.9/628.6/1224.5/2003.2/1364.3ms；最终Rerun为 `data/ik_benchmark/container_1p8m_15box_flow/five_equal_height_full_cycle_final.rrd`，详细结果见 `docs/运控/抽离策略实验/2026-07-20_1.8米集装箱十五箱五任务验证.md`。
- 留给下个 AI：顶吸0.10m仅是携箱放置的中间安全高度，最终放置高度仍为0.20m；箱体只在最终放置高度到位后释放，回负重轨迹不附着箱体。`--extract-only`仍可用于单独抽离诊断。

## 2026-07-20 运控 / Codex / 0.8米双布局十任务原样验证
- 做了什么：增加 `centered/right_shift_0p1/both` 布局选择，最终横向合同为居中 `+0.45/-0.45m`、偏差布局 `+0.50/-0.40m`，两套布局均保持附着箱相对末端5cm偏置；侧吸顶吸统一箱墙距离0.80m，放置高度直接使用0.10m。
- 改了哪里：场景核心、MoveIt附着箱、runtime双抓取适配统一横向常量；五任务脚本支持两套布局连续运行；IK默认启用 `[0,0.7]m`、1cm步长全范围扫描，消除此前64点等分产生的非整厘米高度。
- 验证结果：最小复现确认错误 `±0.40m` 会让338个合法IK全部撞 `box_wall_between <-> left_joint6`；恢复 `±0.45m` 后得到89个去重候选、18个抽离成功候选。严格按用户参数运行10个完整任务结果2/10，居中L1/R3和L4/R6成功；Rerun：`data/ik_benchmark/container_1p8m_15box_flow/ten_tasks_centered_right_shift_x080_full_cycle_corrected.rrd`。
- 留给下个 AI：剩余失败已不是“所有候选被附着场景过滤”；主要是0.80m下的顶吸解析无解/抽离RRT无路径，以及偏差布局侧吸的抽离或负重局部路径失败。不要再次把横向目标改回±0.40m。详细记录见 `docs/运控/抽离策略实验/2026-07-20_0.8米双布局十任务验证.md`。
## 2026-07-20 运控 / Codex / 五任务顶吸直升策略
- 做了什么：箱墙前表面改为 x=0.70m，L7/R9 改为侧吸，吸附后负重 updown 改为 0.10m；第四、第五排顶吸新增保持双臂关节不动、updown 固定上升 0.40m 的无 RRT 抽离模式。
- 改了哪里：alfa_robot_moveit_config 的序列脚本、planner 参数/实现、launch、定义测试和实验文档。
- 验证结果：构建通过；五任务全流程成功 2/5，顶吸 L10/R12、L13/R15 均精确上升 0.40m 并完成全循环。Rerun 见 data/ik_benchmark/container_1p8m_15box_flow/five_tasks_x070_loaded010_top_direct_lift040.rrd。
- 留给下个 AI：三组侧吸均已完成抽离但失败在负重规划；本轮按用户要求不继续优化速度或放宽碰撞。
## 2026-07-20 运控 / Codex / 侧吸 0.90 米复测
- 做了什么：五任务默认距离拆分为侧吸 x=0.90m、顶吸 x=0.70m，其他策略保持不变。
- 验证结果：侧吸三任务全流程 L1/R3、L4/R6 成功，L7/R9 抽离失败，成功率 2/3；Rerun 见 data/ik_benchmark/container_1p8m_15box_flow/front_three_tasks_x090_loaded010.rrd。
- 留给下个 AI：顶吸沿用上一版 0.70m 直升 0.40m 的成功结果；本轮未做速度优化。
## 2026-07-20 运控 / Codex / L7/R9 侧吸直升
- 做了什么：L7/R9 保持侧吸 IK 和侧吸附着箱建模，但抽离改为双臂关节不动、updown 固定上升 0.40m；普通侧吸与顶吸任务策略不变。
- 验证结果：L7/R9 完整流程成功，快照确认模式为 front/front + direct_updown_lift，updown 由 0.02m 到 0.42m。Rerun 见 data/ik_benchmark/container_1p8m_15box_flow/L7_R9_front_direct_lift040_x090_loaded010.rrd。
- 留给下个 AI：序列脚本会为 L7/R9 单独重启对应策略的 planner，避免与 L1/R3、L4/R6 的侧吸 RRT 配置混用。
## 2026-07-20 运控 / Codex / 双布局十任务验证
- 验证内容：居中与右偏布局各运行 L1/R3、L4/R6、L7/R9、L10/R12、L13/R15 五组完整流程。
- 验证结果：10/10 全部成功；侧吸 x=0.90m、顶吸 x=0.70m，L7/R9 与两组顶吸均使用 updown 直升 0.40m。
- 证据：data/ik_benchmark/container_1p8m_15box_flow/ten_tasks_both_layouts_front090_top070_direct_lift040_loaded010.rrd；汇总位于 data/ik_benchmark/extract_sequence_rerun/sequence_20260720_222046/。

## 2026-07-22 运控 / Codex / Linear 主线收口与架构任务重组
- 完成收口：`MOTION-63`、`MOTION-65`、`MOTION-70`、`MOTION-71`、`SEV-8` 补验收评论后设为 Done；后续工作不再扩写这些已达标任务。
- 主线调整：`MOTION-69` 升为 High，统一承载 cuRobo 全流程与局部修补；`MOTION-73` 改为零偏移、`[0,0.7]m` 与四字段动态速度最终合同并继续实机验收；`MOTION-68` 保留 tool0 `0.259m` Review；`MOTION-72` 明确剩余全局坐标/烟测范围；`SEV-9` 退回 Todo 等待机械碰撞模型输入。
- 新增 Linear：`MOTION-75` 运控程序架构冻结，`MOTION-76` 运控与整机系统职责及接口边界冻结。仅新增两个可独立验收的大任务，没有为零散算法修复和场景实验拆小 issue。
- 本地协作：T-0037 已完成并移出当前任务表；后续 Git 提交按实际主任务关联，不再引用已经 Done 的 `MOTION-52`、`MOTION-64`。

## 2026-07-25 运控 / Codex / 控制轨迹速度合同与连续定时
- 做了什么：修复下发轨迹只有位置、没有逐帧速度的问题；轨迹改为30Hz固定频率，每帧完整携带13轴位置和速度。
- 算法调整：合并同一直线上的冗余采样点，不改变规划几何路径；真实拐点和阶段边界速度归零，各直线段按30°/s、60°/s²梯形速度重新分布，避免直接位置插值造成速度突变。速度转换只应用关节方向，不叠加J6位置零偏。
- 验证结果：本机和工控机合同测试均19/19通过；A1~A5、B1~B5两端均10/10规划成功，所有下发帧均含13个速度值，实测整形峰值30°/s、60°/s²。工控机只做隔离ROS域计算验证，未下发实机运动。

## 2026-07-25 运控 / Codex / 取消 RRT 中间点整机停车
- 根因：上一版把每个非严格共线RRT点拆成独立梯形段，导致每个中间点13轴同步降为零速；A5/B5各出现27个、B4出现22个此类停车点，其中大量只是小角度方向变化。
- 调整：整段轨迹只在首尾统一加减速；每个关节仅在运动方向正负翻转时归零，同向转折使用相邻斜率生成连续速度，并继续受30°/s、60°/s²约束。updown单独运动时双臂零速保持不变。
- 验证结果：合同测试21/21通过，十任务10/10成功；机械臂运动段内部整机停车降为0，轨迹时长由上一版约20~39s降至约19~27s，每帧最大关节变化1°。代码已同步并构建到工控机，当前运行中的算法线程未被停止，重启后生效。

## 2026-07-27 运控 / Codex / A/B布局末端间距调整为1米
- 做了什么：保持B布局中心0m、A布局中心+0.05m不变，将左右末端间距由0.90m改为1.00m；B目标为+0.50/-0.50m，A目标为+0.55/-0.45m。
- 改了哪里：统一任务入口、运行时策略和场景核心的横向目标；末端现在位于箱体横向中心，因此附着箱相对tool的横向偏移同步由0.05m改为0m，避免附着箱被错误外移后与箱墙立即碰撞。
- 验证结果：场景、MoveIt规划和runtime共52项测试通过；常驻planner严格碰撞复跑A1~A5、B1~B5仍为10/10成功，算法内部平均687.3ms/任务。单次测试数据已按仓库清理要求删除，结论保留在本日志。
- 留给下个 AI：旧的`20260727_161515`结果0/10是附着箱仍保留5cm横向偏移造成的合同错位，不应作为1m间距的真实成功率结论。
## 2026-07-28 运控 / Codex / armmotion 接入 rt-control Domain 42
- 做了什么：连续定位 `/dual_arm_jtc/follow_joint_trajectory` 不可用和“能发现 action、但等待接受超时”两个问题。第一层根因是 armmotion 原来在 Domain 0，而 rt-control 固定为 Domain 42；第二层根因是 root 容器与普通用户宿主进程的 Fast DDS SHM 文件权限不互通，发现通道可见但 action 数据面不通。
- 改了哪里：armmotion 固定沿用 Domain 42，并用独立 Fast DDS UDPv4 profile 禁用宿主进程 SHM；算法线程把硬件检查提前到 planner 冷启动前；规划启动新增外部控制栈模式，只启动 `move_group`/规划节点，不再在真实控制域重复启动本地 `ros2_control`、RSP 和 `/controller_manager`。README 与错误诊断同步更新，代码已部署并构建到 `/home/ar/demostration0720/src/armmotion`。
- 验证结果：本地 MoveIt 构建通过、armmotion 13/13 测试通过。工控机无运动烟测中 planner 6.6s 就绪；控制器列表仅保留 rt-control 的四个 active controller，`/joint_states` 仅一个发布者，action 恰好一个服务端 `/dual_arm_jtc` 和一个客户端 `/armmotion_algorithm_thread`；未发送轨迹或 PLC 命令。
- 留给下个 AI：先按 `robot_system/rt_control/03-rt-control一键启动说明.md` 启动并看到 READY，再启动 armmotion；不要移除 UDP profile，也不要在 Domain 42 启动第二套 ros2_control。工控机旧 `/home/ar/lhy_dev/run_jog_to_pose.sh` 仍是旧控制器/旧方向转换语义，不得接到新 `/dual_arm_jtc`。

## 2026-07-28 运控 / Codex / 实机轴向合同恢复
- 做了什么：先补齐强制公共边界，armmotion 与 `jog_to_pose` 的全部14轴位置、速度、加速度和反馈均显式调用 `joint.py` 的 `model_to_rt_control_*` / `rt_control_to_model_position`，禁止只读取顺序后直接拼装消息。随后根据实机双臂同角度测试恢复原始四个反向轴：`left_joint3/5`、`right_joint2/4`；命令、速度、加速度和反馈使用同一方向表，J6公共边界偏置保持0。
- 验证结果：本地及工控机合同与armmotion合计24/24通过并完成构建；工控机旧算法线程必须重启后才加载新模块。
- 重要边界：`/home/ar/lhy_dev/run_rt_jog_to_pose.sh` 会把独立 Python 客户端复制进 rt-control 容器直接发送，完全不读取 armmotion 的 `joint.py`。该工具的同角度测试证明 rt-control 当前全轴 `direction: 1` 并未形成对称模型语义；它不能用于验证本次上层合同是否生效，除非后续也改为读取同一合同。

## 2026-07-30 运控 / Codex / CSP 同步放置与解析 IK 性能恢复
- 做了什么：将携箱负重位到放置位改为 updown 与双臂同一条 13 轴轨迹；直连失败时使用同时推进双臂和 updown 的安全中间状态，不再先完成升降再单独运动双臂。Rerun 默认只记录算法输出的 30Hz 原始轨迹，不再生成 250Hz 插值和 90Hz 抽样副本。
- 根因：上一份 Rerun 误走独立序列脚本默认 `loaded_updown=0.10m`，没有使用算法线程的 `loaded_updown=0.45m + preserve_lower` 合同；本地解析 IK 和 MoveIt 包又曾在空 `CMAKE_BUILD_TYPE` 下编译，导致闭式解析核失去 `-O3`，性能退化约两个数量级。
- 验证结果：Release 重编后解析 IK 回归循环（含 FK 校验）10000 次约 0.09s；十任务 10/10 成功，IK 阶段平均 21.5ms，完整计算平均 1.883s。所有任务的放置轨迹从第一个非起点帧开始同时改变 updown 和双臂。Rerun：`data/ik_benchmark/csp_sync_place_raw_rerun/ten_tasks_exact_raw_20260730.rrd`。
- 留给下个 AI：手工构建这两个 C++ 包必须显式使用 `-DCMAKE_BUILD_TYPE=Release`；不要再用独立 `extract_sequence_rerun.py` 的默认参数替代 `PlannerAdapter` 生成当前算法线程验收 Rerun。

## 2026-07-31 运控 / Codex / 2.2米顶板碰撞与失败回放修复
- 做了什么：定位“负重后 updown 单独下降”和“2.2m 顶板全部误判失败”。前者是独立序列脚本把负重高度错误覆盖为 `0.1m`，且旧负重 shortcut 明确先完成双臂再补 updown；后者是 MoveIt/FCL 已判定无碰撞后，又被旋转箱体的保守 AABB 二次硬拒绝。
- 调整：序列默认负重高度恢复 `0.3m`；负重过渡改为13轴同步 shortcut、碰撞段局部RRT，顶吸受后挡墙约束时才回退为双臂独立求路并行合并后调整 updown；精确 FCL 保持硬门槛，保守容器 AABB 仅在显式严格模式下启用。最终阶段失败也会写入当前 snapshot，并把参考轨迹截到首个无效帧供 Rerun 回放。
- 验证结果：Release 构建通过，场景与规划共19项测试通过；`L1/R3` 在2.2m顶板下完整成功约0.72s，负重段 updown 与双臂同步；人为把放置高度设为0.7m时，Rerun 正确停在第20个碰撞帧。A/B侧吸6组及顶吸4组分模式复测均成功；全量脚本跨模式第6次 planner 重启仍偶发等待服务卡住，属于现有生命周期问题，不是规划失败。

## 2026-07-31 运控 / Codex / 负重高度下调至0.1米
- 做了什么：按现场回放观察将抽离后负重目标由 `0.3m` 下调为 `0.1m`，并修复放置后回负重仍错误引用抓取搜索基准 `fixed_updown=0.3m` 的双重口径；回程现在直接返回本次已选负重状态。
- 验证结果：Release 构建通过，场景与规划共19项测试通过；`L1/R3` 完整流程成功，算法耗时708.8ms。抽离后负重、负重到放置、放置回负重末态均为 `0.1m`；携带箱在负重轨迹中距2.2m顶板最小约0.192m。Rerun：`data/ik_benchmark/pose_driven_collision_scene/L1_R3_ceiling22_loaded01_final.rrd`。
- 留给下个 AI：抓取 IK 搜索基准 `fixed_updown=0.3m` 未改，它不是负重目标。严格验证器默认额外要求3cm抽离余量会报差约4mm；按当前合同“与原箱侧面无重叠”使用 `--margin 0` 验证通过。

## 2026-07-31 运控 / Codex / 附着箱顶板碰撞漏检修复
- 根因：`state_clear_in_full_scene()` 名义上检查完整场景，实际把 `dual_arm_with_base` 作为 `CollisionRequest.group_name`；MoveIt 组过滤遗漏了附着在 tool0 的箱体与世界顶板碰撞。同一帧组内检查为 clear，而无组过滤的 FCL 明确返回左右附着箱同时碰撞 `container_ceiling`。
- 调整：所有 shortcut、局部RRT及最终轨迹边验证改用无组过滤的完整 PlanningScene/FCL；失败诊断同步检查完整场景；快照验证新增附着箱穿透顶板门槛。
- 验证结果：旧“成功”快照被准确定位为 `selected_loaded_to_place` 第22点开始穿顶，最深约0.077m。修复后 `L1/R3` 不再假成功，局部RRT未找到绕顶路径时正确失败并回放到首个碰撞帧。失败Rerun：`data/ik_benchmark/pose_driven_collision_scene/L1_R3_ceiling22_loaded01_full_scene_fixed.rrd`。
- 留给下个 AI：本轮解决的是碰撞漏检，不是放置路径可达性；下一步若要求该任务成功，应优化负重到放置的中间状态或让13轴局部RRT支持双臂联合绕顶，禁止重新放宽完整场景碰撞检查。

## 2026-07-31 运控 / Codex / 负重姿态 joint2 下调至负90度
- 做了什么：保持 `shortcut + 碰撞区间局部RRT` 不变，仅将左右默认负重姿态由 `[0,-45,120,-75,0,0]°` 改为 `[0,-90,120,-75,0,0]°`；预抓取姿态不变，实机安全护栏同步锁定新负重位。
- 验证结果：Release 构建和 `alfa_robot_moveit_config` 17/17 测试通过。当前五任务双布局共10次服务口径为6/10成功：侧吸相关任务全部成功，双顶吸 `L10/R12`、`L13/R15` 两种布局均失败在抽离后负重规划，主要为携带箱与后挡墙/顶板冲突。完整Rerun：`data/ik_benchmark/loaded_joint2_minus90_full10/full10_shortcut_rrt.rrd`。
- 后验说明：6 个服务成功样本在真实几何口径下全部通过；其中 4 个样本实际保留约 25.7mm 间隙。验证器默认额外净空已由 30mm 修正为 0，仅在显式传入 `--margin` 时检查附加安全裕量。

## 2026-08-01 运控 / Codex / 顶吸抬升同步后抽
- 调整：双顶吸抽离由“仅抬升 updown 0.40m”改为 updown 与双臂同步运动；默认抬升 0.40m、双末端沿世界 x 负方向后抽 0.30m，解析 IK 逐步求解，每条步进边使用完整 PlanningScene/FCL 检查。
- 参数扫描：后抽 0.20m 时四组仍全部卡负重过渡；0.30m 时 `L10/R12` 两种布局完整成功；0.35m 的 `L13/R15` 仍擦后挡墙；0.40m 会在抽离第12步发生携带箱与 `base_link` 碰撞，因此默认固定为0.30m。
- 验证：Release 构建及19项测试通过；四组顶吸结果2/4成功，成功样本严格后验2/2通过。Rerun：`data/ik_benchmark/top_lift_retreat_sync_final/top4_lift040_retreat030.rrd`。

## 2026-08-01 运控 / Codex / 原负重经预放置的二十任务复测
- 做了什么：完整流程恢复原负重姿态 `[0,-45,120,-75,0,0]°`，新增预放置姿态 `[0,-90,120,-75,0,0]°`，按“抽离→负重→预放置→放置→释放→空载返程”执行；预放置关键帧精确保留，放置后同时清理 PlanningScene 和 RobotState 的附着箱。
- 验证结果：当前正式 A1..A5/B1..B5 十任务连续运行两轮，共20次全部成功；算法内部平均1173.0ms，最慢1934.2ms，最大相邻关节变化4.779°，主要阶段边界位置跳变为0。Rerun：`data/ik_benchmark/pre_place_full20_verified/full20_loaded_pre_place_place.rrd`。

## 2026-08-01 运控 / Codex / 抽离回放统一限速
- 根因：抽离求解保存的是一串单状态快照，独立 Rerun 入口没有经过算法线程执行适配器的全阶段重定时；顶吸每步约28.6mm、侧吸局部最大约4.8°，按等帧播放时明显快于其他阶段。
- 调整：普通抽离、回放重建和顶吸直升都改为保存“上一状态→当前状态”的真实轨迹段，并统一按10Hz、关节20°/s、updown 0.15m/s重采样；快照显式保留每段不同的起止状态。
- 验证结果：构建和4项定向测试通过；A/B十任务连续两轮20/20成功。抽离相邻帧最大关节变化由4.779°降至2.000°，updown由28.57mm降至15.00mm，其他阶段速度分布未改变；算法平均耗时1173.0ms→1185.0ms。Rerun：`data/ik_benchmark/pre_place_full20_uniform_speed/full20_uniform_speed.rrd`。

## 2026-08-01 运控 / Codex / 40厘米方形箱十三任务首轮验证
- 调整：集装箱保持不变，箱体统一为沿车方向0.30m、横向0.40m、高0.40m，三列中心改为 `+0.40/0/-0.40m`；任务序列改为13组高低组合。前两层沿用侧吸抽离，任一箱编号不小于7时沿用下三层抬升策略；一侧吸一顶吸仍按既有规则降级为双侧吸，未增加强行成功的补救路径。
- 验证：Release构建通过，场景/规划/运行时定向测试13项通过。先跑居中布局13组，完整成功7/13；6组均已通过IK和抽离，失败发生在抽离后负重过渡，主要为携带箱与动态左右箱墙或后挡墙冲突，局部RRT未修补成功。
- 数据：平均完整计算1.954s；成功任务平均1.322s，失败任务平均2.692s，失败时间主要消耗在负重规划（平均2.330s）。Rerun：`data/ik_benchmark/box_040x040_uneven13/centered_13_tasks.rrd`；统计：`data/ik_benchmark/box_040x040_uneven13/sequence_20260801_182721/summary.json`。
- 后续：正式全集还包括同一13组的5cm偏移布局，共26组；本轮按要求只跑居中13组，先保留真实失败用于诊断。

## 2026-08-01 运控 / Codex / 不等高 updown 同步低臂抽离
- 调整：算法层继续只根据左右6D接触位姿和吸附模式分类；不等高且使用 updown 的任务中，升降轴抬升时矮侧六轴同步抽离。矮侧先尝试纯上抬，无有效解析IK或发生碰撞时再尝试斜上后退、仅后退；脱离判据只相对高 `0.40m` 的参考箱，不再同时要求离开矮箱原位。
- 修复：长驻 planner 原先把首任务解析后的 `auto` 模式写回配置，导致后续任务错误复用同一抽离模式；现已分离请求模式和本次有效模式，每次配置都按当前6D位姿重新分类。
- 验证：Release构建通过，场景、运行时和规划定向测试20项通过。居中13组完整成功9/13，`L4/R9` 由抽离0成功提升为完整成功；`L7/R12` 三种矮侧首步方向均无有效解析IK，另外三组通过抽离后失败于负重规划的后挡墙/顶板碰撞修补。
- 数据：Rerun：`data/ik_benchmark/box_040x040_uneven13_low_arm_sync_final/centered_13_tasks_low_arm_sync.rrd`；统计：`data/ik_benchmark/box_040x040_uneven13_low_arm_sync_final/sequence_20260801_191435/summary.json`。

## 2026-08-01 运控 / Codex / 逐臂吸附模式与抬升后半箱后抽
- 修复：五排实验任务严格按第1～3排侧吸、第4～5排顶吸逐臂生成；删除一侧吸一顶吸自动降级为双侧吸的入口。算法仍由两个6D接触位姿恢复箱体中心高度做分类，但不再改写任一侧接触位姿或吸附模式。
- 调整：所有使用 `updown` 的抽离在竖直阶段完成后固定升降轴，再向车侧后抽半个箱深 `0.15m`。每步优先纯后抽；解析IK第一分支不可用时枚举至多8个解析分支，必要时允许每步0/3/6mm上让和±1°/±3°姿态微调，随后统一经过整机、附着箱、动态箱墙和集装箱完整场景碰撞检查。
- 验证：Release构建通过，场景/运行时/规划定向测试22项通过。居中13组完整成功9/13；全部成功的updown任务快照均达到 `post_lift_retreat_x=0.15m`。混合任务 `L7/R12=(front,top_suction)`、`L10/R9=(top_suction,front)` 均保持真实模式，但分别在后抽第3步触发左臂 `joint2↔joint5`、右臂 `joint2↔joint4` 自碰撞而失败；另两组失败在负重规划。
- 数据：Rerun：`data/ik_benchmark/box_040x040_mixed_modes_post_retreat_verified/centered_13_tasks.rrd`；统计：`data/ik_benchmark/box_040x040_mixed_modes_post_retreat_verified/sequence_20260801_210734/summary.json`。

## 2026-08-01 运控 / Codex / Motion 域首版 Docker 与五域迁移桥
- 边界：按 `robot_system_docs/拆垛机器人五域接口规范` 首次落地 M-02、M-03、M-04、M-05、M-06 名称和 IDL 快照；Motion 只通过完整 14 轴 FJT 和真空 Action 访问 RT-Control，不管理 rt-control 生命周期。
- 实现：新增 Motion 域 Action 服务、双箱 6D 位姿合同校验、任务去重放和串行仲裁；通过显式迁移桥把五域目标 wire 映射到现行 `/dual_arm_jtc` 与 PLC 接口。
- Docker：使用纯净 ROS/MoveIt 基础镜像、只读源码挂载和容器启动 Release 增量构建；实机 Domain 42、host network、Fast DDS UDP-only，避免 root 容器与普通用户的 SHM 权限故障。
- 验证：容器 Mock 闭环已跑通 M-02 接近→吸附→抽离和 M-03 转运→释放→撤离；合同测试 20/20 通过，容器增量构建约 2s，planner 启动约 6.7s。修复了容器 HOME 无可写 ROS 日志目录，以及宿主 `install` 泄漏造成的 MoveIt ABI 混用。开发容器、本机 Mock 和实机迁移模式统一由 `tools/motion_domain_docker.sh` 管理。
- 限制：尚未接入 Gate/Safety/M-07/版本准入；现行 PLC 只有 `vacuum_established` 布尔量，不等价于五域规范的新鲜表压 `<= -50 kPa` 验收。

## 2026-08-02 运控 / Codex / 删除不等高顶吸重复抬升
- 根因：抽离阶段已经完成 updown 抬升和半箱后抽，负重规划器仍按“不等高双顶吸”无条件追加固定 `+0.4m` 抬升前缀，使箱体再次升至约 `0.7m`，引入顶板/后挡墙碰撞。
- 调整：删除负重规划器的固定前置抬升策略、配置回调和节点残留状态；负重规划现在直接以真实抽离终态为起点，后续仅允许碰撞修补逻辑按实际失败原因采取动作。
- 验证：Release 构建及17项测试通过；`L10/R15`、`L13/R12` 首次完整回放均成功，严格快照后验2/2通过，负重段 updown 分别从 `0.4807m`、`0.5107m` 单调降至 `0.1m`。追加3轮稳定性复测共6/6成功；Rerun：`data/ik_benchmark/pre_loaded_top_lift_removed/L10_R15_L13_R12_after_fix.rrd`。

## 2026-08-02 运控 / Codex / 顶吸有界分层抽离 RRT
- 根因：自动顶吸仍绕过箱体位姿 RRT，固定抬升目标还可能超过 `updown` 上限后被 MoveIt 截断，造成回代误差；旧脱离判据只比较自身原箱位的粗 AABB。
- 调整：顶吸统一进入有界箱体位姿 RRT；先按当前 `updown` 与 `[0,0.7]m` 上限执行公共抬升，再依次偏好末端上抬、顺时针俯仰（最多90°）、向车侧后退。脱离判据改为侧视四边形 SAT；不等高任务的矮箱以高箱原位侧面为参考。
- 验证：Release 构建通过，`robot_motion_core`、`alfa_robot_moveit_config`、`robot_motion_runtime` 共38项测试全部通过。六组顶吸抽离 `L7/R12、L10/R9、L10/R12、L10/R15、L13/R12、L13/R15` 全部成功，所有回放状态 `updown<=0.700m`；本轮无失败样本。Rerun：`data/ik_benchmark/top_priority_rrt_final/top_suction_6_tasks_final.rrd`。
- 限制：本轮按需求只验收顶吸 IK+抽离，没有复跑抽离后负重、预放置、放置的完整循环。

## 2026-08-02 运控 / Codex / 顶吸六任务完整流程验证
- 验证：将有界分层顶吸 RRT 接入完整流程，连续运行 `L7/R12、L10/R9、L10/R12、L10/R15、L13/R12、L13/R15`；六组均完成负重初始位、预接触、吸附、抽离、负重规划、预放置、放置和返回负重位，快照阶段均为 `full_selected`，成功率6/6。
- 性能：六组算法总耗时分别为 `2590.4、2600.5、1682.1、1729.2、1794.3、1607.2ms`，平均 `2000.6ms`；前两组主要增加在抽离（约0.98s），各组负重规划约1.37～1.44s。
- 数据：Rerun：`data/ik_benchmark/top_priority_rrt_full_flow/top_suction_6_tasks_full_flow.rrd`；统计：`data/ik_benchmark/top_priority_rrt_full_flow/sequence_20260802_192032/summary.json`。

## 2026-08-02 运控 / Codex / 十三任务合并前稳定性验收
- 根因：负重规划默认只保留排序前8个抽离候选，`L10/R9` 的稳定可规划候选通常位于第7～10名，随机局部修补会造成同一任务约2/5成功。
- 调整：保留8个并行规划工作线程，但取消候选总数默认硬截断；首批全部失败后才继续下一批，仍在获得第一个成功结果时早停。
- 验证：`L10/R9` 连续5轮5/5成功；居中13组完整流程13/13成功。十三组算法耗时为 `0.775～11.658s`，慢任务仍主要受抽离候选搜索影响。统计：`data/ik_benchmark/premerge_full13_batched_fallback/sequence_20260802_193815/summary.json`。

## 2026-08-02 运控 / Codex / 正面中心6D任务合同收口
- 做了什么：正式任务和双线程演示统一只发送左右箱体正面中心 `pose_6d`；算法按五排标称高度最近邻和 `±0.12m` 容差识别排数，内部决定侧吸/顶吸、工具接触位姿和抽离策略，实际 `x/y/z/RPY` 偏差原样进入 IK 与动态场景。
- 改了哪里：`RunDualGraspTask` 删除外部吸附模式字段；任务线程消息删除任务编号、箱号和距离；旧箱号 adapter 改为默认不启动。旧 C++ monitor 所需整数固定为内部槽位标签，不来自任务输入、不参与策略或几何。
- 根因修复：首轮显式位姿 B1 负重规划失败并非动态墙差异，而是正面姿态绕吸附法向误翻 180°，使 IK 进入完全不同腕部分支。标称姿态已恢复为 RPY `(π,-π/2,0)`；标称显式墙和旧箱号网格墙新增完全同源回归。
- 验证结果：Release 相关包构建通过；场景2项、MoveIt17项、runtime23项、armmotion25项全部通过。相同长驻 planner 下旧 B1 与新6D B1均约0.75s成功，新6D B4完整成功约1.62s。默认完整栈实测11个显式进程、13个ROS图节点、14个话题、11个业务服务。
- 留给下个 AI：本轮已合并到 `v5_dev`；旧 `/robot_motion/run_box_pair_task` 只有显式设置 `enable_legacy_box_pair_task_adapter:=true` 才会出现。

## 2026-08-03 运控 / Codex / 清理退出主线的旧 ROS 包
- 做了什么：删除当前正式运控、Motion Docker 和 rt-control 迁移链均不再使用的 `bio_ik`、`alfa_robot_bringup`、`alfa_robot_hardware` 三个源码包。
- 改了哪里：解析 IK 默认值、包边界测试、description 的 mock 硬件默认值、AI 协作入口和系统架构门户同步去除旧包；真实硬件和生命周期明确归外部 rt-control 域。
- 验证结果：从全新临时 build/install 目录完成 9 包 Release 构建；xacro 与 `check_urdf` 通过；解析 IK、core、scene、MoveIt 和 runtime 共45项测试通过。
- 留给下个 AI：`scripts/ik_benchmark` 和历史文档仍可能提到 BioIK，仅作为历史实验记录，不属于正式构建依赖；旧仓库内 real-hardware/bringup 启动方式不再提供。

## 2026-08-03 运控 / Codex / V3 七轴双臂外观与碰撞模型接入
- 做了什么：接入 `robot_v3.0.1_visual` 高精度外观网格，保留 `robot_v3.0.1` 低面数碰撞网格；右臂由左臂严格镜像生成，关节骨架采用外观版 URDF，补齐双臂第7轴、控制器、SRDF 和 MoveIt KDL 配置。
- 模型口径：外观与碰撞网格分目录管理；碰撞低模逐链接刚体配准到外观模型。左右肩部轴线共点 RMS 约 `0.740mm`，腕部 Joint5/6/7 共点 RMS 约 `0.000439mm`，20组随机姿态镜像 FK 最大位置误差约 `5.03e-9m`。
- 验证结果：7包 Release 构建通过；description `44/44` 测试通过；MoveIt Demo 正常启动且14轴臂控制器激活；左右单臂非零目标 FK→KDL IK 回归均成功，位置回代误差低于 `2e-8m`。
- 清理：删除已退出主线的 `bio_ik`、`alfa_robot_hardware`、`alfa_robot_bringup` 残留 build/install 与忽略文件，避免插件扫描污染。
- 跟踪：创建 Linear `MOTION-94`；本轮只验收模型、控制器、Demo 与数值 IK，旧解析 IK 和完整抓取流程尚未适配 V3 七轴结构。

## 2026-08-03 运控 / Codex / V3.0.2 双臂模型对比分支
- 做了什么：在独立试验分支将 V3.0.1 双套网格替换为 `robot_v3.0.2` 单套网格；visual 与 collision 严格使用同一文件，关节链同步采用 V3.0.2 源 URDF，右臂由左臂镜像生成。
- 模型检查：源 URDF 可被 `check_urdf` 正常解析；腕部 Joint5/6/7 轴线共点 RMS 约 `0.138mm`；100组随机七轴姿态的左右镜像 FK 位置和旋转误差均为0。
- 验证结果：7包 Release 构建通过，description `44/44` 测试通过；MoveIt Demo 正常启动，左右单臂非零目标 FK→KDL IK 均成功，位置回代误差约 `1.9e-7m`。
- 分支边界：该提交用于 V3.0.1 与 V3.0.2 外观、关节结构和 Demo 对比，未宣称完整抓取算法已经适配。
## 2026-08-06 运控 / Codex / V3.0.2 整机单实例模型 Demo
- 做了什么：基于既有 V3.0.2 双臂分支建立独立 worktree `alfa_robot_v3`，新增整机模型和 MoveIt 规划 Demo 的构建、启动入口。
- 改了哪里：模型 RViz 只保留一个整机 `RobotModel`；MoveIt RViz 隐藏 Planning Request 起点和 Planned Path 的机器人外观，目标臂采用半透明预览并保留末端六自由度交互控制器，消除无意义的多套整机显示。
- 验证结果：V3.0.2 左右各七轴安装到 updown 两侧；description 构建、URDF 树检查通过；MoveGroup、控制器和 RViz 正常启动，实际窗口确认只显示一套整机及一个当前规划组的末端控制球。
- 留给下个 AI：本分支仅验证 V3.0.2 整机模型展示与 MoveIt 手动规划；旧结构解析 IK 和完整任务算法尚未迁移。

## 2026-08-07 运控 / Codex / 修复 V3 MoveIt 目标预览与碰撞初始位
- 根因：为消除重复模型曾把 MoveIt 目标状态透明度设为0，连带隐藏了拖动末端球后的目标臂和红色碰撞提示；同时 `initial_positions.yaml` 未接入整机 URDF 自带的 mock `ros2_control`，实际启动仍为全零碰撞姿态。
- 调整：目标臂改为0.65半透明，仅继续隐藏起点和规划路径模型；筛选左右镜像无碰撞初始位并接入真实 xacro 参数链，同步 MoveIt YAML、SRDF `home` 和纯模型 Demo。
- 验证：MoveIt 确认全零状态存在双臂及立柱碰撞；新姿态本体有效且无接触，±5°扰动120/120通过；末端2cm目标 KDL IK 成功，MoveIt 约14.8ms生成225点轨迹；description 回归测试通过。

## 2026-08-13 运控 / Codex / V3.0.3 立方体侧装双臂演示
- 做了什么：接入 `robot_v3.0.3` 七轴单臂模型，将同一机械臂复制安装到立方体左右侧面；左右 `joint1` 轴分别朝外，取消旧车体、立柱、升降轴和传感器模型。
- 配置：MoveIt 收口为左右单臂和双臂14轴规划组，mock 控制器只暴露14个关节；左右末端继续使用 KDL 数值 IK。
- 验证：description xacro/URDF 测试通过；MoveIt、单控制器和 JointStateBroadcaster 正常启动；全零状态通过 PlanningScene 碰撞检查；左臂非零末端目标 KDL IK 求解成功。
- 范围：本分支仅用于 V3.0.3 安装方向、外观、碰撞和手动 KDL 规划测试，不代表旧抓取全流程已适配。
- 执行修复：14轴共用控制器允许接收单侧7轴部分轨迹；未出现在目标中的另一臂保持当前状态，避免 RViz 单臂 `Plan and Execute` 被控制器拒绝。
- 限位调整：左右双臂统一采用 J1/J3/J5/J7 ±180°、J2/J4 ±105°、J6 ±120°；URDF、ros2_control 与 MoveIt 三套配置保持一致。

## 2026-08-14 运控 / Codex / V3.0.4 立方体侧装双臂演示
- 做了什么：将 `/mnt/mydisk/ALFA/backpack/robot_v3.0.4` 的八段网格、惯性和七轴关节链迁入独立测试分支，继续复用立方体左右侧装、14轴 mock 控制器和左右 KDL IK。
- 限位：J1/J3/J5/J7 ±180°、J2 ±105°、J4 ±150°、J6 ±120°；URDF、ros2_control、MoveIt 和回归测试保持一致。
- 验证：用户已在 MoveIt 窗口确认模型与手动操作正常；全零 PlanningScene 无碰撞，左右非零 FK 目标均可由 KDL 回代；description 与 MoveIt 共19项测试通过。
- 范围：仍只验收模型、碰撞、手动规划和 KDL，不代表旧抓取全流程已适配 V3.0.4。

## 2026-08-15 运控 / Codex / V3.0.4 七轴冗余解析 IK
- 做了什么：在保留旧六轴解析接口的前提下，新增 V3.0.4 球肩—肘—球腕七轴闭式求解器；接口采用 `q=F(T, psi, branch)`，其中 `psi` 为肘部绕肩心—腕心连线的冗余转角，并输出肩/肘/腕离散分支、限位裕量和 FK 误差。
- 改了哪里：新增 `v3_redundant_analytic_ik.hpp/.cpp`、随机回代测试和包内说明；几何常量逐字匹配当前 V3.0.4 xacro，Joint1/2/3、Joint3/4/5、Joint5/6/7 共点关系用于闭式分解，不使用迭代求根或随机 seed。
- 验证结果：Release 下10000组随机 FK→冗余角→IK 全部有解，最大位置回代误差约 `3.33e-8m`、姿态误差约 `4.01e-8rad`，解析核平均约 `12.6us`；与 MoveIt FK 独立交叉验证位置误差约 `2.4e-16m`、姿态误差为数值零；包内21项测试通过。
- 留给下个 AI：当前只提供确定性运动学分支，尚未替换 MoveIt KDL 插件，也不在解析核内做碰撞；下一步应在一维 `psi` 上做少量确定性采样/区间优化，再用统一 PlanningScene 对有限分支做碰撞和连续性筛选。
## 2026-08-17 运控 / Codex / V3 单臂40cm连续可达性扫描
- 做了什么：新增 V3 七轴冗余解析 IK 连续可达性扫描工具；把 `X/Y/Z` 三维体积内每个点作为独立起点，分别沿 `base_link +X` 按1cm连续推进40cm。下一点继承上一点关节解与冗余角参考，并检查10°关节跳变、相邻状态插值碰撞、测试臂自碰撞及与立方体本体碰撞。
- 改了哪里：`alfa_robot_moveit_config` 新增 `v3_continuous_reachability`、对应 launch 和 Rerun 可视化脚本；新增 `docs/运控/IK/V3单臂连续可达性扫描.md`。
- 验证结果：左臂起点范围 `X=0.35～0.75m`（1cm）、`Y=-0.55～0.85m`（5cm）、`Z=0.05～1.20m`（5cm），共28,536个独立起点；6,042个能够完整连续40cm，成功率21.17%。总耗时约23.6s，其中解析IK 2,264,920次累计约12.5s、FCL碰撞检查844,387次累计约7.2s。Rerun以绿色记录6,042个成功起点、红色记录22,494个失败起点，不记录40cm中间轨迹采样点。结果位于 `data/ik_benchmark/v3_continuous_reachability/left_xyz_start_volume_40cm_1cm.json` 和 `.rrd`。
- 留给下个 AI：当前是固定末端姿态 `RPY=[0,90°,0]` 的单臂连续起点体积，默认忽略另一只机械臂；如用于双臂协同或环境障碍评估，应保留“每个XYZ独立完成40cm”的判据并改用完整场景碰撞口径。
- 完整空间补充：发现初版绿点贴住左侧和下方扫描边界后，继续扩展并用零可达切片封闭边界。最终起点体积为 `X=0.15～0.75m`、`Y=-0.20～1.40m`、`Z=-0.75～1.25m`，共82,533点；单点可达50,759点（61.50%，13.385s），完整连续40cm为20,565点（24.92%，60.581s）。绿色实际边界分别到 `Y=1.35m/Z=-0.70m` 和 `Y=1.20m/Z=-0.60m`；结果为 `left_full_point_reachability_xyz.rrd` 与 `left_full_continuity_40cm_xyz.rrd`。
- 局部平移连续性补充：扫描器新增 `planar_disk` 模式；每个中心固定末端姿态，在 `base_link YZ` 平面依次走过半径3/6/9/12/15cm同心环，环向约1cm采样，并沿用相邻关节最大10°和边插值FCL碰撞判据。完整82,533个中心中29,364个通过，成功率35.58%，总耗时275.467s；记录中的最大相邻关节变化9.999°。结果为 `left_full_planar_disk_continuity_r15cm_xyz.json/.rrd`。
- 连续性口径修正：相邻关节变化10°改为高关节增益质量标记，不再直接判不连续；复用2.5°关节边插值同时检查FCL碰撞和末端FK任务路径误差，默认门槛1mm/1°。高增益边只搜索邻近±2档ψ，失败边最多三层二分且额外预算2ms。原红区5490个中心完整通过率由5°ψ旧口径43.64%提升到77.81%，总耗时73.55s；结果为 `left_red_region_continuity_final.json/.rrd`。
- 孤立碰撞回放补充：扫描器支持按需保存失败中心从圆盘起点到失败前的完整关节/目标轨迹；可视化自动筛选某一网格轴两侧中心均成功、当前中心因碰撞失败的案例并串行播放。当前共172段，失败帧使用红色包围盒标出MoveIt/FCL报告的碰撞link；结果为 `left_sandwiched_collision_failures_playback.rrd`。
- 安装位置敏感性补充：连续扫描入口支持左右臂沿 `base_link +X` 同步前移，默认偏移仍为0。本次前移10cm后，同一5490点、半径15cm圆盘测试通过率由77.81%升至81.02%（4448点）；恢复515个旧失败中心，同时丢失339个旧成功中心，净增176点。结果为 `left_red_region_mount_forward_10cm.json/.rrd`。

## 2026-08-18 运控 / Codex / V3.0.5 整机双臂 MoveIt 展示接入
- 做了什么：将 `/mnt/mydisk/ALFA/backpack/robot_v3.0.5` 的完整底座、固定升降架和两条七轴链接入 V3 仓库，保留45个上游网格；源整机绕X轴旋转+90°对齐标准Z-up坐标，并交换源左右链语义，使左臂保持在+Y、右臂保持在-Y；J4限位改为±145°。
- 验证结果：URDF解析为19 links、18 joints、14个可动关节；description测试通过，MoveIt/OMPL、双臂控制器和RViz均正常启动；当前状态碰撞校验有效，左臂当前末端KDL IK返回成功。
- 留给下个 AI：本轮 `link_002_joint` 固定在源模型零位，仅用于模型与MoveIt展示；完整运控算法尚未针对3.0.5重新验收。

## 2026-08-18 运控 / Codex / V3.0.5 三项连续可达性复测
- 做了什么：把七轴冗余解析 IK 的固定几何扩展到 V3.0.5 左右臂，并在每个起点用 MoveIt/URDF FK 复核解析候选；沿用完整扫描空间 `X=0.15～0.75m`、`Y=-0.20～1.40m`、`Z=-0.75～1.25m`，完成单点可达、前后40cm连续、周围15cm圆盘连续三项测试。
- 验证结果：共82,533个中心。单点可达51,351个（62.22%，13.00s）；前后40cm连续21,507个（26.06%，157.77s）；周围15cm连续30,753个（37.26%，655.42s）。三份 JSON/RRD 位于 `data/ik_benchmark/v3_0_5_continuous_reachability/`。
- 质量说明：固定末端姿态、左臂、忽略对侧臂，碰撞检查包含测试臂自碰撞及本体；解析 IK 包2项回归测试全部通过。
- 可视化修正：初版 JSON 错把 `arm_carriage` 在 `base_link` 下的绝对 X 坐标 `0.370414616m` 记录成 xacro 的附加安装偏移，Rerun 因而把模型重复前移。扫描数据本身未受影响；现已改为记录真实 launch 参数并重建三份 RRD，单点烟测确认默认偏移为0且目标坐标保持 `[0.5, 0.3, 0.3]`。

## 2026-08-18 运控 / Codex / V3.0.5 升降轴坐标与工具中心修正
- 做了什么：把 `world` 原点放到实体升降轴上，并补偿上游整机模型约 `12.92°` 的固定倾角，使升降轴严格沿世界 Z；同时将左右工具中心从 Joint7 轴心分别外移 `0.1865m`、`0.1895m` 到实际末端面。
- 改了哪里：同步修改 description 固定变换、左右 `tool0`、V3.0.5 七轴解析 IK 的腕心/TCP 几何、连续性扫描坐标说明和 Rerun 最远前向点标记。
- 验证结果：Release 构建、description 与解析 IK 回归通过；同一82,533点空间中，单点可达54,472点（66.00%，14.33s），前向40cm连续30,817点（37.34%，129.05s），周围15cm连续30,826点（37.35%，675.48s）。
- 数据：结果位于 `data/ik_benchmark/v3_0_5_updown_tcp_continuous_reachability/`；`left_reachability_with_forward_farthest.rrd` 以绿色显示可达点、蓝色显示世界 `+X` 最远可达点及对应机械臂姿态。

## 2026-08-18 运控 / Codex / V3.0.5 向内末端朝向可达性
- 做了什么：连续可达性 launch 新增可配置末端 `roll/pitch/yaw`；左臂采用 `RPY=[90°,0°,0°]`，使工具局部 Z 朝世界 `-Y`，即朝右臂方向。
- 验证结果：扩展空间 `X=0.15～1.35m、Y=-0.30～1.40m、Z=-0.75～1.25m` 共173,635点，向内姿态可达47,161点（27.16%），耗时33.89s。
- 数据：`data/ik_benchmark/v3_0_5_inward_reachability/left_inward_minus_y_point_reachability_xyz.json` 与 `left_inward_minus_y_point_reachability.rrd`；本轮仍按单臂口径忽略对侧机械臂，仅检查测试臂自碰撞及本体碰撞。

## 2026-08-20 运控 / Codex / V3冗余解析IK交互Demo
- 做了什么：新增 RViz+Rerun 双窗口交互 Demo；RViz蓝色6D目标球只负责给定末端位姿，算法在 `psi=-180°～180°` 内采样全部合法解析解，Rerun动态播放整个解族。
- 行为：同一 `psi` 的重复解去重；按肩/肘/腕离散分支组织连续段，无解区间或大于20°的关节跳变自动切段。Rerun按相邻解关节变化自适应播放间隔，分支切换明确暂停，不伪装成可执行连续轨迹。
- 验证结果：默认2°分辨率下，每个目标扫描181个 `psi`，得到784个去重合法解和8个连续分支，计算约2.3～2.6ms；末端X移动5cm后Rerun自动从generation 1切换到generation 2。四包Release构建通过，共22项测试0失败。
- 入口：`ros2 launch alfa_robot_moveit_config v3_redundant_ik_interactive_demo.launch.py`；说明见 `docs/运控/IK/V3冗余解析IK交互Demo.md`。

## 2026-08-20 运控 / Codex / V3.0.6 单臂模型镜像双臂接入
- 做了什么：迁入 `/mnt/mydisk/ALFA/backpack/robot_v3.0.6` 单臂模型，以同一关节链镜像生成左右双臂；V3.0.5 的底座、升降架和双臂安装平台保持不变。
- 限位：未采用上游 V3.0.6 的统一 ±90°，继续保持 J1/J3/J5/J7 ±180°、J2 ±105°、J4 ±145°、J6 ±120°，并同步 URDF、ros2_control 和 MoveIt。
- 验证结果：Release 构建通过，共23项测试0失败；默认双臂状态 MoveIt/FCL 有效且无接触，左臂当前末端由 MoveIt KDL IK 精确回代成功。
- 解析与扫描：V3.0.6 仍满足三组轴线共点条件，已加入左右臂冗余闭式模型；17.36万个中心中单点可达37.92%、前伸40cm连续14.98%、周围15cm连续20.59%。冗余Demo默认2°采样得到792个去重解、8个连续分支，约2.27ms。
- 数据：`data/ik_benchmark/v3_0_6_extended_reachability/` 包含三份 JSON/RRD 和冗余解族 RRD。

## 2026-08-21 运控 / Codex / V3.0.7 整机模型接入
- 做了什么：整体迁入 `/mnt/mydisk/ALFA/backpack/robot_v3.0.7` 的45个整机网格、惯性与双侧七轴关节链；保留 `model_base`、`arm_carriage`、`left/right_joint1..7` 和 `tool0` 等既有 ROS/MoveIt 契约。
- 坐标：以 `arm_carriage` 作为工作坐标原点和方向基准；`world -> arm_carriage` 的平移、旋转均为零，整机在 RViz 中保持 Z-up 直立。
- 限位：J1/J3/J5/J7 ±180°、J2 ±105°、J4 ±145°、J6 按本轮要求改为 ±110°；URDF 与 ros2_control 限位一致。
- 验证结果：description 与 MoveIt Release 构建通过，URDF 回归测试通过；MoveGroup、14轴 mock 控制器和 RViz 正常启动；双臂零位 PlanningScene 有效且无碰撞；左臂当前 TCP 的 MoveIt FK→KDL IK 回代成功。
- 留给下个 AI：V3.0.7 当前使用 KDL；现有 V3.0.6 冗余解析 IK 几何常量、连续可达性数据和交互 Demo 结果不可直接视为 V3.0.7 验收，后续需单独重推与复测。

## 2026-08-24 运控 / Codex / V3.0.7 冗余解析与连续可达性复测
- 做了什么：将 V3.0.7 左右七轴固定变换、J6 ±110° 限位和 `tool0` 偏移加入冗余闭式求解器；扫描器和交互 Demo 切换到 V3.0.7 模型。
- 验证结果：Release 构建通过，23项测试0失败。扫描 Y/Z 中点分别对齐左臂 Joint1 的 `-0.2995m/1.3228m` 后，同一173,635点空间中单点可达79,447点（45.76%，298.82s）、前伸40cm连续41,639点（23.98%，766.87s）、周围15cm连续42,764点（24.63%，2013.08s）。默认冗余解族得到760个去重解、8个连续分支，约2.69ms。
- 数据：有效 JSON、日志和三份点云 RRD 位于 `data/ik_benchmark/v3_0_7_joint1_centered_xyz_reachability/`；旧 Z 中心或 Y 中心错误的数据不作为验收结果。
- 留给下个 AI：主要失败是扫描点无解析解，其次为 `arm_carriage`/`model_base` 与左臂连杆碰撞，未放宽碰撞口径。

## 2026-08-24 运控 / Codex / V3.0.8 模型与待测点云预览
- 做了什么：整体迁入 `/mnt/mydisk/ALFA/backpack/robot_v3.0.8`，保留既有左右七轴命名和14轴控制合同；上游额外上部旋转体固定在零位，Joint6 继续采用 ±110°，`tool0` 按新末端实体长度改为 `0.13585m`。
- 解析适配：新增 V3.0.8 左右臂固定几何，1万组随机 FK→IK 回归最大位置误差约 `3.32e-8m`、最大姿态误差约 `4.00e-8rad`。
- 连续可达性：范围为 `X=0.15～1.35m`、`Y=-1.18054221～0.51945779m`、`Z=0.3032993～2.3032993m`，共 `173,635` 个中心；Y/Z 中点对齐左臂 Joint1。单点可达 `79,758`（45.93%，168.73s），前伸40cm连续 `42,026`（24.20%，392.31s），周围15cm连续 `41,835`（24.09%，1577.99s）。
- 数据与清理：三项 JSON/RRD 位于 `data/ik_benchmark/v3_0_8_continuous_reachability/`；按要求删除 V3.0.5～V3.0.7 及更早的历史可达性点云资产。
## 2026-08-25 运控 / Codex / V3基础动作与连续可达性Linear对齐
- 做了什么：确认可达性/连续性由 `MOTION-94` 跟踪并保持 In Review；新建 `MOTION-154` 跟踪单/双臂简单动作测试与演示。
- 改了哪里：Linear `MOTION-94` 更新为 6 点、截止 2026-08-26并补充最新验收证据；`MOTION-154` 设为 In Progress、7点、截止 2026-08-30。
- 验证结果：`MOTION-94` 已记录 V3.0.8 单点、前伸40cm、周围15cm连续性结果；`MOTION-154` 已记录单臂5cm接近、35cm抽离和避障返回原型约251ms。
- 留给下个 AI：不要为同一批可达性/连续性结果重复建 issue；后续基础动作提交关联 `MOTION-154`。

## 2026-08-26 运控 / Codex / V3双臂同步笛卡尔箱体Demo
- 做了什么：完成 `MOTION-154` 的第一个双臂基础动作；双手工具轴相向、间距 `0.40m` 刚性握持 `0.40m` 正方体，用户拖动箱中心后，两臂沿同一空间直线同步运动。
- 算法：每个 `1cm` 笛卡尔采样点分别生成左右七轴冗余解析 IK 候选，再联合选择无碰撞关节对；边验证时左右臂共享同一插值比例，附着箱体作为机器人整体参与 MoveIt/FCL 碰撞检查。
- 入口：`ros2 launch alfa_robot_moveit_config v3_dual_arm_cartesian_box_demo.launch.py`；RViz 青色控制球用于拖动目标箱中心，右键菜单确认计算，Rerun同步显示计算结果。
- 验证：默认后移 `12cm` 成功，13帧、约 `19.2ms`；横移 `8cm` 成功，9帧、约 `14.4ms`；上升 `8cm` 成功，9帧、约 `12.0ms`。Release构建、Python入口语法与RViz/Rerun实际画面均通过。
- 留给下个 AI：当前只实现固定朝向、固定40cm握持间距的同步平移；30cm箱绕前向轴旋转与一横一竖握持仍待用户验收本版后继续。
## 2026-08-26 运控工程师 / Codex / MOTION-154 双臂箱体绕点旋转 Demo
- 做了什么：在既有双臂同步解析平移框架中增加 30cm 箱体绕机器人前向 X 轴的刚性同步旋转模式，并用上一帧冗余角优先采样保持解析分支连续。
- 改了哪里：扩展 `v3_dual_arm_cartesian_box_demo` 核心、RViz 交互、Rerun 回放，新增旋转专用 launch 和运行文档。
- 验证结果：Release 编译通过；默认 +25° 旋转约 23.4ms，用户交互测试正负方向多段旋转成功，超出腕部可达范围时能返回明确失败；原 40cm 箱同步平移回归成功。
- 留给下个 AI：继续 MOTION-154 第三个 Demo——一侧横向、一侧向上握持 30cm 箱体的同步解析平移。

## 2026-08-26 运控工程师 / Codex / MOTION-154 双臂异构握持同步平移 Demo
- 做了什么：完成第三个双臂基础动作；右臂侧向握持30cm箱体，左臂从底面严格向上支撑，用户拖动箱体中心后双臂保持刚性相对位姿同步直线运动。
- 算法：每个1cm箱体直线采样点使用V3.0.8七轴冗余解析IK生成左右候选，联合进行关节连续性、双臂/本体/附着箱碰撞检查；未调用KDL或OMPL。
- 验证结果：严格向上姿态的默认可达箱中心为 `[0.75,0,1.44]m`；沿世界 `-X` 平移10cm成功，11帧、总计算约17.6ms。原双臂平移约22.7ms、绕箱中心25°旋转约23.1ms，回归均成功。
- 入口：`ros2 launch alfa_robot_moveit_config v3_dual_arm_asymmetric_box_demo.launch.py`；说明见 `docs/运控/IK/V3双臂异构握持同步平移Demo.md`。

## 2026-08-29 运控工程师 / Codex / V3.0.9 十六自由度整机模型接入
- 做了什么：接入 `/mnt/mydisk/ALFA/backpack/robot_v3.0.9`，恢复上游 `updown` 升降轴和头部旋转轴；左右七轴、升降和头部共16个可动关节，MoveIt 保留原双臂组并新增含升降、头部和整机规划组。
- 资产处理：V3.0.9 的 URDF 与46个 STL 和 V3.0.8 逐字节一致，因此新增 V3.0.9 语义 xacro，但复用既有 V3.0.8 网格，避免重复存储；双臂解析几何同样复用并提供 V3.0.9 模型枚举。
- 验证结果：Release 构建7个相关包成功；description 1项、解析IK 2项、MoveIt 17项测试全部通过。1万组 FK→IK 最大位置误差约 `3.32e-8m`、最大姿态误差约 `4.00e-8rad`、平均单次约 `13.67us`；动态TF确认 `updown=0.2m` 和 `head_joint=0.5rad` 均真实生效；双臂平移、绕点旋转和异构握持三个Demo无界面烟测均成功。
- 后续修正：用户确认运动方向正确，要求位置范围严格采用 V3.0.9 原始 URDF；已将 `updown` 改为 `-0.1～0.1m`，`head_joint` 改为 `-1.57～1.57rad`，不再沿用旧机器人 `0～0.7m` 升降合同。解析 IK 仍只求单臂七轴，升降和头部由上层规划组管理。
- 最终升降合同：用户进一步确认 V3 实际升降总行程为 `1m`，且当前模型 `updown=0` 对应真实高度 `1m`、`updown=-1m` 对应真实零高度；因此最终规划范围改为 `[-1.0, 0.0]m`，运动方向保持不变。
- 默认初始姿态：用户更正左右臂均为 `J1～J7=[-90,-90,0,-90,0,0,0]°`，共享 `updown=0m`、`head_joint=0°`。已同步 description xacro 默认参数、模型查看器、两套 mock ros2_control、MoveIt initial positions、SRDF `home` 和连续可达性默认 seed。

## 2026-09-10 Codex / V3 Demo Conda 环境隔离修复
- 做了什么：共享 ROS 环境脚本不再只过滤旧机器 Anaconda 路径，同时清理用户默认 Conda 安装和当前激活环境/安装根目录，覆盖 PATH、动态库、pkg-config 和 Python 搜索路径。
- 改了哪里：`tools/ros_humble_env.sh`；新增 `tools/test_ros_humble_env.sh` 回归检查。
- 验证结果：环境检查通过；CMake 选择 `/usr/bin/python3`，`robot_motion_core` 单包构建成功，原 catkin_pkg 报错消失。
- 留给下个 AI：完整 `build_v3_moveit_demo.sh` 仍被缺失 `moveit_core` 阻塞；本机 apt 显示 `ros-humble-moveit` 和 `ros-humble-moveit-core` 均未安装，本轮未安装系统依赖。

## 2026-09-11 Codex / 旧架构单臂 demo 的箱墙前置开发
- 做了什么：用户已向 mentor 确认允许在本地旧架构单臂 demo 开发完整任务；此前“必须等待新架构视频工作树”的阻塞判断失效。同事负责固定距离，我们先做场景/配置/回归，不代替其测量。
- 改了哪里：单臂 demo C++/launch，新增默认参数 YAML 与 `test/test_v3_box_wall_preparation.py`；更新 `docs/运控/IK/V3单臂解析抽箱交互Demo.md`。增加显式 wall_origin 的5×5/1cm场景，目标选择不移动墙，完整16轴输出及配置拒绝；保留旧cross模式与既有未提交改动。未提交、推送、安装软件或控制实机。
- 验证结果：Release 单包构建成功；10项无窗口检查通过（旧单箱四段规划含附着事件、3个固定墙目标、不可达规划拒绝、5个非法配置）；Rerun几何预览RRD生成并通过 `rrd verify`。证据在 `/home/astesia/Sevenova/日志/验收_2026-09-11/old_demo_wall_preparation/`。不是双箱/全墙可达/放置验收。
- 留给下个 AI：RTK引用文件本机缺失；距离需连同基准面/坐标系/初始姿态交付；demo零臂角与NOW的home冲突。优先补目标箱接触前碰撞与 world/attached/released 生命周期，审计全机器人碰撞后再做双箱/共享轴和后放。正式runtime合同、任务issue/验收人未确认，不擅自绑定历史issue或将demo接入生产启动链。

## 2026-09-11 Codex / 距离输入单箱抓取 demo
- 做了什么：按用户新请求在旧单臂抽箱可执行程序上新增 `distance_demo` 模式与独立 `v3_box_wall_grasp_demo.launch.py`，通过 demo-local `PlanWallBoxDemo.srv` 接收 `x/box_id/arm`，每次恢复完整5×5墙后独立规划，支持左右臂/auto。未接硬件、未新增包/外部依赖、未提交或推送。
- 改了哪里：`alfa_robot_moveit_config` 的单臂demo C++、CMake、新srv/launch/test；`.gitignore` 仅为新srv添加例外（原规则忽略所有srv）；复用的Rerun viewer补请求显示/换请求清理；新增 `docs/运控/IK/V3距离输入单箱抓取Demo.md` 并在原文档加入口。保留本轮前已有脏改动。
- 关键实现：车头默认是 model_base 碰撞网格 world 最大X（本模型约0.310000006814m），可显式 `chassis_front_x` 标定；默认墙居中、底面Z=0。目标箱接触前纳入world，最后接触段仅放行tool/joint7；吸附后从world移除并参与负重检测。显式检查整机/有界关节插值/RRT真实起终点连接及固定轴。新模式只需几何轨迹，去掉TOTG重采样，不可下发控制器。RViz携箱随FK运动且停末帧、墙格编号可见；Rerun消费相同结果帧。
- 验证结果：Release两包构建通过；新测试覆盖25箱号固定墙、25个远距离失败、7类非法请求、左右/auto、成功完整路径、16轴固定约束、RViz最终携箱位置与Rerun FK一致、车头标定覆盖。0.30m全墙本次有限搜索找到完整路径的ID为 `[5,6,10,11,15,20,21]`，其余只代表本次未找到，不能证明不可抓取。原10项回归通过；RViz/Rerun实际GUI启动，0.30m/6号完整成功轨迹RRD通过verify且包含最终携箱返回帧。证据 `/home/astesia/Sevenova/日志/验收_2026-09-11/wall_grasp_demo/`。
- 留给下个AI：`success`只代表该模型/固定初态/离散检测/有限预算内完整几何路径，不是全墙都能抓或实机安全证明。新服务同步串行、并发可能排队、无取消；失败 `selected_arm` 为空，JSON side为最后诊断臂，auto失败要查看全部attempts。仍缺实测车头基准/墙摆放/初终态合同，RTK引用文件仍缺；旧零臂角与NOW home不一致，本次刻意保留旧基线。共享轴不搜索，地面和额外环境障碍、动力学/吸附反馈、放置释放、连续拆墙均未实现，正式集成由runtime管理，不把此demo当生产接口。

## 2026-09-11 运控 / Codex / 单箱 demo 关闭终端后 waiting 故障修复
- 做了什么：从用户实际进程 PID339321 命令行确认 `arm:=auto~` 拼写错误；规划节点 PID339324 的 FATAL 为 `arm must be left/right/auto`，但旧 launch 仍保留 RSP/RViz/Rerun。调用端 PID340093 与服务端同为 domain187/localhost1/正确 overlay，本次不是 source/domain 故障。
- 改了哪里：仅新距离 demo launch 增加原生 arm choices 和规划节点 on_exit→Shutdown；不更改 C++、旧单臂 launch、抓取算法或实机接口。新增 `test/test_v3_box_wall_startup.py`；同步 `docs/运控/IK/V3距离输入单箱抓取Demo.md` 和本地 HTML 操作指南。
- 验证结果：config 包构建通过；domain188 隔离回归通过非法 arm 启动前拒绝、非法 x 初始化失败联动停止、三次全新启动真实请求6号箱（均 success、generation1、left、87帧五阶段），并覆盖 SIGINT、规划节点 SIGTERM、终端关闭 SIGHUP 后进程清理与服务下线。突然退出后的 DDS 发现缓存会延迟消失，测试等待上限60秒，不能用缓存 service list 作为存活证据。原生 Shutdown 在 x 初始化失败时外层 launch 可返回0，诊断应看 FATAL/节点退出，不仅看 shell 返回码。
- 实际恢复：仅停止原故障 launch 及其 ROS 子进程；在可见 gnome-terminal「单箱抓取 Demo · 服务端 · Domain 187」以 `x:=0.30 box_id:=6 arm:=auto` 重新启动。原先等待的客户端退出，对应第2轮 SUCCESS；独立新请求第3轮同样 success/left/87帧，验证 RViz 最终箱体 marker 与 Rerun FK 一致，Rerun 日志收到 generation3 SUCCESS。目前有意保留该可视化仿真服务运行，用户在服务端终端 Ctrl+C 即停止，不是后台常驻服务。
- 证据：`/home/astesia/Sevenova/日志/验收_2026-09-11/wall_grasp_startup/` 下 original_processes.txt、original_failure.log、build.log、test.log、startup_summary.json、各 restart 日志/轨迹、restored_visual_launch.log、restored_service_summary.json / restored_service_task.json。独立 Rerun 查看窗口可能保留旧录制，不应宣称关闭所有窗口才算退出；本次未触实机、未提交 Git。RTK 入口 `/home/li/.codex/RTK.md` 本机仍缺失，不影响此次已定位的 launch 修复。

## 2026-09-11 运控 / Codex / 10号可抓而14号失败的逐臂定位与修复
- 做了什么：在隔离domain188以x=0.30逐一复测10/14的auto、left、right。10由left完成；14的auto确实尝试left（precontact_ik无候选）和right（最后接触step5/5，right_joint7与下方neighbor_box_9碰撞）。不是漏算左臂，也未修改auto先left后right的顺序。
- 根因与修复：当前STL变换到tool0后，左右末端前沿分别超出名义TCP面约0.039µm/0.101µm；严格零间隙贴面叠加浮点误差触发邻箱接触判定。仅新距离demo新增默认1µm的contact_numerical_gap（有限0..0.0001m；旧demo保持0），用同一toolToBoxCenter()统一接触目标、附着碰撞体、JSON/Rerun及RViz位置，避免吸附时箱体跳变；没有缩网格/箱体、改ACM、关闭邻箱碰撞或增加搜索预算。这只是仿真数值间隙，不是吸盘压缩/TCP标定/安全距离；实机这些合同仍缺失。
- 改了哪里：v3_single_arm_box_extract_demo.cpp、新距离launch、既有test_v3_box_wall_grasp_demo.py与test_v3_box_wall_startup.py、距离demo技术文档及本地HTML指南。额外逐臂打印arm=... SUCCESS/FAILED stage/reason，RViz成功状态加arm。Rerun已经消费tool_to_box_center，无需再修改viewer。
- 验证结果：构建通过；新回归含10-left/14-right显式及auto、非活动臂/共享轴固定、五阶段、吸附无跳变和双臂RViz最终marker/FK一致；把gap设回0可重复得到14右臂同一邻箱碰撞（严格反例）。原10项旧demo回归通过；启动回归8项通过（含负/过大gap拒绝及三次重启真实6号请求）。本轮0.30m有限全墙扫描成功IDs=[5,6,8,9,10,11,13,14,15,19,20,21,23,24]，不是物理可达证明/永久成功清单；8号此前的右臂接触失败已被修复，历史说明已更新。
- 实际恢复：仅停止旧domain187 demo launch PID352008；在可见终端「单箱抓取 Demo · 已修复14号 · Domain 187」启动新版x=0.30/box14/auto并保留运行。真实桌面新服务请求10->left成功、14->right成功（各79帧完整五阶段），验证RViz FK/arm状态和Rerun generation3 SUCCESS；最后场景是14号。未触实机、未提交Git。
- 留给下个AI：证据在 `/home/astesia/Sevenova/日志/验收_2026-09-11/wall_grasp_arm_symmetry/`：before_summary/逐臂JSON、wrist_mesh_bounds.json、build.log、regression.log及zero_contact_gap.json反例、legacy.log、startup.log、visual_summary.json/visual_10_task.json/visual_14_task.json/visual_launch.log；before_demo.cpp可查看本轮相对已有未提交修改的精确增量。不要把1µm改成真实硬件间距，也不要通过放开邻箱ACM处理更大真实碰撞。

## 2026-09-11 Git / Codex / 单箱抓取版本冻结准备
- 做了什么：用户认可当前版本并要求按仓库 PR 规范冻结；从 alfa_v3_dev 的 1e6c58f 建立 feature/wall-box-grasp，保留旧架构目标，不直接提交基线或发布正式 tag。
- 范围：箱墙前置、距离服务、启动联动退出、双臂数值接触修复、回归和技术/HTML指南；纳入编译前置 Conda 环境隔离及测试。不纳入独立冗余 IK 启动修复、本机安装交接和无关历史日志。
- 验证：复核已有构建/仿真证据；冻结时检查环境脚本、Python AST、HTML锚点和暂存diff。未停止桌面仿真、未操作硬件。
- 留给下个AI：六项PR材料见 docs/运控/IK/V3距离输入单箱抓取Demo_PR.md。作者姓名/邮箱未配置，本任务Issue未确认；禁止冒用历史作者或MOTION-94/154。尚未commit/push/创建PR，远端读取未及时返回；确认身份及Issue后完善关联再提交，核心review/CI通过前不合并、不打正式tag。工作区仍保留未归属修改，勿git add .。

## 2026-09-11 Git / Codex / 复用 VS Code 登录与 Issue 核对
- 用户授权：优先复用 VS Code 的 GitHub 登录；只有能确定对应本任务时才关联 Issue，否则允许留空。
- 核对结果：GitHub 仓库 state=all 列表返回18项且均为PR，无独立Issue；本地NOW/TASKS未提供本任务编号，MOTION-154是不同的双臂动作任务。没有Linear在线访问能力，未声称查遍Linear；本次关联留空。
- 身份/网络：VS Code认证日志确认16:28登录成功；但当前命令环境未继承凭据助手。通过正常VS Code Git askpass请求在Username阶段超时，没有读取到可核验的账号身份或凭据，未读取/导出编辑器秘密存储。GitHub API公开读取成功，github.com HTTPS连接仍超时，不能把登录成功等同于可推送。
- 交接：PR材料已取消Issue阻塞；源码与先前冻结快照一致。仍未commit/push/创建PR；需用户检查VS Code是否有待确认的Git账号授权提示，并恢复github.com连通性。暂存范围和未归属工作区修改保持隔离，未操作实机或演示进程。

## 2026-09-11 Git / Codex / 账号核验与 fork 提交
- 用户确认授权后，VS Code正常Git凭据助手已可使用；GitHub /user 核验账号bestastesia（ID136950813）。仅本仓库设置作者bestastesia / 136950813+bestastesia@users.noreply.github.com，避免公开私人邮箱；凭据不写入仓库或日志。
- 上游权限只有pull，没有push；已创建bestastesia/alfa_robot公共fork，采用fork的feature/wall-box-grasp向kkozia188/alfa_robot的alfa_v3_dev提PR，不推送或合并上游基线。上游alfa_v3_dev经API核验仍为1e6c58f。
- Issue按用户授权留空；源码与先前冻结树一致，只更新PR/交接文档。保留未归属冗余IK修改和历史日志，不扩大暂存范围。
- GitHub API可用但github.com Git传输仍超时，改用GitHub Git数据库API发布同一Git对象；必须校验远端tree/commit SHA与本地完全相同后才创建新分支，不强制更新已有分支。实际提交/PR结果记录到仓库外冻结清单；未经核心review/CI和合并，不发布正式tag。

## 2026-09-11 Codex / 单箱抓取前按箱高下降共享升降轴
- 做了什么：在冻结提交 `27ea05e` 上新建 `feature/wall-box-height-alignment`，按用户要求实现 `max(0, 肩部中心Z-(箱中心Z+offset))` 的下降预阶段；offset默认0.25m。冻结分支和原PR未改，本轮未提交/推送。保留原冗余IK未提交工作，不混入本功能。
- 高度合同：中心取左右肩部前三关节公共轴交点的中点；复用解析IK几何，新增模型级肩中心读取，旧static接口仍保持LegacyV304语义。当前world Z≈1.342432773m；底排下降≈.892432773m、第二排≈.482432773m。updown按模型[-1,0]m限位拒绝越界，至多5mm采样整机碰撞；不做静默clamp、不放松ACM。非正差不抬升。下降后IK基坐标/RRT起态/负重返回目标一致；返回保持升降高度。
- 改了哪里：解析IK hpp/cpp；单臂demo C++、距离launch（默认box0，新增align_height/shoulder_box_offset）、Rerun摘要/米与弧度回放区分；新增 `test_v3_box_wall_height_alignment.py`；原抓取/startup测试显式关闭高度调整以保持冻结回归；更新距离demo Markdown/HTML。
- 验证结果：最终3包Release构建成功；新测试23次真实服务请求+3类非法偏置拒绝通过（初始肩中心独立URDF FK、低两排扫描、显式双臂、10/14、高低重置、不抬升、超行程拒绝/-1m边界、16轴/插值/附着无跳变、RViz最终箱位对齐Rerun FK）；冻结抓取回归通过（含10/14及零gap反例），startup 8项、legacy 10项、解析IK CTest 2/2通过。Rerun RRD verify通过，已记录261帧从lower_to_box_height至携箱rrt_return末帧。证据 `/home/astesia/Sevenova/日志/验收_2026-09-11/wall_height_alignment/`（最终新测试在final_acceptance/）。
- 关键限制：x=.30/offset=.25时低两排只有0、4、5、9在本次搜索成功；1、2、3、6、7、8双臂预接触IK无候选，不能说已覆盖整两排；尤其6号冻结版成功、新高度版失败。未擅自搜索替代高度/改姿态绕过用户公式。场景仍无地板，碰撞离散非连续，未构造下降中途碰撞反例；未做实机动态/载荷/标定验收。每次请求重置零角/升降0/完整墙，不是连续升回与连续搬箱。
- 留给下个AI：本机新升降版双视图已启动在localhost ROS_DOMAIN_ID=188，默认0号success；原冻结演示仍在187（不要混用服务）。同一install现在是新构建，重启冻结动作要显式 `align_height:=false` 或检出冻结分支重编译。HTML新指南已发起打开。后续若要整两排覆盖，应与用户确认偏置/接触姿态/高度搜索策略，不能宣称25cm已全覆盖。实机肩中心/零位/行程与速度负载合同、放置释放/跨箱接续规范仍缺；RTK引用文件仍不存在。

## 2026-09-11 Git / Codex / 升降抓取版独立PR提交准备
- 用户明确要求将本次高度版提交PR；仅暂存高度版代码/文档/测试和本任务交接，不混入冗余IK修复或既有未归属日志。
- GitHub API核验账号bestastesia；上游PR #20仍open未合并，目标alfa_v3_dev仍为1e6c58f。新PR沿用该目标，标注依赖#20并要求先合并#20；合并前累计diff包含前序内容，可用27ea05e到本次HEAD单独审阅升降增量。Issue无明确对应，按用户授权留空。
- PR说明记录默认行为变化、0/4/5/9成功及6号回归差异、离散碰撞/无地板/无实机验收边界和复现命令。只发布待审查分支，不合并、不打正式tag。远端发布结果另存仓库外清单。

## 2026-09-12 Codex / 距离升降单箱Demo加入地面与四周碰撞
- 做了什么：在7d2defe上新建 `feature/wall-box-environment-collision`；共享makeScene统一加入原生CollisionObject环境盒体，复用现有整机状态/边检查覆盖初态、下降、IK、接触附着、抽出和负重RRT返回。未放松ACM、未缩小碰撞网格，保留原冗余IK未提交修改。本轮未提交/推送/创建PR。
- 接口：距离launch默认check_environment=true；environment_file JSON启动时读取，配置frame_id/world、description和轴对齐boxes(id/center/size)，ground必填，数值/尺寸/ID/坐标系/不支持旋转校验失败即退出。result_json.environment、RViz及Rerun使用同一份几何；RViz重启不保留旧障碍。false只给历史回归使用，日志明确警告。旧单臂入口不变。
- 基准与任务风险：独立URDF FK/STL顶点确认model_base最低world Z约-0.4022m，默认地板上表面-0.402201m（1µm仅数值间隔）；示例四周内边界X±3m、Y±2.5m、顶Z2.6m，全部未现场标定。为保留已验收网格，默认墙底仍Z0，比地面高40.22cm，不能称落地场景。允许wall_bottom_z为有限负数；显式落地到-.402201时，0号按25cm偏置需下降约1.2946m，真实超出[-1,0]，返回height_alignment_limits，不clamp/不关闭地板。真实底盘/地面/墙底坐标及障碍尺寸、安全裕度合同仍缺；RTK引用文件仍缺。
- 改了哪里：单臂C++共享场景、距离launch、新config/v3_box_wall_environment.json、Rerun、test_v3_box_wall_environment.py；原高度和冻结抓取测试显式关闭环境以保留原算法回归；更新距离Markdown/HTML。环境固定于进程，修改自定义文件需重启；只支持有限尺寸静态轴对齐盒体，离散碰撞不是连续或实机安全证明。
- 验证结果：2包Release构建通过；14真实请求+6类非法环境全部通过，默认0/4/5/9/10/14/20/24成功，ground/base初态、仅闲置右臂障碍、下降中途台板（updown=-.289168）、仅携箱返回目标及抽出侧挡片碰撞全部拦截。负world墙底可输入且底排超行程拒绝正确。高度23+3、冻结全墙/零gap反例、startup8、legacy10、解析IKCTest2/2通过。实际RViz/Rerun启动，RRD verify成功；6环境实体中心/半尺寸与配置一致，完整261帧到携箱rrt_return。抓录脚本整进程组SIGINT导致launch重复转发，Rerun退出atexit出现KeyboardInterrupt；发生于完整回放之后，录制独立验证通过，不隐瞒此测试清理现象。
- 证据与接手：`/home/astesia/Sevenova/日志/验收_2026-09-12/wall_environment/validation_summary.json`、model_ground_reference.json、final/及visual/；前一轮13请求记录在2026-09-11同名目录third/。测试使用localhost189/190/191/192并只清理自身进程；本轮未留下后台demo，重启时source当前install即可默认启用环境。本功能完成不意味着用户整体新架构接手/仿真复现学习目标已全部完成。

## 2026-09-12 Codex / 环境碰撞后补齐两种箱墙25箱覆盖
- 做了什么：复用环境测试增加可选`--scan-wall`，固定x=.30m、25cm偏置，默认/模型落地各25请求；未改业务算法、环境、预算或ACM。新请求摘要保留箱号/选臂/全部attempts/耗时，对成功携箱回放统一做FK地板高度检查。
- 验证结果：domain193隔离运行55请求（50扫描+5碰撞反例）及6类非法配置检查通过。默认墙底Z0成功12/25：0,4,5,9,10,14,15,19,20,21,23,24；12个precontact_ik失败、22号cartesian_retreat失败。模型落地墙底-.402201成功8/25：5,9,10,14,15,19,20,24；0～4全超行程，其余12个precontact_ik。失败auto均有左右臂尝试；默认17/落地22是候选机身自碰撞，不能统称数学无解。
- 改了哪里：现有环境回归脚本及距离Markdown/HTML，个人接手导航同步覆盖结论。无新框架/依赖，无生产代码变化，未提交推送PR。
- 接手证据：`/home/astesia/Sevenova/日志/验收_2026-09-12/wall_environment/full_wall/`含每请求原JSON、coverage.csv、coverage_summary.json（源码/二进制hash），源码与安装环境配置/launch逐字节一致。测试通过不是全部搬运成功，成功仍附着返回；这是固定测试距离下独立请求覆盖，不是最优距离、连续拆墙、放置释放或实机验收。

## 2026-09-12 Codex / 碰撞demo改为home起终姿态、零地面和单开口仓库
- 做了什么：按本轮用户要求，距离demo直接读取SRDF `whole_body/home`：双臂[-90,-90,0,-90,0,0,0]°、updown/head=0。先按原25cm策略下降，携箱RRT返回home臂角后新增`restore_default_height`，携箱按≤5mm采样升回0；预先验证整段负重上升，阻挡返回`return_lift_collision`，不跳过上升伪报成功。旧交互launch仍保持原零姿态/坐标。
- 坐标/仓库：仅距离launch传`model_ground_offset=.402201`修正base_to_model安装高度，world/base_link仍Z0，箱墙底Z0，地板上表面Z0；独立碰撞STL/FK测得底盘最低Z=.947µm（数值容差，非实机安全裕度）。仓库内尺寸X长4m/Y宽2.38m/Z高2.35m，+X正墙、−X开口，两侧/正墙/地板/顶棚五个有碰撞实体，10cm厚度向外。箱墙Y居中，宽2.04m，两侧各17cm，背面距正墙1µm（复用contact_numerical_gap，避免右臂吸附FK误差误判穿正墙）。没有缩网格、放开ACM或扩大搜索预算。
- 改了哪里：description增加默认0的落地标定arg；距离launch传同一URDF给规划器/RViz/Rerun；C++初态、负重回升、JSON initial_joints及环境坐标；默认环境配置增加`anchor:box_wall_back`，每次独立距离请求重建仓库相对箱墙的位置（不是底盘行走）。自定义world环境默认不平移，配置仍只从启动文件读一次。Rerun不再插入全零预览，用规划器initial_joints，摘要更新。环境/固定升降/高度/startup回归已迁移新home合同；更新距离Markdown/HTML，未触本轮开始前其他未提交修改。
- 验证结果：3包Release构建成功（description仍有已有pytest检测warning）；仓库42真实请求+7非法配置通过，含五个仓库实体分别侵入整机、空闲臂/下降中途/仅负载返回/仅负载回升/抽出挡片反例；独立全机器人初态碰撞网格均在仓内。固定x=.90m扫描成功17/25：5,6,8,9,10,11,13,14,15,16,18,19,20,21,22,23,24；0～4超行程（下降需1.294634m），7/12/17接触路径无解。不是连续拆墙或全策略可达性结论。新home导致x=.30初态碰箱墙，x=.75携箱home碰剩余箱体，不能沿用原零姿态距离示例。
- 其他验证：高度21请求+3非法偏置；固定高度25墙格/7非法输入/20左24右/零gap反例/车头覆盖；startup8项；旧交互demo10项；解析IK CTest2/2；py_compile和diff --check均通过。右臂9号RRD verify通过且完整418帧至restore_default_height，最终joint_states/携箱RViz marker与同一落地URDF独立FK一致，录制进程干净退出。已实际查看RViz/Rerun新版仓库及home携箱终态截图；Rerun中文缺字方框为现有字体问题，本轮未处理。
- 接手证据/运行：`/home/astesia/Sevenova/日志/验收_2026-09-12/warehouse_home/`含environment/summary.json、home_collision_mesh_bounds.json、各回归日志/JSON、visual/warehouse.rrd及截图、源码/二进制hash。仅SIGINT停止原可视化launch PID94173及其子进程，以可见终端「仓库 Demo · home起终姿态 · Domain 0」重启x=.90/box5/auto，新请求SUCCESS/left/422帧并保留运行；沿用原ROS_DOMAIN_ID=0、ROS_LOCALHOST_ONLY=0，终端Ctrl+C停止。原引用`/home/li/.codex/RTK.md`仍不存在；未接实机、未提交推送。

## 2026-09-12 Git / Codex / 仓库碰撞与home起终姿态PR准备
- 做了什么：按用户要求提交当前仓库碰撞demo；仅包含环境/home/零地面代码、配置、测试、距离指南和对应交接，不混入冗余IK启动修复、本机MoveIt交接或历史未归属日志。
- 分支/依赖：`feature/wall-box-environment-collision`，基于`7d2defe`；GitHub核验上游#20/#21仍open，目标`alfa_v3_dev`仍为`1e6c58f`。本PR要求先合并#20再#21，本次增量从`7d2defe`审阅；沿用前序Issue留空约定，不编造或关闭历史Issue。
- 验证/边界：源码与完整验收的6项实现/二进制SHA256一致，提交前重跑仓库环境与25箱扫描42请求+7非法配置通过，解析IK CTest 2/2、暂存语法/JSON/diff检查通过，无关文件哈希未变；完整历史验收见`/home/astesia/Sevenova/日志/验收_2026-09-12/warehouse_home/`，本次发布核验见其`pr/`子目录。本机证据不是远端附件；PR给出复验命令和17/25覆盖、升降超限、离散碰撞及非实机边界。
- 发布约束：中文提交/PR及Codex协作署名；只创建待审查PR，不合并、不打tag、不强推。发布后的PR编号、commit/tree一致性和CI/review状态另存发布清单并追加交接。

## 2026-09-12 运控 / Codex / 单次肩心-TCP舒适高度实验收口
- 做了什么：在 `feature/wall-box-comfort-height` 完成每臂一次几何选高的新实验入口，先调整 updown 再抓取，保持携箱 home + 升降归零；原 launch 的固定0.25m默认不变。仅仿真，未连接硬件。
- 改了哪里：共享单臂Demo、解析IK模型臂长接口、纯选高头文件/测试、新comfort launch、Rerun诊断、实验/分析脚本；新增 `docs/运控/IK/V3单次舒适高度抓取Demo.md` 和 `V3单次舒适高度实验.json`。
- 实验结论：9600次训练后冻结候选 `[1.10,1.15]` / preferred1.15，再跑1350次验证。三种子显式臂训练84/450→102/450，独立距离验证48/300→42/300（.95左5/右9稳定退化）；共同成功的限位裕度和行程指标不支持更自然。**未证实优势舒适区间**，不可替换旧默认或将含训练距离的auto总计当独立验证；验证后未回调参数。
- 验证结果：Release构建通过，CTest18/18；旧环境42请求+7非法、旧高度21请求+3非法、旧/新启动8/12项通过。77轮10954响应独立重验通过（包括预期规划失败，并非全部抓取成功）；左右完整RViz回放及406/397帧RRD核验通过。可选Rerun原生截图panic，不计通过。
- 留给下个AI：证据 `/home/astesia/Sevenova/日志/验收_2026-09-12/comfort_height/`，含冻结协议/原始任务/最终哈希/复现驱动；早期FK容差失败已保留并整段重跑，不能删掉失败证据。训练后运行时只修正初始预览臂选择，未扩大预算/放宽碰撞/增加换高重试。本轮未commit、未创建PR；保留其他AI的交互IK及交接文件改动。`/home/li/.codex/RTK.md` 不存在，无法读取。

## 2026-09-12 文档 / Codex / Demo 操作入口精简
- 做了什么：新增 `docs/运控/IK/V3Demo启动速查.md`，集中七个交互Demo的用途、首次启动和二次操作；强调每个终端加载环境、同domain及关闭后重启的区别。
- 改了哪里：README增加速查入口；历史实验记录保留，不再作为日常操作入口。
- 验证结果：按launch及服务注册核对命令，双臂三模式共用服务，舒适/固定高度两版共用服务；文档shell语法及入口存在性检查通过，未启动或打断运行中的Demo。
- 留给下个AI：日常命令只维护速查，不往操作入口堆实验日志；本轮未改代码、未提交。

## 2026-09-12 Git / Codex / 舒适高度实验与Demo速查PR准备
- 做了什么：按用户要求发布 `feature/wall-box-comfort-height`，仅提交单次选高实现、可复现实验/负结论、操作速查及本任务交接；排除独立冗余IK启动修复、本机交接和未归属历史日志。
- 依赖：已核验上游 #20/#21/#22 均未合并，目标 `alfa_v3_dev` 仍为 `1e6c58f`；需依次合并前序PR，本轮增量从 `cc3aa86` 审阅。沿用前序无对应Issue留空约定，不编造ID或关闭旧Issue。
- 验证/风险：保留旧默认、完整碰撞和规划预算；独立验证未证明舒适区间优势，PR不得写成成功率或自然度全面提升。发布前核验历史证据与源码哈希、重跑CTest及启动回归；不接实机，不合并、不强推、不打tag。
- 留给下个AI：发布清单与最终检查在 `/home/astesia/Sevenova/日志/验收_2026-09-12/comfort_height/pr/`；日常操作入口为README链接的速查，不新增冗长PR操作文档。发布后的编号另行追加。

## 2026-09-12 运控 / Codex / 舒适选高加入初态相连的升降碰撞边界
- 做了什么：按用户要求，不能进入候选舒适区间时取当前姿态可达范围内最近高度；先以≤5mm检查升降首个碰撞边界，再复用一次几何选高。保留整机/负载碰撞、固定偏移默认、home归零及原IK/RRT预算；请求/换臂清理边界，JSON/RViz区分未检查提案和最终选高。
- 改了哪里：共享单臂Demo、现有选高单测/benchmark/环境回归；速查仅改用途一句，详细说明标明大规模实验是修正前历史结果，未混写冻结参数或实验JSON。
- 验证结果：Release构建、CTest18/18、新入口启动12项、环境35请求+7非法配置断言通过；6组独立FK/回放对照通过（不表示全抓取成功）。x=.90时6号左/auto完整成功、q=-.682676949；7号两臂q=-.990，绕开-.995的地面碰撞但接近第5/5步仍无IK候选。中途障碍-.280时选-.275，不跨越阻挡。
- 进一步诊断：7号q=-.990固定接触姿态，离线0.1°冗余角扫描启用限位0解，禁用限位每臂28800解，最接近限位候选仍需|J2|≈105.43°>105°；不是臂长不足，也不能推成改变整机/接触姿态后的全局无解。诊断未改变运行时限位/预算。
- 留给下个AI：新证据在`/home/astesia/Sevenova/日志/验收_2026-09-12/comfort_height/safe_lift_fix/`（final回归、独立腕心/接触IK诊断、源码哈希）；旧training_frozen哈希保持8e4d9d6f…634d8e。未重启用户domain188可见旧进程，使用新代码须按原命令重启；本轮未提交/推送PR，保留独立交互IK和原有未归属改动。

## 2026-09-12 运控 / Codex / 5×5连续吸附后放序列实现与部分验收
- 做了什么：基于现有comfort版本新增连续25箱入口，默认x=0.90m/auto，按20–24→15–19→10–14→5–9→0–4。非底排正吸失败再顶吸（auto先正吸左右再顶吸左右），底排只顶吸；后方悬空释放消失前保留携箱碰撞，空载回home/升降归零；完整周期成功才提交，失败保留剩余箱并停止。未连接硬件，未提交/推送，未关闭用户已有GUI（PID228870）。
- 改了哪里：`v3_single_arm_box_extract_demo.cpp`、小型`wall_sequence.hpp`/C++测试、新`v3_box_wall_sequence_demo.launch.py`及原launch参数；Rerun按逐帧scene_index/可见性回放，真实工具-箱体变换支持顶吸与非立方体。新增序列/播放器测试，调整原单箱/高度/环境回归的回退次数及失败不执行契约；说明见`docs/运控/IK/V3箱墙连续搬运测试.md`。原PlanWallBoxDemo请求不变，独立调用仍携箱home；连续入口才后放消失。Trigger返回接受状态，最终JSON在task_json。
- 验证结果：两包编译通过，CTest19/19，播放器无窗口测试通过；原单箱25ID扫描、左右臂/FK/零间隙回归通过；高度21请求+3非法参数，环境/comfort35请求+7非法配置通过。默认新种子104729连续测试为**17/25**：20–24、15–19、10–14、5、6完成；7号箱正吸后放碰撞，顶吸左预接触IK失败/右后放碰撞；保留0–4、7、8、9，不可宣称25箱成功或底排顶吸已完整执行。后方障碍注入四种尝试均被carried_target_box<->environment_rear_block拒绝，0箱消失/25箱保留。证据根目录`/home/astesia/Sevenova/日志/验收_2026-09-12/wall_sequence/`，默认`final/sequence.json`，障碍`rear_obstacle/sequence.json`及`check.json`，回归`environment_final/`、`height_verified/`、`grasp_verified/`。
- 留给下个AI：原home实际朝前，不能当后放点；当前后放TCP在实测车尾X−rear_clearance（默认.02m），Y/Z取本次选高后的home、朝向绕Z转180°，整箱至少离车尾1cm。低位后放仍受底盘碰撞/IK限制，下一步应独立评估携箱二次升高或其他后放姿态，再用`test_v3_box_wall_sequence.py --require-complete`验收25箱。普通脚本PASS只证明成功前缀/失败保留契约，不证明全完成。没有放宽默认距离、碰撞或跳箱来提高数字。

## 2026-09-12 运控 / Codex / 7号箱顶吸独立诊断入口
- 做了什么：核查用户正在运行的x=.80/box7单箱结果，确认顶吸左右臂均在提前检查的携箱home目标`return_goal`碰撞失败，不是此次已验证顶吸接触IK无解；与序列x=.90/已清上层/后放策略不同。保留用户domain188可见进程，未重启、未提交。
- 改了哪里：共享Demo新增启动参数`suction_mode=auto/top`，默认auto不变，top只尝试顶吸；共用`wallGraspAttempts`增加top_only参数，初始预览与结果JSON同时反映指定方式。原箱墙launch透传，comfort/sequence包装入口复用；新增`test_v3_box_wall_top_suction.py`及小型C++断言，说明追加到`V3箱墙连续搬运测试.md`。
- 验证结果：构建成功、CTest19/19；独立domain199顶吸启动/服务共5请求通过契约检查，7号在x=.80（auto/left/right）及.90（auto）均为携箱home与邻箱碰撞，0号底排只顶吸规则保持。未删除邻箱或关闭碰撞；失败只有初始帧。证据`/home/astesia/Sevenova/日志/验收_2026-09-12/wall_sequence/box7_top/verified/`，用户旧节点结果在同级`existing_x080.json`。
- 留给下个AI：复现命令为`ros2 launch alfa_robot_moveit_config v3_box_wall_comfort_grasp_demo.launch.py x:=0.80 box_id:=7 arm:=auto suction_mode:=top`，需重启才加载新参数。单箱任务仍有24个邻箱，不是孤立箱；neighbor_box_N是排除目标后的邻箱数组索引，不可误认为墙box_id。没有解决搬运可行性，勿宣称顶吸已成功。

## 2026-09-12 运控 / Codex / Demo失败诊断回放与现场冻结
- 做了什么：按用户要求将完整可复制命令约定写入AGENTS.md；自研V3单臂/箱墙/舒适高度/序列、双臂平移/旋转/异构入口共用失败诊断回放，RViz/Rerun显示有效前缀及被拒绝状态、红色接触点与碰撞双方，末帧冻结。无IK解保持最后可用状态并标记目标；冗余IK入口明确不做碰撞检查。旧全排orchestrator首次失败停止且不清场；旧pick_place脚本保留现场/失败目标并停止后续轮次。
- 改了哪里：两个V3动作节点与冗余IK节点、共享demo_failure_markers.hpp、Rerun demo_failure.py及三个viewer；旧orchestrator/pick_place脚本及RViz标记订阅；新增/扩展诊断、顶吸、序列和IK回归。详细合同和覆盖边界见docs/运控/IK/Demo失败诊断回放.md。
- 安全边界：diagnostic_frames独立于可执行frames/事务提交；预检查目标或未连接搜索候选明确标为快照，不冒充成功连续轨迹，不向控制器下发碰撞姿态、不放宽ACM。自动换臂/正吸转顶吸保持，最终失败才显示最后一次尝试。序列失败箱不移除，逻辑final_joints仍为最后成功周期。上游MoveIt GUI插件不提供失败轨迹时仅保留其原生错误显示，不宣称改造了第三方规划器；旧控制器流程未做实机验证。
- 验证结果：两包构建通过、CTest20/20；真实viewer解析/冻结及序列viewer测试通过。独立localhost domain197/198无窗口ROS回归：顶吸5请求及实际关节冻结通过；共享7场景（双臂平移/旋转/碰撞/异构/初始IK失败、单臂升降碰撞/IK失败）通过；升降冻结在首个失败采样，红色接触点保留。后方障碍序列0/25与默认序列17/25均完整播放到失败末帧并验证冻结、失败箱场景保留；冗余IK成功启动376解且不可达目标保持原关节通过。
- 证据：/home/astesia/Sevenova/日志/验收_2026-09-12/failure_replay/，含box7_top、shared、sequence_rear、sequence、ik_family与构建/回归日志。
- 留给下个AI：此功能没有解决抓取可行性，默认序列仍17/25、7号rear_placement失败。单箱7号x=.80顶吸仍return_goal携箱home碰邻箱，发生于接触IK规划之前，此时展示拒绝目标快照而非成功吸取。用户domain199旧Demo未被停止/重启，必须由用户Ctrl+C并重启才加载新二进制；未提交或推送，保留其他已有改动。

## 2026-09-14 运控 / Codex / 箱7假附着根因修复与独立回放验收（原距离尚未完成）
- 做了什么：确认旧单箱实现先检查携箱home，再将未连接的拒绝目标附着状态加入回放，是“降高后箱体瞬移、穿模”的实际代码根因。先规划真实接触/抽离，再检查携箱返回；未连接候选只保留rejected_*诊断，实际机器人停在最长已连接前缀。真实初始碰撞保留原姿态和contacts，升降冻结首个失败采样。此条取代09-12把旧return_goal快照当作顶吸失败证据的结论。
- 改了哪里：v3_single_arm_box_extract_demo.cpp及箱墙launch：顶吸按腕心选高、碰撞检查地折叠双臂后降高、吸盘长轴横置、携箱折臂返回，保留环境/邻箱碰撞；sequence只在后放释放时消失并空载复位。wallRearPlacementPose按旋转后的整箱前沿计算后放点；完整sequence不使用单箱sequence_prefix预清空夹具。新增独立MoveIt全场景回放验证器和实际ROS/Rerun回放检查，扩展失败/环境/选高/顶吸回归。用户命令约定与两份IK诊断/序列文档同步。
- 验证结果：最新两包构建、CTest20/20、环境35请求+7非法配置通过（含最新initial_state真实接触点断言）。高度21请求+3非法偏移、共享原7失败用例及新增single_return用例通过。x=.50、明确sequence_prefix（先移除17箱的夹具）左右臂与多种种子真实回放成功；live_auto_final为1593帧/1506不同关节采样/1611箱体标记，最新验证器复核2288次0.5度/2.5mm碰撞和限位采样通过，附着位姿连续，负例瞬移/碰撞被拒绝。有限采样不代表数学连续碰撞证明。
- 未完成：本轮开始时用户窗口为旧x=.90/full/top；收尾时外部已将其重启为x=.75/full/top（见下条只读检查）。已安装版原距离预接触不可达，不会假吸附。独立URDF几何证据：箱顶任一点腕心水平距离至少1.065525m，大于肩肘腕总长0.979m；升降不能改变水平距离。x=.80只证明顶面中心超界，不能扩大成所有偏心点无解。完整默认sequence仍17/25，在箱7/cartesian_approach失败；不能把较近夹具成功称为原任务解决。
- 后放附加证据：x=.50/预清空17箱的原生隔离单周期planTask完成2623帧顶吸、后放、释放、空载home，独立验证3629采样通过；不是安装launch完整25箱或该隔离周期实际GUI回放验收。
- 证据：/home/astesia/Sevenova/日志/验收_2026-09-14/box7_top_fix/，summary.json含最新源码哈希和验收边界；environment_final、live_auto_final、top_final、sequence_final、top_sequence_cycle及最终构建/CTest日志。用户domain199 PID21794/21797未触碰；测试仅使用空闲localhost域。
- 留给下个AI：目标仍active；不能关闭碰撞、暗中改箱墙距离/删邻箱或修改未经核实的物理模型来声称解决。需要确认用户所说可达对应的真实距离/模型，或另行明确底盘前移的任务范围；x=.80偏心顶吸尚未验证。共享双臂demo仍有REJECTED_SNAPSHOT逻辑，本次单臂修复不能宣称所有demo都消除了未连接快照。未提交/推送，保留大量此前工作区改动。
- 收尾现场变化：用户自行启动新domain199 PID72593/72596，x=.75/box7/auto/top/full，已加载12:43构建。本Agent仅订阅task_json，未调用服务或改变场景；user_current_x075.json为实时结果：precontact_ik，顶面中心腕心XY=1.134010m > 0.979m，无附着、无碰撞contacts，保持原位。旧PID21794/21797已不存在，不应再让用户重启“旧.90窗口”。该数值仍只针对中心吸点；偏心点/不同物理模型未验收。

## 2026-09-14 运控 / Codex / 箱7完整场景偏心顶吸遮挡检查
- 做了什么：在未修改生产规划器/距离/场景的前提下，补查x=.75/full的偏心吸点，避免把中心吸点不可达扩大解释为全部偏心点无解。复制当前模型参数到离线探针，强制摆放吸盘头碰撞网格（不冒充IK、不给ROS回放），全场景FCL检测并核验变换。
- 验证结果：两侧各31×41位置、24个垂直顶吸yaw，30504采样/侧全部与上方12号箱碰撞；1512采样/侧仅通过水平臂长必要条件，仍全部受遮挡。负对照将离线头网格移远后不再有12号接触，避免陈旧变换假阳性。实际用户.75窗口只读41组关节及41箱标记确认原位冻结且无附着。证据topface_x075_full和user_current_x075_freeze_check.json（既有box7_top_fix根目录），文档追加有限扫描边界。
- 留给下个AI：仍未证明任意连续吸点/yaw/倾斜或密封范围可行/不可行，不应为了成功自动删上方箱。目标仍active，等待用户确认其确信可达场景的实际测距、模型和上方清空条件。收尾前观察到用户自行把窗口从.75改成x=1.00；已提示x变大会离墙更远。本Agent未重启/终止用户进程。无生产代码修改、无实机操作、无提交推送。

## 2026-09-14 运控 / Codex / 箱7目标阻塞交接
- 状态：连续三轮目标推进仍缺少同一关键输入——用户确认的实际可达场景（测距基准、是否已搬走上方箱体、模型是否对应实物）。本轮再次检查现有验收证据，原x=.90固定底盘全顶面腕心距离下界仍超模型臂长；.75/full有限偏心扫描仍受12号箱遮挡。既不能以.50/sequence_prefix成功替代原场景，也不能擅自改物理模型、清场、前移底盘或放宽碰撞。标记目标blocked而非complete，等待明确输入后恢复。
- 已完成保留：单臂假附着回放根因修复、真实失败冻结验证、明确夹具顶吸成功及有限步长独立碰撞检查。完整序列仍17/25，原场景箱7顶吸成功未验收；详见box7_top_fix/summary.json和各原始证据。
- 当前外部状态：收尾只读进程检查未见运行中的箱墙launch或本Agent测试进程；没有停止用户进程。git diff --check通过。无需在相同条件下重复跑已通过回归来假装推进。

## 2026-09-14 运控 / Codex / 用户确认从失败箱直接开始（解除场景确认阻塞）
- 用户已明确：直接从失败箱开始，前序箱体视为已经消失，不要求先执行此前搬运。复用wall_context=sequence_prefix，不另写清场实现、不改变完整25箱sequence默认行为。箱7删除10–24及5、6；保留目标7、邻箱0–4/8/9和全部环境碰撞。
- 改了哪里：test_v3_box7_top_replay.py补验实际RViz DELETEALL、7个邻箱坐标与碰撞场景一致、剩余箱号标签；诊断文档更新用户确认的语义。生产参数功能已存在，无需新增接口或构建生产程序。
- 验证结果：安装版x=.50/box7/top/auto/sequence_prefix/seed104731真实回放成功，1966帧、1879不同关节状态、1984箱标记；独立2661次碰撞/限位采样通过、附着连续，Rerun及瞬移/碰撞负例通过。证据confirmed_prefix_live。另原样保留x=.75做prefix对照，确认前17箱确实删除后，仍因中心腕心XY1.134010m超过.979m在precontact_ik停止，不再把上方遮挡当作已清场后的失败原因；证据confirmed_prefix_x075。
- 留给下个AI：此前“等待上方是否清空”的阻塞已由用户解除，目标仍active（上一轮打断前并未调用update_goal blocked）。给用户的可跑通演示命令明确使用.50m，不暗称.75/.90顶吸已通过；.75偏心策略的密封/IK/运动仍未验收，完整序列仍17/25。保留工作区改动，无实机执行、无提交推送。

## 2026-09-14 运控 / Codex / 箱墙连续后放释放与可复制长命令
- 做了什么：按用户要求将完整可编辑长命令约定写入AGENTS.md（环境、路径、全部续行符齐全，不以短包装脚本替代）。箱墙单箱和序列统一后放→消失，无每箱home/升降复位，下一箱继承真实末态；失败只播放相连前缀，未连接搜索候选不瞬移。
- 改了哪里：v3_single_arm_box_extract_demo.cpp、箱墙launch、Rerun描述、独立回放检查器和安装版单箱/序列/失败测试、两份箱墙/失败诊断文档。顶吸独立肩高于腕心0.10m策略；当前姿态可安全升降则不强制折臂；先提5cm并后退2cm，再水平抽离。墙任务边检查收紧0.25°，按得分顺序检查实际消费的笛卡尔候选，保留全部障碍和负载碰撞。
- 验证结果：2包构建安装、CTest20/20、8项安装版失败停帧回归通过。最终x=.50/正吸ratio=.95/top offset=.10/地面间隙1微米/seed104731：完整25/25，54830帧、独立61624次碰撞/限位检查；单箱7/prefix2089帧、独立2537次检查及真实ROS/Rerun标记回放通过。证据 `/home/astesia/Sevenova/日志/验收_2026-09-14/wall_rear_release/summary.json`，最终目录single_backoff/sequence_backoff/failure_final；不要将早期失败或仅规划SUCCESS误作最终证据。
- 留给下个AI：完整近墙初始home碰撞，命令显式initial_pose=arms_down（只改仿真初始条件，非声称home可无碰撞到达，全局home不变）；单箱prefix仍从home起，不等于完整序列的前序末态。上排箱12、7回退顶吸，底排全顶吸。实测完整规划203.24s，当前20Hz名义播放45.7min，先整段规划再播放；完整GUI未实时间等待45.7min，已独立检查全部帧和插值。单箱规划4.74s、播放104.45s。未承诺其他距离、姿态或实机安全。旧用户窗口未主动关闭；不提交/重置已有大量未提交改动。完整复制命令位于docs/运控/IK/V3箱墙连续搬运测试.md顶部。

## 2026-09-14 运控显示 / Codex / Rerun完整时间轴批量写入
- 做了什么：仅修改V3单臂/箱墙共用Rerun显示端，移除播放timer及速度/阶段停顿等待；机器人变换每1024帧用send_columns写入，复用同帧FK记录携箱，邻箱只在场景切换更新。全部轨迹/吸附/消失/失败末帧保留；task_frame时间轴默认暂停、关闭循环，交给界面控制。新任务另起时间戳，不能覆盖上一失败末帧。
- 改了哪里：v3_single_arm_box_extract_viewer.py；两个既有viewer测试适配；新增test_rerun_wall_timeline.py真实RRD回读/计时验收；箱墙文档增加Rerun完整长命令。未改规划器、碰撞精度、RViz timer或launch实现（与上一轮验收SHA256一致）。
- 验证结果：alfa_robot_rerun构建安装；CTest20/20。完整25箱54830帧，RRD写入22.64s含FK/flush（回调含JSON22.92s）；本机Rerun headless gRPC接收端23.88s（含JSON24.14s，无OS窗口，不等同GUI绘制耗时）。回读完整RRD，23链接所有变换/各帧载荷状态正确；失败958帧0.35s，最后准确为FAILED_HOLD: rear_placement；两份RRD及footer验证通过。命令经过bash -n和无环境新shell --show-args检验。
- 留给下个AI：证据 `/home/astesia/Sevenova/日志/验收_2026-09-14/rerun_timeline/summary.json`。Rerun仍须等规划完整结果（上轮约203s）；只是取消随后逐帧等候，不能声称解算变快。用start_rerun=true/start_rviz=false的新长命令，用户界面选择task_frame、播放/暂停/FPS/拖动。双臂Rerun显示器没有本轮批量化；RViz完全不变。测试未停止用户正在运行的x=.75箱墙/RViz进程，无硬件操作。

## 2026-09-14 Git / Codex / 箱墙搬运与Rerun全时间轴PR准备
- 做了什么：新分支`feature/wall-sequence-rerun-timeline`基于`557b53f`，目标`alfa_v3_dev`；GitHub核实#20→#21→#22→#23仍open，需先合并依赖。按六项中文模板与Codex署名提交，沿用已确认Issue留空约定，不冒用历史Linear任务，不合并/打tag/强推。
- 提交范围：箱墙连续后放/正吸顶吸回退、各Demo失败观察、Rerun完整时间轴及其测试/当前文档/命令偏好。独立冗余IK启动修复只暂存失败标记相关hunk，其余启动修复、启动回归、本机交接、旧短脚本及未归属历史日志留在工作区。
- 验证结果：实际暂存树独立构建2包通过、该树CTest20/20、既有解析IK2/2、独立安装版本8类失败冻结、两项Rerun观察器回归通过；11项关键源码哈希与最终验收一致。重写54,830帧23.6816s（含FK/flush），全回调23.9639s，实际RRD全部23个link逐帧变换与箱体状态读回通过。初次中文验收目录触发rosidl路径解析失败，换ASCII临时目录构建通过，未因此改实现。
- 留给下个AI：完整25箱证据沿用本日`wall_rear_release/summary.json`，仅该显式参数组成立；Rerun未缩短约203s规划时间，非实机或GUI渲染耗时证明。本次复验/发布清单在`/home/astesia/Sevenova/日志/验收_2026-09-14/wall_sequence_pr/`，本机证据非远端附件；PR合并仍需规范要求的审查及CI。未停止用户Demo，未操作硬件。

## 2026-09-14 Codex / 箱墙Rerun逐箱增量接收验收
- 做了什么：在 `c0bf653` 基础上，每箱 `planWithFallback` 与失败诊断补帧完成后一次性发布完整段；不发布中间失败候选。后台继续规划，Rerun复用1024帧批量写入、不等播放；RViz逻辑、规划顺序、末态衔接与0.25°碰撞检查不变。
- 改了哪里：`v3_single_arm_box_extract_demo.cpp` 新增 `~/task_json_segments`（reliable/transient-local/depth32），原 `~/task_json` depth1仍保留最终完整快照；`sequence_timeline.py` 按publisher/task/segment与半开帧范围校验去重、缓存缺段并用最终结果补齐；`v3_single_arm_box_extract_viewer.py` 按轨迹时间切换场景，不按收包时刻提前删箱。新增分段单测、扩展安装版序列与RRD回读测试；更新 `docs/运控/IK/V3箱墙连续搬运测试.md`。
- 验证结果：2包构建、CTest20/20、分段/旧场景单测通过。安装版25/25成功、25段51154帧：首箱5.634s、首段写1.017s、规划213.226s、请求至全部写完214.727s；逐段与最终帧/场景完全一致，独立57717次碰撞/限位检查通过。真实17箱后失败及首箱失败均保持准确末帧；最终结果先到补27帧、后到分段去重；乱序漏段补48802帧、RRD全帧23链接和场景切换时间戳回读通过，真实ROS晚订阅完整快照一致。
- 原生UI：Rerun0.33.1隔离Xvfb/gRPC实测1000FPS追到2352仍Playing等待，收到第二段自动到6044；主动暂停6044后第三段/最终补齐均不恢复播放，速度不变；手动输入1234及重复结果保持定位。未新增UI强制播放/blueprint重置；鼠标拖动寻址未单独验证成功，不混称为自动通过。
- 留给下个AI：证据 `/home/astesia/Sevenova/日志/验收_2026-09-14/rerun_segments/summary.json`，`live_verified/` 是修复summary段字段后重新跑通版本，早期 `live/` 失败不能算通过。命令见文档（ROS域201，已bash -n及空白shell --show-args）；实时窗口不要设置recording_path，该参数现有实现会切换到文件sink。QoS最终补齐要求发布节点仍在且快照未被后续任务覆盖，不提供跨进程持久化。未改/覆盖本轮前已存在的交互IK及其文档/测试改动，未关闭用户现有Demo/RViz/Rerun。

## 2026-09-14 Git / Codex / Rerun逐箱增量追加PR准备
- 做了什么：已核验PR #24仍open、head为`c0bf653`，复用原分支追加本轮增量，不新建重复PR；目标`alfa_v3_dev`，依赖#20→#21→#22→#23仍未合并。中文提交与六项PR说明、Codex署名；沿用无明确Issue留空约定。
- 范围：只提交逐箱发布、分段校验/连续写入、三项测试、箱墙操作文档及本轮交接；其他AI的交互IK文件/文档/测试、短脚本、历史未归属日志保持原样。
- 验证：6项实现/测试SHA256与实测证据一致；提交前再次运行CTest20/20及分段/场景回归。25箱与失败实跑、RRD全帧/场景读回、原生UI等待数据/主动暂停证据见`rerun_segments/summary.json`；发布核验在其`pr/`目录。只提交待审PR，不合并、不强推、不打tag。

## 2026-09-15 运控 / Codex / V3 双吸盘主动悬挂模型接入
- 做了什么：从 `SevenovaHangzhou/robot_description` 的 `robot_v3_suction_chassis` 分支导入提交 `17f5bdc46b8f2580ee81aed919da7b404da3bdaf`，将默认整机描述更新为 V3.0.9 双吸盘主动悬挂版本，并保留双夹爪入口及现有 `world` 根链接兼容层。
- 改了哪里：更新 description 的 URDF/xacro、mesh、初始姿态、关节限位、mock ros2_control、查看 launch 和语义测试；同步 MoveIt SRDF、初始姿态、限位与 controller 配置。复用仓库已有且逐字节相同的 46 个 V3.0.8 机械臂 STL，未重复提交约 20 MB 资产。
- 验证结果：description pytest 25/25、MoveIt CTest 17/17、`git diff --check` 通过；新模型双臂刚性箱体平移规划成功（pairs=12、collision=24、frames=13）。
- 留给下个 AI：来源模型 `updown=[-0.5,0.5]m` 仅用于 description/mock/离线规划；真实执行桥仍保持 `[0.0,0.7]m` 安全合同，完成升降零位、方向、行程与吸盘 TCP 标定前不得直接用于实机执行。

## 2026-09-15 运控 / Codex / V3 双臂全搬运
- 做了什么：将双臂 25 箱全搬运适配 V3.0.9 双吸盘主动悬挂整机，补齐 V309 解析 IK、物理侧分配、共享升降、完整底盘包络、双负载碰撞与镜像折肘。
- 改了哪里：解析 IK、箱墙序列/规划节点、SRDF、launch、测试及 V3 单箱文档。
- 验证结果：V3 完整序列 25/25，双臂 10/10，fallback 0；MoveIt 20/20；IK 2/2；description 24 passed。证据 `/home/astesia/Sevenova/日志/验收_2026-09-15/v3_full_wall/`。
- 留给下个 AI：仅完成离线/仿真几何、运动学和碰撞轨迹验收，不代表实机吸盘动力学安全；实机前仍需升降、TCP、负载、吸附力和速度/加速度标定。
## 2026-09-15 运控 / Codex / V3.1.1 解析几何与远端流程整合
- 做了什么：以 `origin/alfa_v3_dev@8ce2e23` 的远端箱墙流程为主编排，接入权威 `robot_v3.1.1-hybrid` 模型；新增 `V311Left/V311Right` 解析模型并切换所有活跃 V3 Demo，`V309Left/V309Right` 仅保留历史回归。完整旧本地方案冻结于 `feat/motion-94-v3-local-extract-preserved@e33146c`。
- 改了哪里：当前整合分支 `feat/motion-94-v311-remote-integration`；解析几何位于 `alfa_robot_analytic_ik`，MoveIt 随机回代测试位于 `test_v311_analytic_moveit_fk.cpp`。远端主入口仍为 `v3_box_wall_grasp_demo.launch.py`；同一入口新增显式 `target_only + loaded_home` 研究策略，复用原有边插值碰撞、刚体附着和失败诊断，不复制第二套规划器。
- 验证结果：权威 description 源锁与本地快照均通过；Release 三包共60项测试零失败。左右臂各256组随机 MoveIt FK/解析 IK 回代，最大 FK 位置差约 `8.4e-12m`，最大 IK 回代位置误差约 `6.3e-8m`，平均解析 IK 约 `12.4～14.7us`。`target_only + loaded_home` 连续三次 ROS 启动规划成功，并通过异常退出/服务下线检查。
- 留给下个 AI：默认仍为远端 `full + rear_release`。旧 V3.0.9 后置落地目标在 V3.1.1 工作区可能无解析 IK，需要单独重新标定任务目标；禁止通过关闭碰撞或移动障碍伪造成功。`target_only` 明确省略其他24箱，只用于保留本地研究流程，不能作为整墙成功率证据。
## 2026-09-16 运控 / Codex / V3.1.1 初始与卸货命名姿态
- 做了什么：将用户确认的第一组姿态固化为默认 `home`（updown=-0.3m，左臂[155,-105,20,90,-90,-40,0]deg，右臂[25,-105,-20,90,-90,40,0]deg）；第二组固化为 `unloading`（左臂[-50,90,-50,50,20,-40,-60]deg，右臂[-130,90,50,50,-20,-40,60]deg），未列出的关节全部为0。
- 改了哪里：权威 description 变更位于 `robot_description` 分支 `feat/motion-94-v311-named-poses@62662f4`、Gitea PR #9；消费仓同名功能分支同步 description 哈希锁，更新 MoveIt SRDF、初始位置、mock ros2_control/Xacro 默认值及文档。
- 验证结果：两组姿态经运行中 MoveIt `/check_state_validity` 返回 `valid=True, contacts=[]`；description 36项测试通过；消费仓 Release 构建及63项测试通过，新增 `test_v311_named_pose_collision` 使用安装后的URDF/SRDF和FCL校验两组命名姿态无自碰撞、无越界。
- 留给下个 AI：建议先合并 description PR #9，再合并消费仓 PR；消费仓锁定内容源提交 `62662f4`，同步器允许目标分支 merge/squash 后在所有受管文件哈希完全相同时视为等价。

## 2026-09-17 进度管理 / Codex / 功能规范与PR #26历史补录
- 做了什么：按功能统一Issue、开发分支、逐提交评论及最终PR压缩合并流程；将PR #26范围内的搬运基础设施/完整箱墙搬运补录为Linear MOTION-201，负责人文子轩，10条历史提交评论+最终验收评论后设为Done。
- 改了哪里：精简`.ai_teamwork/LINEAR_WORKFLOW.md`；完成锚点为GitHub PR #26及主干压缩提交`8ce2e234a2e269b6cfd662e410d0c5f279c350e8`，实际合并日期为2026-09-15，2026-09-17仅为补录日期。
- 验证结果：GitHub确认PR已合并，Linear确认负责人/里程碑/Done；使用历史本机测试结论，不声称本次重新仿真或实机通过。 文档diff check通过；原9个工作文件中8个哈希未变，`v3_single_arm_box_extract_demo.cpp`在本次期间被并行开发更新（未由本任务写入或回退），分支和HEAD未变。
- 留给下个AI：多维算法开发与比较、回归点算法是两项独立的开发中功能，本次均未创建或更新Issue；不把`ec36fc0`或回归点/转运缓存代码和测试数据计入MOTION-201。未切换分支、未提交/暂存/修改现有功能代码；规范文件尚未提交。
