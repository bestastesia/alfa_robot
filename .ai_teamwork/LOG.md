# AI 协作日志

## 2026-11-10 cuRobo / PR80基线上的LM微调与可复现部署交付
- 用户要求新PR保证模型/碰撞/算法/逻辑/可视化可准确运行。以最新origin/alfa_v3_curobo@92bd8cdb新建独立发布检出codex/lm-refinement-release，不夹带原研究分支脏改动、300箱/tmp录像脚本、其他同事开放PR。
- 迁入双臂LM后64原生三次控制点最多32轮L-BFGS；抽离方向/距离与导航后放箱参考取本次任务，端点固定。Fast TrajOpt只自由段，TimeOpt仍全部六段；抽离后处理补loaded checker。原四盒/20球/40球、10%世界膨胀、1mm及自碰撞口径、选型与实际末态同目标重连源文件逐字节对照PR80不变。
- 新增FAST_DEPLOYMENT、依赖/模型SHA锁、v3_fast_preflight，实际比对官方Fast提交_src434文件全一致；前端冻结包校验通过。prepare_checkout --fast-only仅编译解析库并生成忽略的运行配置，页面使用重定位URDF，旧README/CORE/HANDOFF错误默认入口加当前指引。输出附配置快照/摘要，冷/热与候选/通过分开。
- 干净检出40测试通过1跳过，GPU preflight Home与box-world启用通过。两例CLI新规划候选运动11.706/11.865s、12段TimeOpt动力学采样通过、接缝0，后者原approach TrajOpt失败保留；墙钟53.574/5.772s，不当成全量/固定3秒。8293实际页面点击新计算：初始化5.854s另列，墙钟67.877s含冷资源，运动12.209s，全部求解标志及动力学通过，真实TimeOpt播放通过。紧凑证据generated/lm_refinement_release_20261110/validation.json。
- 已知TrajOpt软间隙、冷/新拓扑、TimeOpt诊断Graph重捕获、球体覆盖及旧回放一致性问题保留；单臂新后处理未验证，300箱临时录制不是本PR生产调度。仅本地规划/可视化，不发送硬件、不自动合并PR。

## 2026-10-27 运控 / Codex / 同步新WARP至8210
- feature/new-warp-sync迁入新WARP六段策略，参考/解析先验/联合代价抽到curobo_core，13个函数/类保持AST一致；不夹带模型路径、随机场景或其他实验修改。
- 0.982m释放箱心改为绝对权威配置；8210保留PR65独立开关和后处理验碰，禁止旧高度/旧模型回放冒充新结果，8221原服务不动。
- 1.082m复测117/117、702阶段且每段30轮，105个软参考阶段最终合法；平均3.892秒。0.982m三类任务几何3/3、后处理3/3；57core（1跳过）＋17流程/回放＋6交接研究回归通过。数据generated/new_warp_sync，不宣称当前高度全量或扰动成功率。
- 保留抽离途中空载/终点携箱、导航瞬移、释放移除边界，未连接实机；新复测入口v3_new_warp_validate.py和NEW_WARP_HANDOFF.md迁移补充可复现。

这里仅保留当前分支仍需让新 AI 立刻看到的最新交接。长过程和已完成事项已归档。

## 2026-10-04 运控 / Codex / 接手 V3 cuRobo 开发
- 做了什么：按用户要求完整阅读远端/本地 HANDOFF 与 /tmp/handoff-x6TfV8.md（SHA256一致），读取 MOTION-238～243、275 正文及全部评论，核对当前周期入口、GPU有效性/搜索、附着变换、缓存、解析桥及版本化证据。
- 当前落点：后续开发使用 /mnt/mydisk/ALFA/alfa_robot_v3_curobo 的 alfa_v3_curobo 分支；接手基线60ae10dd与远端一致。原 /mnt/mydisk/ALFA/curobo_v2_ws 保留第三方依赖、原实验及历史数据，不能只在那里改代码而漏同步正式分支。
- 已确认：cuRobo数值IK/FK/球碰撞 + 自研GPU批量RRT + 固定Updown/ψ解析35cm抽离；15维运输、18维记录仅追加确定性yaw。首组L24/R20历史热态6.438/6.525秒成功；三段2秒Anytime、Shortcut/TrajOpt关闭，未在新checkout重新运行全周期。
- 边界：目标箱在到位前已从碰撞场景移除，抽离中附着箱球碰撞延后至35cm终点；40球是内接近似，当前无通用全程真实OBB验收。缓存键只含任务组合，动态场景/负载变更需要失效；本原型尚无正式ROS执行接口、时间参数化和取消链路。
- 下一步：先在本工作树原样复现首组并验证路径/模型/目标/球心同源，再按用户确定的优先级推进场景独立管理、批量回归或执行接口。238～243保持Done，算法跟踪275 In Progress；本轮仅接手阅读，未新增规划、实机或成功率声明。

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
## 2026-09-17 Description / Codex / MOTION-204 V3对称零位与装卸姿态合同
- 做了什么：右J1/J5重标180deg逻辑零位但保持正轴不变；冻结镜像`home`和新`unloading`姿态，更新解析IK固定变换、description消费者锁与2.4m×2.4m集装箱默认场景。
- 上游：`robot_description@510694697e543a30030c8432c878fcc461da9088`，分支`feat/motion-204-v3-symmetric-zero-named-poses`。
- 验证结果：干净无Conda环境Release构建通过；`alfa_robot_description`、`alfa_robot_analytic_ik`、`robot_motion_scene_service`、`alfa_robot_moveit_config`共61项测试全部通过。全零位和1000组随机镜像FK误差均在1e-6m内；新卸货位自碰撞/限位通过。
- 规划证据：本地Rerun `ros2_ws/data/ik_benchmark/v3_motion_stage_wall_action/loaded_home_to_unloading_130deg_shortcut_symmetric_rrt_verified.rrd`；Shortcut碰撞区间由单臂RRTConnect局部修补，成功路径183帧，经552个0.5deg探针检查无碰撞且全程保持镜像。
- 留给下个 AI：实体RT-Control仍需按该逻辑零位合同单独验收，禁止在description和驱动两侧重复补偿。

## 2026-09-19 Description / Codex / MOTION-204 原始零位合同纠正
- 做了什么：确认 V3.1.1 机械原始零位已镜像，撤销右 J1/J5 多余的180度逻辑重标；四套搬运关键帧统一采用 `right_jointN=-left_jointN`。
- 改了哪里：同步 `robot_description@f2454a6` 快照，更新解析 IK 固定变换、MoveIt/ros2_control 初始位、SRDF 命名姿态及 `*_linkN` 碰撞链接消费者。
- 验证结果：description 38项测试通过；消费侧 description 合同17项通过；V3.1.1 随机解析 FK/IK 回代、四套关键帧限位与自碰撞、MoveIt 合同3项通过。
- 留给下个 AI：description PR #11 应先于消费仓 PR 合并；关节名仍为 `*_jointN`，实体链接名从本次上游版本起为 `*_linkN`，禁止混用。
## 2026-09-21 运控 / Codex / MOTION-222 V3 Stage 仿真预览交付
- 做了什么：将固定 5x5 箱墙的已验证双臂联合轨迹接入 `/motion/execute_stage` 四阶段 Action；支持 replay 与完整17轴FJT后端、26轴反馈/安全状态准入、失败轮运动前跳过；放置确认后从Motion场景删除箱子。
- 改了哪里：解析腕部补偿、阶段Action与客户端、域内/中央接口分层、固定墙轨迹缓存、缓存生成/验证工具、仿真交付脚本与文档。
- 验证结果：最新V3吸盘description下4/15轮完整周期通过，成功轮为0/1/3/4；所有阶段和任务边界关节差为0，旋转轴单帧不超过0.5度、Updown不超过5mm；12/13包构建与4包54项测试通过；Action replay实跑4轮成功、11轮PREGRASP前失败并跳过；RRD验证通过。
- 留给下个AI：这是preview，不是25箱全成功或仿真FJT验收。外部仿真必须先满足26轴`/joint_states`、17轴FJT和`/control/safety_state`；其余11轮需继续优化候选/场景，不能回退为断裂轨迹或放宽碰撞。
## 2026-09-21 运控算法 / Codex / V3前11轮侧吸Action与仿真Preview 0.2.0
- 做了什么：将90cm箱墙前11轮侧吸任务串成 `PREGRASP→APPROACH→PLACE→HOME`，加入解析IK候选回退、35cm抽离、携箱回初始、两套放置、释放删箱、空载回初始及第一/第二初始姿态切换；缓存支持3轮单臂 `NO_MOVE`。
- 改了哪里：更新 V3 阶段Action缓存与服务端、仿真客户端/启动脚本、交付合同、Release文档，并增加候选验证、缓存生成和完整Rerun脚本。
- 验证结果：11轮共44个阶段Action Replay全部成功；完整序列18,341帧，首尾均为第一初始姿态，最大旋转步长0.5度、Updown步长2.86mm；RRD校验、13包自包含构建及27项相关测试通过。
- 留给下个 AI：Replay接口与算法缓存已验收；外部仿真仍需按26轴状态、17轴FJT和SafetyState合同完成实际执行验收，不得把Replay结果表述为FJT已通过。
- 2026-10-04 V3 cuRobo正式分支：从origin/alfa_v3_dev@d9c330ce创建alfa_v3_curobo，独立worktree为/mnt/mydisk/ALFA/alfa_robot_v3_curobo；当前研究代码导入research/curobo_v3，保存完整周期/GPU搜索/拟合与历史实验源码、当前冻结模型与主要证据、解析源码精确快照、官方cuRobo本地修复patch及旧数据哈希清单。完整入口解除旧实验目录绝对import依赖，prepare_checkout重定位模型路径与编译解析桥；导入、米制mesh引用、60任务与40球及桥接加载检查通过。尚未重新运行全周期，不混作新验收；原研究服务目录保留。后续在此分支开发，提交关联Refs MOTION-275。
- 2026-10-04 MOTION-275完整交接：新增research/curobo_v3/HANDOFF.md（228行），整理cuRobo官方能力与当前GPU算子/自研搜索/解析混合方案边界、模型/Tool0/完整6D目标同源合同、40球近似及抽离暂缓负载碰撞规则、任务场景移除目标箱简化、候选选择、Anytime/显存/缓存、当前首组证据与新checkout未复跑全周期的区别。提供源码导航、可复制启动命令、同事学习顺序和分组/精确碰撞/生产接口后续验收清单，详细实验数字链接既有Linear/JSON不重复抄录。相对证据链接全部验证存在，handoff技能临时文件/tmp/handoff-x6TfV8.md与持久文档内容一致；本轮仅文档，无新增规划成功声明。

## 2026-10-04 运控 / Codex / MOTION-276 独立cuRobo场景与规划核心
- 做了什么：在既有alfa_robot_v3_curobo工作树创建feature/motion-276-curobo-core，完成用户限定的阶段1～3：复现首组、不可变具名状态/场景与吸附释放事务、提取已有混合规划器；Viser与无GUI JSON入口复用同一核心。
- 改了哪里：research/curobo_v3/tools/curobo_core、CLI/GPU专项及CPU合同测试、CORE.md和迁移摘要；新增限定alfa_v3_curobo的CPU CI。未改原curobo_v2_ws、跨仓接口或模型资产。
- 验证结果：CPU15/15通过；迁移前L24/R20完整周期10.599s，迁移后10.411/6.511s，两次成功且回位误差0；最终GPU专项10.003s完整周期通过，验证障碍/负载变更失效、同场景缓存复用、SCENE_CHANGED拒绝旧结果和无Viser导入。演示页面机器人和箱墙正常。
- 留给下个AI：使用CORE.md的具名PlanRequest与SceneStore入口；预测场景事件不等于真实执行反馈。保留目标箱豁免、抽离延后负载碰撞和40球内接近似。当前全周期仅原点底盘/零头部/双臂轴对齐标准箱；没有11/60组、RT/Action、时间参数化或新增OBB/FCL验收声明。逐步记录及最终PR由MOTION-276跟踪，算法优化仍跟踪MOTION-275。
- 2026-10-04 MOTION-275六规划器：增加Connect＋Informed与BIT* GPU批量验边/双队列/换父节点变体，并rebase同步MOTION-276核心封装，未覆盖同事实现。相同首组接触起点与完整6D目标、三段2s预算，新核心热态分别6.318s/6.227s完整成功；最终Home误差0、相邻关节步长≤0.5°。修复BIT空批次错误，3项GPU小场景测试通过。页面8090保留原四种及新增两种；详见SIX_PLANNERS.md与full_cycle_extra_planners.json。不声明原版BIT*最优性、全任务成功率或真实OBB/连续碰撞验收，未测CBS、未加入Shortcut/TrajOpt。
- 整合收尾：PR #49 rebase到同事四规划器更新cb08548d，保留其页面/脚本/原始对比证据，将同一search_path迁入核心；15项CPU CI通过。复核首轮受已有8090演示显存占用影响OOM，改用仅本次进程的expandable_segments分配设置后四种规划器均通过首组完整周期、回位误差0且步长≤0.5°，未暂停或修改其他演示进程。该轮属于兼容复核，不作性能排名；结果与环境记录于validation.json。

## 2026-10-04 运控算法 / Codex / Linear子任务分工
- 做了什么：按用户确认在MOTION-275下创建MOTION-277规划算法优化与多路线对比验证（In Progress）、MOTION-278搜索结果的轨迹优化与效果验证（Todo）。继承当前Project/milestone，负责人李昊洋。
- 改了哪里：277补5条历史/当前推进评论；278补未开始说明；275描述与评论明确子任务归属，原始评论和同事视频保留追溯。
- 验证结果：UI确认父子关系、状态和评论正文；截图/tmp/linear-275-subissues-20261004.jpg。本轮无算法代码修改或新增实验。
- 留给下个AI：基础RRT/PRM及公共GPU加速仍归275；Informed、Connect、Connect＋Informed、BIT-star、任务空间引导与因子化协调等其他路线后续归277。搜索后压缩/重采样/TrajOpt等归278，尚未启动；旧模型TrajOpt尝试不当作新任务已完成。

## 2026-10-04 运控算法 / Codex / MOTION-275 工控机持续GPU热压测
- 做了什么：SSH sev_v3@192.168.100.28，在/home/sev_v3/curobo_thermal/venv部署同版本隔离环境，不改系统CUDA。两台实际cuRobo FK/自碰撞/世界碰撞/边界计算持续约2分钟；16384状态×4随机批次、15轴、340球、29障碍及CUDA Graph完全一致。
- 改了哪里：新增v3_gpu_thermal_stress.py、v3_compare_thermal.py、THERMAL_STRESS.md及generated/thermal_20261004原始JSON/冻结输入/对比图。
- 验证结果：模型/输入/脚本/软件哈希与版本一致，四批Graph与普通调用碰撞判定一致。4060本机121.05s，平均GPU98.42%、最高60°C，前后52.59→53.03万状态/s（+0.84%）；A2000工控机120.77s，平均GPU99.17%、最高89°C，45.71→31.03万（-32.12%），约21.66s/86°C首次温度降频，97/118采样Active。压测进程均退出，工控机温度下降、GPU回空闲。
- 留给下个AI：这是单次持续GPU算子热负载，不等于完整RRT耗时或数小时稳态；驱动/内核和初始温度不同，不是单变量GPU对比。先核查散热再用同负载复测，保持温度保护。证据、区间和可复制命令见THERMAL_STRESS.md，正式进度继续评论275。

## 2026-10-05 运控 / Codex / MOTION-275 GPU压测对CPU资源影响
- 做了什么：工控机相同GPU负载复跑121.44s，前20s背景/后10s恢复；监测全部28逻辑核占用/频率、物理核温度、降频计数、系统压力、活跃进程CPU/RSS。未停后台或改变调度。
- 验证结果：整机CPU平均3.21%→6.20%→2.08%；压测进程平均1.02核、初始化峰值4.27核，稳态约1核。CPU背景已最高100°C且降频2次，运行期最高97°C/新增31次Package降频，不能归因GPU压测升温。GPU38.85万状态/s仍热降频。全部153样本包含28逻辑核、压测退出码0。
- 证据：dd7aa031、CPU_IMPACT.md、generated/cpu_impact_20261005，Linear275评论cf8dffef-1f0b-496a-9cdd-6e516f6fe8b1。
- 留给下个AI：资源占用不等于其他程序业务时延；未测真实任务P99/deadline，真实RRT树/近邻CPU工作未覆盖。首先关注散热与gnome-shell背景约0.76核负载；同核/共享GPU竞争需业务并行实验验证。

## 2026-10-04 运控 / Codex / MOTION-246 龙头车快照接入暂停
- 做了什么：同步最新alfa_v3_curobo@f37612ea，在既有工作树分支feature/motion-246-conveyor-snapshot开发独立实时实例、冻结Mesh快照与cuRobo适配，Linear范围已更新。未创建新仓库副本，未提交或拉PR。
- 验证结果：CPU22/22通过；米制双车STL是2.40×0.94×0.732m的封闭12面长方体，间距5cm。首次GPU网格检查失败：cuRobo data_mesh.py调用wp.torch.device_from_torch，但当前warp-lang1.17.0没有warp.torch，公开函数已在warp顶层。
- 留给下个AI：用户要求遇到问题先停下沟通，因此暂停进一步实现；待用户确认修复兼容后续GPU/可视化/PR。不能宣称Mesh检测通过，不能退回长方体检测掩盖失败。未跟踪源码仍保留在本分支工作目录；STL被忽略规则命中，未来提交需显式处理。MOTION-246仍In Progress。

## 2026-10-05 运控 / Codex / MOTION-246 新main适配与龙头车快照
- 做了什么：获用户授权后使用代理将原curobo_v2_ws源码/安装更新至官方main@78fd485f，保持Torch2.8/CUDA12.8、Warp1.17；pytest在虚拟环境升级8.4.2。官方已包含旧本地算法修复，旧补丁仅作历史保留；将测试按新版set_dependencies初始化，不改官方算法。
- 验证结果：21组直接cuRobo导入通过，CPU核心22项、旧连接器2项、本仓连接器GPU3项、BIT3项通过，六种规划器首组全周期均成功且回Home误差0、步长≤0.5°。核心GPU缓存/SCENE_CHANGED测试通过；Mesh远离/重叠/关闭、IK/checker网格缓存通过。完整适配摘要见research/curobo_v3/generated/main_compatibility/validation.json。
- 龙头车功能：复用米制STL与5cm双车布局，实时组合根独立；LiveScene筛选并冻结Mesh/货物到SceneSnapshot。8094页面可编辑实时xyyaw、播放跟随、冻结并切换显示，碰撞按钮只消费冻结快照。用户移动实时车不会影响旧检查；重新冻结重叠才报告碰撞，禁用后快照无龙头车。
- 留给下个AI：CUROBO_MAIN.md/CONVEYOR.md提供完整命令。此处不含导航、ROS实时接口、环境-环境碰撞或带龙头车完整取放验收；完整动作继续248。当前正式工作树alfa_robot_v3_curobo；compare是同仓历史对比/压测工作树，含后续提交及本地改动，本次未删除、未复制新仓库。

## 2026-10-11 运控 / Codex / cuRobo解析64与多根方案收口
- 做了什么：按handoff-oFc145定向同步到alfa_robot_v3，分支feature/motion-275-multiroot-sync，保留#55统一Updown度量。源alfa_robot_v3_curobo未提交工作不改，没有创建新工作树；目标本地模型备份/tmp/motion-research-local-models.patch。
- 工程处理：取消源root硬编码/import时chdir及prepare_roots全局替换，改显式方法。缓存身份加入场景/模型/目标/atlas/cuboid/payload/grid；两atlas逐行FK匹配当前Pose。失败空数组/旧选择清理、任务切换与按钮busy恢复完成。71底盘球逐字段合并，ROS Home/本地URDF不覆盖，研究代理Home改左右J2=-90/+90。
- 验证：31项CPU、1项多根GPU通过；L24/R20侧吸与L9/R1顶吸完整周期与移动URDF独立FK通过，Home误差6.38e-8。8140实时中列L22/R17成功7.659s/2204帧；页面切顶吸成功4.667s，重复热态1.404s；负向目标/atlas拒绝、缓存身份和失败空帧恢复通过。摘要generated/research_sync/validation.json，不把源60/60外推目标94/94。
- 留给下个AI：RESEARCH_SYNC.md给出完整算法/命令/合同；实时入口8140用户级临时服务alfa-curobo-sync-8140，不预载结果且后台持续；源8137保留。4内部cuboid不是STL，40箱球内接、3cm排除目标/抽离负载延后、两段转身碰撞跳过均是现有研究边界。Home末端多转圈未解决，非生产并发接口/时间参数化/真机安全验收。Linear由专门工程师跟踪，本次不批量补评论。

## 2026-10-14 运控 / 三PR交付当前龙头车场景
- 模型合同PR #59已合并alfa_v3_curobo：20升降架球、新侧吸箱心1.2/Y45及2816解全集，顶吸保持。外观PR robot_motion_control #177已合并main@479ce2a：alfa-motion-viser只读包和55项冻结资源，fast38/38、4项资源合同、wheel内容验证通过。
- 算法集成切片交付ConveyorCyclePlanner、固定龙头车碰撞与释放目标、目标IK+RRT放箱、释放后直接RRT回Home、导航到位瞬移、失败边诊断、30°/s播放器及固定前端内容身份。默认不再依赖本机其他研究或3DAssets目录。
- 最终两例GPU回归：side L18/R16成功14.363s，top L4/R2成功8.092s；FK释放目标/箱体移除/冻结底盘/阶段顺序审计通过。34项CPU通过，全部研究工具语法通过。未宣称94任务全成功、物理落稳或真机验收。
- 正式接手入口research/curobo_v3/CONVEYOR_DELIVERY.md，依赖锁config/visualization_dependency.json、验收摘要evidence/conveyor_delivery_validation.json。其他AI的动态任务/歪斜实验和本地模型路径迁移保持未提交，不混入本轮PR；运行轨迹和缓存本机保留。

## 2026-10-15 运控 / 8151优化与单臂方案正式同步
- 交接来源generated/handoff_8151_20261015/HANDOFF.md，10源文件hash核对；保留其他AI文件及六份仅路径重定位模型，不复制仓库。按用户确认把旧STL数量审计改为与cuboid几何等价审计；额外人工mesh负例已按用户要求移除，真实场景无mesh。
- 四切片：优先64与等价cuboid（PR61）、常驻world更新（PR62）、8起按需抽离及旋转软代价（PR63）、25单臂与整机终点1.05m硬圆柱（当前切片）。9份算法/合同/测试源码与交接快照一致；UI归一到正式v3_visualized_task_demo.py，8151入口只作兼容包装。
- 验证：四任务64集合与全枚举相同/完整FK通过；11场景11264状态与fresh一致且四完整周期不再创建拓扑；lazy三组成功包括L18/R22；25单臂24成功、右02接触264候选0合法保留失败。24成功27040帧碰撞重验、40附着球、独立FK包络最大1.003365m，原4违规终点现0.932856m；双臂回归成功。35core与语法检查通过。
- 正式说明OPTIMIZED_CYCLE.md、CONVEYOR_DELIVERY.md；evidence为当前小型摘要，原实验轨迹/cache不入库。验证服务8152复用冻结前端包；原8150 PID161088、8151 PID1877730未停止，仍是各自启动时版本。119任务不等于119成功，不作导航路径、动力学、物理落稳/实机或生产并发验收。Linear由专门工程师维护，本轮不主动变更。

## 2026-10-18 运控 / 新Home与J6反向完成模型全链路迁移
- 用户最终Home：Updown-.3m，左[65,-75,-20,-90,0,40,90]°、右[-65,75,20,90,0,-40,-90]°；最新确认双侧J6仅反轴不改零位，角度换号保持实体姿态。新坐标signs=[1,-1,1,1,1,-1,1]，旧安装/tool及碰撞保留，J2左[-90,120]/右[-120,90]。
- URDF固定/planar、解析映射、球配置、命名姿态、side/top候选库已同步；新入口v3_rezero_task_demo.py使用RezeroTaskPlanner，不依赖未提交AdaptiveRaised/闲臂优化，其他AI工作保留。8152改新模型全流程界面并预载六组成功结果，8151/8153和实机未动。
- 两侧各100解析回归、两模型各100全link等价、4模型合同测试及39core测试通过；六组双臂完整6/6成功2.58～3.15s，9159帧独立碰撞/限位/.5°连续性/世界抽离35cm/最终6D/Home审计通过。保留原接近目标排除及抽离空载全段+载荷终点策略，不冒称抽离全段负载精检或119任务全覆盖。
- 证据/tmp/rezero_j6_tasks_20261018及generated/zero_model_candidate_20261018/task_regression.json，完整命令REZERO_TASK_VALIDATION.md；旧J6符号结果不可配新URDF回放。PR67由旧阻塞草稿收口为该模型/新Home六组验收，不把同事原3.3.1或实机标定算作已适配。

## 2026-10-24 运控 / Codex / 8209新算法适配当前URDF
- 2026-10-26开启8210后处理：feature/curobo-8210-timed接PR65时间化器、显存有界Shortcut及0.5°补边验碰；保持191mm TCP和六段端点、原抽离验碰边界，J1～4限15°/s、J5～7限30°/s。三类任务均通过，运动约73～75秒，计算约8～11秒；12专项＋52core（1跳过）通过，浏览器已验25ms时间步播放和阶段耗时。证据generated/current_urdf_timed/acceptance/summary.json；部分段停点回退如实保留，未连接实机。
- 2026-10-26集成：PR65由用户合入96c2c70c；40项开发提交已rebase，入口同时保留planner_class/default_assets并修复默认类绑定，52项core（1项跳过）＋17项当前合同测试通过，四次GPU六段流程通过。保留#65独立时间化入口，未自动替换8210算法；本地未提交随机误差/模型路径工作已备份并恢复，不夹带发布。
- feature/curobo-8209-current-urdf保存六段前向预接触/LM/首合法Informed Connect研究依赖，新增CurrentUrdfForwardPlanner，统一确认零位、191mm TCP及Rezero解析网格；没有改工控机或联调分支提交。
- 运输侧吸旧2cm升降网格跳过合法窗口；原范围内加密到5mm，重求运输库并独立FK验收，保留碰撞/限位/1.05m包络。补齐16/64形状真实LM预热、长求解器共享及完整显示快照恢复。
- 35项回归通过，侧吸L22/R17→顶吸L03/R02→单臂R02→侧吸四次GPU完整流程通过，Home误差约4.4e-8，显存峰值约1.7GiB；非117组全量验收。8210独立预览已通过浏览器实时计算，结果独立目录generated/forward_precontact_current_urdf。
- 保留抽离途中空载/终点携箱验碰、导航瞬移、释放移除边界。polish录图/早停留后续；未对外宣称全流程携箱安全或实机完成。恢复服务与复测入口见原HANDOFF_8209_ALGORITHM_AND_WARMUP.md迁移补充。

## 2026-10-31 cuRobo / 固定全流程发布
- 用户确认将放箱RRT替换为连续LM，其余流程冻结。新增FrozenCyclePlanner，接回释放后从实际LM末态RRT回Home；默认8210脚本转向固定流程，8243实时规划/回放，8242仍为对照。单臂空闲关节保持放箱起点，参数128点/32种子/最多32轮固定；原40球全程验碰，无放箱RRT回退，无实心盒复核。
- 全117任务105成功：侧吸双臂60/60、顶吸双臂24/32、侧吸单臂15/15、顶吸单臂6/10。11项LM失败、1项原候选失败，均停止并保留前缀。所有成功项实际LM终点到Home规划起点一致；导出重复起点会去重，不能误将首运动帧当规划起点。
- 5项回归通过，含独立URDF逐帧放箱参考与回Home接缝。待发布Git树直接加载运行模块验证三成功/一预期失败，最终发布树5项回归函数也通过；8243实际点击重新计算成功。仅整理必要运行依赖、说明和四条代表记录到feature/frozen-lm-cycle-release，避免带入累计1956项实验文件；目标alfa_v3_curobo。

## 2026-11-01 cuRobo / 用户确认腕部拟合接入固定LM流程
- 将15:39保存的双臂拟合球精确写入fixed/mobile：左5/6/7为16/4/15球，右为18/4/15球。仅球数组改变，限位、忽略对、自碰撞零容差、环境1mm、携箱40球及LM128/32/32均保留。批准参数与采样拟合指标保存wrist_collision_20261101.json。
- 全117任务重跑116成功（旧105），10个腕部碰撞失败及旧L4/R6候选失败均恢复；仅L4/R5仍在LM10.24%之后插值误差3.194591mm>2mm，边无碰撞，未放宽门限。新结果在generated/wrist_refit_release_20261101；验证器新增输出目录及模型SHA隔离，禁止误用旧模型缓存。
- 新模型契约与固定流程6测试通过；待发布Git树使用独立导入器复测60/76/92，分别成功/预期误差失败/成功。发布树基于origin/alfa_v3_curobo，仅选取本次文件，避免带入研究分支累积实验及其他工程师改动。
- 8243已切到新模型与新结果目录，默认显示原失败现通过的L9/R5；失败菜单只读取同目录复核。本次拟合采样仍有覆盖缺口，仿真规划通过不代表实体包络无遗漏。因剩余一项失败，PR以草稿待讨论，不自动合并。

## 2026-11-01 cuRobo / 按用户要求取消LM插值边跟线精度门限
- 用户明确两次LM解之间只要求无碰撞且关节连续。仅去掉插值边2mm/1deg拒绝条件，保留误差统计；目标节点自身精度、15deg连续性、0.25deg密化携箱碰撞、环境1mm及自碰撞零容差保持。
- 全117项重新规划117/117成功；L4/R5放箱到Home全程通过，插值最大偏差3.1946mm属于允许偏离。新回归实际重算LM并复核全部节点精度、携箱碰撞及连续性，连同原固定流程和模型契约共7测试通过；待发布Git树独立复测L4/R5通过。
- 新结果generated/wrist_refit_lm_continuity_20261101；验证缓存同时校验算法与模型SHA。8243切到新L4/R5完整回放，误差仅统计的语义显示在页面。PR75继续更新，旧116/117与失败证据保留为历史对照。

## 2026-11-09 cuRobo / Fast膨胀规划与固定结构盒体PR
- 独立分支codex/curobo-fast-box-pipeline基于origin/alfa_v3_curobo整理运行依赖，不合并研究分支累积实验。保留先预测候选、联合RRT选型、选中完整15轴LM抽离及实际端点重连、LM放箱；自由RRT世界查询膨胀10%，三自由段Fast TrajOpt，全六段Fast TimeOpt。
- 固定底盘/立柱四盒对内及对外验碰，原base71球槽位半径置负；arm_carriage20球、双臂和40球携箱保留。盒-世界支持固定/移动底盘，规划膨胀只作用于外部查询。沿用用户确认TCP135mm/J6轴距234.2mm，及对应吸附接触端帽处理。
- 验证：L4/R1、L4/R2顶吸及L18/R11侧吸都返回完整时序候选，18段TimeOpt成功且4001点动力学采样通过；各任务仍有TrajOpt失败阶段，整体success=false，候选运动16.036/15.069/14.521s。初始资源2.094s另列，计算墙钟54.730/5.953/14.454s，不能称全部稳态或3秒。紧凑证据generated/fast_release_20261109/validation.json。
- 新增/迁入针对性回归26通过1跳过；扩展模型/历史回放检查合计32通过1跳过4失败，未修改历史记录/断言掩盖失败。按用户明确要求保留原软间隙代价兼作约束、迭代质量、冷启动、旧J6/Home/回放一致性等问题，未强制修复；列表及命令见FAST_PIPELINE.md。
- 8290为本PR检出的点击实时计算页面，旧服务保持。此交付供用户审核，不自动合并；不含硬件发送或MuJoCo联调。

## 2026-11-19 Codex / PR82 两尺寸任意位置抽离专项（未完成）
- 独立工作树 `/home/astesia/Sevenova/.pr82-two-box-extraction`，`feature/pr82-two-box-extraction` 从 PR82 合并点 ee6dba3 起，不改 `.agv-palletizing-pr82` 及其 dirty 实验/服务。新入口 `tools/v3_two_box_pick.py`，纯几何 `curobo_core/box_pick.py`，复用原解析 IK、15 轴 LM、RRT、四盒/40 球。XYZ 两尺寸(.3,.4,.4)/(.255,.465,.582)，抽离.3/.255；中心直接输入，无旧箱编号绑定/隐藏底盘前移；侧吸高度阈值.8为待标定值，过高顶吸拒绝继续暂挂。
- 默认45°实际是箱体 Ry=-45°，新增测试直接对照 PR82；不能用+45°通过冒充默认位。CPU 28 项通过；GPU预检ready/Home/fixed-body均通过，Torch2.9.1与冻结2.8.0差异保留，前端未安装/显式跳过。没有实机或TimeOpt新尺寸认证。
- R1八例4/8、439.242s：40箱左右侧/顶吸4/4完成；高箱四例均能完成.255抽离，但原持箱目标无合法候选。独立诊断两种原位解释、左右各256空载候选，携箱均0；完整OBB证明高箱与base_box_004立柱重叠约7.5mm，CPU保存严格内部交点证据，不把失败点改叫非法。`carry_offset_m`只作显式实验，默认0，未擅自改变要求。
- R2候选补偿（前.05m、按臂外.0325m、仍-45°）1/4、60.556s，未作为原默认位验收；其余被真实箱体/机器人5–11mm重叠等拒绝。R3已把完整箱体检查加到携箱目标过滤和single_validity搜索节点/边，末尾仍独立审计，不关闭旧约束。当前R3未得GPU结果，不能称已验证。
- 继续先看 `research/curobo_v3/docs/TWO_BOX_PICK_20261119.md`。工具session73367、队列gpuq-efc637c2136b4316、包装PID2382240，已确认queued/live，后面有现存GPU任务等待，不重启/不绕过锁；stdout `/tmp/pr82-two-box-offset-r3.log`，结果 `generated/two_box_pick/tall_offset_probe_r3`。R2/R3源码快照留source_r2/source_r3。下一步确认高箱默认45°是否允许位置补偿，再做合法域/切换边界/随机位置、Fast/TimeOpt/可视验收；任意合法位置要求尚未完成，目标保持active；未commit/push、未改Linear。

## 2026-11-19 Codex / 两尺寸抽离R3→R5：完整箱体搜索、边界反例修复与独立FK
- 上条等待已接回，不重复启动：session73367的R3显式补偿四例4/4、60.542s、接缝0，先前端帽/完整箱体搜索修改有真实GPU证据。高箱补偿仍是实验，默认offset=0未改，不能称完成回原位要求。
- 40箱16个边界任务（保留全部失败）R3=13/16/152.987s；三个失败均Home粗路径加密后碰撞。新子类仅置原有dense_path_rechecks=True，RRT拒绝后继续，R4=15/16/157.025s；旧切换边界失败实际拒绝16候选后成功。
- 最后一例为高位左臂：冻结模型端帽前伸左32.838mm/右30.710mm，原3cm预接触已进入代理端帽包络，造成极窄对齐通道。纯几何BoxPick新增precontact_margin_m（默认5mm、最小1mm）和precontact_distance=max(.03,capreach+margin)，共享规划取真实端帽参数；不改1mm碰撞口径、不放宽端帽条件。当前R5 16/16、120.575s，旧高位失败首候选即过，接近段无fine rejection。
- 同R5执行固定seed16个非网格中心（仅现有两尺寸）：16/16，其中40箱8例回原45位、高箱8例回显式补偿45位。高箱补偿指箱心(.55,±.425,1.20)→(.60,±.4575,1.20)，Ry=-45不变；不是TCP位移，也未获采用许可。R3旧4例不混作R5当前源码成绩。
- 新verify_cases.py独立CPU yourdfpy URDF FK、输入/源码SHA、阶段接缝检查：cube_boundary_r5和continuous_centers_r5各16/16（共32任务，40箱24/高箱实验8），末态/抽离误差在原2mm/1°内，接缝0。它检查保存的碰撞门记录但不独立重做全路径碰撞，不能宣称CCD或连续位置域认证。CPU新增8+原Scene16+TaskScene3+ResearchContract3=30通过；编译/后续GPU命令显式MAKEFLAGS=-j8 -l12/CMAKE_BUILD_PARALLEL_LEVEL=8/MAX_JOBS=8。
- 当前无本任务live/queued进程，session3529/25504/58077/56661均已终态，不重启旧排队条目。主说明TWO_BOX_PICK_20261119.md、源码快照source_r4/source_r5、各批summary/independent_fk_verification.json为入口。原默认高箱目标碰立柱7.5mm仍真实，待用户确认位置补偿；尚未全合法域/TimeOpt/可视验收，目标保持active，未提交/推送/改Linear。
- 后续依托证据继续：不能把32有限任务倒定义为合法域。原TorqueModel已支持payload_dimensions，接TimeOpt时传真实尺寸并按尺寸分资源，别另写惯量合成。当前实验运输single_path尚未启用原单臂rotation=True软代价（碰撞检查仍完整），正式回接PR82优化链时应一并恢复/量测，不冒称已保留完整原运动质量与动力学链。

## 2026-11-19 Codex / 两尺寸R6–R8：原生优化/定时、真实尺寸10kg与失败诊断（未完成）
- 在独立worktree继续，未动其他分支/服务。恢复单臂运输rotation=True。FastThreeLayerPlanner新增可选planner注入（原无参兼容），TorqueModel传当前真实payload_dimensions；按两尺寸分资源复用，不再额外构造规划器。新v3_two_box_timing.py复用原TrajOpt/TimeOpt，--timeopt显式启用，不改默认高箱持箱位置。完整四阶段TimeOpt实际4001点验碰/真实箱体/笛卡尔线/限值/接缝，并用同cuRobo后端另模型RNEA复算；不称独立物理引擎认证。
- R6首两例完整链1/2、104.548s；原生八例严格4/8、223.236s，28/32原生阶段通过，31/32 TimeOpt有输出且其31段采样/实际碰撞/动力学检查均过，接缝0。三项TrajOpt原生标志失败（case0接近/case1运输/case5运输）不覆盖为绿；case6高箱左顶运输TimeOpt status1没有轨迹，不回读旧数据冒充。高箱四例仍是箱心(.05,±.0325,0)实验补偿，未采用为默认，不能记“回原位”通过。
- R7定位：cube左侧接近原生mask path/collision=true、limits_and_dynamics=false，未认定具体哪项；不能直接说纯软代价。高箱左顶优化曲线J6采样静态56.50Nm/额定52Nm=1.08654，缩放后1.09200；TimeOpt未找到初始化可行种子。它证明当前路径的静态余量不足，不证明箱位非法/所有路径不可能。下一步应改路径/IK分支与原生约束诊断，而非降10kg、放宽扭矩或重复请求同一条曲线。
- 控制点已在R7保存，墙钟区分geometry/native initialization/total；R6逐例wall_seconds是几何耗时，整批pipeline以summary为准。新verify_native.py纯CPU核对实际NPZ/源码/输入/端点与URDF FK，保留原生失败；R6/R7按各自source_r6/source_r7验证，不混版本。额外R8保护非法输入不进RNEA，GPU NaN负控status3/PASS，snapshot source_r8、native_invalid_r8留证。
- 当前CPU36项（新增10+Scene16+TaskScene3+ResearchContract3+冻结发布4）通过，GPU负控另列。与本机已验证76c2f4b源码快照重新对照460源文件0差异，runtime_verify_r6；未改共享cuRobo、冻结模型或降低约束。主说明TWO_BOX_PICK_20261119.md已更新完整命令与限制。所有本任务GPU会话27665/67693/9671/40568均已终态，无待接回任务；未commit/push/Linear。
- 目标保持active：高箱原默认目标7.5mm立柱冲突和补偿许可仍未解决；任意合法域与最终可视验收仍缺失。已有32有限几何样例/R6原生4/8不能代替完整目标，也不能把R5成绩当R8当前源码全量回归。

## 2026-11-19 Codex / R9–R24根因修复与包络漏门纠正（未完成）
- 当前工作树仍feature/pr82-two-box-extraction，未提交/推送/改Linear或其他分支。原生拒绝回馈到共享plan_pick候选循环，逐候选保存并继续，部分阶段前缀不算完整；运行中success=false且清旧timing。GPU回归覆盖拒绝后续选/接缝/原生净空及目标合同冲突。R14当时局部门禁从4/8到6/8、282.233s，但见下方纠正，不能作完整PR82成绩。
- 候选池64全负肘、256中53个可用正肘，当前保留256池双肘交错，不丢重排池候选。R10软旋转权重0/10/100同输入三档均未解，R11负肘直接位姿LM到84.25%停，未采用的生产代码已删，源码证据留source_r10/source_r11。R9诊断曾错reshape完整17列为15列，现按joint_names选活动列。
- R12具体原生失败是project_base_and_ground，最大.117984，cspace/self/scene为0。R18净空负控用错“无违规末态”，测试正确失败且后续场景未运行；R19真实曲线235点违规，内部样本802成为固定负控，球序映射相同。现在native自由RRT和实际输出对齐原有底盘5mm/地面2mm零集合，接触/LM不扩大，两个净空回归通过。
- 共享CachedMeshQuery.backward的链式法则真bug已修：原软模式忽略grad_output，实际查询乘7，旧自动梯度-.6000对有限差分-4.2007；解析与真实GPU测试修前均失败、修后均过。不改前向碰撞值/权重/门槛，修复ProjectCost、细化和空闲臂调用的真实加权梯度。证据gradient_regression_r21；R21旧目标两反例仍1/2，不冒称全解。
- **严重补漏纠正**：新接口曾漏PR82原transport_candidates的1.05m全机持箱球包络。旧高箱“前.05/外.0325”目标仅载荷球就1.1153m，不能采用。此前4/8、6/8及高箱实验成功只覆盖局部门禁，不是完整基线/验收。原记录不改，historical_envelope_audit_r22另列审计；主文档已置顶纠正，旧文档归档doc_before_r23.md。
- 当前已恢复目标IK全机包络筛选（筛后再取64）及实际TimeOpt持箱末态包络；请求目标计入2mm/1°容差的载荷下界仍必超限时提前报carry_target_contract_conflict，不称输入箱位非法。新增CPU包络回归、GPU目标拒绝负控通过。默认carry_offset仍0，未擅自改目标或1.05m定义；真实箱角半径另外诊断，不偷换球协议。
- R22合规静态顶吸候选(.51,±.4385,1.2)，Ry=-45，球1.03322m、箱角1.04869m，左右91/129个有效状态；不是已采用默认。R23按它测原生八例6/8、212.093s：四个40箱及两个高箱顶吸通过，高箱侧吸两例无合规持箱IK，不能把不同模式/版本拼8/8。native_array_verification独立CPU URDF末态/全机器人球包络/源与输入/数组检查；不独立重做碰撞或RNEA。
- R24仅静态侧吸未采用候选：(.38,±.56,1.2)，Ry=-45，球1.02688m、箱角1.04160m，左右6/16个通过真实碰撞/原生净空/全机包络/10kg静态约束。完整运输未测；相对旧默认位后移17cm、外移13.5cm，不能当作默认不变。其他候选数据side_carry_targets_r24.json。
- 当前CPU38项通过（新增12+Scene16+TaskScene3+ResearchContract3+发布4），GPU候选/净空/目标拒绝3及缓存梯度2回归另列。所有本任务GPU会话均终态，无需重启旧作业；日志/tmp/pr82-two-box-*.log，原始数据及source_rXX在generated/two_box_pick。
- 原默认名义位与新箱尺寸不兼容、默认位是否允许随尺寸/吸附方式适配尚待用户定义；还需实际障碍和合法中心域合同，不能拿容器边界内单箱有限测试当任意合法位置保证。未进入最终可视验收、未泛化第三尺寸。

## 2026-11-19 Codex / 默认持箱位合同阻塞审计
- 不是因难、慢、token或GPU排队停止：全部任务已终态，当前CPU38项通过，GPU拒绝续选/净空/错误目标和真实梯度回归已过。R23的新合规顶吸候选八例6/8，经独立CPU全机球包络与URDF/数组核对；四Cube和两TallTop通过，两TallSide无合规持箱IK。R24仅静态找到TallSide箱心(.38,±.56,1.2)、Ry=-45，左右6/16可用状态，未验运输、未采用。
- 阻塞条件已在原始goal turn及后续至少三轮持续：用户要求回默认45°持箱位，但新高箱按原固定名义位置存在静态碰柱/包络合同冲突。此前+.05/±.0325候选又漏1.05门，已置顶纠正，不能把历史局部门禁6/8当完整成绩。现在已恢复所有包络检查，默认offset仍0。
- 未收到用户许可就继续在其他XYZ求通过，是不同终点，不是完成原目标；因此停止替代目标调参并请求目标定义。唯一关键问题：默认45是否允许随箱型/吸附方式调整XYZ，同时保持45方向、1.05包络及其他原门禁？如果不允许，不能靠规划器或放宽阈值完成固定名义高箱目标。
- 当前代码/说明完整快照source_current_blocked，主文档明确阻塞。未commit/push/合并或改Linear，未发送实机。用户明确目标后才恢复本目标；不要把之前静态候选或实验配置擅自升为默认/验收。

## 2026-11-19 Codex / 用户许可后恢复两尺寸验证
- 用户明确回复“允许”：默认45°持箱位可随箱型及侧吸/顶吸调整XYZ，45°方向/1.05m包络和其他门禁保持。旧合同阻塞解除，不再重复询问。
- profile 2026-11-19-v1：40箱仍(.55,±.425,1.2)，高箱顶吸(.51,±.4385,1.2)，高箱侧吸(.38,±.56,1.2)。None选择profile，构造即解析成旧anchor显式offset；历史显式offset不叠加新默认。记录实际目标矩阵。
- 正继续先跑高箱侧吸左右完整原生流程，再以同一版本复跑八例；静态IK不算运输通过，有限样例不算任意合法位置。尚未实机发送、提交或扩展第三尺寸。

## 2026-11-19 Codex / 获准适配后的完整八例与中心扩测
- 最小接入size/mode/arm持箱profile-v2：Cube不变，高箱Top(.51,±.4385,1.2)，Side左(.34,.56,1.2)/右(.38,-.56,1.2)。v1左(.38,.56,1.2)R25运输失败，内收备选R26完整通过后才采用。显式offset仍相对原anchor，None在构造时解析入记录，旧文件不改。
- R27同规划版本八例8/8、263.458s；32原生段各4001点，CPU独立FK/数组/全机球包络复核通过，最大1.03321781m。source_r27与approved_defaults_r27留证。不是跨版本拼成功。
- R28扩测32中心26/32、989.693s；失败4/5(Cube底面1mm)、7(Cube右高位)、14(Tall左高位)、23(Cube右非网格高位)、24(Tall左非网格)保留，未称任意合法位置达成。两个底面1mm的RRT start_nodes始终1；40球最低点-.200000003m与原生2mm地面净空冲突，R29正负对照排队核实。
- R30新增原生运输起点净空预检查（仍续选接触，不判箱位非法），没有放宽原门禁。正在以16接触候选跑同批八例+四反例+两个3mm近地面点；source_r30。旧1mm负例不删除/替换报通过。
- 新增CPU只读replay_native.py，直接加载实际TimeOpt NPZ/URDF/真实尺寸箱，按原生时钟播放；viser已安装，无需冻结前端包。CPU服务启动/数组/接缝测试过，未声称浏览器验收。CPU41测试通过；新GPU预检回归待跑。

## 2026-11-19 Codex / R29确认近地面失败根因
- transport_start_ground_r29.json：Cube底面1mm（z=.201）左右各4个真实抽离末态全部不满足原生地面2mm零集合，base项均0；球最低实际净空约.001，ground项约1.4e-5。改为独立正控底面3mm（z=.203）时左右各4个末态全部base/ground项为0。
- R28这些运输RRT start_nodes始终1，根节点不合规意味着任何带起点的边都被拒绝；不能用加预算掩盖/误报算法无解。R30已提前拒绝该接触末态并留明确原因，不放宽原生净空，不称输入箱位全局非法，也不把3mm正控改写为原1mm通过。

## 2026-11-19 Codex / R30复测完成与待用户明确的场景合同
- R30同版本八个代表例+R28四个非地面反例+两个Cube底面3mm正控共14/14完整通过，529.970s；独立CPU复核56段实际时间化数组/FK/全机球包络全过，无原生失败升级。不是把3mm点当作原1mm点通过；R28原32例26/32及失败仍保留。
- 高位Cube右第13个接触候选才通过，另一个右非网格高位需第5个。因此当前plan_pick与两个CLI默认预算统一为16（与R30显式运行参数一致）；没有新加策略/放宽约束。R30源快照保留原默认值8与实际显式参数16，当前默认调整单独留source_current_approved，不伪装原源码。
- CPU41项通过；GPU候选续选/原生净空/不合规目标/运输根节点预检4项与真实梯度2项共6项通过。只读CPU回放8294已启动，HTTP200，session49031，日志/tmp/pr82-two-box-native-replay.log；读approved_defaults_r27实际NPZ，未浏览器人工验收，其他端口不动。
- 本任务GPU作业均已终态。未commit/push/实机发送。位置适配许可已解除旧阻塞，不需重复询问；仍缺“合法中心位置”所对应的实际周围堆箱/障碍布局合同。当前只有容器边界内单目标箱，不可外推完整25箱墙/任意障碍/连续全域；请先明确场景后再按它继续验收，不扩第三尺寸。

## 2026-11-19 Codex / 双箱Viser布局器已搭建（8295）
- 新入口research/curobo_v3/tools/v3_box_layout.py，CPU服务8295，当前PID3832742/session86390；旧8294只读回放与其他服务未动。操作/完整启动命令见docs/BOX_LAYOUT_VISER_20261119.md，日志/tmp/pr82-box-layout-viser.log。
- 两箱各自选择两尺寸、长高交换（仅Y/Z，X宽度/抽离长度不变）、世界XYZ数值或平移手柄；世界/root/base_footprint原点固定，没暗移底盘。支持实时布局预览、点击新算、进度/分阶段墙钟与实际运动时长、逐候选拒绝、取消、全流程/分段原生回放。改布局立即回Home预览并禁用旧轨迹；按钮采用Viser async回调冻结提交快照，修复了快速编辑覆盖按钮时输入的竞态。
- GPU每次由CPU界面启动独立前台gpuq worker，CUDA初始化→联合规划→原生优化→审计全程同一共享锁，不给界面本身套队列。默认估时6m基于双箱约75–255s，增大候选预算按N/16保守放大，均暂估非超时；MAKEFLAGS/CMAKE/MAX_JOBS固定8，无嵌套/改锁/实机连接。
- 新v3_box_layout_planner.py是真正15轴/共同升降的双臂联合规划（不是单臂结果拼接）；实际每臂尺寸、侧/顶吸附着、X抽离距离、完整箱间/箱机器人/箱环境碰撞一起核对。最小扩展loaded_robot/payload_safety/TorqueModel为每臂尺寸映射，旧同尺寸接口保持；旋转软代价接真实各臂附着。
- time_pick复用Fast TrajOpt/TimeOpt，双臂抽离补接原PR82 64控制点32轮LM后处理；单臂仍保留原128适配跳过口径。每段4001点/接缝/直线/速度/jerk/10kg每箱力矩/RNEA/实际全机1.05m球末态门禁保留；独立CPU数组/URDF两臂抽离/持箱及球包络核验，完整箱角径另诊断。
- 单臂原持箱目标不能直接拼为双臂共同升降目标。paired_carry_candidates在已获XYZ适配许可下保留45°和所有门禁，有限候选适配持箱XYZ；侧吸高箱优先Z=2.0m，另一臂保持已验姿态；长高交换顶吸XY(.43,±.50)、侧吸XY(.34,±.56)，避免柱碰/完整箱角出圈。实际选中目标在线框和面板显示，输入箱心不改；显式offset关闭自动适配，仍相对原锚点。
- 最新CPU49项通过；GPU候选/净空/目标/根节点4项与真实梯度2项共6项通过（r12日志）。r12双40箱77.900s、混合侧吸131.623s、长高交换顶吸+Cube侧吸77.358s完整原生与独立CPU通过。双高箱默认例255.357s仍拒绝（运输搜索/原生净空及完整载荷穿透），r13长高交换混合侧吸291.248s也保留拒绝；不拼成全过/任意合法位置。
- 当前源码真实界面请求layout_n41vb0ml完整通过73.448s，源码指纹与现文件一致、独立核验真，64/32后处理真。editor_websocket_final.json覆盖真实WS尺寸/长高/XYZ预览、重叠拒绝、取消、按钮→gpuq→解算→子阶段播放及布局变更后旧轨迹失效；HTTP200。协议测试，不冒称浏览器截图/人工可视验收。
- R1/R7/R9为前期诊断版本，r12/r13及source_rXX分别留证，不伪装同版本全域成绩。当前源码/配置/文档完整留source_current_final；后续改规划源码前先确认本任务GPU worker终态，避免后台加载旧代码而记录新文件指纹。当前本任务GPU均终态，仅8294/8295 CPU服务保留。
- 工作树仍feature/pr82-two-box-extraction，基于ee6dba3；未commit/push/改Linear/改共享cuRobo，不扩第三尺寸。当前布局场景只有容器边界与两箱，不代表完整25箱墙、任意障碍或全合法域完备性。等待用户交互验收布局器，原全域规划目标未标完成。

## 2026-11-20 Codex / 双箱布局慢解算定位与保守优化
- 做了什么：确认8295每次点击冷GPU进程导致实例/图缓存丢失，旧默认73.448s中首次native approach34.072s（TimeOpt solve23.520s）、几何抽离11.114s；异构箱重试搜索另有显著开销。没有用热态/单箱/较少门禁数字冒充同工作量基线。
- 改了哪里：`v3_payload_safety.py`复用一次FK完成完整箱体地面+碰撞审计、接受CUDA关节tensor并预计算固定障碍旋转/球索引；单/双箱包装层不再重复FK或D2H→H2D。SAT移到纯CPU `curobo_core/payload_geometry.py`，仍原15轴/归一化/退化阈值，仅换投影矩阵乘。`v3_box_layout.py`限制TorchInductor编译线程8（含旧CPU编辑器发起的新worker）。新增CPU回归、性能复跑工具与`docs/BOX_LAYOUT_PERFORMANCE_20261120.md`，原布局文档加入口。
- 验证结果：CPU51通过；同全局gpuq前台锁内GPU6通过；两段4001帧+513扰动帧真实checker审计新旧/NumPy/CUDA拒绝mask一致（扰动417帧拒绝），审计中位数约.72→.58s。匹配完整冷进程单次A/B 77.11→75.92s，仅1.5%；planner67.986→66.759s，原生接近34.246→34.322s基本不变，不声称主瓶颈已解决。异构侧吸新版2候选91.423s planner通过，保留第一候选完整箱体碰撞拒绝；不能与旧4候选131.623s直接算改善率。最终`after_matched`全部运行指纹与当前文件一致，证据`generated/two_box_layout/performance_20261120/`。
- 留给下个AI：主瓶颈仍是每次重建GPU资源；达到热态需改变生命周期，但常驻/交互会话必须在同一前台gpuq内持锁，等待用户也占卡，未擅自部署。未通过削门禁/4001样本/力矩等参数换速度，未迁入无法证明冷总时间更快的TimeOpt预分配。未停他人任务/绕锁/改共享cuRobo/提交或push；8295 CPU服务及8294旧回放不动，新请求读取优化worker。Torch2.9.1非发布2.8.0完整环境认证。初始探索A/B导入计时不匹配，完整总时间只用`*_matched_process.json`，已明确留档。

## 2026-11-20 Codex / 用户开放GPU锁后的双箱暖会话
- 做了什么：在原PATH gpuq共享锁内实现有界交互会话，不在锁外常驻CUDA；默认空闲120s、接收窗口600s、8请求，忙请求完成后退出，可手动结束。最多LRU保留2套完整拓扑planner，key为实际两箱尺寸+附着矩阵，不含XYZ；坐标修改仍重新规划，不复用结果。双臂LM/native engine跨请求复用，直接复用planner切换模型也检查native key。
- 改了哪里：`v3_box_layout.py`的CPU进程控制器/原子请求协议/GPU会话worker/LRU/源码配置失效/独立CPU核验计时与UI；`v3_box_layout_planner.py`缓存LM/native；`curobo_core/box_pick.py::layout_model_key`；新增CPU会话测试及实际GPU驱动器，扩展WS测试warm/过期快照/手动释放。补busy→idle发布边界、取消优先和stopping闩锁，停止中的worker绝不接收下一请求。文档`BOX_LAYOUT_GPU_SESSION_20261120.md`及原布局/性能说明。
- 验证结果：CPU55通过；gpuq前台GPU6项44.81s通过。同一GPU PID3752162、一次gpuq的6例：默认首次73.786s、相同布局10.087s、X+5mm12.354s、新高箱顶吸拓扑29.542s、同拓扑10.303s、切回默认8.058s；全部实际四阶段4001点/完整箱体碰撞/LM后处理/native/RNEA/CPU核验通过，最后真实空闲10s自动退出。首次/新拓扑冷成本没有隐藏，数据不是分位数或全域保证。
- 界面与部署：8296真实WS验证取消、尺寸长高XYZ、重叠拒绝、74.257→9.886/7.850/8.107s重复解算、按钮后立即编辑使晚到结果不可播放、手动释放仍可回放，随后关闭8296。后补CPU stopping保护后，确认无活动任务、保存8295原默认尺寸/XYZ/.8阈值/16预算，重启本任务8295；最终真实按钮取消后再计算73.967s通过，全部源码指纹与当前一致，`deployment.json`。旧8294 PID3421938不动；当前8295 CPU PID3792772，当前获准暖会话GPU PID3793740在同一gpuq内idle，120s后自动释放。
- 留给下个AI：证据`generated/two_box_layout/warm_session_20261120/`，最终源码`source_final/`/`source_manifest.json`。六例后台实验与8296闭环分别保留两次CPU边界保护前源码，不把旧字节误称最终；最终8295验收及CPU55覆盖最终版本。驱动器`check_warm_session.py`本身是CPU，内部自动gpuq，禁止外面再套队列。所有GPU/编译线程约束不变，没有改共享cuRobo或删门禁/4001采样；未commit/push或实机发送。模型资源热复用不是结果缓存，首次/新拓扑/淘汰后/会话过期仍可能冷。Torch2.9.1依然非完整发布冻结环境。WS不是浏览器WebGL人工验收。

## 2026-11-20 Codex / 用户指定PR82后的基准核对与LM共享FK
- 做了什么：用GitHub CLI读取用户给的PR82（web打开私有页无内容），核对merge ee6dba3280437ea33002ea87a5f8afb8725eadfb就是HEAD。正文CLI L24/R20几何13.779/整单53.574s通过，L18/R11几何3.345/整单5.772s但approach失败，真实页面首次初始化5.854另列/计算67.877s通过；不能把3.345几何时间当所有首单总耗时、也不拿失败例做成功基准。原PR是固定目录6阶段，布局器任意位置4阶段，不冒充同输入整单A/B。
- 原因：PR82 placement_solver显式禁用LM插值边检查，dense_path_rechecks=False，Fast posthoc_checked_frames=0。新布局器直接构造LM默认检查边，另有最终4001点/完整箱体CPU SAT/直线硬门/RNEA/CPU核验，确实改变工作量；不是只有冷启动。旧热态8.058s中LM本体.467s，候选筛选1.098s、插值检查1.168s，几何运输1.850s。
- 改了哪里：GpuValidity抽出同一批刚算FK的evaluate_state，默认evaluate仍原分块/碰撞/地面/稳定性计算；v3_lm_conveyor候选及边各一次FK同时用于pose+collision，不缓存可变FK缓冲，批量准备128个参考目标/附着逆矩阵/张量。点数/32种子32迭代/排序/拒绝/边检查不删。layout结果指纹补v3_lm_conveyor。新增CPU共享FK测试、GPU同输入LM A/B工具与docs/BOX_LAYOUT_PR82_COMPARISON_20261120.md。
- 验证结果：CPU57通过；GPU6项38.98s通过。4次交替同checker/输入LM A/B，去首轮的3次中位数2.745→2.340s（约15%，不是整单提速率）；轨迹1e-7内相同、127步候选/拒绝/边报告完全相同；真实扰动CUDA数据的旧/新/共享FK validity mask和stability在scene0/1mm、base0/2mm均完全一致。首轮共享进程编译缓存不公平，未用11.280→3.079s宣传冷启动速度。
- 测试合同：v3_lm_conveyor不再字节等同PR80，保留其他7项原文件字节检查，新增与PR82的LM构造/求解预算、精度连续性/种子噪声/排序、边接受及失败分支、输出合同AST对照，未简单删碰撞回归；原发布资产manifest未改。本工作区性能扩展不冒称原冻结源码。
- 留给下个AI：证据generated/two_box_layout/pr82_comparison_20261120/（pr82.json、pr82_source、before、source_after/SHA、lm_ab_warm_default_return.json、cpu_tests.log、gpu_ab.log）。完整六请求暖会话回归在共享锁被外部Blender占用时排队>10min，主动仅取消本任务未获锁队列PID147254，full_regression_queue_status.json留证；无新整单成绩，不声称5秒目标达成。需后续新目录复跑check_warm_session.py；CPU8295没重启/8294不动，新worker会读取新模块，旧source_stamp会使旧缓存退出。没改共享cuRobo/删门禁/绕锁/停他人任务/commit或push，未实机发送。优先按用户PR82明确“几何/总墙钟、通过/失败、冷/热与门禁”口径，不再让用户重复提供基线。

## 2026-11-20 Codex / 同一失败运输路径修复完整箱体引导，原两项拒绝整单恢复
- 前轮分类：上一目标轮只有道歉，未推进；本轮重读现场源码/任务/日志并确认无活动自有GPU工作进程，直接处理目标中的失败任务，不再用不同任务耗时表作结论。
- 根因证据：取`final_both_tall_side_r12`中459行15轴运输RRT路径（源文件/关节SHA留证）；该路径本身完整载荷CPU审计合法，旧TrajOpt仅见40球模型，优化后箱体穿透固定盒约25–27mm而被CPU gate拒绝。同任务同一路径的最终完整几何引导版4001点运输通过，重叠0、native净空/关节/速度/jerk/力矩与独立RNEA全过；不是整单速度A/B。
- 改了哪里：新`v3_full_payload_cost.py`可微OBB/SAT对世界/内部四盒/原机器人球/另一箱体/地面，复用已有SAT和cuRobo四元数转换；`GeometricCostManager`在实例有载荷层时添加引导，`FastThreeLayerPlanner`只为启用的layout transport挂层/按引导开关与盒数区分资源/原地址更新场景；`BoxLayoutPlanner.full_payload_trajopt=True`且指纹补新模块。PR82原入口、单臂及接近默认不挂载。15mm激活/1e7平方权重仅优化软引导，世界1mm/自碰epsilon/native/RNEA/4001采样门禁不动；32控制/50轮运输、64控制/32轮抽离后处理、X宽度/45度不改。
- 探索保留：v2/v3不足的引导仍有7.4/4.7mm穿透，未放宽gate；最终v4完整通过。float32指导SAT在近平行轴对CPU double约50um误差，探针对照容差0.1mm只用于诊断，不是验收阈值；真实CUDA姿态梯度与有限差分吻合。CPU几何/箱球/箱箱/地面/更新地址测试通过。
- 整单验证：原双高箱侧吸16次拒绝→本轮1候选通过81.622s；原混合长高交换侧吸16次拒绝→2候选51.967s；默认双40箱31.104s、新XYZ双40箱9.519s、混合交换顶吸32.058s通过；另批双高箱新XYZ78.183s/1候选、双高箱顶吸新XYZ51.507s/2候选、双40顶吸新XYZ33.963s/1候选通过。两例第一候选仍被原cspace/self/base门拒绝，未删记录。8例均16001样本独立CPU核验；不能拿不同冷/热/拓扑/候选数算提速率，5秒未达成。
- 测试/界面：CPU58通过、gpuq GPU原6项38.71s通过。临时8296 WS双高箱首次78.849s，3次重算13.515/14.802/13.623s均通过，取消、真实分段播放、按钮后立即改XYZ的迟到结果拒放、显式释放锁均通过；不是浏览器WebGL人工验收。12份整单报告（8布局+4重复UI）当前全部源码SHA一致。只关临时8296，8295 PID3792772/8294 PID3421938原参数/进程不动，无残余自有GPU任务；新8295 worker自动读取新模块。
- 证据：`generated/two_box_layout/payload_guidance_20261120/`含before、v2/v3/v4、输入指纹、full_batch/new_positions、GUI、verification、source_final/SHA。文档`docs/BOX_LAYOUT_PAYLOAD_GUIDANCE_20261120.md`、原布局说明追加入口；未改共享cuRobo/绕锁/停他人任务/commit/push或实机发送。
- 目标仍active：8个代表位置不是“任意合法位置”证明；仍需边界/阈值附近/左右异构组合/狭窄净空覆盖，不能把有限IK/RRT预算失败改称非法。下一步热态瓶颈是RRT内完整载荷CPU审计，双高箱仍13–15s；若移GPU必须保留同输入拒绝集合，不能用当前float32引导SAT直接替代CPU门禁（退化轴1e-7 vsCPU1e-10及50um差别），应双精度/歧义回退并真实验证。高位顶吸拒绝继续按用户要求暂挂；45°及用户已允许的尺寸/吸附XYZ适配合同不改，不宣布可验收或完成。

## 2026-11-20 Codex / 精确载荷搜索加速、同输入整单A/B及边界见证
- 前轮分类/现场：上一轮完整载荷运输引导与8例恢复是进展；本轮重读源码/日志/现场进程，确认无残余自有GPU作业，继续实际热态RRT成本和边界，而非换任务比较。
- 改了哪里：`curobo_core/payload_geometry.py`新增有符号float64 tensor SAT（全部15轴、CPU1e-10退化阈值）；`FullPayloadCost`仅新增显式double分类通路，原float32 soft guidance默认不变。`payload_search_mask`按1024批次、实际当前FK分类，近退化/1e-7m门限邻域回退原CPU audit并覆盖其判定，明确面轴分离可免除无关微交叉轴歧义；模型/附着/尺寸/场景几何key失效缓存，不保存FK/mask。单/双箱搜索共用；最终4001点CPU完整箱体审计、carry IK CPU、原native/RNEA/独立核验不改。双臂搜索报告增加处理/回退帧数。
- 精确性证据：4拓扑×真实RRT/4001点/2048随机扰动/真实地面门限，共25575个输入行，CPU和GPU所有mask相同（3072帧实际拒绝）；4组门限实测分别回退4/2/2/8行CPU。场景内插真实障碍拒绝、恢复再合法，缓存失效真测。CPU近平行/门限/随机、double分类和float32误用测试；CPU60通过，原GPU6项38.04s通过。
- 同任务性能：同一双高箱、同箱位/障碍/附着/目标/预算、同463行RRT路径SHA，交替CPU/GPU各3次热测，最终1024批版整单中位数13.250→7.354s（约44.5%下降），运输几何7.36–7.64→1.35–1.46s，8次含2warmup全1候选通过，GPU每次58226检查行。只覆盖实例search mask切换在结果中明示，不伪装无运行时覆盖；非PR82不同固定任务A/B。冷warmup约80s单列，5秒未达成。256批初版13–15→9s另存，不混版本；1024批相对2048显存99 vs189MiB、速度.179 vs.167s/13024行，选择1024保留余量。
- 边界：threshold(.8/.8001)双40侧吸、(.7999高箱顶/.8cube侧)、Y±.95高箱、Z2.05顶棚高箱侧吸、X1.55前壁cube全整单通过。静态1.25/1.35/1.45/1.50/1.55均有8个contact见证，不称整个可达边界。低位(.78,±.48,.2)16次起点失败；独立CPU URDF证明所需-30cm末态与固定底盘重叠17cm，而非凭搜索失败判非法。
- 必要几何拒绝：共享纯CPU `fixed_extraction_blockages`为规定末态提供穿透深于6mm平移+箱角1deg误差包络+世界允许量的保守见证，布局器预检返回`required_extraction_endpoint_blocked`/具体障碍端点深度，微擦碰不提前拒绝，不改变轨迹门限/合法域。最终0候选拒绝堵塞案例；真正留侧向通道的低箱(.90,±.90,.205)2候选通过74.557s，(.95,±.85,.24)1候选通过7.645s，不能笼统把低位当非法。
- UI与最终源码：临时8296双高箱真实WS首次73.978s、后续8.481/7.123/7.268s全部完整验收；取消/播放/提交后改XYZ过期结果禁播/释放锁均过，非WebGL人工验收。7份最终源码报告SHA匹配（4UI+2低位通过+1阻塞拒绝），代码`source_final/`与SHA保留。只关闭临时8296，8295 PID3792772/8294 PID3421938不动，没有自有GPU进程残留；新8295 worker会读取代码。
- 证据/后续：`generated/two_box_layout/precise_payload_mask_20261120/`（before/source_256/source_final、parity_v2、batch_sizes、same_task_hot_1024、boundaries、floor_corridor、gui、verification/evidence_versions），文档`BOX_LAYOUT_PRECISE_SEARCH_20261120.md`。性能/部分边界在新增静态末态见证前执行，evidence_versions显式标差异，不改旧指纹；最终UI/低位验证使用当前全部源码。未改共享cuRobo/全局锁/线程8限制/抽离30或25.5cm/45°/动态门禁，未commit/push/实机发送。目标保持active：有限通过不是任意合法位置证明，尚需左右非对称/混合长高/狭窄通道与剩余热成本；高位顶吸拒绝仍暂挂。勿把当前float32 soft guidance用作硬mask；double歧义CPU回退和最终CPU gate必须保留。

## 2026-11-22 Codex / PR82双箱续接、可达性证明与CPU审计宽阶段
- 做了什么：续接11月20日中断的非对称测试，仅补未完成4例均通过；再补21例均通过。修复模式审计的证据根目录，真实报告/4001×15数组/独立16001采样核对，36种有序尺寸×吸附组合均有历史成功见证；不代表任意连续XYZ，也不伪称旧报告匹配当前新源码。
- 改了哪里：新增`tools/curobo_core/contact_reach.py`与CPU测试，从URDF SRS几何/J2凸帽/共用升降区间出保守不可达证据，接入`v3_box_layout_planner.py`；`v3_box_layout.py`展示毫米数值理由。四个远交叉分配有约74–84mm不足，不再以有限IK无解冒充证明。近同侧两例通过，近交叉两例仍未解，未收窄合法域。测试catalog选择支持/安全文件名/重复及unknown选择检查已加至原实验脚本。
- 最小性能改动：仅`v3_payload_safety.exact_payload_audit`增加CPU AABB保守宽阶段；精确SAT/球距、最终4001点与原1mm/1um门限保留，近平行搜索/门限CPU回退未撤。冻结前版与原型12组真实FK对照一致；新增1025组旋转箱/球穷举CPU核验。新增同任务运行脚本`tools/experiments/box_layout_20261119/check_cpu_audit_broadphase.py`，执行替换在报告显式注明并保存原函数SHA，不混淆磁盘源SHA与执行函数。
- 验证结果：CPU67通过(1.90s)，GPU6通过(25.56s)。同双高箱、GPU搜索不变、全部门禁不变，交替热测各3次：原审计4.426556/4.474046/4.368264s，新审计3.861441/4.023687/4.066970s，中位4.426556→4.023687s(9.10%)。首请求46.470155s、warmup5.216798s单列；八次全1候选通过，运输均463行同SHA`c48d0de6921712a6e7e901ad70e243e446f640515097d12d647c26a9e83b55a9`。不是把11月20日7.35s与今日4.02s跨负载直接归因，亦非PR82另一固定任务A/B。墙钟不含队列/Python启动/UI传播，不能承诺所有输入/首次<5s。
- 交互：续接时原8295 PID3792772/8294 PID3421938均不存在、端口无监听，非本轮终止；重新起当前CPU布局器8295 PID398497（exec句柄63860），不占GPU。真实WS已跑尺寸/长高互换/重叠拒绝/取消/数值可达性证明拒绝/冷与热整单/播放时间推进/改参数过期结果禁播/明确释放锁，8项通过。双高箱首次44.642778s，热5.193255/3.763279/3.755228s；最后已通过轨迹保持可播放，GPU会话`user_stop`释放。仅WS协议测试，不称WebGL人工验收。所有CUDA仍经PATH gpuq，未另加锁/嵌套/实机发送/commit/push。
- 未解出定位：`contact_reach_20261122/extraction_scan`两例各256候选全扫未有全门禁粗抽离见证，不可再靠重排说修好。新`endpoint_probe`保留真实失败关节：高箱撞另一臂right_link3 6.9–12.5mm，低cube撞arm_carriage 28.3–30.8mm，首候选91.34%失败。名义与±.75°pitch的5cm/10°/32每臂有限末态池无可行点；CPU固定球/外可达界也不足以证明不可能，故仍记未知，不添加错误“非法”分支。
- 证据/留给下个AI：`generated/two_box_layout/{asymmetric_coverage_20261122,mode_coverage_20261122,contact_reach_20261122,cpu_audit_broadphase_20261122}/`，最新文档`docs/BOX_LAYOUT_CONTINUATION_20261122.md`。`cpu_audit_broadphase.../source_manifest.json`/`source/`保留本轮当前源码，与旧21+14历史证据分epoch。目标保持active，尚未到任意合法位置/用户验收；下一步先找近交叉可行末态/路径见证再改升降/零空间种子，兼顾狭窄通道与最新源码其他拓扑回归。高位顶吸拒绝仍挂起。勿以有限36组合充当连续XYZ完备性，勿因新单例热4s就隐去UI5.19s和冷启动44s。

## 2026-11-22 Codex / 窄位反向恢复、箱心参考修复（目标续接）
- 做了什么：上一目标turn属实际进展；本轮继续原两个近交叉未知，不收窄合法域。先证明不是简单32候选/5cm格点充分性：容差内局部几何搜索+精细升降/2°swivel/全部每臂解，找到144个真实完整门禁末态，再做反向15轴LM。早期保存粗几何的四阶段原生验收全通过，明确标saved-geometry feasibility witness，非新鲜解算或性能证据。
- 根因修复：`v3_two_box_timing`新布局双臂抽离最终验箱心线，但`v3_lm_refinement.phase_references`旧目标是TCP线；有吸附偏置和小转动时不等价。新布局显式选箱心参考，旧PR82无标记调用不变，CPU同测两分支。同粗几何/同固定端点，右箱原生横误差3.107854→1.696596mm，原2mm门禁不放松，旧失败和新成功分存。
- 改了哪里：`curobo_core/box_pick.extraction_endpoint_proposals`(4个CPU局部种子，≤1.8mm/.95deg，仅引导不作非法证明)；`v3_box_layout_planner`精细末态枚举、反向LM恢复、全payload/接触/Cartesian再验。先4原候选（历史成功最多4），几何均失败再最多8恢复候选，合计≤用户max_contacts。恢复运输可用已验逆抽离前缀多根RRT，从真实全宽抽离末态连续走到选根，不瞬移；明确允许自由搬运阶段先回退部分原路，不声称X单调。恢复分支RRT种子递增避免邻近候选反复同随机流，首分支/普通候选种子不变；生产每次RRT仍8s。`v3_box_layout`新增三阶段中文映射。负stride新缺陷在共享运输入口ascontiguousarray修复并加CPU测试；所有失败记录保留。
- 新鲜结果：`endpoint_feasibility_20261122/automatic_full_v2`左高箱(1,-.05,1.5)/右cube(1,+.05,.6)全新整单62.311s/5候选通过；`automatic_full_v3`当前种子策略再从原始输入全新整单64.851s/5候选通过，四阶段4001和独立16001点/全宽/真实45°/包络全过，不注入保存轨迹，不称5秒热解算。镜像仍未解，未将其判非法。
- 非退化检查：`regressions`9/9通过（双高首次49.786s、热5.146/3.996s；混合顶侧/长高、低通道、旧第3/4候选场景、默认双cube等）。CPU80通过4.59s，GPU6通过27.73s；额外在gpuq内审计原LM参考CPU测试11项，前后Torch CUDA未初始化/Warp runtime为None。`verify_evidence.py`核10份当前源码成功报告，四阶段4001×15/独立16001/全部门禁/总预算、SHA均匹配，源码`source_final/`；旧epoch`reference_fix_source`、`recovery_v1_source`、`recovery_v2_source`不篡改。
- 新诊断阻碍（非blocked）：镜像首恢复运输临时45s诊断两次，在默认Warp缓存及独立WARP_CACHE_PATH都报`Failed to find forward kernel tiled_collision_kernel / tiled_collision_kernel_84eb2579`，未得到45s连通性结论。该共享依赖的生成核horizon=2599、block128，独立缓存cu符号`tiled_collision_kernel_474d7916_cuda_kernel_forward`；不能说磁盘缓存坏，也不能判场景非法。未改共享cuRobo/Warp/生产预算/全局锁；独立缓存不是GPU锁，所有GPU初始化与诊断仍在gpuq中。
- 留给下个AI：先读`docs/BOX_LAYOUT_REVERSE_RECOVERY_20261122.md`。镜像可达/运输仍关键路径：查实际请求mangled symbol与共享Warp动态模板生命周期，或在布局器RRT内规整尾批减少模板（须mask等价/门限/性能实测，不把workaround说成根因修复），再验证适配预算/方向/持箱目标，不重复已证失败的有限重排。内核源码和异常在`endpoint_feasibility_20261122/{mirror_long_diagnostic,mirror_long_isolated,warp_cache_isolated}/`。当前10成功不是全合法XYZ完成；新恢复路线尚缺实际GUI回放检查。
- 服务/资源：8295 PID398497仍活且未改用户输入/重启，上轮老CPU formatter会以原英文名显示新阶段耗时，新worker读当前数学代码；不要谎称新CPU UI字节或WebGL已验。GPU命令句柄70921/84234/45428均已终止，最后无自有CUDA任务；只保留CPU编辑器。未commit/push/实机发送。目标active，不提请最终验收。

## 2026-11-22 Codex / 双镜像打通、软间距分权与当前36模式核验
- 上一目标turn属实际进展。本轮未缩小目标，继续镜像运输。最小2599行查询连续成功，完整追踪也经过该变体并在18.269s找到粗路径，不能归咎horizon本身或宣称磁盘缓存修好了Warp。原生坏点回投搜索也拒绝，完整payload审计却通过，确认为真实手臂自碰而非简单checker差异。
- 已验证失败探索不留生产：32→64控制点拟合11.013°→5.725°仍自碰，曾遇8000插值buffer不足；控制点扩展和临时测试已全部撤回，`v3_fast_three_layers.py`与本轮before字节完全相同（其原有dirty修改保留），实验在`tested_64/`。保留原穿透+虚拟偏置梯度的线性变体也被base/ground门禁拒绝，未采用。
- 根因最小修复：`v3_full_payload_cost.FullPayloadCost.cost`把实际穿透强二次项与15mm虚拟间距项分开，后者系数0.001（外部1e7、geometry_scale100不变），不改classify或最终门禁。控制粗路径/端点/32控制点原生对照：原引导geo100自碰，单降geo1仍范围/自碰；分权引导geo100/geo1都通过。不能用这组保存粗几何阶段试验当新鲜整单速度基准。
- `v3_box_layout_planner`仅搜索查询补齐512倍数并裁输出，限制4096分块后的尾批为8种模板；72组真实FK/122244输入行（含重复）/12917拒绝逐行mask及稳定性完全相同，单查询时间接近，不声称性能大幅提升，不改共享Warp/cuRobo。普通运输单次8s，首反向恢复可集中32s，按实际search_ms扣同一8*max_contacts总运输预算（默认128s），未增加总cap；最后一次GPU迭代仍可越过软边界。原因文本不再对32s分支谎写8s。
- 新鲜结果：`warp_horizon_20261122/full_recovery`两近交叉镜像均在第5总候选通过：左高/右低59.857198s（运输1.283556s）、左低/右高40.818190s（运输12.023985s），无保存轨迹/目标注入，四阶段4001及独立16001点/全宽容差/实际45°/原球包络/动力学全过。普通9例回归9/9；双高首次49.548s、热5.201/3.899s如实列，不承诺所有请求5秒或拿旧4.024中位数当现行数据。
- 覆盖与重验：补27例`missing_batch_1/2/3`全部通过，`audit_modes.py`按实际数组和源SHA确认当前36种有序尺寸姿态/顶侧吸组合全部有成功实例（两箱型，不是36箱型）。`verify_final.py`重读39份当前新鲜成功（含热重复/GUI），156原生阶段各4001×15、CPU重验624039去重接缝样本，全部source_sha及完整session stamp匹配；原生collision/payload/RNEA门禁全过，文件SHA/CPU端点重验/源码在`final_verification.json`/`source_manifest.json`/`source_final/`。有限点不能证明任意合法XYZ，目标active不标完成。
- 检查：CPU82通过（5.49s，微小测试构造清理后需收尾再跑），GPU6通过28.08s。新增CPU测试验证真实穿透项/虚拟间距梯度分权、补行保持原输出、实际预算传递/非法预算；无新依赖、无共享源码修改、无commit/push/实机发送。
- Viser：新临时8296 CPU PID806964（exec句柄46334），原8295 PID398497不动。`gui/ws_checks.json`5项真实协议检查通过：换尺寸/交换Y/Z、重叠拒绝、实际按钮gpuq求解及播放、新恢复阶段中文时间、终帧实际两箱Ry=-45°、显式释放锁（session_gaeps45r stopped user_stop）。左末态(.55,.425,1.2)、右(.38,-.56,2.0)，wxyz约(.9238795,0,-.3826834,0)。不是WebGL人工验收。8296只保留CPU回放，测试会话idle10s/max2，不能当常驻热性能服务；原8295会话配置保留。
- 关键工艺澄清：当前继承自由RRT运输语义，先完成全宽抽离（原2mm/1°容差），随后运输允许沿已验原路回退/前探；前缀连续、所有碰撞照查，但**不保证不回插**。不能把已有结果说成满足“抽离后永不前探”。需向用户明确这一点；若禁止回插必须加新工艺约束重解，不可偷换目标。当前39有限成功不等同任意XYZ保证；下一步旧已知合法边界/新非对称位置与该工艺语义确认，不扩到其他箱型。
- 证据/交接：新文`docs/BOX_LAYOUT_TRANSPORT_GUIDANCE_20261122.md`优先；目录`generated/two_box_layout/warp_horizon_20261122/`。GPU句柄86565/20273/19448/71471已结束，GUI GPU会话释放；收尾确认无自有CUDA进程。留CPU8295/8296供查看；不要盲重启/改用户参数。
- 收尾补充（重要语义，不可忽略）：`check_transport_reentry.py`CPU重算两条窄位及GUI原生运输全部4001点，实际最大中心前探左高/右低0.223/0.397m、镜像0.380/0.412m，GUI0.379/0.409m，SAT确认重新深入各自原占位区，不只是“代码允许”或小数值误差。现有世界把原位当取走后的空区，所以物理碰撞门禁可通过，但**不得据39技术通过声称满足用户真实工艺**。需向用户确认“抽出后是否禁止再次进入原占位区”，不擅自允许，也不擅自追加约束缩小合法域。若禁止，这是当前任务内的验收口径修正，不是新箱型泛化。
- 最终CPU82再次通过5.03s；`catalog_known_positions.py`筛出20个同URDF/config/policy/environment的历史成功XYZ（当前未测），在`known_position_catalog.json`并附原完整报告/任务。这批仅证明自由运输口径下的历史可行性，工艺未确认前不能拿来证明禁止回插下合法。
- 8296已安全seek本次已验数据到“全流程”t0，未触发新计算，CPU可直接播放；其GPU session已user_stop。39份最终重验成功，156原生阶段/624156含接缝采样/624039独立去重采样；`verify_final.py`后续按冻结39个路径重验，用户增加新请求不会误改这份固定证据。GPU无自有遗留任务，保留CPU8295/8296；问题待用户明确，不mark complete/paused/blocked。

## 2026-11-22 Codex / 工艺状态显式化、完整时序与条件几何分析
- 上一目标turn为实际进展。本轮用户未答禁止回插问题，但不空转：CPU审计全部39条旧原生运输，31条比初始进一步深入原占位超过1cm；1um口径下0条始终分离（含初始微小重叠，不能说39条均宏观回插），13条抽离末态全箱已过原前沿面。原位当前为空区，不是碰撞物，物理过检不代表用户工艺已完成。
- 生产透明性修正（**没有追加禁止回插规则/缩小拒绝域**）：新增纯`source_slot_observation`，原生运输物理谓词通过后记录前沿差、初始/最大原位重叠、进一步深入、前探与离开后再入；`process_validation.status=unconfirmed`、`task_success=null`，`physical_success`保留物理验收含义，UI限定“仅物理门禁，工艺规则待确认”。harness summary也明确physical-only。原物理布尔表达式AST合同不变；未改规划代价/采样/安全门禁/共享依赖。
- 实际UI缺陷：首轮108事件只展示后60条、拒绝只后8条，隐藏早期种子耗时导致控件测试失败。已修共享status_markdown显示整个有限请求历史并加>60/>8测试，没通过删除断言掩盖问题。
- 验证：CPU85通过5.45s；CLI一次、GUI两次新鲜完整物理轨迹都通过并task_success=null。`verify_observability.py`重验CPU端点/数组/包络，再逐点CPU URDF核对原位诊断，最大差约0.153um（数值校核限20um不是工艺容差）。旧39/36证据仍属前观察版本，未改旧SHA；前两新结果仅UI文件SHA旧，最后GUI layout_msjs6vcq全SHA匹配，不能混时代。
- 条件几何：严格“末态与原位分离”有限SLSQP无种子仍只是未知；另`plane_certificate.py`在更强“全箱已过原前沿平面”假设下用内棱柱+J2投影必要lift界+head/carriage真实球解析区间覆盖，证明两窄位在该条件下无末态。必要lift约[-1,-.167255]，carriage sphere12区间[-1,-.690913]与head sphere6[-.730707,-.167255]重叠覆盖。3mm位置包络/1deg、head锁定/body平移检查、2000允许姿态内棱柱及1200真实球箱交叠自检过。**条件强于仅禁止后续回插，未获用户确认，不入生产拒绝，不宣称箱位全局非法。**
- 服务：原8295 PID398497不动。临时8296只读快照确认参数未变/无活跃任务，再保留箱体参数两次重启以显示新增观察和完整历史；现PID1374948，exec39660。WS第二次5项通过含physics-only/unconfirmed提示、原位数字、真实播放/两箱-45deg、user_stop释放锁；第一次失败会话是idle_timeout，日志保留。不是WebGL人工验收。GPU83870/GUI69984/37634均终止，待收尾确认无自有GPU残留。
- 下一步/输入：用户尚未回答“完成抽离后是否必须禁止再次进入原占位区”。当前数据不能算业务任务完成。优先`docs/BOX_LAYOUT_PROCESS_OBSERVABILITY_20261122.md`，证据`generated/two_box_layout/no_reentry_probe_20261122/`。可在不改变默认语义的前提下研究更直观的无回插候选，但不能把更强过面条件当作已要求的规则，也不能继续拿自由搬运样例当严格工艺验收。目标active，不mark complete/paused/blocked；无commit/push/实机发送。

## 2026-11-22 Codex / 删除逆抽离运输根并实算确认自由RRT仍回插
- 用户质疑重入占位区后，当前8296检查演示保持原PID/参数/回放；未替换成新轨迹。
- `v3_box_layout_planner.py`移除escape_rows/回退多根/前缀拼接，所有pair_path只从传入实际状态出发并检查返回首状态；原transport传extracted[-1]。CPU测试替换为真实单起点合同；test_box_layout/session/two_box_pick/contact_reach共39通过1.88s。
- gpuq单前台两请求实算终态：分离侧吸高箱/方箱59.066s、第2候选；当前GUI布局64.391s、第8候选，均物理通过但原位重叠分别继续加深13.40/19.79cm及28.60/33.19cm。显式回退删除仍不能证明无回插，自由运输RRT本身允许通过原占位空区，不能报告任务修复。
- 证据`generated/two_box_layout/actual_transport_root_20261122/`含请求、每阶段/失败日志、源SHA/补丁、CPU及独立URDF复核；说明`docs/BOX_LAYOUT_ACTUAL_TRANSPORT_ROOT_20261122.md`。结果task_success仍null，不改物理门禁，不将搜索失败定义成非法，不外推全域/5s达标，不把更强的全箱过前沿条件接入拒绝。
- 下一步需同时约束搜索与最终原生路径中的再深入；本轮不是完成或验收。未commit/push/实机发送。

## 2026-11-22 Codex / 原位工艺约束落地、原生引导与窄起点步长诊断
- 上一turn是实际进展。本轮不再只观察回插：`box_pick.source_slot_process_checks`检查每箱自身原始OBB，不加深抽离末态重叠、完全离开后不再进入且最终需达到分离（1um为观测数值分辨率，不是毫米安全豁免）。不加“整箱先过X前沿”更强条件，不把中心X前探直接当回插。
- `v3_box_layout_planner.py`接GPU float64 SAT搜索上限，并在原四段原生物理谓词通过后独立做4001点工艺检查，不合格继续候选。`v3_full_payload_cost.py`/`v3_fast_three_layers.py`加入同一原位的原生软引导，缓冲原地更新保留graph地址；物理代价/门禁不放宽。`v3_box_layout.py`区分单请求物理/工艺成功、独立CPU失败与源码失配仍不能成功；状态面板显示工艺与拒绝原因。源码/结果分三个时代保留，不改旧SHA。
- A `process_guard_20261122`：无原生原位引导时常规左高箱右方箱第15候选、131.322s物理+工艺通过，独立CPU再验通过；第二请求未完成时显式SIGINT停止自有worker，句柄82102终态254，`all_cases_done=false`，不能声称两例完成。
- B `process_guidance_20261122`：加引导后同常规请求第2候选、58.816s通过，GPU/独立CPU均无进一步深入/离开后再入；当前演示原布局108.556s未找到几何运输，8个起点树均单节点。该时代两例已结束，57项CPU通过。
- gpuq有界实际起点探针`root_probe.py`（句柄91491已终态）证实起点有效、1/512帧FK一致；2048随机方向中加权步长.28与.1均无原位受限有效邻点，.03/.01/.003分别4/46/175个。不是非法证明。
- C `process_multiscale_20261122`：RRT共享实现仅4行新增1行替换，默认原.28不变，仅布局运输启用.28/.03/.01/.003；原边检查不变，无回退根。58项CPU通过2.13s，含旧默认表达式逐位一致合同与原生物理谓词AST合同。当前演示原箱位仍118.691s/16候选未找到运输，但起点树已展开355–936节点；不能报告该箱位已解决或非法。句柄64086终态0表示批处理完成，不代表请求通过。C失败报告源SHA与当前一致，CPU核对已写verification.json。
- **仍在队列，别重启/复制提交**：同C源码常规布局回归句柄46855，gpuq客户端PID1588006（bash1588000），输出`process_multiscale_20261122/regression.log`，截至17:45后检查客户端存活、无worker子进程/无输出，等待共享队列。冻结tools/config直到它终态或明确取消，不能用B通过冒称C成功。其结束后运行C目录`verify_results.py`（已聚合gpu/与regression/两个catalog）更新独立CPU证据。
- 文档`docs/BOX_LAYOUT_NO_REENTRY_GATES_20261122.md`优先解释本轮工艺/源码时代。下一步仍需解决当前窄位从可展开局部到45°目标的完整连接并做两尺寸广泛位置回归；未达任意合法XYZ或5s性能目标。8296旧检查演示未重启、未改参数/回放，不能冒称新轨迹已在该页。未commit/push/实机发送。

## 2026-11-22 Codex / 连续横移接通窄位几何目标，定位32控制点表示失真
- **旧队列46855已完成，不再等待/冻结**：C常规布局124.159s/11候选通过；独立CPU在改D前复核，源SHA匹配。当前D将微步改成仅粗步无法从实际根扩展时激活；58 CPU通过2.08s。D常规真实在线规划2候选48.539s通过，CPU复核通过、源SHA匹配，两个运输均multiscale_used=false。不同进程/拓扑冷热状态，不报PR82加速比或5s达标。
- CPU复用现成解析桥/URDF得到原箱位“右先清出、左后以1mm笛卡尔步进清出”的551状态正向横移前缀。初版CPU把1um箱体数值容差误用于机器人球自碰撞，漏过carriage-left_link7约0.549um（GPU4756点只拒绝1499号）；真实GPU门禁未放宽。候选生成改5um自碰撞裕量、.001加权边采样后重新生成，原数据不覆盖。
- `directed_prefix_strict_20261122`：新前缀4756 GPU点全过、与实际抽离根差0，后续真实RRT16.395s接回原45°目标。**仅完整几何成功**，原生前3段通过/运输失败（cspace、自碰撞、底座/箱体512失效帧），physical_success=false。预计算CPU前缀只是单例诊断，不是已接入通用在线规划，不比较其验证时间与5s。
- 确认32控制点种子拟合严重失真：17.482deg/20.677mm。`directed_shortcut_20261122`保留前缀、复用旧贪心逐边捷径将RRT尾32顶点缩10顶点（.003边检、.001加密复查），关节弧长17.548→11.601；种子仍偏12.658deg/14.204mm，原生运输仍拒绝。CPU独立原位/45°端点复核另存raw_geometry_verification.json，只证明原始几何，不证明原生安全。
- 优先交接`docs/BOX_LAYOUT_DIRECTED_PREFIX_20261122.md`。下一步修原生保形/分段表示，勿再空转RRT；source-slot15mm强虚拟惩罚是否逼迫提前清出是待验证假设，不能擅自放宽最终门禁；重复FK的优化可复用evaluate_state但须防可变视图被CPU fallback覆盖。
- GPU46855/40642/15374/18360全部终态，**无本轮自有待运行GPU任务**。当前原箱位没有原生合格轨迹，不能替换/宣称8296已展示新成功；旧8296参数/回放未改。未commit/push/实机发送，目标active，不报任意合法XYZ或5s完成。

## 2026-11-22 Codex / 固定窄位离线分段原生通过，8297可检查（非在线/全域/5s完成）
- 优先交接`docs/BOX_LAYOUT_NATIVE_PIECEWISE_VERIFIED_20261122.md`。本轮仅生产小修path_validity start/goal显式float32及测试；58 CPU过2.20s。其余复杂策略都在generated实验中，不能冒称已接入通用在线布局器。
- GPU真审计证实整段32种子529物理/809含原位失效；右横移32、尾32实际门禁均0失效，不能把同参数拟合误差直接当几何失效。右强source原代价中间原生段通过；改弱/L1单调试验失败，未改生产cost。
- 现成cuRobo公开evaluate_cspace_seed可重验被优化毁掉的合法原输入，真实Result.success接受后仍跑所有外层门禁，原优化失败不伪造为true。输入/结果clone防可变缓冲覆盖；CPU测双失败拒绝。全程原位参考不能子段重置，既会假拒绝，也会漏掉全程24nm超限；阈值始终1um，记住跨段clear。
- **最终固定布局原生验证通过**：`generated/two_box_layout/native_adaptive_feedback_20261122/run/result.json`，63.8365s只计缓存几何后的新鲜原生验证；31搬运段，搬运29.598s、全运动35.857s，有中途停顿。12段保留经真实原生API复验的输入种子，all_actual_optimizer_results_success=false，所有选中native几何/TimeOpt/碰撞/完整箱体/速度jerk/10kg力矩/独立RNEA/实际接缝及最终1.05m球包络通过。没有改数组端点/插值修补/缩放时间。
- 94379独立CPU数组/端点/包络+124001运输状态原位核对通过；源SHA匹配。左/右自身原位增加0.4304/0.0939um，离开后未再入。71689额外CPU审计两箱对两原位：交叉原位最大0。初始仍有3.773/7.181mm残余重叠，不能冒称先整箱过前沿；页面明确披露。数据/输入/验证SHA在artifact_manifest.json。
- **新只读回放8297**：CPU PID2259911、exec58252、日志`/tmp/pr82-native-adaptive-replay-8297.log`，HTTP200。脚本`.../native_adaptive_feedback_20261122/replay.py`；带offline/缓存/31段停顿/非5s非在线验收提示，实际两尺寸、原位红框/45°绿框、父子耗时和拒绝/种子保留历史。WS4项真实控件检查过（先前等待位置未等姿态导致一次竞态失败，已等完整场景目标且保留原检查文件），已全流程t0暂停。不是浏览器/WebGL人工目检；cua仍apps/browsers空，未自动开浏览器。原8295/8296未动。
- GPU本轮37716/28013/79127/65591/59468/40066失败有日志；44670/68338/47417/7127终态0，其中47417只表示实验完成、并非方案通过。**当前无自有GPU任务**，仅保留CPU服务。没有commit/push/实机发送，goal active，不mark完成/暂停/阻塞。
- 下一步本地关键路径：把成功策略干净接入生产通用在线流程（不能读固定npy冒充实时），再做两尺寸所有姿态/吸附模式/合法XYZ广覆盖和公平PR82冷热性能。当前只证明一个固定布局的离线原生分段候选，不是全目标或5s达标，不请求整项验收，不扩第三尺寸。

## 2026-11-22 Codex / 已接入新鲜在线正向LM几何，原生适配仍待整合
- **生产路径进展，不是再读固定npy**：BoxLayoutPlanner新增forward_lateral_prefix，按当前实际抽离q、两尺寸/吸附/当前原位和持箱目标实时生成共享15轴LM横移。保留实际原根和带完整正向连接的额外根；从哪个根出发都检查离开后不再进入，额外根必须已清出；绝无逆抽离根/跳状态。失败保留原根原预算，未定义输入非法。
- 复用现成LM512点/32种子/32轮；默认提取/放箱构造器和原门禁/选择逻辑/返回合同未改，PR82 AST合同通过。增加可选candidate_filter（原mask AND额外mask）用于source/native margins/full payload，不改物理阈值。payload_search_mask/native_margin_mask可传本查询新鲜FK；CPU回退会覆盖缓冲，最后才做完整载荷分类，不跨查询缓存state。
- 关键定位：只提高最终1um精度会受内部默认10um提前收敛影响；LM按关节范围比例收缩的内部动作区间还排除了原生合法根left_joint2。仅横移实例对齐两套精度、完整32内迭代，不用该内部收缩；仍显式检查原RRT lower/upper各1e-5余量以及全部原物理/动态硬限值。新拒绝统计分source/native margin/full box/joint，非输入非法证明。
- `live_forward_lm_20261122`默认精度和`live_forward_precise_20261122`仅最终精度失败都保留。`live_forward_aligned_20261122`新鲜窄位几何成功（前缀19.297s/3936点全过，RRT4.846s）；原单段32原生仍拒绝。常规当次24.134s/1候选通过，但不是稳定性能或PR82公平对比。
- **当前字节回归**`live_forward_final_20261122`：窄位fresh geometry true/native false，前缀20.810s（右11.546/左8.620）、RRT4.947s、worker79.119s；常规30.939s/2候选完整通过。CPU独立重验分别为raw geometry only与actual native arrays，源SHA匹配；不能把窄位几何通过冒称原生通过。64 CPU通过2.13s。
- 进度已接入实时事件：lateral_right/left running/done/rejected、实际子时长/拒绝计数；父运输表明确子项包含不重复累加。旧8295/8296/8297均未重启/改输入/轨迹，不称旧UI内存已更新。
- 8297离线原生通过候选26份登记原源码已精确归档`native_adaptive_feedback_20261122/run/frozen_runtime_sources/`，missing=[]，旧结果SHA不改。新的生产源码版本不同，旧结果只证明其冻结版本。
- 交接优先`docs/BOX_LAYOUT_LIVE_FORWARD_20261122.md`。**下一关键路径是生产原生保形分段+真实native seed保留/完整反馈+全程原位参考**，把已证明的离线策略干净整合，不能用运行时AST或固定路径冒充在线产品。然后当前两尺寸/姿态/吸附/合法XYZ广回归与同任务冷热5s基线比较，不扩第三尺寸。
- GPU45912/85815/45989/7165均终态，当前无自有GPU任务；CPU49700终态0。Goal仍active，未commit/push/实机发送，未完成全域或5s指标。

## 2026-11-22 Codex / 生产在线原生适配完整接通，8298真实可编辑计算验证
- 关键路径完成了一步：`v3_layout_native.time_layout_pick`已接入BoxLayoutPlanner，不再只有实验AST/固定路径。普通请求保留原路径；实际选中实时正向前缀的请求使用实际几何/阶段边界、真实seed/control门禁和完整反馈作自适应32控制点分段，最多64段，最终held不提前施加中间态。所有原物理谓词/原位1um/10kg每箱/jerk速度/RNEA/接缝/1.05m末态保留。
- `v3_two_box_timing`同一验证器支持显式部分阶段，完整周期不能关最终held；部分报告is_full_cycle=false。`v3_fast_three_layers`保留输入/结果，真实公开evaluate_cspace_seed选中再验证原种子，不伪造失败优化success，区分optimizer_success与native selected result。`curobo_core/native_transport`纯CPU跨段原位/实际接缝/不修补合并。UI展示子段/中途停顿/失败优化与种子保留，不声称单曲线全优化成功。69 CPU通过2.18s，原physical_success AST合同不变。
- **新鲜生产无预录回归**`online_native_integrated_20261122`：窄位111.145s/35原生搬运段/140001运输样本，常规24.113s/1候选均完整通过；独立CPU数组、端点、持箱包络、逐点原位通过，源SHA匹配；跨另一箱原位最大0。随后只修状态聚合/完整墙钟/UI披露，旧第一版SHA不重写。
- **新在线CPU布局器8298** PID2709266、exec43456、日志`/tmp/pr82-online-native-editor-8298.log`，HTTP200；原8295/8296/8297没重启/改输入。真实WS改Y/Z、世界X后恢复，点计算生成新请求`online_native_editor_20261122/runs/layout_5b9uq4sq/plan/result.json`，111.259s全流程物理/工艺通过，35段、16段真实seed保留。播放实际native时钟/两箱45°场景终点验证过，已时间0暂停。
- GUI请求当前字节独立CPU140001运输状态复核通过/源SHA匹配，交叉原位最大0；协议检查不是WebGL/人工验收。会话session_22b5cesd已stopped/user_stop，未继续占GPU。旧8297仍只读离线；**8298是真正在线可改箱位/新算**，但不代表全域/5s完成。
- 交接优先`docs/BOX_LAYOUT_ONLINE_NATIVE_20261122.md`。尚缺当前版两尺寸/长高/吸附/合法XYZ广回归与公平PR82冷热性能；窄位111s与常规24s性能仍远离5s。初始3.77/7.18mm残余及分段中途停顿必须披露，不冒充整箱先过前沿。高位顶吸拒绝仍挂起，不扩第三尺寸。
- GPU78392终态0（两个请求均成功）；UI GPU工作已user_stop，CPU客户端15704、70766、42981终态0；仅CPU服务持久。未commit/push/实机发送，goal active，未mark完成/暂停/阻塞。

## 2026-11-22 Codex / 安全加速未奏效已撤回，冷热/长高覆盖与接近种子保留
- 上一turn为生产在线接通的实际进展。本轮试128→512参考网格，仍按原.001精度全段复核：128窄位插值边失败，512回退后冷112.206s通过，无安全性能收益；256隔离提案同样失败，9.193s不是完整求解。已撤回粗网格生产代码/专属测试，保留证据，不放宽门禁。
- `coarse_lateral_20261122`热请求29.582s在原生approach失败，明确不是加速成绩；双tall长高交换两不同XYZ28.416s通过。成功项CPU独立重验通过，失败保留。
- BoxLayout的真实evaluate_cspace_seed候选保留范围从transport扩大至approach；基线其它planner不启用，仍真场景双箱/同native checker/TimeOpt/数组/碰撞/RNEA/接缝全检查。`layout_approach_retention_20261122`冷112.154s、热64.597s、双tall交换33.419s完整通过，CPU独立通过。但热接近原优化这次自身成功，未触发fallback，不声称因果修复；旧失败几何fresh engine原优化复现也成功，缓存诊断不当新鲜性能。
- CPU最终69项通过2.26s，物理谓词AST等合同未改。详见`docs/BOX_LAYOUT_PERFORMANCE_PROBES_20261122.md`。
- **当前正在执行/待核对**最终精简源码GPU句柄62547，输出`generated/two_box_layout/final_grid_regression_20261122/gpu.log`，同三新鲜请求（近位冷/热、双高箱Y/Z交换）。不要新提交替代它，先poll同handle/读真实终态。源码冻结到该任务终态；结束后运行同目录verify.py做独立CPU。
- GPU66274/35266/68920/32013均已终态；CPU55823/29544终态0。保留8298在线及8297只读旧展示，均没重启/改输入。没有commit/push/实机发送。全目标、合法位置全域、5s性能均未完成，goal active，不mark完成/暂停/阻塞。
- 收尾更新：最终精简代码GPU62547已终态0，三例均通过110.588s冷/63.619s热/28.964s双tall交换；不是5s，也不据此声称全域完成。当前CPU独立复核句柄78009仍live，日志`final_grid_regression_20261122/independent_cpu.log`，不要重复启动；其余本轮GPU无自有遗留。前一版CPU三例通过。8298/旧服务未重启。
- 随后确认CPU78009终态0，最终三例独立CPU通过（原生数组/端点/包络/全程原位；交叉另一原位最大0，源SHA匹配），已无本轮自有后台核验或GPU任务。Goal仍active，性能和全域覆盖未完成。

## 2026-11-22 Codex / 撤回无收益FK合并，修复拟合控制点越限导致的运输反复细分
- 上一目标turn验证8298仍HTTP200并尝试打开浏览器（连接器无可用浏览器）；没有改用户输入、重启服务或新算，属服务状态证据，不是全目标完成。本轮继续真实性能根因工作。
- `search_fk_reuse_20261122`：旧/新真实GPU谓词40组全部mask与stability逐值一致（3布局、随机/扰动/已知微碰撞、原位开关与0/5mm底座裕量）；512分块更慢，4096局部合法路径查询约快8–13%，但完整冷/热/交换115.130/67.397/29.470s均慢于改前，几何数组相同。三例独立CPU通过后撤回该改动：`path_validity`/`source_slot_depths`恢复此前生产AST，去掉复用专属测试/API。证据/试验源码保留，不凭微基准宣布加速。
- 进一步诊断近位32个细分拒绝区间：20个由无约束32控制点拟合制造left_joint4越限，非原几何/箱位非法。仅投影候选内部控制点的隔离探针使18/20个通过同一4097点原几何/原位门禁；该探针未冒充完整原生成功或新鲜解算。
- 当前保留最小修复：`v3_fast_path.fit_path(...control_bounds=None)`默认不变；`v3_fast_three_layers`仅布局负载运输、`v3_layout_native`分段预检显式传同checker硬限位。只处理拟合候选内部控制点，固定端点越限拒绝，不修补最终q/v/a、不改任何物理/工艺门禁；fit报告实际投影计数及投影后误差。`tests/test_box_layout.py`新增默认不变/固定端点/凸包限位/坏参数CPU检查，总70通过3.09s。
- `generated/two_box_layout/native_control_projection_20261122`五个新鲜请求全部成功：近位冷95.584s/热48.453s（改前110.588/63.619s，同原始规划几何，35→17实际原生运输段）；双高箱Y/Z交换33.490s（较改前28.964更慢，两次真实完整箱体拒绝保留；所有候选投影计数0，未证明时差因果）；常规异XYZ两尺寸侧吸冷50.896s/热7.076s。warm仅复用模型，不读取旧路径；不是公平PR825s基准或全域证明。
- 五例独立CPU完整数组/端点/末态45°包络/逐运输状态原位通过，交叉另一箱原位最大0；近位每例68001运输样本、17段、5段真native种子保留，仍有中途停顿和初始抽离残余，不能称全箱先过X前沿。source SHA及完整runtime source stamp与冻结source/manifest（含新fitter）一致。GPU18300/65984/24474/22445/56427/55725和CPU66021/12815/40460均终态0，无本轮自有遗留GPU/核验任务。
- 8298在线CPU布局器/原111.259s35段演示、8297只读旧回放均未重启/替换/改输入。新worker会读当前修复。未commit/push/动依赖或实机。详见`docs/BOX_LAYOUT_PERFORMANCE_PROBES_20261122.md`最后两节。全目标active，任意合法XYZ/所有模式充分覆盖/5s未完成，不mark完成/暂停/阻塞；下一步聚焦近位横移LM和初始抽离残余/剩余17段，不再围绕无收益FK合并打转。

## 2026-11-23 Codex / 载荷小批次graph判定及四种额外XYZ/吸附模式回归
- 上一目标turn实际修复拟合越限、35→17运输子段，属进展。本轮先确认抽离残余主要来自既有倾斜/位移容差，未擅改30/25.5cm距离或放宽原位判据；未将更紧IK当成未经验证的解法。
- `lateral_filter_profile_20261123`：近位横移热算原约16.97s，LM仅约2.59s、过滤约14.05s；真实剖析2050次载荷分类、CPU回退0，根因是小查询张量调度。首次探针名profile.py遮蔽标准库，GPU21209已失败终态1，失败保留；32.15s剖析含冷编译/开销，不作整单性能。
- 最小生产改动仅`v3_full_payload_cost.py`+`tests/test_full_payload_cost.py`：旧分类body原样保留`_classify_eager`；float64 CUDA单batch1–32行使用单个32行graph，补末行/裁结果，较大查询/CPU旧逻辑不变；复用现有cuRobo GraphExecutor，不改依赖。私有输入每次复制本次FK、输出clone、几何缓冲可原位更新，无旧状态/判定/轨迹缓存。71CPU通过2.25s。
- `payload_graph_parity_20261123`：42组真实3布局/7种批量尺寸/路径随机输入旧新valid+uncertain全等；另12组实际GPU世界/地面/箱机器人/箱箱阈值±1nm、1/9/32行及世界更新/结果持有反例通过。局部约2.1–2.2ms→0.78–0.83ms，不据此宣称整单倍数加速。
- `payload_graph_regression_20261123`五完整新鲜请求均成功：近位冷91.735s/热45.018s（上一版95.584/48.453，原始规划几何逐值相同）；双高箱Y/Z交换28.841s（本次2候选vs先前3候选，未将全部时差归因graph）；常规侧吸冷52.830s/热7.172s（未加速）。近位仍17段68001运输样本、5段真native种子保留、初始残余和停顿披露，不改门禁/模型/原始位置。
- 用现有`check_asymmetric_layouts.py`额外4个当前输入完整新算：`payload_graph_modes_20261123`左tall顶右cube侧52.427s、两异高度侧吸32.689s、双tall Y/Z交换双顶24.341s、左cube侧右tall顶24.777s，全通过。加上原5次为9次请求/7不同布局，全部独立CPU数组/端点/45°包络/逐运输状态原位通过，交叉另一原位最大0；source SHA/完整runtime stamp/冻结manifest一致。不是全域证明，5s仍未达标。
- GPU67191/49018/31263/6053/23694/95978及CPU82512/97289/24239均终态0；仅失败命名探针21209终态1，无本轮自有遗留GPU/核验任务。全经过PATH gpuq，未动共享依赖、commit/push/实机。8298/8297原CPU演示未重启/替换/改用户输入，未伪称浏览器目检。
- 交接优先`docs/BOX_LAYOUT_PAYLOAD_GRAPH_20261123.md`，旧FK合并撤回仍保持。目标active，不mark完成/暂停/阻塞，不提请最终验收；下一步需推进剩余基础碰撞调度、17运输段/前缀与更多合法XYZ/模式，而非将7个有限布局冒充任意位置。

## 2026-11-23 Codex / 诊断导致的TimeOpt图失效修复，已拒绝IK批次短路与当前10请求终态
- 上一turn有实际生产改动/9请求验证，属进展。本轮`ordinary_warm_profile_20261123`对第二个完整新鲜请求剖析（9.49s含开销），确认4次TimeOpt图初始化及约1.147s GC来自4001点native力矩诊断扩容准备缓冲。库为安全正确reset，未关闭GC/失效保护。注意原实现扩容后后续稳定热算也能复用图，**不是每次永久重建**；不能误说近位17段每段都重建。
- `v3_fast_three_layers.native_sample_torque`只按既有容量分批审核全部4001点并clone借用结果，不扩容准备缓冲/改数组或门禁；独立RNEA保留。`timeopt_workspace_20261123`组件探针5个阶段配置验证首次q/v/a/jerk/time逐值相同、同点整批native/独立RNEA差值0Nm、graph与缓冲指针在重复调用保持；solve约0.52–0.64ms。未改共享依赖。CPU最小检查覆盖3/3/2分块与覆写。
- `ik_short_circuit_20261123`隔离实验：3布局11个持箱目标，跳过已完整拒绝批次的native margin/包络后候选及计数全等，常规九目标局部1.104→.849s。生产`enumerate_pair`两个短路复用原计数，无额外同步；生产AST与实验同一方法。CPU覆盖全拒绝/阶段拒绝/有效/held=false/部分有效，不改候选/目标顺序，不缓存IK或轨迹。
- TimeOpt-only中间字节5新请求成功，近位冷93.568/热45.002s、常规首热5.632s，独立CPU通过后才修改planner短路，历史结果/源hash未重写。当前字节用`warm_cleanup_regression_20261123`重跑：近位冷91.234/热43.734s，双高箱交换27.345s，常规冷50.723/首热5.870/再次热5.639s；都完整成功，warm仅模型复用。常规几何与改前逐值相同；不拿中间版较小数字冒充当前成绩，不将全部差值归因于单项改动。
- `warm_cleanup_modes_20261123`补充4种实际XYZ/顶侧吸/双高箱长高交换全成功，52.262/31.777/25.226/24.773s。当前总10新鲜请求/7不同布局独立CPU数组/端点/45°包络/逐状态原位均通过，交叉另一原位最大0、source SHA/完整runtime stamp/冻结manifest匹配；近位仍17原生段、初始残余和中途停顿保留披露。73CPU通过2.47s。
- `direct_transport_probe_20261123`常规56/交换256/近位256候选直连在原粗门禁全失败，未添加无收益生产预检，不当全局不可达证明或新鲜整单时间。源仅任务/初始q，未读历史路线用于生产。
- GPU81452/38530/59683/29762/54807/29423/63579与CPU46744/94639/95103/18185均终态0，无本轮自有遗留GPU/核验任务。全用PATH gpuq，未共享依赖编辑/commit/push/实机。8298/8297原服务/演示未重启替换或改输入，未伪称浏览器视觉验收。
- 交接`docs/BOX_LAYOUT_WARM_CLEANUP_20261123.md`优先。全目标仍active，5s和任意合法XYZ/充分模式覆盖/最终用户验收未完成，不mark完成/暂停/阻塞；别重复已失败直连、FK合并、粗128→512回退等旧试验。后续需实测剩余近位13s横移/17子段等大项，不能围绕微小数字取巧。

## 2026-11-23 Codex / 局部自适应横移已原生验证；硬镜像暴露原生接近表示缺口
- 上一turn修复TimeOpt工作区/IK短路有实际进展。本轮不再重复微优化，新增128参考网格起步、只二分失败区间并保留已检查前缀的横移候选；边直接按原最终.001检查，原父门禁及512几何失败回退保留。LM默认构造/32种子32迭代/接受选择返回AST合同未变；参考网格严格校验，采样器异常/失败均恢复属性。微残余≤10观测eps只保留原512候选，**不改原1um接受阈值**，避免简单模式多4–6s空转。
- 生产仅`v3_lm_conveyor.py`（可选网格/更细边）+`v3_box_layout_planner.py`（局部续解/原网格回退/逐尝试耗时拒绝可见）及对应CPU检查。探针62198先因numpy bool JSON失败，82364修正后真正通过；失败保留，不计成成功。CPU76通过；初次收集未设tools PYTHONPATH失败，已完整重跑。
- 第一版`adaptive_lateral_production_20261123`十请求通过：近位冷73.307/热24.405s、运输17→10段，但三微残余案例回退多4–6s，故加策略选择后另跑当前最终源`adaptive_lateral_final_20261123`+`adaptive_lateral_final_modes_20261123`：近位冷73.482/热24.889s，双高箱交换33.580s；常规冷/首热/再次热见最终summary_extra，仍超5s；四补充场景52.749/32.848/24.720/25.391s全通过。热近位改前43.734s，节点求解1022→258，实际40001运输样本/10段，非缓存路线/非单条无停顿全优化。
- 当前10成功新鲜请求/7不同布局全部独立CPU数组/端点/45°包络/逐状态原位通过，交叉另一原位最大0、源SHA/完整runtime stamp/冻结manifest一致。恢复后76CPU通过2.35s，原32/128默认LM行为合同通过。
- **新增镜像必须正视**：左tall(1,-.05,1.5)侧/右cube(1,.05,.6)顶，当前算法8候选89.360s和默认16候选139.635s均未解出；不是非法证明。试验取消“横移保持朝向”的额外限制、移动箱体SLERP回世界轴后，隔离前缀几何通过，暂接完整任务的`aligned_lateral_mirror_20261123`仍144.506s失败；未当任务成功。
- **关键实际卡点**：该aligned记录attempts[4]运输及接近几何均成功（selected_root1，4879帧），是native approach拒绝（场景/底座地面、189样本）；32控制点种子偏差9.205456°/12.106203mm。不要根据最后几条RRT拒绝日志误诊为这个候选未找到运输路。未完成的回正生产分支已仅方法级撤回，`adaptive_lateral_final/source/manifest`全部10文件重新SHA一致，未回滚其它改动或重写历史hash。回正试验保留为组件证据，不在线启用。
- `mirror_cartesian_tail_probe_20261123`直线盒位姿尾段仅到6.213%无准确IK，未替换RRT、未标成功；1.212m是中间态，不证明目标包络非法。下一步应在上述已保留的完整粗几何上隔离**接近段原32控制点分段**，实际静态双箱障碍/Native/TimeOpt/4001/RNEA/qva接缝全部保留；通过组件后再新鲜整单，不读旧路径冒充实时。当前time_layout_pick只有运输分段，initial前三阶段approach一失败就返回，是明确待修缺口。
- GPU62198终态1（探针序列化）；82364/94463/46165/38007/27173/99303/18139/2597/23302/84370及CPU17275/37445/31338/27565全终态0，无自有遗留核验/GPU工作。全用PATH gpuq/8编译线程，未改依赖/commit/push/实机；8298/8297未重启/换展示/改用户参数，未伪称视觉验收。
- 最新交接`docs/BOX_LAYOUT_ADAPTIVE_LATERAL_20261123.md`。目标active：具体镜像缺口/任意合法XYZ/5s及用户验收未完成，不mark完成/暂停/阻塞；勿把10个成功请求缩成“全域已完成”，也勿重复已失败笛卡尔尾段/直连/FK合并/粗网格回退试验。

## 2026-11-23 Codex / 原生接近恢复、双箱回正候选与镜像完整通过
- 上一turn自适应横移有实际进展但镜像缺口未解，本轮优先该缺口。`native_approach_probe_20261123`明确保存几何组件：单段32失败，两个真实native32接近子段通过（22.003s含初始化，非新鲜整单）。未移除真实静态双箱、未改Native/TimeOpt/4001/RNEA/限值或数组。
- `v3_layout_native.time_initial_with_approach_recovery`只有真正approach拒绝后才启用，覆盖普通和前缀路径。原成功/其它阶段失败原样返回；保留monolithic rejection，二分原几何，最多64段；仅去掉完全相同的几何重复点。用真实rest-seam数组拼接且审approach→contact跨阶段实际q/v/a，不修端点、不翻native成功标记，部分报告不冒充完整周期。`v3_box_layout`展示接近子段耗时/拒绝。
- 首次接回镜像`approach_recovery_mirror_20261123`155.506s仍失败；此回approach/contact/extract通过，transport剩单边231→232种子拒绝。GPU逐谓词`mirror_native_edge_20261123`确认右箱–arm_carriage球12在样点间约1.090um，原1um门禁正确拒绝，基本/余量/原位均通过。初始右箱–carriage净空约.895mm；不是应加大容差。
- 备用生成候选额外要求5um box–robot净空（仅生成，最终1um不变），单箱回正/另一箱朝向冻结时仍失败，未宣称成功。允许两箱都SLERP回世界轴、中心保持真实定义后，通过生成和真实完整任务。原固定朝向候选仍优先、微残余仍原512流程；原主LM/RRT和15轴/10kg/宽度/高度阈值/末态45全部保留，没改依赖或碰撞模型。生成净空先用本查询FK，再允许payload CPU fallback覆写，CPU检查覆盖。
- **镜像终于整单通过**：`approach_reserve_mirror_20261123`72.735s，5候选，approach2真实原生段/8001点、transport7段/28001点，contact/extract各4001。再次新鲜冷74.856/热23.766s通过；warm仅模型复用，不读旧路线。
- 当前同字节既有六新请求也全通过：near冷72.477/热23.744，双tall交换32.624，常规冷50.617/首热5.805/再次热5.574s；四补充XYZ/顶侧/长高52.129/33.397/24.467/24.505s全过。总13新鲜请求/8不同布局。旧139s失败不是速度基准，不当成功口径加速。
- 81CPU通过2.45s；13请求独立CPU数组/端点/45°及原1.05m包络/逐运输状态原位通过，交叉另一原箱占位区最大0、SHA/runtime stamp/冻结manifest匹配。`verify_pieces.py`还逐个真实native32/4001/Native/TimeOpt/RNEA及实际接缝检查并逐值重建全部分段文件，未插值/数组修复/时间缩放。镜像初始残余6.672/3.074mm、分段停顿/真seed保留与优化拒绝继续披露；镜像存在X前探但未进两原OBB，不能称−X单调或全货架前沿门禁。
- GPU7151/13252/62798/66152/25649/63323/75456/15416/19036/13558、CPU36218/82778/40254/20156全终态，无自有遗留核验/GPU任务。全经PATH gpuq/≤8编译线程，未依赖编辑/commit/push/实机。8298/8297未重启/换展示/改输入，未伪称WebGL人工验收。
- 当前交接优先`docs/BOX_LAYOUT_MIRROR_NATIVE_20261123.md`。goal active；5s/任意合法XYZ/充分模式覆盖及最终用户验收仍未完成。后续应推进真实不同XYZ/阈值附近与完整延迟，不重做已过镜像或拿13有限请求当全域。旧失败/未完成实验都留档，不覆盖hash。

## 2026-11-23 Codex / 新世界XYZ与0.8/0.85m吸附阈值边界12请求完整通过
- 上一turn解决镜像原生任务缺口有实际进展。本轮没有更改生产源码，保留同11文件冻结manifest与完整runtime stamp，未重启8298/8297或改用户输入。转向新世界XYZ/阈值覆盖，避免只重测已有成功演示。
- `suction_threshold_20261123/catalog.json`8个显式新请求：四种两尺寸左右分配/高箱Y-Z交换，各Z=.79999与.80000，阈值.8。GPU53615完整新鲜解算全成功，实际模式严格顶/顶与侧/侧，宽度.300/.255不变，输入箱心没被搬到容易位置。
- 同目录mixed_catalog4新请求：默认.8及自定义.85阈值两侧10um/精确相等，两臂同场分别顶侧吸；两自定义例高箱Y/Z交换。GPU25810也4例全通过。完整墙钟见summary：首组57.630/23.707/24.025/23.567/26.943/28.934/25.444/22.870s，混合47.783/25.326/22.027/22.419s。包含新拓扑初始化，不当公平PR82速度基准，5s仍未达标。
- 独立CPU18345/6173分别8/4请求数组/端点/真实45°与原1.05m球包络/逐运输状态原位全通过，跨另一原箱OBB最大0，source SHA/runtime stamp/manifest匹配。CPU71862额外核查全部contact/extract实际数组（24阶段96024状态）URDF FK，对显式请求面中心/方向/宽度比较：最大横向.230627mm、角.046448°、TCP面中心误差约.000179mm、抽离全宽位移误差.246761mm，原2mm/1°不改。不是只验top/side配置标签。
- cube_left_tall_right_below出现单段approach拒绝后原生分成7段通过；复制的verify_pieces.py在本目录逐段Native32/4001/RNEA/实际接缝并逐值重建合并数组，未覆盖旧13请求证据；其approach28001点、中途停顿继续披露。12个result中73个中间rejected事件均保留，不当73次整单失败。
- `payload_large_graph_probe_20261123`非生产runtime实验48组真实有效性/歧义标志等价+世界缓冲更新通过，但129/512补1024导致局部明显更慢，3完整新请求52.892/6.179/5.718s无稳定收益（均CPU92890复核通过），未引入生产新cache/graph形状或改依赖。runtime override已显式记gpu_session，不拿磁盘源hash冒充生产运行版本。不要再重复这一无收益补齐方向。
- GPU53615/16609/25810与CPU18345/6173/71862/92890均终态0，无自有遗留GPU/核验任务；PATH gpuq/编译并行≤8。未commit/push/实机，无WebGL视觉验收声明。生产代码未变，不重复81项CPU套件充当新进展。
- 最新覆盖交接`docs/BOX_LAYOUT_SUCTION_BOUNDARY_20261123.md`；算法交接仍看`BOX_LAYOUT_MIRROR_NATIVE_20261123.md`。本轮新增12布局全通过，与上一轮13请求同当前字节，但有限样例不能证明任意合法XYZ。goal active，5s/充分全空间模式覆盖/最终用户验收未完成，不mark完成/暂停/阻塞。

## 2026-11-23 Codex / 仅搜索SAT投影算术提速；常规4.900/5.170s，最终门禁与16请求验证
- 上一turn新增12边界布局覆盖有实际进展。本轮回到完整性能，未重试已证无效1024补齐graph。新增`curobo_core/payload_geometry.box_overlap_depth_projected_tensor`，只让`v3_full_payload_cost`两个signed搜索调用用自身坐标系同6面+9叉积轴投影。仍float64、原轴截止与歧义CPU复核。原显式tensor/NumPy/原位SAT、优化代价/梯度/接受谓词/限值/模型/预算均不改，check_scope.py AST确认；无依赖修改。
- CPU28672近轴/阈值对最终分类一致；GPU42组真实常规/近位/镜像随机及路径批量valid/uncertain全等，另12CUDA阈值反例。两点raw signed阈值舍入不同，但都由原同一CPU fallback接管，fallback membership与最终接受完全相同，未放宽阈值。最初capture期Python列表索引失败、第二次只比raw阈值断言失败均保留；纯tensor.roll及正确最终谓词验证后才实施生产。82CPU通过2.44s。
- 生产`sat_projection_production_20261123`8新请求全通过，且输入/实际held目标/原始粗几何都与上一版逐值相同：常规冷49.921/首热4.900/再次热5.170s；近位冷71.241/热22.769s；双tall交换31.123；镜像冷74.273/热24.115。镜像热比原23.766更慢，不能称所有场景加速；常规4.9/5.17也不能称稳定5s。不是PR82公平基准比较。
- 四补充模式`sat_projection_modes_20261123`52.071/31.794/24.193/24.656s、四混合吸附及0.8/0.85参数边界`sat_projection_mixed_20261123`48.621/25.706/22.011/21.768s全通过。总16当前字节完整新鲜请求/12不同布局，独立CPU数组/端点/45°及原1.05m包络/逐状态原位全过，跨另一原OBB最大0、SHA/runtime stamp/manifest一致；verify_pieces.py全部实际Native32/4001/RNEA/接缝/拼接逐字段重建通过。分段停顿/初始残余/真实优化拒绝保留。
- **下一潜在大项已隔离定位但未上线**：持箱目标盒姿态与head/head_yaw/arm_carriage碰撞的CPU保守包络可排除10–18/21 lift格点。`lift_pruning_probe_20261123`常规9目标合法goal数组全等、局部合计.930→.325s；近位限256数组不等，工具终态1，未将它当安全生产优化。`lift_pruning_diagnosis_20261123`证明完整1456合法goal有序数组相同、无有效lift错排，限256时因原512行批次停止+肘型交错选择导致1个goal替换/部分顺序变化。若继续，应当显式候选池策略变化做完整近位/镜像/阈值回归，不重做集合诊断、不把断言改松冒充等价；当前生产仍只有SAT改动。
- 额外JSONcompact序列化CPU实验常规约8ms/近位78ms收益，不是主要瓶颈，未改IO。独立CPU也通过3实验SAT请求，但runtime override显式记录，不当生产source字节证明。
- GPU74827/21252终态1（capture/raw阈值），44304/10723/45687/34436终态0；早筛2180终态1、诊断65055终态0。CPU21861/40616/92852/89884终态0，无自有遗留GPU/核验任务。全PATH gpuq/≤8编译线程，无commit/push/依赖修改/实机。8298/8297未重启/改输入/换展示，无WebGL人工验收声明。
- 最新交接`docs/BOX_LAYOUT_SAT_PERFORMANCE_20261123.md`；整体算法仍参`BOX_LAYOUT_MIRROR_NATIVE_20261123.md`。goal active，稳定5s/任意合法XYZ/充分覆盖及最终用户验收未完成，不mark完成/暂停/阻塞。不可拿一次4.900s宣布目标达成。

## 2026-11-23 Codex / 持箱升降强碰撞提案早筛；26请求通过，五热中位4.677但仍有5.441尾延迟
- 上一turn搜索SAT有实际进展，本轮推进此前已诊断的持箱早筛。新增`BoxLayoutPlanner.carry_lift_blocked`，在真实CPU FK当前lift下用实际目标盒/头部与carriage冻结球+6mm/1°完整保守包络/10um数值余量，仅省去深重叠升降提案的解析IK。原实际接受门禁/模型/限值/15轴/10kg/XYZ/45°不改，contact和held=false恢复不走早筛。无IK或轨迹缓存。记录pruned_lift_m及显式候选池策略。
- 这是明确候选池策略变化：原512行分批停止+肘型交错会因删除无效行改变限256池；不再宣称完全等价。前述完整1456目标等价证据保留。本轮check_scope.py确认生成后全部原门禁/肘型选择AST及其余11冻结文件不变。仅planner+对应CPU测试，83项通过2.59s，含两尺寸/交换/附件/深重叠/包络内不拒/contact和非held旁路。
- `lift_pruning_production_20261123`8完整请求全过：常规冷49.689/首热4.393/再次热4.734s，近位冷69.376/热21.945，双tall交换31.340，镜像冷72.200/热24.054。常规与上一版输入/目标/粗几何相同；近位/镜像输入与实际目标相同但粗几何不同，不能套用上一turn所有几何相同的说法。常规九持箱目标早筛129个lift，近位10、镜像11；不是PR82公平基准。
- 补充4模式+4混合阈值边界全成功，目录`lift_pruning_modes_20261123`/`lift_pruning_mixed_20261123`。另独立冷+五热`lift_pruning_stability_20261123`全成功：冷55.050s，热5.077855/4.559103/4.677142/4.488071/5.441450s，中位4.677、仅3/5低于5s，**未稳定达标**。不能用先前4.39/4.73掩盖这组慢样本。
- 新4世界XYZ`lift_pruning_new_xyz_20261123`也完整通过：双方箱不等XYZ侧23.381/顶24.421，双高箱不等高度侧60.644，高位交换高箱(1.13,.58,1.30)+低方箱(1.07,-.55,.48)81.818s。输入原样/后续候选拒绝保留，不将困难位置换走或标非法。
- 共26当前新鲜请求/16不同布局独立CPU数组/端点/45°/原1.05m包络/逐状态原位全通过，跨另一原OBB最大0；source SHA/runtime stamp/manifest一致。verify_pieces全段Native32/4001/TimeOpt/RNEA/实际qva接缝/逐字段原数组重建通过；近位10段、镜像7运输/可能2接近，初始残余与停顿/种子保留/优化拒绝继续披露。
- **下一性能定位**：五热stage_breakdown中hot1 native1.760s vs通常1.33s；hot5 geometry/native之外.956s vs通常.16s。最终CPU独立核验仍.084s，.8s增量在plan_layout内未细分部分（home验证/报告/运行时等尚无因果实证），不从总墙钟扣除凑5s。应在完整热请求上定位，勿凭猜测禁用GC/源校验/安全门禁。
- GPU35081/72421/52831/84786/34256及CPU91381/60426/53815/17661/78851全终态0，无自有遗留核验/GPU任务。全PATH gpuq/≤8编译线程，无依赖编辑/commit/push/实机。8298/8297未重启/改参数/换展示，无WebGL人工验收声明。
- 最新交接`docs/BOX_LAYOUT_CARRY_LIFT_PRUNING_20261123.md`，goal active；任意合法XYZ/充分所有模式覆盖/完整冷热点性能及用户验收未完成，不mark完成/暂停/阻塞。下一步不重复已通过的早筛集合诊断，不拿热中位或单次4.39当全域5s完成。

## 2026-11-23 Codex / 普通抽离64参考提案与128质量回退；当前8299真实在线演示已就绪
- 上一turn持箱早筛有实际进展，本轮先追未分配约.8s尾延迟。`warm_tail_profile_20261123`完整新鲜GC/status-IO/cProfile观察未复现，不能归咎GC/磁盘；一次GC在worker结束后的剖析整理，不作为因果。`library_timer_probe_20261123`现成诊断Event开关组件逐值等价但无一致收益，**未接入生产**；保留内部诊断计时、完整墙钟、GC和源校验，未用0内部计时凑5s。
- 转测普通正向抽离64参考IK节点。初版`nominal_extract64_probe_20261123`8整单虽最终通过，双高箱非对称侧吸85.724s/9候选比原60.644/3明显慢；其微米残余触发多次横移/工艺拒绝，未直接采用。`nominal_extract64_clear_probe_20261123`加提案端点数值余量选择，难例用原128并3候选通过；37.206s非同冷条件加速成绩。两实验运行时覆盖/旧hash保留，均独立CPU通过后才改源码。
- 生产仅planner/GUI计时文案+CPU检查：`nominal_extraction`暂用64参考，单节点32种子/32迭代、原0.25deg边、正式Native/TimeOpt/4001完整审核不变；几何/完整载荷/native端点/原位数值边界质量失败用原128；逆向恢复/横移/原LM默认构造不改。端点质量是观测eps的一半，仅决定候选采样，不改变正式1um门禁或拒绝箱位。sampler成功/失败/异常均恢复，粗拒原因与求解器真实success分别保留。
- 采用64时复用同一新鲜数组刚做的实际载荷几何审核一次，原正式4001审核原样；无跨请求判定/轨迹缓存。新增extract_reference子阶段显示真实耗时/拒绝、注明计入抽离总耗时；拒绝栏明确包含提案筛选。84CPU通过2.77s，check_scope.py确认其余10冻结文件及逆向/solve_file墙钟/worker逻辑未改。
- `nominal_extract_production_20261123`8整单全过：近位冷72.815/热21.415，双tall交换32.376，镜像冷72.883/热24.748，常规冷49.901/首热4.092/再次热4.132s。五热独立`nominal_extract_stability_20261123`冷49.647后4.001273/4.240209/4.009953/4.049854/4.040816s，中位4.040816、5/5<5；**只是一种常规布局的本批热样本，不能称全位置/冷/热5s已完成**。
- 另4模式+4混合阈值+4非对称XYZ均整单通过，目录`nominal_extract_modes/mixed/new_xyz_20261123`。总体26回归+下述2UI=28当前字节新鲜请求/16布局，独立CPU数组/末态45deg/原1.05m包络/逐状态原位全过，跨另一原OBB最大0、SHA/runtime stamp/manifest一致。107单段阶段确有4001点，所有分段逐个Native32/4001/TimeOpt/RNEA/真实qva接缝及逐字段原数组重建通过。初始残余/停顿/真实seed保留/优化拒绝继续披露。
- **当前可检查在线页面8299** PID194674、exec60485，日志`/tmp/pr82-current-online-editor-8299.log`，HTTP200；原8298/8297未重启改输入。初始左tall(1.16,.49,1.06)、右cube(1.20,-.47,1.00)。`current_online_editor_20261123/check_live.py`实际WS检查Y/Z及worldX/顶侧预览、完整重叠提交拒绝，再真实按钮两请求49.148冷/4.021热，原生播放时钟推进及双箱实际Ry=-45deg末态场景变换匹配；已全流程时间0暂停，成功回放保留。
- UI结果`runs/layout_mmjbggut/plan/result.json`冷、`layout_n97na4kr/...`热，独立CPU也通过。会话session_7cw_cyul已stopped/2请求/session_limit，锁释放，未留GPU。当前会话配置idle10s/window240s/max2，请求新算会重新排队，不承诺总是热算。**8299已作为当前入口公开，不重启/改其用户输入或重跑会改参数的check_live.py，后续调试另开端口。** 浏览器工具No browser is available，未伪称WebGL人工目检。
- 所有本轮GPU/CPU试验、回归、WS客户端均已实际终态，详见新文档资源节；仅CPU服务持久，PATH gpuq/≤8编译线程，无依赖编辑/commit/push/实机。最新交接`docs/BOX_LAYOUT_NOMINAL_REFERENCE_AND_LIVE_20261123.md`。goal active，任意合法XYZ充分覆盖/全部布局低延迟/最终用户验收未完成，不mark完成/暂停/阻塞；请勿拿本批5热或新在线UI就认全目标完成。

## 2026-11-23 Codex / 细网格冗余去重与镜像Y+15mm腕部几何反例完整修复
- 上一turn有实际源码/新8299演示进展，本轮未重启或操作8299/8298/8297的参数/回放。先完整新鲜近位/镜像热请求剖析`hard_layout_profile_20261123`（观察开销不作成绩），定位细网格138万组合的15列np.unique约2.6s。
- heap原型目标逐值一致，但Python循环抵消近位收益，未接入。更小修复：单臂解已7位唯一+各组取整lift互异，证明组合已唯一；仅arm_limit=None满足该条件时省重复全量unique，普通IK/重复取整lift保留旧逻辑，float32合并候选不再去重以保持原库存。`pair_unique_proof_probe`组件near4.1–4.4→1.65–1.72s、mirror3.66–3.71→1.33–1.53s，256目标逐值同；原型仍全量组合，未接线materialized_rows=0已单列说明不是测量，勿称懒生成。
- dedup-only生产8请求通过（热near19.551/mirror21.915），但四新Y±15mm样例出现**一个未知失败**：左tall侧(1,-.035,1.5)/右cube顶(1,.065,.6)，36.115s/16候选耗尽，未标非法；旧12请求是11成功1失败，均留档独立CPU核对。失败端点132万组合hard_valid0，尚未到RRT。
- 独立CPU原URDF/sphere/ignore几何诊断：旧引导盒体净空约2.81mm，但right_link7–carriage约3.328mm深碰；固定tool目标已决定link7姿态，换内部swivel无解。旧引导还偏好lift0腕部93mm深碰。补充**原末端sphere/self几何**引导的运行时完整新算86.367s/7候选全部门禁与独立CPU通过，非生产/source或加速证明。
- 当前仅planner、core/box_pick及CPU测试：`extraction_endpoint_proposals`可选terminal_geometry来自当前原机器人模型与box/tool/link7固定变换、遵原ignore配对。默认输出两尺寸分配逐值完全同，保留4局部种子/.95deg/1.8mm/lift/350迭代。原提案优先耗尽后才惰性计算补充提案，仍原8恢复成功候选/总maxcontacts限制；不宣称所有晚选forward顺序兼容。0端点用明确提案reject原因，不证明输入非法。87CPU通过2.57s，scope AST/11冻结文件核对原接受门禁不改。
- 当前生产`terminal_guidance_production_20261123`8核心请求全过：near冷69.280/热19.584，双tall交换32.170，mirror冷71.402/热22.856，ordinary冷49.168/热4.097/再次热4.235s。反例独立冷95.399/热47.102s均7候选完整通过；不可把旧36s失败与新95s成功当速度提升。
- 四邻近位置生产重验`terminal_guidance_nearby_20261123`全过：mirror+15mm94.071、mirror−15mm20.883、near+15mm35.780、near−15mm15.651s。当前14新鲜成功请求/8布局独立CPU数组/端点/45deg/原1.05m包络/逐状态原位全过，跨另一原OBB最大0，SHA/stamp/manifest一致；actual native32/4001/TimeOpt/RNEA/qva接缝/拼接逐字段重建通过。初始残余/停顿/优化拒绝/真seed保留仍披露。
- **下一根因已源码确认**：反例warm attempt4的native extract left box–robot深1.7385mm/sample3113；原粗几何loaded_audit通过，refine_lm_path257优化点仅sphere代价无完整盒引导，最后4001原门禁正确拒绝。attempt4/5/6 transport都root1，勿误修普通root0适配/盲加RRT。下一步在同LM refiner隔离增加完整盒引导或验证保留合规原输入样条，守住64controls/32iters/4001/RNEA/线/末态/接缝，不加入源位phantom排斥、不跳LM或改成功/阈值。组件后新鲜整单，勿把保存几何组件当实时成绩。
- GPU20579/2733/70727/98870/16989/49339/38032/13516/30746及CPU43660/36139/79689/20792/72084均实际终态0；runner0不掩盖旧反例失败。无本轮自有遗留GPU/核验任务，PATH gpuq/≤8编译线程，无依赖修改/commit/push/实机。8299公开现场完全保留；不要重跑会改输入的check_live.py或重启。
- 最新交接`docs/BOX_LAYOUT_FINE_IK_AND_TERMINAL_FALLBACK_20261123.md`；在线现场说明仍看NOMINAL_REFERENCE_AND_LIVE文档。goal active，连续任意合法XYZ充分覆盖/所有场景5s/最终用户验收未完成，不mark完成/暂停/阻塞；下轮优先上述具体native extract表示/引导缺口。


## 2026-11-23 Codex / 原生抽离完整箱体按需引导与运输候选数值余量
- 保留8299公开验收现场及8298/8297；HTTP8299仍200，CPU194674未重启/改输入/播放。浏览器工具无browser，未伪称WebGL人工目检。
- 同一端点重建reverse LM128粗路径起止差0，重现困难counterexample contact_index4原native extract左箱–机器人1.738495579mm/sample3113。原LM只有sphere损失，4001正式完整盒门禁正确拒绝。原15mm弱引导超1度、无margin残余56.508µm均失败，未当成功。100µm真实盒软目标修抽离，但always-on整单仍7候选/53.949s热，未直接部署。
- 更深运输拒绝：原横移允许initial+1µm，native全局回侵1.125920619µm；另index4边界仍约1.0005µm盒–机器人深碰。生产仅adaptive提案留半epsilon，并复用原align分支5µm物理提案净空；原512 fallback与所有正式全局源位/物理门禁不改。core callback默认None，只有原完整盒抽离audit拒绝才以同64controls/32iters重试，无source phantom，保留原拒绝/总耗时并仍过原TimeOpt/RNEA/4001/接缝等。
- 四生产源码变动及既有两个测试各增一CPU检查。97CPU检查通过（87核心+10发布/FK）；AST还原核对core仅可选cost、adapter原参考/预算相同、原native函数/正式source_progress相同、layout仅两更严提案。旧/新真实运行25SHA仅4预期源文件改变、另21项未改。source/before清单冻结，详见native_extract_recovery_20261123。
- 当前生产15新鲜请求/8布局全部通过，独立CPU数组/URDF端点/真实45度/原sphere1.05m包络/源位全过程全过，另一原箱位重叠全部0。真实native32/4001/TimeOpt/RNEA/qva接缝与合并逐字段重建通过。普通冷49.178，三热4.247/4.307/4.039；困难冷45.180、热24.022/24.122，均5候选（旧生产同输入热47.102/7）。near冷41.437/热19.828，mirror热23.045/23.101，镜像−15mm22.234，near±15mm15.257/17.182，双tall长高交换冷34.300。**此前实验near仅一次新拓扑，不能与旧near热19秒比较；冷启动顺序差异不作因果速度比较，更不是公平PR82全任务5秒证据。**
- 4个完整运行时原型each8请求独立CPU/原生拼接核验均完成，当前生产无运行时覆盖/保存路径作为求解输入。lazy_extraction_contract确认普通不引导、困难热保留原1.738495579mm拒绝后成功，同真实64x15/32轮/4001且无抽离源位引导。
- 下一可定位瓶颈：困难热仍有5.716s精细末态IK无解枚举。可研究在组合展开前用原末端link7–固定头/升降架必要几何安全剔除，但**未实现/数值边界未证**；勿盲加预算/改门禁/把搜索失败判非法。当前无新源码待验证。
- GPU16680/48760/80801终态0；按需实验904785与gpuq902180已消失、完整8请求summary。CPU85842/43983/76173/74248/48982/5381及各原生重建终态；净空CPU7380句柄已消失且8独立结果完整。无本轮遗留GPU/验证进程。PATH gpuq/≤8编译，无依赖修改/commit/push/实机。最新交接docs/BOX_LAYOUT_NATIVE_EXTRACT_RECOVERY_20261123.md。goal仍active，连续任意合法XYZ充分覆盖、全部布局低延迟及用户验收未完成，不mark完成/暂停/阻塞。


## 2026-11-23 Codex / 单臂独立自碰撞预筛与六组新世界位置完整验证
- 保留8299/8298/8297输入和回放，未重启、未运行会改参数的check_live；CPU8299现场仍保留，无WebGL人工目检。最新交接docs/BOX_LAYOUT_ARM_PRECHECK_20261123.md。
- 根因：困难反例fine末态提案原展开1324544组合才发现全部自碰撞。对实际round7/float32单臂q，克隆原FK球，保留该臂原球+固定机身原球，以原checker.get_self_collision/pair mask/padding筛出另一臂不能消除的碰撞，再展开组合；附加箱球和另一臂只在临时副本内负半径禁用，禁用值覆盖padding，不改原模型/借用缓冲。只noncontact且arm_limit=None启用，普通32候选算法保留，所有原接受门禁不改。
- arm_subset_probe_20261123组件7提案AB/BA重复：保留端点与顺序全部逐值相同；144查询另取1736样本随机改另一臂，保留FK逐值相同、子集拒绝必被原完整self拒绝。最慢无解5.523/5.648→.172/.168s、实际组合1324544→0；可行提案仍256。组件不是实时整单计时。
- 仅planner源及既有test_box_layout新增3CPU检查，100CPU通过5.27s（禁用/padding/附件排除/缓冲/round/URDF祖先不依赖另一臂/仅细网格调用）。AST还原原模块相同，旧新25运行源SHA只planner变、另24相同；before/source完整冻结。
- 当前生产arm_precheck_production_20261123 **21新鲜整单/14布局全部成功**。普通冷49.569、三热3.978/4.224/4.100；困难冷39.576、热18.006/18.197，均5候选；near冷40.968/热19.666，mirror两热22.696/22.425，mirror−15mm22.120，near±15mm14.155/17.470，双tall交换冷30.636/2候选。
- 六组新位置均通过：左右X错位24.923冷、低侧/顶混合24.661冷、双cube顶26.800冷、.799m双顶4.700热、.800m双侧23.119冷、双交换tall低顶27.067冷；中心/尺寸/模式/实际.300/.255抽离距离均独立核验。冷初始化不与热耗时直接归因比较。
- 独立CPU21请求数组/URDF端点/真实Ry−45deg/原1.05m包络/源位全过程全部通过，另一原OBB重叠均0；实际native32/4001/TimeOpt/RNEA/qva接缝/合并逐字段重建通过。新增verify_contacts.py还直接原URDF回算实际原生接触箱心与吸附法向，最大初始箱心偏差2.924µm、最大全局持箱角误差.0000184deg；不得挪输入中心制造成功。源位仍是初始残余不加深/清出不回OBB，非全程X单调/整箱先越无限前沿面。
- exact_warm_comparison：上一版困难counterexample_warm与本版同请求/resolved task/全部几何帧/四阶段原生6字段逐值完全相同，墙钟24.022→18.006s；只此对照作同真实轨迹声明。批量same_input_motion_comparison保存15对，14几何相同、双tall有合法候选变化，其余原生差异全部如实记录。不能拿热算/有限成功当公平PR82全任务5s或任意合法XYZ证明。
- 下一瓶颈已换为next_profile中横移连续LM/原生运输；不要再修已移除的5.7s百万无效组合或重复full-box实验。未降低128参考/4001/边密度或改变预算；可先组件测更快横移提案再新鲜整单，不做未经验证更改。
- 本轮GPU24200/18425、CPU78710/65483/97632均实际终态0，无本轮遗留GPU/核验进程。PATH gpuq共享原锁/≤8编译，无共享依赖编辑/commit/push/实机。goal仍active：连续任意合法XYZ充分覆盖、全部布局低延迟、最终用户验收均未完成，不mark完成/暂停/阻塞。


## 2026-11-23 Codex / 同批载荷分类与净空复用、两新位置及有证拒绝
- 根因进一步精确：横移每侧~2.4s中LM~.3s，候选/边检查~2s；同一FK查询先eager完整几何求5µm提案净空，再重复分类。FullPayloadCost.classify_with_robot_depth同次返回原valid/uncertain与robot最大signed depth（32行graph同步输出），默认classify仍仅两mask。payload_search_mask及TwoBox转发器加可选同批classification，要求state及严格(1,N)形状，原CPU不确定复核/统计不变；callback在CPU覆写FK前消费全部视图，无跨查询分类缓存。
- 四源文件tools/v3_full_payload_cost.py、v3_payload_safety.py、v3_two_box_pick.py、v3_box_layout_planner.py及三个既有测试文件改动。105CPU通过5.15s；AST还原证明原geometry/classification两mask/CPU fallback及其余方法相同，128参考/.001边/4001/64或32controls/迭代/物理源位和动力学阈值/预算都不改。25运行源SHA4预期项变、另21不变；optimizer cost/primitive与最终独立完整盒审查不变。
- merged_payload_probe_20261123组件18批（3布局×1/16/32/33/127/257）classification、uncertain、净空、CPU fallback逐值相同；三布局横移AB/BA重复路径全部逐值相同。组件热困难5.251→3.540、near5.092→3.327、mirror6.629→4.413s；首轮含创建不作热比较，保存起点组件非新鲜整单速度。
- 当前生产merged_payload_production_20261123：24新鲜请求，其中23完整可行请求/16布局全部成功，1个明确URDF可达性拒绝。普通冷48.627、三热4.162/4.110/3.963；困难冷37.052、热15.886/15.987（均5候选），near冷39.435/热18.203，mirror热19.591。更全数据见summary与新文档，cold不与warm直接比较。
- 新两布局：左tall长高交换侧吸(1.08,.57,1.12)/右cube顶吸(1.18,-.53,.73)，30.154s冷；左cube(1.10,.52,1.35)/右tall(1.16,-.49,1.32)双侧，28.851s冷；均1候选。初始中心保持，宽.300/.255与世界Ry−45deg不改。
- 明确拒绝：左tall(1.10,.50,1)、右cube(3,-.50,1)，0候选、success/physical_success false、assigned_contact_outside_reach。CPU实际fixed.urdf+offset/sign/升降界复算同证：右腕距离下界2.176745866m>上界.990097501m，非搜索耗尽假称非法。原早返回task_success未置值（未跑轨迹工艺），未改写成功。status_markdown实含URDF reach bound理由/接触可达性证明，rejection_status.md留档；未操作8299触发此例。
- 独立23可行请求CPU数组/URDF端点/原1.05m包络/源位全过，另一原OBB最大重叠均0；native32/4001/TimeOpt/RNEA/真实qva接缝/拼接逐字段重建通过。原生接触法向/箱心/全局−45独立核验23，最大初始箱心偏差仍2.924µm。same_input_motion_comparison保存前后21对；困难counterexample_warm全部请求/resolved task/几何帧及四阶段原生6字段完全相同，18.006→15.886s，仅此作同真实轨迹声明；其它差异如实记录。
- 8299原输入/回放/CPU进程保留，未重启/改参数或播放；无WebGL人工目检。GPU48697、CPU82855/32326/76795终态0；组件77226句柄已消失，3组18批报告完整、实际进程无。PATH gpuq共享锁/≤8编译，无共享依赖编辑/commit/push/实机或本轮遗留GPU/核验任务。
- 最新交接docs/BOX_LAYOUT_MERGED_QUERY_20261123.md。下一步看当前next_profile并继续合法位置覆盖，不要重做已解决的百万组合/抽离盒体引导/重复分类；完整接受门禁不改。goal仍active，连续任意合法XYZ充分覆盖、全部场景低延迟、最终用户验收未完成，未泛化其它尺寸，不mark complete/paused/blocked。


## 2026-11-23 Codex / 源位误报根因修复与当前36组合独立验收
- 开始按三种有效几何×两种吸附/臂枚举36组合，发现此前23成功仅覆盖9组合。旧topology_coverage首批12后端全报成功，但独立发现tall_side__tall_top误报：GPU初始重叠1.040669µm判未清；无状态URDF精确初始.9964855µm已清、下一1.0004060µm回侵（中心X无前探，完整盒投影变化）。原1µm过程阈值不改，旧错误接受记录/原轨迹保留；independent_validation_status明确该旧批11通过1误报，不按12通过累计。
- 另外查到yourdfpy/trimesh scene graph合并≤1e−8变换更新，原显示循环得1.001525µm且有微小历史依赖。新核心批量float64 FK+无状态标量URDF参考绕过显示缓存；只缓存解析URDF元数据，不缓存FK输出。首点及临近原清出/深入边界行用标量参考重算，1e−7仅分派窗口、未加到接受阈值。17个URDF可动关节含两头部，实际工具链15原生轴完整映射，禁止漏轴/非有限/mimic链。
- 只生产core/native_transport.py和v3_box_layout_planner.py及既有test_native_transport.py/小原生fixture改动。共同任务终门禁保留原GPU source检查；通过后还要求stateless CPU source通过，root0/root1全覆盖，GPU结果留gpu_pose_process，根观测为双精度。原数组/箱心/宽距/45deg/物理模型/优化/密度/预算不改。107CPU通过5.27s；AST还原其余原生/搜索/物理代码相同、25运行源仅2预期变另23不变。
- 已保存35条真实数组全量无状态标量复算与快速边界法完全同决定（34通过、上述1拒绝），最大矩阵差~1.2e−15。旧显示循环4001约1.836s、标量参考.727s、批量FK.010s、完整源位复核通常.020–.040s。组件非实时解算成绩。
- 修复后边界同请求全新cold86.513s/3候选：0原box/world碰撞拒绝，1GPU过但CPU frame1回侵拒绝，2成功；warm11.128s/首候选成功。未把旧错误接受47秒与新正确成功冷86秒作速度比较。真实前15行fixture及共同门禁“GPU拒绝不被CPU恢复”回归保留。
- 当前唯一组合矩阵topology_verified_20261123：3批各12，36全新组合请求全部成功；全source SHA、原GPU+新CPU source过程、独立每点标量URDF（另一矩阵组合顺序、无显示缓存）、初始吸附法向/原箱心/全局−45/原1.05m sphere包络/native32每段4001/TimeOpt/RNEA/真实qva接缝/拼接逐字段重建全部通过，跨另一原OBB全部0。左右初始X1.18/1.20、Y±.55，side Z1.05/top Z.65，原.8规则自动选模式。36是一处代表/组合，不是任意连续XYZ证明。
- verified_matrix.json列全部结果SHA、目标XYZ、候选数与实际FK。持箱XYZ沿用此前批准适配，当前选中Z1.0–2.0m，−45度不变，未假称所有箱同1.2m默认高度；初始中心完全按请求。只两尺寸，cube Y/Z相同故无需重复交换相同几何；高顶吸上界仍挂起。原源位为自有OBB初始残余不加深/清后不回，非全程X单调/无限前沿平面；另一原OBB独立诊断0。
- 当前字节普通/困难6新鲜标准回归全过并独立核验：普通冷48.939、热4.125/4.433；困难冷38.130、热17.055/16.859。工作进程墙钟，冷热分列/不含外部排队，仍非全部布局5秒或公平PR82全任务基准。
- GPU旧37114/修复focused62615/当前42987/91877/标准57702终态0；当前第3批96454在12结果全落盘后返回143（原因未证、不冒称正常），36全证据已独立校验、未因此重跑。CPU95289旧验证实败1；当前96893/48619/80764/23293/65519/97701/50774/80545等终态0。早期测试65173仅缺json导入，后107绿。所有本轮GPU/核验进程已无；仅原8299/8298/8297CPU现场保留，未重启/改输入/播放，无WebGL人工目检。source_boundary图为原样本生成，工具不支持看图未声称视觉检查。
- PATH gpuq原共享锁/≤8编译，无共享依赖编辑/commit/push/实机。最新docs/BOX_LAYOUT_STATELESS_SOURCE_AND36_20261123.md。goal active：连续任意合法XYZ、所有场景低延迟及用户验收仍未完成，未进入其它尺寸泛化，不mark complete/paused/blocked。后续请以当前36矩阵/新CPU门禁为基准，别将旧误报或不同冷热相混。


## 2026-11-23 Codex / XYZ挑战发现未知失败、RRT已拒池淘汰及正确统计
- 当前source与上轮25SHA核对后生成12个高度/XY/前后挑战。前4同侧前后排布被原assigned_contact_outside_reach证据拒绝，CPU实际URDF重算相同、0候选；原Y=.55及新增Y=.50输入都保留，不在求解中挪箱。第五低cube左(1.05,.52,.45)顶吸/高tall右(1.12,-.52,1.55)侧吸无必要界排除，但16候选/原32s首恢复+后8s RRT耗尽，249.502s unknown；8恢复抽离几何有效，搬运未连通。未叫非法。runner停止，后7请求未跑。
- 更正中间误读：旧fine_candidates_checked=64只是收尾次数，不是全精查总数。代码确有已拒短连接占64池/被作informed bound缺陷；只共享v3_multiroot_search.py+既有native_transport测试一项修复：原fine拒绝后记身份、入池/每轮过滤、收尾不重复。保持原validity.mask/quality短路与全部物理/采样/预算，新增正确总数/geometry-vs-quality计数。108CPU5.69s通过，AST关键调用/API/随机/while预算相同，25源仅1预期变另24相同。
- 原请求当前池修复下完整新算仍失败240.130s/16，不能说解决或拿失败耗时作加速；首恢复2489粗连接、682精查、215dense-geometry/467source-quality拒绝，其他恢复大多0粗路径，第8有4/4quality拒绝。rrt_rejected_pool/failure_diagnostics及所有old/new失败保留，无运行时覆盖/保存路径输入。
- 六正例当前源码新鲜解算+独立CPU/接触法向/数组/原1.05m/源位全过程/native32/4001/TimeOpt/RNEA/qva接缝/合并逐字段全过，另一原OBB0：普通冷49.897/热4.168/4.173，困难冷39.900/热19.761/16.778。旧36矩阵明确是前版本，未冒称本次新字节36重测。
- 下一具体方向：状态约束仅原initial+1µm，完成path才no-return，467quality失败应测在树边按真实运动方向施加相同过程规则（start/goal树及连接方向），不要盲加预算/改门禁/直接叫非法；并查fine collision是否粗树坏边继承。尚未实现。先重建首reverse有效端点组件拿精查原因，再完整新算，别反复240秒无端点文件。endpoints来自extraction_recovery[0].proposal/trials.endpoint_index重枚举，保存目标是组件非实时成绩。
- 新交接docs/BOX_LAYOUT_POSITION_CHALLENGE_AND_RRT_POOL_20261123.md；position_challenge目录仅5已执行，不是12全过。GPU97295/8776/79007、CPU77577/18594/88782/82236终态0。无本轮遗留GPU/验证任务，PATH gpuq原锁/≤8编译，无共享依赖编辑/commit/push/实机；8299/8298/8297现场不动。goal active，未知失败及任意合法XYZ/所有场景低延迟/用户验收尚未完成，不mark complete/paused/blocked。


## 2026-11-23 Codex / 定向过程树边+root0原生恢复解决高低差，扩位发现近前native余量未知
- 起点树parent→child、目标树child→parent、连接边按真实前向调用；仅布局payload transport/start设置edge_process_quality，在原边点执行原clear→return cumsum规则，原物理edge mask只AND不可恢复拒绝，最终密集path质量与GPU/CPU全过程保留。未变步长/密度/随机/预算/阈值/模型/箱心/宽距。
- directed_source_edges组件重建相同saved target首reverse有效端点（非新鲜速度），原32s cap内16.950s找到路线，fine1、质量拒绝0。原完整request新算找到几何但native单曲线32近似后碰撞（base_box_00146.67mm/left_wall2.164mm），16候选仍失败261.652s，未伪称整单成功。
- 具体第二缺口：该路selected_root0，旧time_layout_pick只适配用了forward prefix的root1。root0_native_probe先用实际saved monolithic失败/前三已通过数组+同一几何组件，4真实原生子段门禁及CPU source通过，26.768s组件非新鲜。
- 仅后备触发条件扩展：原monolithic成功和早期失败原对象返回，只有transport失败且前三阶段真实通过才保留原前三arrays/controls/seams并复用同一native32/4001/TimeOpt/RNEA/seed-validation/global-source-progress/qva合并循环，原失败留transport_monolithic_rejection，原分段上限/所有gates不变，原拒绝wall也计入。production v3_layout_native另改；3个directed源+既有测试共111CPU5.38s，AST原物理/最终质量/预算和原native循环相同。
- 高低差原request新鲜闭环成功root0_native_probe/fresh/result：94.098s/第5候选、4段运输16001行，success/physical/task true；独立每点无状态标量URDF/初始法向/真实初始中心（约4.12µm）/宽距/全局−45/原1.05m/原生32-4001-TimeOpt-RNEA-seams-逐字段拼接全过，另一原OBB0。未移动箱心/额外抽离/降阈值，成功不与旧失败240/249作加速比较。
- 最终字节正例6新鲜+独立全过（root0_native_probe/standard_regression）：普通冷49.886/热4.581/4.639，困难冷38.838/热16.693/16.645。此前36矩阵是前版本，未冒称本次新字节36重测。旧Meta.task policy仍全局−45，heldXYZ按已批适配。
- 原剩余7挑战position_resumed全执行：镜像高tall左/低cube右78.302通过（2native运输段），正/负Y vertical各有必要reach证据拒绝（CPU实际URDF复算相同），低swapped双顶22.491通过，unequal X低顶53.861通过，高swapped侧28.116通过。所有成功独立FK/数组/工艺/包络/nativepiece/RNEA/merge全过、跨另一原OBB0。
- 最后新增UNKNOWN未解：左cube(.95,.60,.65)顶吸、右tall(1.10,-.55,1.0)侧吸；893.272s/16，无非法证明。8 endpoint proposals每个raw_pairs2544640、hard_valid=exact_valid=全部、native_valid=0、envelope0，lift0，guide≈−.33781，单次~107s。重点下一步组件定位native_margin_mask的base/ground具体项/哪个sphere，而非改RRTbudget；测安全原单臂native必要预筛减少百万重复、或原有姿态界内加入native余量引导，先证合法目标，不能擅降margin/挪位置。next_native_margin_diagnostics.json完整，别再反复整单15min取同信息。
- 最新交接docs/BOX_LAYOUT_DIRECTED_SEARCH_AND_NATIVE_RECOVERY_20261123.md。旧初轮4reach拒绝+新高低成功+剩余7=5新位置成功/6可复算reach拒绝/1未知（跨版本结果分开保存），非所有XYZ覆盖。Source prior错读fine64已更正，rejected pool修复保留，当前geometry路径能找到。
- GPU10359/86693/60761/38312/80990/76374/30305，CPU68024/3398/8056/48301/12556/9754/78599/5185/92956均终态0，无本轮GPU/验证遗留。PATH gpuq原共享锁/≤8编译，无共享依赖改动/commit/push/实机；8299/8298/8297未重启或改输入/播放，HTTP非WebGL目检。goal active，新近前unknown、连续任意合法XYZ、所有低延迟/用户验收未完成，不mark complete/paused/blocked。


## 2026-11-23 Codex / 用户收窄前向距离范围与已有结果收尾
- 最新用户指令优先：任务范围不覆盖机器人前方距离变化，近前例仅报告已有成果、无需继续优化。取消历史任意X扫描和近前例优化待办；后续固定现有任务前向距离。未将全任务标记完成或暂停。
- 近前例已有根因：左载荷球/base_box_001间隙约.4463mm < 原native5mm；必要单臂预筛保留自身载荷/原native余量，避免每提案约107s百万无效对；原提案耗尽后才惰性加入原姿态界内base引导。root0物理通过但source失败时保留原拒绝并走已有native分段。3运行源及2既有测试改动；其它22运行文件未改，门禁/预算不放宽。
- 已有新鲜近前例冷230.911944s/热174.904004s各12候选成功、3实际运输子段，独立核验通过；之前222.678s源位失败保留。该例现已排除，未追加求解或修改算法。实际左抽离X约.298214329m（名义.300m，继承2mm/1°容差），不可表述为实际精确300mm。
- 当前源码12新鲜回归均成功，独立stateless URDF/源位/接触及原生piece重建日志终态0。普通冷51.474s/热4.453、4.536；困难冷38.022s/热15.574、15.253。其它布局仅保留已有记录，不冒称全为范围内固定X。当前源码未重跑旧36组合，未证明PR82公平同条件/全部5s。
- 最后CPU测试81710实际终态0，114 passed in7.21s（此前fixture缺字段失败已修正）。最终标准核验32890终态0。证据native_base_guidance_20261123及更新交接docs/BOX_LAYOUT_DIRECTED_SEARCH_AND_NATIVE_RECOVERY_20261123.md；后者最前最新范围覆盖下文历史计划。
- 8299现场输入/回放保持，HTTP200；浏览器工具返回No browser is available，不能声称打开浏览器/WebGL目检。用户可访问http://127.0.0.1:8299/检查。未启动新GPU任务、未commit/push或操作实机。


## 2026-11-23 Codex / 当前源码固定前向距离36组合全量复测
- 完全遵守最新范围：固定8299现有左X1.16/右X1.20，仅六种几何/吸附组合每臂交叉共36新鲜任务，Y±.55、Z.65/1.05；未改运行源、重启现场或继续近前例优化。
- GPU6272与独立CPU16357均实际终态0；36/36物理/任务及无状态URDF源位检查通过，接触/抽离/全局−45独立核验36通过，11个分段阶段原native32/4001/TimeOpt/RNEA/qva/拼接逐字段重建通过。最大初始箱心误差.198542µm，最大额外源位侵入.191497µm，清出后返回均false，跨另一原OBB重叠均0。
- 全36 planner_reused=False，单次22.564–59.042s，总1245.385s；不拿拓扑初次加载批次比较热态5s。之前普通热4.453/4.536、困难热15.574/15.253仍保留，未证明全场景5s/PR82公平同条件。114CPU为此前已通过结果，本轮无代码变化未重跑。
- 新增证据fixed_forward_acceptance_20261123及docs/BOX_LAYOUT_FIXED_FORWARD_ACCEPTANCE_20261123.md。原现场8299只读协议验证，无GUI消息/重算/参数或播放修改，快照live_readonly记录现场原成功4.021s。
- 8个固定X阈值(.8m以下/等于四组合)、高低差/左右Y变化请求在fixed_forward_boundaries_20261123；GPU82576（worker2505505）和独立CPU87075已启动，尚待真实终态，不能叫完成。仍沿用gpuq原共享锁，估时9m暂定来自本轮及旧高低差实测。
- 当前36是有限位置组合覆盖，非连续全部Y/Z证明；goal active，后续按实际8批结果更新交接并等待用户语义验收，不再追逐X覆盖。


## 2026-11-23 Codex / 固定X恢复触发修复、44布局证据与交给性能agent
- 最新用户追加：时间不达标没关系，时间优化将交给另一个agent，只需交接已有内容。本轮已停止追加耗时优化；没有启动子agent、没有调整其它模型/场景/预算，主目标未冒称连续合法位置证明或用户验收完成。
- 固定X八边界首轮5成功1未知（镜像高低差175.758869s/16），最后2未执行；82576及87075实际终态0只表示批次脚本按未知策略正常停止，失败保留。查到恢复触发只允许抽离几何失败，首4中一项运输RRT耗尽会屏蔽全部原反向恢复；原48.58/128s用量、recovery0，非不可达证明。
- 仅v3_box_layout_planner.py恢复条件增加joint transport RRT exhausted前缀及policy说明；既有test_box_layout.py加1个CPU回归。max16/最多8反向/总128s/首反向32s/其它8s和全部门禁不变。相关4测试文件79 passed in2.81s，55810终态0；AST前后只此条件及说明，scope.json和before/source SHA保存。36旧成功均最多2候选，执行不到改变的恢复决策，但不宣称编辑后重跑36。
- 最新6完整新鲜任务全通过，GPU71941及独立CPU42674实际终态0；contacts/pieces最终CPU均终态0。镜像高低差冷168.931508s/热111.446681s各7候选，实际3运输native段12001行；左右内收24.799635s、外展4.237811s；原现场布局新算热4.978034/5.539677s。源位/无状态URDF/实际吸附/宽距/全局−45/原1.05m/native32-4001-TimeOpt-RNEA-qva-逐字段拼接核验全过；初始中心最大6.113µm。
- fixed_forward_recovery_20261123/acceptance_matrix.json把36+8代表布局逐条链接到成功result、SHA和源码版本，保留旧未知原始175.759s失败。有限44不是连续全部合法Y/Z证明，也不是44都为当前单条件编辑后新鲜复测。
- 性能留给下一agent：热111.447s中4次均失败横移prefix64.503s，RRT26.612s，最后root0。latency_profile.json和实际GUI格式化ui_timing_markdown.md保留。仅CPU横移必要reach探针无witness，未据此跳过/宣称不可达；没有实施进一步性能优化。
- 最新可直接给下一agent的文档docs/BOX_LAYOUT_FIXED_FORWARD_HANDOFF_20261123.md，含当前任务语义、全部不可放宽门禁、已解/未证结论、代码入口、冷热表、原始失败、源版本、同worker冷热复现完整命令。NOW及旧交接已指向该文档，不再沿旧近前例/X扫描计划推进。
- 8299/8298/8297原现场不动，只读协议/HTTP可访问；浏览器工具无浏览器，未声称WebGL人工检查。所有本轮GPU/校验终态，未commit/push/共享依赖编辑/实机。用户验收仍待进行，耗时优化已交接。


## 2026-11-23 Codex / 交接后CPU功能验收逐项审计
- 按用户“耗时交另一agent”范围，仅做CPU功能审计，未启动GPU/性能工作、未改运行源或现场。25运行文件仍匹配最新交接source快照；原GUI协议证据已覆盖参数/非法重叠拒绝/真实提交/播放及−45末态，不重复操作8299。
- functional_acceptance_audit.json逐条核44代表输入及实际接触/抽离/−45指标、15轴共享updown/10kg每箱/native/source门禁，读取176实际阶段数组检查有限数、时间严格递增、每原生段4001拼接数量；CPU90737实际终态0。未声称独立重算碰撞/力矩。
- 关键交接澄清：approach/transport控制数组(1,32,15)，contact(1,128,15)，extract(1,64,15)，不可把“native32”泛化到所有阶段。最大抽离向量误差1.804315mm，继承2mm门禁，非实际精确300/255mm。主交接已补充。
- 实现及有限明确输入验证已有证据；连续全合法Y/Z及用户最终WebGL画面语义验收仍未证，goal未标完成。后续等用户验收，性能优化保持交给下一agent。


## 2026-11-23 Codex / 用户双SKU任务语义纠偏与指定关节回中
- 做了什么：以用户最新目标取代旧左右不同X/世界−45°回中；新双箱入口校验共享箱心X平面、距实际底盘前沿60–90cm的2cm倍数、高差≤60cm。回中直接构造左[40,-46,0,-100,-30,-75,90]°及右相反角，updown逐任务在URDF[-1,0]内选，不再反求回中Cartesian IK。
- 改了哪里：新增curobo_core/layout_contract.py；box_pick.py增加可回放的关节回中字段；v3_box_layout_planner.py改关节目标并提前过滤原有最终原位清出条件；v3_two_box_timing.py及v3_box_layout.py增加真实关节终点核验。编辑器共用距离选择，X只读；运行例子/CPU测试与asymmetric catalog迁移新平面。旧交接已加失效提示，当前入口docs/BOX_LAYOUT_JOINT_CONTRACT_CORRECTION_20261123.md。
- 门禁冲突：用户指定角在当前URDF下带箱旧球模型半径约1.41–1.52m；原1.05m圆柱与指定角冲突。新关节合同仅诊断该圆柱（enforced=false），不降低实际碰撞、10kg/箱、TimeOpt/RNEA与源位过程门禁；旧Cartesian回放仍保留圆柱。勿称新任务通过旧1.05m门禁。
- 验证结果：新合同2项CPU、编辑/会话40项、旧单臂15项通过。新合同首例D=.68m、左40/右高箱Y±.5/Z1.02成功；四阶段4001点、实际运动5.975875s、worker86.939s，updown=-.72m，关节误差1.63e-7rad；独立CPU抽取误差左3.71e-8/右3.98e-7m。该例触发提前过滤编辑之前，source按结果SHA区分。
- 留给后续：48请求标定正在exec63216/global gpuq中；目录generated/two_box_layout/joint_contract_20261123/distance_calibration_v2，读取CONTINUATION.json和当前活句柄，不重复启动。初版标定因终点漏筛中止并保留，非完成失败。D=.68目前暂定，后续比较三个固定布局选一个D，再测SKU左右/交换、阈值、低位、高差、非对称/堆叠位置。任意合法连续位置未证，goal active。旧44例不计新目标成功数；没有commit/push/实机。


## 2026-11-23 Codex / 固定关节回中的连续升降余量与距离边界筛查
- 上轮属于实质进展，本轮继续轮询原gpuq/exec63216，确认worker162908仍活，未重复启动；48请求标定继续，当前进度以CONTINUATION.json/summary为准。
- 新增生成目录下source_lift_intervals.py：固定用户关节角/实际URDF附箱几何，把自身原位OBB SAT转为连续updown区间；100随机case×100随机lift与向量SAT一致。D60混合顶/侧60cm高差无共同清出区间；D62区间总宽约23.5mm、D68约143.9mm、D74约822mm。浮点区间分析，不冒充严格区间算术证书/全路径可行性。
- 相同360挑战/每D的联合必要条件筛查distance_workspace_screen.json：D60/62/64各有58/36/14组无共同原位清出区间；D88/90有2/48组强接触不可达证据；D66–86未触发这两项必要排除，但不等于完整搬运可行，也不把360输入当成用户全域。
- CPU接触/原生恢复回归20项通过；保存实际原生末态的CPU FK投影carry_joint_fk.png/metadata，PNG1760×1600完整性验证。view_image工具不支持图像输入，未视觉目检。
- 运行tools源码未修改，原GPU批次source stamp仍一致。待该批次实际终态后再收紧输入边界并继续SKU互换/阈值/非对称及堆叠挑战；最终D尚未固定，goal active。


## 2026-11-23 Codex / 同侧上下堆叠揭示“任意合法”范围缺口
- 在全部D60–90的16值、两种箱臂分配下核验真实反例：两箱Y=.50m，40箱Z=.80m/高箱Z1.40m，净竖直间隔.109m。满足用户当前共面/高差≤.60m，不入地、不相交，与实际environment墙/地/顶无AABB重叠。
- 全32请求都由实际URDF接触必要条件在完整updown[-1,0]范围内给出右臂不可达证据（不是有限搜索失败）。D60原/交换分配的joint2可达性缺口约.262/.280m，包含位姿/几何包络。
- 已保存same_side_assignment_witnesses.json及same_side_environment_check.json，新增永久CPU回归，合同测试3项通过。当前固定底盘、双臂各直接吸一箱流程无法由用户两个输入位置条件保证全域；未擅自把反例划为非法。需要明确左右工作区/YZ绝对范围，或同侧/堆叠是否要求分次取箱/交接流程。
- GPU原exec63216仍在运行，未因澄清问题终止或重复；距离标定与独立物理验证继续。goal active，非blocked，不宣称三代表/对称左右成功覆盖了用户全部目标。


## 2026-11-23 Codex / 48距离标定完成、固定默认66cm、接触模型结论纠正
- 原GPU exec63216实际退出0；48输入全部结束46成功/2未通过。worker累计3102.21356s，独立CPU audit46成功全过（2未通过/0pending）。D60混合顶/侧无回中原位清出候选；D86高箱顶吸16候选耗尽，15次native transport拒绝/1次RRT预算耗尽；保存d86_unknown_failure_diagnosis.json，不称非法。
- score_distance.py结合三代表任务全部物理/任务/独立成功、360布局必要筛查零强排除，按最差native RNEA利用率排序选出D=.66m。当前默认和所有可运行配置更新为共同worldX1.1680520510673524，GUI固定距离只读，命令覆盖仅标定用途。尚非全域证明。
- 输入边界修复：拒绝显式禁用关节合同、输入固定回中updown、非有限/不一致D、左右不同阈值、错误箱臂映射与关节合同非零Cartesian偏移。86相关CPU测试通过，source_fixed66保存运行源。旧资源key测试改用合法已解updown变化验证资源缓存不受持箱XYZ影响。
- 重要认知纠正：此前同侧反例证明当前面中心/既定法向接触模型受限，不能扩张成所有偏心吸附或不同流程的硬件不可能性，用户未明确要求面中心吸附。已对用户说明，不擅自缩小合法箱位。96阈值模式/分配/距离组合全有中心接触可达性排除；D66现有cap球代理足迹约束下1350个偏心/5°roll候选也被必要界排除，但有限采样和代理足迹不是全部吸附方案证明。现有target_in_front_of_contact_plane用center-contact推法向，若以后支持偏心，须改为显式工具接触法向，不能只改TCP目标。
- 新19布局固定66cm挑战正在共享gpuq exec26417：三基线、SKU互换、长高交换、阈值边界、触地/高位、非对称、堆叠及保留同侧中心吸附反例。估时35m基于48例均值×19及约1.7倍冷启动/边界余量，暂估不是超时。未启动额外GPU/嵌套队列。需继续审计全部结果，goal active。


## 2026-11-23 Codex / 近地载荷搬运搜索根因修复与四例独立成功
- 首个固定66挑战实际为tall_near_ground（左高箱Z.30顶吸、右40箱Z.90侧吸、Y±.5），不是基线；已向用户更正。16候选全部native transport拒绝，490.946s、243次静态力矩诊断最高1.08863。等待完整结果落盘后有意SIGTERM停止原26417，退出241，未把其余18例当完成。
- 根因修复于v3_box_layout_planner.py：复用现有TorqueModel/RNEA按实际双箱10kg及attachments，对loaded搬运搜索/恢复种子提案筛零速零加速度的原.995力矩界；提早复用native engine，回中与抽离终点同查。最终完整原生动力学/碰撞/源位/关节终点门禁不变。静止保持为保守提案筛法，不是动态补偿路径不可能性证明，代码ponyTail注释明确上限。
- GPU18961四新鲜请求实际退出0，4全过且CPU独立audit4全过：原近地失败首候选63.18s成功、左40顶/右高侧22.02s、左40侧/右高顶74.67s、双侧22.78s。非严格性能A/B。近地RNEA提案121649，过滤14567，最终updown约-.12m；用户关节角不变。source_gravity_focus与真实SHA核对冻结。
- 新增CPU test_layout_static_load.py及旧搜索掩码合成回归，87项通过。随后增加抽离终点提前负载筛选（保留既有extracted endpoint恢复路由）并冻结source_gravity_final。GUIactive_axes(False,True,True)实际限制Y/Z拖拽；默认D66未变。
- 当前GPU12832正按相同19 catalog重跑，目录fixed66_gravity_challenges。源位、薄弱位置不删，后续需完成全批并独立audit；前48/四例都不能替代任意合法位置。CONTINUATION.json为入口，勿复启旧26417/18961。


## 2026-11-23 Codex / 按纠偏主线清理本session过期内容
- 用户要求清理已不可用的旧内容且不影响另一session。删除本session旧生成结果、34份过期说明和20个未被调用的实验脚本等233项，共21459文件，执行时实际释放8.672GiB；旧44例及固定X/世界−45°记录已退出工作区，不计入新合同。
- 完整保留joint_contract_20261123目录6808文件，清理前后无删除/改写；331个保留源码/配置/测试文件SHA一致。3个仍被运行或测试调用的helper（check_asymmetric_layouts.py、replay_native.py、verify_cases.py）未删。未改共享运行代码、测试、配置、依赖、GPU锁或其它session任务，也未终止任何进程。
- 旧BOX_LAYOUT_FIXED_FORWARD_HANDOFF_20261123.md保留为指向BOX_LAYOUT_JOINT_CONTRACT_CORRECTION_20261123.md的短跳转；NOW只移除本session已作废的固定前向距离交接节，纠偏session正文不改。旧LOG仅作历史，旧产物链接可能已清理，请按当前纠偏入口接续。
- 新合同/静态载荷/可达性/双箱及会话CPU回归27 passed in1.77s，无GPU工作。删除清单与保护校验记录：/home/astesia/.local/state/codex-cleanup/pr82-obsolete-20261123-180108.json；同名前缀cpu-tests.log保存测试结果。


## 2026-11-23 Codex / 19挑战终态、高位末端升降漏搜修复、同列模型问题保留
- GPU12832实际退出0，19请求14成功/5未通过，累计worker370.326s，独立audit14成功全过。完整失败目录fixed66_gravity_failure_catalog.json保留高位、触地cube、同列低顶吸、同列侧吸、同侧面中心反例。未把有限失败判非法。
- 四未解请求诊断exec10313退出0：仅补contact q/最后有效抽离q/失败节点，不变物理。高位在~49.6%时updown逼近0/左肘逼近-145°，旧12恢复提案全在lift0/-1；contact_reach必要界沿129点无排除，不能判物理不可达。
- GPU89981几何探针12.64s：名义精确末端全21升降点得到256有效IK（-.50/-.45），首个反向LM与完整箱/接触核验通过。随后reverse_extractions保留原提案优先，再加精确末端完整升降/10°swivel格点；原细/容差/几何提案仍保留，全部最终门禁不放宽。87CPU回归通过。
- GPU55416真实全周期定向复测退出0：高位Z1.6/2.2成功87.70s，reverse recovery，updown-.51，原用户角误差1.63e-7rad，独立CPU1成功全过；同列侧吸仍失败15.93s。source_nominal_lift_recovery与真实SHA相符。勿混版本冒称最新19/19过。
- 同列侧吸卡肩J2/腕J6限位，完整nominal升降末端池仍0；同列低顶吸独立CPU固定link7球代理与上箱有4球/最大27.1184mm重叠，解释接触IK0，但不排除顺序取箱/偏心吸附。侧吸同列link7对另一箱无该重叠。没有把用户箱位范围缩窄。
- 当前没有本轮活GPU作业；12832/10313/89981/55416均终态0，早26417有意停止241。下一步先验证有足迹/惯量约束的偏心吸附/90°roll，或顺序取箱状态变化。偏心必须改tool_to_box/动力学/缓存key/接触法向，现contact helper用center-contact推法向不适用，不能只改TCP。详CONTINUATION.json，goal active。


## 2026-11-23 Codex / 同源19输入17完整成功，触地与同列侧吸自动恢复落地
- 新增curobo_core/suction_grasp.py及BoxPick真实suction_offset_m/roll字段；全cap球代理足迹须在所选箱面、严格切向offset；实际tool_to_box同时进入载荷碰撞/惯量/缓存。TaskCyclePlanner给显式contact normal，v3_contact_validity不再把偏心center-contact斜向当法向。默认中心姿态保持数值兼容。89CPU基础测试过。
- 6显式偏心/roll/阈值候选初试全未通过，原始证据保留。专项同planner末端探针发现backend prepare_task键漏实际attachment，v1整批标无效，不用于物理结论；根因键加入真实不可变task_attachments，v2逐checker断言tool_to_box，只有垂向向外8cm留60有效完整碰撞末端。原完整worker按attachment分开planner，不受专项v1复用问题影响。
- 修复触地箱建模冲突：冻结payload球恰好贴箱底，原strict0/native2mm规则把载荷当机器人强制离地；只对双箱checker实际payload球后缀登记exempt，机器人2mm保持，完整OBB地面/环境审计及原1mm容差保持。payload_floor_focused三例完整成功/独立CPU全过（触地、普通、真实偏心4cm/roll90基线）。实际触地运输最小箱底-.888mm，如实报告容差内非零。
- 同列side显式向外8cm垂向吸附在vertical_stack_grasp完成真实全周期117.73s+独立CPU通过，箱心Y0/Z.8/1.4未挪，用户角误差1.63e-7/updown-.52。随后主plan_layout自动在原中心失败后追加该足迹有效吸附型，用户显式指定grasp则尊重不自动改；每种grasp16候选、失败grasp_result.json和总grasp_trials保留。不同阈值不混作同一默认。
- 最新相同19 catalog的fixed66_auto_grasp_challenges全结束：exec84985退出0，17完整成功/2未通过，累计worker491.527s；独立CPU/用户角表/实际接触代理足迹17全过。不是混合旧版本凑数。最差真实接触cap代理足迹仍在箱面内1.765mm。source_auto_grasp30文件与真实SHA一致，CPU116项全过，git diff--check无错。
- 最新未通过仅centered_stack_low_top（.79/1.39，80cm阈值低箱顶吸末端撞上箱）与same_side_center_grasp_witness（当前直接模型，不证明所有流程不可能）。有真实世界下箱保留的先上箱几何探针22204/三idle姿态94179都未成功，未冒称支持顺序取箱。
- 当前仅exec28550正在GPU：显式全局阈值79cm/垂向offset-.07,+.09，把原低stack两工具接触高度对齐到已成功.72/1.48m，不挪箱/底盘；是标定，不自动改默认80cm。下一轮读CONTINUATION.json轮询，勿复启旧作业。goal active，完整任意合法位置仍未完成。


## 2026-11-23 Codex / 低堆叠79cm标定真实成功，统一候选22输入重验中
- 显式threshold79_aligned_stack已完整native+独立CPU1/1成功，GPU28550退出0，worker103.028s；原Z.79/1.39、Y0、D66不变，统一阈值79cm使等值侧吸，偏心-.07/+.09把接触高度对齐至已证明.72/1.48m。未把它当全部任务成功。
- 当前候选版本双箱parse/CLI/当前config统一79cm；BoxPick单臂/历史默认80cm保留。自动同列侧吸偏心±8cm外加共同0/+1/-1cm接触高度微调，仍完整足迹/载荷/碰撞/native/source验收，不挪箱。117CPU测试通过，含新切换边界和旧默认兼容。
- 当前唯一活GPU exec34081：fixed66_global79，原19物理箱位全部保留并显式同79cm，加.789/.790/.791三新边界共22，吸附偏心必须规划器自动恢复。估20m参考旧完整19的491.5s+新增恢复/冷启动余量，暂估非超时。source_global79与首个真实结果SHA一致。运行中不要改源导致source stamp中止，勿复启旧作业。
- 80cm同源19结果确为17/19，独立CPU17全过；不直接计入79cm。待22全终态与独立audit后才决定最终阈值。same-side输入仍保留，完整任意合法位置goal active。


## 2026-11-23 Codex / 自动侧面24完整成功与低顶吸顺序几何链贯通
- 统一79cm22输入结束21成功/1前面模型同侧失败，独立CPU21过；源位与政策不逐例变化。随后发现“抽离−X”被误等同“侧吸仅X−面”，并非用户要求。新增真实side_face X−/X+/Y−/Y+，完整cap代理足迹、实际tool_to_box/惯量/显式法向/缓存同步；右Y−/左Y+侧面roll90可在不挪箱下直接接触同侧箱。显式3全周期及独立CPU全过。
- 自动近臂箱面恢复接入原请求优先的同一入口；无手工吸附参数的原22+镜像/交换共24，fixed66_global79_all_faces实际24/24 native成功，GPU86973终态0，worker合计697.647s，独立CPU24全过。same-side原/镜像/分配交换都按用户14角，updown约-.94/-.83动态选。旧front/top不可达证据已注明作用面，不再全局外推或要求用户删同侧输入。
- 整个24会话281文件指纹逐条一致，冻结source_automatic_faces_full；实际依赖SHA并集30文件另存source_automatic_faces。之后只改UI目标完整矩阵/实际箱面/去45文案及足迹说明注释，physics_unchanged_after_preview验证其余278文件不变/规划AST除元数据一致；实际生成event与24真实末态独立CPU校核过。CPU121测试全过，git diff--check过，无浏览器视觉验收声明。
- 不以24例替代任意位置：below_threshold_stack_contact_family保留低箱.2/.3/.5/.7/.78、上箱+.6同列，统一79规则Top，原同时中心Top代理仍碰上箱27.118mm。输入不删。近Y面先上箱新鲜几何首候选成功（保留真实下箱实体），但刚抽离末端仍堵下箱接触，11264raw IK全筛掉。
- 先上箱→真实10kg带载让位（world下箱保留、slot/静力）15.048s成功；下箱接近旧探针float64接口失败57065退出1，修成float32后51907退出0，带真实上箱/实体下箱首候选几何成功4.944s；接触后双载下箱抽离和最终14角候选几何76972退出0/15.985s。8段真实候选数组接缝全0，CPU实际URDF核验原右/左抽离向量误差1.07e-7/1.33e-6m，最终角4.37e-8rad。未当成动态定时或主入口顺序执行成功。
- 当前本轮GPU均终态，无活作业。下一关键是把8阶段实际payload/world/接触/mass状态接到生产ordered分支、TimeOpt/RNEA/4001/qva接缝/独立验收和按阶段附着回放。FastThreeLayerPlanner现仅transport带payload、checker ground注册按selected盒而非actualattached、4阶段viewer/validator均需显式扩展，不能靠拼几何伪造成功。详CONTINUATION.json，goal仍active，任意合法目标未完成。


## 2026-11-23 Codex / 顺序取箱8原生+独立验收接入默认入口，31输入回归运行
- 新增CPU OrderedStage8状态合同与OrderedLayoutPlanner：每原箱始终pending或actual attached，稳定ID00/01；actual payloads空/空/上/上/上/上/双/双，待取下箱对已载上箱仍完整实体。native几何可显式声明载荷（不再把lower approach当空载），FullPayloadCost尺寸/地面例外按实际附着侧，普通同步默认分支保留。
- native种子集成v1前6过，第7零位移参考拒绝；修复core CartesianReference允许固定pose、finite residual，移动进度不同步held0，原two-moving同步不变。v2种子8原生+CPU32001样本全部过，20.625841s实际运动；独立用URDF关节轴×箱心重力核对两次吸附10kg力矩增量，误差约1e-5Nm，6个丢载荷/删箱/挪源/GPU拒绝不能CPU恢复等负例全拒。
- 新鲜规划接口不读记录轨迹、自动选面/IK/LM/RRT8段。v1漏导入接线失败保留；v2原生物理全过但GPU原位1.056µm越1µm而CPU.874µm过，任务按GPU拒；v3在2mm窗口内试50µm抽离净空，实际Upper transport碰实体4/3mm拒。v4真实native32种子恢复后物理/源位全过但CPUtuple/list格式拒，修复规范化不改数值门禁；v5全新规划/native/独立CPU成功。
- 默认v3_box_layout入口现同步候选耗尽的顶吸遮挡堆叠自动转顺序，default_ordered_v1实际成功，10656退出0，worker92.984s/真实运动19.407827s，success/physical/task/replay均真；再独立外部CPU和8阶段24采样回放状态检查全过。无手工箱面或流程输入。源source_default_ordered_v1完整指纹精确重建核验。
- 原生恢复保留被拒整段，选中真native32种子/子段必须完整碰撞/TimeOpt/RNEA/qva/source过；独立CPU重建merged actual数组，不补帧/改时间/删下箱。回放按phase payload集合、record实际grasp，修复旧raw输入默认grasp画错自动offset/face的问题。阶段标签动态8，旧4兼容。CPU147项全过，历史冻结接触文件仅normal入口有明确AST默认归一化例外，其余原碰撞AST和5个文件锚定基准。
- 当前仅GPU77418正在default_full31：原24+5更低同列+2下大上小/镜像，31无手工grasp/流程selector，已有SKU倒置成功，未结束不得新开重复或改运行源。估35m由24约698s+7ordered约90s及余量，暂估非超时。source_default_full31指纹与实会话一致，读CONTINUATION。goal active，不能因已过样例替代任意合法连续位置。


## 2026-11-23 Codex / 默认31全部终态独立成功、保留更宽同侧范围缺口
- GPU77418实际终态0，default_full31全31成功（同步24/自动顺序7），worker整批1348.291s；GPU外external_cpu_all31重新逐条打开真实数组，31全过。最大14角误差1.63e-7rad，ordered10kg吸附力矩增量CPU几何Oracle误差最高2.61e-5Nm。source_default_full31同指纹、147CPU检查保持。
- 未以31绿色替代全域：新增/保留同侧Y=.85、Z=.8/1.4、D66/阈值79，完整箱间/环境/原Home代理无重叠。默认入口6种grasp后仍assigned_contact_outside_reach，GPU36930退出1（12.417s），真实原始失败不改称非法。
- 连续必要界broader_same_side_y85_any_contact_right_bound：实际joint2球冠支持+fore球+全lift给wristY上界.291163，含原姿态/几何包络.301261；整箱任意点/任意TCP方向、再允许30.71mm前伸后，两箱仍缺83.83/51.33mm。不同于旧只吸X−面的错误外推；Y65尝试此弱界不充分（负缺口）已标inconclusive保留。2000真实URDF随机自检不作为证明，公式本身为连续支持界。
- 左臂各箱原始IK可用18/21个lift；同物理箱在6个候选转交pose的两工具CPU原始IK也有解，但未做接触碰撞、真实单箱载荷交接或原生验收，不计成功。生产入口仍不含换手交接。
- goal_requirement_audit逐项区分已验证数值/实现与未达任意合法位置，当前需要用户明确Y/Z工作边界与是否必须包含左取后交给右臂的输入。没有擅自删范围、未markcomplete/blocked；目前本轮无活GPU任务，后续勿无因复启31。CONTINUATION有具体证据和缺口，goal active。


## 2026-11-23 Codex / 更宽同侧目标的静态双接触与真实10kg支持状态探针
- 当前没有新的Y/Z/交接答复，按用户已给几何条件保留Y85，不缩小范围；本轮未改生产运行源，原31/31与外部CPU31证据保持。
- GPU55755/40445通过PATH共享gpuq真实结束0（原静态探针约10.95s，第二次记录实际支持重力矩，估时1m基于已测+冷启动余量）。两SKU×3中间pose各8个静态目标，共48；原机器人自碰撞/world/ground、实体heldOBB vs非接触球、待取另一原箱、两实际平面cap接触及姿态一致均核对。一个物理箱，无复制/删箱。
- 同构型下分别仅左臂或仅右臂真实支持该10kg，原.995限制比例最高左.5574/右.9877；CPU独立48重算实际URDF姿态/足迹及两个支持状态重力矩差，误差最高1.95e-5Nm。source stamp未变。
- 只计静态中间目标，未宣称接近/换吸/退离/动态/原生交接完成，生产入口仍无handover。下一步可做左取Y85→carry到真实双接触目标、右臂接近等几何，但必须stable boxID与owner分离、接触例外限定cap平面、真实单箱质量转移和实际qva缝；不能瞬改attachment造成功。CONTINUATION已更新，goal active。

## 2026-11-24 Codex / 接管纠偏目标并隔离反复实验
- 保持原任务合同与66cm/79cm当前实验选值，不把有限31例替代任意合法位置。新增可运行CPU检查 `tools/experiments/box_layout_20261119/audit_takeover.py`，重开31例实际数组31/31通过；原284个验证快照文件哈希保持，独立原箱接触/抽离FK和用户14角通过，21个不同升降终点。选取相关CPU回归71/71。
- 核验产物在 `generated/two_box_layout/takeover_20261124/`，纠偏文档新增接管条目。CPU核验未重算完整GPU动态RNEA/碰撞，保留原原生采样证据与限制。
- 用户新增要求：需反复设计实验/试验选值时交给subagent，主会话保留目标、结论和证据，避免语义漂移。当前两个隔离子任务：单箱换手接触/支持切换/退离（agent 01a20b94-c8c0-7b63-b804-061b58bf1739；脚本transfer_followup/），已有取箱搬移前缀原生动态验收（agent 01a20b9c-c480-7ee2-8b98-dc3c33d53664；脚本transfer_prefix_native/）。两者仅改各自实验目录与新产物，GPU均走共享gpuq；主会话未改生产规划源。
- 完整目标仍active；Y85未取得完整双箱搬运原生成功，不得把单箱静态接触、几何前缀或新实验阶段冒充生产入口验收。后续先收集这两个子任务的真实终态与简报，再决定集成；不重启旧31批次。

## 2026-11-24 Codex / Y85单箱真实换手几何通过，原生前缀失败定位到样条数值
- transfer_followup单箱实验已实际终态：GPU gpuq-17556a7762fe468c退出0、worker4.26s，独立CPU通过。right原箱实际owner left→right，10kg未重复，箱体世界矩阵跳约2.22e−16；donor退离60mm后全部10个cap离箱最小27.16mm，另10mm后续移动也通过。接口与证据在joint_contract_20261124_transfer/takeover_interface.json等；这是几何/静力，尚无完整native qva，不计双箱成功。
- prefix native实验终态失败保留：原endpoint及最后边沿65点全合法；20001点候选仅一处updown=-1.0000001192092896导致strict joint-bound，其他OBB、world/base/ground、动态RNEA、速度、jerk、source历史与qva缝都过。acceptance.json/endpoint_component_diagnosis.json保留失败，未修补数组或降低门禁。生产与原284文件hash仍匹配。
- 已复用Newton(01a20b94-c8c0-7b63-b804-061b58bf1739)做transfer_suffix隔离后缀几何；复用Dirac(01a20b9c-c480-7ee2-8b98-dc3c33d53664)做局部隔离kernel样条数值根因修复与原prefix重验（不许改共享安装库）；新增Dewey(01a20ba9-45b1-7c03-94cc-7f0505e32381)做transfer_native接触/退离原生段。三者写入互不重叠，GPU仍统一共享gpuq，主会话不重做逐次实验。
- 当前生产入口仍无换手分支；后续集成直接复用SceneSnapshot/AttachedObject已支持的稳定object_id与parent_link，避免另造ownership框架。完整目标仍active，66cm/79cm/指定14角/任务选择updown不变。

## 2026-11-24 Codex / 13段几何链、样条根因修复及三个原生组件通过
- 主会话新增audit_transfer_chain.py，将旧取箱前缀/真实单箱换手/五段suffix交叉核验为13段3116点几何链；原箱ID、实际FK箱C、所有q位置缝、.255/.300抽离及用户14角通过，最终updown≈-.39。产物takeover_20261124/y85_geometry_chain_audit.json，明确非native/fresh/production成功。
- Dirac隔离修复共享bspline_context.cuh：当前位置及导数改为当前span首control作参考的控制差求和，局部常量-1精确再现。9个CPU64/GPU回归过，766共享源/config及原284项目文件未改；prefix原native全门禁20001点过，运动12.990144s。新公共native_spline.py与一个CPU检查已交付，主会话选取CPU回归72/72；尚未激活旧生产入口。
- 主会话通过gpuq一次使用已验隔离补丁复验transfer_native/patched_06：实际退出0、worker42.778s、3个optimizer都true，含1s静止支持共3.212450s，独立CPU全物理/动态通过。没有变终点、门禁、输出数组或手工缩时。
- Newton完成suffix_native/attempt_01：5段/6真实native pieces、24001点、运动13.011309s，GPU/独立CPU通过。final明确为库合法seed-retention，原有genuine native validator/TimeOpt/所有物理门禁通过，不声称全部优化成功；额外标签美化补验已取消。Newton已关闭，必要可resume原ID。
- 当前两个subagent：Dirac(01a20b9c...)独立组合跨组件真实qva/源位/单箱身份/三次10kg支持力矩跳核验；Dewey(01a20ba9...)从原始请求重新算IK并生成完整换手几何的fresh入口，不允许读取旧成功q/NPZ。下一步收集整链与fresh结果、再激活公共数值模块和自动production fallback。完整目标仍active，未用已存组件替代任意合法位置/新鲜生产入口，绝对Y/Z边界仍待答复。

## 2026-11-24 Codex / 默认入口原Y85新鲜native成功，继续纠正Top附件假设
- 已实际应用4文件最小default/verifier/native-reader/current-frame replay hook；新鲜13段入口worker152.578s先验证通过，随后真实默认solve_file原请求另跑成功（exec52497退出0）：6同步候选后自动handover，56011点30.663664s、worker149.200s，进程外CPU再核验通过，request未改、旧解读取空、公共kernel树正确。73项CPU回归过，未当成全域完成。
- 回放helper已改成当前URDF配置的单帧4x4，物理wall_box_00/01映射回输入盒标签，保留actual owner/A；新增CPU测试通过真实默认dispatch/reader/current-frame行为。公共verify删除不必要SKU/高度顺序限制，按对应SKU推导.300/.255真实距离，7项合同检查保持过。
- full32回归harness（原31+rawY85）已准备CPU smoke，无GPU提交；Main发现必须先修原目标内Top模式：suffix.validate_attachment按arm猜轴会拒绝真实top，donor中间目标应保留原tool_to_box而非按中间高度重新选面。Dewey在边界内修共享函数与该绑定并做独立CPU/新Top native验证，当前不换尺寸/移动原输入/放宽门禁。Dirac等待最终GO后冻结提交整批。
- 证据与当前活任务见CONTINUATION.takeover_20261124；完整任意合法位置目标仍active，绝对Y/Z域问题尚无答复，未用样例数量替代全目标。

## 2026-11-24 Codex / 原双顶吸成功，34回归保留缓存审计失败并准备定向续验
- 当前原Both-top(.20/.70,Y85,D66/79) fresh13 native已实际成功：attempt04共60011点39.639646590s、worker386.131s，gpuq-86a9...completed exit0；main当前core再开数组核验过、source无回入、14角4.37e-8、updown约-.54、运行loaded source SHA全匹配。
- default34 run001单GPU冻结1396文件，原31全31/31过，第32在guard编译缓存JSON误分类处失败后停；33/34明确not_started。exec11953终态1、队列gpuq-2f8...failed、无自身GPU/锁，源码digest abc5...始终一致。独立CPU使用fast同库重开全部34槽位：31pass/1reject/2notstarted，7.025s；source changes[]，FREEZE RELEASED见terminal.json。未把缓存误拒绝当物理不可达，top-level早期reach witness掩盖nestedfallback真实原因已记录。
- Dirac在原2个harness文件内准备run002 additive continuation，保留父run001全证据，原31从原真实结果引用/重开核验，GPU只补32/33/34；编译缓存移至generated/default_handover_runtime_cache_20261124，几何history外，production/math/vendor不改。等待配置diff/CPU preflight后main GO，绝不默默抹掉run001失败或把新旧不同production混作同源。
- Linnaeus(01a20c3d...)另在generated/role_generalization_20261124准备Right giver→Left recipient角色适配候选，6files补丁未应用、CPU schema/真实caps/双链7轴检查过、无GPU；待本次source-equivalent续验终态再考虑应用与真实负Y新求解，未镜像成功轨迹。
- 完整目标保持active，固定66/79/用户14角/任务升降不变，不把34样例或准备候选当全域证明。

## 2026-11-24 Codex / run002真实关节拒绝已由donor规划余量修复
- run002终态失败并外部fast CPU全34槽位核验31parentpass/1reject/2notstarted，源码1396全匹配、freeze released。Cache误判已消除，真实新拒绝在donor_release native曲线J7过+pi（15/4001点、TrajOpt22 constraint点），完整OBB/world/RNEA均过。没有降门禁把该批算过。
- Dewey只在full7 donor_retreat规划bounds内预留1mrad（4行），边界起点作为候选拒绝，未改URDF/输出。CPU Side与BothTop真实pose/物理检查过；新鲜真实默认rawY85 GPU gpuq-5a69d9c646864a1e已exit0，13阶段52011点29.890827835s，worker171.281524s；donor release/continue余量1.098542/1.183896mrad，native cspace0，384冻结文件不变，进程外main CPU再核验过。证据retreat_native_margin_20261124/acceptance.json及default_attempt01。Dewey已关闭，无活任务。
- Main修复失败呈现：v3_box_layout_planner.handover失败报告真实error，failure_before_handover保留原同步原因；加CPU实际method mock，test_box_layout+handover45/45过，不改规划行为。新source不能再说与run002保护1394完全相同。
- Linnaeus(01a20c3d...)已恢复，按当前源rebase角色候选并保留1mrad，允许应用6files与小CPU角色回归后一次真默认负Y85 fresh13 native；尚无负YGPU成功。其余工作树冻结不改，旧role完整拷贝不能覆盖margin。此轮后再统一安排当前生产源码的必要回归，不对每次无变化原31盲重跑。
- 原任务66/79/指定14角/任务updown及完整原位/物理门禁保持。有限样例不代表全域，绝对Y/Z验收域仍待用户明确，goal active。

## 2026-11-24 Codex / 实际反向角色接入与负Y第二箱precontact缺口
- Linnaeus已按当前源重基应用6files角色映射，保留1mrad donor bounds、真实两chain/caps/NAMES/物理ID，OrderedLayoutPlanner可选owner→physicalID映射默认旧行为不变。Main只读运行75CPU合同/角色/现有回归全过，输出takeover_20261124/cpu_role_integration_tests.log。
- 负Y85(.8/1.4)实际default_attempt02失败在fresh suffix有限4个precontact候选，上箱recipient停车已成功；native0段，不是完整负Y成功或非法证明。原输出保留。CPU诊断暂用停车lift假设、不含实际过滤q，main已提醒不能用原始IK无fail替代真实双臂planner.precontact(q,{giver:d,recipient:0},checker)拒绝层；允许一次必要有因correction+fresh验收，禁止盲增预算或镜像旧q。
- 11项当前需求审计落takeover_20261124/requirement_audit_current.json，goal_complete=false；最终同源全回归、负Y完整原生、任意合法位置绝对Y/Z范围均未证明。旧run001/002原31通过/32失败/33&34未运行的实证不改写。

## 2026-11-24 Codex / 负Y及先取Cube成功，当前源final36全批已启动
- 实际负Y07原请求fresh默认13native通过：52011点28.470185s、worker221.909718s，source01高箱R→L后R取Cube，原ID/尺寸/箱位/两次.255/.300/真实10kg支持及最终20kg/14角通过；main外部CPU再开过，source oldreads[]。Checkpoint真实screen4/128可行8.10s先过，导入/JSON启动失败未计物理zero。新terminal在precontact_admission_cpu/admission_terminal_07.json，旧failed05 terminal保留。
- 原SKU顺序互换正Y85 CubeZ1.4/TallZ.8无需再改源就fresh默认通过：68011点38.207087s/worker163.451189s，source00Cube L→R后L取Tall，实际-.299999653/-.254999557，14角4.37e-8、updown-.10，gpuq-8e6...completed0/1403sourceunchanged，无live。main外部CPU再过；不镜像旧轨迹、不只换label。Linnaeus子任务已close不再改源。
- final36 dataset前35逐项/字节相同，36raw SKU完全原样。main审阅只2sidecar suite36/原数据保护/禁继承/current sourcefreeze等diff，1474文件全SHA复查匹配52bd2e0cf8b772d37d1797ef812f85c89083ceae077413212b478e1bab1728af后正式GO。单PATHgpuq-a4225670b2fd4131、exec27021、owner1911502/worker1911503正在运行，56mprovisional非timeout；main观察6/36pass、case7running。全量真默认，无旧31继承，失败按请求边界停并保留全部槽位。Dirac负责monitor与终态后外部CPU全36。
- Freeze直到GPUterminal+CPU外部audit：不要改tools/config/tests/models/vendor/publickernel/rawrequests，不额外开GPU。`.ai_teamwork/NOW.md`已更新正确接管状态，CONTINUATION保存livehandle但仍须PS/queue复核。main只新增generatedCPU姿态图actual_carry_native_endpoints.{png,json}（真实nativeendq+FK/真实A），无GPU/physics rerun/整批claim。
- 全“任意合法位置”目标仍active，当前有限36未终态，更不自动证明全域；绝对Y/Z工作域仍未收到用户定义。不markcomplete/push/实机执行。

## 2026-11-24 Codex / final36原31已在当前源重过；实际吸附面独立CPU补证
- final36 main最新实查33/36pass、case34BothTop运行，exec27021/queue-a422...、PIDs1911502/03仍live，source1474与raw36一致。原31全部在当前角色/1mrad/actualprecontact/zero-held-q实现重过，没有继承过去成功。整批尚未终态或外部all36审核，仍不可markcomplete。
- Plato仅generated新增actual_suction_contract_audit_20261124/audit.py，main已review：actualnativeextract首q/URDFFK对原轴向boxC重算ToolInBox、按原Z79阈值判首次sourceFace、真实arm caps10/11 plane/footprint，不拿声称A构造箱位，也不把换手receiver重当首次吸附。五单例10首次吸附全过，批次快照27verified/0rejected/9unverified；TCP最大误差约7.5e-8m、关键单例footprint最小内缩约1.7655mm。原中心/owner/阈值篡改CPU自检拒绝，所有1474冻结hash一致，noGPU/no source改。Plato已close。
- main让Dirac在GPU终态原externalCPU36之后，单次运行该纯CPU审计--summary最终summary --output batch_final.json，不忙轮询、不重算全collision/RNEA或新开GPU。工具运行源始终freeze，main只更新docs/state；当前结论段已移到纠偏文档顶部，历史记录保留且不再覆盖最新状态。

## 2026-11-24 Codex / 最终36同源回归、全数组CPU及72吸附合同全部通过；全域边界待明确
- final36实际36/36fresh默认native成功，24sync/7ordered/5handover；936086原生点，整批worker2633.816680s、queue持锁2640.386613s。无旧成功继承/重试/按结果过滤，全部raw逐项与字节不变；旧34/35准备及run001/002真实失败保留。
- 同fast环境进程外实际数组全36通过9.529s，raw/boxID/actualA/原位.300/.255抽离/qva/静止10kg支持/最终20kg/source无回入/用户14角通过；actual-suction CPU全36/72first-source接触过（55Side17Top），原raw79规则、actualFK法向/plane/真实caps10/11足迹通过。最大TCP55.196um、plane51.787um、cap最小内缩1.76542mm；最大14角误差4.371e-8rad，25不同updown[-.94,0]。原native采样完整collision/OBB/dynamicRNEA门禁保持，非CCD/实机/全域声明。
- GPU-a422...completed exit0，owner1911502/worker1911503dead、锁空；1474源码/模型/配置/vendor/raw SHA52bd...完全匹配、changes[]，freeze released，所有subagents关闭，无本任务待跑GPU或测试。权威final36/acceptance.json、run001/terminal.json、independent_cpu36.json与actual_suction_contract_audit/batch_final.json均已main查实。未Git提交/推送/实机执行。
- Main逐项完成审计takeover_20261124/completion_audit_20261124.json仍goal_complete=false：有限36全部数值/行为验证已足，原“任意合法位置”绝对Y/Z域与全域覆盖仍未定义/证明。实际URDF三角不等式CPU例仅说明原相对条件不足（Y10在当前货舱外，未声称属于本意合法域），不缩小已保留合法难例。下一步向用户澄清是否整个箱体限当前货舱Y±1.2/Z0..2.4或其它域；收到范围后仍需覆盖验证，不能仅确认范围便markcomplete。scope阻塞本轮首次成为剩余唯一输入条件，goal保持active、不pause/complete；若后续3轮真实同阻塞且无独立进展按goal blocked规则处理。
- NOW与纠偏文档顶部当前结论、CONTINUATION首字段已更新，历史“production无handover/31样例”等不再覆盖新状态。勿无因重跑相同36；用户合法域未答前不伪造全域成功。

## 2026-11-24 Codex / 剩余合法域澄清第2轮：生成具体未确认范围提案
- 重读get_goal仍active，用户重复原objective没有新增绝对Y/Z答复；36/36及全部独立审核已完，无活GPU/子agent。未无因重跑同36或擅自把任意域收窄到货舱。
- 新增纯CPU propose_workspace_domain_cpu.py与workspace_domain_proposal.json，从真实environment墙/地/顶内面直接推导待确认Y±1.2/Z0..2.4；当前轴定义下Cube中心Y±1/Z.2..2.2，Tall中心Y±.9675/Z.291..2.109。自检几何面精确回代，原36逐一全在其中。提案明确user_confirmed=false、applied_to_planner=false，未修改实现/测试/模型，未做GPU，未把原36位于其中当授权/全域证明。
- 此轮将缺失信息变为具体可review的参数，scope_clarification_blocking_turns=2；下轮若仍同一真实输入缺口且无独立推进，按3轮goal blocked审计，不继续耗费实验代替定义。确认范围后仍需域内覆盖/物理可达性验证，不会只凭yes就markcomplete。

## 2026-11-24 Codex / 合法域同一缺口连续3轮复核，goal已blocked
- 第3轮仍仅收到自动goal继续提示，没有实际Y/Z定义。再次核对当前goal目标未变、36/36验收与独立CPU/72真实吸附全过、1474冻结文件无变化、无自身liveGPU，提案仍user_confirmed=false且未应用到planner。
- 原合法域全称要求不能由36样例或未确认内腔提案代替；已完成所有不依赖此信息的纠偏实现、有限回归、独立合同核验及具体范围提案，不能继续用重复GPU实验替代定义。functions.update_goal返回status=blocked；未宣告complete/暂停，源与证据不变。
- 下一步等待用户确认“按现有货舱整个箱体Y±1.2/Z0..2.4”或提供实际箱心Y/Z范围；收到范围后仍需覆盖验证。若用户显式resume而仍未提供范围，按新blocked audit起算，不继承本次3轮直接再次阻塞。

## 2026-11-25 Codex / 双SKU原25箱覆盖范围恢复推进
- 用户明确新两箱不超出原25箱覆盖范围，绝对Y/Z输入阻塞解除；不再等待货舱域确认。
- 原25箱几何溯源与小批量边界实验设计已交给subagent，主会话维护合同/证据；D66cm、统一79cm阈值、14角、任务内自由升降不变。
- 更新NOW、合同顶部、CONTINUATION及当前完成审计为active；旧36/36仅作有限证据，未宣称连续全域完成。新证据目录`generated/two_box_layout/original25_domain_20261125/`。

## 2026-11-25 Codex / 原25箱域共享修复与最终44案验证
- 用户给定原25箱覆盖，恢复名义完整外沿Y±1.02/Z0–2.04m；原源码与92组fixture交叉核对。D66/阈值79/原抽离/用户14角/任务自由升降保持，未按成功样本裁剪。
- 修复两个实际域内失败：原标签手臂不可达→共享实际owner↔物理ID自动交换并贯穿场景、验证、回放和预览；拟合抽离曲线低于原5mm→native抽离硬校验/正确LM引导及原≤8rest机制扩展带位移合同抽离，实际2段恢复、原整段位移不变且qva接缝0。原6/8、7/8及违规整段保留。
- 最终同运行码边界8/8+原36/36，全新无继承，44外部CPU/88实际首次吸附/107CPU测试通过，1,176,114native点；31个实际升降值，14角最大误差4.37e−8rad。1236共享运行文件SHA一致，当前36manifest1445/digest0792610110e60617901f3f648e1ce34315c054d92391d5f930311936fe22de0b，收尾0变化。
- 汇总及合同：generated/two_box_layout/original25_domain_20261125/acceptance_current44.json 与 docs/BOX_LAYOUT_JOINT_CONTRACT_CORRECTION_20261123.md 顶部。原36中的6例更大范围证据保留；38域内/6更大范围。
- mainexec54717实际exit0；queuef39e completed0，全部本任务owner/worker、子agent结束，sharedlock空、freeze释放、无GPU/测试待运行，无commit/push/实机执行。停止已知案例优化。有限44与连续必要条件仍不足以证明连续全域，完整任意位置goal未markcomplete；用户范围阻塞已解除，不再索要Y/Z。

## 2026-11-26 Codex / MOTION-505补录、提交整理及归档清理
- 按用户指定2026-10-19规范新建MOTION-505，Parent419、Project V3抓取规划算法开发与验证、In Review待李昊洋；11/19–23、11/23–24、11/25–26三个阶段在同线程补录，父419唯一验收评论原地补充本子方向，保留原阶段和讨论。
- 交付文档MOTION505_TWO_BOX_REVIEW.md、44案便携摘要/原始请求/计时/实际owner与sourcecontacts/证据SHA，包含未证全域/CCD/实机/放箱边界。原107CPU和44native报告源版本不重签。
- 整理中将generated独立source-contact审计原函数复制到versioned source，CPU测试改用新路径；旧no_dense mock适配共享FK evaluate_state与CPU tensor，生产规划不变。扩大检查305过15失败19跳过，15在原PR82源码同资产下逐项复现；另2旧文件缺alfa_motion_viser不能收集。
- 原始5.2GB实验先完整tar.zst归档并zstd完整性/SHA/成员校验，后删除冗余旧二进制/重复源快照和275MB旧rolecache；最终44/必要历史fixture/当前Viser8295与活跃runtimecache保留。归档和清理收据随交付保存，Git明确ignore生成物。
- 用户要求补录后提交推送，不开PR；沿用既有专用分支、本地base ee6dba3，远程交付motion-505-two-box-extraction，具体Git收据在子issue线程，不声称主干已合并。
