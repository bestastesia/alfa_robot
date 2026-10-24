# MOTION-311 小网络构型选型

本页说明 MOTION-311 的代码：默认不变的钩子、场景变体与划分、离线强制打标（MOTION-335），三阶段网络（MOTION-336），逐阶段选择规划器与在线对照（MOTION-337）。预注册见 `config/motion311_preregistration.json`（v1，`correction_v2` 与 `amendments_v2` 为 v2 纠偏与修订）。

v2 目标（2026-10-24 纠偏）：一次搬运有三个相互耦合的 IK——接触 IK（连同 35 cm 抽离终点）、回中 IK、放箱 IK。按阶段依次选定，不回头，使整轮代价最小。代价以秒计：T_lb 为各运动段 max_j |Δq_j| / v_j 之和（URDF 限速），评价用沿规划路径逐帧累加的 T_lb_path；J_arc 并列报告。

## 基线与边界

- 基线：PR #65 头 a77abd5（规划代码 541fdbc，含 MOTION-294）。这是旧零位/Home 版本，结论不外推到上游 0942d18。
- 网络只给出候选尝试顺序，不替代 IK、规划和碰撞验收。所有钩子在不覆写时与基线逐帧一致，已对冻结的 15 轮箱墙做 L0 核对。

## 钩子（默认行为不变）

| 位置 | 钩子 | 默认 |
|---|---|---|
| `v3_lazy_extraction_planner.py` | `lazy_schedule`、`lazy_search_budget_ms`、`generate_contacts`、`baseline_ranking`、`lazy_batches()` | 8/16/32/64 批次，共享 2 s |
| `v3_single_arm_planner.py` | `single_contact_order()`、`order_transport_goals()` | 代价顺序；按加权距离取前 64 个目标 |
| `v3_goal_matching.py`（原两处内联代码合并） | 规划器可定义 `placement_goal_order()` | 距离 + 17 点旋转打分，贪心取 8 个间距 > 0.35 的目标 |
| `v3_scene_variants.py` | `DockingOffsetMixin`，以及划分与清单生成 | 偏移 (0, 0) |
| `v3_search_recorder.py`（仅工装） | 种子偏移 k·1000003、预算倍率、搜索记录 | 不安装 |
| `v3_conveyor_cycle_planner.py` | `conveyor_ik_solver()`、`conveyor_place_targets()`、`dual_place_candidates()` | cuRobo 64 种子放箱 IK，按到运输终点距离排序 |
| `v3_single_arm_planner.py` | `single_place_candidates()` | 单臂解析枚举 |

另修复了一个基线潜在崩溃：顶吸任务在多根选择失败时，部分结果缺少 `phases`/`payload`，`TaskCyclePlanner.full_cycle` 会因此 KeyError。改动只影响原本会崩溃的路径，修复后 L0 复核仍逐帧一致。

## 场景与划分

- 箱位按镜像分组：(row, min(col, 4−col))，共 15 组。
- rng 311 抽 3 组测试格：(0,0)、(2,2)、(3,1)；2 组 dev 格：(0,1)、(4,1)。
- 任务含任一测试箱位就归入测试，否则看是否含 dev 箱位，其余为训练。同一任务的所有偏移和种子都在同一侧。
- 停靠偏移 (dx, dy) ~ U(±5 cm)，平移取箱侧场景，做法同 MOTION-327。
- 清单 `config/motion311_registry.json` 带 sha256，可以复现：`python tools/v3_scene_variants.py --output config/motion311_registry.json`。
- rng 3114 的封存组留给 313/306。

## 打标

`tools/v3_motion311_labels.py` 对每个实例依次做：

1. **基线**：照常规划一轮，不做时间化。
2. **S1**：把 64 个接触候选逐个单独强制尝试。
   - 双臂：一个只含该候选的懒批次，运输预算 2 s。
   - 单臂：尝试顺序设为 [i]。
3. **S2**：对基线选中的根和 1 个随机成功根，各取 16 个目标逐个单独强制尝试。
   - 双臂：基线的 8 个目标，加上 512 近邻中按距离分位取的 8 个。
   - 单臂：实际到达的目标，加上基线 64 个里取 7 个、其外按距离分位取 8 个。

放置后缀只取决于运输终点，所以在实例内缓存。结果分类见 `tools/v3_label_classes.py`。超时记为 `budget_exhausted`（删失），报告用语为"预算内未连通"，不称不可行。

代价 J_arc = 整轮 15 轴加权弧长（升降 10 cm = 15°），加上加密运输路径上的箱体旋转软代价（`BoxRotationCost.edge_cost`）。
每次成功运行还会记录各运动段的起止构型，可以离线算出按关节限速的直线时间下界 T_lb，作为诊断量。
dev 实例另外对基线选中候选和 3 个 J 分位候选做时间参数化，用来核对 J_arc 与执行时长的一致性。

```bash
# 在已执行 prepare_checkout.py 的运行时副本 research/curobo_v3/tools 下运行；每块 ≤25 分钟，各自持有 GPU 锁
setsid nohup /home/astesia/Sevenova/motion311-acceptance/labels_driver.sh <OUT> --groups train,dev --s2 half > <OUT>/driver.log 2>&1 < /dev/null &
python experiments/motion311/label_analysis.py g1 --labels <OUT> --repeat <OUT_K1> --output <OUT>/g1.json   # CPU
```

## v2：放箱打标（D3）

`tools/v3_motion311_place_labels.py`：每个模式（双臂侧/顶吸、单臂左右×侧/顶吸）取 P3 中 150 个实际运输终点作回中 IK g，每个 g 依次跑基线放箱规则、4 个解析 T_lb 最优、2 个最靠近基线放箱解、2 个随机放箱 IK，都走真实的放箱后缀（停车、放箱 RRT、回 Home）。放箱 IK 集合由 `tools/v3_place_ik.py` 按升降 × 臂角 × 分支解析枚举并做碰撞筛选。每个模式另取 20 对 (g, p) 在另外两个任务/偏移下重跑，核对与任务无关。CUDA 显存不足记为 `exception`/`cuda_oom` 并继续。

## v2：三阶段网络（MOTION-336）

`tools/v3_triple_ranker.py` 定义特征与以秒计的价值组合，`tools/experiments/motion311/train_triple.py` 按 N3 → V3 表 → N2 → N1 逆序训练：

| 网络 | 输入 | 输出 | 用法 |
|---|---|---|---|
| N3 | (g, p) 66 维 | 放箱 + 回 Home 在预算内成功概率、实际/T_lb 比值 | 给实际到达的 g 选放箱 IK |
| N2 | (r, g) 56 维 | 运输在预算内连通概率 | 回中阶段的失败风险 |
| N1 | 接触 c 的 73 维 | 抽离 + 贴近成功概率、log V2（蒸馏） | 接触阶段 |

- V3(g) = min_p [T_lb(g,p) + T_lb(p,Home)]（只看 T_lb 最小的 64 个放箱 IK），各模式离线建表（`assets/<mode>/v3_b0.npz`，与规划器的回中候选逐位一致）。N3 只见过每模式 150 个 g，外推到整个回中解集时偏差大，所以回中和接触阶段用解析 V3（A10）。
- 显式 V2(c) = 预测抽离终点的基线目标池（512 近邻）上 min [T_lb(r,g) + (1−P2)·2 s + V3(g)]；N1 的回归头学它（A13）。
- 每个网络：分组 5 折交叉验证（按箱位格）、8 组网格、按联合损失选型、5 种子集成、输入 ±6σ 截断、Platt 校准；特征超出训练分布 6σ 的候选不推荐（A11）。

```bash
# CPU，运行时副本 tools 下
python experiments/motion311/train_triple.py --d3 <D3> --primary <P3>/primary --output <TRAIN> --stages n3,tables,n2,n1
python experiments/motion311/g1v2_report.py --primary <P3>/primary --d3 <D3> --output <G1>      # G1'：各模式收益空间
python experiments/motion311/g2v2_report.py --primary <P3>/primary --models <TRAIN> --output <G2>  # G2'：dev 离线
```

## v2：逐阶段选择与对照（MOTION-337）

`tools/v3_triple_selection_planner.py` 的 `arm`：A = 原规划器；B0 = 只按 T_lb；B = 网络；Bx = B 但接触阶段用显式 V2（消融）。

1. 接触：按 Q1 = T_lb(Home,c) + T_lb(c,r) + V2 + (1−P1)·2 s 选出 c\*，作为单元素 lazy 批次先试（双臂，350 ms，计入共享 2 s），或排在单臂顺序首位；失败后按基线排序继续。
2. 回中：对 c\* 的根，按 Q2 选一个目标单独搜索；基线自己的最优目标排第一或没有候选达到 θ = 0.5 时，用原规则。
3. 放箱：存储放箱 IK 按本轮目标重新解析求解，与基线规则最近的放箱 IK 一起按 Q3 排序，依次单目标尝试至多 2 个，再回到原放箱规则。
4. 整轮失败时由 A 重规划，两轮时间都计入。

`tools/v3_motion311_ab.py` 在同一场景上依次跑各臂（逐实例交替顺序，每块先给每个臂热身一轮），`tools/experiments/motion311/ab_report.py` 给出按模式的配对差（聚类 bootstrap）、判定和 G3′ 一致性。

```bash
# GPU，加锁分块
/home/astesia/Sevenova/motion311-acceptance/ab_driver.sh <OUT> <TRAIN> --groups dev --arms A,B0,B,Bx
python experiments/motion311/ab_report.py --ab <OUT> --offline <G2> --output <OUT>/report.json   # CPU
```

## 测试

- CPU：`python -m unittest tests/test_core_motion311_variants.py tests/test_core_motion311_classes.py tests/test_core_candidate_ranker.py tests/test_core_triple_ranker.py`
- GPU（加锁）：`flock /tmp/sevenova-curobo-gpu.lock python tools/experiments/motion311/l0_equivalence.py --frozen <baseline-wall> --output <dir> --mode plain|instrumented|triple [--models <TRAIN>]`；triple 模式检查载入模型和 V3 表后的 arm A 与冻结基线逐帧一致。
