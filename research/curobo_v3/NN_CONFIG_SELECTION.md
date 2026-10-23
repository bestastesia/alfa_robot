# MOTION-311 小网络构型选型：数据与钩子（MOTION-335）

本页说明 MOTION-311 子任务 a 的代码：默认不变的钩子、场景变体与划分、离线强制打标。网络训练见 MOTION-336，接入与对照见 MOTION-337。预注册见 `config/motion311_preregistration.json`。

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

## 测试

- CPU：`python -m unittest tests/test_core_motion311_variants.py tests/test_core_motion311_classes.py`
- GPU（加锁）：`flock /tmp/sevenova-curobo-gpu.lock python tools/experiments/motion311/l0_equivalence.py --frozen <baseline-wall> --output <dir> --mode plain|instrumented`
