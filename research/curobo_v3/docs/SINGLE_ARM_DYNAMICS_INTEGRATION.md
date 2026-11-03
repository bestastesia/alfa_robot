# 单臂笛卡尔 RRT / 动力学依赖整合（2026-11-03）

## 状态：单臂链路已验证，混合墙兼容验收未通过

本地工作树：`/home/astesia/Sevenova/.single-arm-cartesian-dynamics-integration`。
分支：`motion-312-cartesian-dynamics-baseline`。未创建提交、推送或修改 PR / Linear。
证据根目录：`/home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103`。

| 来源 | 锁定 SHA | 共同祖先 |
| --- | --- | --- |
| 主线（#74/#75） | `ac82ff0e9adb4606a7cb77278dbf6861c8cf8d52` | 工作树 HEAD |
| PR #73 | `46a3a24e326b00de0c33af343fcc7a0aabe43cd4` | `43567709e337afd5fd0962dd24539d611950968d` |
| PR #71 | `73f40c5c447398ed57089a767481a9fc9f9f04b4` | `3e0ec15d556249a685f7f78bb1251c91da0014e4` |

版本经 GitHub API 复核。通过 fork 获取完全相同的主线对象；按共同祖先生成两个 PR 的研究目录补丁并 `git apply --3way`，不是整树覆盖。原工作树、双臂未提交代码、冻结 runtime 与历史结论均未修改。`pr73.patch`、`pr71.patch` 及 merge 日志保存在证据目录。

## 合并与接口

- 文本冲突：`v3_timed_single_arm_planner.py`、`v3_current_urdf_timed.py`、`test_current_urdf_timed.py`。
- PR71 的时间化不再调用 PR73 的旧钩子。改为现有 `_baseline_waypoints` + 新 `_time_phase` 扩展点，移除失效的 `_shortcut_phase` / `_extra_timing_candidates`，同步测试与旧 spike 调用。
- 搜索返回 `path_source=joint/cartesian/wrist_flip`；失败回退提升真正采用的 sparse waypoints，不能沿用 Cartesian 失败记录。按结果来源而非上一次实例状态分流；周期开始清除旧来源和换腕状态。
- joint：已验碰稀疏路点、0.5° shortcut、固定直边 rest-to-rest；缺失、非有限或端点不匹配的数据拒绝，不能静默退回密化帧。
- Cartesian：不做 joint shortcut；保留 cubic/TOPP-RA 候选及验碰的 polyline fallback。曲线时间化可能改变原始折线，不能称为搜索原始位置逐点冻结。
- wrist_flip：只允许带原停箱边界的三段 TOPP-RA；不再与无停车边界的全段候选竞争。
- `CartesianTransportPlanner` 末尾追加可选 `torque_model=None, use_trajopt=False`。真实 RNEA 接在时间化和验碰之后，TrajOpt 默认不运行。
- 保留新模型的 `plan_geometric_cycle` 动态派发；`trajectory_optimization_enabled` 是新模型 UI 的后处理开关，不等同于 `use_trajopt`，不能因为关闭 TrajOpt 就跳过整个时间化流程。
- 动力学 overlay 写入前核对 URDF 关节几何一致性；证书绑定动力学 URDF、载荷、有效力矩限额、关节顺序及导数限额。位置 hash 使用 float64，分段必须与全局位置一致；修改 payload / 限额后旧证书失效。
- 输出继续区分规划成功、RNEA 通过和 controller-ready。导出检查几何/证书、时钟、有限值、停车端点及力矩数组；5 秒门禁失败不能在再次导出时变为 ready。等时接缝不允许臂轴跳变，底盘导航瞬移仍是外部前置条件，不导出为臂轴指令。
- 失败保留 partial result 与已发生的计时；未测项用 null，不补零。记录每阶段 baseline/时间化尝试、Cartesian 候选及验碰、证书、RNEA 包装与总墙钟。

## 模型边界

单臂仍使用 `generated/v3_analytic_071cb95`、原 Home/TCP/解析 IK/碰撞资产；动力学为 PR71 的 `v3_dynamics_20261019` overlay 与 eRob profile，名义载荷 10 kg、箱尺寸 0.3×0.4×0.4 m、限额比例 0.9。

主线新模型使用 `zero_model_candidate_20261018`（新零位、191 mm TCP、#75 腕部拟合），单独回归。没有把旧动力学 overlay 或旧 IK 混入该入口；没有宣称 Cartesian 已迁移至新模型。

`prepare_checkout.py` 只在新 runtime 中重定位路径并重新编译 PR73 的解析桥，工作树中的资产文件未改。未启用 MOTION-326/334 full-p3 / carriage-lean / lean2 补丁；原碰撞球覆盖缺口仍然存在。`asset-manifest.json` 与 `loaded-assets.json` 保存文件 SHA 和加载清单。模型级 RNEA 不是实机力矩认证，导出力矩仍是需求/前馈参考。

## 验证结果与限制

- 未改动 PR73：22 项 CPU 测试通过；未改动主线：6 项通过、1 项 CUDA 测试在 CPU 环境跳过。
- 机械合并先复现旧钩子失效；新增集成检查在修复前为 8 failed / 2 passed，日志 `integration-red.log`。
- 最终 CPU：**113 passed / 1 skipped**（core、Cartesian 集成、TOPP-RA、current URDF、LM、腕部等）；另有 **4 项 CUDA 测试通过**。
- PR73 开发集 14 场景 × seed 342001 × 3 形态：升级前后 **42/42 成功**，来源分布一致（14 原 joint、18 路由 joint、10 wrist_flip）。最终版全部通过 5 秒门禁与 RNEA，**252 段控制器导出通过**，RNEA 位置变化为零；证据 `baseline73-dev` / `final-dev` / `controllers`。
- 最终 42 次 `full_cycle`：中位 **1.399 s**、最大 **2.452 s**。分项均值：几何规划 1375.65 ms、时间化 96.27 ms、RNEA 包装 35.68 ms、导出 17.78 ms、总计 1529.53 ms（差额为包装开销）。
- 同口径 `run_once.wall_ms` 中位：joint 4546.60→1401.22 ms，B 4450.87→1380.17 ms，B+换腕 2948.49→1519.84 ms。该口径包含 configure_task/full_cycle，不含构造、warmup、指标提取和磁盘写入；主线也发生变化，不能将差值全部归因于 RNEA。中位数不相加，不将历史 PR71 的性能数字作为保证。
- 另跑 25 个开发场景、单一种子，**25/25 成功**：15 Cartesian RRT、9 direct_line、1 joint fallback，记录在 `cartesian-cases`，不是新增留出验收。不能用 42 项中全部非换腕路径均为 joint 的集合代替此验证。
- 降低限额的真实 RNEA 诊断：限额比例 **0.68737337**，`local_window` 峰值时间倍率 **1.33691406**，q 变化 **0**，控制器导出通过；进一步降低至静态重力限额以下正确拒绝。`diagnostic-*.json` 只作诊断，不能并入名义 0.9 限额统计。
- 主线新模型入口实际重算 indices **0/60/76/92：4/4 成功**，`main-model`。
- 回放使用 PR73 自带 CPU replay（不依赖缺失的冻结 alfa-motion-viser 包）。在真实浏览器中切换 right15→left24、播放并确认帧推进，截图 `replay.png`，交互记录 `replay-interaction.json`。显示实际来源、模型、时间化与 RNEA/ready 状态。

### 尚未通过：15 轮混合墙

最终版在第 1 轮 L24/R20 的**几何规划**被拒绝，0 箱完成，未进入时间化/RNEA。`final-wall/round-01.json` 中各候选的 `approach.failure` 是“短接近机器人/目标箱及其他环境碰撞”。主线 #74/#75 将共享 `TaskCyclePlanner.plan_contact_approach` 从排除目标箱的检查改为显式目标箱接触检查（1 mm）。保留该检查后，旧模型双臂流程不能继续。

原 PR73 在独立目录重跑同一混合墙，前 9 轮完成 15 箱，第 10 轮在龙头车放置 RRT 的 2 秒预算内失败（`baseline73-wall`）。这不是旧冻结证据重写，也不能套用旧计划或 PR71 历史的 15/15 结论。

**不得把本整合标为全量验收完成。** 后续需要单独决定旧模型接近阶段的兼容/适配方案；本次没有放宽主线碰撞检查、关闭目标箱检查、偷偷换碰撞模型，或用新模型结果替代旧模型整墙通过。混合墙后续阶段尚未获得本次实际运行证据。

## 直接运行

CPU 回归（使用已准备的独立 runtime，不修改源码工作树）：

```bash
cd /home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103/integrated/research/curobo_v3
env -u PYTHONPATH CUDA_VISIBLE_DEVICES='' PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/astesia/Sevenova/curobo_v2/.venv/bin/python -m pytest -q \
  tests/test_core_*.py \
  tests/test_cartesian_dynamics_integration.py \
  tests/test_toppra_path_timing.py \
  tests/test_current_urdf_timed.py \
  tests/test_frozen_cycle.py \
  tests/test_lm_edge_continuity.py \
  tests/test_wrist_refit_model.py
```

单臂开发集 + 完整动力学链（先检查 GPU 和锁占用，不抢占其他任务；输出新目录）：

```bash
source /home/astesia/Sevenova/curobo_v2/activate.sh
cd /home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103/integrated/research/curobo_v3
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
lslocks | grep sevenova-curobo-gpu || true
flock -n /tmp/sevenova-curobo-gpu.lock python -u tools/v3_motion363_wrist_flip.py run \
  --split dev \
  --seeds 342001 \
  --arms joint_rrt,cartesian_v21r_toppra,cartesian_v22_toppra \
  --payload-mass-kg 10 \
  --payload-dimensions-m 0.3 0.4 0.4 \
  --torque-limit-scale 0.9 \
  --dynamics-urdf /home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103/integrated/research/curobo_v3/generated/v3_dynamics_20261019/robot_dynamics_overlay.urdf \
  --joint-torque-profile /home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103/integrated/research/curobo_v3/generated/v3_dynamics_20261019/erob_joint_profiles.json \
  --keep-full all \
  --output "/home/astesia/Sevenova/single-arm-dynamics-acceptance/rerun-$(date +%Y%m%d-%H%M%S)"
```

回放（当前验收服务在 8283；以下新开 8284，不停止旧服务）：

```bash
cd /home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103/integrated/research/curobo_v3
CUDA_VISIBLE_DEVICES='' /home/astesia/Sevenova/curobo_v2/.venv/bin/python -u \
  tools/experiments/motion312/replay.py \
  --runtime /home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103/integrated/research/curobo_v3 \
  --results-dir /home/astesia/Sevenova/single-arm-dynamics-acceptance/20261103/final-dev/full \
  --cases right15-o1-s342001,left24-o1-s342001 \
  --arms joint_rrt,cartesian_v21r_toppra,cartesian_v22_toppra \
  --port 8284
```

CPU/RNEA 检查、GPU 运行、原始失败、名义/诊断输出均在证据目录。`summary.json` 为机器可读汇总，`integration.diff` 为包含新增文件的本地 Git diff。C/C++ 构建限制始终为最多 8 个任务；没有安装/升级共享虚拟环境依赖。
