# MOTION-292：接入 PR #65 的新规划基线（开发验收中）

## 已确认的基线

- 用户指定的新 PR：`kkozia188/alfa_robot#65`。
- 冻结 HEAD：`541fdbc7250b50ffd5daadae6c8385c54f7b3a92`。
- 292 分支已 `rebase --onto 541fdbc 3b50c9e`；原分支备份：`backup/motion-292-before-pr65`。
- 不修改 PR65 的 `backend.py`、`planner.py`、`contracts.py`、`SingleArmPlanner`、`LazyExtractionPlanner`、`PersistentCollisionPlanner`、`TimedSingleArmPlanner` 和原有时间化流水线。
- Git fetch 因 TLS 失败退出；PR API确认的HEAD与本地已有完整对象相同，直接在该SHA上rebase，不依赖未更新的远程跟踪指针。未推送/force-push。

## 最小接入方式

`curobo_core/rolling_backend.py` 继承 `TimedSingleArmPlanner`：

1. 原大段规划与时间化仍走 PR65 的 `full_cycle()`；不覆盖 `search_path()`、`collision_checker()`、`warmup_resources()` 或 `time_parameterize_result()`。
2. 沿用此前实际运行的 cuRobo `TrajectoryOptimizer.solve_cspace()` 包装，为在线调用提供非零q/qdot/qddot边界。不是改写优化器，也不把旧完整周期规划器覆盖回来。
3. 局部重搜索调用 Mentor 已有 multiroot Connect；碰撞资源调用 Mentor 的持久碰撞检查器，包含 `base_cuboid_layer` 以及相应query cache失效。
4. `approach` 复用 `TargetContactValidity`，并把尚不能接触的任务箱加入优化器世界，避免“求解器省略目标箱、终检再拒绝”的不一致；未放宽终检。
5. 输出每段的 `rolling_stage_inputs`，从 PR65 自己的阶段坐标系与载荷构造，不拿旧模型/旧墙的快照拼接到新轨迹。15轴机械臂阶段允许基座固定在非零位置/朝向；基座真的运动时仍拒绝将其丢弃。
6. 放箱/回程已处于翻转后的底盘坐标系；Viser按实际snapshot基座变换显示，不能强制放回原点。

原完整流程不是一个无边界的连续轨迹：导航瞬移等事件保持 PR65 的明确边界，不冒充动态移动底盘规划。接触/抽离仍受原笛卡尔合同约束，不能用自由空间局部规划替换。

## 时间口径

首次实测（本机Python3.11/CUDA环境，新鲜生成非旧回放）：

- Mentor大段一轮规划：**2754.568 ms**。
- 轨迹shortcut与时间参数化：**2893.957 ms**。
- 含阶段事实快照导出的准备墙钟：**5.677 s**。

这证明相较旧完整周期基线已明显提速，不意味着在线t2=2s已获得最坏情况保证。初始化/预热另列，不混入warm计算。

在线报告分别保留：

- `queue_ipc_s`：主进程入队→worker开始（含序列化、传输与调度）。
- `compute_wall_s`：worker开始→安全结果ready，含当前轮场景更新/搜索/优化重试/复核。
- `request_to_ready_s`：主进程入队→安全结果ready。
- `result_ipc_s`：ready→主进程收到完整结果。
- `sdk_solve_s`：每次SDK诊断求解时间，嵌套在TrajOpt墙钟内，**不能再相加**。
- 观测→ready、实际接受、实际接续反馈仍分别记录。
- 真实感知/网络尚未实测，报告明确unmeasured，不填0冒充实测。

## 当前证据与未通过项

- CPU 84/84（新PR自身测试与在线合同组合；与旧89项不直接比较，旧基线测试集合已由PR65更新）。
- 无障碍approach完成5次非零速度接续，独立GPU复核初始/5候选/尾段通过。
- 新基线初次20场景pilot已经结束；存在动态障碍失败、当前前缀碰撞及放箱浮点切片问题，保留所有负例，不能称完整验收通过。
- 根因之一：单调时钟相减使切点距原节点约1e-12s，生成极短五次插值区间导致伪加速度。修复仅丢弃距精确切点一纳秒内的内部节点，保留实际边界q/v/a；二次曲线回归验证不改变运动与速度。
- `pr65-gates-v2/pilot`已结束6项：approach clear/static/moving及conveyor_place clear共4项通过，conveyor_place static/moving两项无可行替换路线，保留拒绝。全部成功项有独立GPU终检，不能冒称6/6或整体交付完成。
- PR65原生持久碰撞的新增/移动/删除障碍GPU回归通过；`baseline-preservation.json`证明9个核心文件与PR65 HEAD逐字节一致。
- `pr65-gates-v2/full-sequence.json`新鲜完整规划已结束：**15/15轮、25/25箱**。复用`tools/verify_motion233_timed_trajectory.py`逐轮独立检查，15/15通过；结果见`round-audits/summary.json`。
- 全15轮规划ms median/p95/max=2164.003/2990.275/3014.932；时间化ms median/p95/max=4261.147/8858.368/10526.793。整个准备wall=105.454s，含阶段事实导出，不是机器人动作时间。明确不把2.164s中位数当成t2上界。
- CPU反馈过期负例已改为有效轨迹内的sampled_at=10、now=10.2、位速匹配，明确断言stale；不再靠越界错误假通过。

证据目录：`/home/astesia/Sevenova/motion292-acceptance/pr65-integration` 和 `pr65-gates-v2`。

## 复现（隔离资产，不修改原基线工作树）

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
export MAKEFLAGS='-j8 -l12'
export CMAKE_BUILD_PARALLEL_LEVEL=8
export MAX_JOBS=8
mkdir -p /home/astesia/Sevenova/motion292-acceptance/pr65-repro/runtime/research
cp -a research/curobo_v3 /home/astesia/Sevenova/motion292-acceptance/pr65-repro/runtime/research/
python /home/astesia/Sevenova/motion292-acceptance/pr65-repro/runtime/research/curobo_v3/tools/prepare_checkout.py
python /home/astesia/Sevenova/motion292-acceptance/pr65-repro/runtime/research/curobo_v3/tools/v3_rolling_prepare.py \
  --round 0 \
  --output /home/astesia/Sevenova/motion292-acceptance/pr65-repro/round-01.json
python /home/astesia/Sevenova/motion292-acceptance/pr65-repro/runtime/research/curobo_v3/tools/v3_rolling_replan.py \
  --initial-cycle /home/astesia/Sevenova/motion292-acceptance/pr65-repro/round-01.json \
  --stage approach \
  --scenario clear \
  --compute-bound-s 1.8 \
  --link-bound-s 0.1 \
  --link-delay-s 0.02 \
  --slices 7 \
  --output /home/astesia/Sevenova/motion292-acceptance/pr65-repro/approach-clear.json
python /home/astesia/Sevenova/motion292-acceptance/pr65-repro/runtime/research/curobo_v3/tools/v3_verify_rolling.py \
  --result /home/astesia/Sevenova/motion292-acceptance/pr65-repro/approach-clear.json \
  --gpu
```

1.8s/0.1s仍是试验预算，不是用户已验收的最坏上界。CPU检查：

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
python -m unittest discover -s research/curobo_v3/tests -p 'test_core*.py' -v
```

## 完整交付仍需证明

新基线全阶段/全场景、多种子和故障重验；真实停止/预定接续合同；新模型/控制接口对齐；同版本完整延迟证据及PR评审。已授权rebase不等于已授权更改另一仓库Frozen Interface。任务路线由文子轩决定，不自行向他人请求任务修改。仍不发布硬件指令，不将代理限制通过称为真实力矩动力学安全。
