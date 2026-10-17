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

- CPU 101/101（新PR自身测试与在线合同组合；与旧89项不直接比较，旧基线测试集合已由PR65更新）。
- 无障碍approach完成5次非零速度接续，独立GPU复核初始/5候选/尾段通过。
- 新基线初次20场景pilot已经结束；存在动态障碍失败、当前前缀碰撞及放箱浮点切片问题，保留所有负例，不能称完整验收通过。
- 根因之一：单调时钟相减使切点距原节点约1e-12s，生成极短五次插值区间导致伪加速度。修复仅丢弃距精确切点一纳秒内的内部节点，保留实际边界q/v/a；二次曲线回归验证不改变运动与速度。
- `pr65-gates-v2/pilot`已结束6项：approach clear/static/moving及conveyor_place clear共4项通过，conveyor_place static/moving两项无可行替换路线，保留拒绝。全部成功项有独立GPU终检，不能冒称6/6或整体交付完成。
- PR65原生持久碰撞的新增/移动/删除障碍GPU回归通过；`baseline-preservation.json`证明9个核心文件与PR65 HEAD逐字节一致。
- `pr65-gates-v2/full-sequence.json`新鲜完整规划已结束：**15/15轮、25/25箱**。复用`tools/verify_motion233_timed_trajectory.py`逐轮独立检查，15/15通过；结果见`round-audits/summary.json`。
- 全15轮规划ms median/p95/max=2164.003/2990.275/3014.932；时间化ms median/p95/max=4261.147/8858.368/10526.793。整个准备wall=105.454s，含阶段事实导出，不是机器人动作时间。明确不把2.164s中位数当成t2上界。
- CPU反馈过期负例已改为有效轨迹内的sampled_at=10、now=10.2、位速匹配，明确断言stale；不再靠越界错误假通过。

证据目录：`/home/astesia/Sevenova/motion292-acceptance/pr65-integration` 和 `pr65-gates-v2`。

## 放箱障碍用例的分类（不能混为同条件回归）

2026-10-17对PR65放箱失败进行同一GPU碰撞门禁取证：

- **原默认位置**（初始路径60%时的TCP中心、8cm立方体）：静止障碍使原大段终点碰撞；没有障碍时起点/终点均合法。现在返回`GOAL_ENDPOINT_COLLISION`，不调用优化器、不改变终点。`original-goal-blocked.audit-v2.log`由独立GPU重建场景并确认`[start=True, goal=False]`。这是正确拒绝，不是绕障成功，也不证明物理停止已执行。
- **新增中途阻挡正例**：中心`[-1.0486119239962108, 0.635462145723246, 1.277392864227295] m`，尺寸不变。依据基线机器人球体运动几何选取，选择时没有调用路径搜索/TrajOpt，也没有根据求解成功筛选。检查证明已承诺前缀、起点、终点合法，原路中段碰撞。
- 该静止正例3种子均完成5次非零速度接续、1次实际优化绕行，逐次候选、旧前缀和终段经独立GPU审核。证据：`pr65-scene-contract-v4/conveyor-interior`中的static组。
- 同一新增位置的`[0,-0.025,0] m/s`移动组仍被预测包络的终点门禁拒绝；取证在预计到达时刻目标也确实碰撞。`[0.025,0,0] m/s`组起终点合法，但求解/终检仍失败。它们是另两组参数，不与原`[0,0.025,0]`默认方向或静止组混算。
- 试验过将几何搜索提前到第二次尝试，未带来可验证成功率改善，已撤回该编排改动；`pr65-search-order-v5`仅为失败实验存档，不是当前发布代码。

运行器新增可见的`--obstacle-center x y z`（map系，参考时刻0.6*t1）和`--obstacle-velocity vx vy vz`（moving场景）参数，默认值保持原样。参数进入每次报告，完整观测仍逐条保留。不能通过更改场景参数把原失败组写成通过。

在线每次先检查固定接续点/原大段目标；不可用时报告`SPLICE_ENDPOINT_COLLISION`或`GOAL_ENDPOINT_COLLISION`，附完整检查时域与独立安全处置需求。仅说明在当前保守预测包络内不允许发布；不声称所有其他到达时刻都无解。可用`v3_verify_rolling.py --expect rejected --require-rejection-code GOAL_ENDPOINT_COLLISION --gpu`独立确认拒绝原因。

阶段内环境更新不得悄悄改变基座位姿、载荷拓扑或碰撞策略；这些变化应由明确阶段转换处理。运行指纹覆盖所有tools Python文件（包含继承的Mentor类），startup同时记录模型资产及底盘cuboid文件哈希。

最新冻结`pr65-contract-v6`复核已结束：三个静止正例3/3，分别有5次非零接续、6条轨迹（含初始）及尾段独立GPU复核；原终点阻挡负例0次接续、独立GPU确认[true,false]。CPU88/88。此证据不能替代尚未完成的多阶段动态矩阵和保护停机接口验收。

## 非零接续的减速回退（明确含内部停稳）

**行为说明：这不是不停顿优化，也不是硬件保护停机接口。** 正常优先调用原cuRobo优化器；仅当其未给出可接受结果、旧剩余轨迹又已在最新场景中碰撞时，尝试：

1. 保留下一接续点的真实q/qdot/qddot，用已有SciPy `BPoly`轨迹表达生成C2减速候选；不把起始速度清零，不钳制位置来掩盖约束失败。
2. 减速段的完整插值、多阶导数限值及碰撞先通过原门禁，才能以其**已验证的停稳终点**调用Mentor搜索和原TrajOpt，目标仍是原大段终点。
3. 拼接后复核全轨迹，再按实际新时长重建动态包络复核；全过程仍在原t2截止前完成。零时长位置跳变、位速不连续、制动不可行、场景过期或超时都不发布。
4. 统计分别记录直接优化候选/实际直接接续、减速候选/实际减速接续、旧路复核/实际旧路接续；被取消或未安装的候选不冒充已执行的停稳。

保留`--braking-duration-s`校准参数（默认0.5s；下列验证使用显式0.4s）及`--disable-braking-fallback`用于同条件对照。减速候选不是强制最短制动，也不能替代独立安全控制器。

### 同源码、同输入、同预算 A/B 证据

`pr65-braking-release-v13/controlled-comparison.json`逐项核对：初始轨迹、输入文件SHA、阶段、种子、n/t2/t3、障碍位置/速度、减速时长和全部运行源码hash相同，唯一运行开关是是否启用回退。

- 场景仍是已记录的放箱中途移动障碍，center为上一节位置，速度`[0.025,0,0] m/s`；没有更换障碍来覆盖此前失败。
- n=8，t2=0.85s，t3=0.1s，模拟链路0.02s，减速时长0.4s。这里是验证配置，不是硬实时最坏保证。
- **关闭：0/3完成，3项按deadline拒绝，0次接续。开启：3/3完成。** 三个种子实际接续分别7/8/7次，分类为：直接优化接续均0；含内部停稳的减速重规划均1；复核旧路接续6/7/6。
- 每条减速候选接续点仍有非零速度，最大绝对关节速度0.0760454（旋转轴rad/s、updown为m/s）；0.4s后轨迹内部v/a为零。原任务终点保持。独立GPU审核包含原前缀、所有新轨迹、动态时域、最后尾段及内部停稳检查。
- 三种子的compute wall max分别0.6421/0.6491/0.6325s；不能将此小样本的最大值声明为普遍t2上界。
- `pr65-braking-regression-v12`同输入接近段clear/static/moving开启/关闭各3/3；原目标阻挡负例仍拒绝。PR65九个核心文件未修改。
- `pr65-braking-release-v13/replay-playing.png`和DOM证据通过真实Firefox播放/暂停核验，界面明确显示“直接0、减速1、复核6；含内部停稳，非保护停机”。截图不是硬件执行证据。

### 未被掩盖的试验及修复

- 增加SDK控制点16→24/32并未解决同场景非零位速约束失败，未新增/保留此配置扩展。
- 清零初速的诊断可求解，但记录明确`NOT_AN_ONLINE_RESULT`，从未用于在线接续。
- 0.5s减速A/B曾为2/3；扩大规划预测包络的试验仅1/3，因过于保守未保留。全部记录仍在`pr65-solver-diagnostic`、`pr65-braking-ab-v8`和`pr65-braking-time-v9`。
- 0.4s初期在最后短段遇到浮点墙钟加减使窗口末端略大于duration；现直接使用已知相对末端，不放宽超时或碰撞阈值。
- 任意接续点切片不再只用q/v/a重新拟合：子轨迹保存并精确切分原五次Bernstein多项式系数，避免4.5ms首段产生数值伪jerk。系数和q/v/a不一致会被拒绝；旧文档仍走兼容路径。
- 被拒绝候选的实际时域验证会临时更新世界；后续尝试现在显式恢复其规划世界，避免前一失败候选污染下一次求解。
- API拒绝未支持的阶段、阶段与载荷不匹配和非布尔回退开关，不将接触/抽离等受约束阶段误当自由空间已支持。

复现这个已验证配置（使用冻结验证副本，直接复制；新输出目录必须尚不存在）：

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
python /home/astesia/Sevenova/motion292-acceptance/pr65-braking-release-v13/runtime/research/curobo_v3/tools/v3_rolling_matrix.py \
  --initial-cycle /home/astesia/Sevenova/motion292-acceptance/pr65-integration/round-01-v2.json \
  --output-dir /home/astesia/Sevenova/motion292-acceptance/pr65-braking-user-repro \
  --stages conveyor_place \
  --scenarios moving \
  --seeds 20261003 20261004 20261005 \
  --rounds 0 \
  --slices 8 \
  --obstacle-center -1.0486119239962108 0.635462145723246 1.277392864227295 \
  --obstacle-velocity 0.025 0 0 \
  --braking-duration-s 0.4 \
  --compute-bound-s 0.85 \
  --link-bound-s 0.1 \
  --link-delay-s 0.02 \
  --execution-timeout-s 60 \
  --gpu-audit
```

## 受约束接触与抽离阶段

PR65时间化结果还包含`contact_approach`与`extract`。这两段现在进入同一个节拍/版本/延迟/尾段审计框架，但**禁止自由空间绕路**：

- `contact_approach`继续使用PR65的空载阶段碰撞合同；`extract`沿路径使用PR65空载门禁，并在原大段终点额外执行带载门禁。中间小段末端不能误当“抽离完成”。
- 环境更新后，旧受约束后缀完整安全则发布同几何后缀的新证书；若当前已承诺前缀不安全，报告`UNSAFE_COMMITTED_PREFIX`；若前缀安全但后缀通道受阻，报告`CONSTRAINED_PHASE_BLOCKED`。两者都要求独立保护处置，不调用普通搜索或减速绕行。
- 阶段快照来自PR65同次结果：extract包含已确认附件，contact_approach不含附件。阶段、载荷不匹配直接拒绝。
- `pr65-constrained-v17`：清场景contact/extract三种子各3/3；默认static/moving共12项全部按预期`UNSAFE_COMMITTED_PREFIX`拒绝，独立GPU重建碰撞，0次越权接续。该证据说明fail-closed，不等于保护停机已经执行。

`approach/transport/conveyor_place/return_home`允许自由空间重搜索；所有阶段均保持原大段终点，端点被占用时先返回`SPLICE_ENDPOINT_COLLISION`或`GOAL_ENDPOINT_COLLISION`。候选耗尽且旧路仍碰撞时返回`NO_VALID_REPLACEMENT`。监督线程/通信超时返回`DEADLINE_MISS`。每个正常拒绝必须携带failure_code和required_action，独立审计按代码复现原因。

## 验收汇总工具

`v3_rolling_acceptance_report.py`只接受：完整matrix、每项verified、正常场景独立audit、6阶段×5场景×指定种子覆盖、8类故障覆盖、PR65核心保留及15轮/25箱基线。它汇总原始阶段时长，不平均p95；拒绝样本和截尾计算保留。

输出中的`simulation_contract_complete=true`只表示仿真合同证据闭环；`overall_issue_done=false`和external_gates持续列出MOTION-296、真实15轴条件换轨/停止、真实障碍与反馈源、时钟同步及安全评审。该工具不能把外部前置改写成通过。

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
