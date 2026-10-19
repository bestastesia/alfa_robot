# MOTION-292：提前一小段计算的动态重规划

> **2026-10-17更新：用户已授权rebase PR65。当前入口与复现请先看 `docs/motion292_pr65_rebase.md`；以下3b50c9e结果为历史证据，不能代表新基线验收。**

状态：仿真实现已形成可复核版本；完整验收仍等待负责人确认对接路线、保护停机接口与MOTION-296前置，不是Done。基线 `feature/motion-233-curobo-trajopt@3b50c9e`。

## 已确认的合同

- 先取得一个大段的时间化轨迹及 `t1`；整数 `1 <= n <= floor(t1/(t2+t3))`，周期 `Δt=t1/n`。
- 第 k 小段开始时取得环境和实际反馈；在执行当前小段的同时，计算从 **下一接续点的 q/qdot/qddot** 到原大段终点的完整剩余轨迹。
- 保留旧轨迹的当前小段；计算完成不立即换路。在承诺的接续时间，核对执行轨迹 ID、场景版本、状态新鲜度、位速误差后才换路。
- 新轨迹可改变剩余时间；保持 Δt，在新轨迹上重新取下一接续点。最后不足 Δt 的尾段不拉长；最新预测若完全包含在此前已验证的障碍包络内，复用该安全证据，否则重新检查。
- t2 覆盖排队、场景更新、当前前缀复核、搜索、TrajOpt、时间化及终检；一次性初始化/预热单列。t3 是可配置的暂定通信上界，不是实测硬件保证。
- 超时/落后/约束失败不能提交。模拟器中报 `safety_required` 并终止试验；这不等于证明真实机械臂能安全刹停。

## 为什么扩展这些位置

| 位置 | 必要性与复用 |
| --- | --- |
| `backend.optimize_rrt_path(start_state=...)` | 原接口只有位置、隐含静止初态；添加可选导数，仍使用同一 TrajOpt 与碰撞门禁。旧调用不变。 |
| `joint_state_trajectory(preserve_timing=True)` | 原整周期序列去重会压缩时间；在线轨迹不能把真实停留/重复位置的时间删掉。默认保持旧行为。 |
| `TimedTrajectory` | 复用 SciPy 的五次 Hermite 插值，从 q/v/a/time 一致地取任意接续点；检查整个插值多项式的保守导数界，不只检查节点。 |
| `set_snapshot(allow_attached=True)` 与附件解析 | 完整周期入口要求空载；阶段内规划必须消费实际附着物，不能重新推断吸附事件。默认完整周期限制不变。 |
| `rolling.py` | 无 GPU 的节拍、双缓冲、版本、新鲜度、deadline 和接续合同。 |
| `rolling_curobo.py` | 薄适配：复用现有 `GpuValidity`、搜索、TrajOpt；不重新实现规划算法。 |
| `v3_rolling_replan.py` | 独立 GPU worker 与墙钟模拟执行并行；父进程监督超时并记录执行/计算时间线。没有任何硬件发送接口。 |

在线回退搜索复用已有 `informed_connect` 的首个细验可行解，而非消耗全部预算的 anytime RRT；搜索最多领取剩余预算一半，为优化/验证留时间。不是硬 WCET 证明，最终总 deadline 仍由独立监督检查。

非零初态探针中，默认 `.55` 时间收缩导致部分剩余轨迹在插值碰撞检查失败；SDK 原有的保守 dt 设置可以恢复部分可行解。因此在同一deadline内允许有限的原生dt/种子重采样重试（保留基线首个尝试），仍然经过全部门禁，绝不将失败轨迹当作成功。

## 动态场景与检查边界

`SceneObservation` 绑定单一单调时钟、完整快照、观测/接收/失效时刻、移动物体速度与预测误差界。未在移动清单中的物体由生产者明确声明静止；未知运动不能靠省略速度伪装静态。预测只覆盖声明有效期内的恒速平移与给定误差包络，不预测旋转。

求解沿用静态碰撞世界：分别用“观测→接续”检查正在执行的前缀、“接续→预测截止”检查候选后缀的保守扫掠 AABB；不能把很晚才来的障碍扫掠范围误用到当前前缀。它可能拒绝真实可行的时序穿越，但不会把物体冻结在观测位置。撤离由完整快照显式移除。负载状态变化需要结束当前阶段并重新初始化相应模型。

轨迹验证沿用冻结代理的 GPU 碰撞/地面/载荷稳定性策略：时间采样不超过 25 ms、关节边检查分辨率 0.5°（原基线合同），并额外检查五次插值的关节/导数包络。**这不是精确连续几何证明，也不是实机安全认证。** 不改变现有接触豁免。GPU float32 位置与限位比较仅容许两倍相对 ULP 量级的舍入误差；已有回Home边界实测约 1.13e-7 rad 的表示差，否则会误拒绝。

当前命令支持固定基座 15-DoF 的 approach、transport、return_home；拒绝丢弃移动基座轴。接触抽离、转身等受约束阶段及完整任务执行接口仍待接入验证，不能把这三个入口当作全部阶段已交付。

## 本机复现

先隔离复制与准备资产，避免重定位改动污染源码；编译由原有脚本执行，最多 8 jobs。

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
export MAKEFLAGS='-j8 -l12'
export CMAKE_BUILD_PARALLEL_LEVEL=8
export MAX_JOBS=8
mkdir -p /home/astesia/Sevenova/motion292-acceptance/runtime/research
rsync -a research/curobo_v3/ /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/
python /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/prepare_checkout.py
python /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/v3_plan_cycle.py \
  --trajopt-rrt \
  --left-box 24 \
  --right-box 20 \
  --search-seed 20261003 \
  --runs 1 \
  --output /home/astesia/Sevenova/motion292-acceptance/initial_cycle.json
```

独立终端运行墙钟模拟；以下 2 s/0.1 s 仅为测试配置，**不是已证明的上界**。

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
python /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/v3_rolling_replan.py \
  --initial-cycle /home/astesia/Sevenova/motion292-acceptance/initial_cycle.json \
  --stage approach \
  --scenario clear \
  --compute-bound-s 2.0 \
  --link-bound-s 0.1 \
  --link-delay-s 0.02 \
  --execution-timeout-s 120 \
  --seed 20261003 \
  --output /home/astesia/Sevenova/motion292-acceptance/clear.json
```

### t3 / t4 独立预算

当前`t3=0.10s`只负责规划结果从ready到完整接收，`t4=0.10s`负责接收后连续性、场景版本、终点和有效期验收并进入双缓冲区；切片约束为`n <= t1/(t2+t3+t4)`。为留出t4，抽离35厘米切片由32改为24，固定底盘放箱由8改为7。结果报告分别记录`transport_latency_s`与`admission_latency_s`，不再把两类抖动混算。

### t2 分阶段实际预算（验证继续进行）

基于最终正常场景与15轮clear证据，按同一控制流分支的各span最大值组合上界，加25%余量并向上取整50ms，当前实际预算为：初始姿态→预接触点0.95s、最后3厘米贴近0.05s、抽离35厘米0.10s、携箱运输0.65s、固定底盘放箱0.80s、返回初始姿态1.10s。第1轮默认动态场景六阶段通过且0 deadline miss；15轮、最坏障碍多种子、GPU压力和故障矩阵仍在继续验证，若证据要求会继续调整。证据见`motion292-acceptance/t2-calibration-20261018/summary.json`。

### shortcut 性能优化

旧shortcut一次检查当前点到全部后续点的0.05°精确边，并按最长边horizon补齐，15轮时间化累计69.916s。现改为从最远端开始每16条一批反向精确检查，保持分辨率、最远合法端点和最终轨迹不变；15轮时间化降至26.790s（-61.68%，2.61×），证据见`motion292-acceptance/shortcut-optimization-20261018/summary.json`。

### 时间化方法同阶段对照

第1轮`approach`阶段的Mentor rest-to-rest与cuRobo TrajOpt量化对照、代价收益和当前选择建议见`docs/motion292_time_parameterization_comparison.html`；机器可读证据为`motion292-acceptance/time-parameterization-compare-20261018/approach.json`。

### 实时 Viser 完整轮诊断界面

新界面启动时不载入任何历史轨迹。点击“从头规划并实时演示完整一轮”后，它会：

1. 在本次点击后重新运行 `v3_rolling_prepare.py --round N`，完成该轮六阶段初始轨迹规划；
2. 一次性启动常驻CUDA滚动worker，并在执行前预热空载/带载TrajOpt与碰撞资源；随后按 `approach → contact_approach → extract → transport → conveyor_place → return_home` 逐阶段复用同一worker；
3. 灰色分别显示左右末端本次新生成的原轨迹，绿色/蓝色在每次接续轨迹真正产出并激活时才追加左右末端新路线；
4. 主流程把点击到可执行轨迹拆成进程/CUDA启动、模型与GPU预热、Mentor核心规划、轨迹时间化、快照/写盘/通信五项；核心规划与时间化各自再提供可加和内部总账；
5. 每个阶段分别显示核心规划归属账、RRT首次解/首次验收/总搜索等嵌套诊断、路径快捷化/时间参数化/时间化碰撞验证/结果组装，以及每次滚动求解和仿真实际运动墙钟。共享项与嵌套项明确禁止重复相加。

在上面已准备的隔离 runtime 中直接启动（8104 若仍有旧界面，使用 8105）：

```bash
cd /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3
source /home/astesia/Sevenova/curobo_v2/activate.sh
PYTHONPATH="/home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools:/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main" \
python -u /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/v3_rolling_interactive.py \
  --preparer /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/v3_rolling_prepare.py \
  --runner /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/v3_rolling_replan.py \
  --round-runner /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/v3_rolling_round.py \
  --round 0 \
  --execution-timeout-s 120 \
  --port 8105
```

`--initial-cycle`/`--initial-result`只为兼容旧启动命令而保留，界面不会读取其中的预计算路线。

CPU 合同测试（不要求 GPU）：

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
python -m unittest discover -s research/curobo_v3/tests -p 'test_core_*.py' -v
```

## 当前证据（2026-10-17；不同版本不混算）

- CPU core **89/89**，包含28项滚动接续合同、证据篡改和缓存兼容性回归。
- `checkpoint-v2/faults/matrix.json`：**8/8**（1个正常闭环 + 计算超时、通信迟到、场景过期、反馈偏差、场景变化、worker退出、solver失败7个负例）。拒绝不等于物理刹停验证。
- `checkpoint-v2/transport-static-seed.json`：带载静止障碍闭环完成，5次非零速度接续、1次旧剩余路线被阻挡后成功换路；独立GPU复核6条初始/接续轨迹及当前前缀通过。计算墙钟最大1.332s，是单次结果而非已证明上界。
- `return_home-clear-v5.json`：4次非零速度接续完成，修正了连续重算不断推迟终点导致的120s超时。安全旧路已复核时，不接收更晚到达的新路；障碍确实阻挡时允许绕行增加时间。
- `frozen-v1/pilot/matrix.json`：只跑到8项，进程已结束；前6项通过，带载static/moving的2项失败，不能视为整矩阵完成。新增当前前缀/未来后缀分时域检查后，static在t2=2s的3s小段中仍于2.961s真实撞障，保留该拒绝记录。改用假定t2=1.8s得到2s小段后前缀安全，但原单种子优化仍不收敛；已有SDK时间步和等权重采样重试才得到上述通过结果。**未修改碰撞或运动限值来使其通过。**
- **最新分阶段矩阵 `acceptance-v3`：49/52。** approach 22/22（含7故障），transport 12/15，return_home 15/15；正常场景42/45完成，3个transport/appear在当前不可修改前缀中碰撞，拒绝下发。不能把“需要独立保护停机”改计为绕障成功。该组固定同模型/硬件/三种子，n分别7/3/5，t2=1.8s、t3=0.1s；全部成功项有独立GPU复核及最后尾段证据。
- `watchdog-v1/checks` 9/9：正常/移动闭环及7故障通过；`admission-v1/checks` 3/3：正常/移动闭环及故意延迟接收门禁通过。后者的晚接收候选0次接续，不允许回填一个过去的接续时刻。
- **schema3与端到端门禁：** 最后不足一个周期的尾段必须记录新场景复核或有效证书复用，审核器重建实际尾段并检查终点反馈。管道send/recv移到单个独立I/O线程，主监督线程只访问有界本地队列；半包/阻塞发送测试证明不会阻塞截止检查。完整回复允许使用t2之后的t3，但实际ready_at、接收门禁完成时刻及接续时刻分别受限，不将通信时间算成计算超时，也不倒填接受时间。
- **模型锁定关节：** 复用原全周期的锁定关节检查并下沉到共享set_snapshot；在线模式要求明确提供头部锁定轴状态，不再悄悄忽略这些轴的真实运动。静态旧入口保留缺省兼容。
- `candidate-20261017/matrix/`（0d790cd代码）：**52项已经结束，43过/9失败**。45个正常场景36完成、9拒绝；7故障项正确拒绝。失败是approach/moving三种子优化不收敛、transport/moving三种子deadline miss、transport/appear三种子当前前缀碰撞。该矩阵不是通过，不能隐藏失败。`raw-latency-summary.json`按阶段/场景汇总原始时长及超时截尾数，未对p95求平均。
- 后续修复：预测TTL不再被当成机器人占用时长；规划先用原剩余时长，最终复核按候选实际时长重建包络。用SDK原生`update_world()`保留预热机器人，移动障碍不再每次重建模型；GPU新增/搬走/删除障碍回归通过，确认不会沿用旧世界。schema v2记录终点反馈，审核不只看success布尔值。
- `warm-world-v1/approach-moving-n7.json`：显式n=7、t2=1.8s、t3=0.1s，7次移动接续、1次真实避障，独立GPU审核8条轨迹及前缀通过；计算墙钟max 0.988s。这里只是该配置单次证据，不能替代失败的n=8矩阵。`warm-world-v1/transport-moving-n3.json`也通过，5次移动接续、1次避障、max计算1.061s。突现更短节拍的试验仍有超时，未当成通过。
- `phase-progress-v1`再次跑同样的approach n7、transport n3移动场景：独立GPU审核分别通过8/6条轨迹，终点反馈通过；优化失败且旧后缀已完整复核安全时，不再白跑其余搜索/优化重试，明确计为revalidated而非optimized。这两个场景compute max分别0.701s/0.642s，不能宣称普遍上界。通信迟到计入deadline miss，缺少计算测量的失败计入censored而非从统计中消失。
- n是用户可选的正整数，不要求永远取最大值。`v3_rolling_matrix.py --slices`可固定n；预算符合公式只是计算能赶上的必要条件，不保证任意突然进入当前不可修改前缀的障碍都可避开。
- 静态对照：原`3b50c9e`和首次开发树均完成24箱并在最后箱子的接触IK失败；时间/阶段/完成箱序列一致，但q存在最大0.0194rad差异，因此不称为逐帧等价。已发现并修正静态loaded solver缓存key被无意改变的问题：只有真实在线附件扩展key，静态完整周期保留原复用规则。修正后的`acceptance-v3/static-sequence.json`已重跑；与原3b50c9e的frames/qdot/qddot/jerk/time逐元素完全相同（最大差0）、阶段与载荷相同，同样24箱后在箱2接触IK失败。证据`acceptance-v3/static-regression.json`，证明该冻结基线无静态轨迹回归，不证明25箱验收通过。
- Viser只读界面已通过Firefox WebDriver BiDi实际加载、播放/暂停与截图检查；`replay-playing.png`显示4.991s、2次接续、场景版本3及真实事件延迟。不是此前的空白截图。服务器是否存活以当前进程为准。

证据根目录：`/home/astesia/Sevenova/motion292-acceptance`。历史单次试验仅用于定位；正式统计必须同版本、同模型、同配置分组，成功、回用、拒绝、超时分别计数。

### 已核实的真实执行接口不匹配

现有`robot_motion_runtime/common.py`和`execute_trajectory_service_node.py`面向固定14轴rt-control；本基线为15轴（updown + 双臂各7轴），`left_joint7/right_joint7`不在现有执行映射中。服务也没有“期望旧trajectory_id + 未来接续时刻 + 条件替换确认”的合同，场景消息没有移动预测有效期。

因此不能直接把此轨迹塞入旧执行服务、丢掉第7轴或把action accepted冒充接续确认。

补充查证：旁边的`robot_motion_control@a2c7cf4`已有可复用的**15轴仿真**FJT，配置是`alfa_motion_ros2/config/twin_fjt_3_1_1_suction_chassis.yaml`，接口为`/simulation/dual_arm_with_updown_jtc/follow_joint_trajectory`。它支持future header stamp和replacement，但`TwinFjtServer::apply_replacement()`在下一tick立即取消旧goal，再等待新goal开始；这不满足“旧前缀继续走，到接续点才换”的原子语义。其3.1.1权威模型也不等于当前冻结cuRobo代理，不能仅按关节名一致就放行。`V3_EXPERIMENT_BASELINE.md`明确真实Runtime/安全合同未冻结；未修改该仓库。

下一步应优先复用该15轴孪生接口，而不是另造控制器；需先由本任务负责人文子轩决定对接/变更路线，再由其协调必要的架构与控制接口确认。先前直接向李昊洋提出的任务接口授权请求已撤回，遵循Linear更正4f5a357f；该更正不等于变更授权，MOTION-296原有验收前置不取消。涉及Frozen Interface仍不得擅自修改。真实执行仍需**对应控制器/反馈来源、障碍源、时钟同步及保护停止接口**。当前交付为模拟执行与核心合同，不假装完成ROS或实机闭环。

## 验收清单（不能用窄测试代替）

- [x] 初始节拍公式、下一点非零速度、仅边界换路、原大段目标不变的 CPU 检查。
- [x] 迟到、旧场景、已取消结果、位速不连续及边界跟踪误差拒绝检查。
- [x] 新基线单轮完整周期生成；非零初态 GPU 求解与既有碰撞门禁通过的探针。
- [ ] 静止、移动、突现、撤离，空载/带载，各多种子墙钟闭环与失败矩阵。
- [ ] 全部必要阶段接入；完整 25 箱静态回归。
- [ ] deadline miss、碰撞拒绝、取消与完整延迟时间线统计、可视化回放。
- [ ] 输入/输出及所有模型、SDK、代码、配置哈希的可追溯证据。
- [ ] 真实感知和执行反馈来源、时钟同步、控制器预定替换/停止接口确认及 ROS 联调。
- [ ] MOTION-296 验收、独立实机安全门禁、PR/review/检查。

质量/惯量/驱动参数前置仍未解除，`load_dynamics=false`，不得把代理关节限制通过称为真实力矩动力学通过。

只读回放命令（先检查8099是否有已有服务，不能重复占用）：

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
python /home/astesia/Sevenova/motion292-acceptance/runtime/research/curobo_v3/tools/v3_rolling_replay.py \
  --result /home/astesia/Sevenova/motion292-acceptance/static-v2.json \
  --port 8099
```

## 继续完整交付所需的决定

1. 由文子轩确认是否扩展现有15轴孪生接口：提前缓存候选、旧前缀继续运行、在未来边界按执行ID和位速原子接续；拒绝/过期时具备可确认的保护停止。
2. 确认继续使用冻结3b50c9e代理模型，还是迁移到获准的模型/控制合同；不能仅凭15个关节名相同替换3.1.1模型，也不能覆盖正在改动的原基线工作树。
3. MOTION-296的25箱与动力学适用边界验收，以及真实状态、障碍源、时钟域和制动/执行接口来源。

当前成果没有发送硬件命令，没有更改robot_motion_control的冻结接口，也未发布到公开fork。基线分支已发生并发变更，PR目标基线与公开发布也须先确认。完整目标保留，不将仿真子集当作整项完成。
