# MOTION-292：提前一小段计算的动态重规划

状态：开发/验证中，不是已完成验收。基线 `feature/motion-233-curobo-trajopt@3b50c9e`。

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

CPU 合同测试（不要求 GPU）：

```bash
cd /home/astesia/Sevenova/.motion-292-dynamic-replan
source /home/astesia/Sevenova/curobo_v2/activate.sh
python -m unittest discover -s research/curobo_v3/tests -p 'test_core_*.py' -v
```

## 当前证据（2026-10-17；不同版本不混算）

- CPU core **77/77**，包含16项滚动接续合同、证据篡改和缓存兼容性回归。
- `checkpoint-v2/faults/matrix.json`：**8/8**（1个正常闭环 + 计算超时、通信迟到、场景过期、反馈偏差、场景变化、worker退出、solver失败7个负例）。拒绝不等于物理刹停验证。
- `checkpoint-v2/transport-static-seed.json`：带载静止障碍闭环完成，5次非零速度接续、1次旧剩余路线被阻挡后成功换路；独立GPU复核6条初始/接续轨迹及当前前缀通过。计算墙钟最大1.332s，是单次结果而非已证明上界。
- `return_home-clear-v5.json`：4次非零速度接续完成，修正了连续重算不断推迟终点导致的120s超时。安全旧路已复核时，不接收更晚到达的新路；障碍确实阻挡时允许绕行增加时间。
- `frozen-v1/pilot/matrix.json`：只跑到8项，进程已结束；前6项通过，带载static/moving的2项失败，不能视为整矩阵完成。新增当前前缀/未来后缀分时域检查后，static在t2=2s的3s小段中仍于2.961s真实撞障，保留该拒绝记录。改用假定t2=1.8s得到2s小段后前缀安全，但原单种子优化仍不收敛；已有SDK时间步和等权重采样重试才得到上述通过结果。**未修改碰撞或运动限值来使其通过。**
- `candidate-20261017/matrix/`：冻结源文件，3阶段×5场景×3种子及7故障项，共52项串行GPU运行中；每项有源码hash与独立审核。查看`matrix.json`的complete字段，不能把运行中的结果视作通过。
- 静态对照：原`3b50c9e`和首次开发树均完成24箱并在最后箱子的接触IK失败；时间/阶段/完成箱序列一致，但q存在最大0.0194rad差异，因此不称为逐帧等价。已发现并修正静态loaded solver缓存key被无意改变的问题：只有真实在线附件扩展key，静态完整周期保留原复用规则。修正后仍需重跑25箱对照，不能归因未证实。
- Viser只读界面已通过Firefox WebDriver BiDi实际加载、播放/暂停与截图检查；`replay-playing.png`显示4.991s、2次接续、场景版本3及真实事件延迟。不是此前的空白截图。服务器是否存活以当前进程为准。

证据根目录：`/home/astesia/Sevenova/motion292-acceptance`。历史单次试验仅用于定位；正式统计必须同版本、同模型、同配置分组，成功、回用、拒绝、超时分别计数。

### 已核实的真实执行接口不匹配

现有`robot_motion_runtime/common.py`和`execute_trajectory_service_node.py`面向固定14轴rt-control；本基线为15轴（updown + 双臂各7轴），`left_joint7/right_joint7`不在现有执行映射中。服务也没有“期望旧trajectory_id + 未来接续时刻 + 条件替换确认”的合同，场景消息没有移动预测有效期。

因此不能直接把此轨迹塞入当前执行服务、丢掉第7轴或把action accepted冒充接续确认。需要确认**与15轴冻结模型匹配的真实控制器/反馈来源、障碍源、时间同步和保护停止接口**后才可接实机。当前只有模拟执行与核心合同，不假装完成ROS闭环。

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
