# MOTION-292 仿真产品验收摘要

冻结代码：`b4998a3`（基于 PR #65 `541fdbc`）。正式证据：
`/home/astesia/Sevenova/motion292-acceptance/pr65-release-v30/`。

> 结论：**仿真合同验收完成；MOTION-292 不能标 Done。** 真实执行、保护停机、真实感知/反馈/时钟以及 MOTION-296 验收仍是外部门禁。

## 产品行为

对每个 PR65 时间化大段，选整数 `n <= t1/(t2+t3)`。执行当前小段时，使用下一边界的真实 `q/qdot/qddot`、最新版本场景和原大段终点计算候选；只在承诺边界安装。

输出严格区分：

- 直接非零速度优化接续；
- 经最新场景完整复核后继续旧后缀；
- 保留非零边界、内部平滑停稳一次后再规划到原终点；
- 当前前缀不安全、固定端点被占、预算内无有效替换、deadline miss；这些情况不发布新轨迹，要求独立安全处置。

接触靠近和抽离保持 PR65 约束通道，不允许自由空间绕路。抽离路径按空载模型检查，只有原大段终点额外检查带载模型。

## 证据

### PR65 基线

- 15/15 轮，25/25 箱；逐轮时间化门禁 15/15。
- 单轮规划 ms：median **2109.056**、p95 **2807.312**、max **2831.987**。
- 时间化单独计时：median **4219.805 ms**、p95 **8725.850 ms**、max **10416.210 ms**。
- PR65 的 9 个核心规划/时间化文件逐字节不变。

### 动态合同矩阵

同一冻结源码、模型和 round-01 输入，6阶段 × 5场景 × 3种子，正好90项：

- 完成45项；安全拒绝45项。
- 拒绝原因：`NO_VALID_REPLACEMENT` 9、`UNSAFE_COMMITTED_PREFIX` 30、`GOAL_ENDPOINT_COLLISION` 3、`DEADLINE_MISS` 3。
- 实际接续分类：直接优化48次、内部停稳后重规划3次、复核旧路181次。
- 8类故障全部按预期拒绝：计算迟到、通信迟到、场景过期、反馈偏差、场景变化、worker退出、solver失败、接收门禁迟到。
- 45个拒绝是刻意把障碍放入当前前缀、固定终点或不可行通道后的 fail-closed 证据，不应从成功率中删除，也不等于产品随机失败率50%。

部分 `request→safe-ready` 秒数（median/p95/max）：

| 阶段/场景 | median | p95 | max | 结果 |
|---|---:|---:|---:|---|
| approach/moving | 0.338 | 0.387 | 0.406 | 完成3/3 |
| conveyor_place/moving | 0.388 | 0.584 | 0.591 | 完成3/3 |
| transport/static | 0.245 | 0.500 | 0.502 | 完成3/3 |
| contact_approach/clear | 0.021 | 0.021 | 0.022 | 完成3/3 |
| extract/clear | 0.030 | 0.037 | 0.041 | 完成3/3 |
| return_home/moving | 0.157 | 0.426 | 0.427 | 完成3/3 |

真实感知生成、跨机网络和真实控制器耗时没有实测，报告中标记为 `unmeasured`，不以0填充。

### 完整序列消费

清场景使用新鲜15轮输入：

- 自由空间阶段：60/60；
- 接触靠近：15/15；
- 抽离：15/15；
- 合计90/90，覆盖双臂/单臂、侧吸/顶吸、非零固定基座与载荷切换。

### 检查与回放

- CPU `test_core*.py`：106/106。
- GPU 原生世界新增/移动/删除回归：1/1。
- 每个成功正常场景有独立 GPU 复核：执行前缀、安装候选、动态时域、最终尾段和终点反馈。
- Firefox WebDriver BiDi 实际加载、播放/暂停并截图；UI明确区分直接、减速后规划和复核旧路，且声明不是实机安全证明。
- `release-manifest.json`记录源码提交及核心证据 SHA-256；`invalidated-evidence-20261017.json`列出并发OOM、旧源码和中途停止的作废目录。

## 复现入口

完整命令见 `docs/motion292_pr65_rebase.md`。主要入口：

- `tools/v3_rolling_prepare.py`：生成 PR65 时间化轨迹和准确阶段快照；
- `tools/v3_rolling_replan.py`：墙钟执行/计算双缓冲模拟；
- `tools/v3_verify_rolling.py`：独立 CPU/GPU 审核；
- `tools/v3_rolling_matrix.py`：首错即停、串行矩阵；
- `tools/v3_rolling_acceptance_report.py`：检查覆盖并汇总原始延迟；
- `tools/v3_rolling_replay.py`：只读 Viser 回放。

## 尚未完成的外部门禁

1. MOTION-296 仍为 In Progress；真实质量、惯量、驱动/力矩边界未验收，`dynamics_enabled=false`、`torque_constraints_enabled=false`。
2. 现有15轴 TwinFjtServer 的 replacement 会立即取消旧goal，尚不具备“提前缓存，未来边界原子替换”的执行合同。
3. 未确认真实障碍源、执行反馈源、时钟同步误差及独立保护停止接口。
4. 未做硬件安全评审、实机Canary、正式PR review/merge。

因此本交付可用于仿真复验、接口评审和后续执行层接入，不允许直接下发实机，也不将仿真拒绝等同物理机器人已经停止。
