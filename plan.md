# MOTION-311 四路线研究 v3

## Context

用户2026-10-25明确要求执行四路线直至最终结果对比，并在无意外时提供GPU独占使用。保留全局flock分块、Linear草稿确认及PR审阅约束。四路线为R1全程监督、R2物理辅助监督、R3候选条件DoubleDQN在线RL、R4连续冗余参数SAC＋解析IK。禁止部署时展开未来候选或查询未来价值表。

## Files

- research/curobo_v3/tools/motion311_learning/contracts.py
- research/curobo_v3/tools/motion311_learning/stage_env.py
- research/curobo_v3/tools/motion311_learning/redundancy.py
- research/curobo_v3/tools/motion311_learning/features.py
- research/curobo_v3/tools/motion311_learning/dataset.py
- research/curobo_v3/tools/motion311_learning/networks.py
- research/curobo_v3/tools/motion311_learning/train.py
- research/curobo_v3/tools/motion311_learning/evaluate.py
- research/curobo_v3/tools/motion311_learning/driver.py
- research/curobo_v3/config/motion311_v3_experiment.json
- research/curobo_v3/tests/test_core_stage_contracts.py
- research/curobo_v3/tests/test_core_learning_networks.py
- research/curobo_v3/tests/test_core_redundancy_actions.py
- research/curobo_v3/tests/test_core_stage_features.py
- research/curobo_v3/tests/test_gpu_stage_env.py
- research/curobo_v3/NN_CONFIG_SELECTION_V3_PLAN.md
- 本机motion311-fourroutes/新数据、模型、运行日志与交付manifest；旧交付不覆盖。

## Analysis

完整讨论设计在/home/astesia/.claude/plans/rosy-crafting-trinket.md。执行顺序：合同与三步环境→共享经验/监督R1→同数据物理R2→在线DoubleDQN R3→连续参数SAC R4→冻结公平对照。先首阶段试点≤6GPU小时，吞吐确认后逐阶段运行，总试验初始上限38GPU小时；若资源不足报告学习曲线而非虚构收敛。

环境每次固定单候选、失败终止、不暗中换根/目标；step1贴近/抽离，step2运输，step3放箱/回Home及完整时间化验收。奖励成功+1、失败-1、全程路径时间归一化势差扣分，基础设施故障truncated不训负样本。

离散三路线候选一致；连续只输出升降与臂角，解析固定参考分支，单臂idle固定当前姿态，公平对照同合同。GPU不多进程共享全局patch，先一个常驻实例；CPU训练与GPU采集分离。旧D3混合train/dev来源不直接作干净标签。

输入仅白名单当前场景/任务/历史、当前候选；下一状态/候选仅训练更新访问。训练数据只train，dev仅选型，原test暂封存等待既定验收人确认。新研究若需要独立比较集，在训练前登记非reserve场景与独立偏移/种子，明确其dev/测试语义。

## Rules Applied

- TEST-REGRESSION-001：奖励/转移/候选/解码合同先测试。
- ENF-POST-004：无未来泄漏、无隐式重试、解析精度、终止目标均有测试。
- ENF-OPS-001：预算为上限不是收益承诺，吞吐和效果用实测证据。
- ERR-TIMEOUT-001：GPU锁/规划/运行块有超时。
- ERR-GRACEFUL-002：转移原子落盘，终止/重启不丢经验。

## Capabilities

- [ ] 真实三步环境通过固定动作/段端点/安全门测试，无隐藏回退。
- [ ] 数据精度、split、来源和候选身份可追溯。
- [ ] R1/R2学习全程未来回报，推理不访问未来。
- [ ] R3实际在线探索并使用DoubleDQN目标/回放。
- [ ] R4输出连续冗余参数经解析IK，分支与idle规则固定可复现。
- [ ] 四路线同场景预算比较，严格首选/末段失败/代价/延迟/采集成本分列。
- [ ] 多种子、学习曲线、负面结果与所有失败保留。
- [ ] 最终源码、模型、数据摘要、runbook与报告交付，外部发布遵守确认。
