# MOTION-311 完整研究交付收口

## Context

执行用户已要求继续的收口工作，沿用批准的三IK计划和 /home/astesia/.claude/plans/rosy-crafting-trinket.md。当前90项CPU测试、15轮L0、96个dev结果等价均有证据；完整选择p95=17.7ms仍超15ms，且回中hook有计时遗漏。代码、模型、证据交付与正式验收、Linear发布、PR是独立状态。

## Files

- research/curobo_v3/tools/v3_candidate_ranker.py
- research/curobo_v3/tools/v3_triple_ranker.py
- research/curobo_v3/tools/v3_triple_selection_planner.py
- research/curobo_v3/tools/v3_goal_pool.py
- research/curobo_v3/tools/v3_motion311_ab.py
- research/curobo_v3/tools/experiments/motion311/ab_report.py
- research/curobo_v3/tools/experiments/motion311/v2_latency_bench.py
- research/curobo_v3/tools/experiments/motion311/g3_fallback_check.py
- research/curobo_v3/tests/test_core_candidate_ranker.py
- research/curobo_v3/tests/test_core_triple_ranker.py
- research/curobo_v3/tests/test_core_goal_pool.py
- research/curobo_v3/tests/test_core_motion311_report.py
- research/curobo_v3/NN_CONFIG_SELECTION.md
- research/curobo_v3/NN_CONFIG_SELECTION_DELIVERY.md
- research/curobo_v3/NN_CONFIG_SELECTION_RUNBOOK.md
- research/curobo_v3/config/motion311_preregistration.json
- .ai_teamwork/LOG.md
- 本机 motion311-acceptance/README.md、latency-20261025/README.md、新证据目录下的JSON与manifest。

## Analysis

1. 先测试再修正：回中hook从GPU→CPU转换前计时，原基线评分单列，整轮wall不剔除任何工作。
2. 最多两轮等价优化：接触swivel复用、临时数组/静态数据复用、CPU推理线程控制。模型、候选池、阈值、预算不变。
3. 复用exact_goal_pool同距回退、contact_downstream非负风险剪枝和v2_latency_bench的全值验证；全dev值和排序一致、L0帧一致、故障回退成功轨迹一致。
4. 新GPU预算不超过45分钟，单块≤25分钟，每次flock /tmp/sevenova-curobo-gpu.lock。若仍超标写未通过，不无限调参。
5. 用最终源码、模型和证据摘要同步交付文档，历史结果不覆盖。报告严格单选、最终成功、T_lb_path/J_arc、完整选择时延；不宣称数学全局最优或网络相对B0增益已证实。
6. 代码审查和验证后提交/推送现有337分支，不rebase/force-push；PR仍须用户代码审阅。
7. 留出test须验收人确认判定。Linear草稿确认后才可写入并读回，父任务不提前Done。

## Rules Applied

- TEST-REGRESSION-001：缺陷先复现再修复，保留回归。
- SKL-PROC-VERIFY-001：成功声明只引用新鲜运行证据。
- ENF-OPS-001：时延与等价性都给出原始记录。
- ENF-POST-004：候选、值、顺序、帧、计时与单选证据合同有对应测试。
- ERR-TIMEOUT-001：锁等待和GPU运行有上限。
- DOC-ONBOARD-001：运行手册说明环境、命令、恢复与审批边界。

## Capabilities

- [x] 回中选择计时包含GPU→CPU转换，基线评分独立记录。
- [x] 优化不改变全部候选V2、排序和成功轨迹帧。
- [x] CPU测试、全dev等价、15轮L0和单/双臂成功回退验证有最终证据。
- [x] 明确整轮p95是否满足15ms；未满足不得称通过。
- [x] 最终报告、runbook、manifest使用同一版本证据。
- [x] 经验证代码提交并推送337分支，未擅自创建PR。
- [x] Linear、留出测试、正式验收分别记录批准状态。

## Outcome

技术验证：94项CPU测试、15轮L0、96个dev类别/帧等价、90例线上离线一致、两场景成功回退。严格单选80/93，最终成功91/93，完整计时p95=21.34ms，时延门槛未通过；网络对解析B0无额外收益证据。Linear/test/PR未批准状态明确保留，非验收通过。提交推送及交付包摘要由本机manifest记录。
