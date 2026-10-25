# 四路线 v3 可复现执行计划

用户于2026-10-25要求执行到四方案结果对比并提供GPU使用。路线、闸门和38 GPU-h初始上限见config/motion311_v3_experiment.json；GPU每块≤24min、全局flock。旧69be4f9交付不覆盖。

## 共同合同

每阶段只选择一次：接触/贴近/抽离→回中运输→放箱/回Home。生成器只用当前目标和已确定构型；网络观察不含未来候选、未来选择、规划结果或V3表。

部署候选：64/512/64；升降21格、臂角36格；与连续actor使用同一个解析nearest branch规则。单臂idle端点保持上一构型，路径可动idle保留既有搜索合同。当前候选未做碰撞预筛，碰撞/IK无解/路径失败如实终止，不能隐藏重试。

所有stage的新增运动阶段在返回下一观测前完成时间化与碰撞检查；终局拼接已检查timed blocks再验证速度/加速度/jerk合同。顶吸内部local frame转换回map base_x后才调用原时间化代码。

奖励为完整成功+1/必要阶段失败-1，加原始路径T_lb累计势差惩罚0.2×C/(C+40s)。shortcut后的路径T_lb和实际执行时间并列报告，不混称同一目标。OOM/系统中断为truncated，不训负标签。

## 共享数据

- 仅group=train AND split=train采集（seen_test/reserve虽可能split=train也排除）。
- 旧P3/D3混合来源只诊断，不作未审计warm数据。
- branching teacher离线每节点4个当前候选：1个近邻＋3个不同构型；实际后续沿树展开。
- root/edge checkpoint以gzip JSON原子保存，node固定随机流；完成节点不重跑，部分树不用于MC监督。
- 每行保存selection_probability；suffix_weight是后续选支概率乘积，纠正“成功分支叶子多”偏差；不取最幸运叶作最优标签。
- transition JSON保留full precision、group、策略版本、真实next/终止/代价；连续不枚举候选时next_candidates=None保留bootstrap。

## 四路线

- R1：反向stage2→1→0全程监督；相同state/action聚合概率加权MC、冻结next-stage教师目标；同任务pairwise排序＋value loss，成功率/成功代价辅助头。宽32/64、3个训练seed，完整task train内holdout早停。
- R2：与R1完全相同标签和任务分组；增加23维当前身体特征，报告额外FK耗时。
- R3：三阶段candidate-conditioned DoubleDQN，当前候选argmax；真实环境采样后更新，回放/target每stage独立，terminal不bootstrap。固定clean warm data、记录全部新增在线经验。
- R4：三阶段SAC，actor输出u/psi参数，解析解码验证；双critic、固定alpha0.05。单臂inactive psi在actor/critic/entropy全链mask；成功动作BC热启动，连续真实探索不替换成查表。

RL恢复：episode先原子落盘，agent.pt原子提交参数/优化器/RNG及last_committed_episode；未提交episode重放更新，不重跑环境。warm内容hash冻结，重复teacher prefix去重。

## 验证与公平比较

比较r1/r2/r3/r4及同当前动作合同的heuristic、uniform连续对照。统一dev场景与规划种子，分别报告三IK严格首选、末段条件/全回合失败、路径代价、实际执行时间、全部wall、生成/特征/推理/解码时延、规划调用和GPU小时。不能把R4更大动作空间收益算成纯学习增益。

原test在既定验收判定确认前封存；dev仅评估不更新权重。额外新数据若入RL，R1/R2也可回灌对照但另存版本。任何失败/OOM/预算中断必须保留；达到预算即报告学习曲线和不足，不承诺收敛。

## 运行

从research/curobo_v3/tools用已有Python启动motion311_learning.driver；driver显式从prepared runtime导入，避免cwd优先导入未准备URDF。同步runtime前必须无运行worker；如需改代码，先在边界停止并保留旧输出，禁止运行中覆盖。

示例：

```bash
/home/astesia/Sevenova/curobo_v2/.venv/bin/python -m motion311_learning.driver \
 --module motion311_learning.collect --log-dir /home/astesia/Sevenova/motion311-fourroutes/clean-teacher-v3b/logs \
 --hours 4 -- --registry /home/astesia/Sevenova/motion311-acceptance/runtime-311/research/curobo_v3/config/motion311_registry.json \
 --output /home/astesia/Sevenova/motion311-fourroutes/clean-teacher-v3b --branches 4 --repeats 2
```

Linear新子任务待草稿确认，不把新研究追加到旧已交付子问题；不自动建PR。所有结果以本机motion311-fourroutes目录下manifest及原始文件为准。
