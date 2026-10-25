# MOTION-311 复现与运行手册

本机代码工作树：`/home/astesia/Sevenova/.motion-311-nn-selection/research/curobo_v3`。
运行时副本：`/home/astesia/Sevenova/motion311-acceptance/runtime-311/research/curobo_v3`（含编译解析桥）。
Python：`/home/astesia/Sevenova/curobo_v2/.venv/bin/python`。

## 移机复现前提

本轮环境与仓库早期README不同：Python 3.11、torch 2.9.1、cuRobo 0.8.0（源码78fd485f）、RTX4060 Laptop；完整包清单随交付包的environment.txt保存。移机须先安装匹配依赖、g++/Eigen3及可用CUDA驱动，再在解出的源码research/curobo_v3执行`python tools/prepare_checkout.py --compiler g++`，重定位冻结路径并编译解析桥。

本地模型/标签不在git源码中；解出交付证据包并按manifest核验摘要，再设置MODELS/OUT等路径。自包含不等于免安装CUDA或自动获得实机验收；原始P3标签及冻结L0帧的本机路径也在manifest中列出。

## 约束

- 所有GPU运行（含短测试）都必须持有 `/tmp/sevenova-curobo-gpu.lock`；每块不超过25分钟。
- 运行中不覆盖runtime文件；先确认本任务无标签/评估进程再同步。
- 默认臂为A；不加载模型不改变基线选择。网络只推荐，碰撞/IK/RRT/时间化验收不能绕过。
- 同一输出目录只跑一个写入者；续跑按实例文件跳过，变更模型/代码须使用新目录。
- 不覆盖历史结果。当前B=explicit映射到历史dev报告的Bx；历史B=distilled不是同一臂。
- 留出测试必须先完成冻结与验收判定确认；本手册不自动批准测试。

## 1. 同步、CPU测试

```bash
bash /home/astesia/Sevenova/motion311-acceptance/sync_runtime.sh
# 从代码工作树 research/curobo_v3 执行；不占GPU
CUDA_VISIBLE_DEVICES='' /home/astesia/Sevenova/curobo_v2/.venv/bin/python \
  -m unittest discover -s tests -p 'test_core_*.py'
```

## 2. 训练（CPU，只读train/dev标签）

在runtime的tools目录执行以下命令，OUT必须是新目录；三个网络和表生成按逆序完成。

```bash
CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=4 /home/astesia/Sevenova/curobo_v2/.venv/bin/python \
  experiments/motion311/train_triple.py \
  --d3 /home/astesia/Sevenova/motion311-acceptance/d3-20261024 \
  --primary /home/astesia/Sevenova/motion311-acceptance/labels-20261023/primary \
  --output "$OUT" --stages n3,tables,n2,n1
```

完整复训可能产生不同浮点结果与摘要；不要用新权重覆盖已有A/B证据。当前冻结候选权重的摘要见 `NN_CONFIG_SELECTION_DELIVERY.md`。

## 3. L0基线与G3回退

从runtime tools执行，`MODELS`为当前训练目录，`OUT`为新证据目录。

```bash
flock -w 1200 /tmp/sevenova-curobo-gpu.lock /home/astesia/Sevenova/curobo_v2/.venv/bin/python \
  experiments/motion311/l0_equivalence.py \
  --frozen /home/astesia/Sevenova/motion312-acceptance/frozen-20261019/baseline-wall \
  --output "$OUT" --mode triple --models "$MODELS"

flock -w 1200 /tmp/sevenova-curobo-gpu.lock env OPENBLAS_NUM_THREADS=4 \
  /home/astesia/Sevenova/curobo_v2/.venv/bin/python experiments/motion311/g3_fallback_check.py \
  --models "$MODELS" --output "$OUT/g3-fallback.json"
```

故障注入只用于验证成功整轮回退与弃权帧一致，不能纳入有效率统计。

## 4. 等价与时延

```bash
flock -w 1200 /tmp/sevenova-curobo-gpu.lock env OPENBLAS_NUM_THREADS=4 \
  /home/astesia/Sevenova/curobo_v2/.venv/bin/python experiments/motion311/v2_latency_bench.py \
  --primary /home/astesia/Sevenova/motion311-acceptance/labels-20261023/primary \
  --models "$MODELS" --output "$OUT/contact.json"
```

仅测接触选择，不是整轮时延。整轮需统计在线报告中contact/carry/place的ms之和；A17后carry.ms包含基线评分和GPU→CPU转换，baseline_score_ms/value_ms仅作分解，不能重复累加。基线IK生成另记baseline_ik_ms，全流程开销看wall_ms；旧计时缺少转换/基线评分，不与新值混用。

新日志的contact_value决定离线对照策略；place.attempt_ranks/resolution决定是否真的首选重解成功。旧日志缺这些字段时不能认证“首选一次成功”，需结合原provenance识别策略或重新运行。

## 5. dev在线与报告

```bash
OPENBLAS_NUM_THREADS=4 /home/astesia/Sevenova/motion311-acceptance/ab_driver.sh \
  "$OUT" "$MODELS" --groups dev --arms A,B0,B --contact-value explicit

CUDA_VISIBLE_DEVICES='' /home/astesia/Sevenova/curobo_v2/.venv/bin/python \
  experiments/motion311/ab_report.py --ab "$OUT" --output "$OUT/report.json"
```

`ab_driver.sh`每块自行拿锁；不要再在其外层持有同一个锁。原始文件保留成功/超时/异常；报告排除基线也无接触IK的场景，但列出其ID。首次单选成功和带回退最终成功分开看。

## 6. 留出测试（尚未执行）

前提：验收人确认判定、模型/代码/臂配置冻结、G3与时延缺口写明。届时沿用上节命令，仅将 `--groups dev` 改为 `--groups test`，输出必须新建。test有双臂72+单臂30实例。不得用其结果继续调参并仍称留出测试。

## 7. 失败与恢复

- chunk exit 3：仍有任务，驱动会释放锁后续跑；exit 0：已选实例均有落盘。
- 异常、OOM、时间化碰撞门：保留为该臂失败，不删除后冒充成功。
- 单实例先写 `.tmp` 再rename；中断后仅重跑缺失完成文件的实例。
- 基线A亦失败并不等于场景不可行：只有无接触IK等固定场景门禁可这样分类；RRT超时称预算内未连通。

## 8. 评审与发布

本地报告可交付；Linear草稿逐条确认后发布并读回。父MOTION-311只在最终验收时写一次总结；子任务不能因本地原型可运行就自动Done。PR仍需用户代码审阅，不自动创建。
