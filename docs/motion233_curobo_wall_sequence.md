# MOTION-233：完整箱墙顺序与单臂收尾

> 本页为 `ac38877` 序列节点的历史证据。最新顶吸增量和当前可执行命令见 `motion233_curobo_top_suction.md`，不要用旧运行副本核验最新HEAD。


2026-10-06，基线仍为 `alfa_v3_curobo@f044bf1`，模型/关节限位/TCP/SDK不改。

## 为什么原先没有排4、排5

`fixtures.tasks()`把有箱行截成了最高三行。那是导师单对研究演示的任务目录限制，**不是排4/5不可达的证明**。现已取消截断，5×5目录包含112个双臂组合和5个中列单臂任务；整墙模式是另一个明确的15轮任务列表，不把组合菜单当成连续序列。

## 完整顺序

排1为最高排、排5为最低排。箱号沿用导师从底部/-Y开始的零基行优先编号；L/R表示载箱手臂。

| 排 | 第一轮：外侧 | 第二轮：内侧 | 第三轮：中列收尾 |
|---|---|---|---|
| 1 | L24 + R20 | L23 + R21 | L22 |
| 2 | L19 + R15 | L18 + R16 | R17 |
| 3 | L14 + R10 | L13 + R11 | L12 |
| 4 | L09 + R05 | L08 + R06 | R07 |
| 5 | L04 + R00 | L03 + R01 | L02 |

共10轮双臂、5轮单臂；每箱只出现一次。缺箱和非方形布局沿用同一生成规则，按有箱行从上到下处理，不能用固定25箱表冒充泛化。

## 真实规划与状态继承

- 整墙模式逐轮调用同一个 `FullCyclePlanner`，不导入预设运动、也不另建搬运状态机。
- 单臂任务只附着一个真实箱体、增加一套40球负载。空臂仍参与整机碰撞检查，搬运目标包含空臂收拢姿态；它可以运动，但不虚构第二个载荷。
- 完整周期成功后，用该轮真实规划结果的释放快照和最后一帧生成下一轮请求。已搬箱从场景移除，revision更新，既有cache按新场景失效。
- 一轮失败即停止。失败轮及未规划轮保留在列表中，**只拼接已通过轮次的轨迹**，不删除失败箱、不播放失败轨迹冒充搬运。
- 回放会跟随当前轮次隐藏此前已移走的箱子，只显示本轮真正载箱的手臂；可直接选排4、排5对应轮次。

## 本轮修复及边界

1. 单臂搬运最初让空臂末端不受约束，导致转身时空臂碰侧墙。现使用原有完整双工具搬运目标，检查空臂姿态及整机转身；没有关闭碰撞。
2. 接触IK仍使用原512个优化种子，返回候选从32扩到128，去除重复解，保留原32候选优先。最低排内侧双臂在扩展分支中找到合格解，而不是按箱号硬写关节角。
3. 空载回Home搜索失败时，允许显式反向复用本轮已验证的去程；复用前重新检查空载整条路径及起终点。失败搜索耗时保留在累计耗时中，`home_search_stats`和`fallback_rounds`明确记账。这不是零fallback承诺。
4. 最低排中列L02的独立诊断目前没有合格接触IK：换右臂、约束空臂姿态，以及仅用于诊断的无碰撞IK均未解决。左右臂无碰撞诊断的最小位置误差约12.7～13.0mm，高于2mm阈值；姿态误差也未达标。原解析桥进一步按101个升降值×73个swivel采样，每臂7373次求解都未产生几何解；这是有限采样诊断，不是连续可达空间证明。**诊断不能证明数学上绝对不可达，但不支持将失败归咎于初态自碰撞。** 没有缩短0.75m净距、移动箱子或放宽容差。

本轮从头连续实跑：**14/15轮、24/25箱通过**，剩L02；26197帧，累计规划147.816s。第12轮一次空载回程反向复用，逐帧复核通过。CPU28项、GPU6项通过，原默认0.9m L24/R20两次回归通过（11.859/6.603s，Home误差0）。排4/5抽离、终态剩1箱与第15轮不可播放已由实际浏览器确认；页面显示的是本次headless计算的显式回放，不冒称页面重新规划。

完整连续实跑状态以 `sequence.json` / `sequence.audit.json` 为准，失败试验保留在同目录，不用独立单轮成功拼凑整墙通过。PRM此前5/6回归失败证据仍保留在 `curobo-layout-v1`，本轮不声称它已经修复。

## 运行和可视化

产物目录：`/home/astesia/Sevenova/motion233-acceptance/curobo-sequence-v1/`。
页面8095属于本任务；8090是其他窗口，未改动。

打开页面：

```bash
xdg-open http://127.0.0.1:8095
```

若页面已退出，启动包含完整顺序的验收回放（不是实时重算；界面会明确标注）：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-sequence-v1/runtime/research/curobo_v3
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_full_cycle_demo.py \
  --port 8095 \
  --wall-distance 0.75 \
  --sequence \
  --sequence-json /home/astesia/Sevenova/motion233-acceptance/curobo-sequence-v1/sequence.json
```

界面“规划范围”选整墙序列，展开“完整搬运顺序”看各轮状态；“查看轮次”切到10～15轮检查排4/5。“回放步进”仅控制观看速度，**不是实机时间参数化**。点击“计算完整流程并播放”会从初始墙重新在线计算，而不是继续播放已有结果；失败仍停止。

在无其他GPU规划任务时，从头复跑完整顺序：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-sequence-v1/runtime/research/curobo_v3
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_plan_cycle.py \
  --sequence \
  --wall-distance 0.75 \
  --output /tmp/motion233-wall-sequence-rerun.json
```

校验提交、冻结模型、逐轮请求/状态/释放事件、连续帧、剩余箱与显式回退（未搬完时输出报告并退出1）：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/verify_motion233_wall_sequence.py \
  --source-ref HEAD \
  --result /home/astesia/Sevenova/motion233-acceptance/curobo-sequence-v1/sequence.json \
  --runtime-root /home/astesia/Sevenova/motion233-acceptance/curobo-sequence-v1/runtime/research/curobo_v3 \
  --sdk-repository /home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main
```

继续使用f044的研究碰撞策略：接触前排除当前任务箱、抽离结束才纳入负载碰撞、40个内接球、89°倾角上限。不等同于完整三角网格/FCL/实机认证，不输出硬件命令。未push或写Linear。
