# MOTION-233：侧吸失败后的顶吸分支

> 本页保留 `831edb8` 顶吸节点的历史24/25证据。2026-10-07已专项定位并修复L02：最新25箱证据及运行命令见 `motion233_l02_contact_diagnosis.md`。


2026-10-06。基线继续固定 `alfa_v3_curobo@f044bf1`，未改模型、限位、TCP、0.75m净距和研究碰撞策略。

## 接入范围

- 新请求支持 `auto / side / top`。auto先做侧吸；仅接触IK或抽离/到位候选失败时尝试顶吸，不借顶吸掩盖初态碰撞或资源故障。旧无字段JSON仍为side，原请求ID不变。
- 顶吸朝向沿世界-Z吸附。固定模型左右link7接触平面实测均为约0.175×0.355m；箱顶落点向机器人方向内缩，保留名义5mm边距。0.30×0.40m箱对应X偏移−0.0575m，箱体本身不移动。外形足迹是当前冻结代理模型的几何约束，不等同于真空吸力认证。
- 先检查箱体竖直扫过体与其它箱体/围壁/mesh AABB是否冲突，再用原机器人碰撞检查器验收路径。上方预抓取位下降到接触；吸附后升降轴顶升0.35m，检查升降限位、直线/朝向和带载终点。
- 吸附变换、负载40球、箱体稳定性上方向、缓存键和Viser跟随同一模式，不只改IK朝向。模式变化导致缓存失效。
- 保持箱体最终放置位姿：输入`targets`仍是原侧吸参考Tool0目标；顶吸换算实际Tool0目标并记录`carry_tool_targets`。不能直接拿原侧吸Tool0目标去搬顶吸箱。
- 顶吸搬运目标先过滤不能安全转身180°的IK分支，再进行原路径及整条转身密检；未放宽碰撞判断。

## 本轮实跑

证据：`/home/astesia/Sevenova/motion233-acceptance/curobo-top-v1/`。

| 项目 | 结果 |
|---|---|
| CPU合同/几何测试 | 33项通过，含遮挡拒绝、附着与放置变换、稳定性坐标轴、模式缓存/旧JSON兼容 |
| GPU连接器/BIT* | 6项通过 |
| 默认0.9m L24/R20 | 两次通过，11.729/6.729s，Home误差0 |
| 顶吸对照：剩余箱1/2/3，选择L03/R01，0.75m | 完整周期通过，12.859s、2194帧、Home误差0 |
| 顶吸几何验收 | 模型接触平面尺寸吻合；实际接触平面未越出箱面；垂直抬升35cm；箱体目的位姿不变 |
| 整墙auto，从头连续计算 | **14/15轮、24/25箱通过**；26197有效帧，151.774s；第12轮一次显式空载回程复用 |
| 最后一箱L02 | 侧吸、顶吸均未找到合格IK，失败记录保留，未假定25/25 |

L02本轮侧吸最小位置误差约12.755mm、顶吸约28.055mm，均没有满足位置/姿态双阈值的候选。顶吸目标为`(1.350552051, 0, 0.4)m`、工具Z朝下，接触面仍在箱顶内。额外有限解析扫描也没有找到合格几何候选，**不构成连续空间绝对不可达证明**。

早先箱顶中心/Home参考偏航试验失败；早先顶吸控制目标有一支手肘转身碰侧墙，目标转身预检已修正。试验产物全部保留。两次并发GPU OOM单独保存为`*-resource-oom.*`；空闲后从头串行重跑的结果才作为上表依据，不用资源错误判定算法成败。

## 可视化（不占规划GPU）

8096展示成功的顶吸对照周期，8095展示整墙auto结果及L02阻塞。当前页面使用`--replay-only`，实时计算按钮被禁用，可播放、拖帧、选择阶段/轮次；这是本轮真实headless结果的回放，不是宣称在页面重算。

```bash
xdg-open http://127.0.0.1:8096
```

页面退出后重启顶吸回放：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-top-v1/runtime/research/curobo_v3
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_full_cycle_demo.py \
  --port 8096 \
  --wall-distance 0.75 \
  --active-box-ids 1 2 3 \
  --suction-mode top \
  --cycle-json /home/astesia/Sevenova/motion233-acceptance/curobo-top-v1/top-control-turn-filter.json \
  --replay-only
```

## GPU串行复跑

已与独立顺序卸载线程协调：不停止对方进程。后续GPU批次持有`/tmp/sevenova-curobo-gpu.lock`；不要在别的GPU任务运行时启动未加锁的实时页面。

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-top-v1/runtime/research/curobo_v3
flock /tmp/sevenova-curobo-gpu.lock \
  env PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
  PYTORCH_ALLOC_CONF=expandable_segments:True \
  /home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_plan_cycle.py \
  --request /home/astesia/Sevenova/motion233-acceptance/curobo-top-v1/top-control-request.json \
  --output /tmp/motion233-top-control-rerun.json
```

CPU几何核验：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/verify_motion233_top_suction.py \
  --result /home/astesia/Sevenova/motion233-acceptance/curobo-top-v1/top-control-turn-filter.json \
  --runtime-root /home/astesia/Sevenova/motion233-acceptance/curobo-top-v1/runtime/research/curobo_v3
```

整墙严格门禁仍输出未完成并退出1。原PRM 5/6问题、全网格/FCL、固定种子矩阵和实机验证未借此次顶吸通过冒认完成。未push或写Linear。
