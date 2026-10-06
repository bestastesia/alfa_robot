# MOTION-233：f044bf1 上的箱墙参数化增量

> 本页是 `52a8c06` 布局节点的历史记录与当时命令。当前完整顺序/单臂实现、验收状态及可执行命令见 `motion233_curobo_wall_sequence.md`；不要用本页旧运行副本对拍后续HEAD。


日期：2026-10-06。工作树 `.motion-233-curobo`，分支 `motion-233-curobo-f044bf1`。
本轮是 PR #43 布局能力在用户指定 cuRobo 核心上的第一批迁移，**不是原 PR 全功能完成，也不是直接重放旧 MoveIt 补丁**。

## 修改与保持不变的内容

- 在现有 `WallLayout → SceneSnapshot → PlanRequest → FullCyclePlanner` 链中增加行列、缺箱集合、车头间距、横向/高度平移和箱间距。没有新增搬运编排器。
- 接触点直接取选中物体的快照几何；规划和随机抽离起点共用该计算，不再由固定箱号反推 5×5 坐标。
- 默认 5×5 场景和60组候选逐值保持 f044 一致；快照变化仍使既有缓存失效。
- ID 从底部/-Y 开始，零基、按行递增；UI 的排/列标签从顶部/+Y 开始。任务目录限最高三个有箱行、双臂同排或相邻排；不是整墙顺序。
- 环境沿用上游 `make_scene`：围壁尺寸不扩大，车头净距改变时围壁随箱墙沿X等量平移；横向/高度/行列修改不会自动扩大或移动围壁。
- 显式空集合表示无箱，不回退到25箱；无可用双臂组合报错。`--request` 不得同时覆盖墙体几何；自定义墙不得加载默认墙的 `--comparison-json`。
- 模型、解析桥源码、限位、joint origin、TCP、SDK及碰撞策略不改。固定箱尺寸仍为0.30×0.40×0.40m、轴对齐；负载尺寸不匹配由原球模型适配器拒绝。
- 2026-09-28 mentor 的机械坐标合同意见仍遵守：不回放旧右臂J2/J4符号或origin修改，不删除上游 planar/TF/Stage/cache文件。旧25/25、固定种子、全网格FCL和性能结论不能转记为本轮通过。

## 实跑证据

目录：`/home/astesia/Sevenova/motion233-acceptance/curobo-layout-v1/`。
运行副本隔离于源码和其他窗口；SDK显式锁定 `78fd485fa82d9b9a063fb4985e371814587e666a`。

| 验证 | 结果/边界 |
|---|---|
| CPU核心 / GPU连接器及BIT* | 26通过 / 6通过，无跳过 |
| 默认5×5、0.9m、L24/R20 | 两次新计算通过，10.424/6.453s，Home误差0 |
| 六规划器 | **5/6，PRM稠密验证失败；严格总门禁未通过** |
| 21组SDK导入 / mesh与缓存隔离 | 通过 |
| 3×4缺箱，10实箱，L11/R8 | 所选一对通过，10.531s，Home误差0 |
| 4×3稀疏，8实箱，L11/R9 | 所选一对通过，10.344s，Home误差0 |
| 5×5、**车头至墙近面0.75m**、L24/R20 | 所选一对通过，10.458s，Home误差0 |
| 默认墙 / 3×4 / 0.75m Viser | 浏览器实际点击重新计算，逐阶段截图及最终帧确认，非预载回放冒充实跑 |
| 无效行列/冲突场景源/旧墙缓存 | 均在规划前拒绝，退出码2 |

3×4使用底部z=0.82m，4×3使用底部z=0.41m，使顶排中心仍处于基线1.84m；这验证布局输入，不代表覆盖所有高度/全部实箱。原默认0.9m对照和用户要求0.75m分别存档，**0.75m是初始车头到箱墙近面的净距，不是箱心X或抽离行程**。该模型车头X约0.508052m，0.75m场景墙近面X约1.258052m。

`rebase-audit.json`校验提交/运行副本/模型一致性和默认回归，遇到PRM失败仍输出报告但**退出码1**，不把部分通过变成总通过；`layout-audit.json`校验布局几何、实际任务、目标/策略、请求ID、有限轨迹、Home和步长。`ui-{custom,075}-check.json`、`ui-{custom,075}-cycle.json`和`viser-*.png`提供可视化证据。`SHA256SUMS`只固化验收产物，不包含运行期间变化的日志/运行目录。

## PRM回归失败：保留，不用重跑成功覆盖

严格验收读取 `six-planners.json` 后发现此前交接摘要的“6/6”不实，实际是5/6：PRM在搬运第464帧失败。诊断为 `left_link7` 负载侧球与 `wall_box_19` 穿入约1.817mm；没有自碰撞或越限。这不是初始Home无效。

在同SDK、同接触候选下，原始f044和本候选各重新规划两次都成功（`mentor-prm-control.json` / `candidate-prm-control.json`），因此未稳定复现生成该路径的条件。进一步将**同一条失败轨迹**分别交给原始f044与候选碰撞检查器，两者都在相同第464帧拒绝、共4个无效帧，碰撞对象和穿入量完全一致（`*-prm-replay.json`，帧SHA256一致）。这证明该失败的碰撞判据没有因布局迁移改变，**不证明PRM根因已修好，也不能据此归咎mentor的新模型**。

原始失败文件不覆盖；搜索采样/预算依赖与稠密复核差异还需查明。没有调整随机种子、放宽碰撞策略或用后来的成功替换该失败。完整六规划器门禁仍不通过。

## 可视化验收

本轮收尾时8095保留**0.75m**页面；右侧显示墙尺寸、实箱数、初始车头净距和研究范围。8090属于其他窗口，不操作。

```bash
xdg-open http://127.0.0.1:8095
```

页面退出后，用以下完整命令重启（端口占用时不要再启动第二份）：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-layout-v1/runtime/research/curobo_v3
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_full_cycle_demo.py \
  --port 8095 \
  --wall-distance 0.75
```

选择L24/R20，点击“计算完整流程并播放”；用“查看阶段”检查吸附、抽离、携箱转身、释放、回Home。“显示附着箱40球”显示本轮实际负载近似。轨迹是研究用关节帧，未验收实机时间参数化。

复跑3×4缺箱的所选一对（请勿与页面实时规划同时占用GPU）：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-layout-v1/runtime/research/curobo_v3
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_plan_cycle.py \
  --wall-rows 3 \
  --wall-columns 4 \
  --wall-bottom-z 0.82 \
  --wall-distance 0.9 \
  --active-box-ids 0 1 2 4 5 6 7 8 9 11 \
  --left-box 11 \
  --right-box 8 \
  --runs 1 \
  --output /tmp/motion233-sparse-3x4-rerun.json
```

校验已保存的布局证据（更新 `layout-audit.json`，不触发规划）：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/verify_motion233_wall_layout.py \
  --artifacts /home/astesia/Sevenova/motion233-acceptance/curobo-layout-v1
```

## 未完成项，不继承旧结论

- 单臂/中列收尾、状态继承、完整25箱顺序与固定种子矩阵未迁入验收。
- 任意尺寸/朝向需重建负载表示和接触合同；本轮不开放。
- `exclude_task_objects_before_contact=true`、`defer_payload_until_extract_end=true`、40个内接负载球、89°倾角原样保留；当前成功不证明完整三角网格/FCL/OBB无碰撞，更不证明实机安全。
- Shortcut/局部降维RRT/轨迹修复/时间参数化及原性能门槛尚未全部在此核心复验。单对完整周期约10s不能宣称达到原逐箱3s目标。
- 仅本地提交；未push、改PR、写Linear或执行硬件指令。
