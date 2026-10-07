# MOTION-233 迁移要求与当前证据（2026-10-07）

## 基线和提交边界

用户明确指定 `alfa_robot/alfa_v3_curobo@f044bf1`，并再次确认主线保留RRT。2026-10-07只读核对远端，`alfa_v3_curobo`仍为完整SHA `f044bf1463d6ab138bb5aac078056d1e3d17a1b9`。本地候选工作树为`.motion-233-curobo`，分支`motion-233-curobo-f044bf1`。

原PR #43仍是`b72ba30`→`alfa_v3_dev`，Open/Conflicting。mentor的2026-09-28评论明确禁止合并旧模型/旧流程，要求重建替代PR、选择性迁移，不回退模型/planar/执行链。**因此不是强行解决冲突再重放整个旧栈。** 本轮没有push、改PR或写Linear。

## 要求映射

| 要求/来源 | 已有证据 | 当前判定 |
|---|---|---|
| 用户指定最新cuRobo基线及模型 | 远端SHA、祖先关系；生成资产路径归一化和模型/vendor逐文件核验 | 已落在指定基线；不沿用误选的alfa_v3_dev Stage实验结论 |
| mentor统一机械坐标、零位、限位、TCP，不回退右J2/J4与joint origin | 相对f044，模型、vendor、生成模型资产未修改；全墙逐帧URDF限位复核 | 已保留，面内抓取变化不是修改TCP或机械限位 |
| 不删除planar根、底盘TF、Stage Action、轨迹缓存 | `git diff f044bf1 -- ros2_ws`为空 | 源码保留；没有声称新研究入口已接入/验证实机执行 |
| 用户要求RRT主线 | 默认informed_rrt；三种已有RRT函数AST与f044一致；25箱记录全为informed_rrt | 已保留。PRM修复隔离，不是本迁移门禁 |
| 车头净距0.75m、完整搬运序列、单臂收尾及顶吸 | 15轮25箱实跑，实际模型前缘差值、状态继承、面内足迹/附着/目标位姿和只读回放 | 当前研究碰撞口径通过；旧固定中心点没有被冒报可达 |
| 原PR参数化箱墙/缺箱 | WallLayout、快照取接触点、CPU合同；历史3×4/4×3选中双箱实跑 | 接口已迁；不同尺寸仍受冻结负载模型限制，不能说原全部L4场景已迁完 |
| 原PR固定公开种子 | 复用原8数值并保存原manifest；新请求/CLI将seed传入RRT各阶段，8例各一次25箱 | 球模型口径8/8通过；不等于旧OMPL轨迹或位级确定性已等价 |
| 原固定种子C/D精确轨迹对拍 | 新同请求同seed两次预检都成功，但运输阶段轨迹/冗余终点不同 | 尚未达到严格位级重现；不能用“有seed参数”冒认完成 |
| 原逐箱时延、预算、时间参数化、质量指标 | 新入口记录总时间及阶段/失败尝试；当前单轮多为8～12s，仍输出未计时关节帧 | 不能继承旧3s/5s门槛或实机时序结论；未完成 |
| 原完整FCL复核 | 新基线原样保留球模型。CPU对旧默认25箱抽查90个关键状态，发现3处未豁免网格相交，成对复查可复现 | 不可沿用旧模型FCL结论；需要专门处理模型表示差异，未改豁免/阈值 |
| 原Golden算法与参考资产的最终取舍 | 原PR/Git对象和冻结报告仍在；新实现复用导师核心，未移植旧坐标补丁 | 不宣称旧全部算法/预算/执行接口已迁移；需按要求继续收口 |

**整体目标仍未完成。** 已通过的研究序列、种子矩阵和模型对拍只是上述相应项的证据，不替代其它门禁。

## 种子合同

`PlanRequest.search_seed`和CLI`--search-seed`只控制RRT采样，不改IK策略、IK默认随机配置或2秒单段搜索预算。省略该字段保持旧请求ID、旧阶段种子与默认行为；接近阶段使用base+候选索引，搬运使用base，回程使用base+1，全部按uint32取值并写入结果。

`tests/fixtures/OMPL_SEEDS.json`从原PR `b72ba30:tools/v3_scoop_golden_20260921/OMPL_SEEDS.json`逐字节保留。它的旧名称不表示当前使用OMPL，数值集合为104729、104743、104759、104761、104773、104779、104789、104801。原manifest中的时延要求没有被删除，但当前不能宣布满足。

每个矩阵例独立进程、每例持共享GPU锁并检查其它compute PID，例间释放锁。固定清单每例只执行一次；失败要保留，禁止挑成功样本。壁钟预算和缓存中的IK采样历史都可能影响最终路径，因此只固定RRT种子不是完整确定性证明。预检分阶段对比显示起点/吸附/抽离完全一致，运输阶段开始不同；具体确定性收口尚未完成。

## 独立网格诊断的边界

诊断读取`l02-diagnosis/sequence.json`中**此前默认种子**的成功轨迹，并使用同一固定模型的URDF collision meshes、同一状态和研究碰撞豁免。它不是对八个新种子逐帧完成的FCL验收。

| 轮次/阶段 | 网格接触对 | CPU球对最小间隙 |
|---|---|---:|
| 第9轮，抽离终点 | arm_carriage / left_link5 | 约3.07mm |
| 第13轮，抽离终点 | base_link / right_link2 | 约56.89mm |
| 第14轮，携箱转身 | right_link4 / right_link7 | 约3.47mm |

两个独立组织方式的FCL查询都报告网格相交；上述球对仍分离，说明当前模型表示给出了不同判断。OBB这里只是辅助包围盒，不把非凸网格的OBB重叠当作实物碰撞证明。没有新增忽略项、没有缩放网格/球半径，也没有据此更换RRT。最终物理安全结论需要继续核对碰撞表示及完整路径。

## 第12轮单箱动作诊断（2026-10-07）

当前8095回放来自`l02-diagnosis/sequence.json`；第12轮右臂搬R07，共2993帧。接近904帧、回Home976帧，空闲左J1累计分别275.47°/310.53°，左J4在接近阶段累计257.29°；这些是累计绝对关节运动，不是净角度，也不是执行时长。

确认的机制：单箱接触只约束作业TCP，IK/搜索仍允许空闲臂运动；携箱阶段仍要求两个手达到mentor指定carry目标。接近/回程分别保留27/26个路径点，该旧回放使用`apply_shortcut=False`；第12轮回程首解约2012ms才出现，未触发反向复用回退。没有PRM切换。

**纠正初步判断：不能把所有空闲臂运动都判成不必要，也不能直接锁回Home。** 保留同一轮快照/右手目标的局部实验，仅锁住接触与接近阶段的左7轴（搬运两手目标不改），三个侧吸点和顶吸均未返回合格IK。CPU独立FK把原成功接触状态的左7轴换回Home，右TCP变换完全不变，但在当时updown=-0.715465m处，左腕与底盘的未豁免球对相交（最小球间隙约-0.244198m；原接触状态未检出球对相交）。该反事实只证明原选中升降位置不能保持左臂Home，不是全部约束下无解的证明，也不是精确网格穿透深度。

因此问题是缺乏明确的安全停放姿态和动作质量约束，不是简单“空闲臂必须完全不动”。需区分升降时的必要避让、明确carry目标要求的动作、IK冗余分支和未精简搜索绕行；不能通过取消碰撞或删除mentor目标来缩短轨迹。

局部锁Home补丁已从候选树撤回，保留在`motion233-acceptance/round12-idle-arm/rejected-home-lock.patch`。该目录包含原轨迹度量、两次实验日志（第一次SDK结果重复补全错误，第二次修正后正常返回IK失败）、输入同一性审计、CPU反事实及哈希。未进行整墙重跑，未用失败补丁替换原回放；轨迹质量问题仍未解决。

## 第12轮空载捷径修正与中文统计（2026-10-07）

在原第12轮保存轨迹上重建27/26个原始折点（逐段核对与原插值一致），不重新求IK或搜索RRT，只应用导师已有的最远可连接点规则。保持起终点、完整机器人碰撞上下文和0.5°加权采样间隔：接近段加权路径长度15.8906→11.9359（减少24.9%），回程17.5831→13.7533（减少21.8%），球模型边及密化状态检查通过。这直接证明存在可移除绕行，但不证明当前接触/搬运IK分支最优。

运行改动只有现有参数接线：`search_path(..., apply_shortcut=False)`保持携箱默认；`joint_plan`的接近/回程显式开启它。未改RRT函数、IK、目标、模型、2秒搜索预算或豁免。

- 原快照第12轮重新计算一次成功：2596帧（旧2993），接近折点27→10、回程26→6。抽离、携箱运输、携箱转身、释放、空载转身的全部关节帧与旧记录逐值相同。新总规划13.452s，旧10.850s；**减少几何绕行不等于降低规划耗时**。
- 默认双箱原快照单次回归成功：2027帧、10.693s。两例接触/35cm抽离/足迹/附着/目的箱位姿复核通过；CPU40项通过，三种RRT函数AST保持与f044一致。
- 未重跑整墙或8种子矩阵，旧25/25和8/8只属于先前版本；新版本不能直接继承那些全量回归结论。精确网格门禁仍未通过。
- Viser新增“中文阶段耗时与统计”：当前轮各规划阶段、运输子项、首解/搜索时间、原始与精简路径点、迭代/树节点/解数量、候选/种子及动作帧数；切换轮次更新，缺失字段显示未记录。明确子项不可重复累计，帧数不是执行时间。
- 8095已更新显示代码，但仍读取原`l02-diagnosis/sequence.json`，**没有将局部重算伪装成新整墙结果**。因此原第12轮显示27→27、捷径未启用是正确的历史记录；新结果在`round12-shortcut/round12-cycle.json`。
- 浏览器实际切换第12轮并检查中文字段与原记录数值，截图`round12-shortcut/viser-chinese-stage-timing.png`、`viser-chinese-search-stats.png`；`ui-check.json`记录界面核验。

证据目录：`motion233-acceptance/round12-idle-arm/shortcut-audit.json`为同轨迹对照；`motion233-acceptance/round12-shortcut/`为两次局部/默认任务回归、几何审计和UI证据。GPU已释放；只读回放不进行GPU规划。

## 逐帧离线检查收尾（2026-10-07）

规划和8种子矩阵均已结束；以下是CPU读取既有轨迹的检查，不是再次规划，也没有更换RRT或模型。

- `tools/verify_motion233_mesh.py`从第1轮开始检查，在首次检测到相交时停止；核对模型身份、关节集合、阶段/载荷标志后调用独立URDF FK与FCL。
- 已修复检查器对URDF省略collision origin的处理（单位矩阵），以及STL重复顶点导致凸体识别失败的问题（仅精确去重，不改表面坐标）。合成测试同时覆盖机器人相交/分离、重合世界箱、限位及NaN拒绝；CPU核心测试 **38/38**。
- 修正后的检查再次得到同一失败：第1轮2054帧未检出相交；第2轮approach的**零基帧3（本轮第4帧）**，`base_link:0 / left_link7:0`报告相交。累计检查2058帧后退出码1，未检查后续完整序列，不声称全路径通过。
- FCL报告的局部接触深度约26.413mm，不能等同于精确最小分离距离或实物穿透结论。非凸BVH检查不证明实体完全包含的安全性，也不是连续路径/硬件认证。
- 证据目录：`/home/astesia/Sevenova/motion233-acceptance/mesh-representation/`，其中`full-mesh-audit-verified.json/.log`是修正后结果，`cpu-tests.log`是38项测试。旧报告保留，不覆盖。
- 可视化仍使用现有8095只读整墙回放；其中“整墙通过”指基线球模型的规划结果，**不能视为此次网格验收通过**。本次无GPU规划、无模型/豁免/ROS执行树修改。

复核命令（只用CPU；预期在上述相交位置退出码1）：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
CUDA_VISIBLE_DEVICES='' \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/verify_motion233_mesh.py \
  --sequence /home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/sequence.json \
  --runtime-root /home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/runtime/research/curobo_v3 \
  --output /tmp/motion233-mesh-manual-audit.json
```

## 本轮产物

`/home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/`：
- `matrix-01/matrix-plan.json`、`matrix-status.json`、八份逐seed结果与日志；代码/模型前后哈希一致。
- `default-preflight.json`、`seeded-preflight.json`、`seed-preflight-audit.json`、`warm-repeat-phase-diff.json`。
- `mesh-checkpoints.json`（90点抽查）、`mesh-pair-recheck.json`（3对复查）及CPU-only复现脚本。
- `migration-structure-audit.json`记录RRT函数AST和ROS执行树保留情况。

## 复跑命令

新建独立输出目录，运行完整公开种子清单（脚本逐例持同一flock；已有矩阵目录拒绝覆盖）：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/run_motion233_rrt_seed_matrix.py \
  --runtime-root /home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/runtime/research/curobo_v3 \
  --sdk-repository /home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
  --output-dir /home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/matrix-manual-rerun \
  --case-timeout 600
```

若只是单例复核：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/runtime/research/curobo_v3
flock /tmp/sevenova-curobo-gpu.lock \
  env PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
  PYTORCH_ALLOC_CONF=expandable_segments:True \
  /home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_plan_cycle.py \
  --sequence \
  --wall-distance 0.75 \
  --search-seed 104729 \
  --output /tmp/motion233-seed-104729-rerun.json
```

主线基线核验器默认只要求Informed RRT；只有显式`--check-optional-planners`才检查其它规划器。可选PRM实验不会阻塞RRT迁移，也不能拿来替代原有安全/性能要求。
