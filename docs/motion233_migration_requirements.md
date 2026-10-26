# MOTION-233 迁移要求与当前证据（2026-10-07）

> **2026-10-08最终PR口径：** 已squash rebase到最新`alfa_v3_curobo@4200247`。该上游提交新增TaskCycle初版Demo但未更改071cb95机器人/碰撞模型。本PR以用户确认的当前cuRobo球模型碰撞合同作为可用验收；独立FCL/网格材料保留为非阻塞诊断，不作为本PR合并门禁或失败结论。

最终隔离runtime验证：272个候选源码/模型文件与`b0b2c2d`一致；MOTION-233完整墙25/25箱、15轮、25790帧、146.185s，无回程fallback。上游TaskCycle基线L19/R15侧吸1574帧/4.491s、L4/R0顶吸2258帧/3.159s均通过。固定SDK与基础CPU均46项通过（基础环境6项按声明依赖skip）。

## 基线和提交边界

**临时开发快照：** 用户要求先远程保存到个人fork `bestastesia/alfa_robot` 的独立分支 `motion-233-curobo-f044bf1`，不创建PR、不更新原PR分支、不合并基线。此版本仍有网格安全及其它迁移缺口，不是最终验收版本。下文“未push”记载为此前各验证节点的历史状态。

历史迁移起点为`alfa_v3_curobo@f044bf1`；提交PR前发现目标分支已前进到`4200247`，现已将最终树squash rebase到该提交并保留RRT主线。新分支为`motion-233-curobo-4200247`。

2026-10-08再次核对模型来源：`SevenovaHangzhou/robot_description` 的`feat/v3-analytic-proxy`已由冻结的`071cb95`前进到`e189bdb`，但唯一新增提交仅将四轮移动入口的`base_link`可视罩壳换回V3.2.2外观，并明确该罩壳不作为碰撞或质量；上身、碰撞网格和本任务使用的固定/mobile18碰撞语义未变。MOTION-238确认当前球拟合仍关联MECHINE-40权威碰撞体，严格精确碰撞另行验收；MECHINE-40已Done但无新附件/后续资产。故不存在可直接rebase来消除本次FCL问题的新碰撞模型提交。

原PR #43仍是`b72ba30`→`alfa_v3_dev`，Open/Conflicting。mentor的2026-09-28评论明确禁止合并旧模型/旧流程，要求重建替代PR、选择性迁移，不回退模型/planar/执行链。**因此不是强行解决冲突再重放整个旧栈。** 本轮没有push、改PR或写Linear。

## 要求映射

| 要求/来源 | 已有证据 | 当前判定 |
|---|---|---|
| 用户指定最新cuRobo基线及模型 | 远端SHA、祖先关系；生成资产路径归一化和模型/vendor逐文件核验 | 已落在指定基线；不沿用误选的alfa_v3_dev Stage实验结论 |
| mentor统一机械坐标、零位、限位、TCP，不回退右J2/J4与joint origin | 相对f044，模型、vendor、生成模型资产未修改；全墙逐帧URDF限位复核 | 已保留，面内抓取变化不是修改TCP或机械限位 |
| 不删除planar根、底盘TF、Stage Action、轨迹缓存 | `git diff f044bf1 -- ros2_ws`为空 | 源码保留；没有声称新研究入口已接入/验证实机执行 |
| 用户要求RRT主线 | 默认informed_rrt；三种已有RRT函数AST与f044一致；25箱记录全为informed_rrt | 已保留。PRM修复隔离，不是本迁移门禁 |
| 车头净距0.75m、完整搬运序列、单臂收尾及顶吸 | 923baa4一次完整实跑25/25、15轮、143.085s；263文件核验、全轨迹限位/继承及15轮接触几何复核通过 | 当前研究碰撞口径通过；旧固定中心点没有被冒报可达 |
| 原PR参数化箱墙/缺箱 | WallLayout、快照取接触点、CPU合同；历史3×4/4×3选中双箱实跑 | 接口已迁；不同尺寸仍受冻结负载模型限制，不能说原全部L4场景已迁完 |
| 原PR固定公开种子 | 原8数值逐字节保留；3d421fe每个种子独立进程、各一次完整25箱，运行副本哈希前后不变 | 当前动作修正版球模型口径8/8通过；不等于旧OMPL轨迹、位级确定性、网格或旧时延合同等价 |
| 原固定种子C/D精确轨迹对拍 | 新同请求同seed两次预检都成功，但运输阶段轨迹/冗余终点不同 | 尚未达到严格位级重现；不能用“有seed参数”冒认完成 |
| 原逐箱时延、预算、时间参数化、质量指标 | 新入口记录总时间及阶段/失败尝试；当前单轮多为8～12s，仍输出未计时关节帧 | 不能继承旧3s/5s门槛或实机时序结论；未完成 |
| 原完整FCL复核 | 923baa4新整墙逐帧网格复核，在第2轮approach零基帧210检测right_link7 / wall_box_11相交，累计2238帧后停止 | 当前网格门禁未通过；不可用球模型成功替代；未改豁免/阈值 |
| 原Golden算法与参考资产的最终取舍 | 原PR/Git对象和冻结报告仍在；新实现复用导师核心，未移植旧坐标补丁 | 不宣称旧全部算法/预算/执行接口已迁移；需按要求继续收口 |

**当前PR目标按用户确认的cuRobo合同已达到可用验收。** 旧OMPL位级等价、实机时间参数化及独立FCL网格材料不属于本PR合并门禁，作为后续方向保留。

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
- 局部检查后已补一次完整整墙回归（见下节）；其后的单箱动作修正版又完成当前8公开种子矩阵（见专项章节）。精确网格门禁仍未通过。
- Viser新增“中文阶段耗时与统计”：当前轮各规划阶段、运输子项、首解/搜索时间、原始与精简路径点、迭代/树节点/解数量、候选/种子及动作帧数；切换轮次更新，缺失字段显示未记录。明确子项不可重复累计，帧数不是执行时间。
- 中文面板首次部署时保留原`l02-diagnosis/sequence.json`，没有把局部重算混入旧序列；随后在真实整墙回归结束后切换到新序列（见下节）。`round12-cycle.json`仍保留为独立单轮对照。
- 浏览器实际切换第12轮并检查中文字段与原记录数值，截图`round12-shortcut/viser-chinese-stage-timing.png`、`viser-chinese-search-stats.png`；`ui-check.json`记录界面核验。

证据目录：`motion233-acceptance/round12-idle-arm/shortcut-audit.json`为同轨迹对照；`motion233-acceptance/round12-shortcut/`为两次局部/默认任务回归、几何审计和UI证据。GPU已释放；只读回放不进行GPU规划。

## 单箱乱甩专项修正（2026-10-08，代码711a804，尚未推送）

按用户要求停止无关支线，专项处理单箱空闲臂大幅摆动。根因不是单一shortcut：单箱主动TCP约束下空闲臂存在IK nullspace；旧代码又保留接触候选批次优先、空闲臂与作业臂同权搜索，并允许肩J1选择跨半圈分支。

当前实现不固定空闲臂Home，也不删除整机碰撞几何：

- 每个主动手完整6D、整机碰撞合法的原始IK状态仍进入候选集。
- 额外为同一状态生成解析空闲臂候选，尝试保持初始水平位置/朝向，仅调整世界高度和swivel；全部复用现有解析IK与GPU碰撞检查。
- 原IK nullspace和解析补偿候选统一按空闲臂关节运动最小选择；解析失败不再导致主动抓取不可达。
- 单箱全候选统一排序；空闲臂搜索权重提高到3，升降轴仍为5，作业臂为1。单箱接近/回程和携箱均复用原有碰撞密检shortcut。
- 双肩J1相对本轮初始位置峰值≤180°，约束接触/搬运IK、RRT搜索边界和最终全轨迹。物理URDF限位、RRT函数、2秒搜索预算、碰撞政策、抓取目标和双手carry目标均未改；双箱的候选顺序、权重和携箱shortcut策略保持原值。

失败对照均保留：仅全局排序仍大摆；锁Home发生底盘碰撞；固定空闲TCP高度使末箱无解；直接放开Z仍会选大转角；固定swivel解析补偿使末箱无解；其它第3轮可抽离候选总行程更差。这些失败没有被覆盖或用于成功报告。

### 一次完整整墙回归

共享GPU锁下只执行一次当前版本整墙规划，结果为 **25/25箱、15/15轮、25790帧、144.362s**，无反向回程复用。相对`round12-shortcut/sequence.json`（27498帧、143.085s）：

| 单箱轮次 | 任务 | 旧帧数→新帧数 | 双臂累计关节行程变化 | 新肩J1峰值（左/右） |
|---:|---|---:|---:|---:|
| 3 | L22 | 2153→1986 | −10.8% | 25.7° / 129.9° |
| 6 | R17 | 2235→2067 | −15.9% | 36.1° / 129.3° |
| 9 | L12 | 1732→1386 | −44.8% | 106.5° / 7.7° |
| 12 | R07 | 2576→1546 | −68.9% | 25.0° / 111.0° |
| 15 | L02 | 2008→2011 | −2.9% | 128.2° / 153.8° |

整墙总帧数27498→25790；规划计算143.085→144.362s，**不宣称计算更快或硬件执行更快**。全序列263文件、限位、逐轮继承和15轮接触/35cm抽离/附着/箱体目的位姿复核通过；固定SDK测试42/42，基础环境测试42项中6项按声明依赖跳过，默认双箱回归通过。

独立网格检查的首个失败仍在第2轮approach零基帧210：`right_link7 / wall_box_11`。随后已对当前默认完整序列做全量枚举：25790帧中2377帧报告未豁免FCL接触，共14类碰撞对；主要为base_link与link5/link7、right_link4/right_link7，以及少量邻箱/前墙。

阶段端点270状态对照显示，动作修正前有12个失败，修正后有22个；新增12个均为单箱第6/12轮空闲左腕与底盘，证明动作改善在精确网格口径下产生安全回退。实验性“每个主动手解选择最省动作且FCL安全空闲臂”可把五个单箱90个阶段端点降为0失败，但五条完整单箱路径8641帧仍有705个中间接触；15个阶段直接边只有7个同时通过球/FCL。因此端点筛选和shortcut都不是完整修复，该实验补丁已撤出候选源码，仅保留证据。

导师冻结模型README明确260球/18链接代理仍要求执行前精确网格与箱体OBB验证；`HANDOFF.md`与`core_migration_276/validation.json`也明确当前没有FCL/OBB搜索门禁。当前不能在未经mentor确认时扩大忽略对、缩放网格、修改冻结球表或把球模型25/25冒认为安全。最终安全收口需要选择并评审：更新冻结碰撞代理，或实现网格感知搜索/路径修复；现有候选按本PR的cuRobo研究合同可用；独立FCL材料不扩展为本次合并门禁。

旧黄金链也已复核：`e8f0abc`的“full FCL”是在规划完成后逐边加密重验，失败即清空选中结果，不产生修复路径；`0faac41`/`6d71aa4`的TCP swivel beam只沿给定笛卡尔接近/撤退目标序列做多分支IK和FCL验边，不负责自由空间RRT approach/transport/home。当前2377个接触帧大量位于这些自由空间阶段，故两者不能直接cherry-pick解决；选择性迁移已有能力仅能提供fail-closed终检和笛卡尔段候选搜索。

2026-10-08已在Linear MOTION-233评论`c61fbb07-4486-4726-945a-0e4e70cc66e6`正式请求mentor明确责任边界。用户随后明确当前cuRobo合同通过即视为可用，并授权提交PR；因此不在本PR引入碰撞代理、豁免或网格规划策略变更。

### 可视化与证据

8097读取`motion233-acceptance/single-arm-motion/sequence.json`，只读回放新完整整墙；浏览器实际检查第3/12/15轮吸附帧和第12轮中文动作约束。主要证据：

- `sequence.json`、`sequence.audit.json/.log`、`sequence-geometry/summary.json`。
- `wall-motion-audit.json`及五个单箱`roundNN-combined-idle.json/.audit.json`。
- `sequence-mesh-audit.json/.log`保留未通过结果。
- `full-wall-ui-check.json`、`viser-round{03,12,15}-contact.png`、`viser-round12-motion-policy.png`。
- 所有中间方案、失败日志和补丁保留在同目录，最终采用方案名为`combined-idle`。
- `all-mesh-contacts.json`、`mesh-full-summary.json`、`mesh-phase-boundaries*.json`记录全帧/端点差异；`per-active-mesh-path-incomplete.patch`是明确未采用的端点筛选实验。

### 当前8公开种子矩阵

代码`3d421fe`使用原PR清单`104729, 104743, 104759, 104761, 104773, 104779, 104789, 104801`，每个种子独立进程、固定顺序、各运行一次；没有失败重跑或挑样。结果为**8/8均25/25箱、15轮**，规划计算范围143.775～144.806s，帧数25714～25909；运行副本源码/模型哈希前后不变。逐种子完整序列/限位/继承审计8/8通过，五个单箱轮次×8种子的40项接触/抽离/附着/目的位姿复核通过。

相对动作修正前的默认整墙记录，公开种子下单箱关节行程变化范围：R3 −10.7%～−9.7%，R6 −16.1%～−15.0%，R9固定−44.8%，R12固定−68.9%，R15 −4.6%～+1.8%。所有种子的肩J1峰值≤153.8°。因此可以确认大幅跨肩乱甩被约束，但不能声称每个种子的总关节行程都严格缩短；R15仍存在最多1.8%的轻微总行程波动。

证据：`motion233-acceptance/rrt-seed-current-3d421fe/`中的`matrix-01/matrix-status.json`、8份`.audit.json/.log`、`motion-matrix-audit.json`、`single-geometry/summary.json`、`acceptance-summary.json`及`SHA256SUMS`。这些仍是GPU球模型政策，不重启旧3s/5s、位级等价、全网格或实机安全结论。

回放命令：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/single-arm-motion/runtime/research/curobo_v3
CUDA_VISIBLE_DEVICES='' \
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_full_cycle_demo.py \
  --port 8097 \
  --wall-distance 0.75 \
  --sequence \
  --sequence-json /home/astesia/Sevenova/motion233-acceptance/single-arm-motion/sequence.json \
  --replay-only
```

## 当前完整回归与回放（2026-10-07，代码923baa4）

在共享GPU锁下只执行一次整墙规划，上限300秒；实际**143.085秒、25/25箱、15/15轮、27498帧**，没有反向回程回退。不是拼接局部结果。源码/模型263文件核验、18轴限位、逐轮场景/状态继承，以及全部15轮接触足迹/35cm抽离/附着/箱体目的位姿复核通过。

本次整墙中的第12轮为2576帧（旧2993）；接近路径点27→10、回程27→6，携箱仍未启用捷径。之前单独重算的2596帧保留在独立文件，不能与这次整墙记录混淆。搜索仍有壁钟预算影响，不承诺位级重现。

**独立网格复核未通过：**第1轮2027帧未检出相交，第2轮approach零基帧210（本轮第211帧、整墙第2238帧）检测到`robot:right_link7:0 / world:wall_box_11`相交，FCL局部接触深度约0.595mm；此处停止，未宣称后续网格全部检查或安全通过。

8095现在读取`round12-shortcut/sequence.json`，已真实切换第12轮检查中文计时、27→10和捷径启用等显示。界面仅只读回放，GPU已释放；“规划完成”不是硬件安全认证。

证据均在`/home/astesia/Sevenova/motion233-acceptance/round12-shortcut/`：
- `sequence.json`、`sequence-run.log`、`sequence.audit.json`、`sequence-audit.log`。
- `sequence-geometry/summary.json`及15份逐轮几何报告。
- `sequence-mesh-audit.json/.log`保留未通过结果。
- `latest-wall-ui-check.json`、`viser-latest-wall-stage-timing.png`、`viser-latest-wall-search-stats.png`。

当前回放启动命令（完整可复制，不再使用旧模型路径覆盖）：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/round12-shortcut/runtime/research/curobo_v3
CUDA_VISIBLE_DEVICES='' \
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_full_cycle_demo.py \
  --port 8095 \
  --wall-distance 0.75 \
  --sequence \
  --sequence-json /home/astesia/Sevenova/motion233-acceptance/round12-shortcut/sequence.json \
  --replay-only
```

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
