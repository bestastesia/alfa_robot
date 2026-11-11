# 新WARP迁移交接——给运控工程师1.1

## 1.1迁移完成补充（2026-10-27）

- 功能分支`feature/new-warp-sync`；8210入口`v3_forward_precontact_demo.py`默认使用`TimedNewWarpPlanner`，六阶段均由`NewWarpPlanner/PolarRefinedCyclePlanner`执行新WARP。8221研究服务保留，未更换其场景或重启。
- 运行时参考、解析先验、联合代价已从研究demo/benchmark抽到`tools/curobo_core/new_warp_reference.py`、`new_warp_prior.py`、`new_warp_objective.py`；13个函数/类AST与交接源码完全一致，记录于`generated/new_warp_sync/extraction_identity.json`。
- 放箱权威配置为`config/new_warp.json`的绝对箱心高度0.982m，构造器不再叠加减10cm。1.082m可显式用于历史基线复现。保存结果必须匹配策略/高度/模型，不能把旧回放当成当前配置验收。
- 本轮独立复测：1.082m纯几何117/117、702阶段，105个阶段使用软参考；平均3.892秒。0.982m侧吸L22/R17、顶吸L03/R02、单臂R02纯几何3/3和后处理3/3通过，不能宣称当前高度117/117。
- 57项core测试（1项历史资产测试跳过）、17项模型/流程/回放测试、交接所列6项研究回归通过。失败参考仍进入RRT、无合法先验仍正常采样、共享升降固定而RRT15轴自由、端点和最终0.25°验收均保留。
- 8210增加PR65后处理独立开关，默认保留用户已启用的后处理；关掉可单独观察新WARP几何结果，切换后必须重算。后处理补边检查也使用0.25°，不更改原抽离途中空载/终点携箱的验碰边界。算法计算耗时与名义运动时长分开保存。
- 新复测入口`tools/v3_new_warp_validate.py`，须指定`--output`独立目录，`--height 1.082 --all-tasks`复现历史基线，默认0.982m测三个代表任务，`--timed`追加后处理验证。三组summary/protocol在`generated/new_warp_sync/{baseline1082,current0982,timed0982}/`；大轨迹留本地，不混入普通源码提交。
- 未改模型、Home、碰撞球、底盘cuboid或实机接口；随机停车/箱体扰动不在本轮验收内。下文描述原研究交接时点及背景，以上迁移记录优先。

更新：2026-10-27。用户已指定：今后本轮工作的“WARP／新WARP”指本文方案。WARP是用户指定的策略名，不自行扩写英文缩写。

**新WARP = 球面变杆长软参考 → 128点固定升降＋平滑冗余角解析 → 整段30轮联合调整 → 合法先验引导Informed＋Connect → 最终路径验收。**

当前可运行基线是 [8221页面](http://localhost:8221/) 与 `PolarRefinedCyclePlanner`。迁移目标是把该行为纳入1.1负责的开发分支/核心接口，保留已有工程化成果；不是重新实现旧的“并行LM＋逐点32轮polish”。算法命名基线已确定，生产接入和实机验收仍须由对应工程流程完成。

## 1. 先确认这两件事

1. **当前释放箱心高度是0.982m。** 用户刚要求把龙头车放箱位置再降低10cm。已有117/117证据对应此前1.082m；降低后按用户要求未跑全流程，不能拿旧结果证明新高度已通过。
2. **参考路线不是最终执行路线。** 参考可碰撞、可不可达、可缺点。不能因为参考没通过碰撞检查就停止，必须让后续调整和RRT有机会处理。碰撞构型不能作为合法节点直接采用，最终输出路径仍严格验碰/限位/端点。

这两条分别是最新参数变更和最重要的迁移回归条件。

## 2. 工作区、代码版本与移交清单

实际仓库：`/mnt/mydisk/ALFA/alfa_robot_v3`，研究根目录：`/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3`。不是聊天默认cwd `/mnt/mydisk/ALFA/alfa_robot`，也不要恢复历史 `alfa_robot_v3_curobo` 工作树。

本次核对分支 `feature/curobo-8210-timed`，HEAD `f8463abf6248b0734167b77cc97282bd02264a5d`。**HEAD不包含全部新WARP实现**；工作树同时有其他工程师的模型/后处理修改，不能整包提交所有dirty文件。

仓库设置 `status.showUntrackedFiles=no`。普通 `git status`会漏掉未跟踪的新文件；本次静态入口依赖共77个本地Python文件，其中11个未跟踪。详细路径、SHA256、跟踪状态、核心资产和已安装包版本见 [NEW_WARP_SOURCE_MANIFEST.json](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/NEW_WARP_SOURCE_MANIFEST.json)。这是静态导入闭包，含可选导入，不是完整运行资产闭包或Git提交清单。

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
git status --short --untracked-files=all
```

先对照清单保存当前文件，再迁移。不要只checkout上述HEAD、只复制两个入口、自动reset/clean，或把其他工程师的未提交工作覆盖掉。此交接没有创建PR、提交、切分支或给其他AI发送消息。

## 3. 与旧WARP的关键差异

|部分|新WARP当前行为|
|---|---|
|笛卡尔引导|共享升降＋肩腕方向球面运动＋平滑变杆长，69.3334cm软偏好|
|路径IK先验|从真实起终构型求冗余角，沿128点平滑变化，每臂每点一次解析；不使用LM生成中间先验|
|构型调整|全部中间控制点联合优化30轮，直接包含相邻关节变化；不是每个IK点独立polish32轮|
|参考安全性|择优参考时考虑碰撞，但无安全参考也继续；不能拦截RRT|
|先验缺口|可作为优化初值插值，但插值未验收；最终只注入合法点。没有合法先验时RRT仍正常采样|
|RRT|同一真实起终点，Informed＋Connect，2秒上限，首个细验合法解即停|
|最终验收|搜索内细验后再做0.25°加密复核、真实端点Pose验收|
|放箱目标|当前箱心Z=0.982m；身体范围内的运输收束目标不是这个参数|
|后处理|本入口没有搜索后Shortcut、TrajOpt或执行时间参数化；1.1已有相关工作保持独立|

“路径先验不运行LM”不等于项目彻底删除cuRobo IK：双臂龙头车等阶段的**终点构型求解**仍可能使用cuRobo数值IK。源码内部兼容容器键 `methods['lm']` 也不代表执行了LM。

## 4. 全流程及阶段衔接

|阶段键|动作与端点来源|本轮路径策略|
|---|---|---|
|`approach`|从吸附6D Pose向外退3cm得到预接触Pose，参考Home独立求IK；Home→预接触|新WARP|
|`contact_approach`|实际到达预接触后，参考到达构型求吸附IK；正向接近|新WARP|
|`extract`|参考实际吸附构型选35cm后撤终点；不是回头改选吸附构型|新WARP|
|`transport`|参考实际抽离结果，从运输库选合法收束端点，检查含空闲臂和箱体的1.05m包络|新WARP|
|`conveyor_place`|接收导航到位状态，参考运输构型选龙头车放箱目标IK|新WARP|
|`return_home`|释放后从实际放箱构型回Home|新WARP|

保持前驱串行选型：固定本段端点后连接，不通过试下一段来反选本段终点。每段成功后把**实际输出的终点关节态**交给下一段；检查接缝连续性。六段都执行30轮，没有按任务挑10/60轮。

单臂仍在共享Updown＋14臂关节空间规划，空闲臂并非固定；保持既有空闲臂/携箱旋转软代价。路径先验生成时两侧TCP都可有参考，只有活动臂附着箱子。

## 5. 核心算法说明

### 5.1 球面变杆长软参考

每侧腕点 `W(s)=S(u(s))+r(s)d(s)`。肩点由当前URDF的肩部轴关系获得，两肩共享升降；腕点使用真实末三轴交点。`r*=sqrt(0.506²+0.474²)=0.693333967m`来自模型几何，不是关节硬约束。

- 升降使用起终值之间单调五次进度；两个肩点同时移动。
- 肩腕方向沿球面平滑转动；首尾保持真实TCP Pose。TCP朝向独立沿原旋转进度变化，不要求速度沿TCP＋Z。
- 杆长在前35%平滑靠近中间目标、35%～65%保持中间长度、末35%平滑过渡到终值。允许减弱靠近69.33cm的程度。
- 当前候选：收缩强度`1/.75/.5/0`，球面侧偏`0/±0.2rad`，共享升降进度比`1/.75/1.333333333`。
- 优先沿用通过末端/箱球外部连续验碰的候选；若不存在，按路线/距离/转向与碰撞穿入软评分选择**仅供引导**的参考。`accepted=False`和`guidance_available=True`可以同时成立。
- `reference_collision_free`只描述参考；最终任务成功不能由它决定。`reference_used_soft_fallback=True`也不应变成失败。

这一层只用末端/携箱球组与外部cuboid；不会把整个机器人简化成球后当成执行安全证明。也没有实现“全机器人动力学最优”或严格半球可达域证明。

### 5.2 128点解析先验

同一归一化曲线进度取128个TCP Pose和Updown，不能独立重采样后错配。由真实起终关节态反求左右冗余角，采用最短有符号圆周角差＋五次平滑插值。每点两次单臂解析，共256次。

固定Updown和ψ后仍可能有离散解析分支，每臂本批最多8个，双臂组合最多64/点。保留分支后做整机碰撞、限位及FK回代；位置2mm、姿态1°用于IK候选/端点，不是整个路径偏离参考的硬门槛。

从合法候选中选距起终参考最近的构型；缺层插值仅用作联合优化初值，绝不能称为合法运动。手臂基座变换由真实URDF求得，固定模型的共享升降可用仿射平移批量构造，避免128次完整FK。

### 5.3 整段联合30轮

32个三次样条控制点，首尾为真实关节态，内部控制点最小二乘初始化。14个手臂关节一起调整；**Updown保持参考采样值，冗余角此时可以变化**。Adam步长0.018，梯度裁剪30，统一30轮。

代价包含关节路程、二阶差分、较大相邻关节变化、TCP位置/朝向参考、腕限位余量、累计朝向变化、世界/自碰撞/底盘cuboid/地面。具体权重以 `CorridorObjective.terms()` 为准，避免迁移时凭文档抄漏公式。

样条0轮本身就能摊开跳变，不能把这一收益全归于30轮迭代。联合调整允许TCP偏移，曾出现15cm甚至更大偏移；因此它是构造RRT先验，不是直接执行或替代RRT。

### 5.4 引导RRT和最终验收

调整后的每点再做整机碰撞和限位筛选，只注入合法关节态。RRT可调整全部15轴，**不被参考Updown或ψ曲线硬锁住**。

`RouteGuidance`按参考层均衡抽样，当前约90%采样走先验及其扰动，其余保留原搜索采样；已有端点/关节距离与TCP软代价保留，单臂/携箱阶段原有代价继续相加。先验稀疏或断开不拒绝任务。

每段2秒上限、首条细验合法路径即停；搜索内约0.5°密化检查，输出前0.25°加密复核和固定端点Pose验证。Updown代价权重2.617994rad/m（10cm等价15°）；碰撞密化使用的Updown尺度还可能至少为5，不能把二者混为一个参数。

内部底盘粗搜索余量2mm；龙头车阶段继承外部场景3mm搜索余量和既有粗边间隔。最终检查原几何，**本轮没有统一删除或降低这些余量**。

## 6. 模型、参数与坐标

|项目|当前值/来源|
|---|---|
|坐标版本|`dual-zero-discrete-only-tool191-20261024`|
|TCP|J7＋Z191mm；不能混用旧151mm或实测杆70mm|
|Home Updown|−0.3m|
|Home左J1～J7|`[65,-75,20,-90,0,-40,90]°`|
|Home右J1～J7|`[-65,75,-20,90,0,40,-90]°`|
|新零位适配|`RezeroAnalytic`负责旧解析库到当前坐标转换；不要重复补偿或绕开它|
|身体范围内运输目标|侧吸按当前箱心1.2m/Y45°策略及新TCP修正；顶吸TCP高1.2m。以运行时变换为准，不从旧配置抄未经校正的四元数|
|运输包络|1.05m，以当前footprint中轴为参考，含空闲臂与携箱|
|龙头车释放目标|**箱心**Z=0.982m，不是TCP Z；用工具到箱体附着关系生成TCP目标|

`adaptive_raised_cycle.json`仍保存基础高度1.082m；新入口在初始化时通过`conveyor_release_drop_m=.10`降低。迁移成正式配置时必须选一个权威来源，避免配置先减10cm、构造器又减10cm。推荐工程化后统一为显式绝对箱心高度，同时保留1.082m复现实验配置。

整体输入输出仍遵守world/map场景语义，阶段内部可以在base_footprint计算。必须明确`T_world_footprint`及附着变换，不能把世界场景或停车Yaw忘掉。既有标准场景后撤方向等价world−X；对接非零停车Yaw时应把world−X变换到阶段坐标，而不是直接沿工具−Z或车辆−X。当前117证据不含随机停车/箱体扰动，不能继承旧8171扰动实验的验收结论。

## 7. 代码地图与依赖迁移

以下路径均以研究根目录为基准，完整绝对路径与hash见源清单。

|文件|迁移职责|
|---|---|
|`tools/v3_polar_refined_cycle.py`|主集成类；六阶段路径入口、参考/解析/30轮/RRT/细验、释放高度、预热|
|`tools/v3_polar_refined_cycle_demo.py`|8221入口，动态策略名称、六段播放和分项耗时|
|`tools/v3_polar_wrist_compare.py`|`PolarCurve`、`polar_plan`、软参考分支；不能只复制旧strict模式|
|`tools/v3_shoulder_wrist_reference.py`、`v3_spherical_wrist_route.py`|真实肩腕几何和姿态曲线工具|
|`tools/v3_smooth_swivel_benchmark.py`|运行时借用`swivel_schedule`、`enumerate_smooth`，虽然文件名叫benchmark|
|`tools/v3_swivel_joint_refine.py`|运行时借用`nearest_chain`、`FixedLiftObjective`，不是只用于离线报告|
|`tools/v3_joint_corridor_optimize.py`|联合代价、样条、原生碰撞梯度、轨迹指标|
|`tools/v3_route_guided_compare.py`|`RouteGuidance`、`TcpRouteCost`|
|`tools/v3_multiroot_search.py`、`v3_batched_loaded_search.py`|RRT和完整验碰/密化，不能误开启Shortcut|
|`tools/v3_current_urdf_forward.py`、`v3_forward_precontact_cycle.py`、`v3_lm_chained_cycle.py`、`v3_serial_endpoint_planner.py`|继承的阶段编排、独立端点选择、当前URDF/运输库适配；旧类名不表示新路径仍运行LM|
|`tools/v3_endpoint_guided_ik.py`、`v3_lm_parameter_engine.py`|当前仍导入`pose_filter`/`clock`等工具，不可因名称含LM直接删文件|
|`tools/v3_visualized_task_demo.py`|仅增加可选策略/预热说明及实际放箱高度显示，兼容其他入口|

建议先保留完整导入闭包跑通，再把核心函数迁到稳定模块，例如几何参考、解析先验、联合调整、引导搜索四个模块。不要让正式核心长期依赖`*_demo`/`*_benchmark`及Viser；但本轮不要为清文件名而漏掉运行依赖。

关键资产包括当前零位URDF/fixed/mobile/named_poses、零位manifest、Home配置、箱球模型、底盘cuboid定义、解析原生`.so`、运输库源NPZ及按当前TCP重建的缓存。`CartesianComparison`在Rezero有`solve_grid`时不使用旧笛卡尔库；不要默认把旧坐标批处理库接回来。模型网格和前端资产需按URDF/现有渲染器依赖解析，源清单未穷举所有可视化网格。

## 8. 预热、缓存和性能口径

当前`warmup_resources()`真实运行128/512构型碰撞查询及一次30轮联合调整，不缓存规划答案。它不是“所有场景/所有负载都完全预热”；负载拓扑变化仍可能准备新资源，联合目标也仍逐段创建。沿用既有检查器常驻更新策略，逐次生成候选与路径。

原完整LM链路4任务单次热态均值6.425s。新软参考链路在**原释放高度1.082m**下117任务各一次：

|指标|结果|
|---|---:|
|完整成功|117/117|
|完成运动阶段|702|
|使用未获连续无碰撞认证的软参考且最终通过的阶段|105|
|完整任务平均/中位|4.007s / 3.934s|
|最短/最长|2.889s / 6.699s|

这是标准场景固定种子的一轮，不是跨随机种子的可靠性上界；4任务旧样本与117任务新样本也不是严格配对性能试验。当前0.982m目标未批测。

保留额外30轮是用户明确选择：配对RRT实验显示部分路线质量改善，但30轮总开销未被搜索提速完全抵消。不要在迁移中擅自删除或改成任务特定轮数。分段历史与配对数据见 [REFINED_GUIDANCE_RRT_COMPARE.md](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/REFINED_GUIDANCE_RRT_COMPARE.md)。

## 9. 必须保留的验收边界

- Home→预接触包含目标箱碰撞；最后接近阶段排除目标箱。
- **抽离途中仍按空载模型检查，终点才追加携箱检查**。回放显示携箱不等于沿途按携箱验过；不得声称已解决完整携箱抽离安全。
- 运输/放箱使用相应负载，运输终点包络包含空闲臂。
- 导航到位/返回是状态瞬移，不是底盘导航轨迹；底盘对环境专项检查、转身检查不由本轮新增承担。
- 释放后箱体移出当前研究场景；尚非释放物体持续参与环境的真实闭环。
- 全部为本地规划/回放，无实机指令；没有在本入口接上1.1的时间参数化或执行接口。

失败必须区分：参考诊断、终点无解、RRT未找到粗连接、已有连接但细验未通过、资源/程序异常。前者不再是阻断条件；后几种按实际硬验收停止，不能用改场景/换端点掩盖。

## 10. 建议分3个PR迁移

|PR|范围|验收重点|
|---|---|---|
|1：新WARP核心策略|迁入/抽取球面软参考、平滑ψ解析、整段30轮、合法先验RRT和参数对象；接入现有FK/碰撞接口|参考碰撞可继续且不标为安全；原安全参考不变；首尾固定；共享升降一致；最终硬验收保留|
|2：六段流程与目标配置|接到当前开发分支的端点串行流程、单臂/双臂、当前URDF/Home、运行时世界变换和0.982m释放配置；新增策略选择，兼容既有入口|六阶段全部走新策略且30轮；接缝/最终Home；释放高度只减一次；失败无隐蔽旧LM回退|
|3：资源与验证交付|迁入必要资产/依赖声明、预热和计时、独立回放/报告与回归；把benchmark/demo的运行时依赖清理成核心模块|先复现1.082m的117任务，再单独验0.982m；结果和参数/代码/模型身份绑定，重启可复现|

1.1现有Shortcut/平滑/时间参数化建议作为后续独立集成PR，先确认接口再接，不把其性能/成功证据混入本文的裸几何规划。不要在同一个PR顺便修改碰撞模型、Home、零位或抽离验碰口径。

## 11. 运行、测试和证据入口

Python环境：`/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python`。本机已核对torch2.8.0、numpy2.2.6、scipy1.15.3、viser1.1.1、yourdfpy0.0.60、warp-lang1.17.0。cuRobo使用当前环境API及若干内部cost类，不能仅凭包名假定任意版本兼容；版本无法由`curobo`分发名查得，部署时核对实际安装源。

当前8221常驻服务：`alfa-polar-refined-cycle-8221`。运行参数不带旧高度`--result`，避免旧回放冒充新目标结果。不要在服务占用8221时启动同端口副本。

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
systemctl --user status alfa-polar-refined-cycle-8221 --no-pager
```

独立迁移预览可用空闲8223端口（本交接不自动启动）：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
PYTHONDONTWRITEBYTECODE=1 \
PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -u \
  /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/tools/v3_polar_refined_cycle_demo.py \
  --port 8223
```

聚焦回归：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3
PYTHONDONTWRITEBYTECODE=1 \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pytest \
  tests/test_polar_soft_reference.py \
  tests/test_polar_wrist_compare.py \
  tests/test_lift_wrist_lab.py -q
```

最近6项通过。迁移后还应运行目标分支的核心/当前URDF/碰撞/接口测试，不以这6项替代整机回归。部分研究测试读取生成样例，迁移时需带fixture或改成等价冻结fixture。

当前高度的全量测试命令（**本交接不执行**，用户此前要求先自行选择测试）：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
PYTHONDONTWRITEBYTECODE=1 \
PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -u \
  /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/tools/v3_polar_full_batch.py \
  --output /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/new_warp_release0982_validation
```

复现旧1.082m验证时，请在独立测试Planner显式设`conveyor_release_drop_m=0.0`，不要回改运行中的8221或覆盖旧结果目录。后续正式配置应直接记录绝对高度与策略版本。

证据按优先级读取：

1. [修正门槛后117任务报告](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/polar_soft_reference_all117_20261027/REPORT.md)、同目录`comparison.json`、`protocol.json`、`000.json`～`116.json`。这是新WARP验证主证据，释放高度1.082m。
2. [当前六阶段实现说明](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/POLAR_REFINED_FULL_CYCLE.md)，以及`generated/polar_refined_full_cycle_20261027/live/runs/`，每次页面计算存档；检查记录的`conveyor_release_adjustment`，不要混用高度。
3. [整段联合调整实验](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/SWIVEL_JOINT_REFINE.md)、[平滑冗余角采样实验](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/SMOOTH_SWIVEL_BENCHMARK.md)，用于理解构型多分支、偏移及耗时，不是全流程最新验收。
4. [旧门槛35/117报告](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/polar_refined_all117_20261027/REPORT.md)，仅作负例，防止重新引入参考安全门槛。
5. [8209旧交接](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/HANDOFF_8209_ALGORITHM_AND_WARMUP.md)，只补背景/底层继承；其中LM路径、旧分支信息不覆盖本文。

## 12. 交付检查清单

- 文档和UI用“新WARP”，策略可追溯到当前代码/模型/目标高度，不依赖临时端口名辨认算法。
- 工作树未跟踪文件、生成模型、解析库和运输库依赖完整；不存在只合入口却缺依赖的情况。
- 参考无碰撞不再是RRT前提；最终安全检查保留；无引导点仍能启动搜索。
- 每段30轮，解析采样无LM；阶段终点数值IK允许存在；固定ψ不误认为只有一个离散解。
- 0.982m是箱心高度；旧117/117仅用于1.082m基线；当前目标单独验收。
- 保留1.1既有工程接口与后处理成果，但分别验证，不自动混入本次性能结论。
- 新问题先明确报告阶段/原因；不擅自放宽碰撞、改Home、换目标或回溯上游。

下一位工程师若继续整理交接，可用handoff技能；具体故障再选择诊断流程。本次只准备交接材料，不自动联系1.1、不更新Linear、不创建PR。
