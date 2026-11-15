# 当前部署入口（2026-11-10）

本文件下方为历史研究交接。当前PR80＋LM微调链路、模型/碰撞口径、冻结Fast依赖、
完整实时计算/可视化命令以[FAST_DEPLOYMENT.md](FAST_DEPLOYMENT.md)为准，
不要照旧解析抽离、旧TCP、旧cuRobo或旧成功率部署。

# V3 cuRobo 新方案研究与工程交接
日期：2026-10-04。读者：后续运控同事、算法工程师及接手 AI。
目的：接手已实现的方案，知道应学什么、在哪里修改、哪些结果可信，以及下一步如何验证。逐次探索记录不重复抄写，统一引用 Linear 评论和版本化证据。

后续更新（MOTION-276）：已从交接基线迁出独立状态/场景和无GUI规划核心，正式工作树
首组基线与封装后冷/热态完整周期已复跑通过；最新入口、边界和证据见[CORE.md](CORE.md)。
本文其余实验数字与原路径保留为交接时的历史证据。

## 1. 接手时先建立正确认识

当前成果是研究级混合规划原型：cuRobo负责GPU批量数值IK、FK及球碰撞；自研Batched RRT负责自由空间路径搜索；既有C++解析IK负责固定高度的35cm直线抽离；Viser负责实时可视化。

它已经展示从第一初始姿态出发的完整抓放回位过程。已有证据仅确认当前校正模型首组L24/R20多次成功，没有当前模型11组或60组合全部成功的结论。旧模型11/11不能直接沿用。

这里的“实时计算”指用户选任务后在线规划、全部通过再播放，不是对真实机器人做闭环实时控制，也不是预录轨迹冒充在线结果。三段搜索各使用约2秒主动选优预算；完整规划热态约6.44～6.52秒。

cuRobo在本任务中最有价值的是GPU并行算子。TrajOpt仍可研究，但当前完整流程未接TrajOpt二次优化或Shortcut。不能把本方案说成官方MotionPlanner、全GPU RRT、RRT*，或一次优化整个任务的端到端规划器。

## 2. 公司任务、研究归属与交接位置

- [MOTION-237](https://linear.app/sevenova/issue/MOTION-237)：三代机适配与基本功能Demo验证。公司节点是初步适配；开发完成待验证，In Review。不要再把后续所有算法成熟度塞进它。
- [MOTION-238](https://linear.app/sevenova/issue/MOTION-238)：机器人/环境模型；[239](https://linear.app/sevenova/issue/MOTION-239)：基础IK；[240](https://linear.app/sevenova/issue/MOTION-240)：附着箱；[241](https://linear.app/sevenova/issue/MOTION-241)：抽离；[242](https://linear.app/sevenova/issue/MOTION-242)：全流程比较；[243](https://linear.app/sevenova/issue/MOTION-243)：官方MotionPlanner能力边界。均已按用户授权Done，保留失败结论与后续边界，不表示所有初始优化设想实现。
- [MOTION-275](https://linear.app/sevenova/issue/MOTION-275)：独立的具体GPU搜索算法开发，In Progress。已按关键调整补齐评论；后续每个独立算法方向按实际工作建具体issue，不建立空泛大父任务。
- 正式分支：[alfa_v3_curobo](https://github.com/kkozia188/alfa_robot/tree/alfa_v3_curobo)，来自alfa_v3_dev基线d9c330ce。本文编写前已推送基线提交9bdd50fd。
- 当前工作树：/mnt/mydisk/ALFA/alfa_robot_v3_curobo；研究模块：research/curobo_v3。
- 原始研究目录：/mnt/mydisk/ALFA/curobo_v2_ws，保存更多旧数据和第三方源码。它不是完整Git仓库，不要继续只改这里而忘记同步正式分支。

阅读顺序：本文件→[模块README](README.md)→275最新结论→对应实验issue→当前入口代码。Git仓库开工先读根AGENTS.md及.ai_teamwork/START.md。原工作树另有同事未提交改动，使用本独立工作树，别混进本次提交。

## 3. 已走过的路线：应该保留哪些结论

完整演进与实验数字见275的“补录275-01～13”和238～243的补录/验收评论。这里保留影响方案选择的结论：

| 探索 | 有效结论 | 接手时不能作出的推断 |
|---|---|---|
| 官方MotionPlanner：IK→多种子TrajOpt→Graph回退 | 官方单臂Demo可成功，复杂双负载窄通道旧试验受起点、搜索连接与碰撞近似影响 | 官方单臂毫秒级演示不等于当前任务必然毫秒级成功；超时不证明不存在路径 |
| 已有轨迹作初值的整体变形 | 可用于局部优化、代价验证和回归 | 不能宣称无参考端到端全局规划成功 |
| 官方MPC | 短视野、上一周期热启动的跟踪/局部调整能力 | 不承担从零发现整段绕障拓扑的保证；当前完整Demo没调用MPC |
| 多种搜索与因子化协调 | 有部分历史可行案例，起点和时序检查成本很重要 | 简化TaskRRT＋时序组合不是完整CBS；未做隐式多路径乘积图就不叫完整dRRT |
| GPU RRT/PRM | 批量FK、状态与边碰撞是可复用加速核心；目标多解可共用一棵树 | 增加种子/节点不必然提高性价比；PRM连接数量增大会带来显存和道路图管理成本 |
| 解析与数值IK | 根据模型条件和阶段分工，而不是一律选其一 | 代理支持解析不代表任意URDF自动可解析；固定ψ能解一个目标不保证整条连续直线 |

两只手分别抓两个箱子，从来没有要求它们抓同一个刚性箱体。无工具间距/相对朝向固定闭链合同。历史有关“箱子拉伸”“双手闭链流形”的解释已纠正，不得带入后续算法。

## 4. 目前的任务合同

场景是固定底盘、标准5×5箱墙、约0.9m墙距。箱体尺寸0.30×0.40×0.40m、箱缝0.01m、集装箱宽/高2.4m。墙距由底盘前缘参考计算，不是笼统的base_link原点到箱心距离；实现见[场景工具](tools/v3_wall_ik_benchmark.py)。

60组合入口针对前三排，左两列/中列归左臂，右两列/中列归右臂；左右任务同排或邻排，不允许两臂同抓一个中箱。这是任务枚举能力，不是60组已批量验收。

当前一次任务的状态顺序：

1. 从第一Home启动，检查原始状态合法。
2. cuRobo接触IK生成候选，最多保留32个成功双末端构型，按加权Home关节距离排序。
3. 各候选固定自己的Updown及左右ψ，预计算解析35cm直线抽离。
4. 按排序选择第一个“抽离可行＋Home到接触的RRT可行”的候选。
5. 从Home执行到接触；吸附后显示附着箱；执行已预验的直线抽离。
6. 从抽离末态搜索到新放置完整6D目标；携箱原地yaw 0→180°。
7. 模拟释放，箱体从显示与任务地图消失；空载yaw 180→0°。
8. 双臂空载RRT回到同一个第一Home。

有候选抽离/到位失败时，按用户明确的候选次序继续。选定起点后，后续运输/转身/回位失败就停止，不自行反复换策略、起点或放宽约束强行成功。这里“放置”是释放事件，没有模拟箱体下落、接触放置台、真空信号或单独下降动作。

## 5. 模型、坐标与数据：必须会核对

### 5.1 当前模型和解析求解器

description研究来源：feat/v3-analytic-proxy@071cb954d8c626626c41cdeff599bc451a56c117；当前用解析代理上身入口，保持固定底盘实验。新四舵轮/IMU整机资产是另一入口，未切为本Demo运行模型。

代理只理想化左右J3/J4/J5/J7八处毫米级原点，使肩/肘/腕满足解析共点假设。它是研究/仿真模型，不是实机几何标定。细节见[sync_manifest.json](generated/v3_analytic_071cb95/sync_manifest.json)。

V322Left/Right参数固化在[vendor解析实现](vendor/alfa_robot_analytic_ik/src/v3_redundant_analytic_ik.cpp)。world目标必须通过当前固定Updown的arm_carriage变换送入solveInArmBase；函数名里的arm base不能误当world/base_link。ψ由swivelAngle得到，理论分支受限位/数值重复约束，不保证每次八解。

### 5.2 工具与箱体唯一变换链

统一写法：

```
T_world_box = T_world_tool × T_tool_box
p_world_sphere = T_world_box × p_box_local_sphere
T_link7_box = T_link7_tool(当前URDF) × T_tool_box
```

- 四元数输入cuRobo/Viser是wxyz；SciPy默认xyzw，转换必须明确。
- 当前侧吸接触四元数由旧→新Tool0严格变换，函数是canonical_side_suction_quaternion_wxyz。不要再凭“Y+90°看起来正确”手改。
- tool_to_box旋转是接触旋转的逆，箱心沿工具局部+Z偏移0.15m；本地箱体+Z定义为箱顶方向。
- loaded_robot读取实际URDF的link7→tool0固定关节；不要硬编码旧yaw/pitch或151mm补偿后又重复应用。
- 固定15维模型与虚拟mobile18同源；后者只加base_x/base_y/base_yaw链，零底盘状态下同一臂姿态的FK必须完全一致。
- 共享Updown只有一个；双臂IK不能由两个不同高度解随意拼起来。
- 运输搜索15维，不搜索x/y/yaw；输出18维是为了追加确定性转身和显示，不等于已经做18维联合搜索。
- yaw目前是普通有界角度，不是已经验证过±π环绕的SO(2)空间。

更换URDF后先验证工具FK、箱体朝向和GPU球心。过去出现过外观修好了而碰撞球仍错位，必须同时验证，而不是只看截图。

### 5.3 目标是完整6D Pose

放置位置与旋转都来自已确认物理目标。当前冻结在[current_carry_target_6d.json](generated/current_carry_target_6d.json)，包含用户最后要求的世界X减5cm。不要用新模型对旧关节角重新FK后悄悄替换目标；关节构型可以变化，目标朝向不能任意放宽。

Home默认来自命名姿态文件，最终要求回到同一个关节状态。关节±180°限位与轨迹跳变要按真实有界关节检查，不能把归一化后的长绕转伪装成小角差。

## 6. 碰撞模型：已定版什么、尚未保证什么

箱体默认20个官方VOXEL内部拟合球＋12个边中点球＋8个内收角球，没有面心球。冻结表在[40球JSON](generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json)，策略见[cuboid_collision_policy.json](generated/v3_suction_v322_6bb184b/frozen_collision_model/cuboid_collision_policy.json)。

它的价值是零外凸，减少密集箱墙初始误碰；代价是内接球并不完整覆盖长方体边角。球数和拟合方式改变必须记录覆盖/外凸/缺口，不能只比较渲染观感。VOXEL内部覆盖率与整个40球组合覆盖率是两个不同指标。官方附着物球表示与世界cuboid/mesh表示不要混淆。

当前GPU检查包含机器人自碰撞、世界障碍、关节限位、地面与箱体朝向；地面只豁免base_link正常支撑接触。箱体稳定性是几何上方向点积/倾角89°门限，不是动力学承载、吸盘真空或惯性判定。

**沿用用户此前认可的实验合同：抽离途中暂缓附着箱球碰撞检查，到35cm完成后启用。** 机器人/箱墙/集装箱/地面仍检查。这不能宣称为严格负载全程安全。

另外，当前scene(boxes)在到位搜索前就将任务箱从静态箱墙中移除，界面初期仍显示这些箱子。接手者必须认识到“可视化存在”和“世界碰撞场景存在”不必一致；这是当前原型的接触任务简化，不能未经验证当作生产语义。

当前没有把真实箱体OBB全程/连续复核接进所有搜索门禁。OBB/SAT可以作为下一步精检方向，尚不能声称已完成。历史精确验证代码往往固定箱号和原场景，不能拿来直接冒充60组合通用门禁。

## 7. 搜索实现与调参要学会的内容

入口：[v3_batched_loaded_search.py](tools/v3_batched_loaded_search.py)。

- GpuValidity.evaluate将状态分块批量处理，cuRobo GPU算FK/球碰撞；CPU/Python仍管理父节点、候选路径和部分结果，系统不是全GPU。
- edges按加权关节最大差分确定插值点数；Updown权重5使米和弧度在自定义度量中折算。0.05是加权尺度，不是全轴统一0.05rad或5cm。
- 单起点多目标共用一棵树，不为每个末端IK构型再建一棵树。共享目标探索能减少重复计算。
- 2秒Anytime在首解后继续保留更低代价连接。当前代价含加权路径长度和箱体倾斜，不是执行时间或动力学能耗最优。
- Informed采样用于提高首解之后采样效率，当前无完整重连/渐近最优证明。不要称RRT*或BIT*。
- 粗搜索通过后进行0.5°加权细验边及密化；仍是离散检查，不是连续碰撞证明。
- apply_shortcut=False为当前完整周期实际配置。代码还有最远可直连功能及stats中shortcut字段，字段存在不能据此认定当前用了Shortcut。
- 原始搜索→轨迹密化→图示播放是不同时间；要按CUDA同步边界计时。2秒预算可能被最后一批工作和末尾验收超出，不是硬实时截止。
- 正常有效性查询不需要FCL接触详情。需要诊断时再逐球审计，不要把百秒级串行Python审计放进每条边。

GPU加速不能解决目标模型错误、不可达目标、窄通道采样拓扑或起点分支不连通。不要只用更多节点/种子掩盖这些问题。

## 8. 解析抽离和候选筛选如何读代码

[FullCyclePlanner](tools/v3_full_cycle_planner.py)负责选择候选和阶段编排，[AnalyticBridge](tools/v3_analytic_bridge.cpp)暴露C接口，由Python ctypes调用。

抽离每个1cm目标点解左右解析IK，固定各自ψ和同一Updown；按关节平方距离选择相邻解。随后密化并GPU验收、独立yourdfpy FK检查直线/旋转。目标点满足直线不保证两点间关节插值仍严格沿直线，密化FK验收因此必要。

当前流程不是同时先算完32条再全排序：按排序逐条试，首个成功就停止筛选。首组第1候选成功，不能据此说32条都成功。也没有多起点全局择优，用户明确限制了该方向。

IK“success”与后验collision/ground/tilt通过分别记录。一个提示说“全部碰撞”可能实际是稳定性失败；接手者须输出各阶段数，不能继续将所有拒绝合并成“无解”。

## 9. 性能与有效证据入口

不在本文重复实验全表；请读版本化报告：

| 要核对的结论 | 证据 |
|---|---|
| 解析FK与独立URDF FK、IK回代和均值12.4µs | [analytic_fk_ik_validation.json](generated/v3_analytic_071cb95/analytic_fk_ik_validation.json) |
| 初始完整周期/阶段衔接 | [full_cycle_first_pair.json](generated/v3_analytic_071cb95/full_cycle_first_pair.json) |
| 未复用前分项计时，包含首轮/热轮区别 | [full_cycle_timing_samples.json](generated/v3_analytic_071cb95/full_cycle_timing_samples.json) |
| 缓存后连续3轮完整成功、显存占用与热态6.44～6.52s | [full_cycle_cached_timing.json](generated/v3_analytic_071cb95/full_cycle_cached_timing.json) |
| 当前运行及边界说明 | [README](README.md)、[full_cycle_usage.md](generated/v3_analytic_071cb95/full_cycle_usage.md) |
| 大型旧实验文件未复制，查路径/大小/hash | [historical_artifacts_manifest.json](historical_artifacts_manifest.json) |
| 官方Graph连接器本地修复和测试 | [curobo-v2-local-fixes.patch](curobo-v2-local-fixes.patch) |

旧实验数据使用源目录、旧模型和旧工具合同。明确标记“作废”的球错位统计不可作为任何通过率。回归时固定模型SHA、目标Pose、箱球SHA、起点、场景删箱集合、搜索预算、种子、机器和GPU参数；随机种子相同也不保证墙钟预算结束时树完全相同。

热态主要剩下三段各约2秒选优预算。对象复用省掉的3秒不意味着数学优化速度突然快了三倍。首次建对象、CUDA/JIT/首次shape调用仍有成本；任务切换会失效重建。

## 10. 环境、第三方依赖与启动

2026-10-05当前环境已适配官方main@78fd485fa82d9b9a063fb4985e371814587e666a，Python3.10、PyTorch2.8/CUDA12.8、Warp1.17.0、pytest8.4.2、RTX4060 Laptop。旧正文实验数字仍基于4ea77366；旧本地连接器补丁不应用到新main。初始化变更、回归范围与可复制命令见CUROBO_MAIN.md。环境并非可在任意CPU机器直接运行的自包含包。

当前分支保存我们的代码、冻结米制mesh、模型配置、解析源码和patch，未提交完整NVIDIA第三方仓库、venv和.so。prepare_checkout编译解析桥并重定位URDF/config路径，可能修改checkout里的生成文件；只提交有意变化，别把自己机器绝对路径无意推回。

本机新终端可用：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/prepare_checkout.py --compiler g++
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/v3_full_cycle_demo.py --port 8090
```

遇到端口冲突先读进程确认是哪个Demo，再停止相应服务或使用8093，不能随意清掉同事进程。旧8091/8092都是局部诊断工具，网页地址存在不意味着后台进程仍在运行；本文编写时未发现Demo服务运行，需启动。

.env/鉴权信息不写交接、不提交Git。当前已可用Linear MCP，优先用MCP；之前CIMD token交换invalid_client及旧token invalid_grant问题可查会话记录，DCR重新登录后工具已成功读取/写入。

## 11. 源码导航与可维护性边界

| 文件/目录 | 职责 |
|---|---|
| tools/v3_full_cycle_demo.py | Viser任务选择、在线计算、阶段播放、负载显隐 |
| tools/v3_full_cycle_planner.py | 整周期编排、候选选择、解析抽离、回位和计时 |
| tools/v3_interactive_60_tasks.py | 60组合、cuRobo求解器与checker缓存、运输/转身、冻结目标 |
| tools/v3_batched_loaded_search.py | GPU有效性、负载装配、RRT/PRM及搜索比较 |
| tools/v3_wall_ik_benchmark.py | 命名轴、场景、世界箱位、接触及附着合同 |
| tools/v3_search_helpers.py | 当前入口需要的图参数、审计；旧OBB函数是历史转接 |
| tools/prepare_checkout.py | 路径准备和C++桥编译 |
| vendor/alfa_robot_analytic_ik | 与代理匹配的精确解析源码快照 |
| tools/experiments/ | 历史探索；不承诺全部在新目录开箱运行 |

历史脚本有绝对路径、固定箱号/模型及重复模块同名。当前支持入口已解开这些import依赖；不要在新实验中再从旧绝对sys.path偷加载代码。vendor源码不能无记录更新，仓库ROS解析包与vendor未必同版本；本研究编译明确取vendor。

当前规划对象仅按活动组合缓存，1个IK共享接触/目标＋四类checker。场景几何或负载配置如果运行中变化而任务编号相同，现有缓存键不会自动辨别版本，必须主动失效。不得在两个线程同时修改共享checker缓冲区；GUI当前串行任务工作线程，不是并发任务服务器。

## 12. 必须补做的验收与接手优先级

先复现，不同时改模型、目标、碰撞和搜索。建议按以下顺序推进：

1. 在新正式分支路径复跑首组，确认使用的模块__file__、URDF/config绝对路径、模型/目标/40球版本；保留完整JSON和逐阶段结果。此前新分支只做导入、编译和路径检查，完整成功证据来自原研究目录，不能混成新checkout全周期验收。
2. 验证接触位箱心位于原目标箱中心、箱顶向+Z；GPU附着40球与独立URDF FK同位。对固定/mobile零底盘及yaw变化至少各抽样。
3. 回归首尾Home一致、所有阶段接缝、Updown抽离不变、密化直线/姿态误差、转身释放显隐和失败停止。
4. 用当前同一模型逐组验证基础11组，再测60组合；记录IK无解、解析分支断连、到位搜索超时、负载碰撞、目标筛选、yaw失败、空载回位各类原因。
5. 单独补精确OBB或真实网格验收方案及接触/抽离允许规则，识别内接球漏检；不得靠单纯放大世界障碍同时引入密集起点假碰而声称解决。
6. 再做公平四搜索器对比：同起点、同完整目标、同负载、同场景和预算；先解时间与2秒最优连接质量都统计。预算策略优化属于下一实验，不能未授权直接改变拉满2秒合同。
7. 若要对接RT/仿真正式执行，另立接口/阶段任务：状态反馈、时间参数化、关节速度/加速度/jerk、取消/超时、场景同步、真空释放与执行轨迹Action。当前原型未提供这些生产验收。

“不成功就停下来沟通、不要不断换办法强行做成”是用户明确要求。需要范围/约束变化时，先保存具体失败证据再沟通。已授权候选循环可以继续，不因每个候选失败都中断整条既定筛选。

## 13. 同事需要学会什么

接手不是先背所有算法论文，应能解释并实际操作以下内容：

- SE(3)齐次变换、四元数顺序、Tool0与box-local坐标，以及旧模型目标转换后的物理等价性。
- 7轴解析IK的冗余ψ、分支、限位、singularity、FK回代；Updown如何统一作用于两臂的臂基座。
- 数值IK多种子成功/可行性/目标误差与后验碰撞筛选的区别；正确批量使用GPU，不重新造串行多轮。
- RRT树、PRM道路图、goalset、informed/anytime、粗/细边验证；哪些部分我们写、哪些算子来自cuRobo。
- 球近似、世界primitive、附着变换、ACM/接触规则、离散与连续碰撞；为何零外凸不等于无漏检。
- GPU同步计时、冷/热态、显存分块、缓存对象生命周期；不能用nvidia-smi单次利用率推断整个程序已“拉满”。
- 研究结果有效性：固定试验合同、留失败、区分参考轨迹与无参考、区分Demo与正式执行。
- Linear父节点/子实验边界和Git可追溯性：提交Refs MOTION-275，评论一次关键调整，状态按当期验收而非未来想法变化。

建议阅读路线：先通过界面跑首组→阅读FullCyclePlanner与GpuValidity→自己画一条tool-to-box变换链并验球心→阅读多目标RRT→阅读解析测试→最后追溯TrajOpt/Graph/MPC实验。代码与固定版本公开API比“官方演示宣传”更适合作为本任务能力判断依据。

## 14. 后续AI协作与文档维护

当前用户的偏好：直连任务目标、减少无关验证、不反复确认已授权操作、不擅自换算法、成功失败都能具体看出来。提供启动指令要带cd、全部参数和续行符；中文说明简洁，技术边界准确。

推荐技能：handoff用于继续交接；diagnose用于具体现象/性能退化（先按仓库Agent skills要求确认上下文）；用户明确要求先测后写时才用tdd；只有跨模块重构确有需求时再用improve-codebase-architecture。这些不是默认要求额外建审批或泛化任务。

每次完成后更新275具体评论、证据路径与提交，必要时补当前工作树.ai_teamwork/LOG.md。不要复制过期会话解释；若新结果纠正旧结论，应指出哪条结果作废、哪个模型/目标变化，旧历史保留可追溯。

后续接手最重要的边界：**这套原型已经能演示一个正确衔接的周期；下一步是把当前合同下可复现、可诊断、可验证的多组算法能力做扎实，而不是把首组成功扩写成生产可用。**
