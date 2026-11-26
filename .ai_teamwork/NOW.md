# 当前项目状态

## 2026-11-26 MOTION-505 Review与远程交付整理

- 正式进度：MOTION-505（父MOTION-483「不同SKU箱体抓取方案设计与验证」），状态In Review，负责人文子轩/验收李昊洋；用户2026-11-26更正原419误绑定，419原验收汇总已恢复，本方向按483四条标准另留唯一子方向验收汇总。三个阶段进度仍在505原线程，父483保持In Progress，不自动Done。
- 当前交付入口：`research/curobo_v3/docs/MOTION505_TWO_BOX_REVIEW.md`与`docs/two_box_acceptance_20261125/`（44份原输入、逐例时间/owner/吸附、SHA、边界与失败根因）；原GPU44/相关CPU107证据原样保留。独立吸附审计脚本已从generated复制为版本化源码，函数AST相同；CPU mock适配现有共享FK接口，生产规划源码未在整理阶段再改。
- 扩大检查：可收集CPU 305过/15失败/19跳过；15失败在隔离PR82源码+同资产下全部复现，无新增失败。另2个旧测试文件缺`alfa_motion_viser`不能收集；不宣称整仓测试全绿。完整报告随交付版本化。
- 历史实验/失败/源快照完整归档至`/home/astesia/Sevenova/验收归档/motion505-two-box-20261126/two_box_layout-evidence.tar.zst`，SHA5259294b0a1d5bc14d67508485624a66a7a86414c9e18f6387b5e8e320fed7c2；已校验后清掉冗余历史二进制/重复源快照与旧role缓存，最终44实际数组、必要历史集成fixture及Viser8295保留。生成物/编译缓存明确忽略，不提交5GB实验目录。
- Git：基线PR82 ee6dba3，本地feature/pr82-two-box-extraction；实现提交eea760e7ab33bc7d4312250e1474150616f9d920已推fork/motion-505-two-box-extraction，远端SHA已核对。未开PR/未合主干，505保持In Review。Git/Linear收据见`docs/two_box_acceptance_20261125/delivery_receipt_20261126.json`；下方历史“未commit/待scope”等仅对应其当时状态。

## 2026-11-25 最终当前结果

- 原25箱覆盖已由用户确认并恢复：完整名义外沿Y±1.02m/Z0–2.04m，箱心连续、按SKU半尺寸内缩；D66cm/阈值79cm/30与25.5cm抽离/指定14角/任务内自由升降保持。
- 最终同运行码全新44/44通过（边界8+原36；29同步/8顺序/7换手），外部CPU44+88次实际首次吸附全过；107项相关CPU合并回归过，1,176,114native点，31个实际升降值，14角最大误差4.37e−8rad。原6/8、7/8失败批次及违规整段全部保留，没有继承旧成功。
- 共享修复：实际手臂与物理ID独立、场景/核验/回放/预览绑定；抽离native原5mm底盘/2mm机器人地面余量与LM引导；原≤8rest子段支持抽离并守住整段SKU位移/qva接缝/10kg真实载荷与source清出不回入。无几何、足迹或动力学门槛降低。
- 权威汇总`research/curobo_v3/generated/two_box_layout/original25_domain_20261125/acceptance_current44.json`；最终8 `run_003`、最终36 `original25_current36_20261125/run_001`。1236共享运行文件SHA一致，当前36manifest1445/digest0792610110e60617901f3f648e1ce34315c054d92391d5f930311936fe22de0b，收尾0变化。全部本任务进程/子agent结束，queueexit0/sharedlock空/freeze释放，没有待运行GPU/测试或commit/push/实机执行。
- **完整连续任意位置goal仍未证明，不markcomplete**：44有限案与放松必要几何条件不是连续全域证明；没有范围输入/用户许可缺口，也没有已知仍失败的最终源码案例。合同详情以`BOX_LAYOUT_JOINT_CONTRACT_CORRECTION_20261123.md`顶部为准。下方都是阶段历史。

## 2026-11-25 原25箱域验证及共享修复（初批历史）

- 用户已确认新两箱整体在原25箱覆盖内，名义完整外沿Y±1.02m/Z0–2.04m；连续箱心，仍为D66/阈值79/原抽离/指定14角/自由升降。域、源SHA及CPU派生见`generated/two_box_layout/original25_domain_20261125/domain.json`，不再等待范围输入。
- 新8例首批`run_001`完整执行6过2失败；6个接受结果进程外CPU及12次首次吸附通过，源无变更，持锁GPU约12.5分钟。失败06为换手后实际抽离曲线底盘间隙4.9336mm<原5mm，07为并排精确接触时原标签分配双臂接触均不可达而交换实际手臂有运动学解；均保留域内、不是不合法证明。
- 正在并行共享修复：抽离原生margin验证/LM引导；箱体物理ID独立于实际手臂的自动交换fallback及CPU/回放绑定。初批freeze已释放，当前源码已改，下面原36的同源结论只对应其原快照；待新源码CPU及fresh8验证，不继承老成功。无commit/push/实机执行。

## 2026-11-24 双SKU任意合法位置纠偏（优先于下方历史交接）

- 合同与最新证据入口：`research/curobo_v3/docs/BOX_LAYOUT_JOINT_CONTRACT_CORRECTION_20261123.md`顶部当前结论；`generated/two_box_layout/joint_contract_20261123/CONTINUATION.json`。D66cm、统一阈值79cm、初始共面/高差≤60cm，原SKU world−X抽离30/25.5cm，直接用户14角回中、updown任务内选、不固定。
- **最终当前源码36/36默认fresh native+外部CPU通过**（同步24/顺序7/换手5），72次首次实际吸附面/79规则/cap足迹另独立CPU全过；没有继承旧成功。source1474全SHA一致`52bd2e0cf8b772d37d1797ef812f85c89083ceae077413212b478e1bab1728af`；76CPU相关回归过，最大14角误差4.37e−8rad，实测25个升降值。支持正/负Y实际双向handover、Top/Side及先Cube后Tall；physicalID/owner/真实质量与原位历史保持。
- 权威报告：`generated/two_box_layout/default_handover_final36_20261124/acceptance.json`及run_001/terminal.json、independent_cpu36.json；实际吸附面审计`actual_suction_contract_audit_20261124/batch_final.json`。exec27021实际终态0、queue-a422...completed、owner/worker已退出、锁空，freeze已释放，所有子agent关闭、无本任务GPU/测试待运行。旧run001/002失败及34/35准备产物保留；勿无因重跑完整36。
- **2026-11-25用户已补齐范围，恢复推进**：按原有25箱的覆盖范围，新两箱整体不超出原范围；不再等待Y/Z确认，旧货舱内腔提案已被取代。原25箱几何及92组fixture已核对：完整外沿Y±1.02m、Z0–2.04m，原36中30例域内/6例更大范围证据均保留；正在准备边界fresh默认入口验证；D66cm、阈值79cm、14角及任务内自由升降保持。有限36不是连续全域证明，未markcomplete。证据写入`generated/two_box_layout/original25_domain_20261125/`。尚无Git提交/推送/实机执行。

## 2026-11-10 LM后处理发布PR准备

- 最新合入基线PR80：alfa_v3_curobo@92bd8cdb。当前独立分支codex/lm-refinement-release只迁入双臂LM后64控制点32轮L-BFGS、extract后处理loaded checker、部署核对与可视化路径适配；原碰撞/RRT/实际LM末态重连编排不改。
- 当前唯一部署说明research/curobo_v3/FAST_DEPLOYMENT.md；冻结依赖与模型SHA在config/fast_release_manifest.json，prepare_checkout.py --fast-only不再重写历史跟踪模型，v3_fast_preflight.py --gpu核对前端、Fast源码、模型与Home。旧README/HANDOFF历史说明已加优先入口警示。
- 独立检出40测试通过1跳过；两例全新规划候选运动11.706/11.865s、12段TimeOpt动力学采样通过、接缝0。L18/R11原接近TrajOpt失败保留。8293独立实时页面实际点击计算及播放通过，原8290/8291不变。已知冷/Graph重捕获等问题本次保留，不声称全任务/实机验收。

## 2026-10-31 固定解析抽离／RRT运输／LM放箱流程

- 用户确认以 `FrozenCyclePlanner` 固定全流程：旧多候选解析35cm抽离、多根多目标RRT运输、导航瞬移、128点15轴连续LM放箱、释放后从LM实际末态RRT回Home。放箱不回退RRT，携箱仅用原40球模型。
- LM参数固定32局部种子、最多32轮；单臂空闲关节保持放箱起点值。当前URDF、191mm TCP、0.982m放箱箱心高度沿用；不再继续调整算法，后续变更需用户提出。
- 默认 `v3_forward_precontact_demo.py` 已切换固定流程；独立实时入口 `v3_frozen_cycle_demo.py` 使用8243，8242保留放箱段对照。全117任务105成功，11项LM停止、1项原候选失败；失败不补画、不回退，属于离线几何验收。
- 核心说明 `research/curobo_v3/CORE.md`，紧凑验证记录 `generated/frozen_cycle_release/validation_report.json`。下方新WARP记录为历史方案，不再是默认入口。

## 2026-10-27 新WARP迁移

8210默认新WARP：球面软参考→128点平滑ψ解析→整段30轮→合法先验Informed Connect→0.25°验收，PR65后处理独立开关保留。当前释放箱心0.982m，模型/191mm TCP/Home不变。1.082m基线117/117复测通过；0.982m仅三代表任务几何和时间化各3/3，不混算全量成功率。运行核心已从benchmark抽出，复测及数据见research/curobo_v3/NEW_WARP_HANDOFF.md迁移补充。8221研究服务保留，未涉及实机。

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目；当前仓库只保留运控、电控、机器人模型及其必要运行适配，历史感知、导航和仿真实现已退出主线。

## 开工入口

- 先读 `.ai_teamwork/START.md`。
- 再读本文件和 `.ai_teamwork/TASKS.md`。
- `.ai_teamwork/archive/` 默认不读；只有追溯旧方案、验收证据、责任边界时再查。

## 当前推进重点

- 当前cuRobo正式研究入口：v3_visualized_task_demo.py → SingleArmPlanner，119任务选择，
  优先解析64、常驻cuboid场景更新、双臂8起按需抽离、旋转软代价、单臂整机运输终点1.05m硬圆柱。
  side箱心1.2m/Y45°，top原目标，71底盘球/4内部cuboid及20升降架球；导航到位瞬移、释放消失、直接回Home。
  详见OPTIMIZED_CYCLE.md/CONVEYOR_DELIVERY.md；24/25单臂验证通过，右02接触无合法候选是已知失败。
  保留Updown10cm=关节15°；无生产接口/实机验收。8152为本轮同步验收，原8150/8151服务保持。

- `alfa_v3_curobo` 当前方向：MoveIt替代的基础阶段1～3（MOTION-276），研究入口在
  `research/curobo_v3`。独立快照/状态与规划核心见该目录`CORE.md`；GPU搜索算法仍跟踪
  MOTION-275。以下旧V3/旧主线信息属于历史上下文，不作为此分支当前模型和入口。

- 当前主线：`v5_dev` 已收口左右箱体正面中心 6D 位姿任务合同；功能分支正在接入 V3 七轴双臂模型。
- V3 模型工作跟踪：Linear `MOTION-94`。当前试验分支已接入 `robot_v3.0.9` 十六自由度整机模型；双臂外观与碰撞网格和 V3.0.8 上游资产逐字节一致。
- V3 基础动作跟踪：Linear `MOTION-154`。分支 `motion-154-v3-dual-arm-simple-motion` 已完成首个40cm箱双臂同步解析笛卡尔平移Demo，待用户交互验收后继续旋转和异构握持任务。
- 正式任务输入只包含 `request_id`、左右正面中心 `pose_6d` 和 `execute`；算法内部按高度容差识别排数、吸附方式和抽离策略，禁止从箱号或外部吸附模式获取帮助。
- 旧 `/robot_motion/run_box_pair_task` 仅保留为显式兼容入口，默认完整栈不启动 `box_pair_task_adapter_node`。
- 当前任务表只保留未完成/需确认事项：T-0030/T-0031/T-0032/T-0037。
- 已完成/已同步 Linear 的长过程已归档到 `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/`。
- PM 必须持续把完成任务移出当前表，避免后续 AI 误认为仍需处理。
- 架构目标、包职责和调试代码准入规则见 `docs/运控/系统架构与包职责边界.md`。
- 可视化架构入口见 `docs/system_portal/architecture.html`。

## 当前仍需注意

- 当前 V3.0.9 恢复 `updown` 升降和 `head_joint` 旋转自由度，连同左右七轴共16个可动关节；`updown` 逻辑范围为 `-1.0～0.0m`（最低点 `-1.0m`、升高1m后的最高点 `0.0m`），`head_joint=-1.57～1.57rad`。因 V3.0.9 与 V3.0.8 的46个上游 STL 完全相同，继续复用 `ros2_ws/src/alfa_robot_description/meshes/robot_v3_0_8/`，不重复存储。
- V3 默认初始姿态固定为左右臂相同的 `J1～J7=[-90,-90,0,-90,0,0,0]°`，共享 `updown=0m`、`head_joint=0°`；模型查看器、mock ros2_control、MoveIt 初始状态和 SRDF `home` 必须保持一致。
- Linear/Git 关联提交标题优先使用 `Refs MOTION-xx: ...`；只写 `MOTION-xx:` 不稳定。
- 一个 issue 只对创建时的验收目标负责；后续探索/测试应拆新 issue 或放 Backlog，不要让已达标 issue 永远开着。
- `alfa_robot_moveit_config` 已不再编译或包含 `scripts/ik_benchmark/` 的头文件；公共 IK 候选类型已迁入 `robot_motion_core`，Rerun 公共实现已迁入 `alfa_robot_rerun`。
- `dual_arm_planner_node` 仍承载完整候选排序、抽离和负重规划适配；这些实现尚未全部迁入独立 core/planning service。
- 历史 `bio_ik`、仓库内 `alfa_robot_hardware` 和旧 `alfa_robot_bringup` 已退出主线；实机硬件与生命周期由外部 `rt-control` 域负责。
- 当前 V3.0.9 已完成 description、16轴 ros2_control/MoveIt 契约与冗余解析 IK 回归；V3.0.8 的单点、前伸40cm、周围15cm数据仅作为双臂几何参考，恢复整机自由度后的完整抓取流程尚未验收。

## 当前主要模块速查

- 接口契约：`ros2_ws/src/robot_motion_interfaces/`
- 纯算法公共核心：`ros2_ws/src/robot_motion_core/`
- 核心运行时：`ros2_ws/src/robot_motion_runtime/`
- 场景能力：`ros2_ws/src/robot_motion_scene_service/`
- 运动算法与 MoveIt 适配：`ros2_ws/src/alfa_robot_analytic_ik/`、`ros2_ws/src/alfa_robot_moveit_config/`
- 执行适配：`ros2_ws/src/alfa_robot_execution_bridge/`；真实硬件由外部 `rt-control` 域负责
- 模型与启动编排：`ros2_ws/src/alfa_robot_description/`、`ros2_ws/src/robot_motion_runtime/launch/`
- 可视化：`ros2_ws/src/alfa_robot_rerun/`
- 实验资产：`scripts/ik_benchmark/`，仅用于验证和迁移，不应反向成为运行时职责来源。

## 常用追溯入口

- 本次归档摘要：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/COMPLETED_SUMMARY.md`
- 本次归档前完整日志：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/LOG.before_archive.md`
- 控制层硬编码：`docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- 控制架构梳理：`docs/REFACTOR_ARCHITECTURE_NOTES.md`
- IK 服务说明：`docs/运控/IK/ik_service.md`
