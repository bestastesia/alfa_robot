# 8209 cuRobo研究链路交接：算法、代码、预热与性能优化

2026-10-27：用户已指定8221的球面软参考＋平滑冗余角解析＋整段30轮＋RRT为**新WARP**。最新迁移以 [NEW_WARP_HANDOFF.md](/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/NEW_WARP_HANDOFF.md) 为准；本文保留旧LM链路及1.1后处理迁移背景，不覆盖新算法定义。

更新：2026-10-24。面向接手实现、同步仓库和继续优化的工程师。

## 当前URDF迁移补充（2026-10-24）

2026-10-26后续开启：8210默认改为TimedCurrentUrdfPlanner，直接复用PR65时间化器并保留当前六段规划器；四个关节路径阶段GPU验碰Shortcut、所有运动阶段时间化、0.5°补充验边、阶段端点不变。J1～J4限制15°/s和30°/s²，J5～J7为30°/s和60°/s²，jerk沿用模型配置；仍非力矩动力学验证。Shortcut查询按16条边分批以限制显存，不改变原最远合法点选择规则。侧吸L22/R17、顶吸L03/R02、单臂R02三例成功，速度/加速度/jerk及独立差分速度检查通过；名义运动72.56/74.16/74.61秒，计算7.82/10.51/10.51秒。部分段使用PR65的validated_shortcut_stops安全回退，不能宣称全部无停点。证据generated/current_urdf_timed/acceptance/summary.json。页面支持0.1～3倍播放，按真实相邻时间戳调度；阶段截取同步时间/导数及帧索引，不修改实机。

2026-10-26集成：PR65已合入alfa_v3_curobo@96c2c70c；本开发分支40项提交rebase完成，GUI入口兼容planner_class与default_assets，缺省仍使用原选择的FullCyclePlanner。旧94双臂测试同步为现行92双臂＋25单臂合同，明确排除L07/R02及L02/R07。52项core测试（1项历史资产测试跳过）及17项当前模型/GUI合同测试通过；rebase后四次GPU全流程通过，证据/tmp/curobo-pr65-rebase-verification。PR65时间化入口保留独立，未默认套用在六段LM结果上，不把原时间化25箱证据冒作新模型验收。

- 本轮分支 `feature/curobo-8209-current-urdf`；当前入口仍是 `tools/v3_forward_precontact_demo.py`，默认端口改为8210，与原研究入口和工控机分开。
- 当前规划器 `CurrentUrdfForwardPlanner` 使用已确认的 `dual-zero-discrete-only-tool191-20261024` 固定根／18轴模型、191mm TCP、现行Home和限位。解析网格统一走RezeroAnalytic，不调用输出旧坐标的笛卡尔批处理库。
- 运输库按新TCP重新解算、独立URDF FK复核，运行时仍作完整碰撞及1.05m包络筛选。旧2cm升降网格遗漏了侧吸可行窗口；现于原[-0.2,0]m范围内加密为5mm，未改运输Pose、碰撞豁免或限位。
- Ready前真实运行16点及128点（64点分批）LM与32轮polish；长段与抽离共用求解器／目标缓冲。不缓存预热任务的候选或路径；预热结束精确恢复原完整显示快照。
- 35项相关回归通过；GPU双臂侧吸L22/R17、双臂顶吸L03/R02、单臂R02以及再次切回侧吸已通过完整六段，接缝与Home误差通过。不是117组全量验收。
- 复测入口 `tools/v3_current_urdf_smoke.py`；结果在 `generated/forward_precontact_current_urdf/verification/`，页面结果另存其 `live/` 子目录。原抽离途中空载／终点携箱、导航瞬移和释放移除规则保持，未涉及实机。
- 本轮未做polish CUDA Graph融合或早停，也未宣称达到3.5秒上限。本文后续原8209性能和未完成事项为迁移前历史，以上补充优先。

## 1. 先读结论

当前是“球腕笛卡尔参考 → 起终构型共同引导的并行LM → 构型软调整 → 碰撞筛选 → Informed RRT-Connect首条细验合法路径”的六段串行抓取实验。最新改动是取消从吸附关节解反推3cm路径：先独立求预接触IK、走到位，再求吸附IK并正向接近。

不要误解成已完成全部性能优化：
- 已有检查器常驻更新、部分求解器缓存与共享。
- 已有的启动预热主要针对碰撞检查器，没有完整覆盖16点／64点LM的构造和首次CUDA Graph录制。
- 完整LM预热、32轮软调整录图／融合／早停，均是下一步建议，尚未实现。
- 最新8209不是已验证117/117；旧8208的117/117不能移植为新版结论。

## 2. 实际工作区和版本保存风险

实际代码仓库：/mnt/mydisk/ALFA/alfa_robot_v3
研究目录：/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3
不是用户当前聊天cwd /mnt/mydisk/ALFA/alfa_robot，也不是历史实验目录 alfa_robot_v3_curobo。

本次核对：
- 分支：feature/wall-min-tool-rotation
- HEAD：390b96b50c2d711831b87bc5904fafc78f070299
- HEAD并非8209研究方案的完整提交。
- v3_forward_precontact_demo.py、v3_forward_precontact_cycle.py、v3_lm_chained_cycle.py、v3_endpoint_guided_ik.py、v3_cartesian_compare.py均是未跟踪文件。
- 仓库还有其他模型、配置和算法的已修改文件，不能全部打包成这次改动。
- 仓库设置status.showUntrackedFiles=no；普通git status可能看不到关键新文件，务必显式加--untracked-files=all。

先保存本地研究树、依赖配置和生成资产，再开展迁移。不能只checkout HEAD或只复制两个8209入口来恢复。不要reset/clean，也不要自动切换分支或覆盖其他工程师改动。此交接没有创建分支、提交、PR或发送其他AI消息。

复核命令：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
git status --short --untracked-files=all -- research/curobo_v3
git diff --stat -- research/curobo_v3
```

先遵守仓库AGENTS.md、.ai_teamwork/START.md及NOW/TASKS/LOG。当前任务归属仍需用户决定，不主动同步Linear。

## 3. 最新全流程

所有阶段以前一段实际输出关节状态作为输入，不因后续失败回选上游。任务目标的几何关系仍然保留，不等于阶段可行性的前瞻依赖。

1. 配置当前选中任务，生成半已知箱墙场景和该任务吸附6D Pose；顶吸按当前策略包含底盘前进。
2. 吸附Pose沿工具本地+Z反方向退3cm，姿态保持不变，得到独立预接触目标。
3. 解析候选枚举和当前合法性筛选，参考Home的加权关节距离选择预接触构型；不先求吸附关节解、不先反向计算3cm运动。
4. Home→预接触：球腕场景参考＋起终共同引导LM先验＋Informed Connect；目标箱参与碰撞。
5. 到位后，以实际到达构型为参考，重新求吸附IK并固定本段目标。
6. 正向3cm接近：16点软参考＋LM＋Informed Connect；按原规则排除目标箱碰撞；末端实际到位后附着。
7. 35cm抽离：128点直线软参考＋LM＋Informed Connect，允许中途适度偏离参考，不再固定Updown/冗余角强行解析推进。
8. 以实际抽离终点选择运输构型，检查整机及携箱包络；单臂也包含空闲臂。
9. 实际抽离终点→运输位：球腕参考＋起终引导LM＋Informed Connect。
10. 导航到位采用底盘瞬移；然后求龙头车放箱目标数值IK，串行固定目标后连接放箱路径。
11. 释放后箱体按当前实验策略移出场景，再规划回Home；底盘回抓取位仍瞬移。
12. 检查实际阶段接缝，记录帧、TCP位置和朝向，可切全流程或六段独立回放。

六个运动阶段键：approach、contact_approach、extract、transport、conveyor_place、return_home。

## 4. 不可随意改变的规则和安全边界

- 关节顺序以ACTIVE_JOINTS为准：共享updown＋左右各7轴；不要把米和弧度直接无权相加。
- Updown距离权重为2.617994rad/m，即10cm等价15°；关节距离参考是软偏好，不是硬插值要求。
- 参考路线上各采样点同时参考起终构型，不采用“上一点解完才做下一点”的纯串行依赖；保留批量并行。
- 不因累计旋转、回退角、偏离参考直线而直接拒绝；这些用于软代价和诊断。不能重新加入用户未要求的2mm沿途直线硬限制。
- IK/FK目标精度检查、关节限位、碰撞和固定端点验收仍保留。候选图节点的Pose精度筛选不同于对完整路径强加直线硬约束。
- 粗搜索内部底盘碰撞余量2mm，最终原模型细验；不等于所有障碍、所有检查无差别统一膨胀。
- 搜索采用首条细验合法路径即停、2s上限，不是强制运行满2s。
- 路径参考不是整机可达性／连续性证明；最终仍依靠整机关节搜索与验收。
- 已知限制：35cm抽离沿途仍按空载模型验碰，终点才验携箱，payload_path_validated=false；不能声称全段携箱无碰撞。
- 导航不验证真实底盘运动；释放后的箱子移出场景。不能冒充完整现场安全闭环或实机验收。
- 规划失败明确停止，不擅自回选上游、改任务、删除碰撞或输出未验收轨迹。
- 当前117任务中已移除不可能的L07/R02和L02/R07，勿拿旧119任务直接混算。

## 5. 代码地图（相对研究目录）

| 文件 | 职责 |
|---|---|
| tools/v3_forward_precontact_demo.py | 8209页面入口，替换共享GUI使用的Planner类型，无全量结果预载 |
| tools/v3_visualized_task_demo.py | 任务选择、启动预热、实时计算、六段/TCP回放、保存每次结果 |
| tools/v3_forward_precontact_cycle.py | offset_precontact、pose_candidates、initial_motion：独立预接触IK→实际到位→吸附IK→正向3cm |
| tools/v3_lm_chained_cycle.py | 六段实际串联、阶段接缝、实际抽离、共享64点求解器、携箱布局切换时资源释放 |
| tools/v3_serial_endpoint_planner.py | 前驱关节距离选型、起终加权参考、普通运动阶段的LM和搜索接入 |
| tools/v3_wrist_guided_cycle_planner.py | 球腕参考生成、真实末端球组读取、实际TCP诊断 |
| tools/v3_spherical_wrist_route.py | 球腕参考路径规划；不约束速度必须对齐工具Z |
| tools/v3_endpoint_guided_ik.py | biased_seeds、solve_numeric、polish、pose_filter；长段LM及32轮Adam调整 |
| tools/v3_cartesian_compare.py | 接近/抽离软参考、16或64点batch求解、固定关节目标、LM先验连接与验收 |
| tools/v3_route_guided_compare.py | RouteGuidance先验采样、TcpRouteCost连接代价 |
| tools/v3_multiroot_search.py | 批量Informed Connect及路径检查；当前串行方案固定一组起终点 |
| tools/v3_batched_loaded_search.py | GpuValidity、碰撞/FK等基础能力 |
| tools/v3_persistent_collision_planner.py | 检查器常驻、场景参数更新、当前warmup_resources |
| tools/v3_conveyor_cycle_planner.py | 龙头车放箱IK、放置、释放、回Home外围流程 |
| tools/v3_research_assets.py | atlas/配置路径、模型身份和一致性验证 |
| tools/curobo_core/ | 状态、场景、资源、基础Planner与backend |

双臂预接触及吸附候选走原生解析网格枚举＋GPU合法性筛选；单臂沿用既有空闲臂候选策略，并非所有目标构型都用cuRobo数值IK。cuRobo/LM主要承担参考路径上的并行IK与整机FK/碰撞能力；龙头车目标另有数值IK。

关键扩展点：
- LmChainedCyclePlanner.initial_motion：旧8208默认反推仍保留，8209继承替换。
- CartesianComparison.lm_search：fixed_goal_joints用于固定已选择目标，不能悄悄换终点。
- endpoint_ik_cache[('lm', True)]与cartesian_engine.solvers[64]共享同一列表和目标缓冲；短段solvers[16]独立。

## 6. 预热、缓存和显存：已做与未做

已做：
- GUI创建planner后调用warmup_resources，再启动可交互服务。
- 预热部分侧吸／顶吸碰撞检查器，实际在Home单状态调用碰撞。
- 同拓扑场景可更新参数并复用检查器；并非每换箱号都重建。
- 长段LM max_batch_size=64，128点分两批，每点16种子；短段16点×16种子。
- 共享64点求解器和目标缓冲，底盘内部查询缓存最多4种尺寸。
- 在侧吸/顶吸或活动臂集合变化时释放旧携箱检查器，防止8GB显存累积OOM。

尚未做：
- 在Ready之前真实执行16/64点LM，完成首次Graph录制。
- 把自定义polish的32轮Python/PyTorch Adam环录图或融合。
- 让polish按收敛退出；当前固定32次更新＋末次FK检查。
- 持久复用龙头车目标IK求解器；现在每次构造、求解后删除，并gc.collect/empty_cache。

不要为追求预热保留无限量检查器和Graph。历史全量实验确实出现OOM；需测试跨吸附类型、单/双臂切换，而不仅仅同任务重复。资源预热可以用代表输入，不得复用代表任务的候选或结果替代当前任务的新计算。

## 7. 性能证据怎么读

详细数据不在这里重复，直接读：
- FORWARD_PRECONTACT_TIMING.md及同名_DATA.json：8209已有5次记录、4种任务；平均9.684s、最慢L22/R17为14.929s，含不同初始化条件，不是117种任务总体平均。
- FORWARD_PRECONTACT_PERFORMANCE_DIAGNOSIS.md及_PROBE.json：新增同对象同任务连续2次诊断，两次均成功，总计8.590s→4.002s，纯RRT 0.613s→0.599s。
- 第二次4.002s中，LM核心0.274s，32轮构型调整1.565s，参考生成0.497s，纯RRT0.599s；六运动段共3.073s，外围余额0.929s。它是稳定调用的一个样本，不是“预热后最慢任务”或服务承诺。
- 放箱IK构造＋求解约68ms，显式GC约133ms，不可把外围余额全部归因于IK或检查器。

总时间统一用measured_wall_ms；total_ms只是内部时钟，边界不同。
guided_search_events.lm_ms已包含soft_refine_ms，不能相加重复算。
cartesian_segments的inclusive_ms与对应阶段total_ms是同一段，不能再加一遍。
资源事件耗时可与阶段重叠；阶段外余额是扣除项，不是精确归因。

建议优先级：
1. 补齐真正LM预热，先不减少候选／采样／迭代／碰撞密度。
2. 优化polish实现的张量缓冲、录图或融合；再评估收敛早停，必须回看构型连续性和关节/TCP指标。
3. 给外围余额细分计时，适度复用放箱IK和减少不必要GC。
4. 最后再考虑RRT和参考路径算法微调，不应把目前主要开销误认为树搜索本身。

## 8. 数据和依赖不能漏拷

研究树包含大量未提交脚本；普通git diff不含新文件。代码还依赖生成库、模型、YAML、atlas、冻结参数及外部前端；不是纯Python单文件可移植Demo。

尤其注意：
- tools/v3_serial_endpoint_demo.py的SETTINGS仍引用历史_curobo目录的frozen_wrist_settings.json。
- CartesianComparison通过ctypes加载OUTPUT/libcartesian_grid.so；OUTPUT由v3_cartesian_cases.py定义，迁移时读取真实路径并保留构建来源，不能只复制.so后当跨平台可移植。
- config/placement_task_policy.json决定实际atlas；tools/v3_research_assets.py还使用冻结base_cuboids配置。
- PlannerAssets与页面默认参数决定实际URDF/YAML/命名姿态/箱球配置，不要拿正式分支另一套模型覆盖研究模型。
- 页面通过alfa_motion_viser和依赖校验加载前端，环境也要保留。
- 本机Python：/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python；诊断环境为RTX4060 Laptop 8GB、Torch2.8/CUDA12.8及本机cuRobo版本；迁移前重新核验安装，不建议直接升级依赖。

原始结果：
- 3次新版验证：/mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3/generated/forward_precontact_20261024/smoke/
- 页面跨版本共享档案：/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/priority_visualized_task_live/runs/
- 共享档案必须筛forward_precontact_policy=true或precontact_policy.mode=direct_pose_ik_then_forward_lm；latest.json与runs可能重复，不可重复统计。
- 旧8208全量：/mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3/generated/lm_chained_20261024/shared117/，仅作旧版对照。

## 9. 启动和测试

本次交接核对时8209端口没有监听，原alfa-forward-precontact-8209用户服务也不在；代码仍在。旧文档中“保持运行”描述的是当时状态，不是当前保证。本轮没有重启服务。

检查：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
ss -ltnp 'sport = :8209'
systemctl --user status alfa-forward-precontact-8209 --no-pager
```

确认没有重复进程后启动，勿在已有服务时重复执行：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
systemd-run --user --unit=alfa-forward-precontact-8209 \
  --property=WorkingDirectory=/mnt/mydisk/ALFA/alfa_robot_v3 \
  --property=Restart=on-failure \
  --property=RestartSec=3s \
  --setenv=PYTHONDONTWRITEBYTECODE=1 \
  --setenv=PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
  /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -u \
  /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/tools/v3_forward_precontact_demo.py \
  --port 8209
```

诊断：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
journalctl --user -u alfa-forward-precontact-8209 -n 80 --no-pager
```

相关回归：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
PYTHONDONTWRITEBYTECODE=1 \
PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pytest -q \
  research/curobo_v3/tests/test_forward_precontact_cycle.py \
  research/curobo_v3/tests/test_lm_chained_cycle.py \
  research/curobo_v3/tests/test_stage_playback.py \
  research/curobo_v3/tests/test_serial_endpoint_planner.py \
  research/curobo_v3/tests/test_cartesian_comparison.py
```

历史实现时上述相关测试21项通过；本次写交接未重新跑，不把历史结果称本轮验证。新策略只已完整试过L03/R02、L22/R17、单右02三例，加两次页面成功。新策略没有117全量统计。页面支持选择117任务实时计算，不自动预载全量。

## 10. 接手落实顺序与建议PR划分

先做本地保存与依赖清单，再由负责同步的工程师对目标分支做实际diff；下面是建议边界，不代表已创建PR，也不是完整跨分支差异审计。

1. PR-A：保存可复现研究基线。
   收齐入口、六段框架、球腕/LM/连接依赖、测试与必要配置；生成大资产采用清单/哈希/构建说明而非盲目全量提交。明确当前模型和旧路径依赖。保持行为不变，复现3例及六阶段回放。
2. PR-B：独立预接触与正向3cm策略。
   合入ForwardPrecontactPlanner、initial_motion扩展点、固定目标支持和顺序/接缝测试；确认不带回反推依赖、不回选上游。如目标分支尚无六段框架，PR-A应先提供框架。
3. PR-C：性能优化。
   补16/64形状真实预热、保留缓存、量化冷/热和跨拓扑开销；再单独优化polish执行。每个改动做同输入质量对照和显存峰值测试，不以减少实际随机候选来伪装性能提升。

用户目前没有授权本次交接直接提交/建PR；不要自动执行上面Git动作。

## 11. 验收清单

- 预接触IK先于Home路径；吸附IK发生在实际Home路径输出之后。
- 传递真实到位关节，不拿计划目标或历史帧假装实际输出。
- 无反向3cm依赖；前向帧不倒放；阶段接缝连续。
- 首次录图被区分为准备成本，热调用每次仍重新求候选和路径。
- 按双臂侧吸、双臂顶吸、单臂分别测，并跨布局切换；记录GPU峰值和OOM。
- 回放保留六段TCP位置和朝向；计时数据与回放同一结果，不串任务。
- 失败不自动重选、降低验收或输出未验证帧。
- 无全程携箱/导航/释放后障碍/实机验证时，明确保留上述边界。

继续性能定位可用diagnose技能；交接组织可用handoff技能。当前交接不涉及Linear跟踪和其他AI协调。
