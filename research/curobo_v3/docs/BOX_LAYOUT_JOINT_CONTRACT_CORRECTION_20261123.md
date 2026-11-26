# 双SKU任意合法位置抽取搬运：任务纠偏（2026-11-23）

本文件取代旧 `BOX_LAYOUT_FIXED_FORWARD_HANDOFF_20261123.md` 的任务语义。当前工作树已有大量其它修改，本轮不回退、不覆盖性能agent已有改动。

## 2026-11-25 最终当前结果

- 用户指定原25箱完整名义外沿：Y∈[−1.02,1.02]m、Z∈[0,2.04]m，初始箱心连续取值、按SKU半尺寸内缩；保留共X平面、高差≤.60m、D=.66m、阈值.79m、原SKU world−X抽离.300/.255m、用户14角与任务内自由升降。
- **最终当前运行码44/44全新输入通过**：边界8/8 + 原36/36（29同步/8顺序/7换手），进程外CPU44、首次实际吸附88（66Side/22Top）全过；1,176,114实际native采样点，107项相关CPU测试在最终源码同一进程合并通过。实际回中31个升降值[-0.940000,0.000000]m，最大14角误差4.371139e−8rad。原36中30例在新域内、6例为更大范围证据；与边界8合计38域内/6更大范围。
- 两个边界根因已共享修复：实际手臂/原物理ID分离并绑定场景、CPU、回放及目标预览；抽离原生margin校验与LM引导、已有≤8原生rest子段支持带位移合同的抽离（实际失败例用2段恢复，原5mm/2mm与所有完整箱体/足迹/载荷/历史门禁保留）。原失败整段、6/8及7/8批次均保留，没有继承它们的成功。
- 汇总：`generated/two_box_layout/original25_domain_20261125/acceptance_current44.json`。最终8：`run_003/acceptance_summary.json`；最终36：`../original25_current36_20261125/run_001/audit_summary.json`。8与36共用的1236个运行码/vendor/模型/kernel文件SHA完全一致；全36 source1445文件digest=`0792610110e60617901f3f648e1ce34315c054d92391d5f930311936fe22de0b`，收尾再核对0变化。输入/两个完整源快照均保留。
- 所有本任务GPU/CPU进程、子agent均结束；队列exit0、共享锁空、freeze已释放，无待运行工作，没有commit/push/实机执行。**有限44案和连续必要几何条件检查均未证明每个连续合法位置都能成功；未做CCD/实机验收，不据此mark完整任意位置goal complete。** 范围输入阻塞已解除，不再等待Y/Z确认。

## 2026-11-25 用户范围补充（优先于下方历史待确认描述）

用户明确：“按原有25箱的覆盖范围，新的两个箱体的范围不会超出中范围”。绝对域输入阻塞已解除，继续原目标。范围按原25箱完整箱体外沿恢复，连续位置均属任务范围，不把25个格点当成仅有合法位置；不恢复其余23箱或填充障碍。旧Y±1.20m/Z0–2.40m货舱提案保留为历史，未被用户采纳。

已按原`v3_task_scene.py`和`v3_wall_ik_benchmark.py`核对25格及92组fixture：名义完整外沿Y∈[−1.02,1.02]m、Z∈[0,2.04]m。40箱中心Y∈[−.82,.82]、Z∈[.20,1.84]m；高箱正常方向中心Y∈[−.7875,.7875]、Z∈[.291,1.749]m，Y/Z交换后Y∈[−.729,.729]、Z∈[.2325,1.8075]m。旧fixture的整体Z+1mm是数值离地偏移，不把上沿扩大到2.041m。原几何来源、SHA、CPU可重跑派生及36例范围成员检查写入`generated/two_box_layout/original25_domain_20261125/{domain_cpu.py,domain.json}`。实验设计/多次参数试验由subagent执行；主会话保留合同及证据索引。固定D=.66m、阈值.79m、原SKU抽离距离、用户14角与任务内自由升降均保持，范围确认本身不等于全域成功。

原25域内的连续必要几何条件CPU分析未建立两臂首次取箱或所有附件回中都不可能的证书；放松条件覆盖不等于实际连续任务可行性。最终8例边界与原36例均已在相同运行源码下完成真实默认native及外部CPU核验，详见顶部汇总；不同manifest digest来自输入/验证器范围，运行文件SHA一致。

## 2026-11-25 共享修复与当前验证

- **原25域边界最终8/8 fresh默认入口、进程外CPU8/8、16次实际首次吸附全过**，证据`original25_domain_20261125/run_003/{acceptance_summary,independent_cpu}.json`；最终源码107项CPU回归在同一进程通过，日志`cpu_final107.log`。原6/8与7/8失败批次全部保留；不继承其成功到本批。
- 并排精确接触的失败来自原标签分配的双臂接触不可达；共享默认入口先保留原分配尝试，再交换实际手臂，并用`owner_to_physical_id`在场景/附件/CPU核验/回放/目标预览中绑定原始物理ID。箱体原位、SKU与总质量不变，原换手构造器显式传递映射。
- 另一侧同侧极限换手失败是拟合曲线低于已有5mm底盘余量。共享抽离阶段现在使用既有native margin校验及对应LM引导；已有≤8原生rest分段机制扩展至有显式笛卡尔位移合同的抽离段，每段沿原轴使用实际几何端点位移，完整箱体尺寸不变。实测2段各4001点，最小底盘间隙5.215290/5.000187mm，内部q/v/a接缝误差0，实际Cube合计抽离299.999207mm；原违规整段仍保留。物理地面/碰撞/RNEA/吸盘足迹/原位清出不回入门禁均保留。
- 同一运行源码的原36例全新回归已在`original25_current36_20261125/run_001`完成36/36+外部CPU36/72实际首次吸附（mainexec54717、source digest`0792610110e60617901f3f648e1ce34315c054d92391d5f930311936fe22de0b`），含原6个更大范围案例；未读旧成功q/轨迹、未继承结果。冻结期间只改文档和分析产物；收尾已释放freeze。
- **上述有限验证及连续必要几何条件检查，均不等于连续全域成功证明、CCD或实机验收。** 合法域信息已由用户补齐，当前继续检验实现，不再等待Y/Z输入。

## 2026-11-24 历史阶段结论（以顶部2026-11-25更新为准）

- **纠偏实现及有限36案同源回归已通过，完整“任意合法位置”目标尚未证明。** 默认入口`tools/v3_box_layout.py::solve_file`保持两SKU原尺寸、初始共面、高差≤.60m，底盘固定；统一D=.66m（实际X=1.1680520510673524m），统一阈值.79m（低于Top、等于Side），按SKU world−X抽取.300/.255m。
- 回中直接使用用户给定左[40,−46,0,−100,−30,−75,90]°、右逐项取反；无Cartesian45°替代。updown由任务在实际URDF[−1,0]m内选择，36案实测25个不同值（−.94至0），最大实际14角误差4.371139e−8rad。
- **最终当前源码36/36真实默认fresh native通过**：24同步、7自动顺序、5自动换手；936086原生采样点，整批worker2633.816680s（约43分54秒，非单任务运动时长）。没有继承旧成功、重试筛掉失败或从旧成功q/轨迹构造结果；36份原raw逐项/字节保持。
- 进程外同fast环境CPU重开全部36实际数组通过（q/v/a、原位、实际抽离、载荷切换、source清出不回入与用户终点）。另独立从原箱C及actual native q/URDFFK重算72次首次吸附，55Side/17Top均符合79规则，实际arm caps10/11平面/足迹通过；最大TCP误差55.196μm、平面误差51.787μm，cap最小内缩1.76542mm。碰撞/完整OBB/动态RNEA等保留原native采样门禁，不外推CCD或实机验证。
- 自动换手支持L→R、R→L、Top/Side、先取上Cube再取下Tall；稳定physicalID与实际owner分离，同一箱静止支持转移总10kg，最终每臂各10kg。原坏候选/过界曲线被拒绝的记录保持；修复了角色/面轴、保留原A、source审计作用域、七轴退离1mrad规划余量、实际precontact admission、0位移持箱q和错误呈现，未降低URDF/碰撞/载荷/足迹门槛。
- 公共`curobo_core/native_spline.py`在独立kernel树修复局部常量样条数值，TrajOpt/TimeOpt都覆盖；共享vendor未改，CPU/GPU数值回归及76项相关CPU回归通过。1474当前源码/配置/模型/vendor/raw文件SHA全量一致，digest=`52bd2e0cf8b772d37d1797ef812f85c89083ceae077413212b478e1bab1728af`。
- 权威证据：`generated/two_box_layout/default_handover_final36_20261124/acceptance.json`、`run_001/{summary,terminal,independent_cpu36}.json`，以及`actual_suction_contract_audit_20261124/batch_final.json`。GPU queue `gpuq-a4225670b2fd4131`实际completed/exit0，owner/worker不在、共享锁空闲，freeze已释放。旧34/35准备目录及run001/002全部失败产物未改写。没有新GPU工作待做；未commit/push/实机执行。
- **剩余合同缺口：绝对合法Y/Z工作域未定义。** 用户给的共面/相对高差不足以闭合全域保证；`takeover_20261124/workspace_scope_bound_cpu.{py,json}`以实际URDF三角不等式作域不足说明（例在货舱外，未被声明为本意合法输入，不作为任务失败）。需用户确认是否采用整个箱体处于当前货舱内腔Y[−1.20,1.20]m、Z[0,2.40]m，或给出另一个箱心范围；不得自行按成功样本裁剪。`takeover_20261124/completion_audit_20261124.json`标记goal_complete=false，当前goal blocked等待此范围信息（同一缺口连续3轮复核，有限36及范围提案已完成，无活GPU）。

待确认域已经量化为`takeover_20261124/workspace_domain_proposal.json`（user_confirmed=false，未应用到planner）：若要求全箱在当前内腔，则当前XYZ方向下40箱箱心Y∈[−1,1]、Z∈[.2,2.2]m，高箱箱心Y∈[−.9675,.9675]、Z∈[.291,2.109]m，仍附加高差≤.6与不重叠等原约束。此范围从实际场景面推导，原36均在其中；不是按成功案例裁剪，也不是可达性/全域覆盖证明。用户尚未确认，不能把自动重复goal提示视为确认。

接管先读本节与`generated/two_box_layout/joint_contract_20261123/CONTINUATION.json`。以下保留历史阶段；旧阶段的“尚未实现/尚未通过”不覆盖当前实证。

## 2026-11-24 接管核验与实验分工

- 原始目标保持：两个SKU、初始箱心共用固定前向平面、高差≤60cm、完整抽离30/25.5cm、用户指定14角回中、updown按任务选择；有限样例成功不代表任意合法位置已完成。暂沿用已有实验选出的66cm/79cm政策。
- 新主会话重新读取默认31例实际原生数组并运行同步/顺序CPU检查，**31/31通过（顺序7）**；重新核对用户角度表，最大误差1.6292067961387602e−7rad，实际升降终点共21个不同值；另独立按原箱尺寸、所选面/roll重建吸附关系，原位吸附误差最高5.479e−5m、抽离向量误差最高5.736e−5m，未使用移动后的请求箱位冒充原位。旧验证快照内284个文件内容哈希仍匹配。证据：`generated/two_box_layout/takeover_20261124/baseline_audit.json`，可运行检查：`tools/experiments/box_layout_20261119/audit_takeover.py`。
- 本轮任务合同、布局、会话、顺序取箱、吸附几何和必要可达性CPU回归**71/71通过**：`generated/two_box_layout/takeover_20261124/cpu_contract_tests.log`。这里未重新执行GPU碰撞/动态RNEA，保留原原生门禁的证据与限制。
- 发现前一会话最后已保存但未写进本文的左取高箱→右臂预接触几何成功：`transfer_owner_transport_probe.json`，原右ID高箱实际由左臂支持10kg，原低箱保留；wall23.715s，CPU独立实际抽离向量误差6.724e−7m、各位置接缝0。此为几何前缀，不能算完整换手或搬运成功。
- **用户新增执行约定：需要反复设计实验/试验选值时使用subagent**，主会话保留原目标、结论、关键数值和证据路径，避免实验细节占据上下文引起任务语义漂移。当前接触/承重切换/退离由独立subagent在隔离子目录试验；生产源未修改。

### 2026-11-24 单物理箱换手几何已通过

- `generated/two_box_layout/joint_contract_20261124_transfer/transfer_probe.json`及`transfer_cpu_audit.json`均通过：沿用原Y85高箱的真实左取前缀，新增右臂接近、双接触静止支持切换、左臂60mm退离及后续10mm移动。实际原箱ID保持right，owner从left变right，低箱仍在原位世界中；只有真实平面和足迹核验通过的cap允许接触例外，其余机器人/实体箱OBB/world/base/ground门禁保留。
- 高箱实际抽离0.254999768m；q位置接缝全部0，切换时箱体世界矩阵跳量约2.22e−16。同一10kg分别由左右支持时，原.995静力门禁比最高.52297/.98303；静止支持分配两端及线性凸性通过。退离端10个donor cap全部离箱，最小间隙27.16mm，后续移动全程保持分离。
- GPU队列`gpuq-17556a7762fe468c`实际退出0，worker4.26s（队列持锁前台含启动9.47s）；使用fast curobo0.8.0.post1+fast76c2f4b6，CPU独立核验使用curobo0.8.0，实际运行环境与284个旧源文件哈希已记录。两次失败诊断保留；原固定swivel在J7接近+π时换分支，实验用固定J7的局部6轴IK获得连续退离，没有放宽碰撞/姿态门禁。
- 后续接口：`takeover_interface.json`保存实测右attachment、单物理箱C、原ID与owner、退离后q和NPZ字段。**这些仍是采样几何/静力证据，没有原生q/v/a，也不计为完整双箱搬运成功。** 当前继续独立验证取箱前缀原生、换手段原生，以及取第二箱和指定双臂14角回中的几何后缀；生产默认入口仍无换手分支。

### 2026-11-24 原生前缀数值失败已修复，隔离补丁下通过

`generated/two_box_layout/joint_contract_20261124_prefix_native/acceptance.json`保留真实失败：原owner_transport终点及最后边沿65点均通过原门禁；实际native候选20001点中，仅第19999点因updown=−1.0000001192092896m比−1下限低1.19e−7m而被严格joint-bound拒绝，其他碰撞分量均0。完整OBB/world/ground、动态RNEA、速度、jerk、源位历史与q/v/a接缝通过；候选运动13.8541s，但总success仍false，不能把“数值误差很小”直接计为成功。

根因修复已在隔离kernel目录通过：共享`bspline_context.cuh`用当前span首个control作局部参考，对控制点差值求位置/速度/加速度/jerk，令局部常量轴精确再现；普通TrajOpt与TimeOpt所有相关class aliases均指向隔离树。9组（3/4/5阶×三种basis）GPU结果与SciPy float64参考一致，负/正常量及全局起点不同的局部−1尾段精确，导数0。共享cuRobo的766个源/配置哈希未变，原284项目源文件匹配。没有改原输入、几何终点、门禁、输出数组或time scaling。

新真实重验`generated/two_box_layout/joint_contract_20261124_prefix_native/numeric_fix/attempt_02/acceptance.json`为success：20001点全部原生门禁通过（approach/contact/extract各4001，transport两段8001），运动12.990144s，prefix求解40.079s，workload42.391s；所有原位、.255m抽离、OBB、动态RNEA、速度/jerk、源位及实际q/v/a缝保留。旧失败仍在原路径。

可复用入口为`tools/experiments/box_layout_20261119/transfer_prefix_native/numeric_fix/isolated_kernel.py::install_kernel_tree()`，必须在fresh gpuq进程构建native资源前安装；默认已验attempt_02/kernels。该入口目前仍是隔离实验能力，正在提炼为不依赖实验产物的公共模块，尚未激活生产。

### 2026-11-24 Y85完整采样几何链已组合，未计作原生成功

后缀独立GPU/CPU检查通过：右臂保持实测attachment，停放高箱后左臂取原低箱并抽离0.299998462m，再双臂按用户14角回中，最终updown≈−.39m。后缀1407帧，原.995静力门禁最高比例.98814，独立CPU力矩误差最高1.81e−5Nm。原前缀、换手及后缀输入/旧产物哈希保持。

主会话新增可运行`tools/experiments/box_layout_20261119/audit_transfer_chain.py`，交叉组合三个真实组件为13段3116点（移除重复接缝点）：核验稳定物理箱ID→owner、未取箱仍在原位、吸附/切换时实际FK箱C连续、原高箱.255m/低箱.300m抽离、静止支持切换及全部q位置接缝；最终14角最大误差4.371139e−8rad。证据为`generated/two_box_layout/takeover_20261124/y85_geometry_chain_audit.json`和同目录NPZ。

**该链没有native时间、速度/加速度或完整动态门禁，不是生产默认入口成功，不计为第32个完整成功。** 原生前缀样条数值修复、换手原生段和五段后缀原生验收分别由三个隔离subagent推进；随后仍需组合真实q/v/a和源位/载荷历史并接入新鲜自动规划。箱心绝对Y/Z验收边界仍待用户答复，未擅自按环境内壁缩小原目标。

### 2026-11-24 三个组件及缓存原生整链通过，新鲜原生入口仍待完成

- 原前缀：隔离样条补丁下20001点通过，运动12.990144s，见上述numeric_fix验收。
- 换手段：主会话使用同一已验补丁调用原`probe_native.py --attempt patched_06 --optimize`实际退出0；三个运动段均optimizer_success=True、source=trajopt、所有updown_min精确−1。worker42.778s，含1s静止支持转移总3.212450s；GPU/独立CPU的全碰撞、实体OBB、真实cap平面/足迹、动态RNEA、source与静止接缝均通过，CPU证据为`joint_contract_20261124_transfer_native/patched_06/cpu_audit.json`。
- 后缀：`joint_contract_20261124_suffix_native/attempt_01`五段/六真实native pieces通过，24001点、运动13.011309s、worker52.619s；实际低箱X抽离−.299998420m，用户14角误差4.37e−8rad，q/v/a缝0，双source清出无回入。最终transport优化候选拒绝，库合法保留原seed后经genuine native validator、TimeOpt和全部实际门禁通过；**不声称全部优化候选成功，也不额外添加此非物理门槛**。额外美化该标签的补验已取消，失败/取消记录保留。
- 新公共`tools/curobo_core/native_spline.py`已提炼相同字节hash的隔离数值修复，CPU准备/缓存/拒绝未知源检查与相关回归72/72通过；原生产入口尚未启用该模块。

缓存组件的真实整链CPU核验现已通过：`generated/two_box_layout/joint_contract_20261124_native_chain/audit.json`及`native_chain.npz`保存13段56011点，严格递增实际时钟29.213902354s（含1s静止支持），q/v/a最大缝3.9434e−10/0/0。独立URDF轴/实际箱C重算空→左10kg、左10kg→右10kg、右10kg→左右各10kg三次静止力矩跳，误差9.438e−6/1.432e−5/2.808e−6Nm；未变载荷缝力矩差0。单箱总质量始终10kg直到真实低箱吸附；稳定ID/实际A/未取箱、全箱world/ground/pair OBB、source历史及14角通过，原阶段数组逐个保留未修补。仍明确为cached-component candidate，不是fresh/default/universal成功。

独立新鲜几何入口`tools/experiments/box_layout_20261119/transfer_fresh/plan_fresh.py::plan_document(document,output,report)`已从原请求重新算完整13段3128点，worker51.003s、GPU退出0、CPU通过；CLI历史JSON/NPZ/NPY读取审计及源代码审查确认未读取旧成功关节/静态IK候选/轨迹。证据`joint_contract_20261124_fresh/attempt_03/`。当前继续一次原请求→fresh几何→完整native的入口与公共CPU verifier/replay适配。

默认/回放hook现已应用：`v3_box_layout.solve_file`在创建native资源前启用公共隔离数值修复，原同步/顺序候选失败后自动调用`v3_handover_layout.py`新鲜入口；CPU verifier和native reader按新workflow分派，回放按当前URDF配置与每阶段稳定物理ID/实际parent/A计算单帧箱姿态。相关73项CPU回归通过。补丁及前/后源hash在`takeover_20261124/default_handover_integration.{patch,json}`。

**实际默认入口原Y85已通过**：`takeover_20261124/default_y85/result.json`与`external_cpu_after_default.json`记录原始输入逐项相同、六个同步候选失败后自动handover成功、13段56011点，总时长30.663664341s（含1s静止）、worker149.199690s、GPU退出0；最大14角误差4.37e−8rad，updown≈−.39，历史解读取为空。该证据使用当前默认入口和公共kernel树，不依赖旧成功轨迹。

完整目标仍未完成。继续源追踪发现低于79cm的handover顶吸被旧suffix附件校验“按手臂写死平面轴”限制；正在按实际attachment平面修正并验证低箱顶吸、原顶吸附件移到中间位时保持不变。CPU已验证旧side结果保持、真实top/错平面/足迹越界行为；尚未计入完整Top native成功。主会话同时去掉verifier对SKU/高度顺序的额外限制，两次抽离距离由对应物理箱的真实SKU推导，原cached链及合同7项CPU检查保持通过。完整32回归harness已准备，暂缓提交，等待这些原目标内的必要修正后冻结一次运行；未擅自缩小Y/Z工作域。

### 2026-11-24 低箱顶吸/上箱侧吸换手已通过；双顶吸完整原生仍待完成

共享suffix附件验证现按实际`tool_to_box`推导面轴/符号/足迹，保留1mm/1°与完整cap条件；原两个side附件数值保持，默认真实top/roll0通过，错误面/足迹拒绝。中间donor目标改为`C_mid @ inv(A_source)`，保持原吸附关系；原side三个中间pose数值不变。

原始新输入Y=.85、40箱Z=.70、高箱Z=1.30（共同X1.168052051/D66、阈值79、高差.60）完成fresh13阶段native，52011点、运动31.679958s、worker132.280s、GPU退出0；主会话再独立CPU核验通过，实际低箱吸附法线在箱坐标系为−Z，确为顶吸。最终14角误差4.37e−8rad，updown≈−.56，与原side案例−.39不同，未固定回中升降。证据`joint_contract_20261124_top_handover/attempt_01/{record,external_cpu_main_current}.json`。

同Y85、40箱Z=.20/高箱Z=.70的双顶吸原始失败保留：固定J7局部6轴退出曾在4mm拒绝。新全七轴有界IK和退出可行性前置admission已解决该死端，原21-lift格点自动拒绝−.70/−.65，在−.60获得8个全程退出可行候选，未手工固定lift或修改输入。

首轮新GPU246.926s仍在native前被额外的extract单段“必须已完全清空source”审计拒绝，实际robot/OBB/world/cap/10kg检查通过。当前批准最小source_scope_proposal.patch：同一完整物理检查及原策略用于extract+owner_transport共同历史；原transport自身的post-extraction残余参考+1um/no-deepen/no-return gate、native source_progress及最终全链原位历史均不变，抽离长度和姿态门槛不变。正在用剩余一次有因GPU重验原请求；失败不是非法/全局不可达。

最终回归保留原32并追加这两个原始Top输入共34，成功及当前失败案都在清单内，无expected-failure豁免。harness尚未提交GPU，待本次必要根因尝试收束再冻结一次运行。

### 2026-11-24 双顶吸原请求通过，34回归在缓存目录冲突处停下

双顶吸同Y85、40箱Z=.20/高箱Z=.70的原请求已在第二次有因新复验完成13阶段fresh native：`joint_contract_20261124_top_handover/attempt_04/record.json`为success/physical/task均true，60011点、运动39.639646590s、worker386.130579s、GPU实际退出0；主会话当前core再核验通过（`external_cpu_main_current.json`），最大14角误差4.37e−8rad，任务选择updown≈−.54。原输入与全部loaded source hash一致，source清出无回入。旧局部IK/单段审计失败完整保留，不以其判布局非法。

`default_handover_regression_20261124/run_001`按单个gpuq冻结1396文件运行：原31全部31/31通过，第32原Y85执行失败后按请求边界停止，第33/34原样保留not_started。原始请求不变、源digest始终abc5b1557bef16b6e8099d656eb67bf9e8321173627769b1d09329ddf276bbbd；GPU exec11953实际终态1，无source drift。

**第32真实根因是harness编译缓存路径冲突，不是几何/物理不可达。** `handover/record.json`显示fresh几何已过，但历史解读取guard将`runtime_cache/triton/.../__grp__triton_red_fused_sum_1.json`编译metadata误判为旧解；顶层保留的早期assigned-reach文案不能当作换手失败根因。当前完成整批独立CPU槽位审计后，仅将编译缓存移到`generated/default_handover_runtime_cache_20261124`（geometry history命名空间外），保留run001所有失败，计划source-equivalent continuation只补验32–34，再外部逐案重开全34真实数组。不会无因重跑已通过原31或用未运行案充通过。

为继续“任意位置”目标，负Y同侧的右取左接角色适配已准备在`role_generalization_20261124/`为未应用CPU候选，实际URDF左右链重新解，不镜像旧q；当前冻结期不改生产源、不启动新GPU，也不声称负Y成功。

### 2026-11-24 run002真实关节越界已定位，margin修复在CPU通过

run002已实际终态失败并独立CPU全槽位复核：继承父31实际通过，第32拒绝、第33/34未运行，1396文件源快照无变化、freeze已释放；原run001失败保留。编译缓存冲突已消除，真实新拒绝发生在donor_release：全7轴几何退出余量仅76.4μrad，拟合control越界244.4μrad、实际曲线约2.84μrad（GPU峰leftJ7=3.141595602），15/4001采样越URDF+π，门禁正确拒绝；完整OBB/world/RNEA通过，未因此降低关节限制。

只在donor七轴IK规划bounds内预留1mrad，边界起点作为该候选不合适而非任务非法；未改URDF、native gate或修补输出数组。CPU新Side几何/control/曲线余量约1.21/.792/1.10mrad，BothTop曲线余量48.0mrad，原7轴pose/OBB/cap/world/source/静力检查通过，证据`retreat_native_margin_20261124/cpu_margin_report.json`。原rawY85默认fresh GPU重验随后已实际成功：`retreat_native_margin_20261124/default_attempt_01/default/result.json`为success/physical/task全部true，52011点、运动29.890827835s、worker171.281524s；main进程外当前CPU复核`external_cpu_main_current.json`通过，14角误差4.37e−8rad，updown≈−.39，旧失败全部保留。

同时修复默认错误呈现：失败的换手分支现在报告真实handover失败原因，原同步failure保存在failure_before_handover；不再让旧assigned-arm文案掩盖新拒绝。45项相关CPU回归通过。新源码已变，不沿用run002“1394等源”的声明；负Y角色补丁正在按当前源重基并保留本margin修复；随后用实际URDF两链新解负Y默认原请求，不镜像旧q。尚无负Y实际GPU成功证据。

### 2026-11-24 反向换手角色适配已应用，实际负Y前接触仍待解决

负Y请求现在由实际右臂取高箱、交给左臂，再由右臂取剩余40箱。稳定物理箱ID、真实chain index/caps/质量模型与最终用户14角保持，不按箱ID指定最终手臂。`OrderedLayoutPlanner`可选owner→physicalID映射的默认原行为保持，1mrad donor余量及Top/source-scope修复保留。Main选取75项CPU回归通过，日志`takeover_20261124/cpu_role_integration_tests.log`。

原负Y85请求（40箱Z=.8/高箱Z=1.4、D66/79）实际`role_generalization_20261124/default_attempt_02/default/handover/record.json`仅完成fresh suffix的upper_park，有限前4个precontact候选拒绝，native0段、success=false。实际过滤候选/路径诊断随后证明：前4对共享同一个right giver腕部边界姿态，仅recipient冗余变化；固定swivel同pose反解换腕，关节插值违反真实target contact门禁。实际377点/347拒绝由CPU按原cap11规则精确复现，基础self/world/界/地面检查均0，未发生物理ID/载荷sphere索引混淆。

最小纠正仅复用已有`diverse_contact_order(contacts,giver)`，在不变的前4次尝试前按实际giver族排序，不新增grasp/候选/预算或改门槛，原腕翻转路径继续正确拒绝。这次唯一有因物理修正的原负Y默认fresh重验已实际终态失败：gpuq-57dc06176f8b4e29退出1，worker48.185898s，四个precontact全部False、native0段，1402冻结源未变，无活GPU。完整证据`role_generalization_20261124/terminal.json`，仍为有限搜索unknown。

main复查确认`diverse_contact_order`只交错joint4肘部正负，不保证通用giver去重或腕分支覆盖；此前将它当完整posture多样性建议不充分。后续3文件最小admission补丁已获main审阅并应用，CPU检查通过：共享precontact对0位移臂保留实际q；按实际giver7关节分组轮转（不丢recipient变体）；所有原128候选必须过原precontact gate、完整OBB/static/source阶段检查才可消耗最多4完整trial，所有筛查仍算原110s deadline。原坏4路径依旧被拒、admitted=0、完整trial消耗0，未改grasp/预算/物理门槛，hold skip单独证明且不冒称治giver腕根因。

当前先准备一次局部checkpoint物理筛查（旧实际状态仅诊断，不计fresh构造输入），只有筛出真实可行路径才允许一次原负Y默认fresh完整重验；零admission停止，不反复跑wholeprefix。证据`role_generalization_20261124/precontact_admission_cpu/{precontact_admission.patch,cpu_applied_results.json}`，尚无该新admission的GPU通过结果。Positive新源GPU回归仍待，旧positive证据不替代当前源。

11项需求审计见`takeover_20261124/requirement_audit_current.json`，当前负Y、最终同源整批及任意绝对Y/Z域缺口都未证明，goal_complete=false；审计源hash不自动替代后续修改的实际验证。

### 2026-11-24 负Y反向换手默认fresh13原生已通过

局部screen首次实际物理admission已从原128候选中接受4条完整可行预接触路径（8.10s），没有改原110s deadline/4完整trial/候选数/grasp或门禁。模块导入/JSON记录的启动失败单独保留，不计物理零admission。screen旧状态只作诊断；之后另开原请求的新鲜默认规划，历史成功解读取为空。

`role_generalization_20261124/default_admission_attempt_07/default/result.json`现为success/physical/task全部true：负Y85、40箱Z=.8/高箱Z=1.4，D66/79，13阶段52011点、运动28.470185s、默认worker221.909718s、GPU实际结束。Main当前core进程外重开真实数组也通过（`external_cpu_main_current.json`），原物理高箱wall_box01实际Rightgiver→Leftrecipient、另一箱wall_box00随后由Right取走；两箱原world−X抽离约.254999755/.299994312m，三次重力矩跳CPU误差最高2.18e−5Nm，14角误差4.37e−8rad，updown≈−.39。全部原尺寸/箱位/ID保持，无旧解读取或关节镜像。

76项CPU回归通过。最终整批仍未完成：该SKU先后角色互换现在也已实际fresh默认通过（68011点38.207087s）；接下来一次冻结当前源跑原34+负Y+该互换输入共36。新case成功或失败均不删除；不把目前34历史/单例成功凑成当前全域证明。

### 2026-11-24 40上箱先交接/高下箱后取的真实默认验收通过

新增原请求正Y85、40箱Z1.4、高箱Z.8、D66/79（未添成功面/关节/流程hint），当前源码无需再改就由默认入口fresh完成13阶段native：source实际为wall_box00的40箱，Leftgiver→Rightrecipient后，Left取pending wall_box01高箱。实际world−X抽离分别.299999653/.254999557m，稳定ID/实际A/单10kg支持/最终每臂10kg及全source/qva/碰撞OBB/足迹/RNEA门禁通过；没有把旧Tall成功只换标签。

证据`sku_order_handover_20261124/default_attempt_01/default/result.json`与main进程外当前core复核`external_cpu_main_current.json`：68011点、运动38.207087s、默认worker163.451189s，14角误差4.37e−8rad、updown≈−.10，历史解读取为空。`terminal.json`证实gpuq-8e6ff27a2ce547dc completed退出0、1403冻结源未变、进程均不在，无额外GPU。

最终清单现保留原34、原负Y和这个原SKU顺序互换输入共36，前35顺序/请求/字节完全不变。Dirac仅获准准备/激活2个suite侧车脚本并CPU预检/新源快照，尚未mainGO提交长批次；运行后必须外部逐案重开全36实际数组。当前单例成功与历史原31通过不拼成新同源全36通过，更不构成任意合法位置全域证明。

## 2026-11-23 补充：单物理箱双接触静态候选，不计为交接成功

按用户给定几何条件保留更宽Y=.85输入，未将自动goal继续视为缩小工作区的授权，也没有把该输入判为非法。为推进这一剩余类别，另做只读/仿真静态交接候选探针。

`one_box_transfer_contact_probe_v2.json`对两种SKU、三处中间箱心（.85,.20,1.20）、（.90,.15,1.20）、（.95,.10,1.25）共6组，都找到实际机器人碰撞/实体箱体/两工具面接触一致的静态候选，每组保存8个，共48个。待取另一原箱仍在世界中。机器人自碰撞和世界/地面检查不移除；同一箱作为一个OBB，只有经实际接触平面核对的两终端cap球获得接触例外，其他机器人球仍做完整OBB相交检查。

这不是把同一箱复制成左右两个20kg载荷。相同关节构型分别检查同一10kg箱完全由左臂、完全由右臂支持两种状态，保存实际14轴重力矩；最大原.995限制比例左约.5574、右约.9877。`transfer_static_contact_cpu_audit.json`对48个真实候选重新用URDF验证双工具箱姿态一致/代理足迹，并由关节轴到箱心的重力矩差核对支持状态变化，最大误差约1.95e−5Nm。

静态候选只说明中间接触/承重目标有希望，不证明从Y85取箱到该处、接收臂接近、真换吸释放、捐出臂退离、动态载荷共享或全原生交接已经通过。主程序目前仍无换手分支，不能把这些48候选计入原31成功或完成声明。后续若继续该类别，必须保持物理箱ID与arm owner分离，使用真实轨迹和静止交接缝；不能删箱、复制箱或瞬间改tool_to_box造成功。

## 用户目标与实现约束

- 两种尺寸标签分别为40×40×30、46.5×25.5×58.2cm；继承几何轴定义XYZ=(.300,.400,.400)、(.255,.465,.582)m。X为抽取深度；若切换长高，仅交换Y/Z。
- 两箱中心共享同一个X平面。平面距离以机器人底盘**几何最前沿**为起点，距离D∈{.60,.62,…,.90}m；同一次请求固定，不移动底盘、不随箱高或SKU改变。
- 当前实际URDF base_link三块碰撞网格最前沿world X=0.5080520510673523m。因此Xcenter=0.5080520510673523+D，不等于直接把world X设置为D。来源为实际网格变换后的顶点最大X；代码缓存只读CPU计算。
- 两箱中心高度差≤.60m。不通过挪动请求位置或缩小高差来制造成功。完整箱不得入地或相互重叠；不添加按成功样本裁定的Y/Z范围。
- 当前统一侧吸面中心阈值为.79m：Z<.79m用顶吸，等于.79m仍侧吸；历史.80m阶段记录仅供追溯。
- 吸附后沿world −X抽取，40箱名义.300m，高箱名义.255m。保留2mm/1°位姿门禁；实际误差必须从轨迹FK核验，不把名义数值冒充实际值。
- 回中终点左臂关节角严格为[40,-46,0,-100,-30,-75,90]°，右臂逐项取反。**直接构造关节目标，不做回中Cartesian IK，不用世界Ry=-45°代替。**
- updown属于共享第15轴，逐任务在实际URDF[-1,0]m内选择，本轮用1cm候选网格；不是固定回中值，也不声称有限候选网格证明不存在其它解。最终选择值与关节误差写入结果。
- 完整箱/世界/机器人碰撞、每箱10kg、原生TrajOpt/TimeOpt/RNEA、实际q/v/a拼接与源位过程门禁继续检查。原生每阶段4001点是采样验证，并非连续碰撞证明。

## 旧门禁冲突的处理

旧世界Ry=-45°/回中XYZ适配策略已退出本任务规划。单臂旧实验的BoxPick默认仍保留以便历史数据回放，当前双箱入口与planner强制关节合同。

用实际URDF对用户指定角度做CPU FK，末端工具约为X=.728693、Y=±.743773m（updown=0时Z≈1.412151m）。按实际吸附关系带箱后，旧球模型水平包络半径约1.41–1.52m，必然超出旧1.05m圆柱。updown不能改变此水平半径。

因此新关节合同将旧1.05m圆柱作为诊断，记录 `enforced=false` 与 `within_legacy_limit`；不降低实际碰撞、载荷、关节/速度/力矩与源位门禁。旧Cartesian合同仍执行原圆柱门禁。不能声称新任务通过了旧1.05m门禁。

## 已有证据及当前未完成项

- `generated/two_box_layout/joint_contract_20261123/mixed_side_initial/`：D=.68m，左40箱/右高箱，Y=±.5m、Z=1.02m，全流程成功；不是任意位置证明。
- 首例worker墙钟86.939s，实际原生运动5.975875s，4001×4阶段拼接16001点。最终updown=-.7200000286102295m，双臂关节最大误差1.6292067961387602e-7rad。
- 首例独立CPU FK：左/右抽取位移误差分别3.7081e-8/3.9805e-7m；最终箱位姿由指定关节角与实际吸附关系推得。原生物理及源位门禁通过。旧球模型末态半径1.46202354m，仅为诊断。
- 新CPU合同回归2项通过；编辑器/会话40项通过，历史不等X的可运行输入目录与例子已明确迁移为新共享平面。生成目录历史结果不修改。
- `distance_catalog.json`列出16个D×3个固定布局的48请求，含混合侧吸、40箱顶吸/高箱侧吸60cm高差、40箱侧吸/高箱顶吸60cm高差。该批次现已结束，46成功/2未通过；失败不删除，也不自动称为非法布局。
- 距离选择预先约定：首先最大化三个完整任务成功数；并列时比较最差原生力矩利用率，最后比较D。不用耗时最短替代搬运可用性。
- 早期D=.68m是暂定值；现经48请求标定选定默认D=.66m。旧44个成功不参与新合同通过统计。

“任意合法位置”仍是目标，不把有限48请求当成全域证明。用户给定的平面/高差条件没有单独指定Y/Z外边界；不能据此声称无限横向或高度都可达，也不能把规划预算耗尽称为非法。后续应继续挑战箱型左右交换、阈值两侧、触地高度、横向非对称/上下堆叠，并对每个失败区分明确几何不可能与求解尚未完成。

## 终点筛选修复与正在执行的标定

首轮`distance_calibration/`在D=.60m、40箱顶吸/高箱侧吸且高差.60m处，反复搜索无法清出原位的回中终点。独立CPU按1mm升降采样1001点无共同清出终点；这只是采样证据，不是连续不可能性证明。已主动终止该进程并保留全部日志及`STOPPED_FOR_FIX.json`，不把中止当成完成失败。

修复将**原有最终原位清出门禁**提前用于指定关节回中候选过滤，没有更改碰撞或过程规则。当前`distance_calibration_v2/`重新运行48请求；source_v2保存运行源码快照（manifest校验），exec会话63216。独立CPU的`audit.py`另外用硬编码的用户角度表、原始吸附面关系与URDF重算成功结果，避免仅复读planner通过标志。

当前进度与后续事项写在`generated/two_box_layout/joint_contract_20261123/CONTINUATION.json`；它是观察快照，应以活进程与最新summary为准。不得因为观察超时另启重复GPU批次。当前目标保持active，未宣称任意位置完成，未固定最终D。

## 距离不能只按代表成功数选择

`distance_contact_reach_screen.json`采用实际URDF与现成接触可达性必要条件，在每个D上筛查同一组360组合（左右SKU四种、Y=±.25/.4/.55/.7m、Z=.3/.4/.6/.79/.8/1/1.2/1.4/1.6m、高差−.6/0/+.6，过滤入地和初始实体重叠）。CPU完成16×360组合。D=.90有48项明确接触不可达证据，D=.88有2项，D=.60–.86未触发该必要条件。后者不证明可达或全流程成功。

因此三代表布局的成功数/力矩余量仅用于初筛，不能单独决定D；需要同时检查边界工作空间覆盖，不能为让三例全过选取损失大量边界位置的距离。尚未固定最终D。

## 连续升降原位终点分析（本轮新增）

`source_lift_intervals.py`在用户指定14个关节角固定后，将附箱OBB与原位OBB的SAT不等式化为共享updown的标量区间。箱体沿实际URDF升降轴作平移，姿态固定；该分析覆盖整个[-1,0]m升降范围，不是1cm候选网格。实现用100组随机箱位×100个随机升降点，与另一个向量SAT实现核对，全部一致；浮点区间端点不是严格区间算术证书，也不覆盖完整碰撞/动力学/路径。

`distance_source_lift_intervals.json`核对原48代表任务：D=.60m的40箱顶吸/高箱侧吸60cm高差无共同原位清出区间，与原CPU1mm样本和GPU候选过滤一致；D=.62同任务共同清出区间总宽约.0235m，D=.68约.1439m，D=.74约.8220m。这解释了部分距离的升降余量差异，不能当成规划成功数。

`distance_workspace_screen.py`进一步把相同360组挑战输入在16个距离上做接触必要条件及连续升降原位终点筛查。D=.60/.62/.64分别有58/36/14组没有共同原位清出区间；D=.88/.90分别有2/48组触发明确接触不可达证据；D=.66–.86未被这两项必要条件排除。此360布局不是用户全域的替代定义，尚未验完整碰撞/载荷/全过程，不得声称66–86cm全域成功。

额外CPU接触可达性/原生轨迹恢复回归20项通过（cpu_reach_native_tests.log）。保存`carry_joint_fk.png`，由两个实际已接受的原生末态数组做CPU URDF FK，显示前/侧投影、关节原点示意及完整箱体边线；对应updown分别约-.72和-.01m，角度均来自用户表。PNG完整性/1760×1600尺寸验证通过；当前图片查看工具不支持图像输入，没有声称做视觉目检。

## “任意合法位置”的真实范围缺口：同侧上下堆叠反例

用户给定的共面/高差条件本身不足以保证当前双臂直接接触取箱流程可解。新反例：两箱中心同为Y=+.50m，40箱Z=.80m、高箱Z=1.40m；中心X按每个允许D共同设置。高度差恰为.60m，完整箱竖直净间隔.109m，无箱间重叠、不入地。另用实际environment.json对16距离×两箱做CPU AABB核验，全部没有环境墙/地/顶实体重叠（same_side_environment_check.json）。

`same_side_assignment_witnesses.json`枚举16个允许D及两种左右臂分配，共32请求。实际URDF接触必要可达性条件在完整共享升降[-1,0]m范围内均给出右臂不可达证据；不是有限IK搜索耗尽。D=.60m原分配中，右臂腕到受joint2限制的可行肘区距离下界约.746275m，前臂长.474m，含约.010098m位姿/几何包络后仍有约.262178m缺口；交换分配仍有约.280252m缺口。距离调参不能修复这个反例，也不能通过将两箱交换左右臂解决。

这证明当前**固定底盘、面中心接触且采用既定法向、左右臂各自直接接触并持一箱**的模型不能只凭上述两个输入位置条件承诺“任意位置”。没有把该反例擅自删出用户范围，也没有宣称对包含分次取箱/中途交接的不同流程作了不可能性证明。

该阶段曾提出工作区/流程澄清问题。后续已通过改变侧吸箱面完整解决此同侧样本，因此不再把那一问题或固定前面模型的排除当成同侧限制。仍需继续验证更广工作区和更低堆叠，不能把有限成功替代完整目标。

永久CPU回归`test_layout_joint_contract.py`新增该反例，3项测试全部通过；会持续防止把几何输入合法与双臂实际可达等同。

### 阈值自由度的补查

用户允许调整侧/顶吸切换阈值，因此另加`same_side_threshold_witnesses.json`。对于反例中固定的两个高度.80m和1.40m，任意单一全局阈值仅能产生三种模式：两箱侧吸、低箱顶吸/高箱侧吸、两箱顶吸；用阈值.80/1.00/1.50m分别代表。16距离×2分配×3模式=96请求，在实际URDF全升降范围的必要可达性条件下全部被排除。并非仅默认.80m阈值导致，同侧直接双臂取箱的范围问题仍在。永久CPU回归已扩展为96请求，合同测试3项通过。

## 近地高箱失败的根因修复：搜索阶段载荷静力约束

旧固定66cm挑战首请求是左高箱中心Z=.30m顶吸、右40箱中心Z=.90m侧吸，Y=±.50m。不是三个基线之一。16候选均能走到native transport但最终被拒，worker490.94592s；日志243条静态力矩诊断，最高1.08863倍额定。保留`fixed66_challenges/tall_near_ground/`及停止说明。

根因是加载双箱的搬运RRT与原生重分段种子检查只按原碰撞/源位约束筛选，直到TimeOpt才发现中间姿态不能静止持箱。修复在`BoxLayoutPlanner.path_validity`复用现有FastThreeLayerPlanner/TorqueModel的真实RNEA、10kg/箱、实际tool_to_box，将零速度/零加速度力矩按原.995 planning scale筛入**搜索提案**；回中关节候选与抽离终点也检查可静止持箱，后者保留`extracted endpoint`重试原因以触发已有反向抽离恢复。没有新增自造动力学模型，也没有放宽最终原生TrajOpt/TimeOpt/RNEA、碰撞、源位或关节终点门禁。

这是静止可保持的保守搜索提案约束，**不是“任何动态补偿路径都不可能”的判据**；相关拒绝与搜索耗尽不能据此把用户输入认定非法。代码已用ponytail注释记录此上限。

`gravity_search_focused/`四个新鲜请求全部成功且独立CPU audit全部通过，exec18961实际退出0：近地高箱63.180s/首候选通过；左40顶/右高侧22.018s/首候选；左40侧/右高顶74.670s；双侧吸22.777s。这用于验证功能修复，不当作严格性能A/B。近地请求共有121649个静力查询提案、14567被过滤，最终updown≈−.12m；旧回中角未变。

`source_gravity_focus/manifest.json`与四例实际source SHA一致；最新`source_gravity_final/`另加抽离终点预筛和说明注释，对应正在运行的同19输入复测。CPU回归87项通过，含实际重力过滤函数的512+3分块、实际附箱参数传递、非有限/超限拒绝、原updown力约束政策保留，以及搜索物理/源位/重力掩码叠加。GUI拖拽也已限制active_axes=(False,True,True)，不能把箱心平面拖离固定X。

## 19挑战终态与五个失败的保留

`fixed66_gravity_challenges/`完整结束：19请求14成功/5未通过，worker累计370.326321s，exec12832实际退出0，独立CPU audit14成功全部通过（5未通过/0pending）。14成功覆盖三基线、近地高箱、左右SKU互换、双高箱、长高交换、阈值.799/.800/.801、双顶吸、左右非对称；不是全合法位置证明。

失败全部列于`fixed66_gravity_failure_catalog.json`：高位Z1.6/2.2m（16次抽离LM无准确解）；40箱触地Z.2m（9次起点碰撞/5次插值边碰撞/2次抽离末态base-ground margin）；同列低箱顶吸（0接触候选，恢复池亦未找到）；同列双侧吸（5次预接触拒绝/11次LM无准确解）；同侧面中心模型反例（必要接触可达性排除）。前四个有限失败不称输入非法；第五个也只针对当前接触模型。

`remaining_failures_trace/`四个未解请求补存每次接触关节值、最后有效抽离关节值及失败节点汇总，exec10313退出0（脚本正常结束，不代表请求成功）。高位第一失败在63/127≈.49606后，updown≈−.000945m逼近上限、左joint4≈−2.52571rad逼近−145°限位；旧12个恢复提案只在lift0/−1附近，均无末端解。同列双侧吸的失效关节则包括left_joint2≈−89.79°/right_joint2≈+89.79°及right_joint6≈−109.78°，完整升降nominal endpoint池仍无候选。

## 高位抽离恢复漏搜中间升降：已完成真实全周期修复

`nominal_reverse_probe.json`证明：精确用户要求的抽离末端，在全[-1,0]m的5cm升降/10°swivel网格上得到256个有效末端候选，升降值−.50/−.45m；首个候选反向共享升降LM完整成功，完整箱审计和原位接触核验通过。这是几何探针，不单独当成全周期通过。

运行修复在`reverse_extractions.proposal_stream`中保留原提案优先，再加入精确抽离末端的完整升降网格恢复，之后仍保留原细网格/容差/几何提案。不更改箱心请求、抽离距离、位姿容差、回中角或最终物理门禁；明确5cm/10°有限格点上限。

`nominal_lift_recovery/`真实全周期定向复测：高位Z1.6/2.2场景成功，worker87.70s，接触来源reverse recovery，最终updown≈−.50999999m，独立CPU用户角度表最大误差1.6292068e−7rad，实际抽离和原生各阶段/动力学/碰撞/源位门禁全部通过。该批次另一个同列双侧吸请求仍未通过（15.93s），未删除，exec55416实际退出0（1通过/1未通过），独立audit1成功通过。不能将不同版本结果合并冒称最新源码19/19通过。当前源快照`source_nominal_lift_recovery/manifest.json`与该真实高位结果SHA一致，CPU回归87项通过。

## 同列低顶吸为何需要检查顺序/接触模型

`stacked_contact_fixed_tool_proxy_probe.json`只用实际URDF中固定的link7→tool0变换与原碰撞球代理，对两同列请求做独立CPU检查。同列双侧吸的link7与另一箱均无重叠；同列低箱顶吸时，左link7有4个代理球与上箱重叠，最大27.1184mm。这与0接触IK候选相符，表明当前同时面中心吸附模型受阻；不能靠更多同一种IK重试修复，也不等于顺序取箱或其他合法吸附点不可能。后续应先用真实足迹/载荷检查偏心吸附的自由度，再评估先上箱后下箱的分阶段取箱，而不是缩小用户箱位范围。

## 偏心吸附、缓存与地面接触修复的本轮证据

`offset_grasp_probe/`首六个显式候选全部未通过，不能称偏心吸附一加入就解决同列。首个同列末端专项 `stack_grasp_endpoint_probe.json` 后来发现单planner底层碰撞缓存没有按真实attachment更新，整批标记 `valid_for_physical_conclusion=false`；不采用其任何物理结论。

根因修复：`curobo_core.backend.prepare_task`在原scene/task/box-fit资源键后加入不可变真实 `task_attachments`，因此同一箱位的offset/roll变化也清空empty/loaded checker和IK缓存。`stack_grasp_endpoint_probe_v2.json`重新跑8型，逐型断言checker的tool_to_box等于当前请求；只有`vertical_outward8cm`留下60个完整碰撞/实体箱/native-ground有效名义末端候选，lift≈−.65m。这仍不是全周期成功。随后`vertical_stack_grasp/`真实全周期中，同列双侧吸偏心型完整成功117.73s并独立CPU成功；一个70cm阈值的低同列侧吸校准仍失败，保留原结果。

触地修复依据：冻结40球模型最小localZ≈−.2m，40箱中心Z=.2m恰好贴地；旧strict球地面0与native所有球2mm净空会排除此合法支撑状态。仅当前双箱checker登记实际新增payload球后缀；GpuValidity和native margin将这些球排除在机器人净空检查外，native GeometricCost自动沿同一ground indices继承。完整箱体OBB地面检查一直开启且仍拒绝超过原1mm容差的穿地；新CPU测试也证明机器人球仍需2mm且−10mm真箱穿地被拒。`payload_floor_focused/`3请求全成功且独立CPU全成功：原触地箱、默认基线、偏心4cm/滚转90°基线，exec80728退出0。

最终自动恢复按相同19箱位重新跑，`fixed66_auto_grasp_challenges/`17/19；不是把旧分散成功拼成17/19。独立audit从原始请求箱心与尺寸、实际选中attachment字段、actual URDF末端、硬编码用户角度表重建接触/抽取/末态，另读冻结cap球在真实接触姿态下投影到原箱面；17成功的最差代理足迹excess≈−1.765mm，全部仍在面内。触地、同列侧吸、高位均在此同源版本通过。两失败及原始自动恢复尝试全部保留。

## 79cm统一阈值标定与当前正在验证的版本

显式标定 `threshold79_aligned_stack/`已完整成功，exec28550实际退出0，worker103.0277s，独立CPU核验1/1通过。原箱心仍为Z=.79/1.39m、Y=0、D=.66；两箱全局阈值同为.79，使等于阈值的低箱改用侧吸；足迹验证通过的偏心−.07/+.09m将实际吸附点设在Z=.72/1.48m，和此前已验证的同列side工具目标一致。只说明这一标定可行，不能凭它宣称全域或任意逐请求阈值选择成立。

当前统一候选版本将双箱入口默认阈值设为.79（单臂/历史BoxPick默认.80保留回放兼容），所有当前双箱默认配置显式记录.79。自动同列侧吸恢复在原±.08m外试共同接触高度0/+1/−1cm微调，仍逐个完整足迹/碰撞/惯量/原生/源位验收；不改变箱位。117项CPU回归通过，含新阈值.789/.790/.791切换归属以及旧BoxPick默认.80回放行为。

`fixed66_global79_catalog.json`包含完全相同的原19物理箱位和新增.789/.790/.791三组阈值边界，全部显式采用同一.79阈值、无人工给定吸附偏心；规划器必须自己恢复。GPU exec34081正在运行，估时20m（以上一完整19请求491.5s及新增恢复/边界、冷启动余量为基础，暂估不是超时）。源快照`source_global79/manifest.json`与首个真实运行结果SHA核对一致。旧17/19成功属于80cm版本，不会直接复用成新79cm成绩。最终阈值决定和全域结论仍待完整结果。

## 统一79cm验收终态及自动箱面选择

`fixed66_global79/`22输入终态21成功/1未通过（exec34081退出0，worker592.749526s，独立CPU21全过）。低箱同列、三个新边界都通过，唯一残留是固定前面模型的同侧请求。

CPU `same_side_alternate_face_cpu_probe.json`证实：右臂改吸高箱Y−侧面、roll90°时，完整cap代理足迹在该面内约21.336mm，必要接触界没有排除，15个共享升降位置有解析接触解。`alternate_side_face/`随后3个显式真实全周期（同侧原分配、交换分配、左右镜像）全部完整native/CPU通过，exec71754退出0。没有把解析解当成完整成功；经完整成功后才接入自动恢复。

`fixed66_global79_all_faces/`改用无人工吸附指定的原22输入，加镜像/箱臂交换共24，所有箱位和全局阈值不变，实际24/24成功，exec86973退出0；同侧原请求的grasp_trials清楚保留原中心及3个偏心失败，然后右Y−面成功。真实最终角由TimeOpt数组+URDF/用户角度表独立核验，最大误差约1.63e−7rad；updown按请求变化（同侧原样本约−.94m、镜像/交换约−.83m）。这纠正了旧“必须限定左右工作区才能直接搬运”的过宽判断，不需要把已解决的同侧箱位划为非法。

## 更低顶吸堆叠：8段候选几何贯通但尚未定时执行验收

`below_threshold_stack_contact_family.json`在5个低箱高度做实际固定tool0→link7球代理对原上箱OBB核算，重叠27.1184mm与高度平移无关；此分析只约束当前同时顶吸模型，不证明所有姿态/顺序不可能，输入保持在目标范围内。

顺序探针以Z=.5/1.1m、Y=0、D66/阈值79为例：

1. `ordered_upper_near_face_probe.json`和后来重新求解的`ordered_upper_parking/prefix/`均先取上箱，右Y−面；下箱完整世界实体始终保留。原13.806s几何探针首候选成功，移开上箱后低箱末端与上箱代理净空约45.4mm。
2. 仅停在刚抽离末端时，`ordered_lower_contact_probe.json`的11264个原始成对IK被碰撞筛掉，0可用接触；不是IK不存在的证明。诊断脚本有一个prefix几何文件名空hash笔误，原报告保留，另存`ordered_lower_contact_probe_lineage_audit.json`核对真实prefix数组末态与JSON一致。
3. `ordered_upper_parking/`重新求解上箱前缀，再按真实右臂10kg附着、实体下箱世界障碍、原位规则与静力筛选搬到让位位；15.048s几何成功，32个可用让位目标，实际选中updown≈−.23m、右臂给定7角，左臂仍空闲。未声称首前缀全程动态已验。
4. `ordered_lower_contact_after_parking_probe.json`从真实上箱让位末态继续，仍携带上箱并保留下箱实体审计；首候选得到下箱顶吸接近/接触几何，4.944s成功。第一次诊断因numpy float64接口误用退出1，修正显式float32后GPU51907退出0；旧错误log与脚本保留，不计物理拒绝。
5. `ordered_lower_extract_transport_probe.json`在下箱接触后切换到真实双载模型，下箱抽离.300m、上箱保持实际已让位姿态，然后共同回到14个用户角；15.985s候选几何成功，95个可用共享升降回中目标，GPU76972退出0。全实体/源位/静力筛选通过，但没有TimeOpt动态接受结果。
6. `ordered_geometry_chain_cpu_audit.json`独立按实际URDF验证8段真实数组：关节位置接缝全0、全部关节限位、两原箱心/各自抽离向量、最终用户14角通过。payload集合依次为空/空/右/右/右/右/双/双；这些是候选几何连接，不能冒充时间、速度、加速度接缝或全原生执行成功。

下一关键实现是把该真实附着/世界状态切换纳入有原生定时和独立验收的生产顺序规划分支：尤其下箱接近时必须已带上箱、下箱未吸前仍是实体；所有8阶段真实q/v/a和质量切换都要保留。不能把上箱删除、把下箱挪出原位、把8段几何拼成虚假原生成功，或把几何已通当成整体目标完成。

## 原生顺序状态实现与首验

新增CPU数据合同`curobo_core/ordered_pick.py`：8阶段中每个原箱必须恰为pending或实际payload之一，incoming必须仍pending；仅已抽离的真实附箱参加源位历史，明确哪一箱正在抽离、哪一箱保持，以及中间只要求上箱手臂给定7角/最终要求双臂14角。

`OrderedLayoutPlanner`保存稳定left00/right01对象ID；Core缓存键继续包含实际attachments。native几何优化现在可显式声明阶段实际payload sides，带上箱接近下箱不会错误套用空载机器人；FullPayloadCost只取实际附着侧的尺寸；payload球地面例外也按实际附着侧登记，不再按incoming选中侧推断。待取下箱的整个实体仍用于已载上箱OBB审计。

`ordered_native_seed_v1/`用已验证几何种子做原生状态集成（不是新鲜几何基准），前6阶段全部通过，第7因CartesianReference拒绝零位移而停止。`ordered_native_seed_v2/`在零位移保持/有限梯度/只同步两个移动目标修复后8阶段全过，GPU89057退出0，实际运动20.625841s，CPU独立32001样本全部通过。`source_ordered_native_v2/`完整字节指纹与该真实运行一致。

零位移参考仍要求固定方向，原先无意义的极小非零位移继续拒绝；保持目标的XYZ/方向误差仍参与损失和最终2mm/1°门禁，不会拿假微小抽离替代上箱保持。正常双移动目标的原进度同步项保留。历史PR80接触文件仅增加显式法向入口，测试把这一表达式恢复为旧默认后比较整模块AST，其余碰撞逻辑和另外5个冻结文件仍锚定原基准；不是直接删掉冻结碰撞测试。

## 全新几何到原生验收及被拒绝的版本

`ordered_fresh_v1/`首次新鲜几何接线遗漏源位检查函数导入，已定位修复，旧log/脚本保留；新增CPU symtable回调检查防止同类未执行分支遗漏，不归为物理失败。

`ordered_fresh_v2/`8个原生物理阶段均过，但final transport在GPU原位观察从.9809µm到1.0565µm产生已清出后返回，CPU仅.8739µm通过；整个任务按GPU门禁拒绝。没有用CPU通过覆盖GPU拒绝。

`ordered_fresh_v3/`在原2mm笛卡尔窗口内试50µm进一步−X抽离候选后，单段上箱运输被实体箱审计拒（box_robot约4.071mm、box_world下箱约3.038mm）；仍保留原拒绝，不算成功。`ordered_fresh_v4/`使用真实native种子恢复后所有8物理/源位/实际接缝过，但CPU验收发现内存tuple与JSON list比较格式差异而拒绝；随后做双侧规范化，未更改数值门禁。

`ordered_fresh_v5/`请求无轨迹/人工箱面，全新解析IK/LM/RRT几何及8阶段native均通过，自动选上箱近臂Y面并求解让位，独立CPU通过。它与种子集成结果严格区分geometry_fresh=true/false。原失败及原生整段拒绝全部保留。

## 默认入口、独立验收与回放证据

主`BoxLayoutPlanner.plan_layout`在同步所有允许吸附候选失败且存在遮挡顶吸堆叠时，才自动调用新鲜顺序分支；同步成功不重复走顺序。每次失败与最终顺序选择写grasp_trials及独立result文件，调用者不需要手工给流程或成功抓取参数。

`default_ordered_v1/`已完成上述自动入口真实验收（不是单独ordered CLI）：success/physical_success/task_success/replay_checks均为真。`independent_external_cpu.json`再次在GPU外打开实际原生数组复核；`replay_state_cpu.json`调用真实共享回放分派器和actual grasp变换，核对8阶段各首/中/末样本，下箱在吸附前保持原箱心、上箱一直按真实右臂附着运动。`source_default_ordered_v1/`经字节指纹重建核对，和该实际会话完全一致。

当前新增31回归在同一个既有共享gpuq条目内执行，估35m基于原24约698s+7顺序/新组合约90s/例及变化余量，暂估不是超时。原24、5个低同列(.2/.3/.5/.7/.78)、下大上小及镜像全部保留；无界定成功样本为“合法”的裁剪。顺序临时规划资源在返回后释放循环引用，普通拓扑池仍最多2个，不引入新资源池。

## 31默认入口回归终态与独立审计

GPU77418实际退出0，`default_full31/summary.json`全部31成功，`external_cpu_all31.json`在GPU外逐个重新打开真实数组并分派同步/顺序验证，31均通过，自动顺序7。所有原输入和历史失败保留，没有按结果删除案例，也没有逐请求更换66cm/79cm政策。此前147项CPU回归及完整源指纹已保存；此次回归没有编译并发超过8或绕共享gpuq锁。

## 更宽输入的持续必要界，而不是旧面选择误判

`broader_same_side_y85_request.json`取两箱同Y=.85m、Z=.8/1.4m、X=1.168052051m，仍遵守用户目前列出的平面/高差规则，并无初始箱间/环境/原Home代理重叠。当前入口6型均无接触，通过GPUq实际运行仍被`assigned_contact_outside_reach`拒绝；原失败及全部吸附尝试保留，不将其宣称非法位置。

连续支持界来自当前URDF的right shoulder/upper/fore几何及joint2约束：上臂肘位置是半径U的球冠，任意Y投影的精确支持函数再加前臂整球半径F与整个共享liftY变化，得到wristY≤.291163m（原姿态/几何包络后.301261m）。推导没有采样关节空间作为证明；2000个随机实际URDF状态只是对公式实现的自检。原先对Y=.65的同一松弛界不充分（负缺口），报告明确标为inconclusive并保留；没有把它误称不可达。

对Y=.85，接触点被放宽到箱体整个BBox、零足迹；腕到TCP的向量以全长度向任意方向提供最大补偿，因此不局限前面/顶面或几个roll。即便进一步扣除全部forward cap代理前伸约30.71mm，40箱和高箱仍分别缺约83.83/51.33mm。该结论限定在实际URDF/TCP平面/full-footprint接触模型，不能外推未建模局部吸盘/边角接触，也不排除左臂已取箱后改变箱位让右臂交接；后一过程当前未实现，更不能用删除盒或瞬间换attachment代替。

CPU `broader_transfer_two_contact_cpu_ik.json`为同一物理箱的两个工具目标找到了共同升降原始解，但没有做接触碰撞例外、单箱载荷分配或原生交接验证，明确不计成功。此证据用于把剩余任务范围交给用户确定或作为后续真实交接开发入口，不用作完成证明。
