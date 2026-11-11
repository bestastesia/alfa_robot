# 随机误差规划Demo：8171

## 2026-10-20 转身不显示：底盘变换重复应用

后续用户计算的4个成功结果均记录base_reposition Yaw=180度，放箱及Home保持180度，不是planner漏转身。8171错误将planar URDF交给ReadOnlyScene：后者已用外层root处理base_x/y/yaw，planar URDF内又处理一遍，180+180显示为0度，平移及工具/载荷显示也错。已改用同版本fixed URDF，仅显示根处理一次；规划仍用原模型配置，不删规划自由度/不改变轨迹。回归在0/.047/pi三个Yaw核对底盘及双TCP：外层base×fixed FK必须等于单次planar FK，3项random_error测试与4项TCP合同测试通过。

原复现FK：planar显示底盘Yaw约0、左TCP[.443934,.425,1.093934]；fixed正确为180度、左TCP[-.543934,-.425,1.093934]（该次导航终点）。因此旧画面不能作为真实放箱相对位置验收。当前服务alfa-random-error-display-8171默认加载用户最新1261帧成功结果generated/random_error_demo_runs/1792489873051676534/result.json，不重新规划、不拼造转身动画。导航仍按既定需求瞬移到位，不新增导航连续轨迹。未发送实机。

## 2026-10-20 高低目标箱下方漏填修复

截图任务L24/R15的左目标高于右目标。旧reconstruct_scene在整个工作层高度都扣除两箱横向开口，导致高箱下方不应挖空的区域也漏填。已按旋转箱AABB的Z边界分段，在每段仅扣除与其高度实际相交的目标横向区间，其他部分填实。工作层总上限仍沿用最低目标层定义，未擅自增加高层障碍；侧向2mm留缝与world抽离方向不变。

test_higher_target_does_not_cut_opening_under_it修复前失败、修复后通过，连同500随机与新零位变换等4项合同测试通过。截图同输入/tmp/random_error_wall_gap_input_20261020.json实算左箱下方[1.50805,.82,1.4]m已填充。8171服务alfa-random-error-wall-8171以--input-preview加载同一扰动，不随机替换问题现场；旧轨迹不再加载，因为新填充改变了规划碰撞场景，必须重算，不沿用旧成功结论。此次只修场景漏填，原放箱兼容性阻塞未验证解除、未发送实机动作。

2026-10-20，分支feature/random-error-planning-demo。统一入口http://127.0.0.1:8171/；不替换8152或测试杆8162，不连接实机。

## 后续诊断更正：目标分配与外观显示

初版runner错误硬编码世界箱心left Y=+.495/right Y=-.495、X=-2.3，把转身前后左右关系及输送机中心/放箱点混用。原ConveyorCyclePlanner.conveyor_reference_targets实际为left Y=-.425/right Y=+.425、X=-1.45（-2.3+.85），高度1.082。已删除重复常量，runner和显示共同复用原定义，并核验两臂能共享同一停车位，否则拒绝而不平均掩盖差异。

独立CPU诊断：错误目标在导航后footprint中left工具Y=-.495、right工具Y=+.495，肩到腕最短采样距离约1.061m，超过.98m臂长；21高度×5度ψ采样均0解。仅恢复原流程的横向目标后两臂各6816个限位内解析解（统计未去重、不含碰撞，也不是全周期成功率）。证据/tmp/random_error_place_target_diagnostic_20261020.json和/tmp/random_error_place_correct_reference_20261020.json。新回归test_random_error_placement.py用真实canonical方法证明转身后局部left Y=+.425/right Y=-.425，且保持.8m前伸、同一底盘位；另验不一致停车需求拒绝，2项通过。

双龙头车不见的原因是ReadOnlyScene.reference_conveyors返回的根节点默认visible=false，原UI忽略返回值。已保留根节点、默认显示、加“显示双龙头车外观”，光圈按相同canonical目标更新；这是修显示，不是把外观当作碰撞几何。

复测后缀出现独立阻塞：共享fixed.yaml新增顶层coordinate_revision元数据，当前cuRobo RobotCfg.__init__不接受，检查器初始化报TypeError，未进入修正目标后的IK。按用户遇非预期问题先停要求，未剥离/覆盖其他工程师模型元数据，尚不宣称完整动作成功。同期左J5零位+180及Home更新也未覆盖。

当前8171服务为alfa-random-error-view-8171，默认载入/tmp/random_error_failure_rebased_view_20261020.json：这是原失败现场，left J5按共享新零位减pi转换以保持同一物理姿态，原JSON保留，页面明确“历史错误目标现场／复测阻塞”。目标修正后的新轨迹尚未获得。原alfa-random-error-planning-8171已停止，不重复占端口。HTTP200/服务运行已验证，未绕过浏览器限制、未实机发送。

## 使用

页面使用当前重零模型及当前Home。选择原119任务之一或重新随机，生成停车半径5cm/Yaw±5度、箱体Roll/Pitch±2度/Yaw±5度；数值显示在页面。点击“规划当前随机场景”，依次尝试接触、world -X抽离35cm、收束运输、模拟导航到位、外部放箱、释放及Home。成功或失败都保留返回轨迹，可播放、拖帧、跳转阶段。切换输入会清除旧轨迹；规划时禁用选择，防止新场景混用旧结果。

目前默认载入一次真实计算的失败样本，不伪称完整成功：L24/R21，抓取/抽离/收束成功，模拟导航后“龙头车放置目标IK”失败，returned64、solver_success0、collision_valid0。842帧，最后阶段base_reposition；没有生成放箱/释放/Home，不会补画。失败不等于实体机械臂物理不可达，尚未进一步定位或改目标/放宽约束。初始规划约3.680秒、放箱尝试约1.703秒，前者日志未包含后缀失败时间，不能把3.680秒当全流程总耗时。

底层箱绕几何中心微倾可能穿地的问题仍存在，未得到用户确认前不自动抬箱或关闭地面碰撞。该Demo仍是实验接入，不宣称全119任务或随机误差成功率已验收。

## 代码与证据

- v3_tcp_scene_preview.py：场景输入、统一UI、异步规划、当前模型、载荷随动及失败前缀播放。
- v3_tcp_rezero_pipeline.py：恢复feature/tcp-motion-pipeline中的TcpMotionPlanner请求/阶段逻辑，以RezeroTaskPlanner为基础接当前解析映射和已转换候选库。接触工具面内roll/逆附着和场景恢复一致；不把测试杆70mm TCP用于搬箱，此处仍是原吸盘tool0。
- v3_random_error_runner.py：绑定TCP请求、保存输入、运行抓到收束、模拟导航及外部放箱/释放确认；失败立即返回现场，不继续伪造后续阶段。
- test_tcp_rezero_contract.py：60组侧/顶吸扰动下roll变换、目标箱恢复、世界抽离方向等价。连同原合同共3测试通过。
- /tmp/random_error_rezero_initial_20261020/：input.json、pick.json、placement.json、result.json。新请求保存research/curobo_v3/generated/random_error_demo_runs/<时间戳>/。临时证据不是长期归档。
- /tmp/random_error_ui_smoke_20261020.log：真实ReadOnlyScene接口119任务各两次、共239次更新通过。另修复轨迹仅18轴而显示还需头部固定轴的适配；未改规划自由度。

systemd用户服务alfa-random-error-planning-8171；原纯场景alfa-random-error-demo-8171已停。HTTP200和服务运行已验证；本轮浏览器自动访问受既有策略限制，未声称按钮已通过浏览器实点验收。现场用户可刷新测试。

启动（服务已在运行，不要重复占端口）：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3 && \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
  research/curobo_v3/tools/v3_tcp_scene_preview.py \
  --port 8171 \
  --result /tmp/random_error_rezero_initial_20261020/result.json
```

若临时result丢失，去掉--result即可生成新场景，需用户点击规划，不会套用旧录制。
