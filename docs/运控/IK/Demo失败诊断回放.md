# Demo 失败诊断回放

## 默认行为

无需新参数，使用原启动命令即可。当前自研 V3 动作 Demo 在最终规划失败后仍会播放诊断画面，并停在最后一帧，直到提交新任务或显式重置。

- 有已验证的路径前缀：先播放前缀，再显示被拒绝的关节状态；红色接触点标注碰撞双方。
- 升降碰撞：停在本次升降的第一个失败采样点（采样间距不超过 5mm），不是回到 home。
- 箱墙/单臂的目标预检查或候选搜索失败：没有连接路径的拒绝姿态**不进入机器人/箱体回放**。保持最后实际到达的姿态（`FAILED_HOLD`），只标记失败目标；拒绝姿态与碰撞对保存在 JSON 的 `rejected_joints` / `rejected_contacts`，`snapshot=unconnected_rejected_state_not_replayed`。旧版播放未连接的带箱 home 快照会造成假吸附瞬移，已撤销，不能当成抓取失败证据。
- IK 无解、搜索超时且没有被拒绝的具体关节状态：保留最后可用状态，显示失败目标和原因；不伪造 IK 解或碰撞位置。
- 最后一帧显示 `DIAGNOSTIC ONLY - FROZEN (not executable)`。Rerun 同时记录失败阶段、原因和诊断快照类型。
- 自动正吸→顶吸/左右臂回退不变。全部失败后，箱墙选择**实际相连诊断前缀最长的尝试**显示；长度相同取后一次。后一次立即无解不会抹掉前一次实际碰撞的路径。JSON 的 `side/suction_mode/height_alignment` 对应该诊断尝试，`attempts` 仍完整按搜索顺序记录。
- 仍然先完成规划，再启动回放。`CALCULATING` 期间不动不代表回放失效。

## 覆盖入口与边界

| 入口 | 行为 |
| --- | --- |
| `v3_single_arm_box_extract_demo.launch.py` | 单臂抽箱失败诊断；失败回放不再循环 |
| `v3_box_wall_grasp_demo.launch.py` | 箱墙单箱诊断 |
| `v3_box_wall_comfort_grasp_demo.launch.py` | 舒适高度与正吸/顶吸共用诊断 |
| `v3_box_wall_sequence_demo.launch.py` | 先播放成功箱体，再显示失败箱体；保留对应场景快照，失败箱不消失、不提交 |
| `v3_dual_arm_cartesian_box_demo.launch.py` | 双臂同步路径前缀、拒绝姿态和接触点 |
| `v3_dual_arm_box_roll_demo.launch.py` | 同上；旋转失败目标保留请求角度，不再错误显示为部分路径终点角度 |
| `v3_dual_arm_asymmetric_box_demo.launch.py` | 同上；异构握持共用实现 |
| `v3_redundant_ik_interactive_demo.launch.py` | 无解时保持上个有效状态并标红目标；该 Demo **仅做解析 IK，不做碰撞检查** |
| 旧 `pick_place_demo.py` | 失败后保持现场、标记失败目标，等待 Ctrl+C；不继续下一轮/后续控制器。IK 服务不提供失败关节状态时只显示目标，不生成碰撞执行轨迹 |
| 旧 `ExtractDemoOrchestrator` 全排入口 | 首次失败立即停止；不清除失败场景、不重置命令状态、不继续后续排 |

`demo.launch.py` 本身是上游 MoveIt RViz GUI 启动包装，不经过上述自研规划器。其第三方规划器不发布失败路径时，不能凭空回放搜索树；仍使用其原生无效目标/规划错误显示。已给此 GUI 加入旧脚本失败标记订阅，但**不宣称改造了上游所有规划插件**。

## 7 号箱顶吸：2026-09-14 修复与边界

根因是 `planTask` 在求接触路径之前预检查携箱 home，然后把该未到达姿态连同 `box_attached=true` 放进诊断回放。实际没有接触箱体，画面却假吸附并穿模。这是代码错误，不是可接受的失败仿真。

现在先求相连的接近/吸附/抽离路径，再检查返回任务；未连接候选不回放。顶吸选高使用肩到**腕心**而不是 TCP（当前工具腕心在 TCP 上方约 0.23835m），需要时先经碰撞规划收拢双臂再降高（当前姿态可安全升降则保留），吸盘长边转为横向。现已进一步统一：所有箱墙单箱/序列都后放并释放消失，不强制回home或升降归零；下一周期沿用实际末态。顶吸独立肩高于腕心0.10m，附着后先提起5cm并同步朝机器人退2cm，再继续水平抽离至累计35cm。详见 `V3箱墙连续搬运测试.md` 当前合同。

原窗口参数 `x=.90 / box7 / top` 在当前 URDF、固定底盘下不可达。`x` 是底盘前沿到箱墙近面的距离，不是机器人原点到箱中心。独立 URDF 轴线求交：箱顶任一点对应的腕心水平距离至少约 **1.065525m**，而肩—肘—腕总长约 **0.979m**。这是独立几何证据，不是旧 home 碰撞的推断。`.80` 的**顶面中心**也超出范围；未证明 `.80` 所有偏心吸附点都不可达。

用户已确认：单独调试失败箱时，顺序上此前箱体直接视为已搬走；用现有 `wall_context:=sequence_prefix`，不需要回放此前搬运。距离仍须单独明确。下例使用**已验证的 `x=.50` 距离**，显式重建按顺序轮到7号箱时的剩余场景（上面三排和5、6号预先清空，保留0–4、8、9及全部五个环境障碍）。不是声称此前17次搬运已经在该距离完成，也不是原 `.90/full` 配置通过。先在旧 Demo 终端 Ctrl+C，再完整复制：

```bash
cd /home/astesia/Sevenova/alfa_robot
source /home/astesia/Sevenova/alfa_robot/tools/ros_humble_env.sh
source /home/astesia/Sevenova/alfa_robot/ros2_ws/install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=198 \
ros2 launch alfa_robot_moveit_config \
  v3_box_wall_comfort_grasp_demo.launch.py \
  x:=0.50 \
  box_id:=7 \
  arm:=auto \
  suction_mode:=auto \
  wall_context:=sequence_prefix \
  initial_pose:=home \
  comfort_ratio_min:=0.95 \
  comfort_ratio_preferred:=0.95 \
  comfort_ratio_max:=0.95 \
  top_shoulder_above_wrist:=0.10 \
  wall_bottom_z:=0.000001 \
  planning_seed:=104731 \
  check_environment:=true \
  auto_run_once:=true \
  start_rviz:=true \
  start_rerun:=false
```

按用户要求，使用带完整续行符、参数可逐行修改的长命令，不以短包装脚本代替；launch本身的 `wall_context:=full` 为默认值，保留其他24箱；连续序列不采用预清空夹具，始终从25箱开始。原距离复现可改 `x:=0.90`、`wall_context:=full`，会正确报告预接触不可达并保持原位，不再演出假吸附。

## 数据和安全合同

- 失败 `frames`、`success`、序列 `final_joints` 和移除箱号的事务语义不变。单箱成功与序列相同：后放释放，末态保持，不冒用空载home。
- 失败结果另加 `diagnostic_frames`，每帧带 `diagnostic_only=true`；`diagnostic` 包括阶段、原因、接触点、可选失败目标和快照性质。
- RViz/Rerun 只在失败时选择诊断帧；成功时忽略诊断数据。
- 所有碰撞姿态仅进入 Demo 的观察用关节状态和标记，**不进入 FollowJointTrajectory、实机执行、ACM 放行或规划成功结果**。
- 序列的诊断显示可以不同于逻辑提交状态；失败箱体及其他剩余箱体保留碰撞几何，逻辑 `final_joints` 仍为最后完整成功周期的状态。

## 回归

源码测试：`test_demo_failure.cpp`、`test_demo_failure_viewers.py`、`test_v3_failure_replay.py`，并扩展顶吸、序列测试。独立 IK 启动修复及其启动/拖动回归在本地另行交付，不随箱墙/Rerun PR 提交。测试启动自己的无窗口 ROS 进程，使用空闲 localhost domain；不要与已有 GUI 共用 domain。

历史 2026-09-12 证据（旧拒绝快照行为，不能作为当前验收）：`/home/astesia/Sevenova/日志/验收_2026-09-12/failure_replay/`。其中 `box7_top` 验证返回目标碰撞与冻结；`shared` 验证双臂、IK 和升降失败；`sequence_rear` 验证后方障碍导致 0/25 且不消失；`sequence` 验证原默认结果仍为 17/25、失败箱 7，并检查完整回放后的冻结状态。诊断功能并没有把失败的搬运任务变成成功。

### 当前验收入口

`test_v3_box7_top_replay.py` 启动真实无窗口 ROS 回放，采集关节和箱体标记，并使用从该节点读出的 URDF/SRDF 运行 `validate_v3_wall_replay`。独立检查全部环境/邻箱、吸附位置和朝向连续、全轨迹关节界限以及每0.5°/2.5mm碰撞采样；这些有限步长检查不等同数学连续碰撞证明。Rerun 的实际箱体变换函数另以同一轨迹检查。验证器必须拒绝人为注入的瞬移和场景碰撞。

当前证据目录：`/home/astesia/Sevenova/日志/验收_2026-09-14/box7_top_fix/`。`top_regression/x090_reach_certificate.json` 是原距离的独立几何证据；`live_left`/`live_right` 是明确 `.50/sequence_prefix` 的真实回放。`shared_regression` 验证共享失败冻结，不宣称上游第三方插件已被改造。

### 顶吸后放的附加修复

`wallRearPlacementPose` 按旋转后的**整箱最前端**设置后方目标；旧逻辑仅约束TCP，折臂顶吸时会把合法IK候选全部判成“箱体没有完全到后方”。单元测试覆盖正吸/顶吸及旋转的非立方体八角点。`.50/箱7/明确预清空17箱` 的原生单周期探针现已完成顶吸→抽离→后方释放→空载home，2623帧经独立检查通过（3629次采样）。这是 `planTask` 的隔离周期证据，**不是通过安装launch执行了完整25箱序列**；探针和记录在 `top_sequence_cycle/`。

默认 `.90` 安装版序列仍为17/25；`sequence_final/` 还显式传入了 `wall_context=sequence_prefix`，验证该单箱夹具不会使完整序列跳过箱子。

### 完整箱墙的上方遮挡复核（2026-09-14）

用户随后启动的 `.75/box7/top/full` 场景仍有24邻箱，12号箱就在7号上方，表面间隔10mm。只读采集实际ROS回放41组关节/箱体标记，确认机器人保持初始姿态、箱体留在原位置，没有再次假附着。

离线探针直接放置**吸盘头碰撞网格**，不伪造机器人IK或发布变形姿态：两侧分别采样顶面31×41个位置（10mm间距）、24种垂直顶吸绕Z角度（15°间距），每侧30504次均与上方12号箱相交。每侧1512个采样通过水平臂长这个必要条件，但无一同时通过吸盘头与场景碰撞检查。远离箱墙的负对照通过，排除了碰撞变换未刷新的假阳性。完整数据、独立探针及复现命令在证据根目录 `topface_x075_full/`。

这是**有限采样证据，不是所有偏心点/任意角度无解的数学证明**，也未证明通过臂展条件的点有完整IK、密封覆盖或可执行路径。不要据此自动删除12号箱或将full改成sequence_prefix。`full`不是“孤立一个箱”；`sequence_prefix`才明确模拟按顺序轮到该箱时的剩余场景，且不证明之前周期已执行。`x`越大表示底盘前沿离箱墙越远。
