# V3 距离输入单箱抓取 Demo（旧架构，纯仿真）

> 新增独立实验入口：见 [V3单次舒适高度抓取Demo](V3单次舒适高度抓取Demo.md)。按单臂肩心到接触TCP距离选高；独立验证未证明全面优势，本页原启动命令和固定0.25m默认策略不变。

## 1. 入口与范围

按 2026-09-11 用户/mentor 确认，在本地旧架构单臂抽箱 demo 上增量开发，**复用同一个 C++ 可执行程序和解析 IK/RRT，不新增包、不接实机**。

新 launch：`v3_box_wall_grasp_demo.launch.py`。旧 launch 保留原有交互行为。

输入车头到箱墙近面的距离 `x`（米），指定 5×5 箱墙中任一 `box_id`，选择 `left/right/auto`。成功条件是找到完整的：

0. 从文档/SRDF `whole_body/home` 初态按目标高度下降共享升降轴（需要下降时）；
1. 保持调整后的升降高度 → RRT → 预接触；
2. 解析笛卡尔直线接触；
3. 吸附目标箱；
4. 直线抽出 0.35m；
5. 默认携箱 RRT 到远端流程定义的后置位；
6. 默认释放箱体并保留该末态。研究用 `loaded_home` 策略则携箱返回 Home，并按5mm间隔校验 Updown 回0。

**不是双臂抓取、放置/释放、连续拆墙、共享轴搜索或实机轨迹。** 每次请求独立恢复完整箱墙和默认初态，不累计上次抽出的洞。

## 2. 启动

在本机独立终端中使用空闲 ROS domain；同 domain 不要同时启动其他仿真状态源或实机栈（TF 和 robot_description 仍是公共 topic）。

```bash
cd /home/astesia/Sevenova/alfa_robot
source tools/ros_humble_env.sh
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=188  # 新高度版示例；先确保该 domain 空闲

ros2 launch alfa_robot_moveit_config v3_box_wall_grasp_demo.launch.py \
  x:=0.90 box_id:=5 arm:=auto
```

默认同时打开 RViz、Rerun，并执行一次启动请求。`0.90m / 5号箱` 是已验证的**仿真样例**，不是同事的测量结果或推荐最优距离。`x` 必填，不能使用缺省距离冒充实测。

只看场景、暂不规划：`auto_run_once:=false`。

无窗口录制：

```bash
ros2 launch alfa_robot_moveit_config v3_box_wall_grasp_demo.launch.py \
  x:=0.90 box_id:=5 start_rviz:=false spawn_viewer:=false \
  rerun_recording_path:=/tmp/wall_grasp.rrd
```

`start_rerun:=false` 可完全关闭 Rerun；`start_rviz:=false` 关闭 RViz。

RViz：绿色目标、橙色其余24箱，白字是固定墙格编号；吸附后目标箱根据同一关节回放的 tool FK 随臂运动，结束后停在最后一帧。编号表示墙格，不随箱体移动。
Rerun：相同场景/关节帧及附着标志，只读显示阶段、距离、箱号、选中臂、失败原因；两者播放速度不同，**不是同步时钟下的执行监控**。

### 关闭终端后重启 / 一直 waiting 的定位

`waiting for service to become available...` 表示调用端尚未发现服务，**还没有进入箱体规划，更不是返回了“不可抓取”**。关闭启动终端会停止服务；下次必须先重开终端A，执行上面的完整环境设置与 launch，再在终端B调用服务。不要只打开 RViz/Rerun 或只重发 service call。

**2026-09-11 本机已确认故障：**启动命令误写成 `arm:=auto~`（多了 `~`），C++ 节点报 `single-arm extract demo init failed: arm must be left/right/auto` 后退出。旧启动编排没有同步关闭观察器，所以界面仍在、服务却已不存在。本次两端均为 domain 187、localhost 1 且 overlay 正确，不能归因于 domain 或 source。

修复后的新 launch：

- `arm` 用 ROS launch 原生 choices 限定为 `left/right/auto`，`auto~` 在任何子进程启动前即被拒绝；不要静默修剪非法参数。
- 规划节点退出即触发整个 launch 的 shutdown，关闭它管理的其他 ROS 节点；独立 Rerun 查看窗口可能仍保留历史录制，**窗口存在不代表服务在线**。
- 启动失败先读终端A的第一条 `ERROR/FATAL`；看到 shell 提示符重新出现，就不能继续把该终端当作运行中的服务端。
- 推荐在终端A按 `Ctrl+C`、等退出后再关窗口。突然关窗口后发现图可能短暂残留，不能把缓存的 service list 当作可调用证据。

终端B每次重开都执行（终端A保持运行）：

```bash
cd /home/astesia/Sevenova/alfa_robot
source tools/ros_humble_env.sh
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=188
ros2 service call /v3_box_wall_grasp_demo/plan_wall_box \
  alfa_robot_moveit_config/srv/PlanWallBoxDemo \
  '{x: 0.90, box_id: 5, arm: auto}'
```

启动生命周期回归（安装环境已 source，188 须空闲）：

```bash
ROS_DOMAIN_ID=188 python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_startup.py \
  --artifacts /tmp/wall_grasp_startup
```

覆盖非法 arm 启动前拦截、非法 x 初始化失败联动退出、三次启动并在 `target_only + loaded_home` 研究策略下请求9号箱、Ctrl+C / 规划进程退出 / 终端关闭信号以及服务下线。不会调用实机执行接口。

## 2.1 新增：按箱体高度先下降（2026-09-11）

开发分支 `feature/wall-box-height-alignment`；冻结版仍为 `feature/wall-box-grasp` / `27ea05e`，原 PR #20 不在本轮改写。

**高度合同（本 demo 的明确约定，仍需实机标定确认）：**机械臂中心取 V3.1.1 左右肩部前三关节轴线公共交点的中点，转换到 world 后的 Z。不是 `arm_carriage` 原点，也不是末端高度。目标高度取箱体**中心**，不是箱顶面。

```text
height_difference = initial_shoulder_z - (box_center_z + shoulder_box_offset)
descent = max(0, height_difference)      # align_height=true 时
updown_target = 0 - descent
```

- `align_height:=true`：默认开启；`false` 关闭升降；若要精确回归不含环境的冻结场景，还需显式 `check_environment:=false`（仅历史回归）。
- `shoulder_box_offset:=0.25`：默认25cm，可在启动时修改，单位米，必须有限且非负；本轮不扩大服务请求类型。
- 落地后的初始肩部中心 Z≈1.789146353m。底排0～4箱中心Z=.20m，需要下降≈1.339146353m，超过1m行程而拒绝；5～9下降≈.929146353m，10～14下降≈.519146353m，15～19下降≈.109146353m；最高排不下降。
- `updown` 读取模型限位 `[-1, 0]m`；越界返回 `height_alignment_limits`，绝不静默截断。下降按至多5mm间隔检查全机器人关节边界/碰撞；失败返回 `height_alignment_collision` 和失败位置/碰撞对，不回放被拒绝的下降。
- 合法下降前缀使用 `lower_to_box_height` 阶段和完整16轴帧；后续 IK 的基坐标变换、RRT 起态和负重返回目标均用下降后的状态。携箱返回默认臂角后，以 `restore_default_height` 阶段升回初始0m；这段负重上升也做整机碰撞检查，碰撞时返回 `return_lift_collision`。
- 仍然**先算几何路径，再可视化回放**；没有向实机下发升降或机械臂动作。抓取失败时可能只回放通过检查的下降用于诊断，不代表完成搬运。
- `result_json.height_alignment` 给出基准、原始高度差、计划下降量、目标关节值、限位和采样步长。Rerun摘要及RViz成功提示显示升降信息；不是实际编码器反馈。

**姿态合同：**左臂 `J1～J7=[150,90,-5,120,0,0,0]°`，右臂为 `[-150,-90,5,-120,0,0,0]°`，`updown=0m`、头部关节为0。直接读取 SRDF `whole_body/home`，不复制一套角度到规划器。默认远端流程后置释放；显式 `loaded_home` 才携箱回该姿态。

## 3. 选择接口

Demo-local ROS service：

- 名称：`/v3_box_wall_grasp_demo/plan_wall_box`
- 类型：`alfa_robot_moveit_config/srv/PlanWallBoxDemo`

```bash
# 在另一个 source 了同一环境且 ROS_DOMAIN_ID 相同的终端
ros2 service call /v3_box_wall_grasp_demo/plan_wall_box \
  alfa_robot_moveit_config/srv/PlanWallBoxDemo \
  '{x: 0.90, box_id: 5, arm: auto}'
```

请求字段：

| 字段 | 约束 |
|---|---|
| `x` | 有限正数，米；车头参考平面到墙近面沿 world +X 的距离 |
| `box_id` | 0..24，`row=id/5`（整除），`column=id%5` |
| `arm` | `left` / `right` / `auto`；auto 先左后右，找到完整路径即停止，不比较两臂最优性 |

编号按 world 坐标定义，**不按屏幕左右定义**：

```text
                  +Y →
row 4 (+Z高)   20 21 22 23 24
row 3          15 16 17 18 19
row 2          10 11 12 13 14
row 1           5  6  7  8  9
row 0 (底层)    0  1  2  3  4
```

响应：

- `success`：是否找到完整路径；只找到预接触 IK 或部分抽出路径不算成功。
- `generation`：有效规划请求的序号；非法请求不递增、不改变当前场景。
- `selected_arm`：成功臂；失败为空字符串。
- `failure_stage/failure_reason`：升降限位/下降碰撞、非法参数、初态/负重返回目标碰撞、IK、接触、抽出、RRT等诊断。
- `result_json`：与 `~/task_json` result 消息相同，含25箱场景、车头基准、16轴名称、逐帧关节/附着状态及 `attempts`。非法/忙请求无 result JSON。
- `verdict=path_found/no_path_found`：**no_path_found 只表示本次有限搜索未找到，不证明物理上不可抓取**。
- `auto` 两臂均失败时，顶层失败原因及诊断回放来自最后尝试的臂；请同时查看 `attempts`，避免遗漏另一臂失败信息。JSON `side/tool_link` 是当前诊断/回放臂，失败时不代表选中了执行臂。

此服务同步计算、回调串行；同时到来的请求可能排队，不承诺立即返回 busy。调用端应串行发送、等待响应；本 demo 不提供取消/抢占。规划期间 RViz状态发布暂停，不应被解释为实机心跳。若进入生产，应由 runtime/action 合同管理请求、状态版本、超时与取消，不把该接口直接晋升为实机接口。

## 4. 距离与墙几何

坐标系：当前 V3.1.1 模型中的 `world`，车头沿 +X，墙正面垂直 X 轴，不处理车辆/墙体偏航。

当前模型没有命名的车头 frame。默认遍历 **V3 完整移动底盘（车架、主动悬挂、脚轮和车轮）的碰撞网格顶点**，变换到 world 后取最大 X 作为 `chassis_front_x`、最小 X 作为 `chassis_rear_x`。按四轮轴心定义 `base_footprint` 后，当前 V3 安装资产前沿约为 `+0.500000005m`；实测仍可通过参数覆盖。

可用显式标定覆盖（单位仍是 world 米，不是额外偏移）：

```bash
ros2 launch alfa_robot_moveit_config v3_box_wall_grasp_demo.launch.py \
  x:=0.90 box_id:=5 chassis_front_x:=0.42 \
  wall_center_y:=0.0 wall_bottom_z:=0.0
```

默认箱体尺寸沿 world XYZ 为 `0.30 × 0.40 × 0.40m`，横竖间隙均 `0.01m`，不缩小碰撞体。中间列中心 Y 默认0，底面 Z 默认0。单箱中心为：

```text
X = chassis_front_x + x + box_depth/2
Y = wall_center_y + (column-2)*(box_width+gap)
Z = wall_bottom_z + box_height/2 + row*(box_height+gap)
```

所以换箱号不移动整面墙；只有 x/显式墙位置改变时才移动墙。

## 5. 碰撞与搜索边界

新模式相较旧 demo 的补充：

- 接触前：目标箱 + 其余24箱都在 MoveIt world 中。
- 仅最后接触小段允许目标与当前 `tool0/joint7` 发生接触，不放开其他机器人连杆或邻箱碰撞。
- 吸附后：目标从 world 移除，按 tool frame 下 `+Z=depth/2+contact_numerical_gap` 挂载；载荷在接触、抽出、负重 RRT 与目标状态中参与检测。没有同时保留静态目标造成双份碰撞体。
- 显式状态/边检查采用整机器人碰撞（仍服从现有 SRDF ACM）。有界关节按实际差值插值，不把超过π的转动视作可穿越限位的短路。
- RRT输出检查附着、关节边界和非规划关节不变；实际起终点之间的连接也校验并加入回放，避免规划适配器修正后出现未经验证的跳变。
- 新模式只生成几何路径，禁用不需要的 TOTG 时间参数化；不输出可直接下发控制器的时间/速度/加速度轨迹。

明确限制：

- 初态和成功终态均为文档/SRDF home；非选中臂关节及头部固定，双臂随共享升降架先下降、成功后携箱升回0。不搜索底盘或共享轴联合路径。
- 固定姿态正面中心吸附、5°冗余角采样、最多8个预接触候选、单次RRT默认1秒。`auto` 最多两臂；失败可能由策略、初态或搜索预算造成。
- 碰撞检测为离散采样（默认最大关节插值步长2.5°），不是连续扫掠体证明；真实末端接触容差、吸盘压缩、负载/扭矩/稳定性与真实动力学未建模（下述微米间隙仅用于数值处理）。
- 默认 world 场景包含箱墙及五个仓库环境碰撞体；环境来自启动配置，不来自感知。地面、箱墙底面和底盘落地基准均为Z=0，见下节。只检查录入的有限尺寸障碍，不证明配置外区域安全；小于离散采样步长的障碍存在漏检风险。
- `total_ms` 是两臂尝试的总计算时间；`metrics` 是最终/成功臂的原有统计，不含全部后验边检查，不作为精确性能计费。
- 所有结果仅在上述模型和假设内成立，不用于承诺实机安全。

### 落地单侧开口仓库（2026-09-12）

默认**内尺寸：高2.35m、宽2.38m、长4m**。正墙在+X，−X端开口；两侧墙、正墙、地板、顶棚共五个碰撞体，墙厚10cm向外增加，不侵占内尺寸。箱墙沿Y居中（宽2.04m，两侧各17cm），背面靠正墙，底面Z=0，顶面Z=2.04m。

- V3 主动悬挂模型已在 `base_footprint` 坐标中完成车轮落地对齐；该 launch 默认 `model_ground_offset:=0.000005`，只让整机相对 Z=0 地面留 5µm 导入网格容差，不改变机械臂相对几何或升降限位，也不是实机安全裕度。保留该标定参数。
- `contact_numerical_gap:=0.000001` 同时用于正墙贴靠的1µm数值间隔，避免吸附FK微小误差将右臂携箱误判为穿正墙；未缩碰撞体、未添加地面/仓库碰撞豁免。
- 环境进入共享 `makeScene()`，覆盖初态、下降、IK、RRT接近、附着、抽出、负重返回及恢复默认升降高度。RViz、Rerun和 `result_json.environment.boxes` 使用同一组world中心/尺寸。Rerun接收与规划器/RViz相同的完整URDF，预览也读取JSON的 `initial_joints`。
- 环境文件默认 `config/v3_box_wall_environment.json`；`environment_file:=/绝对路径/scene.json` 启动时读取，修改后需重启。根字段 `frame_id:"world"`、`description`、`boxes`；可选 `anchor:"box_wall_back"` 表示盒体中心相对箱墙背面中心XY（Z仍world），每个新距离请求同步平移整个仓库，保持箱墙居中贴正墙。自定义绝对world坐标请删除anchor或设为 `world`。这是独立场景重置，不是底盘行走。
- 每个盒体仅支持唯一 `id`、三维 `center`、正值完整边长 `size`，米、world轴对齐；必须包含 `ground`，最多128个，数值有限且绝对值≤10000。错误坐标系/anchor/旋转/重复ID等拒绝启动。`check_environment:=false` 仅作算法诊断，不代表仓库验收。

**新姿态改变了可用距离：**x=.30m初态伸入箱墙；x=.75m携箱home与剩余箱体相交；x=.90m可演示5号左臂、9号右臂等完整抓取及默认姿态返回。底排0～4落地后按25cm肩部偏置需要下降约1.2946m，诚实返回 `height_alignment_limits`，不截断、不抬高箱墙。

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=195 /usr/bin/python3 \
  ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_environment.py \
  --scan-wall --artifacts /tmp/warehouse_check
```

此回归检查尺寸/开口/贴墙、默认起终16轴、双臂完整流程、距离切换重建场景、五个仓库实体分别侵入整机、空闲臂/下降/携箱返回/上升/抽出障碍、非法配置、RViz同源几何、携箱FK地面净空。历史零姿态全墙扫描不再代表当前版本；旧结果保留在协作LOG及原验收目录。

### 10号能抓、14号失败：已定位的贴面数值问题

2026-09-11，`x=0.30`、默认墙位置与旧初态的对照结果：

| 请求 | 原零数值间隙 | 修复后的默认间隙 |
|---|---|---|
| 10 / auto 或 left | left 成功 | left 成功 |
| 14 / auto | left 预接触无逆解；right 最后接触步与下方 `neighbor_box_9` 碰撞 | left 失败后 right 成功 |
| 14 / right | 同一 `right_joint7` / 邻箱碰撞 | right 成功 |

**不是漏算一侧。** `auto` 原本就是先 left、失败再 right。模型臂名不是观察者面对机器人时的屏幕左右；终端现在逐臂打印 `arm=left/right SUCCESS/FAILED stage=... reason=...`，RViz成功状态也标出 arm。

原因：旧接触目标把名义 TCP 严格放在箱墙平面上，但导出网格的前沿与该平面并非完全一致。当前模型左/右 `joint7` 网格在 tool0 的 +Z 最前沿分别约超出 **0.039µm / 0.101µm**，加上浮点位姿误差，近共面的末端边缘与下方邻箱被判为碰撞。这不是厘米级可达范围不对称。

新距离 demo 默认 `contact_numerical_gap:=0.000001`（**1µm**），让名义 TCP 停在墙面前1µm，并将同一偏移用于附着碰撞体、RViz和Rerun；箱体仍位于原墙格，吸附瞬间不跳动。未缩小箱体/机器人网格，未改变左右臂顺序，未放开邻箱碰撞。旧单臂 demo 的默认值仍为0。

参数仅允许有限的 `[0, 0.0001]` 米，用于该仿真接触模型的数值处理。**不能作为真实吸盘行程、压缩量、TCP误差或安全间距标定。** 用 `contact_numerical_gap:=0.0` 启动，可复现14号右臂原失败；回归同时覆盖此反例，避免把碰撞检查失效误当成修复。

之前指南中8号右臂的同类失败也是零间隙旧结果；修复后本机8号已规划成功。全墙有限搜索结果不代表所有箱体可抓或每次搜索必定成功。

## 6. 仍缺的规范/接口（不阻塞独立 demo）

新增升降仍缺：实机肩部中心基准与零位/行程标定、25cm偏置验收值、升降速度/加速度/负载稳定性合同、跨箱接续策略。已加入地板与环境碰撞，但默认尺寸未经现场标定；底排超行程不能通过关闭碰撞掩盖；仿真不构成实机安全验收。

1. **车头参考面合同**：同事测量采用外壳、保险杠、碰撞网格前沿还是另一标记？需交付 world/base 的标定关系与测量不确定度；当前可通过 `chassis_front_x` 覆盖。
2. **墙摆放合同**：横向偏移、底面高度、箱体实际尺寸/间隙、正对关系是否符合默认值？x单独不能描述任意偏置/偏航墙。
3. **初态/终态合同**：本demo已按本轮要求切换到正式home，包含成功时升回0；其他旧demo的行为不变。
4. **实机与生产合同**：尤其缺少真实吸盘接触面/TCP、压缩量及允许误差标定；1µm数值处理不能替代。吸附反馈、负载、环境障碍、状态/场景版本、执行/取消、验收人等仍需在正式runtime任务中确认；本服务不替代它们。
5. 仓库 `AGENTS.md` 引用的 `/home/li/.codex/RTK.md` 本机不存在；本次遵循可读的 `.ai_teamwork` 和包职责边界，不声称遵循了缺失文件。

## 7. 可重复验收

```bash
cd /home/astesia/Sevenova/alfa_robot
source tools/ros_humble_env.sh
cd ros2_ws
colcon build --packages-select alfa_robot_description alfa_robot_moveit_config alfa_robot_rerun \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
source install/setup.bash
cd ..
# 每条命令均选空闲domain；脚本只清理自己启动的进程
ROS_DOMAIN_ID=195 /usr/bin/python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_environment.py \
  --scan-wall --artifacts /tmp/warehouse_check
ROS_DOMAIN_ID=196 /usr/bin/python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_grasp_demo.py \
  --artifacts /tmp/warehouse_grasp
ROS_DOMAIN_ID=197 /usr/bin/python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_height_alignment.py \
  --artifacts /tmp/warehouse_height
ROS_DOMAIN_ID=198 /usr/bin/python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_startup.py \
  --artifacts /tmp/warehouse_startup
ROS_DOMAIN_ID=198 /usr/bin/python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_preparation.py \
  --artifacts /tmp/warehouse_legacy
```

2026-09-12仓库/home版证据：`/home/astesia/Sevenova/日志/验收_2026-09-12/warehouse_home/`。

- 仓库回归：42真实请求 + 7类非法配置通过；独立STL测量所有初态机器人碰撞网格位于仓内，底盘最低Z≈0.947µm，`base_link`为Z=0；RViz/JSON几何相同。
- 固定x=.90m、25cm肩部偏置、auto的25箱扫描：**成功17/25**，箱号5、6、8、9、10、11、13、14、15、16、18、19、20、21、22、23、24；0～4升降超限，7/12/17在接触路径未找到解。每箱独立重置，不是连续拆墙、最优距离或所有姿态的可达性证明。
- 高度回归：21请求 + 3非法偏置通过，包含默认home起终、恢复升降、负高度差、限位边界、失败前缀、双臂及FK/marker。
- 固定升降服务回归已迁移到新home/落地坐标，覆盖25墙格、7非法输入、20左/24右显式与auto、完整路径、零gap反例及车头标定覆盖；它关闭环境只验证算法接口，不冒充仓库验收。
- 启动重启/退出检查通过；旧交互demo10项回归通过，旧launch仍保持原始行为。
- 右臂9号录制 `visual/warehouse.rrd` 通过 `rerun rrd verify`，418帧完整回放到 `restore_default_height`，最后joint_states/携箱marker与同一落地URDF独立FK一致，录制进程干净退出。

仿真与GUI启动证据不等同于用户交互认可或实机安全验收。历史零姿态数据在2026-09-11/12原目录及协作LOG中保留，不作为当前版成功清单。
