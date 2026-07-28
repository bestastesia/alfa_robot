# local_curobo 在 v5_dev 的实现与验证（2026-07-24）

> 2026-07-28 合并说明：本文第 1～7 节主要记录
> `/home/alfa/alfa_robot-v5_dev` 上的历史 GPU/FCL 验证证据。本实现已按最新
> `origin/v5_dev`（`c47fcc7`）架构选择性移植到 `feature/local_curobo`，没有整体覆盖最新
> planner。当前 ROS 权威关节名是 `updown,left_joint1..6,right_joint1..6`；backend 仅为
> 已验证的旧 cuRobo YAML 兼容 `leftjoint*/rightjoint*`。仓库内 `scene_*_current.yml` 是历史
> 固定几何快照，不代表最新 launch 默认容器场景。

## 0. 2026-07-28 最新分支复核

- 基线：`origin/v5_dev@c47fcc7342bf50164c673c3900f65dd7d2521c2d`；分支：
  `feature/local_curobo`。
- 构建成功；`robot_motion_curobo` 与 `alfa_robot_moveit_config` 共 44 项测试，0 失败。
- 真 GPU 兼容性 smoke（RTX 3060 Laptop，`collision_L1_R3`，3 次）：3/3 成功且质量 gate
  通过；增加 interpolated trajectory 逐点门禁后的 wall P50/P95/P99 为
  231.150/243.758/244.879 ms，solve 为 214.681/225.432/226.387 ms；仍低于 400 ms
  验收目标。interpolation P50/P95 为 14.747/15.081 ms，其中包含新增的逐点可行性检查。
- 最新分支的真实 PlanningScene 诊断确认：cuRobo 候选若与生产容器墙冲突，会被局部或
  最终 FCL 拒绝，并有界回退 local-RRT；该样本回退后最终 FCL 通过。backend 也会逐点
  检查 interpolated trajectory，并以 `curobo_interpolated_trajectory_collision@N` 明确
  拒绝自身模型中的碰撞轨迹。
- 最新 launch 默认容器几何与 2026-07-24 历史 fixture 不同。当前尚未得到“默认完整抓取
  自动选中碰撞 loaded shortcut 且 cuRobo patch 直接通过生产 FCL”的新样本；默认 L1/R3
  在进入 loaded 阶段前因抽离候选失败终止。因此下文历史成功率不能直接当作最新默认
  scene 的生产验收结果，部署时必须先对齐 cuRobo/MoveIt 场景再复测。

## 1. 验收口径

2026-07-24 的原始验证只修改 `/home/alfa/alfa_robot-v5_dev`；2026-07-28 的提交分支以
最新 `origin/v5_dev` 为基线逐块合并，没有把旧版 planner 节点整体覆盖到最新仓库。

目标是：

1. 保留最新仓库现有 local-RRT；
2. shortcut 经完整 PlanningScene/FCL 判碰后，可以二选一调用 local-RRT 或常驻 cuRobo；
3. cuRobo 失败、超时、响应非法或返回轨迹未通过 FCL 时，可以回退 local-RRT；
4. 修补只替换局部窗口，最终末点仍是调用者原始目标；
5. Rerun 和 CSV 能拆分 IK、抽离、cuRobo endpoint/graph/TrajOpt/interpolation、拼接和
   FCL 时间；
6. 用真实 CUDA/cuRobo 测量规划时间和轨迹质量。

## 2. 实现位置

| 层 | 文件 | 作用 |
| --- | --- | --- |
| 稳定接口 | `robot_motion_interfaces/srv/PlanJointSegment.srv` | 显式 13 轴起终点、附着箱、调用预算和分阶段响应时间 |
| 常驻后端 | `robot_motion_curobo/robot_motion_curobo/curobo_backend.py` | 一次加载并 warmup MotionGen，按请求更新 payload 球并规划 cspace |
| ROS 服务 | `robot_motion_curobo/robot_motion_curobo/joint_segment_planner_node.py` | 有界服务队列、响应计时和只读 cuRobo 碰撞审计 |
| C++ 客户端 | `alfa_robot_moveit_config/src/curobo_segment_client.cpp` | 异步服务、超时、joint order/边界/端点/时间戳严格校验 |
| 局部修补 | `alfa_robot_moveit_config/src/extract_monitor_transition_planning.cpp` | shortcut 分段验证、cuRobo 局部窗口、边界组合重试、拼接、RRT 回退、最终 FCL |
| loaded 接入 | `alfa_robot_moveit_config/src/loaded_pose_planning.cpp` | 左右臂负重过渡复用同一修补器并聚合诊断 |
| 参数装配 | `alfa_robot_moveit_config/src/dual_arm_planner_node.cpp` | `rrt/curobo` 二选一、服务 client、附着箱和最终 FCL callback |
| Rerun | `alfa_robot_moveit_config/scripts/extract_sequence_rerun.py` | 常驻服务生命周期、完整抓取 replay、local 阶段时间序列和候选诊断 |
| 基准 | `robot_motion_curobo/robot_motion_curobo/{cspace_tuning_benchmark,segment_benchmark_client}.py` | GPU 直连与 ROS 服务重复测试、轨迹质量、JSONL/Rerun 输出 |

默认 `extract_local_curobo_enabled=false`，因此不改变最新仓库原有运行入口。诊断参数
`extract_loaded_candidate_order_filter=-1` 默认不筛选候选；它只用于确定性复现某一
extract candidate。

## 3. 目标关节角没有被改写

`repair_with_local_rrt()` 中的 patch start/target 是原参考轨迹上的局部边界。边界回退或
扩展只用于给局部 planner 选择两个可行端点，局部结果之后仍拼回原轨迹后缀。

防线包括：

- `CuroboSegmentClient` 要求服务轨迹首点精确等于请求局部起点、末点精确等于请求局部
  终点；
- 拼接后对完整轨迹做无 planning-group 过滤的 PlanningScene/FCL 检查；
- loaded 结果记录完整末点与原始 loaded goal 的 `final_goal_max_error_rad`；
- 本轮完整 L1/R3 成功样本的该误差为 `2.78e-17 rad`，数值上等于 0。

测试中曾把 loaded goal 显式设为全 0 以制造碰撞。cuRobo 在第 14 次边界搜索成功生成
97 点局部段，但原全 0 后缀自身与集装箱顶板碰撞，最终流程被 FCL 正确拒绝。实现没有
把局部 patch target 当作最终目标，也没有为了让测试成功而替换用户目标。

## 4. 真实 GPU 结果

测试设备为 RTX 3060 Laptop GPU（6 GiB）。cuRobo checkout 和当前机器人 YAML 是本机
运行时依赖，通过命令行显式传入，没有写进仓库默认值。

### 4.1 碰撞局部段，direct-first，20 次

任务：`collision_L1_R3`；结果文件：

- `/tmp/alfa-v5-curobo-benchmark/tuned_collision_L1_R3_r20.json`
- `/tmp/alfa-v5-curobo-benchmark/tuned_collision_L1_R3_r20.csv`

结果为 20/20 规划成功、20/20 最终可行性/质量 gate 通过：

| 指标 | P50 | P95 |
| --- | ---: | ---: |
| 端到端 wall | 180.743 ms | 185.145 ms |
| GPU solve | 177.132 ms | 181.042 ms |
| endpoint feasibility | 9.630 ms | 10.263 ms |
| graph | 0 ms | 0 ms |
| TrajOpt | 167.326 ms | 171.198 ms |
| interpolation/extract | 1.271 ms | 1.604 ms |

该样本 direct TrajOpt 成功，所以没有支付 graph 时间；graph fallback 没有被删除。

质量指标：

- 最终 FCL/可行性：true；
- 规划关节运动量 / 参考运动量：`1.00708`；
- 最大相邻关节步长：`0.0253619 rad`；
- 平滑度代价：`0.000457794`；
- 轨迹时长：`1.6 s`。

### 4.2 真实 ROS 服务，3 次

结果目录：`/tmp/alfa-v5-curobo-service-benchmark/`。

3/3 成功；P50 service roundtrip `187.204 ms`、wall `192.282 ms`、endpoint
`9.492 ms`、TrajOpt `172.665 ms`、interpolation `1.261 ms`；每条 65 点，累计关节
运动量 `6.7689 rad`。

成功轨迹已保存为：

- `/tmp/alfa-v5-curobo-service-benchmark/collision_L1_R3_success.jsonl`
- `/tmp/alfa-v5-curobo-service-benchmark/collision_L1_R3_success.rrd`（约 64 MiB）

### 4.3 常见 loaded 端点

5 个 loaded 任务各重复 3 次。可行的两个任务：

| 任务 | 成功/质量 | wall P95 | endpoint | graph | TrajOpt | interpolation | 路径比 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| loaded_L1_R3 | 3/3 | 256.924 ms | 9.169 ms | 79.481 ms | 165.128 ms | 3.649 ms | 1.0000006 |
| loaded_L6_R8 | 3/3 | 231.986 ms | 10.215 ms | 48.702 ms | 169.555 ms | 2.336 ms | 1.000196 |

`loaded_L1_R8`、`loaded_L6_R3` 的起点已被 cuRobo 当前机器人碰撞模型判为不可行；
`loaded_L11_R8` 的保守 payload 球在起点/终点与狭窄开口相交。这些是保留
local-RRT/FCL 回退的直接理由，不能宣称 cuRobo 对所有场景都可替代。

## 5. 完整抓取与 Rerun

当前已保存的完整成功抓取：

- L1/R3 local-RRT/shortcut 全流程：
  `/tmp/alfa-v5-full-rerun-local-rrt/L1_R3_local_rrt.rrd`；
  total `14502.2 ms`、IK `350.413 ms`、extract `13543.4 ms`、loaded
  `366.084 ms`、final `19.905 ms`。
- L6/R8 local-RRT/shortcut 全流程：
  `/tmp/alfa-v5-l6r8-q1d5-rerun/L6_R8_full.rrd`；使用项目已验证的
  `quality_success_quorum=1 + loaded_distance_sum<=5.0` 抽离早停，total
  `26089.5 ms`、IK `424.219 ms`、extract `24835.2 ms`、loaded
  `436.255 ms`、final `21.546 ms`。

为避免 `/tmp` 清理，上述两份常见任务 Rerun 另存于被 `.gitignore` 排除的本地验证目录：

- `data/ik_benchmark/local_curobo_v5_dev/common_tasks/L1_R3_local_rrt.rrd`
- `data/ik_benchmark/local_curobo_v5_dev/common_tasks/L6_R8_full.rrd`
- L1/R3 安全测试目标全流程：
  `/tmp/alfa-v5-full-rerun-curobo-pose75/L1_R3_local_curobo.rrd`；
  该次完整流程成功且末点误差 `2.78e-17 rad`，但选中轨迹直线可行，cuRobo 调用数为
  0，因此只作为“启用 cuRobo 不改变正常 shortcut”的证据，不计作介入成功。

真实 full-flow 人为全 0 测试中，cuRobo 已在生产 C++ 窗口/拼接路径内成功生成并通过
局部 FCL 的绕障段（调用 14，graph `594.680 ms`、TrajOpt `184.531 ms`、97 点，局部
窗口 3→38）。流程随后在原轨迹 waypoint 39 的顶板碰撞处失败；因此保存了失败 Rerun
用于诊断，但没有把它报告为完整成功：

- `/tmp/alfa-v5-full-rerun-curobo-budget/L1_R3_local_curobo.rrd`

这说明“真实 cuRobo 已经介入并完成一个局部修补”和“整条人为目标轨迹可执行”是两个
不同 gate。后者必须继续由完整 FCL 决定。

为形成一条可连续查看、又不伪装成自然目标的成功诊断全流程，最新代码新增只规划不执行
的 `/dual_arm_planner/plan_local_segment_repair` 和
`compose_local_curobo_full_replay.py`。流程由以下真实、逐段 FCL 通过的部分组成：

1. 最新 L1/R3 自然抓取、吸附、抽离和 selected loaded 轨迹；
2. 从自然 loaded 末态到已知碰撞工况的桥接；该段先尝试 cuRobo，最终由保留的
   local-RRT fallback 成功；
3. 已知 shortcut 碰撞段由最新 production transition planner 调用真实 cuRobo 修补；
4. 在请求目标精确保持 30 帧，便于检查终点。

生产修补段结果：`method=shortcut_local_curobo`、cuRobo calls/segments=`1/1`、
local-RRT segments=`0`、fallback=`false`、final FCL=`true`；wall `375.605 ms`、
endpoint `15.096 ms`、TrajOpt `260.878 ms`、interpolation `2.865 ms`、30 个最终
稠密点、累计关节运动 `6.76878 rad`。桥接段最终 FCL 通过，并明确记录
`method=shortcut_local_rrt`、fallback=`true`。

合成器的硬校验结果：自然 full→bridge 边界误差 `0`，bridge→repair 边界最大误差
`2.54e-8 rad`，cuRobo 修补末点对请求 goal 最大误差 `1.46e-7 rad`。输出为：

- `/tmp/alfa-v5-production-repair/full_flow/L1_R3_full_flow_with_local_curobo_repair.jsonl`
- `/tmp/alfa-v5-production-repair/full_flow/L1_R3_full_flow_with_local_curobo_repair.rrd`

项目内本地副本位于
`data/ik_benchmark/local_curobo_v5_dev/full_flow/`（该目录被忽略，不进入源码提交）。

Rerun 共 22 阶段、312 个采样。summary 和 info panel 保留警告：碰撞工况是在真实自然
抓取完成后注入的诊断目标，不是普通 L1/R3 自动选择的 loaded goal。

## 6. 自动测试

已经通过：

- `robot_motion_curobo` Python 单测：19 个；
- `test_curobo_segment_client`：真实 ROS mock service + C++ client + transition planner；
- `test_extract_monitor_transition_planning`；
- `test_extract_monitor_json`；
- package boundary 检查。

覆盖内容包括：

- 返回 joint order 重排、有限数、关节 bounds、严格首尾点和递增时间；
- shortcut 碰撞后首选 cuRobo 并通过最终 FCL；
- cuRobo 起点判碰后回退，随后目标判碰再扩展的组合重试；
- 服务缺失和超时后 local-RRT 回退；
- Rerun/snapshot 中各阶段时间和候选诊断字段。

完整 workspace 测试中仍有一个与本功能无关的已有 safety 检查失败：最新快照缺少
`scripts/lhy_dev/send_dual_grasp_sequence.py`。本轮没有为绕过该基线问题修改安全规则。

## 7. 当前限制与后续验收

- 四个历史常见前侧吸 shortcut 样本本身都是直线无碰撞，项目旧日志也明确写着当时
  local-RRT 分支未触发；它们不能用于证明 cuRobo 介入。
- 已有真实 collision segment 和生产修补服务证明 cuRobo 可绕开 shortcut 碰撞；连续
  诊断 Rerun 证明真实抓取、local-RRT 桥接和 cuRobo 修补可按严格边界连接。
- 要获得“自然自动选择的抓取任务、最终成功且选中 loaded 轨迹本身包含真实 cuRobo 段”
  的单一 Rerun，仍需要
  一个满足以下条件的数据集：原 local-RRT 确实介入成功、同一局部起终点在当前 cuRobo
  模型中可行、原始最终目标及剩余后缀也通过 FCL。不得通过修改最终目标或删除障碍物来
  制造该结果。
