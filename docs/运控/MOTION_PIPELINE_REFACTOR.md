# 双臂抓取运控流程封装说明

本文档用于给其它仓库、其它部门 AI 或工程师对接当前双臂抓取流程。当前代码仍在 `alfa_robot_moveit_config` 包内，但已经把原来集中在 `dual_arm_planner_node.cpp` 的长流程拆成一组按职责划分的 C++ 模块，便于后续迁移到独立运控包或 IK 服务包。

工程化护栏、碰撞真相源、关节命名/方向契约、motion baseline 记录规则见：`docs/运控/工程化护栏/MOTION_ENGINEERING_GUARDS.md`。

## 1. 当前流程总览

当前固定版 monitor 流程支持完整放置循环；关闭 `extract_monitor_place_cycle_enabled` 时仍可只运行到负重位置。

```text
箱子编号 / 视觉目标
  -> 箱垛几何：生成左右末端抓取目标、箱墙开洞、集装箱障碍
  -> 抓取 IK：按侧吸 / 顶吸高度窗生成 h 候选，对每个 h 调解析 IK 生成左右臂解组合并按 cost scorer 排序
  -> IK 候选整理：过滤 legal candidate，按 score 排序，相似姿态去重，截断 TopN
  -> 抽离搜索：对候选 IK 做左右臂抽离 rollout，检查末端箱/机器人/集装箱/箱墙碰撞
  -> 负重姿态选择：对抽离成功结果选择最近的负重姿态族
  -> 负重规划：MoveIt 从抽离末态规划到负重 joint state，附着箱仍参与碰撞
  -> 预放置规划：从原负重姿态 `[0,-45,120,-75,0,0]` 平滑经过预放置姿态 `[0,-90,120,-75,0,0]`
  -> 放置与释放：继续规划到放置姿态，全程携带箱参与碰撞；到位后从 PlanningScene 和 RobotState 同时解除附着
  -> 空载返程：从放置姿态规划回原负重姿态
  -> 记录：JSONL / CSV / Rerun 回放
```

核心原则：IK、抽离、负重规划都保留阶段记录和失败原因；重构只移动代码位置，不改变算法语义。代码文件按职责聚合，不再按每个小类单独切文件，避免后续对接时“到处找算法碎片”。

## 2. 代码入口与责任划分

| 模块 | 责任 | 当前文件 |
| --- | --- | --- |
| `robot_motion_scene_service/motion_core/task_geometry` | 箱子编号、箱垛坐标、抓取 pair、基础碰撞几何数据结构，以及 `updown + 双臂 12 轴` 的标准目标关节顺序 | `ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_core/task_geometry.hpp` / `ros2_ws/src/robot_motion_scene_service/src/motion_core/task_geometry.cpp` |
| `motion_core/pose_math` | 角度解析、抓取姿态、Pose/Eigen 转换、误差计算、JSON 辅助 | `include/alfa_robot_moveit_config/motion_core/pose_math.hpp` / `src/motion_core/pose_math.cpp` |
| `robot_motion_scene_service/motion_core/scene_geometry` | 集装箱板、动态箱墙、末端附着箱、AABB 与邻箱脱离判断 | `ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_core/scene_geometry.hpp` / `ros2_ws/src/robot_motion_scene_service/src/motion_core/scene_geometry.cpp` |
| `MotionSceneAdapter` | 将场景几何转换为 MoveIt collision/attached objects，并管理 ADD/REMOVE 与当前场景状态；当前是库级 Adapter，不是独立 ROS 节点 | `ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_scene_adapter.hpp` / `ros2_ws/src/robot_motion_scene_service/src/motion_scene_adapter.cpp` |
| `optimized_ik_pipeline` | 抓取 IK 选优整体算法：`OptimizedDualIkSolver` 负责 h 高度窗、解析 IK 候选枚举、cost scorer 排序；`IkCandidateSelector` 负责 legal candidate 排序、相似姿态去重和 TopN 截断 | `include/alfa_robot_moveit_config/optimized_ik_pipeline.hpp` / `src/optimized_ik_pipeline.cpp` |
| `extract_planning_pipeline` | 抽箱子整体算法：抽离动作模板、单步 KDL IK、抽离候选评分、单臂/双臂 rollout、候选调度、CSV/summary 统计都在这里 | `include/alfa_robot_moveit_config/extract_planning_pipeline.hpp` / `src/extract_planning_pipeline.cpp` |
| `loaded_pose_planning` | 负重姿态阶段整体算法：从抽离末态选择最近负重姿态族，并调用 MoveIt 批量规划到负重 joint state | `include/alfa_robot_moveit_config/loaded_pose_planning.hpp` / `src/loaded_pose_planning.cpp` |
| `MotionFlowRecorder` | JSONL 文件、stage 序号、轨迹/summary 记录写入 | `include/alfa_robot_moveit_config/motion_flow_recorder.hpp` / `src/motion_flow_recorder.cpp` |
| `BoxStackFlowOrchestrator` | 传统 box-stack flow 的按轮任务顺序、预抓取/抓取/负重/回预抓取流程编排 | `include/alfa_robot_moveit_config/box_stack_flow_orchestrator.hpp` / `src/box_stack_flow_orchestrator.cpp` |
| `ExtractDemoOrchestrator` | 抽离 demo 的单 pair / 多 pair 遍历、失败传播和 summary 记录 | `include/alfa_robot_moveit_config/extract_demo_orchestrator.hpp` / `src/extract_demo_orchestrator.cpp` |
| `extract_monitor_state` | 交互式 monitor 的阶段状态机、候选缓存、候选任务调度、抽离/负重统计、最终候选选择 | `include/alfa_robot_moveit_config/extract_monitor_state.hpp` / `src/extract_monitor_state.cpp` |
| `extract_monitor_json` | monitor 的候选、阶段、快照、replay extra 字段 schema | `include/alfa_robot_moveit_config/extract_monitor_json.hpp` / `src/extract_monitor_json.cpp` |
| `ExtractMonitorSnapshotWriter` | monitor 快照文件读写，保证目录创建和 JSON 落盘错误集中处理 | `include/alfa_robot_moveit_config/extract_monitor_snapshot_writer.hpp` / `src/extract_monitor_snapshot_writer.cpp` |
| `ExtractMonitorTransitionPlanner` | monitor 最终回放中的通用关节过渡规划策略：插值、densify、碰撞验证、失败后局部 RRT/MoveIt 回退、shortcut、再次验证；用于负重→IK、负重→预放置→放置和空载返程 | `include/alfa_robot_moveit_config/extract_monitor_transition_planning.hpp` / `src/extract_monitor_transition_planning.cpp` |
| `ExtractMonitorReplayBuilder` | monitor 最终采用方案的 Rerun/JSON 回放阶段组装：预吸附过渡、抽离记录、横向让位、负重规划按固定顺序合并 | `include/alfa_robot_moveit_config/extract_monitor_replay_builder.hpp` / `src/extract_monitor_replay_builder.cpp` |
| `DualArmPlannerNode` | ROS 参数、MoveIt 后端、场景碰撞判定、service callback 装配 | `src/dual_arm_planner_node.cpp` |
| 启动配置 | 暴露算法超参数和实验参数 | `launch/dual_arm_planner.launch.py` |
| 回放工具 | 将 JSONL 转为 Rerun 场景 | `scripts/visualize_moveit_box_stack_flow.py` |

兼容说明：`alfa_robot_moveit_config/include/alfa_robot_moveit_config/motion_core/task_geometry.hpp`、`motion_core/scene_geometry.hpp`、`motion_scene_adapter.hpp` 仍保留为转发头，方便旧 include 路径继续编译；真实实现已经迁入 `robot_motion_scene_service`，旧 `alfa_robot_moveit_config/src/motion_core/*` 和 `src/motion_scene_adapter.cpp` 不再保留重复实现。

## 3. 模块间数据流

### 3.1 箱垛与抓取目标

- `make_boxes(box_front_x)` 生成当前 3×4 箱垛坐标。
- `parse_box_pair_list()` / `make_pick_pairs()` 生成抓取 pair。
- `make_front_grasp_pose()` / `make_top_suction_pose()` 在 `motion_core/pose_math` 内结合抓取模式生成左右末端 Pose。
- 集装箱和箱墙几何来自 `robot_motion_scene_service/motion_core/scene_geometry`，再由 `MotionSceneAdapter` 注入 MoveIt。

### 3.2 抓取 IK

- `OptimizedDualIkSolver::solve()` 是当前抓取 IK 的 Adapter。
- 输入：左右末端 Pose、当前 seed state、抓取模式。
- 内部：根据侧吸 / 顶吸高度窗计算 `h_interval`，围绕当前 `updown` 生成 `h_candidates`；对每个 h 分别调用 `alfa_robot_analytic_ik::ThreeParallelArmAnalyticIk` 求左右臂解析解，再组合成双臂候选。
- 排序：每个候选记录 FK 误差、`updown_delta`、`joint_delta`、负重姿态族距离，并用 cost scorer 排序。
- 输出：selected joint state、完整 IK 审计 JSON、所有候选统计。
- 当前保留 `ik_seed_count` 字段主要是兼容旧 JSON / CSV schema；解析 IK 不再依赖随机 seed。`ik_workers` 默认已经降为 1，常规入口不再创建 16 个 IK worker。

### 3.3 IK 候选去重

`IkCandidateSelector::select()` 只处理 legal candidate：

1. 输入候选已经按 `score` 从低到高排序。
2. 逐个遍历，若候选和已保留候选相似，则丢弃；否则保留。
3. 最后按 `candidate_limit` 截断。

默认相似标准：

- `h` 差值不超过 `extract_ik_dedup_h_threshold=0.005m`。
- 双臂 12 个关节的 wrapped angle 差值都不超过 `extract_ik_dedup_joint_threshold_deg=1°`。

这一步复杂度是朴素 `O(N×U×12)`，但当前几百个候选规模下通常约 1~2ms，不是瓶颈。

### 3.4 抽离搜索

- `ExtractBenchmarkRunner` 取 IK 候选，按配置调度 `ExtractRolloutPlanner`。
- `ExtractRolloutPlanner` 负责单臂/双臂 rollout 状态机：
  - 每一步由 `ExtractMotionPlanner` 给出后退/上抬/pitch 候选。
  - `ExtractCandidateSolver` 对候选目标做单臂解析 IK 求解，并固定当前 `updown`。
  - `ExtractCandidateScorer` 在合法候选中选择局部最低代价。
  - 通过 callback 调用 `DualArmPlannerNode` 内的碰撞检查，保留现有 MoveIt PlanningScene 语义。
- 支持异步双臂抽离：左右臂分别求抽离路径，再组合检查全程双臂和环境碰撞。
- 支持成功/失败早停：脱离邻箱后可额外走少量步；当前步所有方向失败则该 IK 候选失败。

### 3.5 抽离后负重规划

- `LoadedPoseSelector` 根据抽离末态，从左右各 3 个负重姿态族中选择最近目标。
- `LoadedPoseSelector` 同时负责把最近负重姿态距离、L2 距离和最大关节差写回 `ExtractRolloutTiming`，供候选排序、CSV 和 monitor 快照复用。
- `LoadedPosePlanner` 在保留末端附着箱的情况下规划到负重姿态；`extract_loaded_planning_mode=rrt` 时调用 MoveIt/OMPL，`shortcut` 时直接生成关节空间插值并用同源 PlanningScene/FCL 校验全程碰撞。
- `ExtractBenchmarkRunner` 可对抽离成功候选按负重姿态距离排序，按 `extract_loaded_candidate_limit` 截断，并可 `extract_loaded_stop_on_first_success` 首成功即停。


### 3.5.1 负重规划模式对比

2026-07-09 对当前可稳定进入“抽离后规划到全 0 负重姿态”的四组侧吸任务 `L1/R3`、`L1/R8`、`L6/R3`、`L6/R8` 复测了三种负重规划模式。测试条件：Release 构建，`box_front_x=0.925`、`scene_y_shift=-0.4`、`fixed_updown=0.3`、侧吸高度窗 `0.45~1.25`、进入负重规划的候选数 8、负重规划 worker 8。

注意：旧六组任务 `L1/R2; L6/R3; L7/R8; L11/R12; L16/R13; L17/R18` 在当前机械/任务脚本口径下第一组 `L1/R2` 已失败在最终选择阶段，没有进入负重规划，因此不能用来公平比较 RRT/RRT*/shortcut。

| 模式 | 成功率 | 平均总耗时 | 平均负重规划 batch | 平均累计关节运动量 | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| RRTConnect | 4/4 | 1631.6ms | 1233.5ms | 27.86rad | 稳定但负重阶段约 1.2s |
| RRT* | 4/4 | 1633.8ms | 1235.7ms | 24.73rad | 这组任务没有明显快于 RRTConnect |
| Shortcut | 4/4 | 586.1ms | 208.0ms | 23.74rad | 当前四组最优，前提是关节空间直连经碰撞校验可行 |

复测数据：`data/ik_benchmark/loaded_planning_mode_compare_front4_release_current/summary_compare.csv`。
Shortcut 回放：`data/ik_benchmark/loaded_planning_mode_compare_front4/shortcut_rerun/shortcut_front4_full.rrd`。

工程含义：负重阶段如果不需要绕障，优先使用 `extract_loaded_planning_mode:=shortcut`；若 shortcut 被 PlanningScene/FCL 判碰撞失败，再 fallback 到 `rrt`，不要默认让 RRT 承担直连可行的场景。

### 3.6 记录与回放

- `MotionFlowRecorder` 写 JSONL header/stage/summary。
- `ExtractBenchmarkCsvWriter` 写逐候选 timing CSV。
- `visualize_moveit_box_stack_flow.py` 将 JSONL 转成 Rerun，系统 Python `/usr/bin/python3` 下可用。

### 3.7 交互式 monitor 流程

`extract_stage_monitor_console.py` 用于按一次命令运行或分阶段观察 `IK → 抽离 → 负重规划 → 最终回放`。它仍然通过 `DualArmPlannerNode` 的 ROS service 触发计算，但计算结果的状态、快照和 replay schema 已经拆到独立模块：

1. `ExtractMonitorController` 根据当前 phase 调用 IK、抽离、负重、最终四个阶段；节点只提供四个阶段 Adapter。
2. IK 阶段调用 `OptimizedDualIkSolver` 后，由 `IkCandidateSelector` 排序/去重，再由 `populate_extract_monitor_candidate_states()` 建候选状态缓存。
3. 抽离阶段通过 `run_extract_monitor_candidate_tasks()` 统一调度候选任务，节点只描述“单个候选如何 rollout”。
4. 负重阶段调用 `LoadedPosePlanner::planBatch()` 后，由 `summarize_loaded_plan_timings()` 汇总 attempted/success/failure。
5. 最终阶段通过 `select_extract_monitor_final_timing()` 选择候选；`ExtractMonitorReplayBuilder` 负责把预吸附过渡、抽离记录、横向让位和负重规划合并成最终回放；其中预吸附过渡由 `ExtractMonitorTransitionPlanner` 执行，字段 schema 仍由 `extract_monitor_json` 提供。

当前 monitor 相关测试：

- `test_extract_monitor_state`：阶段状态机、候选缓存、调度、统计和最终选择规则。
- `test_extract_monitor_json`：快照和 replay 字段 schema。
- `test_extract_monitor_replay_builder`：最终采用方案 replay 阶段顺序、预吸附过渡 fallback 和缺失起点处理。
- `test_extract_monitor_snapshot_writer`：快照写入错误处理。
- `test_extract_monitor_transition_planning`：预吸附过渡规划的插值成功、RRT fallback 和失败传播。
- `test_loaded_pose_selector`：最近负重姿态选择、timing 距离指标写回和目标姿态生成。

这部分的迁移建议：不要复制 `run_extract_monitor_*` 的线性实现；应优先迁移上述四个 monitor 模块，再在新仓库里重新写 ROS service Adapter。

### 3.8 启动稳定性 smoke

为了避免“上一轮 ROS 进程没关干净，下一轮测试连到旧服务”的问题，当前自启动 monitor/benchmark 脚本默认使用独立 `ROS_DOMAIN_ID=auto`，并在退出时按进程组清理 `ros2 launch` 及其子进程。公共逻辑在：

- `ros2_ws/src/alfa_robot_moveit_config/scripts/process_lifecycle.py`
- `ros2_ws/src/alfa_robot_moveit_config/test/test_process_lifecycle.py`

推荐 AI 或工程师复测启动稳定性时直接运行：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run alfa_robot_moveit_config extract_startup_stability_smoke.py \
  --rounds 3 \
  --ros-domain-id auto
```

这个 smoke 会在同一个隔离 ROS 域内连续执行 3 轮：

1. 启动 `dual_arm_planner.launch.py`。
2. 等待 `/dual_arm_planner/run_extract_monitor_full_selected` 服务。
3. 调一次完整 `IK → 抽离 → 横向让位 → 负重规划`。
4. 校验返回的 snapshot 路径必须属于本轮，避免误连旧 planner。
5. 检查日志中旧关节名污染 `left_joint/right_joint` 必须为 0。
6. 关闭 planner 进程组，并确认 `/dual_arm_planner` 服务消失。

默认情况下，完整流程本身如果因为 IK/RRT 随机性没有选出最终方案，只会记录为 `flow_success=false`，不会把启动稳定性 smoke 判失败；因为这个脚本主要验证的是“服务能稳定启动、不会连旧节点、退出后不残留”。如果需要把算法完整成功也作为硬门槛，加 `--require-flow-success`。

如果需要故意连接外部已经启动的 ROS 图，可以传 `--ros-domain-id inherit`；否则默认推荐保留 `auto`。

### 3.9 关于算法进程和任务进程拆分

本轮启动稳定性问题的主要根因不是“算法和任务必须拆成两个 ROS 进程”，而是：

- 旧 `ros2 launch` 子进程没有随父进程完全退出，留下旧 `/dual_arm_planner` 服务。
- 自启动测试复用默认 ROS 图，容易被旧 `/joint_states` 或旧服务污染。
- fixed-h IK 流程在部分路径上提前碰到 free-h 求解池初始化，造成首次调用变慢。

这些问题已经通过 `process_lifecycle.py`、`ROS_DOMAIN_ID=auto` 和 IK 懒初始化处理。因此当前不建议为了“看起来分层”立刻把算法进程、任务进程硬拆开：如果只是把现有 callback 包一层 ROS service，而调用顺序、场景状态、MoveIt planning scene 和候选缓存仍然全部外泄，那个 seam 会很浅，接口复杂度接近实现复杂度，反而更容易生成新残留进程和新同步问题。

后续如果真的要拆进程，建议先满足两个条件：

1. `DualArmPlannerNode` 内的 ROS/MoveIt Adapter 继续变薄，核心算法只通过明确 request/result 结构交互。
2. 至少存在两个真实 Adapter，例如“同进程直接调用”和“跨进程 ROS 调用”，否则这个 seam 还只是理论 seam。

当前更实际的维护策略是：

- 用 `extract_startup_stability_smoke.py` 作为启动/关闭 gate。
- 继续把 `DualArmPlannerNode` 中的纯算法请求构造、快照、记录、回放语义下沉到已有 deep module。
- 保留一个一键总体启动/测试入口，避免 AI 或工程师为了验证一次流程手动拼多条命令。

### 3.10 全流程服务复用与预热边界

当前全流程实验不要按“每组箱子启动一次 planner”的方式跑。那样会把 MoveIt、PlanningScene、controller 和解析 IK / PlanningScene 首次初始化都算进单次任务，导致阶段耗时口径失真。正确口径是：

```text
启动期：
  启动 dual_arm_planner.launch.py
  -> 等待 /dual_arm_planner/configure_extract_monitor
  -> 调 configure_extract_monitor 做首个任务配置和 IK solver 预热

每个任务：
  configure_extract_monitor(left_box_id, right_box_id, snapshot_path)
  -> run_extract_monitor_full_selected
  -> 读取 snapshot / 生成 Rerun / 记录阶段耗时

收尾：
  关闭 planner 进程组
  -> 等待 /dual_arm_planner/* 服务消失
```

必须常驻复用的部分：

- `dual_arm_planner_node`：保留 MoveIt 后端、场景状态、monitor 状态机入口、IK solver 缓存。
- `move_group`、`robot_state_publisher`、`ros2_control_node`、controller spawner：由 `dual_arm_planner.launch.py` 管理，整段序列只启动一次。
- `OptimizedDualIkSolver` / 解析 IK / PlanningScene 缓存：通过 `configure_extract_monitor` 触发初始化，后续任务复用。

每个任务允许重置的部分：

- 当前抓取箱号、箱墙开洞、左右 box 目标、snapshot 路径。
- 箱墙前表面距离、横向布局偏移、抽离模式、负重前侧向让位开关和箱体 RRT 迭代上限；这些字段更新时，集装箱与箱墙碰撞体会使用同一份运行时几何重新写入 PlanningScene。
- monitor 阶段状态、上一阶段耗时、候选缓存和最终回放缓存。
- 末端携带箱与动态碰撞对象状态。

新增服务：

- `/dual_arm_planner/configure_extract_monitor`
- 类型：`alfa_robot_moveit_config/srv/ConfigureExtractMonitor`
- 基础字段：`left_box_id`、`right_box_id`、`snapshot_path`、左右吸附模式和显式目标 Pose。
- 运行时字段：`update_runtime_config=true` 时使用 `box_front_x`、`scene_y_shift`、`extract_rollout_mode`、`loaded_lateral_shift_enabled`、`extract_box_pose_rrt_max_iterations`。
- 作用：切换当前任务、重置 monitor 状态、设置 snapshot 输出路径、更新箱墙开洞，并确保 IK solver 已预热。

当前已接入该复用语义的入口：

- `scripts/extract_sequence_rerun.py`：整段 pair sequence 共享一个 planner；启动期预热一次，每组任务只 configure。
- `scripts/extract_stage_monitor_console.py`：自启动 planner 后先 configure/prewarm，再等待用户回车触发计算。
- `scripts/extract_failed_attempts_rerun.py`：失败样本分阶段回放前先 configure/prewarm。
- `scripts/execute_l6_r8_mock_live.py` / `execute_l6_r8_real_live.py`：计算到执行链路启动后先 configure/prewarm。
- `scripts/extract_startup_stability_smoke.py`：把“服务就绪 + IK 预热”计入 startup，而不是计入单次 IK。
- `scripts/run_extract_live_benchmark.py`：旧实时演示入口仍被安装；现在启动后也会先 configure/prewarm，再触发 `run_left_extract_demo`。
- `tools/demonstration0720/armmotion`：`algorithm_thread` 启动时创建一个 planner 长驻会话；`A1..A5/B1..B5` 后续任务只做动态 configure 和 trigger，不再为每个任务启动 MoveIt。

耗时解释：

- `startup_ms`：启动 ROS/MoveIt/controller，加上首次 configure/prewarm；这是整段序列启动成本，不是单任务算法耗时。
- `configure_ms`：单任务切换成本；C++ 内部通常应为几十毫秒以内。长序列入口应复用 `ExtractMonitorServiceClient`，不要每个任务重新创建 rclpy node，否则 Python/DDS 客户端开销会把外部墙钟放大到数百毫秒。
- `ik_elapsed_ms`：预热后的真实抓取 IK 阶段耗时；当前解析 IK + 64 个 h 候选通常约 `55~60ms`，不再是原 BioIK 时代的 `0.5~0.8s`。
- `loaded_elapsed_ms` / `loaded_ms`：负重阶段总耗时，包含候选排序、批量规划、records/snapshot 生成等阶段包装成本。
- `loaded_plan_batch_wall_ms`：负重候选并行规划批次本身的墙钟耗时，更适合用来判断 RRT/MoveIt 规划是否拖慢流程；这个仍受 RRT 随机性影响，是当前秒级波动的主要来源。

维护规则：

- 新增全流程入口时，必须遵守“启动/预热一次 → 每任务 configure → trigger compute”的顺序。
- 不要为了换箱号重启 `dual_arm_planner_node`；除非任务目标就是测试启动稳定性。
- 如果跳过 `configure_extract_monitor` 直接调 `run_extract_monitor_full_selected`，首次调用仍可能触发懒初始化，耗时统计会再次混入启动成本。
- `extract_startup_stability_smoke.py` 是例外：它刻意重启 planner，用于验证进程清理和启动稳定性；它已经把预热时间归到 startup。


### 3.11 当前接口 seam、事实源与运行时服务图

当前代码已经形成几个有用的 Module，并开始拆出独立 ROS 运行时服务。按 interface 看，现状如下：

| 阶段 | 当前 Module / seam | 输入 | 输出 | 当前问题 |
| --- | --- | --- | --- | --- |
| 抓取 IK | `OptimizedDualIkSolver` / `analytic_arm_ik_service_node` | 左右末端 Pose、seed `RobotState`、抓取模式、高度窗参数；或单臂 Pose + seed + fixed updown | `UpdownAwareIkResult`、selected `RobotState`、候选审计 JSON；或 `SolveArmIk` 多解列表 | 主流程仍走算法 Module；单臂解析 IK 已有独立 ROS service 包装 |
| IK 去重/截断 | `IkCandidateSelector` | 已排序 legal candidates、去重阈值、TopN | 去重后的候选列表和统计 | 复杂度 `O(N×U×12)`，当前规模不是瓶颈 |
| 抽离 | `ExtractRolloutPlanner` / `ExtractBenchmarkRunner` | IK candidate state、左右 carried box、箱墙几何、碰撞 callback | 每个候选的抽离结果、失败原因、replay records | 算法 Module 已拆出；碰撞 callback 仍由 `DualArmPlannerNode` 提供 |
| 负重规划 | `LoadedPosePlanner` | 抽离末态、负重姿态族、carried boxes、PlanningScene callback | 负重轨迹、选择的负重姿态、失败原因 | 支持 `rrt` 和 `shortcut`；MoveIt 后端仍在节点内 |
| 场景建模 | `robot_motion_scene_service::MotionSceneAdapter` | 集装箱参数、箱墙开洞、附着箱规格 | MoveIt collision objects / PlanningScene snapshot 更新 | 当前是库级 Adapter，不是独立 ROS 节点 |
| 碰撞检查 | `make_full_scene_snapshot()` + `planned_trajectory_clear_in_full_scene()` / `motion_collision_service_node` | 任意 `RobotState`、轨迹、scene objects、attached boxes | FCL 碰撞通过/失败原因/contacts | 算法内部和独立 ROS service 都支持 explicit requested state，不只检查当前 `/joint_states` |
| 状态事实源 | `robot_motion_runtime/motion_state_source_node.py` + 显式 `RobotState` 参数 | 实际或仿真 joint state；或 `/robot_motion/set_state` | `/robot_motion/state`、规划起点 / 碰撞检查状态 | 已有独立 authoritative publisher；生产系统仍需保证只有一个 authoritative publisher |

关键原则：碰撞检查不能只依赖当前机器人状态。当前实现通过 `make_full_scene_snapshot(start_state, attached_boxes)` 克隆 PlanningScene，再显式 `setCurrentState(start_state)`，因此可以检查任意请求态和任意轨迹点。这是后续迁移到独立 collision service 时必须保留的 interface。

当前已新增域内轻量接口包 `motion_internal_interfaces` 和运行时包 `robot_motion_runtime`。`DualArmPlannerNode` 仍是完整箱垛实验的主流程 Adapter，但事实源、PlanExtract、PlanLoaded、ExecuteTrajectory 和最小任务编排已具备独立 ROS 节点形态：

- `motion_internal_interfaces/srv/SolveArmIk.srv`：单臂目标 Pose + seed state + fixed updown -> 多个单臂 IK 解；当前 `alfa_robot_moveit_config/analytic_arm_ik_service_node` 已实现该服务，默认服务名 `/robot_motion/solve_arm_ik`，支持 `base_link` 和 `{left,right}_arm_base` frame。
- `motion_internal_interfaces/srv/PlanDualArmIk.srv`：左右目标 Pose + seed state + fixed updown -> 双臂组合 IK candidate states；当前 `robot_motion_runtime/dual_arm_ik_candidate_service_node.py` 已实现该服务，默认服务名 `/robot_motion/plan_dual_arm_ik`。
- `motion_internal_interfaces/srv/PlanExtract.srv`：IK candidate states + carried boxes -> 抽离候选轨迹与选中项。
- `motion_internal_interfaces/srv/PlanLoaded.srv`：抽离末态集合 + 负重姿态族 + carried boxes + planning mode -> 负重规划候选轨迹与选中项。
- `motion_internal_interfaces/srv/CheckCollision.srv`：显式 `start_state` 或 `trajectory` + scene objects + attached boxes -> valid/reason/contacts；当前 `alfa_robot_moveit_config/motion_collision_service_node` 已实现该服务，默认服务名 `/robot_motion/check_collision`。该服务禁止静默替换成 live `/joint_states`。
- `motion_internal_interfaces/srv/ExecuteTrajectory.srv`：轨迹 + 速度参数 -> accepted/message；执行层只负责验证和转发，不关心后端是 mock 还是真机。
- `motion_internal_interfaces/srv/SetRobotMotionState.srv`：仿真/mock 启动时显式固定 `/robot_motion/state`；真实机器人通常由硬件反馈持续发布。
- `motion_internal_interfaces/srv/SetRobotMotionScene.srv`：仿真/mock 启动时显式固定箱墙、集装箱等碰撞对象。
- `motion_internal_interfaces/srv/RunMotionTask.srv`：最小任务编排契约；当前输入为 IK candidate states，服务内部串起 `PlanExtract -> PlanLoaded -> ExecuteTrajectory`。
- `motion_internal_interfaces/srv/RunDualArmPoseTask.srv`：目标位姿任务编排契约；服务内部串起 `PlanDualArmIk -> PlanExtract -> PlanLoaded -> ExecuteTrajectory`，用于把“下一次信息启动任务”从 IK candidate 输入推进到目标 pose 输入。
- `motion_internal_interfaces/srv/RunBoxPairTask.srv`：箱号任务 adapter 契约；输入左右箱号、吸附模式和箱墙几何，服务内部生成左右目标 Pose 与附着箱，再转发 `RunDualArmPoseTask`。
- `motion_internal_interfaces/msg/RobotMotionState.msg`：机器人姿态事实源样本；`robot_motion_runtime/motion_state_source_node.py` 可把指定 `/joint_states` 包装成 `/robot_motion/state`，也可通过 `/robot_motion/set_state` 固定仿真事实状态。生产运行时应只有一个节点发布 `authoritative=true`。
- `motion_internal_interfaces/msg/RobotMotionScene.msg`：场景事实源样本；`robot_motion_runtime/motion_scene_source_node.py` 可通过 `/robot_motion/set_scene` 固定箱墙、集装箱等碰撞对象；`PlanExtract/PlanLoaded` 会把请求内显式 scene 或最新 `/robot_motion/scene` 传给 `CheckCollision`。

这些 interface 当前是契约和第一版可运行服务骨架，不强制改变已有实验入口。它们的作用是防止迁移到 `robot_motion_control` 时继续把 IK、抽离、规划、碰撞和执行都塞在一个 demo 节点里。

运行时服务栈启动示例：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_motion_runtime runtime_services.launch.py \
  subscribe_joint_states:=false \
  execute_forward_action:=false
```

如果要让前端看到更完整的服务图，包括解析 IK 和碰撞服务，用完整栈：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_motion_runtime runtime_full_stack.launch.py \
  subscribe_joint_states:=false \
  execute_forward_action:=false
```

运行时前端：

```bash
xdg-open http://127.0.0.1:8766
```

当前 `robot_motion_runtime` 包含：

- `motion_state_source_node`：发布 `/robot_motion/state`，提供 `/robot_motion/set_state`。
- `motion_scene_source_node`：发布 `/robot_motion/scene`，提供 `/robot_motion/set_scene`。
- `dual_arm_ik_candidate_service_node`：提供 `/robot_motion/plan_dual_arm_ik`，调用左右两次 `SolveArmIk` 并组合候选。
- `box_pair_task_adapter_node`：提供 `/robot_motion/run_box_pair_task`，把当前 3×4 箱垛箱号、侧吸/顶吸模式和箱墙参数转换为左右目标 Pose 与 carried box，再调用 `/robot_motion/run_dual_arm_pose_task`。
- `plan_extract_service_node`：提供 `/robot_motion/plan_extract`。轻量模式是 deterministic shortcut；完整栈下会调用 `/robot_motion/check_collision` 并使用请求 scene 或 `/robot_motion/scene` 过滤候选轨迹。后续要把 C++ 抽离 rollout 迁入。
- `plan_loaded_service_node`：提供 `/robot_motion/plan_loaded`。当前生成关节空间 shortcut；完整栈下会调用碰撞服务并使用请求 scene 或 `/robot_motion/scene` 过滤候选。后续要把 RRT/local-RRT 迁入。
- `execute_trajectory_service_node`：提供 `/robot_motion/execute_trajectory`，可 dry-run 或转发到 FollowJointTrajectory action。
- `motion_task_orchestrator_node`：提供 `/robot_motion/run_task` 和 `/robot_motion/run_dual_arm_pose_task`；前者接收 IK candidates，后者接收左右目标 Pose 并串起 `PlanDualArmIk -> PlanExtract -> PlanLoaded -> ExecuteTrajectory`。
- `motion_runtime_dashboard_node`：提供 `http://127.0.0.1:8766`，显示服务是否启动、runtime status、最新 `RobotMotionState` 和 ROS graph。
- `runtime_full_stack.launch.py`：组合启动 `robot_motion_runtime`、`analytic_arm_ik_service_node` 和 `motion_collision_service_node`，用于验证完整服务图。
- `analytic_arm_ik_service_node`、`motion_collision_service_node` 已接入 `/robot_motion/runtime_status`，所以前端不只知道服务是否存在，也能看到请求次数、成功/失败次数和最近一次详情。

2026-07-09 场景事实源 smoke 已通过：在隔离 `ROS_DOMAIN_ID=228` 下启动 `runtime_full_stack.launch.py`，先调用 `/robot_motion/set_state` 固定 h=0.55 的 15 轴仿真状态，再调用 `/robot_motion/set_scene` 固定 1 个显式 collision object，最后调用 `/robot_motion/run_box_pair_task` 跑 L1/R3 侧吸 dry-run。返回 `ik=4`、`extract=4`、`loaded=4`，dashboard `/api/status` 能看到 `latest_scene.scene_object_count=1`，并且 `set_state`、`set_scene`、`run_box_pair_task`、`run_dual_arm_pose_task`、`plan_dual_arm_ik`、`solve_arm_ik`、`plan_extract`、`plan_loaded`、`check_collision`、`execute_trajectory` 均有 request_count。

解析 IK 服务启动示例：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_moveit_config analytic_arm_ik_service.launch.py
```

碰撞服务启动示例：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_moveit_config motion_collision_service.launch.py
```

旧 `alfa_robot_moveit_config/robot_motion_state_source.launch.py` 仍可用于历史验证；新运行时应优先使用 `robot_motion_runtime`：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_motion_runtime runtime_services.launch.py \
  subscribe_joint_states:=true
```

事实源约束：

- 正常运行时只允许一个 `/robot_motion/state` publisher 设置 `authoritative=true`。
- `RobotMotionState.context.state_id` 由来源和时间戳组成，用于把规划请求、碰撞请求和执行请求关联到同一个状态样本。
- 规划/碰撞服务如果请求里已经显式传入 `JointState`，应优先使用请求状态，而不是再读取 live `/joint_states`。

碰撞服务调用原则：

- `start_state` 必须显式传入，服务不会用 live `/joint_states` 自动替代。
- 如果 `trajectory.points` 为空，只检查 `start_state`。
- 如果 `trajectory.points` 非空，则从 `start_state` 逐点覆盖 trajectory joint positions 并检查每个轨迹点。
- `attached_boxes` 会转换成 MoveIt `AttachedCollisionObject` 后参与 FCL 检查；也可以直接传 `attached_collision_objects`。
- `use_current_scene_as_base=false` 时，服务只使用请求里的 `scene_objects` / attached objects 构建临时场景；`true` 时才克隆当前 PlanningScene 作为底图。

如果终端处于 Conda 环境，构建自定义 ROS interface 时必须显式使用系统 Python，否则会生成 `cpython-311` 的 typesupport，Humble 的 `ros2 service call` 会无法导入：

```bash
colcon build --packages-select motion_internal_interfaces alfa_robot_moveit_config \
  --symlink-install \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
```

## 4. 可复用库与迁移建议

当前 CMake 导出三个层级：

第一层是纯算法公共包 `robot_motion_core`。它当前统一 IK solver 配置、候选请求、候选结果和代价函数接口，不依赖 ROS node、MoveIt 或 benchmark：

```cmake
find_package(robot_motion_core REQUIRED)

target_link_libraries(your_target
  robot_motion_core::robot_motion_core
)
```

第二层是独立场景包 `robot_motion_scene_service`，负责场景几何和 MoveIt PlanningScene 适配：

```cmake
find_package(robot_motion_scene_service REQUIRED)

target_link_libraries(your_target
  robot_motion_scene_service::robot_motion_scene_core
  robot_motion_scene_service::robot_motion_scene_adapter
)
```

第三层是 `alfa_robot_moveit_config` 内的 MoveIt 流程适配库：

```cmake
find_package(alfa_robot_moveit_config REQUIRED)

target_link_libraries(your_target
  alfa_robot_motion_core
)
```

`alfa_robot_motion_core` 是历史名称，当前只包含姿态、角度、Pose/Eigen 转换和场景相关辅助，并链接 `robot_motion_scene_service::robot_motion_scene_core`。新代码不要把它误认为独立算法核心；纯算法公共接口应进入 `robot_motion_core`。

```cmake
target_link_libraries(your_target
  alfa_robot_motion_scene_adapter
)
```

`alfa_robot_motion_scene_adapter` 包含 IK 选优、抽离规划、负重规划、benchmark runner 和 monitor 辅助模块。它依赖 MoveIt、KDL、`robot_motion_scene_service` 和 `robot_motion_core`；不再包含或编译依赖 `scripts/ik_benchmark` 的公共头文件。

迁移到新运控包时建议顺序：

1. 先迁移 `robot_motion_scene_service`，让集装箱、箱墙、末端附着箱的碰撞口径先稳定下来。
2. 复用已建立的 `robot_motion_core`，继续迁移 `optimized_ik_pipeline` 中的候选排序、去重和评分算法。
3. 再迁移 `extract_planning_pipeline`，作为抽箱子动作生成、单步 IK、评分和 rollout 的完整模块。
4. 最后迁移 `loaded_pose_planning`，接入新 MoveIt/PlanningScene 后端。
5. `DualArmPlannerNode` 不建议整文件复制；它只应作为 ROS 参数、MoveIt 后端和 service/action 包装参考。

## 5. 关键启动参数

| 参数 | 当前默认 | 含义 |
| --- | --- | --- |
| `extract_demo_pair_sequence` | `2,4;7,9;12,14;17,19` | 当前固定版原抓取任务 |
| `front_z_reach_lower` / `front_z_reach_upper` | `0.45` / `1.25` | 侧吸目标高度窗：`updown + 0.45 ~ updown + 1.25` |
| `top_z_reach_lower` / `top_z_reach_upper` | `0.3` / `0.45` | 顶吸目标高度窗 |
| `ik_h_candidate_count` / `ik_seed_count` | `64` / `32` | 解析 IK 的 h 候选数量 / 兼容旧候选 schema 的 seed 字段 |
| `ik_workers` | `1` | 抓取 IK worker 数；解析 IK 默认单线程，必要时才显式覆盖 |
| `ik_candidate_timeout` | `0.01` | 兼容旧参数；解析 IK 路径不依赖随机 BioIK timeout |
| `extract_ik_dedup_enabled` | `false` | 是否开启 IK 相似姿态去重；实验常显式开 `true` |
| `extract_ik_dedup_joint_threshold_deg` | `1.0` | 去重关节阈值 |
| `extract_ik_dedup_h_threshold` | `0.005` | 去重 updown 阈值 |
| `extract_use_independent_kdl` | `false` | 是否用独立 Orocos KDL 链绕开 MoveIt `setFromIK()` |
| `extract_independent_kdl_seed_attempts` | `1` | 独立 KDL 每个候选的 seed 尝试次数 |
| `extract_benchmark_candidate_limit` | `0` | 抽离候选限制；实验常用 `64` |
| `extract_benchmark_extract_workers` | `1` | 抽离 worker；实验常用 `16` |
| `extract_benchmark_plan_loaded_after_success` | `false` | 抽离成功后是否继续规划到负重姿态 |
| `extract_loaded_candidate_limit` | `0` | 进入负重规划的候选数限制；实验常用 `10` |
| `extract_loaded_sort_by_pose_distance` | `false` | 是否按负重姿态距离排序 |
| `extract_loaded_stop_on_first_success` | `false` | 负重规划是否首成功即停 |
| `extract_loaded_target_updown` | `0.1` | 抽离后负重规划目标 updown |
| `loaded_left_pose_family_deg` / `loaded_right_pose_family_deg` | `[0,-45,120,-75,0,0]` | 抽离后首先到达的原负重姿态族 |
| `extract_monitor_place_cycle_enabled` | `false` | 是否在负重规划后继续执行预放置、放置、释放和空载返程 |
| `extract_monitor_pre_place_left_pose_deg` / `extract_monitor_pre_place_right_pose_deg` | `[0,-90,120,-75,0,0]` | 负重到放置之间必须经过的预放置姿态 |
| `extract_monitor_place_transition_updown` | `0.1` | 预放置姿态的 updown |
| `extract_monitor_place_left_pose_deg` / `extract_monitor_place_right_pose_deg` | `[0,-55,-50,-60,0,0]` | 最终放置姿态 |
| `extract_monitor_place_updown` | `0.1` | 最终放置姿态的 updown |
| `start_move_group` | `true` | 是否由该 launch 启动 move_group 和支持节点；设为 `false` 时必须外部已有完整 MoveIt 栈，否则节点会等待 MoveGroupInterface 依赖 |

## 6. 复现实验命令

构建：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select robot_motion_scene_service alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

运行原四组抽离 + 负重规划复现：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py \
  execute:=false \
  start_move_group:=true \
  extract_demo_all_rows:=true \
  extract_demo_direct_grasp_start:=true \
  extract_benchmark_all_legal_ik:=true \
  extract_benchmark_dual_arm:=true \
  extract_benchmark_dual_async:=true \
  extract_benchmark_record_rollouts:=false \
  extract_benchmark_candidate_limit:=64 \
  extract_benchmark_extract_workers:=16 \
  extract_ik_dedup_enabled:=true \
  extract_benchmark_plan_loaded_after_success:=true \
  extract_loaded_candidate_limit:=10 \
  extract_loaded_sort_by_pose_distance:=true \
  extract_loaded_stop_on_first_success:=true \
  extract_use_independent_kdl:=true \
  extract_independent_kdl_seed_attempts:=8 \
  extract_independent_kdl_seed_jitter_deg:=15.0 \
  extract_demo_pair_sequence:="2,4;7,9;12,14;17,19" \
  record_jsonl_path:=/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/motion_pipeline_replay/original_pairs.jsonl \
  extract_benchmark_csv_path:=/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/motion_pipeline_replay/original_pairs.csv
```

另开终端触发：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /dual_arm_planner/run_left_extract_demo std_srvs/srv/Trigger {}
```

生成 Rerun：

```bash
/usr/bin/python3 /mnt/mydisk/ALFA/alfa_robot/ros2_ws/src/alfa_robot_moveit_config/scripts/visualize_moveit_box_stack_flow.py \
  /mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/motion_pipeline_replay/original_pairs.jsonl \
  --save /mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/motion_pipeline_replay/original_pairs.rrd \
  --stride 4
```

注意：如果同一机器上有旧 launch 残留，service 可能打到旧节点。复现实验建议先清理旧 ROS 进程，或使用独立 `ROS_DOMAIN_ID`。

## 7. 当前验证证据

本次重构后复现实验数据：

- 小规模 L2/R4：`data/ik_benchmark/refactor_replay_review/refactor_replay_single.jsonl`
- 原四组 JSONL：`data/ik_benchmark/refactor_replay_review/refactor_replay_original_pairs.jsonl`
- 原四组 Rerun：`data/ik_benchmark/refactor_replay_review/refactor_replay_original_pairs.rrd`
- 原四组 CSV：`data/ik_benchmark/refactor_replay_review/refactor_replay_original_pairs_L*_R*.csv`

复现结果：

| Pair | 结果 | 说明 |
| --- | --- | --- |
| L2/R4 | 进入抽离 + 负重规划，成功 | `success_any=true`，`loaded_plan=1/1` |
| L7/R9 | 进入抽离 + 负重规划，成功 | `success_any=true`，`loaded_plan=1/1` |
| L12/R14 | 进入抽离 + 负重规划，成功 | `success_any=true`，`loaded_plan=1/1` |
| L17/R19 | 抓取 IK 阶段失败 | 当前侧吸高度窗判定 `h_interval_unreachable`，符合“最低排后续走顶吸”的现阶段设定 |

另有历史固定原抓取任务复跑结果：

- Rerun：`data/ik_benchmark/motion51_original_pairs_current/original_pairs_current.rrd`
- JSONL：`data/ik_benchmark/motion51_original_pairs_current/original_pairs_current.jsonl`
- CSV：`data/ik_benchmark/motion51_original_pairs_current/original_pairs_current_L*_R*.csv`

## 8. 当前仍需注意

- `DualArmPlannerNode` 仍有约 3600 行，主要剩 ROS 参数、MoveIt 后端、碰撞判定和 callback 装配；后续迁移时不要继续在该节点里堆新算法。
- `ExtractRolloutPlanner` 通过 callback 复用节点内碰撞判定，这是刻意保留的 seam，避免重构时改变 PlanningScene 语义。
- `robot_motion_scene_service` 虽然名字里有 service，但当前不是独立运行节点；它是场景几何与 PlanningScene Adapter 包。后续若要做真正场景服务，应在新仓库里另建 ROS node/action/service 包装层。
- 负重规划依赖 MoveIt；如果 `start_move_group=false`，必须外部已有可用 move_group、robot state publisher 和 controller/joint state 相关支持节点，否则 `DualArmPlannerNode` 可能在 MoveGroupInterface 初始化阶段等待。
- 当前 `alfa_robot_motion_scene_adapter` 名字偏窄，实际已经包含 IK、抽离和负重规划模块；后续迁移到新包时可以重命名为更准确的 motion pipeline/runtime 库。

## 9. 箱体位姿 RRT 抽离实验模式

### 9.1 Module 与调用关系

新抽离策略拆成两层，避免把 MoveIt、解析 IK 和搜索树重新揉回主节点：

| Module | Interface | Implementation / Adapter |
| --- | --- | --- |
| 箱体位姿 RRT 核心 | `BoxPoseExtractState`、`BoxPoseExtractRrtConfig`、边可达性回调、候选路径结果 | `robot_motion_core/include/robot_motion_core/box_pose_extract_rrt.hpp`、`robot_motion_core/src/box_pose_extract_rrt.cpp`；只负责箱体搜索空间、单调约束、RRT、shortcut 和路径排序，不依赖 ROS/MoveIt |
| 运控抽离 Adapter | 输入抓取 IK 状态、左右附着箱和抓取模式，输出 `ExtractRolloutTiming` | `alfa_robot_moveit_config/src/box_pose_rrt_extract_planner.cpp`；把箱体路径离散成末端 Pose，调用三平行解析 IK，再用统一双臂碰撞检查筛选路径组合 |
| 场景几何 Module | 箱墙、集装箱、附着箱、后侧封闭板 | `robot_motion_scene_service/motion_core/scene_geometry`；箱墙开洞后方新增 `_rear_guard`，防止抽离搜索把箱子重新推入货墙 |

这一 Seam 的价值是：RRT 核心可以用纯单测验证；解析 IK 和碰撞是两个 Adapter，后续迁移到标准运控仓库时不需要搬运 `dual_arm_planner_node` 的全部实现。

### 9.2 搜索与碰撞顺序

```text
抓取 IK 候选
  -> 左右臂分别在箱体位姿空间生成多条 RRT 路径
  -> 每条边只做解析 IK 连续可达检查，不做碰撞
  -> shortcut 去除冗余节点
  -> 按累计关节运动量排序
  -> 左右路径交叉组合
  -> 从低代价到高代价逐条做统一全场景碰撞检查
  -> 选择第一条双臂、附着箱、箱墙、集装箱均合法且完成脱离的组合
```

侧吸搜索状态为 `retreat + pitch`，顶吸搜索状态为 `retreat + lift`。侧吸末端运动方向由世界坐标中的“远离货墙”约束确定，不再从附着碰撞盒重排后的局部尺寸轴猜测。

### 9.3 使用方式与稳定默认

当前稳定默认仍是旧贪心策略：

```bash
--extract-rollout-mode greedy
```

显式启用箱体位姿 RRT：

```bash
--extract-rollout-mode box_pose_rrt
```

保留的兼容模式还有 `moveit_rrt_legacy`、`top_lift_legacy`。新策略尚未替换生产默认，因为必须先证明典型任务存在碰撞合法路径，而不能靠忽略真实碰撞获得成功。

### 9.4 2026-07-10 验证结论

- `robot_motion_core`、`robot_motion_scene_service`、`alfa_robot_moveit_config` 构建通过；相关测试累计 30 项零失败。
- 固定实验序列为 `1/3, 1/8, 6/3, 6/8, 6/13, 11/8, 11/13, 11/18, 16/13, 16/18, 16/23, 21/18, 21/23`。左右臂分别按箱号决定吸附模式：左侧 `1/6`、右侧 `3/8` 使用侧吸，其余使用顶吸；测试覆盖 4 组纯侧吸和 9 组混合/纯顶吸任务。
- IK 选择顺序改为“全部合法解代价排序与去重 -> 附着箱完整场景过滤 -> 截取候选上限”，避免低代价前 64 个全部碰撞时错误丢弃后续合法解。当前十三组实验使用场景过滤后前 8 个候选。
- 顶吸最终脱离判定已与 RRT 核心统一：附着箱与原箱位的 X-Z 投影只要在水平或竖直方向留出 `extract_neighbor_margin` 即完成脱离，不再要求整箱完全抬到原箱顶面以上。修复后 `L11/R13`、`L16/R18` 均能完成 IK、双臂箱体位姿 RRT 抽离和负重阶段。
- 侧吸 `L1/R3`、`L1/R8`、`L6/R3`、`L6/R8` 以及混合任务 `L6/R13`、`L11/R8` 的主要失败发生在箱体 RRT 抽离阶段：候选通常在第 4～8 个离散步发生 `joint2 <-> updown` 或其他单臂碰撞。该结果说明严格二维侧吸空间当前仍不足，而不是负重规划或最终选择器失败。
- `L11/R18`、`L16/R13`、`L16/R23`、`L21/R18`、`L21/R23` 在附着箱场景过滤后没有合法抓取 IK 起点，主要是 `joint2 <-> updown`，最低一组还包含携带箱与下方箱墙重叠。搜索器不会绕过非法起点。
- 抽离和负重阶段现在在 `0/N` 成功时立即失败，并回传主导原因；不再继续运行到最终选择后统一误报 `no loaded-plan success to select`。失败快照保留实际 IK/抽离/负重耗时。
- `extract_box_pose_rrt_diagnostics` 默认关闭；仅在定位问题时启用单臂隔离碰撞统计，避免正式实验额外重复碰撞检查。
- 最终十三组 Rerun：`data/ik_benchmark/extract_sequence_rerun/box_pose_rrt_13_pairs_final_20260710.rrd`；统计：`data/ik_benchmark/extract_sequence_rerun/sequence_20260710_234401/stats.csv`。该轮 13 组中 2 组完整成功，成功任务均为纯顶吸；完整运行约 4 分钟，单个有效 RRT 抽离阶段约 20 秒，当前实现仍属于实验验证而非实时生产算法。
- 顶吸已获得真实碰撞合法样本，但侧吸和底部任务尚未达到替换条件，所以稳定默认仍保持 `greedy`。后续应分别优化侧吸自由度/终点设计和附着起点选择，不能静默忽略立柱、附着箱、箱墙或集装箱碰撞。

## 10. 代价函数前原始 IK 解诊断

`extract_sequence_rerun.py --ik-only-raw` 用于观察解析 IK 在进入代价函数前产生的全部合法解。这里的“合法”仅表示通过解析求解及 FK 位置/姿态误差校验；这些解尚未计算代价、尚未排序去重，也尚未经过附着箱完整场景碰撞筛选，因此不能当作可直接执行候选。

数据在 `OptimizedDualIkSolver` 的合法性校验之后、`score_analytic_candidate()` 之前按生成顺序捕获。快照使用 `phase=ik_pre_score_candidates`，并明确记录 `cost_scored=false`、`deduplicated=false`、`scene_filtered=false`；每条记录的 `score=null`。

十三组默认任务可使用：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/alfa_robot_moveit_config/scripts/extract_sequence_rerun.py \
  --ik-only-raw \
  --continue-on-failure \
  --service-timeout 120 \
  --save /mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/extract_sequence_rerun/all_tasks_pre_cost_ik_solutions.rrd
```

Rerun 沿时间轴依次显示任务和生成序号，每个时间点只显示一个完整双臂机器人状态，避免数千个机器人模型叠加后不可读。

## 11. IK 关节限位裕量代价实验

2026-07-11 起，解析 IK 评分默认暂时关闭 updown 移动奖励/惩罚，但保留 `ik_updown_cost_enabled` 参数供后续恢复。新增基于 RobotModel 真实上下限的关节裕量代价：关节位于中心 60% 范围内不惩罚，超过后使用非线性 barrier；左右臂权重均为 `joint1~6=[0.5, 3.0, 0.7, 0.5, 1.5, 1.2]`，joint2 最高，joint5/6 次之。

在与 2026-07-10 基线相同的前 16 个候选抽离口径下，十三组成功率仍为 `2/13`，成功任务仍是 `L11/R13`、`L16/R18`。侧吸主导失败仍为 `joint2 <-> updown`，说明机械角限位裕量和中心柱几何净空不是同一指标；该代价可改善关节边界舒适性，但不能替代后续中心柱净空查表先验。结果：`data/ik_benchmark/extract_sequence_rerun/box_pose_rrt_13_pairs_joint_limit_cost_limit16_20260711.rrd`，统计：`data/ik_benchmark/extract_sequence_rerun/sequence_20260711_050757/stats.csv`。
