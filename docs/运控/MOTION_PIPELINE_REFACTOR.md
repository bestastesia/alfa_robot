# 双臂抓取运控流程封装说明

本文档用于给其它仓库、其它部门 AI 或工程师对接当前双臂抓取流程。当前代码仍在 `alfa_robot_moveit_config` 包内，但已经把原来集中在 `dual_arm_planner_node.cpp` 的长流程拆成一组按职责划分的 C++ 模块，便于后续迁移到独立运控包或 IK 服务包。

## 1. 当前流程总览

当前固定版流程先到“负重位置”为止，暂未封装完整放置流程。

```text
箱子编号 / 视觉目标
  -> 箱垛几何：生成左右末端抓取目标、箱墙开洞、集装箱障碍
  -> 抓取 IK：按侧吸 / 顶吸高度窗生成 h 候选，运行 fixed h × multi seed × cost scorer
  -> IK 候选整理：过滤 legal candidate，按 score 排序，相似姿态去重，截断 TopN
  -> 抽离搜索：对候选 IK 做左右臂抽离 rollout，检查末端箱/机器人/集装箱/箱墙碰撞
  -> 负重姿态选择：对抽离成功结果选择最近的负重姿态族
  -> 负重规划：MoveIt 从抽离末态规划到负重 joint state，附着箱仍参与碰撞
  -> 记录：JSONL / CSV / Rerun 回放
```

核心原则：IK、抽离、负重规划都保留阶段记录和失败原因；重构只移动代码位置，不改变算法语义。代码文件按职责聚合，不再按每个小类单独切文件，避免后续对接时“到处找算法碎片”。

## 2. 代码入口与责任划分

| 模块 | 责任 | 当前文件 |
| --- | --- | --- |
| `motion_core/task_geometry` | 箱子编号、箱垛坐标、抓取 pair、基础碰撞几何数据结构，以及 `updown + 双臂 12 轴` 的标准目标关节顺序 | `include/alfa_robot_moveit_config/motion_core/task_geometry.hpp` / `src/motion_core/task_geometry.cpp` |
| `motion_core/pose_math` | 角度解析、抓取姿态、Pose/Eigen 转换、误差计算、JSON 辅助 | `include/alfa_robot_moveit_config/motion_core/pose_math.hpp` / `src/motion_core/pose_math.cpp` |
| `robot_motion_scene_service/motion_core/scene_geometry` | 集装箱板、动态箱墙、末端附着箱、AABB 与邻箱脱离判断 | `ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_core/scene_geometry.hpp` / `ros2_ws/src/robot_motion_scene_service/src/motion_core/scene_geometry.cpp` |
| `MotionSceneAdapter` | 将场景几何转换为 MoveIt collision/attached objects，并管理 ADD/REMOVE 与当前场景状态；当前是库级 Adapter，不是独立 ROS 节点 | `ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_scene_adapter.hpp` / `ros2_ws/src/robot_motion_scene_service/src/motion_scene_adapter.cpp` |
| `optimized_ik_pipeline` | 抓取 IK 选优整体算法：`OptimizedDualIkSolver` 负责 fixed h × multi seed × cost scorer 求解，`IkCandidateSelector` 负责 legal candidate 排序、相似姿态去重和 TopN 截断 | `include/alfa_robot_moveit_config/optimized_ik_pipeline.hpp` / `src/optimized_ik_pipeline.cpp` |
| `extract_planning_pipeline` | 抽箱子整体算法：抽离动作模板、单步 KDL IK、抽离候选评分、单臂/双臂 rollout、候选调度、CSV/summary 统计都在这里 | `include/alfa_robot_moveit_config/extract_planning_pipeline.hpp` / `src/extract_planning_pipeline.cpp` |
| `loaded_pose_planning` | 负重姿态阶段整体算法：从抽离末态选择最近负重姿态族，并调用 MoveIt 批量规划到负重 joint state | `include/alfa_robot_moveit_config/loaded_pose_planning.hpp` / `src/loaded_pose_planning.cpp` |
| `MotionFlowRecorder` | JSONL 文件、stage 序号、轨迹/summary 记录写入 | `include/alfa_robot_moveit_config/motion_flow_recorder.hpp` / `src/motion_flow_recorder.cpp` |
| `BoxStackFlowOrchestrator` | 传统 box-stack flow 的按轮任务顺序、预抓取/抓取/负重/回预抓取流程编排 | `include/alfa_robot_moveit_config/box_stack_flow_orchestrator.hpp` / `src/box_stack_flow_orchestrator.cpp` |
| `ExtractDemoOrchestrator` | 抽离 demo 的单 pair / 多 pair 遍历、失败传播和 summary 记录 | `include/alfa_robot_moveit_config/extract_demo_orchestrator.hpp` / `src/extract_demo_orchestrator.cpp` |
| `extract_monitor_state` | 交互式 monitor 的阶段状态机、候选缓存、候选任务调度、抽离/负重统计、最终候选选择 | `include/alfa_robot_moveit_config/extract_monitor_state.hpp` / `src/extract_monitor_state.cpp` |
| `extract_monitor_json` | monitor 的候选、阶段、快照、replay extra 字段 schema | `include/alfa_robot_moveit_config/extract_monitor_json.hpp` / `src/extract_monitor_json.cpp` |
| `ExtractMonitorSnapshotWriter` | monitor 快照文件读写，保证目录创建和 JSON 落盘错误集中处理 | `include/alfa_robot_moveit_config/extract_monitor_snapshot_writer.hpp` / `src/extract_monitor_snapshot_writer.cpp` |
| `ExtractMonitorTransitionPlanner` | monitor 最终回放中“负重位 → IK 吸附位”的过渡规划策略：插值、densify、碰撞验证、失败后 RRT、shortcut、再次验证 | `include/alfa_robot_moveit_config/extract_monitor_transition_planning.hpp` / `src/extract_monitor_transition_planning.cpp` |
| `ExtractMonitorReplayBuilder` | monitor 最终采用方案的 Rerun/JSON 回放阶段组装：预吸附过渡、抽离记录、横向让位、负重规划按固定顺序合并 | `include/alfa_robot_moveit_config/extract_monitor_replay_builder.hpp` / `src/extract_monitor_replay_builder.cpp` |
| `DualArmPlannerNode` | ROS 参数、MoveIt 后端、场景碰撞判定、service callback 装配 | `src/dual_arm_planner_node.cpp` |
| 启动配置 | 暴露算法超参数和实验参数 | `launch/dual_arm_planner.launch.py` |
| 回放工具 | 将 JSONL 转为 Rerun 场景 | `scripts/visualize_moveit_box_stack_flow.py` |

## 3. 模块间数据流

### 3.1 箱垛与抓取目标

- `make_boxes(box_front_x)` 生成 5×5 箱垛坐标。
- `parse_box_pair_list()` / `make_pick_pairs()` 生成抓取 pair。
- `make_front_grasp_pose()` / `make_top_suction_pose()` 在 `motion_core/pose_math` 内结合抓取模式生成左右末端 Pose。
- 集装箱和箱墙几何来自 `robot_motion_scene_service/motion_core/scene_geometry`，再由 `MotionSceneAdapter` 注入 MoveIt。

### 3.2 抓取 IK

- `OptimizedDualIkSolver::solve()` 是当前自研 BioIK 选优流程的 Adapter。
- 输入：左右末端 Pose、当前 seed state、抓取模式。
- 内部：构造 `ik_benchmark::UpdownAwareIkRequest`，调用 `ParallelUpdownAwareIkSolver`。
- 输出：selected joint state、完整 IK 审计 JSON、所有候选统计。
- 当前 solver 实现来自 `scripts/ik_benchmark`，但 CMake 已把相关实现和头文件纳入 `alfa_robot_motion_scene_adapter`，便于其它包链接复用。

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
  - `ExtractCandidateSolver` 对候选目标做 KDL 求解。
  - `ExtractCandidateScorer` 在合法候选中选择局部最低代价。
  - 通过 callback 调用 `DualArmPlannerNode` 内的碰撞检查，保留现有 MoveIt PlanningScene 语义。
- 支持异步双臂抽离：左右臂分别求抽离路径，再组合检查全程双臂和环境碰撞。
- 支持成功/失败早停：脱离邻箱后可额外走少量步；当前步所有方向失败则该 IK 候选失败。

### 3.5 抽离后负重规划

- `LoadedPoseSelector` 根据抽离末态，从左右各 3 个负重姿态族中选择最近目标。
- `LoadedPoseSelector` 同时负责把最近负重姿态距离、L2 距离和最大关节差写回 `ExtractRolloutTiming`，供候选排序、CSV 和 monitor 快照复用。
- `LoadedPosePlanner` 在保留末端附着箱的情况下调用 MoveIt 规划到负重姿态。
- `ExtractBenchmarkRunner` 可对抽离成功候选按负重姿态距离排序，按 `extract_loaded_candidate_limit` 截断，并可 `extract_loaded_stop_on_first_success` 首成功即停。

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

## 4. 可复用库与迁移建议

当前 CMake 导出两个层级：

第一层是独立场景包 `robot_motion_scene_service`，负责场景几何和 MoveIt PlanningScene 适配：

```cmake
find_package(robot_motion_scene_service REQUIRED)

target_link_libraries(your_target
  robot_motion_scene_service::robot_motion_scene_core
  robot_motion_scene_service::robot_motion_scene_adapter
)
```

第二层是 `alfa_robot_moveit_config` 内的运控流程库：

```cmake
find_package(alfa_robot_moveit_config REQUIRED)

target_link_libraries(your_target
  alfa_robot_motion_core
)
```

`alfa_robot_motion_core` 只包含姿态/箱垛等轻量模块，并链接 `robot_motion_scene_service::robot_motion_scene_core` 复用场景几何。

```cmake
target_link_libraries(your_target
  alfa_robot_motion_scene_adapter
)
```

`alfa_robot_motion_scene_adapter` 包含 IK 选优、抽离规划、负重规划、benchmark runner 和 monitor 辅助模块。它依赖 MoveIt、KDL、`robot_motion_scene_service` 和 benchmark IK solver。

迁移到新运控包时建议顺序：

1. 先迁移 `robot_motion_scene_service`，让集装箱、箱墙、末端附着箱的碰撞口径先稳定下来。
2. 再迁移 `motion_core/*` 和 `optimized_ik_pipeline`，作为独立 IK 服务的核心算法。
3. 再迁移 `extract_planning_pipeline`，作为抽箱子动作生成、单步 IK、评分和 rollout 的完整模块。
4. 最后迁移 `loaded_pose_planning`，接入新 MoveIt/PlanningScene 后端。
5. `DualArmPlannerNode` 不建议整文件复制；它只应作为 ROS 参数、MoveIt 后端和 service/action 包装参考。

## 5. 关键启动参数

| 参数 | 当前默认 | 含义 |
| --- | --- | --- |
| `extract_demo_pair_sequence` | `2,4;7,9;12,14;17,19` | 当前固定版原抓取任务 |
| `front_z_reach_lower` / `front_z_reach_upper` | `0.45` / `1.25` | 侧吸目标高度窗：`updown + 0.45 ~ updown + 1.25` |
| `top_z_reach_lower` / `top_z_reach_upper` | `0.3` / `0.45` | 顶吸目标高度窗 |
| `ik_h_candidate_count` / `ik_seed_count` | `16` / `32` | 抓取 IK 主候选池大小 |
| `ik_candidate_timeout` | `0.01` | 单次 BioIK timeout |
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
| `extract_loaded_target_updown` | `0.3` | 抽离后负重规划目标 updown |
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
