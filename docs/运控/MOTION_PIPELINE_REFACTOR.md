# 双臂抓取运控流程封装说明

本文档用于给其它仓库、其它部门 AI 或工程师对接当前双臂抓取流程。当前代码仍在 `alfa_robot_moveit_config` 包内，但已经把原来集中在 `dual_arm_planner_node.cpp` 的长流程拆成一组普通 C++ 模块，便于后续迁移到独立运控包或 IK 服务包。

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

核心原则：IK、抽离、负重规划都保留阶段记录和失败原因；重构只移动代码位置，不改变算法语义。

## 2. 代码入口与责任划分

| 模块 | 责任 | 当前文件 |
| --- | --- | --- |
| `motion_core/task_geometry` | 箱子编号、箱垛坐标、抓取 pair、基础碰撞几何数据结构 | `include/alfa_robot_moveit_config/motion_core/task_geometry.hpp` / `src/motion_core/task_geometry.cpp` |
| `motion_core/pose_math` | 角度解析、抓取姿态、Pose/Eigen 转换、误差计算、JSON 辅助 | `include/alfa_robot_moveit_config/motion_core/pose_math.hpp` / `src/motion_core/pose_math.cpp` |
| `motion_core/scene_geometry` | 集装箱板、动态箱墙、末端附着箱、AABB 与邻箱脱离判断 | `include/alfa_robot_moveit_config/motion_core/scene_geometry.hpp` / `src/motion_core/scene_geometry.cpp` |
| `MotionSceneAdapter` | 将场景几何转换为 MoveIt collision/attached objects，并管理 ADD/REMOVE 与当前场景状态 | `include/alfa_robot_moveit_config/motion_scene_adapter.hpp` / `src/motion_scene_adapter.cpp` |
| `OptimizedDualIkSolver` | 将 MoveIt `RobotState`/左右末端 Pose 翻译成 fixed h × multi seed × cost scorer 请求，写回 selected joint state，输出 IK 审计 JSON | `include/alfa_robot_moveit_config/optimized_dual_ik_solver.hpp` / `src/optimized_dual_ik_solver.cpp` |
| `IkCandidateSelector` | 对 legal IK 候选按 score 排序后的相似姿态去重和候选数量截断 | `include/alfa_robot_moveit_config/ik_candidate_selector.hpp` / `src/ik_candidate_selector.cpp` |
| `ExtractMotionPlanner` | 抽离动作模板、pitch 调整层、retreat/lift 候选目标盒心生成 | `include/alfa_robot_moveit_config/extract_motion_planner.hpp` / `src/extract_motion_planner.cpp` |
| `ExtractCandidateSolver` | 抽离阶段单臂 KDL / 独立 KDL 候选求解、tip 误差和基础姿态硬约束 | `include/alfa_robot_moveit_config/extract_candidate_solver.hpp` / `src/extract_candidate_solver.cpp` |
| `ExtractCandidateScorer` | 抽离候选的 lift/pitch/连续性/关节变化/tip 变化代价打分 | `include/alfa_robot_moveit_config/extract_candidate_scorer.hpp` / `src/extract_candidate_scorer.cpp` |
| `ExtractRolloutPlanner` | 单臂/双臂抽离 rollout 状态机、成功/失败早停、异步组合、逐步记录字段生成 | `include/alfa_robot_moveit_config/extract_rollout_planner.hpp` / `src/extract_rollout_planner.cpp` |
| `LoadedPoseSelector` | 从抽离后的关节状态选择最近的负重姿态族，并生成负重目标 joint state | `include/alfa_robot_moveit_config/loaded_pose_selector.hpp` / `src/loaded_pose_selector.cpp` |
| `LoadedPosePlanner` | 抽离后到负重姿态的 MoveIt 规划、批量排序/limit/首成功即停、临时附着箱状态和负重规划记录 | `include/alfa_robot_moveit_config/loaded_pose_planner.hpp` / `src/loaded_pose_planner.cpp` |
| `ExtractBenchmarkRunner` | IK 合法候选筛选/去重、抽离 rollout 调度、抽离 worker、负重规划批处理、CSV/summary 写入 | `include/alfa_robot_moveit_config/extract_benchmark_runner.hpp` / `src/extract_benchmark_runner.cpp` |
| `ExtractBenchmarkCsvWriter` | 抽离 benchmark CSV 表头和逐候选 timing 输出 | `include/alfa_robot_moveit_config/extract_benchmark_csv_writer.hpp` / `src/extract_benchmark_csv_writer.cpp` |
| `ExtractBenchmarkSummary` | 抽离/负重 timing 聚合、均值和首个负重成功候选统计 | `include/alfa_robot_moveit_config/extract_benchmark_summary.hpp` / `src/extract_benchmark_summary.cpp` |
| `MotionFlowRecorder` | JSONL 文件、stage 序号、轨迹/summary 记录写入 | `include/alfa_robot_moveit_config/motion_flow_recorder.hpp` / `src/motion_flow_recorder.cpp` |
| `BoxStackFlowOrchestrator` | 传统 box-stack flow 的按轮任务顺序、预抓取/抓取/负重/回预抓取流程编排 | `include/alfa_robot_moveit_config/box_stack_flow_orchestrator.hpp` / `src/box_stack_flow_orchestrator.cpp` |
| `ExtractDemoOrchestrator` | 抽离 demo 的单 pair / 多 pair 遍历、失败传播和 summary 记录 | `include/alfa_robot_moveit_config/extract_demo_orchestrator.hpp` / `src/extract_demo_orchestrator.cpp` |
| `extract_planner_types` | 抽离候选、双臂抽离候选、单臂抽离路径、抽离耗时结果等共享数据结构 | `include/alfa_robot_moveit_config/extract_planner_types.hpp` |
| `DualArmPlannerNode` | ROS 参数、MoveIt 后端、场景碰撞判定、service callback 装配 | `src/dual_arm_planner_node.cpp` |
| 启动配置 | 暴露算法超参数和实验参数 | `launch/dual_arm_planner.launch.py` |
| 回放工具 | 将 JSONL 转为 Rerun 场景 | `scripts/visualize_moveit_box_stack_flow.py` |

## 3. 模块间数据流

### 3.1 箱垛与抓取目标

- `make_boxes(box_front_x)` 生成 5×5 箱垛坐标。
- `parse_box_pair_list()` / `make_pick_pairs()` 生成抓取 pair。
- `makeFrontGraspPose()` / `makeTopGraspPose()` 在 `DualArmPlannerNode` 内结合抓取模式生成左右末端 Pose。
- 集装箱和箱墙几何来自 `motion_core/scene_geometry`，再由 `MotionSceneAdapter` 注入 MoveIt。

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
- `LoadedPosePlanner` 在保留末端附着箱的情况下调用 MoveIt 规划到负重姿态。
- `ExtractBenchmarkRunner` 可对抽离成功候选按负重姿态距离排序，按 `extract_loaded_candidate_limit` 截断，并可 `extract_loaded_stop_on_first_success` 首成功即停。

### 3.6 记录与回放

- `MotionFlowRecorder` 写 JSONL header/stage/summary。
- `ExtractBenchmarkCsvWriter` 写逐候选 timing CSV。
- `visualize_moveit_box_stack_flow.py` 将 JSONL 转成 Rerun，系统 Python `/usr/bin/python3` 下可用。

## 4. 可复用库与迁移建议

当前 CMake 导出两个库：

```cmake
find_package(alfa_robot_moveit_config REQUIRED)

target_link_libraries(your_target
  alfa_robot_motion_core
)
```

`alfa_robot_motion_core` 只包含纯几何/姿态/箱垛模块，适合被非 MoveIt 算法复用。

```cmake
target_link_libraries(your_target
  alfa_robot_motion_scene_adapter
)
```

`alfa_robot_motion_scene_adapter` 包含 MoveIt 场景适配、IK Adapter、抽离 rollout、负重规划、benchmark runner 等模块。它依赖 MoveIt、KDL 和 benchmark IK solver。

迁移到新运控包时建议顺序：

1. 先迁移 `motion_core/*`，保持纯数据和几何不变。
2. 再迁移 `OptimizedDualIkSolver` / `IkCandidateSelector`，作为独立 IK 服务的核心算法 seam。
3. 再迁移 `ExtractMotionPlanner`、`ExtractCandidateSolver`、`ExtractCandidateScorer`、`ExtractRolloutPlanner`，作为抽离规划模块。
4. 最后迁移 `LoadedPoseSelector` / `LoadedPosePlanner` 和 `MotionSceneAdapter`，接入新 MoveIt/PlanningScene 后端。
5. `DualArmPlannerNode` 不建议整文件复制；它只应作为 ROS 参数和 service/action 包装参考。

## 5. 关键启动参数

| 参数 | 当前默认 | 含义 |
| --- | --- | --- |
| `extract_demo_pair_sequence` | `2,4;7,9;12,14;17,19` | 当前固定版原抓取任务 |
| `front_z_reach_lower` / `front_z_reach_upper` | `0.9` / `1.3` | 侧吸目标高度窗：`updown + 0.9 ~ updown + 1.3` |
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
colcon build --packages-select alfa_robot_moveit_config --symlink-install --cmake-args -DBUILD_TESTING=OFF
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

- `DualArmPlannerNode` 仍有约 2300 行，主要剩 ROS 参数、MoveIt 后端、碰撞判定和 callback 装配；后续迁移时不要继续在该节点里堆新算法。
- `ExtractRolloutPlanner` 通过 callback 复用节点内碰撞判定，这是刻意保留的 seam，避免重构时改变 PlanningScene 语义。
- 负重规划依赖 MoveIt；如果 `start_move_group=false`，必须外部已有可用 move_group、robot state publisher 和 controller/joint state 相关支持节点，否则 `DualArmPlannerNode` 可能在 MoveGroupInterface 初始化阶段等待。
- 当前 `alfa_robot_motion_scene_adapter` 名字偏窄，实际已经包含 IK、抽离和负重规划模块；后续迁移到新包时可以重命名为更准确的 motion pipeline/runtime 库。
