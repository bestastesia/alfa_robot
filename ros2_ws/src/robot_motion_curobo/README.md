# robot_motion_curobo

`robot_motion_curobo` 是可选的常驻 cuRobo 关节空间局部规划服务。它接收显式的
13 轴起点、终点、场景标识和附着箱，通过
`robot_motion_interfaces/srv/PlanJointSegment` 执行关节空间规划；该服务不做 IK。当前
backend 会显式生成 graph seed，再调用 `trajopt_solver.solve_cspace()`，最后读取
`get_interpolated_plan()`，不是 `plan_pose()` 或旧说明中的 `plan_single_js()`。

主流程仍保留原有 local-RRT。`extract_loaded_planning_mode=shortcut` 时，直线插值若被
MoveIt/PlanningScene/FCL 判碰撞，可以通过 `extract_local_repair_backend` 选择：

- `rrt`：原有自研 local-RRT，失败后再尝试 MoveIt direct pipeline；
- `curobo`：优先调用本包，失败、超时、响应无效或 FCL 复核失败时可退回上述 local-RRT。

loaded 阶段选择 `curobo` 时，默认 `extract_local_curobo_coupled_13d=true`：从真实抽离
终点到原 loaded 目标只发起一次联合请求，`updown + left_joint1..6 + right_joint1..6`
同时参与 shortcut 和局部 cuRobo 修补。关闭该参数才恢复旧的“左右臂分别规划、随后合并、
最后单独移动 updown”模式。自研 local-RRT 仍保持单臂活动变量约束，不受此参数影响。

局部窗口的起止点只是修补边界，不是新的任务目标。拼接完成后会用完整 FCL 再检查整条
轨迹，并严格检查末点等于调用者原始目标；`final_goal_max_error_rad` 会进入 snapshot 和
Rerun。cuRobo 不会改写用户设置的最终关节角。

## 远程仓库整理：文件清单与职责

local-cuRobo 接入横跨3个 ROS 包，不能只复制 `robot_motion_curobo/`。远程仓库整理时应
按下面的依赖分层合并；尤其不要用本地整份 `dual_arm_planner_node.cpp` 覆盖远程同名文件，
应按职责合并对应代码块。除另有说明外，下面的代码路径均相对 `ros2_ws/src/`。

### 必须保留的运行代码

| 范围 | 文件 | 职责与调整注意点 |
| --- | --- | --- |
| ROS 接口 | `robot_motion_interfaces/srv/PlanJointSegment.srv` | 显式13轴起终点、双 payload、规划选项、轨迹和分阶段计时的服务契约。字段改动后必须干净重编译 interface 和所有依赖包。 |
| ROS 接口注册 | `robot_motion_interfaces/CMakeLists.txt` | 注册 `PlanJointSegment.srv`。接口复用 `msg/MotionContext.msg`、`msg/AttachedBox.msg`，两者也必须在目标仓库存在。 |
| Python 包骨架 | `robot_motion_curobo/package.xml`、`setup.py`、`setup.cfg`、`resource/robot_motion_curobo`、`robot_motion_curobo/__init__.py` | ament Python 包、运行依赖、配置安装和 console entry。新增或删除工具时同步维护 `setup.py`。 |
| 请求契约 | `robot_motion_curobo/planner_core.py` | 13轴按名称重排，校验 NaN/Inf、重复或缺失关节、限位、端点、时间单调性和双 payload；用锁串行化 planner 请求。CPU/CI 不导入 CUDA。 |
| cuRobo backend | `robot_motion_curobo/curobo_backend.py` | cuRobo/CUDA 导入、robot/scene 加载、一次 warmup、graph seed、cspace TrajOpt、插值轨迹、GPU 同步计时、动态 payload 球和世界中被抓箱体禁用。此文件与 cuRobo 版本耦合最强。 |
| ROS 服务节点 | `robot_motion_curobo/joint_segment_planner_node.py` | `PlanJointSegment` 服务、参数、消息转换、ready 生命周期和只读碰撞审计服务。不得静默使用 live `/joint_states`。 |
| 服务启动 | `robot_motion_curobo/launch/joint_segment_planner.launch.py` | 传入 cuRobo checkout、Python 环境、机器人 YAML、场景 YAML、scene id、payload link 和 warmup/规划参数；不能写死本机路径。 |
| C++ 服务适配 | `alfa_robot_moveit_config/include/alfa_robot_moveit_config/curobo_segment_client.hpp`、`src/curobo_segment_client.cpp` | MoveIt `RobotState`/双箱与 ROS 请求互转，异步等待、超时、响应 joint names/端点/数值/时间/限位校验，再转换成 MoveIt plan。 |
| 通用局部修补 | `alfa_robot_moveit_config/include/alfa_robot_moveit_config/extract_monitor_transition_planning.hpp`、`src/extract_monitor_transition_planning.cpp` | shortcut 判碰、局部窗口、边界回退/扩展、preferred cuRobo、local-RRT fallback、拼接、去重、retime、局部与最终验证以及分阶段统计。不要重新做一套 cuRobo 专用拼接器。 |
| Loaded 接入 | `alfa_robot_moveit_config/include/alfa_robot_moveit_config/loaded_pose_planning.hpp`、`src/loaded_pose_planning.cpp` | 在负重 shortcut 碰撞后选择 local backend；默认 cuRobo 模式为 `updown + 双臂` 13轴耦合，保持原生产目标和 prefix/patch/suffix。 |
| 节点集成/FCL | `alfa_robot_moveit_config/src/dual_arm_planner_node.cpp` | 声明参数、创建 client、注入 callback、传递当前候选的双 payload、提供只规划诊断服务、记录 diagnostics，并用显式请求起点和 PlanningScene snapshot 做最终整轨迹 FCL。远程冲突应逐块合并。 |
| 启动参数 | `alfa_robot_moveit_config/launch/dual_arm_planner.launch.py` | 暴露 enabled/backend/coupled/service/scene/timeout/window/call limits/fallback 参数。合入主线时 `extract_local_curobo_enabled=false` 必须保持默认。 |
| C++ 构建 | `alfa_robot_moveit_config/CMakeLists.txt`、`package.xml` | 编译 adapter、链接 `robot_motion_interfaces` 并注册测试；不要删除原 OMPL/local-RRT 依赖。 |

### 必须同步核对的共享权威源

这些文件不属于 cuRobo 包，但 local-cuRobo 依赖其语义。远程仓库已有同类实现时应以远程
主线为基础合并，不能复制出第二份常量或碰撞公式：

- `robot_motion_scene_service/include/robot_motion_scene_service/motion_core/task_geometry.hpp`
  和 `src/motion_core/task_geometry.cpp`：C++ 端13轴权威顺序；必须与
  `robot_motion_curobo/planner_core.py::AUTHORITY_JOINT_NAMES` 完全一致。
- `robot_motion_scene_service/include/robot_motion_scene_service/motion_core/scene_geometry.hpp`
  和 `src/motion_core/scene_geometry.cpp`：携箱世界 AABB、动态箱墙、rear guard 和容器门禁
  的生产几何口径。cuRobo 候选不能绕过这些最终检查。
- MoveIt robot/SRDF/joint limits 与外部 cuRobo robot YAML：关节名、顺序、上下限、
  `updown` 单位（米）和12个旋转轴单位（弧度）必须一致。

### 测试文件

核心回归应随代码一起迁移：

- `alfa_robot_moveit_config/test/test_curobo_segment_client.cpp`：mock ROS 服务、响应校验、
  cuRobo 成功、边界组合重试、服务缺失/超时和 local-RRT fallback。
- `alfa_robot_moveit_config/test/test_extract_monitor_transition_planning.cpp`：原 shortcut、
  direct pipeline 和通用 transition planner 基线。
- `robot_motion_curobo/test/test_planner_core.py`：无需 GPU 的请求/轨迹校验、初始化次数和
  并发串行测试。
- `robot_motion_curobo/test/test_collision_model_visualizer.py`、
  `test_payload_sphere_fit_tools.py`、`test_snapshot_collision_sphere_visualizer.py`、
  `test_rejected_trajectory_dual_scene_visualizer.py`：可视化和 payload 球工具回归；若远程
  仓库不保留诊断工具，可以连同对应工具一起移除，不能只删测试而留下失效入口。

### 可选的配置、基准和 Rerun 工具

以下文件不参与生产规划主链，可以放在单独的 tools/validation 提交中：

- `robot_motion_curobo/config/scene_l1_r3_current.yml`、`scene_l1_r8_current.yml`、
  `scene_l6_r3_current.yml`、`scene_l6_r8_current.yml`、`scene_l11_r8_current.yml`：当前任务
  的历史验证静态 cuRobo cuboid 场景快照；不是通用动态 scene 服务。使用前必须让
  MoveIt launch 的容器尺寸、位姿、箱体布局和 rear guard 与快照逐项一致，不能仅靠相同
  `scene_id` 假定几何一致。
- `robot_motion_curobo/config/cspace_tuning_tasks.yml`、`cspace_tuning_benchmark.py`、
  `segment_benchmark_client.py`：GPU 调参与真实 ROS 服务基准。
- `collision_model_visualizer.py`、`payload_sphere_fit_generator.py`、
  `payload_sphere_fit_visualizer.py`、`snapshot_collision_sphere_visualizer.py`、
  `rejected_trajectory_dual_scene_visualizer.py`：碰撞球、payload 拟合和双场景审计。
- `alfa_robot_moveit_config/scripts/extract_sequence_rerun.py`：启动完整抓取、可选常驻服务并
  保存 snapshot/RRD。
- `alfa_robot_moveit_config/scripts/extract_stage_monitor_console.py`：输出 loaded 候选和
  local repair 结构化 diagnostics。
- `alfa_robot_moveit_config/scripts/compose_local_curobo_full_replay.py`、
  `visualize_moveit_box_stack_flow.py`：把真实阶段、未修补 shortcut 和修补结果整理为
  Rerun；只用于离线验证，不是规划实现。
- `docs/运控/LOCAL_CUROBO_VALIDATION_20260724.md`：当前 GPU/FCL 验证记录和限制。

`data/ik_benchmark/local_curobo_v5_dev/` 下的 CSV、JSON、JSONL 和 RRD 均为生成证据，
不属于编译或运行依赖。远程代码仓库可以只保留小型 summary/README，把大型 RRD 放到
制品存储。`scripts/curobo_web_demo/` 是合成可视化 demo，不在这条真实规划链中。

### 外部文件与当前版本敏感点

- 当前 ALFA cuRobo robot YAML、mesh、cuRobo checkout 和可导入 rclpy/cuRobo 的 Python
  环境没有硬编码进本包，远程部署者必须自行提供路径。
- robot YAML 的 `left_payload_link`、`right_payload_link` 必须预分配 payload 球槽；默认
  0.1 m 网格下，0.4 × 0.4 × 0.3 m 箱体每侧需要48槽。
- `curobo_backend.py` 当前使用 `_get_graph_seed_trajectories()`、
  `trajopt_solver.config.num_seeds` 和 `trajopt_solver.solve_cspace()` 等版本敏感接口。升级
  cuRobo 时优先在这里做兼容层，并重新跑真 GPU smoke，不能只凭 Python import 成功。
- 节点启动时只加载一个静态 scene；请求 `scene_id` 与 `expected_scene_id` 不一致会失败并
  触发 fallback。远程若要动态任务切换，应新增明确的 scene 更新机制，不能忽略 id。
- `planner_core.py` 当前要求请求 payload link 为 `left_tool0/right_tool0`；如果远程模型改
  link 名，需要同时修改契约、backend 参数、robot YAML 和测试。

### 推荐合并顺序

1. 先合并 `PlanJointSegment.srv` 和 interface CMake，清理旧 build/install/log 后单独构建
   `robot_motion_interfaces`。
2. 合并完整 `robot_motion_curobo` 运行核心，先跑 `test_planner_core.py`；此阶段不要求 GPU。
3. 合并 C++ adapter、transition planner、LoadedPosePlanner 和 launch/CMake 变更，保持
   cuRobo 默认关闭，确认原 shortcut/local-RRT/OMPL 测试不回归。
4. 准备并审计外部 robot YAML、双 payload 球槽和目标 scene，启动服务并确认 warmup
   只发生一次。
5. 用 mock 服务验证 shortcut 无碰撞时调用数为0、碰撞时能拼接、服务缺失/超时能有界
   fallback；再运行真实 GPU 局部段 smoke。
6. 最后用生产 PlanningScene/FCL 复核所有返回执行的完整轨迹，再按需合入 benchmark、
   Rerun 和大型验证数据。

整理完成后至少执行：

```bash
colcon build --packages-select \
  robot_motion_interfaces robot_motion_curobo alfa_robot_moveit_config \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select robot_motion_curobo alfa_robot_moveit_config \
  --return-code-on-test-failure
colcon test-result --verbose
```

## 模型约束

机器人 YAML 必须使用当前关节名和固定顺序：

```text
updown,left_joint1..6,right_joint1..6
```

已验证的旧 cuRobo robot YAML 可以继续使用 `leftjoint1..6/rightjoint1..6`；backend 只在
cuRobo 模型边界做名称别名转换，ROS 服务请求和响应始终使用上面的当前权威名称。新模型
不要再使用旧名称。

左右 payload link 必须预留足够的 `extra_collision_spheres`。服务会按每个请求中的
`AttachedBox.center_in_link` 和 `size` 重写预留槽位，请勿选择还承载不可替换机器人本体
球的 link。默认 0.1 m 网格对 0.4 × 0.4 × 0.3 m 箱体需要每侧 48 个槽位。

球覆盖比 MoveIt/FCL 的精确箱体更保守，因此 FCL 可行的局部边界可能被 cuRobo 判碰。
主流程会在有界预算内向前回退局部起点、向后扩展局部终点；仍失败则退回 local-RRT。
不得通过删除墙体或跳过最终 FCL 来规避模型差异。

## 构建

```bash
cd /path/to/alfa_robot-v5_dev/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_motion_curobo alfa_robot_moveit_config
source install/setup.bash
```

本包没有机器路径默认值。运行时必须显式给出 cuRobo checkout、机器人 YAML 和场景
YAML：

```bash
ros2 launch robot_motion_curobo joint_segment_planner.launch.py \
  robot_config_path:="$CUROBO_ROBOT_YAML" \
  scene_config_path:="$CUROBO_SCENE_YAML" \
  expected_scene_id:=L1_R3_current_v1 \
  curobo_python_root:="$CUROBO_ROOT" \
  dependency_venv:="$CUROBO_VENV" \
  left_payload_link:=left_tool0 \
  right_payload_link:=right_tool0
```

服务仅在 CUDA/curobo 导入、模型/场景校验、payload 槽校验和 warmup 全部成功后才
可用：

- `/robot_motion/plan_joint_segment`
- `/robot_motion/check_curobo_collision`（只读模型审计，不替代最终 MoveIt/FCL）

启用后，`dual_arm_planner` 还提供只规划不执行的
`/dual_arm_planner/plan_local_segment_repair`。它使用与 loaded 主流程相同的
shortcut 判碰、局部窗口、cuRobo、local-RRT fallback、拼接和最终全场景 FCL，可用于
确定性复现真实碰撞段；不是执行接口。

## 在抓取流程中选择后端

cuRobo 优先、local-RRT 回退：

```bash
ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py \
  extract_loaded_planning_mode:=shortcut \
  extract_local_repair_backend:=curobo \
  extract_local_curobo_enabled:=true \
  extract_local_curobo_coupled_13d:=true \
  extract_local_curobo_service:=/robot_motion/plan_joint_segment \
  extract_local_curobo_scene_id:=L1_R3_current_v1 \
  extract_local_curobo_timeout_s:=2.0 \
  extract_local_curobo_window_points:=8 \
  extract_local_curobo_boundary_backoff_points:=5 \
  extract_local_curobo_max_segments:=4 \
  extract_local_curobo_max_calls_per_plan:=8 \
  extract_local_curobo_fallback_to_rrt:=true
```

只使用原有 local-RRT：

```bash
ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py \
  extract_loaded_planning_mode:=shortcut \
  extract_local_repair_backend:=rrt \
  extract_local_curobo_enabled:=false
```

`extract_local_curobo_enabled` 默认关闭，不改变最新仓库的原有行为。诊断复现可以设置
`extract_loaded_candidate_order_filter`，默认 `-1` 表示仍使用正常自动候选选择。

## 完整抓取与 Rerun

`extract_sequence_rerun.py` 可以同时启动常驻服务、运行抓取并保存 `.rrd`：

```bash
python3 src/alfa_robot_moveit_config/scripts/extract_sequence_rerun.py \
  --pair-sequence 1,3 \
  --save /tmp/L1_R3_local_curobo.rrd \
  --start-local-curobo-service \
  --local-curobo-robot-config "$CUROBO_ROBOT_YAML" \
  --local-curobo-scene-config \
    src/robot_motion_curobo/config/scene_l1_r3_current.yml \
  --local-curobo-scene-id L1_R3_current_v1 \
  --local-curobo-python-root "$CUROBO_ROOT" \
  --local-curobo-dependency-venv "$CUROBO_VENV" \
  --local-curobo-timeout 2.0 \
  --local-curobo-fallback-to-rrt
```

Rerun 时间线和 info panel 会记录：

- 全流程 IK、抽离、负重规划和最终回放耗时；
- local service roundtrip、queue、endpoint check、graph、TrajOpt、interpolation、响应转换、
  stitching、局部 FCL 和最终全轨迹 FCL；
- preferred/fallback 调用数、局部窗口、planner method、最终目标最大误差；
- 最终选择和每个已尝试 loaded 候选的 `loaded_attempt_diagnostics`。

若自然抓取的 shortcut 本身无碰撞，可以用
`compose_local_curobo_full_replay.py` 把真实完整抓取、FCL 安全桥接和生产修补服务返回的
真实碰撞段合成为连续诊断 Rerun。工具会拒绝阶段边界不连续、修补方法不是
`shortcut_local_curobo` 或末点不等于请求目标的输入，并在 summary 中明确标注诊断
碰撞目标不是自然自动选择的 loaded goal。

已有 `stage_snapshot.json` 可以离线叠加与规划器一致的机器人/负载碰撞球，并额外重建
一段“没有 local_curobo 的 loaded 直线”用于对照：

```bash
ros2 run robot_motion_curobo snapshot_collision_sphere_visualizer \
  --snapshot "$STAGE_SNAPSHOT" \
  --robot-config "$CUROBO_ROBOT_YAML" \
  --scene-config "$CUROBO_SCENE_YAML" \
  --save /tmp/collision_spheres.rrd
```

蓝色为机器人碰撞球，青色/紫色为左右负载球，红色表示球与 cuRobo cuboid 场景相交。
摘要会分别统计修补后全流程与未修补 loaded 直线的红帧和碰撞对。红色只代表
sphere-vs-cuboid；工具不会在离线阶段重新执行 cuRobo 自碰撞矩阵。

如需核对“cuRobo 球模型通过、生产 rear_guard gate 拒绝”的同一条原始候选，可生成
共享时间轴的上下双场景 Rerun：

```bash
ros2 run robot_motion_curobo rejected_trajectory_dual_scene_visualizer \
  --jsonl "$CUROBO_CANDIDATES_JSONL" --stage-index 0 \
  --robot-config "$CUROBO_ROBOT_YAML" \
  --curobo-scene "$CUROBO_SCENE_YAML" \
  --production-scene "$PRODUCTION_SCENE_YAML" \
  --save /tmp/rejected_candidate_dual_scene.rrd
```

上方 `+Y` 视图显示 cuRobo 的机器人/负载球及补偿场景；下方 `-Y` 视图显示真实尺寸
携箱、携箱的透明世界 AABB 与未补偿生产场景。生产侧红色专门重放
`carried_box_clear_rear_guards` 拒绝门；其他生产 cuboid 只显示，不参与该红色标记。

## 独立局部段基准与质量指标

`cspace_tuning_benchmark` 直接加载持久 planner，适合 GPU 参数扫描；
`segment_benchmark_client` 通过真实 ROS 服务重复同一输入。两者都输出分阶段耗时。调优
基准还输出端点/轨迹可行性、相对参考路径长度、最大关节步长、平滑度代价和轨迹时长。

```bash
ros2 run robot_motion_curobo cspace_tuning_benchmark \
  --tasks src/robot_motion_curobo/config/cspace_tuning_tasks.yml \
  --robot-config "$CUROBO_ROBOT_YAML" \
  --curobo-python-root "$CUROBO_ROOT" \
  --dependency-venv "$CUROBO_VENV" \
  --select collision_L1_R3 \
  --repeat 20 \
  --label direct_first \
  --output-dir /tmp/curobo_benchmark \
  --num-trajopt-seeds 1 \
  --trajopt-num-iters 26 \
  --trajopt-inner-iters 10 \
  --trajopt-n-knots 12 \
  --trajopt-finetune-attempts 0
```

服务基准可通过 `--record-jsonl` 保存每条成功轨迹，再交给现有 Rerun 可视化器：

```bash
ros2 run robot_motion_curobo segment_benchmark_client \
  --service /robot_motion/plan_joint_segment \
  --label collision_L1_R3 \
  --scene-id L1_R3_current_v1 \
  --start "$START_13" --goal "$GOAL_13" \
  --left-box-id 1 --right-box-id 3 \
  --repeat 20 --no-force-graph \
  --scene-config src/robot_motion_curobo/config/scene_l1_r3_current.yml \
  --record-jsonl /tmp/collision_L1_R3.jsonl \
  --output-dir /tmp/collision_L1_R3

python3 src/alfa_robot_moveit_config/scripts/visualize_moveit_box_stack_flow.py \
  /tmp/collision_L1_R3.jsonl --save /tmp/collision_L1_R3.rrd
```

## 测试

CPU/CI 测试不导入 cuRobo，也不需要 GPU：

```bash
colcon test --packages-select \
  robot_motion_interfaces robot_motion_curobo alfa_robot_moveit_config \
  --return-code-on-test-failure
```

其中 C++ 集成测试覆盖响应 joint order/端点/时间戳校验、shortcut 碰撞后 cuRobo 修补、
起点回退与目标扩展组合重试、服务缺失/超时后 local-RRT 回退，以及最终 FCL gate。

本机真实 GPU 验证数据、限制和可复现命令见
[`docs/运控/LOCAL_CUROBO_VALIDATION_20260724.md`](../../../docs/运控/LOCAL_CUROBO_VALIDATION_20260724.md)。
