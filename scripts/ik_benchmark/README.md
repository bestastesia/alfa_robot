# ALFA IK Benchmark / Range Grid

这个包用于离线、进程内调用 MoveIt kinematics plugin，避免 `/compute_ik` ROS service 逐点调用的通信开销。

它是实验包。公共 IK 候选类型位于 `robot_motion_core`，URDF/FK 与 Rerun 公共实现位于 `alfa_robot_rerun`。构建外置 benchmark 包时使用：

```bash
cd ros2_ws
colcon build --base-paths src ../scripts/ik_benchmark \
  --packages-select robot_motion_core alfa_robot_rerun alfa_robot_benchmarks
source install/setup.bash
```

## Pick-Place Baseline

核心可执行：

```bash
ros2 run alfa_robot_benchmarks pick_place_baseline --help
```

这个 baseline 按 `alfa_robot_moveit_config/scripts/pick_place_demo.py` 里当前启用的 `PICK_POINTS` 顺序做确定性 IK 测试。每一轮只覆盖抓取前半段：

```text
安全位 → 接近位 → 抓取位 → 后退位
```

每一步都使用上一步成功求解得到的关节角作为下一步 seed，因此它模拟的是连续执行过程，而不是每个点都从 home 重新开始。

默认配置：

- group: `dual_arm_with_base`
- solver: `bio_ik/BioIKKinematicsPlugin`
- tips: `left_tool0` / `right_tool0`
- tool0 offset compensation: 默认按 URDF 中 `link6 -> tool0` 的局部 `+Z 0.1m` 固定偏移补偿后再调 C++ IK；JSONL 中 `target_pose` 仍表示期望 `tool0` 位姿，`ik_target_pose` 表示实际传给 IK 的补偿目标，`ik_result_raw` 保留补偿目标的原始 IK 误差。
- output: `/tmp/pick_place_baseline.jsonl`

示例：

```bash
ros2 run alfa_robot_benchmarks pick_place_baseline \
  --timeout 2.0 \
  --output /tmp/pick_place_baseline.jsonl
```

只跑第 1 轮：

```bash
ros2 run alfa_robot_benchmarks pick_place_baseline --start 0 --rounds 1
```

输出 JSONL 可以直接用现有 Rerun 回放脚本查看目标和实际末端位置：

```bash
ros2 run alfa_robot_rerun visualize_rerun /tmp/pick_place_baseline.jsonl
```

默认会同时加载当前 `alfa_robot_description` 的 URDF visual mesh，并用每个 sample 的 `result.joint_values` 离线 FK 回放整机姿态。只看末端点位时可以加：

```bash
ros2 run alfa_robot_rerun visualize_rerun /tmp/pick_place_baseline.jsonl --no-robot
```

如果只想生成文件、不弹 Rerun 窗口：

```bash
ros2 run alfa_robot_rerun visualize_rerun /tmp/pick_place_baseline.jsonl --save /tmp/pick_place_baseline_robot.rrd
rerun /tmp/pick_place_baseline_robot.rrd
```

默认 baseline 会把 PlanningScene 判定为碰撞的 IK 候选解拒掉。注意这只是安全过滤，不是优化器；因此 `pick_place_baseline` 默认会先用上一步成功关节姿态作为 seed，再用确定性扰动 seed 做多次重试，避免“第一个候选解碰撞就直接放弃”。可调参数包括：

```bash
ros2 run alfa_robot_benchmarks pick_place_baseline \
  --seed-attempts 24 \
  --seed-noise 0.8 \
  --updown-seed-noise 0.12
```

若要复现/诊断旧行为，可以临时允许碰撞解并在 JSONL 中查看 `collision_pairs`：

```bash
ros2 run alfa_robot_benchmarks pick_place_baseline --allow-collision-solutions --output /tmp/pick_place_baseline_allow_collision.jsonl
```

## 实时碰撞状态监控

`alfa_robot_description/view_alfa_robot.launch.py` 只是 description 预览：它启动 `joint_state_publisher_gui`、`robot_state_publisher` 和 RViz `RobotModel`，不会加载 MoveIt PlanningScene，因此碰撞不会自动把机器人变红。

需要在另一个终端启动碰撞监控节点：

```bash
ros2 run alfa_robot_benchmarks collision_state_monitor
```

它会订阅 `/joint_states`，用当前 URDF + SRDF + MoveIt PlanningScene 检查碰撞，并发布 `/alfa_collision_markers`。RViz 配置中已加入 `CollisionStatus` MarkerArray 显示：

- 绿色文字：`collision free`
- 红色文字：`COLLISION` 和碰撞 link 对
- 红色球：MoveIt 返回的接触点

也可以手动添加 RViz Display：`Add -> By topic -> /alfa_collision_markers -> MarkerArray`。

## 新增：固定前向轴范围 IK

核心可执行：

```bash
ros2 run alfa_robot_benchmarks ik_range_grid --help
```

Python 包装脚本：

```bash
python3 scripts/ik_benchmark/scripts/ik_range_grid.py --help
```

语义：

- 给定 `x/y/z` 范围和步长。
- 对每个位置，固定末端某个局部轴朝向基坐标某个方向。
- 允许吸盘绕这个朝向轴旋转：通过 `--spin-samples` 枚举若干个绕轴角度。
- 只要某个 spin 角 IK 成功，这个位置就记为可达。
- CSV 可使用同包工具 `scripts/ik_csv_open3d_visualizer.py` 查看。

示例：当前模型，左臂，末端局部 `+Y` 朝基坐标 `+Y`，绕 `+Y` 自由旋转：

```bash
python3 scripts/ik_benchmark/scripts/ik_range_grid.py \
  --version current \
  --group left_arm_with_base \
  --solver kdl \
  --tip-link leftjoint6 \
  --x -0.3 2.0 0.05 \
  --y -0.3 0.8 0.05 \
  --z 0.5 2.5 0.05 \
  --forward-axis y \
  --target-axis y \
  --spin-samples 12 \
  --timeout 0.02 \
  --output /tmp/alfa_current_fixed_forward.csv \
  --visualize open3d
```

如果实际吸盘前向不是 `+Y`，改 `--forward-axis`，例如 `x`、`z`、`-z`。

## 2/3/4 代独立 URDF

这些 URDF 是独立副本，只用于这里的 IK 对比，不会被 description / MoveIt / MuJoCo 引用：

```bash
scripts/ik_benchmark/models/urdf_versions/v2/alfa_robot.urdf
scripts/ik_benchmark/models/urdf_versions/v3/alfa_robot.urdf
scripts/ik_benchmark/models/urdf_versions/v4/alfa_robot.urdf
```

来源提交：

- v2: `f84dcbb8acb28a616d954d8b7643b66817b2ce2c`
- v3: `9ef92b8ed42f4643db870eaa85c673ce7fd063ac`
- v4: `303f191a0dde617bcd5601cf918eb9b07861abcb`

对比某一代：

```bash
python3 scripts/ik_benchmark/scripts/ik_range_grid.py \
  --version v4 \
  --group left_arm_with_base \
  --solver kdl \
  --tip-link leftjoint6 \
  --x -0.3 2.0 0.05 \
  --y -0.3 0.8 0.05 \
  --z 0.5 2.5 0.05 \
  --forward-axis y \
  --target-axis y \
  --spin-samples 12 \
  --output /tmp/alfa_v4_fixed_forward.csv
```

V2 的 SRDF 当前未单独保存，因此默认仍使用当前 `alfa_robot.srdf`。如果你希望 V2 严格按旧 SRDF 分组跑，需要再提供/导出 V2 对应 SRDF，并用 `--srdf <path>` 指定。

## 解析 IK 姿态 RGB 可达性点云

`analytic_orientation_rgb_reachability` 直接调用 `alfa_robot_analytic_ik`，不经过 ROS Service、MoveIt 或数值 IK。当前定义为右臂工具法向从 `+X=0°` 绕 world `-Y` 连续转到 `+Z=90°`：

- R：`0,5,...,30°` 的成功比例；
- G：`35,40,...,60°` 的成功比例；
- B：`65,70,...,90°` 的成功比例；
- 每个通道全部可达时取 `255`；
- 只把存在 `joint3 > 0` 且 `joint4 < 0` 解析分支的姿态计为可达；
- 只检查解析 IK 与关节限位，不包含 MoveIt 碰撞过滤。

生成 CSV 后使用 `visualize_orientation_rgb_reachability.py` 加载完整机器人并保存 Rerun：

```bash
ros2 run alfa_robot_benchmarks analytic_orientation_rgb_reachability -- \
  --min-x -0.09 --max-x 0.98 --min-y -0.8 --max-y 0.0 \
  --min-z 1.15 --max-z 2.2 --step-x 0.05 --step-y 0.05 --step-z 0.05 \
  --min-angle-deg 0 --max-angle-deg 90 --angle-step-deg 5 \
  --fixed-updown 0.45 --output-csv /tmp/right_arm_rgb.csv

ros2 run alfa_robot_benchmarks visualize_orientation_rgb_reachability.py -- \
  /tmp/right_arm_rgb.csv --save /tmp/right_arm_rgb.rrd --updown 0.45
```

## 径向抽离冻结测试组

`analytic_radial_extract_prototype` 在径向段最后合法帧后固定工具位置与 Updown，按默认 `2°` 分辨率只旋转工具朝向，使工具轴对齐冻结的 Joint2 中心径向线；随后执行到负重位的 Shortcut 与局部 RRT。若转腕接管失败，会回退原径向末帧。

实验参数 `--resume-radial-after-orientation` 会在转腕完成后保持工具轴与径向线对齐，继续把径向角收敛到竖直、工作半径收敛到目标值；遇到下一次解析、限位或碰撞失败时，直接从最后合法续推帧执行 Shortcut 与局部 RRT。该模式不会回退旧径向末帧，用于严格衡量续推策略本身。

同时启用 `--interleave-loaded-shortcuts` 时，从原径向末态开始，在每个转腕和径向续推合法检查点先验证一次到负重位的完整 Shortcut；首次通过立即停止几何调整。全部检查点均被拒绝时，才从最后合法检查点调用局部 RRT。

后续方案比较必须复用相同 IK snapshot，避免重新选 IK 污染成功率：

```bash
ros2 run alfa_robot_benchmarks run_radial_frozen_target_set.py -- \
  --manifest /path/to/success_rate_target_set_v1/manifest.json \
  --snapshot-root /path/to/radial_loaded_grid/cases \
  --output /tmp/radial_frozen_ab \
  --resume-radial-after-orientation \
  --interleave-loaded-shortcuts
```
