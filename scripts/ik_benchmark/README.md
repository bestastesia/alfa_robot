# ALFA IK Benchmark / Range Grid

这个包用于离线、进程内调用 MoveIt kinematics plugin，避免 `/compute_ik` ROS service 逐点调用的通信开销。

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
- CSV 兼容 `ros2_ws/src/alfa_robot_moveit_config/scripts/ik_csv_open3d_visualizer.py`。

示例：当前模型，左臂，末端局部 `+Y` 朝基坐标 `+Y`，绕 `+Y` 自由旋转：

```bash
python3 scripts/ik_benchmark/scripts/ik_range_grid.py \
  --version current \
  --group left_arm_with_base \
  --solver trac_ik \
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
  --solver trac_ik \
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
