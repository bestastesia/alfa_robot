# DH Workspace Sampler

离线 DH 运动学验证工具，用于快速检查 DH 参数方案的 FK 和末端工作空间点云。它不依赖 ROS；如果安装了 `roboticstoolbox-python` 会优先使用 Robotics Toolbox，否则自动回退到内置 DH FK。


## Current Default Example

`configs/example_4dof.yaml` 当前已切换为你的新方案：前置平动 + 5 轴 `Yaw + Pitch + orthogonal spherical wrist`。
为了避免文件名误导，同一份配置也复制为：

```bash
scripts/dh_workspace/configs/alfa_5axis_spherical_wrist.yaml
```

关键 DH 设计：

| Link | Role | d | a | alpha |
| --- | --- | --- | --- | --- |
| L0 | base lift / prismatic | q0, range 0..0.99 | 0 | pi/2 |
| L1 | base yaw | 0.2 | 0 | pi/2 |
| L2 | shoulder pitch | 0 | 0 | -pi/2 |
| L3 | wrist roll / arm length | 0.35 | 0 | pi/2 |
| L4 | wrist pitch | 0 | 0 | -pi/2 |
| L5 | flange roll / tool length | 0.08 | 0 | 0 |

其中 L0 是第一个旋转轴前的平动关节，`alpha=pi/2` 让平动方向与第一个 yaw 旋转轴垂直；L4 的 `d=0, a=0` 保证第 3/4/5 轴在手腕中心交汇。

## Install

```bash
pip install roboticstoolbox-python matplotlib pyyaml
```

如果使用 Open3D 可视化：

```bash
pip install open3d
```


## Visualize Robot Structure with teach()

当前 Robotics Toolbox 版本对 `DHRobot + Swift` 可能不支持；本工具默认提供兼容新版 matplotlib 的本地 slider 示教界面，可拖动关节查看 DH 机械结构：

```bash
pip install roboticstoolbox-python spatialmath-python swift-sim
python3 scripts/dh_workspace/dh_teach.py   --config scripts/dh_workspace/configs/example_4dof.yaml   --q 0 -1.5708 0 0   --block
```

如果不传 `--q`，默认所有关节为 0。

如果你想尝试 Robotics Toolbox 自带后端，也可以用 `--backend pyplot` 或 `--backend swift`，但不同版本可能存在兼容问题。

## Run Example

```bash
cd /mnt/mydisk/ALFA/alfa_robot
python3 scripts/dh_workspace/dh_workspace_sampler.py \
  --config scripts/dh_workspace/configs/example_4dof.yaml \
  --samples 30000 \
  --visualizer matplotlib \
  --output /tmp/dh_workspace_points.csv
```

Open3D 点云窗口：

```bash
python3 scripts/dh_workspace/dh_workspace_sampler.py \
  --config scripts/dh_workspace/configs/example_4dof.yaml \
  --visualizer open3d \
  --point-size 3
```

## Fixed Direction Reachability

如果目的不是单纯位置可达，而是“末端某一个轴始终朝向/平行于平动方向”的可达点云，可以开启单轴朝向过滤。这个过滤只约束一个方向轴，不固定另外两个方向的绕轴旋转，因此等价于：工具某根局部轴对齐目标方向，roll 自由。

当前 `alfa_5axis_spherical_wrist.yaml` 里第一个平动关节 `base_lift` 的平动方向是世界/基坐标 `+Z`，所以默认用 `--align-target z` 表示“朝平动方向”。例如要求末端局部 `+Z` 轴朝 `base_lift` 平动方向，误差 5 度以内：

```bash
python3 scripts/dh_workspace/dh_workspace_sampler.py \
  --config scripts/dh_workspace/configs/alfa_5axis_spherical_wrist.yaml \
  --samples 100000 \
  --backend builtin \
  --visualizer open3d \
  --point-size 3 \
  --require-axis-alignment \
  --align-ee-axis z \
  --align-target z \
  --align-angle-deg 5 \
  --output /tmp/alfa_fixed_direction_workspace.csv
```

如果发现工具实际朝向是反的，可以把末端轴改成 `--align-ee-axis -z`；如果正反平行都接受，可以加 `--allow-opposite`。

只保存 CSV 不显示：

```bash
python3 scripts/dh_workspace/dh_workspace_sampler.py --visualizer none
```

## Config Notes

- `convention`: `standard` 或 `modified`。
- `convention: urdf_origin_axis`: 直接按 URDF joint `origin xyz/rpy + axis + type` 做 FK，用于对比不同 URDF 版本，不能用 Robotics Toolbox 原生 DH 后端，但可用本目录脚本的 `--backend builtin` 和 `dh_teach.py --backend custom`。
- `links[].type`: `revolute` 或 `prismatic`。
- revolute 关节采样 `qlim` 作为角度范围，单位 rad。
- prismatic 关节采样 `qlim` 作为位移范围，单位 m。
- CSV 输出包含 `x,y,z` 和每个关节采样值。

这个目录用于 DH 理论验证；MoveIt/URDF 验证脚本仍放在 `ros2_ws/src/alfa_robot_moveit_config/scripts/`，MuJoCo 物理仿真仍放在 `simulation/mujoco/`。

## ALFA 4-Axis With Elbow Proposal

新建议的 4 轴 DH 配置已放在：

```bash
scripts/dh_workspace/configs/alfa_4axis_with_elbow.yaml
```

生成工作空间点云：

```bash
python3 scripts/dh_workspace/dh_workspace_sampler.py   --config scripts/dh_workspace/configs/alfa_4axis_with_elbow.yaml   --samples 30000   --visualizer matplotlib   --output /tmp/alfa_4axis_with_elbow_workspace.csv
```

用 Robotics Toolbox 原生 PyPlot teach 后端查看结构：

```bash
python3 scripts/dh_workspace/dh_teach.py   --config scripts/dh_workspace/configs/alfa_4axis_with_elbow.yaml   --backend pyplot   --q 0 0 0 0   --block
```

## URDF Version Comparison Configs

为了排查 2/3/4 代机械臂结构差异，已从对应提交的 URDF/xacro 里抽取左臂链并生成这些精确 URDF 关节链配置：

```bash
scripts/dh_workspace/configs/alfa_v2_urdf_left_arm_with_base.yaml
scripts/dh_workspace/configs/alfa_v3_urdf_left_arm_with_base.yaml
scripts/dh_workspace/configs/alfa_v4_urdf_left_arm_with_base.yaml
```

采样并可视化某一代：

```bash
python3 scripts/dh_workspace/dh_workspace_sampler.py \
  --config scripts/dh_workspace/configs/alfa_v4_urdf_left_arm_with_base.yaml \
  --samples 30000 \
  --backend builtin \
  --visualizer open3d \
  --output /tmp/alfa_v4_urdf_workspace.csv
```

查看结构和关节滑块：

```bash
python3 scripts/dh_workspace/dh_teach.py \
  --config scripts/dh_workspace/configs/alfa_v4_urdf_left_arm_with_base.yaml \
  --backend custom
```
