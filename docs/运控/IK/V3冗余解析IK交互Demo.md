# V3 冗余解析 IK 解族交互 Demo

## 作用

- RViz 只负责拖动一个末端 6D 目标。
- 每次目标稳定后，解析 IK 在 `ψ=-180°～180°` 内按默认 `2°` 采样。
- 对同一 `ψ` 下的重复关节解去重，再按肩、肘、腕离散分支组织全部合法解。
- 单一分支遇到无解区间或大于 `20°` 的关节跳变时自动切段，避免把不连续解伪装成连续运动。
- Rerun 自动刷新并循环播放所有连续段，同时显示当前 `ψ`、分支和关节限位裕量。

这里展示的是当前 V3.0.6 URDF、关节限位和解析方程下的完整采样解族，不包含碰撞过滤，也不会向真实执行器发送轨迹。

## 启动

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_moveit_config v3_redundant_ik_interactive_demo.launch.py
```

启动后会出现两个窗口：

1. RViz：选择顶部 `Interact`，拖动蓝色末端目标球。
2. Rerun：自动播放该末端位姿下的全部合法冗余解段。

右臂模式：

```bash
ros2 launch alfa_robot_moveit_config \
  v3_redundant_ik_interactive_demo.launch.py side:=right
```

提高 `ψ` 采样密度并保存 Rerun：

```bash
ros2 launch alfa_robot_moveit_config \
  v3_redundant_ik_interactive_demo.launch.py \
  psi_step_deg:=1.0 \
  rerun_recording_path:=/tmp/v3_redundant_family.rrd
```

Rerun 按相邻解的最大关节变化自适应播放间隔，默认等效关节速度约 `30°/s`。不同离散分支之间会暂停并直接切换，不表示可执行轨迹。

## V3.0.6 验证结果

- V3.0.6 仍满足球肩—肘—球腕结构，肩、肘、腕三组轴线共点残差均小于 `1nm`。
- 上臂和前臂中心距分别为 `0.506m`、`0.476m`。
- 默认 `2°` 冗余角采样得到 `792` 个去重合法解、`8` 个连续分支，整组求解约 `2.27ms`。
- 已录制：`data/ik_benchmark/v3_0_6_extended_reachability/v3_0_6_redundant_solution_family.rrd`。
