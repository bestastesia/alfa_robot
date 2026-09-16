# V3 冗余解析 IK 解族交互 Demo

## 作用

- RViz 只负责拖动一个末端 6D 目标。
- 每次目标稳定后，解析 IK 在 `ψ=-180°～180°` 内按默认 `2°` 采样。
- 对同一 `ψ` 下的重复关节解去重，再按肩、肘、腕离散分支组织全部合法解。
- 单一分支遇到无解区间或大于 `20°` 的关节跳变时自动切段，避免把不连续解伪装成连续运动。
- Rerun 自动刷新并循环播放所有连续段，同时显示当前 `ψ`、分支和关节限位裕量。

当前源码使用 V3.0.9 模型与解析求解器；这里展示的是关节限位和解析方程下的采样解族，不包含碰撞过滤，也不会向真实执行器发送轨迹。

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

## V3.0.9 独立启动回归（2026-09-10）

本 demo 自己发布16轴状态（两臂14轴及 `updown=0`、`head_joint=0`），
不需要先启动 MoveIt mock。先发布状态，再等待 TF：升降是可动关节，
`robot_state_publisher` 必须先收到其位置才能连接 world 与 arm_carriage。
此前的“先等 TF 再发状态”会在独立启动时一直显示 `WAITING TF`。

从仓库根目录执行；先构建修改后的 `alfa_robot_moveit_config`：

```bash
source tools/ros_humble_env.sh
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=181 python3   ros2_ws/src/alfa_robot_moveit_config/test/test_v3_redundant_ik_startup.py   --artifacts /tmp/v3-ik-startup-check
```

使用空闲的本机 ROS domain，不与其他 demo 共用。检查会自行启动无界面 launch，
在25秒内验证默认左臂目标有非空解族、收到全部16轴状态，随后只停止自身启动的进程。
`launch.log` 和 `received.json` 保留结果；不需要 RViz 或 Rerun。

本次同一检查修复前失败，收到 `WAITING TF`；修复后通过，默认2°采样得到376解、8段。
这些数值属于当前默认输入的单次结果，不替代上面的 V3.0.6 历史结果，也不证明碰撞安全或可执行轨迹。
