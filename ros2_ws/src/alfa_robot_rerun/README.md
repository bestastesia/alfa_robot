# alfa_robot_rerun

ALFA Robot 的公共只读 Rerun 适配包。

本包集中维护：

- 当前 `alfa_robot_description` xacro 的展开与 URDF 解析；
- link visual mesh 记录；
- URDF 前向运动学和关节状态回放；
- `/joint_states` 实时观察；
- benchmark JSONL 到 Rerun/.rrd 的离线回放。

它不拥有权威机器人状态，不替代 MoveIt/RViz，也不参与规划或成功判定。生产包和实验工具需要机器人 Rerun 可视化时，应依赖本包，不再复制 URDF/FK helper。

## Dependency

Use the ROS system Python environment:

```bash
/usr/bin/python3 -m pip install --user rerun-sdk
```

The Rerun CLI/viewer should also be available:

```bash
rerun --version
```

## Build

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
colcon build --packages-select alfa_robot_rerun
source install/setup.bash
```

## 实时关节状态

启动任意 `/joint_states` 来源后运行：

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py
```

不弹出 viewer：

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py spawn:=false
```

保存记录：

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py recording_path:=/tmp/alfa_basic.rrd
```

也可以直接运行公共实时节点：

```bash
ros2 run alfa_robot_rerun rerun_joint_state_viewer_node
```

## 离线 JSONL 回放

```bash
ros2 run alfa_robot_rerun visualize_rerun result.jsonl
ros2 run alfa_robot_rerun visualize_rerun result.jsonl --save result.rrd
```

旧的 `scripts/ik_benchmark/scripts/visualize_rerun.py` 仅保留兼容包装，实际实现仍来自本包。

## 使用 joint_state_publisher_gui 验证

In one terminal:

```bash
ros2 launch alfa_robot_description view_alfa_robot.launch.py
```

In another terminal:

```bash
ros2 launch alfa_robot_rerun basic_robot_viewer.launch.py
```

Rerun 包只依赖 `/joint_states` 和机器人 description，不依赖 MoveIt 任务流程。

## 右臂可达性范围编辑器

只确定九向可达性测试的空间范围，不调用 IK、不执行碰撞检测：

```bash
ros2 run alfa_robot_rerun reachability_bounds_editor
```

工具同时打开 Tk 参数面板和 Rerun：

- 显示完整机器人，`updown=0.45m`，其余关节为零；
- 以 `world` 为坐标系动态显示右臂采样包围盒与点阵；
- 可调整 XYZ 最小值、最大值和各轴分辨率；
- 显示实际点数与九向 IK 调用总数，并可复制后续测试参数；
- 该工具不发布关节状态，也不会启动实际可达性计算。

也可以在启动时给定初值：

```bash
ros2 run alfa_robot_rerun reachability_bounds_editor -- \
  --min-x 0.25 --max-x 1.20 \
  --min-y -0.80 --max-y 0.20 \
  --min-z 0.50 --max-z 2.20
```

## 职责边界

- 允许：只读机器人状态、轨迹、场景和诊断数据的可视化与记录。
- 禁止：发布权威状态、修改场景、决定任务阶段、执行规划或把可视化结果当作验收真值。
- 新 overlay 应作为独立 logger 接入公开消息，不直接导入运行时私有实现。
