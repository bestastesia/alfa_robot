# V3双臂同步绕箱体中心旋转 Demo

## 目标

- 左右末端朝向身体内侧，间距 `0.30m`，刚性夹持 `0.30m` 正方体。
- 箱体中心保持不动，绕机器人正前方的世界 `X` 轴旋转。
- 左右末端始终保持相对箱体的初始位姿，使用 V3 冗余解析 IK 同步求解。
- 每个角度采样点及相邻边均经过 MoveIt/FCL 双臂、机器人本体和附着箱碰撞检查。

## 运行

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_moveit_config v3_dual_arm_box_roll_demo.launch.py
```

RViz 中拖动箱体中心处的蓝色旋转环，右键选择“确认并计算同步绕X旋转”。

默认箱体中心为 `[0.55, 0.0, 0.65]m`，默认目标角为 `25°`。可通过启动参数调整：

```bash
ros2 launch alfa_robot_moveit_config v3_dual_arm_box_roll_demo.launch.py \
  initial_box_x:=0.55 initial_box_y:=0.0 initial_box_z:=0.65 \
  initial_target_roll_deg:=20.0 angular_step_deg:=2.0
```

## 结果说明

- 成功：RViz 与 Rerun 同步播放左右臂和箱体的刚性旋转轨迹，并显示计算耗时。
- 失败：保留已经通过的轨迹帧，状态栏显示失败角度步、解析候选或碰撞原因。
- 当前默认工作点已验证 `+25°` 成功；更大角度受具体箱位、关节限位和碰撞共同约束。
