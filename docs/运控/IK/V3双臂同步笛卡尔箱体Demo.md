# V3 双臂同步笛卡尔箱体 Demo

## 目标

- 双手末端朝身体内侧，相距 `0.40 m`，刚性夹持 `0.40 m` 正方体。
- 拖动目标箱中心后，箱体沿空间直线移动。
- 每个笛卡尔采样点同时求左右臂解析 IK。
- 两臂使用同一插值比例做边碰撞验证，不允许一只手先动、另一只后动。
- 箱体附着在机器人末端后参与碰撞检测。

## 启动

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_moveit_config v3_dual_arm_cartesian_box_demo.launch.py
```

默认目标相对当前箱中心后移 `0.12 m`。也可直接指定启动后的自动测试偏移：

```bash
ros2 launch alfa_robot_moveit_config v3_dual_arm_cartesian_box_demo.launch.py \
  auto_run_once:=true \
  target_offset_x:=0.0 \
  target_offset_y:=0.08 \
  target_offset_z:=0.0
```

## 操作

1. 在 RViz 中拖动青色球调整目标箱中心；控制球故意放在箱体外侧，避免被箱体遮挡。
2. 右键青色球，选择“确认并计算同步直线”。
3. 计算完成后，RViz 和 Rerun 同步播放双臂直线搬运过程。
4. 右键菜单可将目标恢复到当前箱位，或恢复初始握持姿态。

也可通过服务触发当前目标：

```bash
ros2 service call /v3_dual_arm_cartesian_box_demo/run_current_target std_srvs/srv/Trigger '{}'
```

## 当前边界

该程序只实现第一个双臂任务：固定朝向、固定 `0.40 m` 抓取间距的同步平移。绕箱体中心旋转和一横一竖握持将在本 Demo 验证后继续实现。
