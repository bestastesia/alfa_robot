# V3单臂解析抽箱交互Demo

## 目标

验证 V3.0.8 左臂完成以下固定流程：

1. 使用 RRTConnect 从初始关节位移动到箱体正面前 5cm；
2. 使用七轴冗余解析 IK 沿笛卡尔直线前进 5cm；
3. 将 `0.30m × 0.40m × 0.40m` 箱体附着到 `left_tool0`；
4. 使用解析 IK 沿笛卡尔直线向机器人方向抽出 35cm；
5. 携带箱体使用 RRTConnect 返回初始关节位。

目标箱上下左右各有一个同尺寸静态邻箱。静态邻箱、机器人、自碰撞和附着箱使用同一份
MoveIt PlanningScene/FCL 检查。为了避免理想几何恰好共面被数值判为穿透，碰撞尺寸默认
向内收 `2mm/面`，Rerun/RViz仍按真实箱体尺寸显示。

## 启动

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_moveit_config v3_single_arm_box_extract_demo.launch.py
```

启动后：

1. 在 RViz 使用箱堆前外侧青色控制球的红绿蓝平移轴调整目标箱 XYZ；青色连线指向实际目标箱；
2. Rerun 会同步显示目标箱、四个邻箱、预接触点、接触点和抽出终点；
3. 右键青色控制球，选择 **确认并计算当前箱位**；
4. 终端和 Rerun 会显示计算开始、总耗时、分阶段耗时和失败原因；
5. 成功时自动播放完整轨迹，失败时播放已经形成的合法前缀并显示失败阶段。

也可以不用右键菜单，直接触发当前箱位：

```bash
ros2 service call /v3_single_arm_box_extract_demo/run_current_box std_srvs/srv/Trigger '{}'
```

## 自动烟测

不打开窗口并自动计算默认箱位：

```bash
ros2 launch alfa_robot_moveit_config v3_single_arm_box_extract_demo.launch.py \
  start_rviz:=false start_rerun:=false auto_run_once:=true
```

## 算法边界

- 长距离自由空间运动：OMPL `RRTConnect`，使用完整机器人和任务场景碰撞。
- 5cm接触和35cm抽出：每1cm生成一个固定末端位姿，七轴解析IK遍历冗余角并选取连续、
  远离限位且碰撞合法的分支；相邻解析状态额外做2.5度关节插值碰撞检查。
- 箱体从接触结束开始附着到末端；35cm抽出和返回阶段均把箱体作为机器人一部分检查。
- 当前 Demo 固定末端朝世界 `+X`，只允许用户调整箱体中心 `X/Y/Z`。
