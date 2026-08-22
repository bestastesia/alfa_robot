# PROTOTYPE：Motion 实时 6D Pose 滚动消息预览

这个原型只回答一个问题：用户拖动末端 6D Pose 后，目标经过 Motion 的解析 IK、关节突变和
连续边碰撞检查，Motion 会以 10/30 Hz 向 rt-control 发送什么滚动消息？

```text
RViz 拖动 6D 目标
  -> Motion 100 Hz：解析 IK -> 关节突变 -> 连续边碰撞
  -> Motion TX 10/30 Hz：生成 6×100 ms、完整14轴 future suffix
  -> 安全预览 topic + Rerun
```

这个版本不启动、不连接也不模拟 rt-control：

- 不创建任何 `/rt/*` service、publisher 或 subscriber。
- 不发布 `/joint_states`，不伪造“机械臂实际反馈”。
- Rerun 紫色机器人表示当前滚动消息的最后一个未来点，不表示机器人已经运动到那里。
- 连续 suffix 以“上一条已发送消息”为拼接基线，仅用于观察 Motion 输出；它不是 RT ACK。
- 活动臂使用项目当前 MoveIt 速度/加速度上限生成连续带速轨迹；固定目标只规划一次绝对时间
  profile，不再在每个500ms窗口末端重复停车。该预览包络没有经过rt-control limits hash协商。

## 一条命令运行

30 Hz：

```bash
cd /mnt/mydisk/ALFA/alfa_robot
ROS_DOMAIN_ID=151 ./run_realtime_pose_teach_pendant.sh --arm left --rate 30
```

10 Hz：

```bash
ROS_DOMAIN_ID=151 ./run_realtime_pose_teach_pendant.sh --arm left --rate 10
```

操作方式：

1. RViz 选择左上角 `Interact`。
2. 拖动橙色球的箭头修改 XYZ，拖动圆环修改姿态。
3. Rerun 显示橙色原始目标、蓝色 Motion 安全目标和紫色 Motion TX suffix 末点。
4. 状态区逐批显示 sequence、实测发送频率、replace 时间、6个future点以及活动臂的角度/速度。

无图形 smoke：

```bash
ROS_DOMAIN_ID=151 ./run_realtime_pose_teach_pendant.sh \
  --arm left --rate 30 --no-rviz --no-rerun
```

## 实际消息

Motion 边界使用正式消息类型：

```text
robot_motion_interfaces/msg/RollingJointTargetBatch
```

但发布在隔离预览 topic，绝不会误发给控制器：

```text
/realtime_6d_pose/motion_tx_preview
```

直接观察完整14轴消息：

```bash
ROS_DOMAIN_ID=151 ros2 topic echo \
  /realtime_6d_pose/motion_tx_preview \
  robot_motion_interfaces/msg/RollingJointTargetBatch
```

检查实际发布频率：

```bash
ROS_DOMAIN_ID=151 ros2 topic hz /realtime_6d_pose/motion_tx_preview
```

其他本地 topic：

- `/realtime_6d_pose/target_pose_cmd`：RViz 拖动输入。
- `/realtime_6d_pose/commanded_joint_states`：通过 Motion 安全门的14轴目标。
- `/realtime_6d_pose/motion_tx_preview`：准备发送的正式 rolling batch 消息形状。
- `/realtime_6d_pose/motion_tx_preview_joint_states`：只给 Rerun 绘制 suffix 末点。
- `/realtime_6d_pose/motion_tx_status`：Rerun 使用的消息展开与频率诊断。

消息里的 controller/session/client UUID 是明确标记的预览占位值，没有经过 rt-control
open/session 协商。将来接真实控制器时应由正式 session client 填入，而不是复用这些值。

## 核心代码

- `realtime_6d_pose_pipeline.cpp`：解析 IK、突变检测、连续边碰撞。
- `rolling_suffix.py`：纯算法的短时域 suffix、限速和连续插值。
- `motion_rolling_preview.py`：10/30 Hz 消息组装、发布和状态展开。
- `realtime_6d_pose_rerun.py`：Motion目标、TX末点和每批消息显示。
- `realtime_6d_pose_prototype.launch.py`：只启动 Motion、TX预览、RViz和Rerun。

## 编译

Humble 的 `rclpy` 使用系统 Python 3.10：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --base-paths src ../scripts/ik_benchmark \
  --packages-up-to robot_motion_interfaces alfa_robot_benchmarks alfa_robot_rerun \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
```

这个目录仍是可删除原型，不是生产视觉伺服或正式 rt-control 客户端。
