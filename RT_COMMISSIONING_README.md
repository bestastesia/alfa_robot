# 实时 6D Pose × RT-Control 联调包

版本基线：Motion `v5_dev` / `0020d98ea285ceefff8043831255f83b81f60ddb`。

这个目录把联调分成三个明确阶段，禁止直接从可视化拖动开始：

1. `observe`：只订阅 `/joint_states` 和 rolling state，不创建
   `/rt/rolling_joint_control/update` publisher，不切控制模式。
2. `hold`：读取连续 25 帧完整 14 轴反馈并确认静止，冻结当前姿态；切换并 Open 后，
   电控返回的 `hold_positions` 必须与冻结姿态在 `0.25 deg / 1 mm` 内一致；首批只发送
   当前姿态、零速度的 future suffix。
3. `track`：启动后仍先走上述 HOLD。只有 `last_accepted_sequence >= 1` 且状态为
   `RUNNING`，才在另一终端显式启动可拖动目标。

## 0. 只读检查

```bash
ssh ar@192.168.0.40
cd ~/motion_realtime_pose_demo_0020d98
ROS_DOMAIN_ID=0 ./run_realtime_pose_rt_commissioning.sh observe --no-rviz --no-rerun
```

看到 `stable 14-axis startup pose captured` 后按 `Ctrl-C`。此阶段机械臂不会运动。

## 1. 当前姿态 HOLD（必须由 Motion 与 ELECTRI 共同值守）

前提：ELECTRI 确认 EC-04 已通过、机器人已使能、FJT 已取消且处于 `FJT_READY`、
现场急停和停机操作员就位。当前若 `rolling_trajectory_controller` 仍为 `inactive`，不执行本阶段。

```bash
cd ~/motion_realtime_pose_demo_0020d98
export ROS_DOMAIN_ID=0
export ALFA_RT_COMMISSIONING_CONFIRM=CURRENT_POSE_HOLD_ACKED
./run_realtime_pose_rt_commissioning.sh hold --supervised --allow-provisional-limits \
  --arm left --rate 30 --no-rviz --no-rerun
```

另开终端观察：

```bash
source /opt/ros/humble/setup.bash
source ~/motion2_test_rolling_native_ws_20260821-212157_clean/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /realtime_6d_pose/rt_status std_msgs/msg/String
```

只有出现 `session_state=RUNNING`、`last_accepted_sequence>=1`、`last_reject=NONE`，
且机械臂保持不动，才算 HOLD 通过。退出主程序使用 `Ctrl-C`；客户端会依次
`REQUEST_STOP -> HOLDING -> FINALIZE -> FJT_READY`，不要直接 `kill -9`。

## 2. 拖动跟踪

重新以 `track` 启动；它仍默认不启动拖动 Marker：

```bash
export ROS_DOMAIN_ID=0
export ALFA_RT_COMMISSIONING_CONFIRM=CURRENT_POSE_HOLD_ACKED
./run_realtime_pose_rt_commissioning.sh track --supervised --allow-provisional-limits \
  --arm left --rate 30
```

确认首批 HOLD 已 ACK 后，在第二终端显式解锁 Marker：

```bash
cd ~/motion_realtime_pose_demo_0020d98
export ROS_DOMAIN_ID=0
export ALFA_RT_TRACKING_CONFIRM=HOLD_SEQUENCE_ACCEPTED
./start_realtime_pose_target_marker.sh
```

第一次只做左臂、低幅、单轴慢速拖动；右臂、`turn`、`updown` 仍随已接受 future
采样，不复制瞬时 `/joint_states`。每次 IK 目标仍经过解析 IK、关节突变和连续边碰撞检查。

## 当前不能宣称已完成真机联调

2026-08-22 只读检查时：`/joint_states` 为 125 Hz 且 14 轴完整，但
`whole_body_jtc` 与 `rolling_trajectory_controller` 都为 `inactive`。因此包已可用于
`observe`，HOLD/track 文件已上传待联调；没有在本次上传中切模式、使能或发送运动指令。
