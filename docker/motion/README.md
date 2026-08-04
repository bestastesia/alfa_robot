# Motion 域 Docker 开发联调

本目录验证 Motion 的单一阶段动作入口。Motion 只负责运动规划、轨迹执行和阶段回执；吸附通路、真空泵及真空阈值由 Autonomy 编排 RT-Control，不再经过 Motion。

## 当前接口

| 名称 | 类型 | 方向 |
|---|---|---|
| `/motion/execute_stage` | `alfa_motion_interfaces/action/ExecuteMotionStage` | Autonomy/测试客户端 → Motion |
| `/motion/readiness` | `alfa_motion_interfaces/msg/MotionReadiness` | Motion → Autonomy/观测工具 |
| `/dual_arm_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion → RT-Control |
| `/joint_states` | `sensor_msgs/msg/JointState` | RT-Control → Motion |

`ExecuteMotionStage` 固定五个阶段：重拍位、预抓取、靠近吸附、放置、返回初始位。一次任务必须先后发送两批目标，且两批使用同一 `task_id`：

1. `MOVE_TO_RECAPTURE`：左右各一个重拍末端 `PoseStamped(base_link)`；Motion 完成解析 IK、碰撞规划和执行。
2. `MOVE_TO_PREGRASP`：左右各一个精定位后的箱体正面中心 `PoseStamped(base_link)`；Motion 从重拍执行后的真实关节状态开始完整规划。

每个目标使用 `MotionPoseTarget`，包含 `pose` 和 `grasp_mode`。模式枚举为 `NO_MOVE`、`SIDE_SUCTION`、`TOP_SUCTION`；本版只完成接口接收和合法性校验，策略仍按现有位姿分类逻辑运行。后续阶段通过同一 `task_id` 使用已缓存计划。Goal 不包含箱号或 PLC 指令。Motion 不订阅或发布 `/plc/*`，也不提供 `/vacuum/grip`。

## 本机 Mock

```bash
cd docker/motion
mkdir -p .workspace ../../data/docker_motion
MOTION_UID=$(id -u) MOTION_GID=$(id -g) MOTION_ROS_DOMAIN_ID=142 \
MOTION_RT_MODE=mock MOTION_DRY_RUN=false \
docker compose up --build motion
```

另一个终端发送测试位姿：

```bash
MOTION_UID=$(id -u) MOTION_GID=$(id -g) MOTION_ROS_DOMAIN_ID=142 \
docker compose --profile manual run --rm task \
  --recapture-left 0.75 0.50 1.60 3.1415926 -1.5707963 0.0 \
  --recapture-right 0.75 -0.50 1.60 3.1415926 -1.5707963 0.0 \
  --left 0.90 0.50 1.60 3.1415926 -1.5707963 0.0 \
  --right 0.90 -0.50 1.60 3.1415926 -1.5707963 0.0 \
  --yes-execute
```

测试客户端只模拟 Autonomy 发送 Motion 阶段，不操作吸附通路。

## 实机联调

1. 独立启动 rt-control，确认 READY。
2. 启动 Motion：

```bash
MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE \
  tools/motion_domain_docker.sh start-external
```

3. 确认接口：

```bash
ROS_DOMAIN_ID=42 ros2 action list -t | grep -E 'motion/execute_stage|dual_arm_jtc'
```

Motion 容器不访问 EtherCAT/CANopen，不调用 `/rt/enable`，不管理 rt-control 生命周期。源码只读挂载到 `/repo`，Release 构建产物保存在 `docker/motion/.workspace`。当前仍使用 host network + Fast DDS UDPv4，避免 root 容器与宿主普通用户的 SHM 权限不一致。

## 当前限制

- `grasp_mode` 已进入正式消息，但本版尚未改变现有任务策略分类。
- N-03 Gate、SafetyState、模型/标定版本准入尚未接入。
- `allow_partial_domain_test=true` 仅用于开发联调，不代表生产验收。
