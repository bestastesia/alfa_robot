# Motion 域 Docker 开发联调

本目录只封装 Motion 运行环境。Motion 通过 ROS 2 原生接口连接 Autonomy 和 rt-control，不通过 Docker 私有通信，也不拥有电磁阀、真空泵或真空阈值控制权。

## 对外接口

| 名称 | 类型 | 方向 |
|---|---|---|
| `/motion/execute_stage` | `robot_motion_interfaces/action/ExecuteMotionStage` | Autonomy/测试客户端 → Motion |
| `/motion/readiness` | `robot_system_interfaces/msg/DomainReadiness` | Motion → Autonomy/观测工具 |
| `/dual_arm_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion → rt-control |
| `/joint_states` | `sensor_msgs/msg/JointState` | rt-control → Motion |

`ExecuteMotionStage` 固定五个阶段：

1. `CAMERA_VIEW`：接收第一对重拍末端 Pose，保留输入姿态，只做 Turn=0 虚拟模型坐标换算后规划并执行重拍位。
2. `PREGRASP`：接收第二对吸附面中心 Pose，Motion 可按顶吸/侧吸标准化抓取姿态并计算完整计划。
3. `APPROACH`：执行预抓取到吸附位的 5cm 靠近轨迹。
4. `PLACE`：执行抽离、负重过渡和放置轨迹。
5. `HOME`：执行放置位到初始位轨迹，完成后清除本轮计划。

两批 Pose 都固定表达在 `base_link`，消息不带 frame、时间戳、任务号或箱号。Action Goal UUID 是请求身份。`APPROACH/PLACE/HOME` 的 `targets` 被忽略。

规划算法始终把 `turn` 视为 `0`。除 `CAMERA_VIEW` 开头的专用对齐轨迹外，Motion 下发所有十四轴轨迹时都把 `turn` 锁定为最新真实反馈值。

## 本机 Mock

首次启动前导入锁定版本的中央接口仓库：

```bash
cd ros2_ws
vcs import src < src/dependencies.repos
```

```bash
cd docker/motion
mkdir -p .workspace ../../data/docker_motion
MOTION_UID=$(id -u) MOTION_GID=$(id -g) MOTION_ROS_DOMAIN_ID=142 \
MOTION_RT_MODE=mock MOTION_DRY_RUN=false \
docker compose up --build motion
```

另一个终端模拟 Autonomy：

```bash
MOTION_UID=$(id -u) MOTION_GID=$(id -g) MOTION_ROS_DOMAIN_ID=142 \
docker compose --profile manual run --rm task \
  --recapture-left 0.70 0.40 1.597906 3.1415926 -1.5707963 0.0 \
  --recapture-right 0.70 -0.40 1.597906 3.1415926 -1.5707963 0.0 \
  --task B1 --front-distance 0.70 --top-distance 0.70 \
  --interactive --yes-execute
```

## 实机联调

1. 独立启动 rt-control，确认其输出 `READY`。
2. 启动 Motion：

```bash
MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE \
  tools/motion_domain_docker.sh start-external
```

3. 查看接口：

```bash
ROS_DOMAIN_ID=42 ros2 action list -t | grep -E 'motion/execute_stage|dual_arm_jtc'
```

Motion 不启动、使能、复位或停止 rt-control。源码只读挂载到 `/repo`，Release 构建产物保存在 `docker/motion/.workspace`。当前使用 host network + Fast DDS UDPv4，规避 root 容器与宿主普通用户间的 SHM 权限问题。

## 轨迹缓存

Motion 包内置默认 Y、`0.70～0.75m × 五排` 的 30 条已验证轨迹。距离按厘米向上取整；缓存文件、排数和起点状态同时匹配时跳过完整规划，否则自动回退实时 planner。模型、场景或关节合同变化后必须重新生成缓存。

## 当前限制

- `left_grasp_mode/right_grasp_mode` 已校验，但现有策略仍主要由吸附面高度分类。
- 当前双臂全流程不支持单侧 `NO_MOVE`。
- Gate、安全状态、模型/标定版本强制准入尚未接入。
- `allow_partial_domain_test=true` 只用于开发联调。
