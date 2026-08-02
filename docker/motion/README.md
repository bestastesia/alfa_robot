# Motion 域初次 Docker 联调

本目录用于验证最小闭环：手工任务客户端按五域规范发送 M-02/M-03，Motion 长驻规划并通过 M-04/M-05 控制已部署的 rt-control。

权威合同以 `/mnt/mydisk/ALFA/SevenovaHangzhou/robot_system_docs/拆垛机器人五域接口规范/` 为准。本实现只覆盖首轮部分域联调，不宣称完成五域 Demo 验收。

## 当前外部接口

| ID | 名称 | 类型 | 当前状态 |
|---|---|---|---|
| M-02 | `/motion/plan_and_execute_pick` | `alfa_task_interfaces/action/PlanAndExecutePick` | 已实现，当前要求恰好 LEFT+RIGHT |
| M-03 | `/motion/plan_and_execute_place` | `alfa_task_interfaces/action/PlanAndExecutePlace` | 已实现，必须匹配上一 M-02 |
| M-04 | `/whole_body_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 迁移桥转发到现行 `/dual_arm_jtc/...` |
| M-05 | `/vacuum/grip` | `alfa_control_interfaces/action/VacuumGrip` | 迁移桥转换为现行 PLC 服务并读取 `/plc/io_state` |
| M-06 | `/motion/readiness` | `alfa_system_interfaces/msg/DomainReadiness` | 已发布开发态摘要，不替代完整五域准入 |

M-07、N-03 Gate、SafetyState、RobotModelInfo 和 CalibrationInfo 尚未进入首轮闭环。`allow_partial_domain_test=true` 只允许用于明确标记的部分域测试。

## 与 rt-control 的通信

- `network_mode: host`
- `ROS_DOMAIN_ID=42`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Fast DDS 仅启用 UDPv4，不使用宿主 root 容器与普通用户之间曾失败过的 SHM 通道

Motion 容器不访问 EtherCAT/CANopen 设备、不调用 `/rt/enable`，也不管理 rt-control 生命周期。
M-05 `RELEASE` 只关闭目标电磁阀，不在任务过程中关闭真空泵。

## 本机 Mock 验证

```bash
cd docker/motion
mkdir -p .workspace ../../data/docker_motion
MOTION_UID=$(id -u) MOTION_GID=$(id -g) MOTION_ROS_DOMAIN_ID=142 \
MOTION_RT_MODE=mock MOTION_DRY_RUN=false \
docker compose up --build motion
```

另一个终端发送两个 6D 位姿。角度单位为弧度：

```bash
cd docker/motion
MOTION_UID=$(id -u) MOTION_GID=$(id -g) MOTION_ROS_DOMAIN_ID=142 \
docker compose --profile manual run --rm task \
  --left-box-id 1 --right-box-id 3 \
  --left-mode front --right-mode front \
  --left 0.90 0.40 1.60 3.1415926 -1.5707963 0.0 \
  --right 0.90 -0.40 1.60 3.1415926 -1.5707963 0.0 \
  --yes-execute
```

## 工控机真实联调

1. 按 rt-control 权威操作说明启动，看到 `READY: rt-control 已启动并完成 /rt/enable。`。
2. Motion 源码目录映射为 `/repo`，首次构建使用 Release。
3. 启动 Motion：

```bash
cd docker/motion
mkdir -p .workspace ../../data/docker_motion
MOTION_UID=$(id -u) MOTION_GID=$(id -g) MOTION_ROS_DOMAIN_ID=42 \
MOTION_RT_MODE=external MOTION_DRY_RUN=false \
docker compose up --build motion
```

4. 确认接口后再发任务：

```bash
ROS_DOMAIN_ID=42 ros2 action list -t | grep -E 'motion/plan|whole_body|vacuum'
```

真实执行前必须用现场精确位姿替换示例参数。首轮实现不会自动启动、使能或复位 rt-control。

也可使用仓库脚本：

```bash
tools/motion_domain_docker.sh start-mock
tools/motion_domain_docker.sh task \
  --left-box-id 1 --right-box-id 3 \
  --left-mode front --right-mode front \
  --left 0.90 0.40 1.60 3.1415926 -1.5707963 0.0 \
  --right 0.90 -0.40 1.60 3.1415926 -1.5707963 0.0 \
  --yes-execute
tools/motion_domain_docker.sh stop
```

实机模式要求 rt-control 已独立 READY，并且必须显式给出二次确认词：

```bash
MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE \
  tools/motion_domain_docker.sh start-external
```

## 开发模式

镜像只提供 ROS/MoveIt/构建环境；源码以只读方式挂载，`build/install/log` 保存在 `docker/motion/.workspace`。源码修改后重启容器会执行 `colcon --symlink-install`。也支持设置 `MOTION_REPOSITORY_URL` 和 `MOTION_REPOSITORY_REF`，在没有 `/repo` 挂载时由容器拉取源码。

默认从 DaoCloud 获取 `osrf/ros:humble-desktop-full`，以避开当前开发机失效的 Docker Hub 镜像代理。能直连 Docker Hub 的环境可设置：

```bash
MOTION_ROS_BASE_IMAGE=osrf/ros:humble-desktop-full \
  tools/motion_domain_docker.sh build
```

APT 默认使用阿里云 Ubuntu 镜像和清华 ROS 2 镜像，可分别用 `MOTION_UBUNTU_MIRROR`、`MOTION_ROS2_MIRROR` 覆盖。

`alfa_*_interfaces` 是当前权威 Markdown 的首轮 IDL 快照；统一接口仓库落地后必须整体替换，禁止与未来不同主版本混跑。`robot_interfaces/PlcIoState` 是现行 rt-control wire 的精确临时副本，仅供迁移桥反序列化。

当前容器显式使用 `allow_partial_domain_test=true`：它还没有接入 N-03 Gate、SafetyState、M-07 和版本化配置准入，不能冒充五域生产验收。现行 PLC 迁移桥只能消费 `vacuum_established` 布尔量，无法完成未来 `<= -50 kPa` 的原始压力合同验收。
