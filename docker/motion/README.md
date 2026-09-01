# Motion 域不可变 Docker 发布

本目录用于构建和部署 Motion 域生产镜像。容器通过 ROS 2 原生接口连接 Autonomy 和 RT-Control，不拥有吸附通路，也不启动、使能、复位或停止 RT-Control。

## 公共接口

| 名称 | 类型 | 方向 |
| --- | --- | --- |
| `/motion/execute_stage` | `robot_motion_interfaces/action/ExecuteMotionStage` | Autonomy → Motion |
| `/motion/readiness` | `robot_system_interfaces/msg/DomainReadiness` | Motion → Autonomy |
| `/whole_body_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion → RT-Control |
| `/joint_states` | `sensor_msgs/msg/JointState` | RT-Control → Motion |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | RT-Control → Motion |

公共接口固定来自 `robot_interfaces@92d6ff2ed0b45684d7da2170d96703ca8be569f4`。

## 镜像结构

- 强制继承 `robot/contract-runtime:interfaces-92d6ff2-20260827`。
- Builder 阶段以 Release 模式编译 Motion 包。
- Runtime 阶段只包含 `/opt/motion` 安装产物和运行依赖。
- 启动时不执行 `git clone`、`vcs import` 或 `colcon build`。
- 生产 Compose 不挂载源码、`build` 或 `install`。
- Planner 从 ROS 安装空间解析并启动，不读取宿主仓库源码。
- 使用基础镜像提供的 `/etc/robot/fastdds.xml`，不覆盖 Fast DDS 公共配置。

## 构建

先确保统一基础镜像已经导入：

```bash
docker image inspect robot/contract-runtime:interfaces-92d6ff2-20260827
```

构建 Release 镜像：

```bash
cd /path/to/alfa_robot
MOTION_VERSION=2.0.2 \
MOTION_IMAGE=alfa-motion:2.0.2 \
  ./docker/motion/build_release.sh
```

## 验证

```bash
MOTION_IMAGE=alfa-motion:2.0.2 \
  ./docker/motion/verify_release.sh
```

该脚本执行：

- `robot-runtime-doctor`；
- `contract-runtime-doctor`；
- 发布清单检查；
- 非 root 用户检查；
- 源码/构建目录泄漏检查；
- Compose 静态解析检查。

完整 Mock 集成测试使用相同镜像启动 Motion 和测试 RT-Control，检查 Planner 预热、`/motion/readiness` 和容器健康状态。

## 打包 GitHub Release

```bash
MOTION_VERSION=2.0.2 \
MOTION_IMAGE=alfa-motion:2.0.2 \
  ./docker/motion/package_release.sh
```

产物包括镜像归档、`compose.yaml`、SHA256、发布清单和部署说明。工控机不需要源码，也不会在启动时编译。

## 运行

```bash
cd /path/to/release
export MOTION_IMAGE=alfa-motion:2.0.2
docker compose up -d motion
docker compose ps
docker compose logs -f motion
```

固定运行约束：

- `ROS_DOMAIN_ID=7`；
- `network_mode: host`；
- `ipc: host`；
- 暂定 CPU `21,22`，不使用 RT-Control 的 CPU 14；
- 用户 `1000:1000`；
- `cap_drop: ALL`；
- 无硬件设备映射和实时调度 capability。
- 运行数据和日志写入 Compose 具名卷，不依赖宿主机源码目录权限。

详细部署和回滚见 [DEPLOYMENT.md](DEPLOYMENT.md)。
