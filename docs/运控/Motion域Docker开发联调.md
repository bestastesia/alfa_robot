# Motion 域 Docker 开发联调

## 1. 目标与边界

本版只验证一条最小闭环：外部客户端发送五域规范的 M-02/M-03，Motion 容器长驻计算并通过 M-04/M-05 控制已独立 READY 的 rt-control。

权威定义只引用：

- `/mnt/mydisk/ALFA/SevenovaHangzhou/robot_system_docs/拆垛机器人五域接口规范/02-Motion域接口规范.md`
- `/mnt/mydisk/ALFA/SevenovaHangzhou/robot_system_docs/拆垛机器人五域接口规范/04-RT-Control域接口规范.md`
- `/mnt/mydisk/ALFA/SevenovaHangzhou/robot_system_docs/rt_control/03-rt-control一键启动说明.md`

Motion 不启动、使能、复位或停止 rt-control，不直接访问 EtherCAT/CANopen 设备。

## 2. 系统形态

```text
手工任务客户端（仅测试）
        |
        | M-02 / M-03（五域合同）
        v
Motion 域服务
  - 任务校验、去重放、串行仲裁
  - 长驻 Planner 和动态 PlanningScene
  - 分段执行抓取、抽离、转运、释放、撤离
        |
        | M-04 / M-05（五域合同）
        v
现行 rt-control 迁移桥
  - /whole_body_jtc/* -> /dual_arm_jtc/*
  - /vacuum/grip -> 现行 PLC SetBool + /plc/io_state
        |
        v
rt-control Docker（独立 READY）
```

Mock 模式只把最后一层替换为本地假控制器。

## 3. 对外接口

| ID | ROS 名称 | 类型 | 生产者 → 消费者 |
|---|---|---|---|
| M-02 | `/motion/plan_and_execute_pick` | `alfa_task_interfaces/action/PlanAndExecutePick` | Autonomy/测试客户端 → Motion |
| M-03 | `/motion/plan_and_execute_place` | `alfa_task_interfaces/action/PlanAndExecutePlace` | Autonomy/测试客户端 → Motion |
| M-04 | `/whole_body_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion → RT-Control |
| M-05 | `/vacuum/grip` | `alfa_control_interfaces/action/VacuumGrip` | Motion → RT-Control |
| M-06 | `/motion/readiness` | `alfa_system_interfaces/msg/DomainReadiness` | Motion → Autonomy/观测工具 |

M-02 请求的核心数据：

```text
context: request_id + task_id + sequence_id
targets: 恰好两个，顺序必须 LEFT、RIGHT
  box_id + row + column + arm_id + suction_mode
  refined_geometry:
    body_pose(base_link, 带时间戳)
    suction_surface_pose(base_link, 带时间戳)
    size_m
motion_profile_id + grip_profile_id
expected_map_version + max_pose_age
expected_gate_owner + expected_gate_token
```

M-03 只接受与上一次成功 M-02 同 `task_id + sequence_id + 箱体集合 + 顺序` 的请求，放置位由 Motion 版本化配置决定，上层不传坐标。

M-04 每个 Goal 必须包含完整 14 轴，禁止 partial goal。M-02/M-03 同一时刻只执行一个，失败或通信未知时不自动重放。

## 4. 现行 rt-control 适配

工控机已部署 wire 与五域目标 wire 尚不一致，因此容器内暂时启动 `current_rt_control_adapter`：

| 五域目标 | 现行 rt-control |
|---|---|
| `/whole_body_jtc/follow_joint_trajectory` | `/dual_arm_jtc/follow_joint_trajectory` |
| `/vacuum/grip` | `/plc/left_solenoid`、`/plc/right_solenoid`、`/plc/vacuum_pump` |
| M-05 真空验证 | `/plc/io_state` 的 `*_vacuum_established` |

该桥只是迁移层。它无法代替规范要求的新鲜原始表压 `<= -50 kPa` 验证，不能作为五域生产验收证据。

## 5. Docker 开发模式

- 基础镜像只含 ROS 2 Humble、MoveIt 和构建依赖。
- 源码只读挂载到 `/repo`。
- `build/install/log` 放在 `docker/motion/.workspace`，不污染源码树。
- 容器启动时使用 Release `-O3` 增量构建，适合频繁修改源码。
- `network_mode: host`，实机使用 `ROS_DOMAIN_ID=42`。
- Fast DDS 仅使用 UDPv4，不使用 root 容器与宿主普通用户之间曾出现权限不一致的 SHM。

详细命令见 `docker/motion/README.md`。

## 6. 启动与联调

本机 Mock：

```bash
tools/motion_domain_docker.sh build
tools/motion_domain_docker.sh start-mock
tools/motion_domain_docker.sh status
tools/motion_domain_docker.sh task \
  --left-box-id 1 --right-box-id 3 \
  --left-mode front --right-mode front \
  --left 0.90 0.40 1.60 3.1415926 -1.5707963 0.0 \
  --right 0.90 -0.40 1.60 3.1415926 -1.5707963 0.0 \
  --yes-execute
tools/motion_domain_docker.sh stop
```

工控机实机：

```bash
cd ~/rt-control-current
./tools/rt_control_ipc.sh
# 看到 READY 后，在 Motion 仓库执行：
MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE \
  tools/motion_domain_docker.sh start-external
```

然后使用额外终端运行测试客户端。它可以传入 A1..B5 仅作测试夹具，也可直接传入左右 `base_link` 下的 6D 吸附位姿。任务编号不会进入 Motion 对外算法接口。

## 7. 首版未完成项

- M-01 相机视角运动。
- N-03 Gate、SafetyState、RobotModelInfo、CalibrationInfo 准入。
- M-07 底盘行驶准备状态。
- 生产级 Motion 唯一事实源与完整启动状态机。
- M-04/M-05 在通信失联、取消和不可逆边界上的全量故障注入。
- 统一 interfaces 仓库。当前 `alfa_*_interfaces` 是权威 Markdown 的首版 IDL 快照，必须在全域同批替换，禁止新旧主版本混跑。

`allow_partial_domain_test=true` 是明确的开发绕过，只保证本文所述的最小闭环，不得标记为五域 Demo 验收通过。

## 8. 已验证基线

- 容器内 Release 增量构建：13 个依赖包约 2 s。
- Planner 长驻会话启动：约 6.7 s，启动后任务复用同一会话。
- Mock 闭环：直接 6D 双箱 M-02/M-03 完整成功，覆盖接近、吸附、抽离、转运、释放和撤离。
- 线路语义：M-03 RELEASE 后左右电磁阀关闭，真空泵保持开启。
- 合同回归：20 项测试通过。

容器必须显式使用容器工作区的 `install/setup.bash`，不得二次加载只读挂载的宿主 `ros2_ws/install`，否则会混用两套 MoveIt ABI。
