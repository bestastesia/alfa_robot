# Alfa Robot 底盘与雷达导航对接交付文档

版本: 2026-04-22

本文档用于底盘驱动侧与雷达导航侧对接，目标是打通“导航规划 -> 速度指令 -> 实机底盘运动”的闭环链路。

## 1. 对接结论

底盘侧已经提供标准 ROS2 速度控制入口 `/cmd_vel`，导航侧不要直接发布 `/cmd_vel`。导航输出应发布到 `/cmd_vel_nav`，由 `twist_mux` 进行速度源仲裁后输出 `/cmd_vel` 给底盘。

推荐链路:

```text
MID360 雷达 / IMU
  -> SLAM / Localization, 例如 Fast-LIO
  -> TF: map -> odom -> base_link -> livox_frame
  -> Nav2 planner / controller
  -> /cmd_vel_nav
  -> twist_mux
  -> /cmd_vel
  -> chassis_node
  -> CAN 电机驱动
  -> 实机运动
```

## 2. 双方职责边界

### 2.1 底盘驱动侧负责

- 启动 `alfa_robot_hardware` 的 `chassis_node`。
- 订阅 `/cmd_vel`，解析 `geometry_msgs/msg/Twist`。
- 将 `linear.x` 和 `angular.z` 转换为履带左右侧速度和电机驱动命令。
- 执行底盘限速、加速度限制、指令超时保护、心跳保护、故障状态上报。
- 提供底盘使能、禁用、故障复位服务。
- 通过 `/chassis_node/status` 发布底盘诊断状态。

### 2.2 速度仲裁侧负责

- 启动 `alfa_robot_twist_mux`。
- 接收导航、遥控、急停等多路速度输入。
- 按优先级输出唯一的 `/cmd_vel` 给底盘。
- 禁止任何上层模块绕过 `twist_mux` 直接发布 `/cmd_vel`。

### 2.3 雷达导航侧负责

- 启动 MID360 雷达驱动、IMU、SLAM/Localization、Nav2。
- 提供导航所需 TF 链路，至少保证 `map -> odom -> base_link -> livox_frame` 连通。
- Nav2 控制器输出 `/cmd_vel_nav`。
- 配置 Nav2 速度、加速度、机器人 footprint、costmap、目标容差和恢复策略。
- 提供雷达到车体的外参，也就是 `base_link -> livox_frame` 静态 TF。

## 3. ROS 接口清单

### 3.1 速度控制接口

| 方向 | Topic | Type | 发布方 | 订阅方 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 导航 -> 仲裁 | `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 controller | `twist_mux` | 自主导航速度输出 |
| 遥控 -> 仲裁 | `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | teleop 节点 | `twist_mux` | 手动控制速度输出 |
| 急停 -> 仲裁 | `/cmd_vel_estop` | `geometry_msgs/msg/Twist` | estop 节点 | `twist_mux` | 急停零速输出 |
| 仲裁 -> 底盘 | `/cmd_vel` | `geometry_msgs/msg/Twist` | `twist_mux` | `chassis_node` | 底盘唯一速度入口 |

`Twist` 字段约定:

| 字段 | 含义 | 单位 | 当前底盘是否使用 |
| --- | --- | --- | --- |
| `linear.x` | 车体前进速度，正值向前 | `m/s` | 是 |
| `angular.z` | 车体偏航角速度，正值逆时针 | `rad/s` | 是 |
| `linear.y` | 横移速度 | `m/s` | 否 |
| `linear.z` | 垂直速度 | `m/s` | 否 |
| `angular.x` | 横滚角速度 | `rad/s` | 否 |
| `angular.y` | 俯仰角速度 | `rad/s` | 否 |

### 3.2 底盘服务接口

| Service | Type | 调用方 | 说明 |
| --- | --- | --- | --- |
| `/chassis_node/enable` | `std_srvs/srv/Trigger` | 操作员或 bringup | 使能底盘电机，实机运动前必须调用 |
| `/chassis_node/disable` | `std_srvs/srv/Trigger` | 操作员或安全节点 | 禁用底盘电机 |
| `/chassis_node/reset_fault` | `std_srvs/srv/Trigger` | 操作员或恢复流程 | 故障后复位，当前实现为 disable -> enable |

### 3.3 底盘状态接口

| Topic | Type | 发布方 | 说明 |
| --- | --- | --- | --- |
| `/chassis_node/status` | `diagnostic_msgs/msg/DiagnosticArray` | `chassis_node` | 底盘运行、超时、心跳、故障、电机命令诊断 |

关键诊断字段:

| Key | 含义 |
| --- | --- |
| `target_vx` / `target_wz` | 最近收到的目标速度 |
| `limited_vx` / `limited_wz` | 限速和限加速度后的速度 |
| `v_left` / `v_right` | 履带左右侧线速度 |
| `rpm_left` / `rpm_right` | 左右侧电机目标转速 |
| `cmd_left` / `cmd_right` | 下发给驱动器的命令值 |
| `enabled` | 底盘是否已使能 |
| `timeout` | `/cmd_vel` 是否超时 |
| `heartbeat_ok` | 驱动心跳是否正常 |
| `fault_a` / `fault_b` | 驱动器故障码 |
| `tx_err_count` | CAN 发送错误计数 |

## 4. 当前底盘参数

参数文件:

```text
/home/ar/alfa_robot/src/alfa_robot_hardware/config/chassis_params.yaml
```

当前关键参数:

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `track_width` | `1.07` | 左右履带中心距，单位 `m` |
| `wheel_radius` | `0.1375` | 等效轮半径，单位 `m` |
| `max_linear_velocity` | `0.8` | 底盘线速度硬限幅，单位 `m/s` |
| `max_angular_velocity` | `1.5` | 底盘角速度硬限幅，单位 `rad/s` |
| `max_linear_acceleration` | `0.3` | 线加速度限制，单位 `m/s^2` |
| `max_angular_acceleration` | `0.5` | 角加速度限制，单位 `rad/s^2` |
| `control_frequency` | `200.0` | 底盘控制循环频率，单位 `Hz` |
| `status_frequency` | `10.0` | 状态发布频率，单位 `Hz` |
| `cmd_vel_timeout` | `0.3` | `/cmd_vel` 超时时间，单位 `s` |
| `heartbeat_timeout` | `3.0` | 驱动心跳超时时间，单位 `s` |
| `can_interface` | `can2` | 底盘 CAN 口 |
| `auto_enable_on_start` | `false` | 启动后是否自动使能 |

导航侧应配置更保守的速度上限，不应依赖底盘硬限幅作为主要安全策略。首次联调建议:

| Nav2 参数类别 | 建议初始值 |
| --- | --- |
| 最大前进速度 | `0.2-0.3 m/s` |
| 最大倒车速度 | 如无倒车需求，先禁用或限制到 `0.1 m/s` |
| 最大角速度 | `0.3-0.5 rad/s` |
| 最大线加速度 | `0.15-0.25 m/s^2` |
| 最大角加速度 | `0.2-0.4 rad/s^2` |
| 控制频率 | `10-20 Hz` |

## 5. TF 与定位要求

导航闭环必须有完整 TF。建议统一使用:

```text
map
└── odom
    └── base_link
        └── livox_frame
```

各 TF 责任:

| TF | 发布方建议 | 说明 |
| --- | --- | --- |
| `map -> odom` | SLAM / localization / Nav2 localization | 全局定位漂移校正 |
| `odom -> base_link` | Fast-LIO、轮速里程计、robot_localization 三选一或融合后唯一输出 | 机器人连续局部位姿 |
| `base_link -> livox_frame` | 静态 TF 或 URDF | 雷达到车体外参 |

重要约束:

- 同一条 TF 只能有一个发布方。
- 当前底盘节点不发布 `/odom`，也不发布 `odom -> base_link`。
- 如果雷达导航侧使用 Fast-LIO 输出位姿，应明确它输出的 frame 名称，并桥接或配置为 Nav2 需要的 TF。
- 如果后续底盘侧增加轮速里程计，需要再和导航侧约定是否由 `robot_localization` 融合轮速和 LIO，避免重复发布 `odom -> base_link`。

## 6. 雷达数据约定

当前 MID360 启动文件:

```text
/home/ar/alfa_robot/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
```

当前配置:

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `xfer_format` | `1` | Livox 自定义点云消息 |
| `multi_topic` | `0` | 所有雷达共用一个 topic |
| `publish_freq` | `10.0` | 点云发布频率 |
| `frame_id` | `livox_frame` | 雷达 frame |

典型输出:

| Topic | 说明 |
| --- | --- |
| `/livox/lidar` | MID360 点云 |
| `/livox/imu` | MID360 IMU |

如果导航侧的 Nav2 costmap 直接消费 `sensor_msgs/msg/PointCloud2`，需要将 Livox 驱动改为 `xfer_format = 0` 或增加点云格式转换节点。如果 Fast-LIO 直接消费 Livox custom msg，则保持 `xfer_format = 1`。

## 7. 推荐启动顺序

### 7.1 底盘与仲裁

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ar/alfa_robot/install/setup.bash

ros2 launch alfa_robot_hardware chassis.launch.py
ros2 launch alfa_robot_twist_mux bringup.launch.py
```

实机运动前使能底盘:

```bash
ros2 service call /chassis_node/enable std_srvs/srv/Trigger {}
```

### 7.2 雷达与定位

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ar/alfa_robot/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

### 7.3 导航

由导航侧启动 Nav2 bringup。Nav2 controller 输出必须 remap 到:

```text
/cmd_vel_nav
```

如果 Nav2 默认输出 `/cmd_vel`，应在 Nav2 启动文件中 remap:

```python
remappings=[
    ('/cmd_vel', '/cmd_vel_nav'),
]
```

或在控制器节点局部 remap:

```python
remappings=[
    ('cmd_vel', '/cmd_vel_nav'),
]
```

具体 remap 写法以导航侧 launch 中的命名空间和节点配置为准，验收标准是 `ros2 topic info /cmd_vel_nav` 能看到 Nav2 controller 发布方。

## 8. 联调检查命令

### 8.1 检查 topic

```bash
ros2 topic list | grep cmd_vel
ros2 topic info /cmd_vel_nav
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once
```

期望:

- Nav2 发布 `/cmd_vel_nav`。
- `twist_mux` 订阅 `/cmd_vel_nav` 并发布 `/cmd_vel`。
- `chassis_node` 订阅 `/cmd_vel`。

### 8.2 检查底盘状态

```bash
ros2 topic echo /chassis_node/status
```

期望:

- `enabled=true`。
- `heartbeat_ok=true`。
- `timeout=false`，前提是 `/cmd_vel` 正在持续发布。
- `fault_a=0` 且 `fault_b=0`。
- `tx_err_count` 不持续增加。

### 8.3 检查 TF

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

期望:

- `base_link -> livox_frame` 可查询。
- `map -> base_link` 可查询。
- TF 树中没有同一条边重复发布。

### 8.4 手动低速验证底盘

绕过导航但不绕过底盘，用于确认底盘方向:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}"
```

停止:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

注意: 这一步只用于底盘单体测试。完整系统联调时，上层控制必须走 `/cmd_vel_nav -> twist_mux -> /cmd_vel`。

### 8.5 验证导航输出

```bash
ros2 topic hz /cmd_vel_nav
ros2 topic echo /cmd_vel_nav
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

期望:

- `/cmd_vel_nav` 频率稳定，建议 `10 Hz` 以上。
- `/cmd_vel` 与 mux 当前选中的控制源一致。
- 停止或目标到达时，最终 `/cmd_vel` 为零速。

## 9. 实机联调流程

### 9.1 阶段一: 底盘单体

1. 架空履带或确保机器人周围安全。
2. 启动 `chassis_node`。
3. 调用 `/chassis_node/enable`。
4. 低速发布 `/cmd_vel`，检查前进、后退、左转、右转方向。
5. 检查 `/chassis_node/status` 中的速度、转速、故障和心跳。
6. 调用 `/chassis_node/disable`，确认底盘停止。

通过标准:

- `linear.x > 0` 时机器人向前。
- `angular.z > 0` 时机器人按 ROS REP-103 约定逆时针旋转。
- 停止命令后机器人停止。
- 断开速度指令超过 `0.3 s` 后自动停车。

### 9.2 阶段二: 加入 twist_mux

1. 启动 `alfa_robot_twist_mux bringup.launch.py`。
2. 发布 `/cmd_vel_nav`，确认 mux 输出 `/cmd_vel`。
3. 发布 `/cmd_vel_teleop`，确认遥控优先级高于导航。
4. 触发急停，确认输出零速度并屏蔽其他控制。

通过标准:

- 导航、遥控、急停优先级符合配置。
- 没有节点绕过 mux 直接发布 `/cmd_vel`。

### 9.3 阶段三: 雷达定位

1. 启动 MID360 驱动。
2. 启动 Fast-LIO 或导航侧定位模块。
3. 检查 `/livox/lidar`、`/livox/imu`。
4. 检查 `map -> odom -> base_link -> livox_frame`。
5. 机器人静止时，定位不应明显漂移或跳变。

通过标准:

- TF 连通。
- frame 名称与 Nav2 配置一致。
- 定位输出频率稳定。

### 9.4 阶段四: Nav2 空跑

1. 底盘先不要使能，或架空履带。
2. 启动 Nav2。
3. 在 RViz 发送短距离目标点。
4. 检查 `/cmd_vel_nav` 是否输出合理速度。
5. 检查目标到达后是否输出零速。

通过标准:

- Nav2 能规划路径。
- Controller 输出连续、平滑、不超底盘限速。
- 目标到达后停止。

### 9.5 阶段五: 低速闭环实机

1. 设置 Nav2 保守限速。
2. 确认急停可用。
3. 调用 `/chassis_node/enable`。
4. 发送 0.5-1.0 m 短距离目标。
5. 观察路径跟踪、障碍物避让、停车精度。
6. 逐步扩大距离和速度。

通过标准:

- 机器人按规划方向运动。
- 定位不发散。
- 遇到障碍或急停时能停。
- 导航结束后底盘零速。

## 10. 安全要求

- 首次联调必须低速，建议 `max_vel_x <= 0.3 m/s`，`max_vel_theta <= 0.5 rad/s`。
- 必须有现场物理急停或可立即触发的软件急停。
- 实机运动前必须确认 `/chassis_node/status` 无 ERROR。
- 调用 `/chassis_node/enable` 前，确认 `/cmd_vel` 当前为零速。
- 导航侧不能直接发布 `/cmd_vel`。
- 不允许多个节点同时发布 `/cmd_vel`。
- 不允许多个节点发布同一条 TF。
- 底盘出现心跳异常、驱动故障、CAN 发送错误持续增加时，应停止联调。

## 11. 常见问题排查

### 11.1 Nav2 有规划但底盘不动

检查:

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /chassis_node/status
```

可能原因:

- Nav2 输出没有 remap 到 `/cmd_vel_nav`。
- `twist_mux` 没启动。
- `/cmd_vel_nav` 超时，mux 没有输出。
- 底盘未调用 `/chassis_node/enable`。
- 底盘心跳异常或故障，底盘强制下发零速。

### 11.2 `/cmd_vel_nav` 有输出，但 `/cmd_vel` 没输出

检查:

```bash
ros2 topic info /cmd_vel_nav
ros2 topic info /cmd_vel
ros2 topic echo /stop
```

可能原因:

- `twist_mux` remap 配置不正确。
- 急停锁 `/stop` 生效。
- 更高优先级控制源正在生效。
- `/cmd_vel_nav` 发布频率低于 mux timeout 要求。

### 11.3 底盘方向反了

现象:

- `linear.x > 0` 机器人后退。
- `angular.z > 0` 机器人顺时针。

处理:

- 先确认 Nav2 和手动测试使用的是同一 frame 约定。
- 再检查底盘参数 `left_sign`、`right_sign`。
- 不要在 Nav2 里用负速度或负角速度临时绕过，应该在底盘标定参数中修正。

### 11.4 Nav2 抖动或原地打转

检查:

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf
ros2 topic echo /tf_static
```

可能原因:

- `map -> odom` 或 `odom -> base_link` 重复发布。
- `base_link -> livox_frame` 外参错误。
- Nav2 的 `robot_base_frame`、`odom_frame`、`global_frame` 配置不一致。
- 定位频率低或时间戳异常。
- costmap 障碍层使用的点云 frame 与 TF 不连通。

### 11.5 底盘走一顿一顿

可能原因:

- `/cmd_vel_nav` 发布频率过低。
- `twist_mux` timeout 太短。
- Nav2 controller frequency 过低。
- 底盘 `cmd_vel_timeout=0.3 s` 被触发。
- 网络、DDS 或主机负载导致消息延迟。

建议:

- `/cmd_vel_nav` 保持 `10-20 Hz`。
- Nav2 controller frequency 设置为 `10-20 Hz`。
- 检查 `ros2 topic hz /cmd_vel_nav` 和 `ros2 topic hz /cmd_vel`。

## 12. 导航侧需要提供的信息

导航同事交付 Nav2 配置时，需要明确:

| 项 | 需要给底盘侧的信息 |
| --- | --- |
| Nav2 输出 topic | 是否已 remap 到 `/cmd_vel_nav` |
| 控制频率 | controller frequency |
| 速度限制 | `max_vel_x`、`max_vel_theta`、加速度限制 |
| TF 配置 | `global_frame`、`odom_frame`、`robot_base_frame` |
| 定位来源 | Fast-LIO、AMCL、SLAM Toolbox、robot_localization 等 |
| 雷达 frame | 是否为 `livox_frame` |
| 外参 | `base_link -> livox_frame` 的 xyz/rpy |
| costmap 输入 | 点云或 LaserScan topic、frame、格式 |
| 急停策略 | 是否使用 `/stop` 或 `/cmd_vel_estop` |

## 13. 底盘侧交付给导航侧的信息

底盘侧已提供:

| 项 | 内容 |
| --- | --- |
| 底盘速度入口 | `/cmd_vel`，类型 `geometry_msgs/msg/Twist` |
| 推荐导航入口 | `/cmd_vel_nav`，由 `twist_mux` 仲裁 |
| 最大线速度 | `0.8 m/s` |
| 最大角速度 | `1.5 rad/s` |
| 线加速度限制 | `0.3 m/s^2` |
| 角加速度限制 | `0.5 rad/s^2` |
| 指令超时 | `0.3 s` |
| 控制频率 | `200 Hz` |
| 状态 topic | `/chassis_node/status` |
| 使能服务 | `/chassis_node/enable` |
| 禁用服务 | `/chassis_node/disable` |
| 故障复位服务 | `/chassis_node/reset_fault` |

## 14. 最小验收标准

对接完成应满足:

- Nav2 只发布 `/cmd_vel_nav`，不直接发布 `/cmd_vel`。
- `twist_mux` 正常输出 `/cmd_vel`。
- `chassis_node` 收到 `/cmd_vel` 后能驱动底盘运动。
- `/chassis_node/status` 显示 `enabled=true`、`heartbeat_ok=true`、无故障。
- TF 链路 `map -> odom -> base_link -> livox_frame` 连通且无重复发布。
- RViz 发送短目标点后，机器人能低速运动到目标附近并停止。
- 急停触发后，最终 `/cmd_vel` 为零速，底盘停止。
- 停止 Nav2 或断开 `/cmd_vel_nav` 后，底盘能在超时保护内停车。

