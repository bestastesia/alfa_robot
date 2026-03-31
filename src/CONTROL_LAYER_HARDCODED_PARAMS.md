# 控制层硬编码清单

## 说明

- 范围只覆盖主控制链上的硬编码：
  - `alfa_robot_description` 中的 `ros2_control` 定义
  - `alfa_robot_bringup` 中的 launch、controller 配置、GUI 桥接、CAN 初始化脚本
  - `alfa_robot_hardware` 中的 `AlfaRobotHW` 和 `CanBus`
- 不包含：
  - 纯模型几何/惯性参数
  - RViz 显示配置
  - 测试文件
  - `alfa_robot_moveit_config` 的规划配置
- 本文中的“硬编码”既包括数值常量，也包括固定的 joint 名、controller 名、topic 名、总线名、协议字和行为阈值。

## 1. Bringup Launch 默认值

### 1.1 `alfa_robot_bringup/launch/alfa_robot.launch.py`

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/launch/alfa_robot.launch.py:35-126`

| 参数 | 默认值 | 备注 |
|---|---:|---|
| `runtime_config_package` | `alfa_robot_bringup` | 控制器 YAML 所在包名 |
| `controllers_file` | `alfa_robot_controllers.yaml` | 默认加载全机关节控制器配置 |
| `description_package` | `alfa_robot_description` | 默认机器人描述包 |
| `description_file` | `alfa_robot.urdf.xacro` | 默认控制用 xacro 入口 |
| `prefix` | `""` | 关节名前缀，多机部署时使用 |
| `use_mock_hardware` | `true` | 默认走 mock，不直接上实机 |
| `mock_sensor_commands` | `false` | mock 传感器命令默认关闭 |
| `robot_controller` | `all_position_controller` | 默认主控制器为整机位置控制 |
| `use_joint_gui_control` | `false` | 默认不开 GUI 滑条桥接 |
| `real_hardware_plugin` | `alfa_robot_hardware/AlfaRobotHW` | 实机硬件插件名 |
| `canopen_profile_velocity` | `50000` | CANopen 轮廓速度默认值 |
| `canopen_profile_accel` | `50000` | CANopen 轮廓加速度默认值 |

### 1.2 `alfa_robot.launch.py` 中固定行为

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/launch/alfa_robot.launch.py:143-189`，`233-318`

| 硬编码项 | 值 | 备注 |
|---|---:|---|
| `~/robot_description -> /robot_description` | 固定 remap | `controller_manager` 固定从 topic 取 URDF |
| `delay_control_node` | `2.0 s` | `robot_state_publisher` 后延迟启动 `ros2_control_node` |
| `delay_joint_state_broadcaster` | `3.0 s` | `ros2_control_node` 后延迟加载 `joint_state_broadcaster` |
| GUI remap | `joint_states -> joint_states_gui` | GUI 关节滑条输出改发到桥接节点输入 |

### 1.3 `alfa_robot_bringup/launch/alfa_robot_gui_control.launch.py`

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/launch/alfa_robot_gui_control.launch.py:43-91`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| `runtime_config_package` | `alfa_robot_bringup` | GUI 实机模式仍从本包取控制器配置 |
| `controllers_file` | `alfa_robot_controllers.yaml` | GUI 默认加载整机位置控制器 |
| `description_package` | `alfa_robot_description` | 默认描述包 |
| `description_file` | `alfa_robot.urdf.xacro` | 默认 xacro |
| `prefix` | `""` | 默认无前缀 |
| `real_hardware_plugin` | `alfa_robot_hardware/AlfaRobotHW` | 固定真实硬件插件 |
| `canopen_profile_velocity` | `50000` | GUI 模式默认 CANopen 速度 |
| `canopen_profile_accel` | `50000` | GUI 模式默认 CANopen 加速度 |
| `use_mock_hardware:=false` | 固定 | GUI 实机启动强制关闭 mock |
| `mock_sensor_commands:=false` | 固定 | GUI 实机模式固定关闭 |
| 主控制器 | `all_position_controller` | GUI 滑条默认控制整机 12 个位置关节 |
| `delay_control_node` | `2.0 s` | 启动排序延迟 |
| `delay_joint_state_broadcaster` | `3.0 s` | 启动排序延迟 |

## 2. `ros2_control` Xacro 默认值和接口定义

### 2.1 `alfa_robot_description/urdf/alfa_robot.urdf.xacro`

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro:5-14`

| 参数 | 默认值 | 备注 |
|---|---:|---|
| `prefix` | `""` | 关节/连杆名前缀 |
| `use_mock_hardware` | `false` | 描述层默认偏向真实硬件 |
| `mock_sensor_commands` | `false` | mock 传感器命令默认关闭 |
| `sim_gazebo_classic` | `false` | 默认不走 Gazebo Classic |
| `sim_gazebo` | `false` | 默认不走 GZ |
| `simulation_controllers` | `""` | 仿真控制器路径默认空 |
| `real_hardware_plugin` | `alfa_robot_hardware/AlfaRobotHW` | 默认真实硬件插件 |
| `canopen_profile_velocity` | `50000` | 默认 CANopen 速度 |
| `canopen_profile_accel` | `50000` | 默认 CANopen 加速度 |

### 2.2 `alfa_robot_macro.ros2_control.xacro` 中的插件和硬件参数

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro:17-55`

| 硬编码项 | 值 | 备注 |
|---|---:|---|
| mock 插件 | `mock_components/GenericSystem` | mock 硬件实现 |
| Gazebo Classic 插件 | `gazebo_ros2_control/GazeboSystem` | Classic 仿真插件 |
| GZ 插件 | `gz_ros2_control/GazeboSimSystem` | GZ 仿真插件 |
| 实机左臂总线 | `can0` | 左臂 RMD 总线名 |
| 实机右臂总线 | `can1` | 右臂 RMD 总线名 |
| 实机 CANopen 总线 | `can3` | CANopen 总线名 |
| 实机底座总线 | `can2` | 底座 `turn` 总线名 |
| `use_safe_shutdown` | `true` | 默认启用停机安全收拢 |
| `safe_position_turn` | `0.0` | `turn` 停机目标 |
| `safe_position_updown` | `0.0` | `updown` 停机目标 |
| `safe_position_leftarmbase` | `0.0` | 左臂基座停机目标 |
| `safe_position_leftjoint1` | `0.0` | 左臂 1 轴停机目标 |
| `safe_position_leftjoint2` | `0.0` | 左臂 2 轴停机目标 |
| `safe_position_leftjoint3` | `0.0` | 左臂 3 轴停机目标 |
| `safe_position_leftjoint4` | `0.0` | 左臂 4 轴停机目标 |
| `safe_position_rightarmbase` | `0.0` | 右臂基座停机目标 |
| `safe_position_rightjoint1` | `0.0` | 右臂 1 轴停机目标 |
| `safe_position_rightjoint2` | `0.0` | 右臂 2 轴停机目标 |
| `safe_position_rightjoint3` | `0.0` | 右臂 3 轴停机目标 |
| `safe_position_rightjoint4` | `0.0` | 右臂 4 轴停机目标 |

### 2.3 关节接口和限位

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro:56-250`

| 关节/分组 | 硬编码值 | 备注 |
|---|---:|---|
| `turn` | `position`，范围 `[-3.14159, 3.14159]` | 回转关节位置控制 |
| `updown` | `position`，范围 `[-0.15, 1.0]` | 升降直线关节 |
| `leftarmbase/rightarmbase` | `position`，范围 `[-0.3, 0.3]` | 左右臂基座直线关节 |
| `leftjoint1/rightjoint1` | `position`，范围 `[0, 0.5]` | 左右臂 1 轴直线关节 |
| `leftjoint2/3/4` | `position`，范围 `[-3.14159, 3.14159]` | 左臂旋转关节 |
| `rightjoint2/3/4` | `position`，范围 `[-3.14159, 3.14159]` | 右臂旋转关节 |
| `left back/left forward/right back/right forward` | `velocity` | 四轮只导出速度命令接口 |
| 所有关节初始值 | `0.0` | xacro 中全部状态初值默认归零 |

## 3. Controller 配置中的硬编码

### 3.1 `alfa_robot_bringup/config/alfa_robot_controllers.yaml`

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/config/alfa_robot_controllers.yaml:21-95`

| 参数/对象 | 值 | 备注 |
|---|---:|---|
| `update_rate` | `200 Hz` | `controller_manager` 更新频率 |
| controller 名 | `joint_state_broadcaster` | 状态广播器固定名称 |
| controller 名 | `base_position_controller` | 底座位置控制器 |
| controller 名 | `left_arm_position_controller` | 左臂位置控制器 |
| controller 名 | `right_arm_position_controller` | 右臂位置控制器 |
| controller 名 | `forward_velocity_controller` | 四轮速度控制器 |
| controller 名 | `all_position_controller` | GUI/调试用整机位置控制器 |
| `base_position_controller.joints` | `turn, updown` | 底座两轴顺序 |
| `left_arm_position_controller.joints` | `leftarmbase, leftjoint1, leftjoint2, leftjoint3, leftjoint4` | 左臂命令顺序 |
| `right_arm_position_controller.joints` | `rightarmbase, rightjoint1, rightjoint2, rightjoint3, rightjoint4` | 右臂命令顺序 |
| `forward_velocity_controller.joints` | `left back, left forward, right back, right forward` | 底盘四轮顺序 |
| `all_position_controller.joints` | `turn, updown, leftarmbase, leftjoint1, leftjoint2, leftjoint3, leftjoint4, rightarmbase, rightjoint1, rightjoint2, rightjoint3, rightjoint4` | 整机关节命令顺序 |
| 位置控制接口 | `position` | 上述位置控制器统一使用 |
| 速度控制接口 | `velocity` | 四轮控制器固定使用 |

### 3.2 `alfa_robot_bringup/config/alfa_robot_controllers_can.yaml`

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/config/alfa_robot_controllers_can.yaml:7-26`

| 参数/对象 | 值 | 备注 |
|---|---:|---|
| `update_rate` | `200 Hz` | 实机最小闭环更新频率 |
| controller 名 | `joint_state_broadcaster` | 状态广播器 |
| controller 名 | `forward_position_controller` | 实机 6 轴位置控制器 |
| `forward_position_controller.joints` | `leftjoint2, leftjoint3, leftjoint4, rightjoint2, rightjoint3, rightjoint4` | 实机 RMD 6 关节固定顺序 |
| `interface_name` | `position` | 6 轴位置命令接口 |

## 4. GUI 桥接和 CAN 初始化脚本

### 4.1 `joint_states_to_controller_bridge.py`

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/scripts/joint_states_to_controller_bridge.py:23-44`

| 参数 | 值 | 备注 |
|---|---:|---|
| `joint_names` | `turn, updown, leftarmbase, leftjoint1, leftjoint2, leftjoint3, leftjoint4, rightarmbase, rightjoint1, rightjoint2, rightjoint3, rightjoint4` | GUI 桥接默认按这个顺序抽取位置 |
| `command_topic` | `/all_position_controller/commands` | GUI 默认发往整机位置控制器 |
| `joint_states_topic` | `/joint_states_gui` | GUI 默认订阅的话题 |
| subscriber/pub queue | `10` | ROS 队列深度 |

### 4.2 `setup_can.sh`

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_bringup/scripts/setup_can.sh:5-18`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| 默认接口名 | `can0` | 不传参时默认初始化 `can0` |
| `txqueuelen` | `256` | 提高发送队列防止 `ENOBUFS` |
| `bitrate` | `1000000` | 默认 1 Mbps |
| 固定启动命令提示 | `ros2 launch alfa_robot_bringup alfa_robot.launch.py use_mock_hardware:=false controllers_file:=alfa_robot_controllers_can.yaml` | 脚本末尾直接写死推荐启动方式 |

## 5. `AlfaRobotHW` 中的硬编码

### 5.1 joint 分类

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/alfa_robot_hardware.cpp:35-54`

| 分类函数 | 固定成员 | 备注 |
|---|---|---|
| `isCanControlledJoint()` | `turn, leftjoint2, leftjoint3, leftjoint4, rightjoint2, rightjoint3, rightjoint4` | RMD 位置关节列表 |
| `isVelocityControlledJoint()` | `left back, left forward, right back, right forward` | 轮子速度关节列表 |
| `isCanopenControlledJoint()` | `leftarmbase, leftjoint1, rightarmbase, rightjoint1, updown` | CANopen 位置关节列表 |

### 5.2 运行时默认控制参数

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/alfa_robot_hardware.cpp:64-80`

| 参数 | 值 | 备注 |
|---|---:|---|
| `can_interface_left` | `can0` | 左臂 RMD 总线默认名 |
| `can_interface_right` | `can1` | 右臂 RMD 总线默认名 |
| `can_interface_base` | `can2` | `turn` 总线默认名 |
| `can_interface_canopen` | `can3` | CANopen 总线默认名 |
| `max_speed_dps` | `1800` | RMD 位置环最大速度 |
| `canopen_profile_velocity` | `50000` | CANopen 默认轮廓速度 |
| `canopen_profile_accel` | `50000` | CANopen 默认轮廓加速度 |
| `filter_cutoff_hz` | `50.0` | 低通滤波截止频率 |
| `low_pass_filter_active` | `false` | 运行时默认关闭低通滤波 |

### 5.3 静态映射

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/alfa_robot_hardware.cpp:214-261`

| 硬编码项 | 值 | 备注 |
|---|---|---|
| RMD 映射 | `turn -> {1, BASE}` | 底座回转轴映射到 `base` 总线 motor 1 |
| RMD 映射 | `leftjoint2 -> {1, LEFT}` | 左臂 2 轴映射 |
| RMD 映射 | `leftjoint3 -> {2, LEFT}` | 左臂 3 轴映射 |
| RMD 映射 | `leftjoint4 -> {3, LEFT}` | 左臂 4 轴映射 |
| RMD 映射 | `rightjoint2 -> {4, RIGHT}` | 右臂 2 轴映射 |
| RMD 映射 | `rightjoint3 -> {5, RIGHT}` | 右臂 3 轴映射 |
| RMD 映射 | `rightjoint4 -> {6, RIGHT}` | 右臂 4 轴映射 |
| CANopen 映射 | `updown -> 1` | 升降轴 node id |
| CANopen 映射 | `leftarmbase -> 2` | 左臂基座 node id |
| CANopen 映射 | `leftjoint1 -> 3` | 左臂 1 轴 node id |
| CANopen 映射 | `rightarmbase -> 4` | 右臂基座 node id |
| CANopen 映射 | `rightjoint1 -> 5` | 右臂 1 轴 node id |

### 5.4 激活、停机和补偿逻辑

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/alfa_robot_hardware.cpp:440-530`，`534-809`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| `turn` 零点补偿 | 激活时读取当前值并置为 `0.0` | 把当前机械位置当软件零点 |
| CANopen 减速比补偿 | `leftarmbase/rightarmbase` 读时 `/ 3.0`，写时 `* 3.0` | 3:1 传动补偿 |
| 轨迹日志服务名 | `traj_log/start_stop` | 开停服务名 |
| 轨迹日志导出服务名 | `traj_log/dump` | 导出 CSV 服务名 |
| 默认轨迹日志 motor | `6` | 默认记录 `rightjoint4` 对应电机 |
| 轨迹日志导出路径 | `/tmp/traj_log.csv` | 默认 CSV 输出位置 |
| `write()` 默认 `dt` | `0.005 s` | 控制周期缺失时的回退值 |
| 安全停机超时 | `5.0 s` | `on_deactivate()` 等待到位超时 |
| 到位容差 | `0.05` | 安全停机位置容差 |
| 安全控制周期 | `0.01 s` | 安全停机循环周期 |

## 6. `CanBusConfig` 结构体中的头文件默认值

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/include/alfa_robot_hardware/can_bus.hpp:25-42`

| 参数 | 值 | 备注 |
|---|---:|---|
| `can_interface_left/right/base/canopen` | `can0/can1/can2/can3` | 四条总线默认名 |
| `max_speed_dps` | `360` | 头文件中的 RMD 默认速度 |
| `canopen_profile_velocity` | `50000` | 头文件中的 CANopen 默认速度 |
| `canopen_profile_accel` | `50000` | 头文件中的 CANopen 默认加速度 |
| `filter_cutoff_hz` | `50.0` | 头文件中的滤波频率 |
| `low_pass_filter_active` | `true` | 头文件默认开启滤波 |

### 备注

- 这里和 `AlfaRobotHW::on_init()` 的运行时默认值不一致。
- 实际生效的是 `on_init()` 里重新赋值后的值，不是头文件初始化值。

## 7. `CanBus` 中的通信、平滑和协议硬编码

### 7.1 全局常量

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/include/alfa_robot_hardware/can_bus.hpp:75-76`，`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/can_bus.cpp:27`

| 常量 | 值 | 备注 |
|---|---:|---|
| `kGearRatio` | `36` | RMD 输出轴减速比 |
| `kCanopenPulsesPerMeter` | `1000000.0` | CANopen 位移脉冲换算比例 |
| `kCanInterFrameDelayUs` | `150 us` | CAN 帧间延时 |

### 7.2 总线和 Socket 参数

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/can_bus.cpp:47-109`，`546-597`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| 日志打印总线顺序 | `can0/can1/can2/can3` | 状态日志写死四条总线名 |
| `SO_SNDBUF` | `65536` | 发送缓冲区大小 |
| Socket 模式 | `O_NONBLOCK` | 非阻塞收发 |

### 7.3 电机使能/停机时序

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/can_bus.cpp:131-248`，`251-284`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| RMD 运行命令 | `0x88` | 使能 RMD 电机 |
| RMD 停止命令 | `0x80` | 关闭 RMD 电机 |
| RMD 使能后等待 | `10000 us` | 给驱动留切换时间 |
| CANopen NMT 启动命令 | `0x01` | 切 Operational |
| NMT 后等待 | `50000 us` | CANopen 节点启动等待 |
| CiA402 控制字序列 | `0x06 -> 0x07 -> 0x0F` | 使能顺序 |
| 每步 CiA402 等待 | `5000 us` | 每次 SDO 写后等待 |
| CANopen 模式 | `1` | Profile Position 模式 |
| CANopen 速度 index | `0x6081` | 轮廓速度对象 |
| CANopen 加速度 index | `0x6083/0x6084` | 加减速度对象 |
| CANopen 位置读取 index | `0x6064` | 实际位置对象 |

### 7.4 读写周期和滤波/平滑

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/can_bus.cpp:286-509`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| `readOnce()` CANopen 等待 | `1500 us` | 发送 SYNC 后等 PDO 返回 |
| `primeSyncCycle()` 等待 | `2000 us` | 激活阶段先打一次同步周期 |
| 低通滤波公式 | `alpha = dt / (dt + rc)` | 一阶滤波 |
| `rc` 公式 | `1 / (2π * cutoff)` | 截止频率转时间常数 |

### 7.5 RMD 协议硬编码

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/can_bus.cpp:675-770`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| RMD 发送 CAN ID | `0x140 + motor_id` | RMD 标准帧基地址 |
| RMD 读角度命令 | `0x92` | 多圈角度读取 |
| RMD 写位置命令 | `0xA4` | 多圈位置闭环 |
| 角度分辨率 | `0.01 deg/LSB` | RMD 数据换算尺度 |
| `drainRmdResponses()` 最大循环 | `10` | 每次最多收 10 帧 |
| 响应 ID 范围 | `0x140 ~ 0x146` | 识别 1~6 号电机返回 |

### 7.6 CANopen 协议硬编码

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/can_bus.cpp:776-1001`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| NMT CAN ID | `0x000` | CANopen NMT |
| SDO 下发 CAN ID | `0x600 + node_id` | SDO request |
| SDO 应答 CAN ID | `0x580 + node_id` | SDO response |
| PDO 位置下发 CAN ID | `0x200 + node_id` | Profile Position 输出 PDO |
| SYNC CAN ID | `0x080` | 同步帧 |
| PDO 接收范围 | `0x181 ~ 0x1FF` | 反馈 PDO 范围 |
| SDO read command | `0x40` | 读对象命令字 |
| SDO write command 映射 | `1B=0x2F, 2B=0x2B, 3B=0x27, 4B=0x23` | 按写入字节数选命令 |
| SDO abort 标志 | `0x80` | 错误应答识别 |
| SDO 重试次数 | `50` | 轮询上限 |
| SDO 轮询间隔 | `200 us` | 每次应答等待 |
| PDO 接收最大 drain | `32` | 每轮最多收 32 帧 PDO |
| `computeControlword()` 边沿值 | `0x002F / 0x003F` | Profile Position 新设定点翻转控制字 |
| `canopenDisableMotor()` 控制字 | `0x07 -> 0x06` | 停机顺序 |

### 7.7 轨迹日志

位置：`/home/kzoia/alfa_robot_ws/src/alfa_robot_hardware/src/can_bus.cpp:1004-1040`

| 参数/行为 | 值 | 备注 |
|---|---:|---|
| 日志预留容量 | `30000` | 约按 1 ms、30 s 预留 |
| CSV 表头 | `time_s,motor_id,p_raw,p_cmd,v_cmd,a_cmd` | 导出列固定 |

## 8. 最需要优先去硬编码的部分

如果后面准备重构，优先级最高的是下面几类：

1. joint 分类和总线/node/motor 映射
   现在散落在 xacro、controller YAML、桥接脚本、`AlfaRobotHW` 里。

2. 运行时控制参数默认值
   尤其是 `CanBusConfig` 头文件默认值和 `on_init()` 默认值不一致。

3. 启动排序和 topic/controller 名
   `2.0 s`、`3.0 s`、`/robot_description`、`/all_position_controller/commands` 都会影响外部兼容性。

4. 协议常量
   RMD/CANopen 的命令字、CAN ID、控制字、SDO 重试等都应该集中到协议配置层。
