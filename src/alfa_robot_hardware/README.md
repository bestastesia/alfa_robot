# alfa_robot_hardware

Alfa 机器人 CAN 电机 ros2_control 硬件接口插件，支持多圈角度读取与位置闭环控制。

![License](https://img.shields.io/badge/License-Apache-2.0-blue.svg)

---

## 主要内容概括

- **功能**：为 Alfa 机器人的 6 个 CAN 位置关节（`leftjoint2`–`leftjoint4`、`rightjoint2`–`rightjoint4`）提供 `hardware_interface::SystemInterface` 实现，通过 SocketCAN 与电机通信。
- **协议**：0x92 多圈角度读取、0xA4 多圈位置闭环控制；0x88 电机运行、0x80 电机关闭；减速比 1:36，角度分辨率 0.01°/LSB（电机侧）。
- **占位关节**：其余关节（turn、updown、armbase、joint1、轮子等）当前为占位实现，待实际硬件接入后替换。
- **依赖**：ROS 2、`hardware_interface`、`pluginlib`、`rclcpp`/`rclcpp_lifecycle`，以及 Linux SocketCAN（`can0` 等）。

---

## 系统版本与依赖

| 项目 | 说明 |
|------|------|
| **ROS 2** | 主要针对 **Humble**（见 `alfa_robot_hardware.humble.repos`），理论上兼容同代及相近版本 |
| **Ubuntu** | 建议 22.04（与 Humble 及 SocketCAN 头文件匹配） |
| **构建** | CMake ≥ 3.8，C++14 |
| **依赖包** | `ament_cmake`、`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle` |
| **系统** | Linux，内核需支持 SocketCAN（`<linux/can.h>`、`<linux/can/raw.h>`） |

---

## 接口规范

### 1. 插件标识（pluginlib）

- **库名**：`alfa_robot_hardware`
- **类名**：`alfa_robot_hardware/AlfaRobotHW`
- **基类**：`hardware_interface::SystemInterface`

在 URDF / ros2_control 中配置示例：

```xml
<ros2_control name="AlfaRobotHW" type="system">
  <hardware>
    <plugin>alfa_robot_hardware/AlfaRobotHW</plugin>
    <param name="can_interface">can0</param>
    <param name="max_speed_dps">360</param>
  </hardware>
  <!-- 关节定义见下文 -->
</ros2_control>
```

### 2. 硬件参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `can_interface` | string | `can0` | SocketCAN 网络接口名 |
| `max_speed_dps` | uint16 | `360` | 0xA4 位置控制时电机最大速度（度/秒，1 dps/LSB） |

### 3. 关节与接口类型

**CAN 位置关节（6 个，电机 ID 1–6 顺序映射）**

- 关节名：`leftjoint2`、`leftjoint3`、`leftjoint4`、`rightjoint2`、`rightjoint3`、`rightjoint4`
- **状态接口**：`position`、`velocity`、`acceleration`（弧度、弧度/秒、弧度/秒²）
- **命令接口**：`position`（弧度，输出侧即减速后角度）

**速度控制关节（占位）**

- 关节名：`left back`、`left forward`、`right back`、`right forward`
- **状态接口**：`position`、`velocity`、`acceleration`
- **命令接口**：`velocity`

**其他占位关节**

- 如 turn、updown、leftarmbase、rightarmbase、leftjoint1、rightjoint1 等
- **状态接口**：`position`、`velocity`、`acceleration`
- **命令接口**：`position`（当前为命令回显占位）

### 4. CAN 协议摘要

| 功能 | 命令字节 | 方向 | 说明 |
|------|----------|------|------|
| 多圈角度 | 0x92 | 请求→回复 | DATA[1–7] 电机角度，7 字节小端，0.01°/LSB，经减速比换算为输出侧弧度 |
| 多圈位置控制 | 0xA4 | 下发 | DATA[1–2] maxSpeed(dps)，DATA[3–6] angleControl(0.01°/LSB 电机侧) |
| 电机运行 | 0x88 | 下发 | 在 `on_activate` 时发送 |
| 电机关闭 | 0x80 | 下发 | 在 `on_deactivate` 时发送 |

- CAN ID：发送 `0x140 + motor_id`（motor_id 1–6）；接收回复使用相同 ID 段（如 0x140–0x146）。
- 帧格式：8 字节，DATA[0] 为命令字节，其余为数据。

---

## 架构详解

### 1. 类与插件

- **类**：`alfa_robot_hardware::AlfaRobotHW`，继承 `hardware_interface::SystemInterface`。
- **插件**：通过 `pluginlib` 导出，由 `controller_manager` / ros2_control 在加载 URDF 时动态加载，无需单独“启动节点”；本包仅提供共享库与插件描述文件 `alfa_robot_hardware.xml`。

### 2. 生命周期与数据流

```
on_init(info)           → 解析 hardware_parameters，建立 joint_name ↔ motor_id / state_index / cmd_index
    ↓
on_configure()          → 初始化 SocketCAN（can0 等），设置 SO_SNDBUF、非阻塞
    ↓
export_state_interfaces() / export_command_interfaces()  → 向 ros2_control 注册 state/command handles
    ↓
on_activate()           → 向 6 个电机发 0x88 运行，读一次 0x92，并用当前位置初始化 hw_position_commands_
    ↓
[ 控制循环 ]
    read(time, period)   → 对 6 个电机发 0x92，收回复解析为弧度，更新 hw_positions_/velocities_/accelerations_
    write(time, period)  → 将 hw_position_commands_ 转为 0xA4 帧下发
    ↓
on_deactivate()         → 向 6 个电机发 0x80，关闭 CAN socket
```

- **读**：先统一发完 0x92，再按 CAN ID 收 6 帧并解析；速度、加速度由位置差分与周期 `period` 计算。
- **写**：仅对 6 个 CAN 关节写 0xA4；若命令含 NaN/Inf，会用当前读取位置替代，避免控制器报错。
- **Franka 风格**：首次进入 `read` 后会用当前 `hw_positions_` 初始化 `hw_position_commands_`，避免激活瞬间目标突变。

### 3. CAN 与常量

- **SocketCAN**：`AF_CAN` + `SOCK_RAW` + `CAN_RAW`，绑定到 `can_interface_`；发送缓冲区 64KB，减轻 ENOBUFS。
- **帧间延时**：发送后 `usleep(150)`（约 150 μs）。
- **减速比**：`kGearRatio = 36`，输出侧角度 = 电机角度 / 36。
- **关节→电机**：固定顺序 leftjoint2→1, leftjoint3→2, … , rightjoint4→6。

### 4. 目录与构建

- `include/alfa_robot_hardware/alfa_robot_hardware.hpp`：类声明与成员。
- `src/alfa_robot_hardware.cpp`：生命周期、read/write、CAN 收发与协议解析。
- `alfa_robot_hardware.xml`：pluginlib 描述。
- `test/test_alfa_robot_hardware.cpp`：加载 2dof 示例 URDF 的 ResourceManager 测试。

---

## 启动与调试终端命令

### 1. 编译

在工作空间根目录（如 `alfa_robot_ws`）下：

```bash
cd ~/alfa_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select alfa_robot_hardware
source install/setup.bash
```

### 2. 运行前：CAN 接口（防 ENOBUFS）

**必须先执行**（或通过 udev/systemd 在启动时执行）：

```bash
sudo ip link set can0 txqueuelen 256
```

如需同时设置比特率（示例 1 Mbps）：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 3. 本包作为插件被加载

本包**没有**独立可执行节点，由搭载 ros2_control 的机器人 launch 在加载 URDF 时拉取插件。例如在机器人总 launch 中：

- 启动 `robot_state_publisher`（发布 URDF）
- 启动 `controller_manager`，其从 URDF 的 `<ros2_control>` 中读取 `<plugin>alfa_robot_hardware/AlfaRobotHW</plugin>` 并加载本库。

因此“启动”本包的典型方式即：**启动你的 Alfa 机器人 bringup/control launch 文件**（具体名称以你工程为准），例如：

```bash
ros2 launch <your_robot_package> alfa_control.launch.py
```

### 4. 单元测试（调试用）

```bash
cd ~/alfa_robot_ws
source install/setup.bash
colcon test --packages-select alfa_robot_hardware
colcon test-result --verbose
```

测试会加载包含 2dof 示例的 URDF，验证 `AlfaRobotHW` 插件能否被 ResourceManager 正确加载（当前未接真实 CAN，仅做加载与配置测试）。

### 5. 查看接口与控制器（运行时调试）

```bash
# 查看硬件接口与控制器
ros2 control list_hardware_interfaces
ros2 control list_controllers

# 若使用 joint_state_broadcaster
ros2 topic echo /joint_states
```

---

## 小结

- **接口规范**：见上文“接口规范”（插件名、硬件参数、关节名、state/command 类型、CAN 协议）。
- **系统版本**：ROS 2 Humble + Ubuntu 22.04，依赖 `hardware_interface`、`pluginlib`、`rclcpp`、SocketCAN。
- **启动/调试**：编译用 `colcon build --packages-select alfa_robot_hardware`；运行前执行 `sudo ip link set can0 txqueuelen 256`；通过机器人 launch 加载插件；测试用 `colcon test`。
- **架构**：`AlfaRobotHW` 实现 `SystemInterface`，pluginlib 导出；生命周期 on_init → on_configure → export_* → on_activate → read/write 循环 → on_deactivate；CAN 通过 SocketCAN 与 0x92/0xA4 协议驱动 6 个关节，其余关节为占位。
