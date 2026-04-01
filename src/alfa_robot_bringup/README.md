# alfa_robot_bringup

Alfa 机器人启动与配置包：launch 文件、控制器 YAML、CAN 初始化脚本及关节命令桥接，用于实机/Mock/Gazebo 场景下启动 ros2_control 与控制器。

![License](https://img.shields.io/badge/License-Apache-2.0-blue.svg)

---

## 主要内容概括

- **功能**：提供 Alfa 机器人的统一启动入口（实机 CAN、Mock 硬件、Gazebo 仿真）、控制器配置（joint_state_broadcaster、forward_position_controller / joint_trajectory_controller、Gazebo 下 base/左臂/右臂/速度控制器），以及 CAN 接口初始化、GUI 滑块到控制器的桥接等脚本。
- **实机 CAN**：使用 `alfa_robot_controllers_can.yaml` 仅加载 6 个 CAN 关节（left/right joint2–4）的 forward_position_controller；启动前需执行 `setup_can.sh` 或 `sudo ip link set can0 txqueuelen 256`。
- **Mock**：默认 `use_mock_hardware:=true`，配合 `alfa_robot_controllers.yaml` 可测试整机或仿真配置；可选 `use_joint_gui_control:=true` 用滑块控制位置。
- **Gazebo**：`gazebo_alfa_robot.launch.py` 启动 GZ 仿真、spawn 机器人、按序加载 joint_state_broadcaster、base/left_arm/right_arm/forward_velocity 控制器。
- **MoveIt 实机测试**：见 `MOVEIT_REAL_HARDWARE_TEST.md`，包含 `move_group -> follow_joint_trajectory -> AlfaRobotHW` 数据链路说明与测试脚本使用方法。

---

## 系统版本与依赖

| 项目 | 说明 |
|------|------|
| **ROS 2** | **Humble**（见 `alfa_robot_bringup.humble.repos`） |
| **Ubuntu** | 建议 22.04 |
| **构建** | CMake ≥ 3.8，ament_cmake |
| **核心依赖** | `alfa_robot_description`、`controller_manager`、`robot_state_publisher`、`joint_state_broadcaster`、`forward_command_controller`、`joint_trajectory_controller`、`xacro`、`rviz2` |
| **仿真** | `gazebo_ros`、`gazebo_ros2_control`（Gazebo Classic）；`ros_gz_sim`、`gz_ros2_control`（Gazebo / GZ） |
| **测试/工具** | `ros2_controllers_test_nodes`、`joint_state_publisher_gui`、`rclpy`、`sensor_msgs`、`std_msgs` |

---

## 接口规范

### 1. alfa_robot.launch.py 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `runtime_config_package` | string | `alfa_robot_bringup` | 控制器 config 所在包名 |
| `controllers_file` | string | `alfa_robot_controllers.yaml` | 控制器 YAML 文件名；实机 CAN 用 `alfa_robot_controllers_can.yaml` |
| `description_package` | string | `alfa_robot_description` | 描述包名 |
| `description_file` | string | `alfa_robot.urdf.xacro` | URDF/xacro 文件名 |
| `prefix` | string | `""` | 关节名前缀，多机时使用 |
| `use_mock_hardware` | bool | `true` | 是否使用 Mock 硬件（命令回显到状态） |
| `mock_sensor_commands` | bool | `false` | Mock 时是否启用传感器命令接口 |
| `robot_controller` | string | `forward_position_controller` | 主控制器：`forward_position_controller` 或 `joint_trajectory_controller` |
| `use_joint_gui_control` | bool | `false` | 是否启动 joint_state_publisher_gui + 桥接节点（需 forward_position_controller） |
| `real_hardware_plugin` | string | `alfa_robot_hardware/AlfaRobotHW` | 实机时的 hardware_interface 插件名 |

### 2. 控制器配置与关节对应关系

**alfa_robot_controllers_can.yaml（实机 CAN 最小配置）**

- `joint_state_broadcaster`：发布所有关节状态到 `/joint_states`。
- `forward_position_controller`：6 个关节 `leftjoint2`、`leftjoint3`、`leftjoint4`、`rightjoint2`、`rightjoint3`、`rightjoint4`，接口 `position`。  
  命令话题：`/forward_position_controller/commands`（`std_msgs/Float64MultiArray`），顺序与上述关节一致。

**alfa_robot_controllers.yaml（Mock / Gazebo 全关节）**

- `joint_state_broadcaster`。
- `base_position_controller`：`turn`、`updown`，position。
- `left_arm_position_controller`：`leftarmbase`、`leftjoint1`–`leftjoint4`，position。
- `right_arm_position_controller`：`rightarmbase`、`rightjoint1`–`rightjoint4`，position。
- `forward_velocity_controller`：`left back`、`left forward`、`right back`、`right forward`，velocity。

### 3. 话题与服务接口

- **命令**：`/forward_position_controller/commands`（Float64MultiArray），关节顺序见对应 YAML。
- **状态**：`/joint_states`（JointState），由 joint_state_broadcaster 发布。
- **robot_description**：由 robot_state_publisher 发布；controller_manager 通过 `~/robot_description` 重映射到 `/robot_description` 获取 URDF。
- **控制器**：通过 `ros2 control load_controller` / `switch_controllers` 等与 `/controller_manager` 交互。

### 4. 脚本接口

- **joint_states_to_controller_bridge.py**  
  - 订阅：`/joint_states_gui`（JointState）。  
  - 发布：`/forward_position_controller/commands`（Float64MultiArray）。  
  - 参数：`joint_names`（默认 6 个 CAN 关节）、`command_topic`、`joint_states_topic`。  
  - 从 JointState 中按 `joint_names` 顺序提取 position，写入 Float64MultiArray 并发布。

- **setup_can.sh**  
  - 用法：`sudo ./setup_can.sh [can0]`。  
  - 行为：设置 `txqueuelen 256`；若接口未 UP 则 `ip link set <if> up type can bitrate 1000000`。

---

## 架构详解

### 1. 目录与文件职责

```
alfa_robot_bringup/
├── config/
│   ├── alfa_robot_controllers.yaml      # Mock/Gazebo：base、左臂、右臂、forward_velocity
│   ├── alfa_robot_controllers_can.yaml  # 实机 CAN：仅 joint_state_broadcaster + forward_position_controller（6 关节）
│   └── test_goal_publishers_config.yaml # 测试用：forward_position / joint_trajectory 目标与关节列表
├── launch/
│   ├── alfa_robot.launch.py             # 主启动：robot_state_publisher → controller_manager → spawners，可选 GUI 桥接
│   ├── gazebo_alfa_robot.launch.py      # GZ 仿真：空世界、spawn、按序加载 5 个控制器
│   ├── test_forward_position_controller.launch.py  # 启动 forward_position 目标发布节点
│   └── test_joint_trajectory_controller.launch.py   # 启动 joint_trajectory 目标发布节点
└── scripts/
    ├── setup_can.sh                      # CAN 接口 txqueuelen + 可选 up/bitrate
    ├── joint_states_to_controller_bridge.py  # GUI 关节状态 → forward_position_controller/commands
    └── debug_mesh_path.py                # 诊断 mesh 路径（未安装到 lib，可选）
```

### 2. alfa_robot.launch.py 启动顺序

1. **robot_state_publisher**：用 xacro 生成 URDF（传入 prefix、use_mock_hardware、mock_sensor_commands、real_hardware_plugin），发布 `/robot_description`。
2. **延迟 2 s** 后启动 **ros2_control_node**（controller_manager）：从 `/robot_description` 读取 URDF、加载硬件插件与控制器配置（由 `controllers_file` 指定）。
3. **延迟 3 s** 后 **spawn joint_state_broadcaster**。
4. **joint_state_broadcaster 退出后** 依次 spawn 主控制器（如 forward_position_controller）及可选的 inactive 控制器。
5. 若 `use_joint_gui_control:=true`：同时启动 **joint_state_publisher_gui**（发布到 `joint_states_gui`）和 **joint_states_to_controller_bridge**，将 GUI 位置转发到 forward_position_controller。

controller_manager 不直接读 launch 传入的 robot_description 参数，而是依赖 robot_state_publisher 已发布的 topic，避免弃用警告。

### 3. gazebo_alfa_robot.launch.py 启动顺序

1. 设置 **GZ_SIM_RESOURCE_PATH** 指向 description 包 share 父目录，便于 GZ 解析 mesh。
2. 用 xacro 生成 **sim_gazebo:=true**、**simulation_controllers** 为 bringup config 绝对路径的 URDF，启动 **robot_state_publisher**（use_sim_time）。
3. 启动 **gz_sim**（empty.sdf -r）、**spawn**（-topic /robot_description）、**joint_state_publisher**（source_list joint_states）、可选 **rviz2**。
4. **spawn 退出后** 依次：load joint_state_broadcaster → base_position_controller → left_arm_position_controller → right_arm_position_controller → forward_velocity_controller（均 `--set-state active`）。

控制器名与 `alfa_robot_controllers.yaml` 中定义一致；URDF 内 gz_ros2_control 插件通过 `simulation_controllers` 路径加载该 yaml。

### 4. 控制器与硬件对应

- **实机**：`use_mock_hardware:=false`、`controllers_file:=alfa_robot_controllers_can.yaml`；URDF 中加载 `alfa_robot_hardware/AlfaRobotHW`，仅 6 个 CAN 关节有真实接口，其余关节由硬件层占位。
- **Mock**：`use_mock_hardware:=true`，URDF 中为 `mock_components/GenericSystem`，所有关节命令回显到状态；可用 `alfa_robot_controllers.yaml` 测试多控制器，或仍用 `alfa_robot_controllers_can.yaml` 仅测 6 关节。
- **Gazebo**：描述中 `sim_gazebo:=true`、`gz_ros2_control/GazeboSimSystem`，由仿真提供全部关节状态与命令接口。

---

## 启动与调试终端命令

### 1. 编译与安装

```bash
cd ~/alfa_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select alfa_robot_bringup
source install/setup.bash
```

需已安装并 source `alfa_robot_description`、`alfa_robot_hardware`（实机时）。

### 2. 实机 CAN 启动（6 关节）

**必须先执行（防 ENOBUFS）：**

```bash
sudo $(ros2 pkg prefix alfa_robot_bringup)/lib/alfa_robot_bringup/setup_can.sh
# 或
sudo ip link set can0 txqueuelen 256
```

**启动机器人与控制器：**

```bash
ros2 launch alfa_robot_bringup alfa_robot.launch.py \
  use_mock_hardware:=false \
  controllers_file:=alfa_robot_controllers_can.yaml \
  robot_controller:=forward_position_controller
```

可选：用 GUI 滑块控制 6 个关节（需 forward_position_controller）：

```bash
ros2 launch alfa_robot_bringup alfa_robot.launch.py \
  use_mock_hardware:=false \
  controllers_file:=alfa_robot_controllers_can.yaml \
  robot_controller:=forward_position_controller \
  use_joint_gui_control:=true
```

### 3. Mock 启动（无硬件）

```bash
ros2 launch alfa_robot_bringup alfa_robot.launch.py
```

默认已 `use_mock_hardware:=true`、`alfa_robot_controllers.yaml`、`forward_position_controller`。若要 joint_trajectory_controller：

```bash
ros2 launch alfa_robot_bringup alfa_robot.launch.py robot_controller:=joint_trajectory_controller
```

### 4. Gazebo（GZ）仿真

```bash
ros2 launch alfa_robot_bringup gazebo_alfa_robot.launch.py
```

可选关闭 RViz：`use_rviz:=false`。控制器会在 spawn 后按序自动加载。

### 5. 测试目标发布（需先启动对应控制器）

**Forward position 周期目标（左臂 3 关节示例）：**

```bash
# 终端 1：先启动机器人（Mock 或 CAN）
ros2 launch alfa_robot_bringup alfa_robot.launch.py controllers_file:=alfa_robot_controllers_can.yaml

# 终端 2：发布目标
ros2 launch alfa_robot_bringup test_forward_position_controller.launch.py
```

**Joint trajectory 目标：**

```bash
ros2 launch alfa_robot_bringup test_joint_trajectory_controller.launch.py
```

目标与关节列表在 `config/test_goal_publishers_config.yaml` 中配置（当前示例为 leftjoint2/3/4）。

### 6. 调试与检查

```bash
# 查看硬件接口
ros2 control list_hardware_interfaces

# 查看已加载控制器
ros2 control list_controllers

# 查看关节状态
ros2 topic echo /joint_states

# 手动发位置命令（6 个值，对应 leftjoint2–4, rightjoint2–4）
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---

## 小结

- **接口规范**：launch 参数（controllers_file、use_mock_hardware、robot_controller、use_joint_gui_control 等）；控制器 YAML 与关节列表；话题 `/forward_position_controller/commands`、`/joint_states`；桥接脚本的订阅/发布与参数；setup_can.sh 用法。
- **系统版本**：ROS 2 Humble、Ubuntu 22.04；依赖 alfa_robot_description、controller_manager、forward_command_controller、joint_state_broadcaster、joint_trajectory_controller、Gazebo/GZ 相关包。
- **启动/调试**：实机 CAN 先运行 setup_can.sh，再 `alfa_robot.launch.py use_mock_hardware:=false controllers_file:=alfa_robot_controllers_can.yaml`；Mock 直接 `alfa_robot.launch.py`；仿真 `gazebo_alfa_robot.launch.py`；测试目标用 test_forward_position_controller / test_joint_trajectory_controller launch。
- **架构**：robot_state_publisher 先发布 URDF → 延迟启动 controller_manager → 依次 spawn joint_state_broadcaster 与主控制器；可选 GUI 桥接；Gazebo 为独立 launch，spawn 后链式加载 5 个控制器；两套 YAML 分别服务实机 CAN 与 Mock/Gazebo。
