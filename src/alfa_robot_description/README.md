# alfa_robot_description

Alfa 机器人描述包：URDF/xacro 模型、网格、rviz 配置与 ros2_control 接口定义，用于可视化、仿真与真实硬件控制。

![License](https://img.shields.io/badge/License-Apache-2.0-blue.svg)

---

## 主要内容概括

- **功能**：提供 Alfa 机器人的完整机器人描述（几何、惯性、关节/连杆树）、可选 ros2_control 配置（真实硬件 / Mock / Gazebo Classic / Gazebo 仿真），以及用于查看与测试的 launch 与 RViz 配置。
- **机器人结构**：底盘（base + turn + updown）→ 左右臂（leftarmbase/rightarmbase → joint1–4）+ 底盘下四轮（left/right back/forward）；关节含回转（continuous）、平移（prismatic），与 ros2_control 的 position/velocity 接口一一对应。
- **依赖**：`robot_state_publisher`、`xacro`、`rviz2`、`joint_state_publisher_gui`；若与 `controller_manager` 联合使用需配合 `alfa_robot_hardware` 或相应仿真插件。

---

## 系统版本与依赖

| 项目 | 说明 |
|------|------|
| **ROS 2** | 主要针对 **Humble**（见 `alfa_robot_description.humble.repos`） |
| **Ubuntu** | 建议 22.04（与 Humble 匹配） |
| **构建** | CMake ≥ 3.8，ament_cmake |
| **运行依赖** | `joint_state_publisher_gui`、`robot_state_publisher`、`rviz2`、`xacro` |
| **测试依赖** | `ament_cmake_pytest`、`launch_testing_ament_cmake`、`launch_testing_ros`、`liburdfdom-tools`、`xacro` |

本包**不直接依赖** `ros2_control` 或 `controller_manager`，但 URDF 内嵌的 `<ros2_control>` 会在被上层 launch 加载时由 controller_manager 解析；真实硬件需配合 `alfa_robot_hardware` 包。

---

## 接口规范

### 1. Xacro 入口与参数

**主入口**：`urdf/alfa_robot.urdf.xacro`

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `prefix` | string | `""` | 关节/连杆名前缀，多机时使用 |
| `use_mock_hardware` | bool | `false` | 使用 Mock 硬件（命令回显到状态） |
| `mock_sensor_commands` | bool | `false` | Mock 时是否启用传感器命令接口 |
| `sim_gazebo_classic` | bool | `false` | 使用 Gazebo Classic 的 ros2_control 插件 |
| `sim_gazebo` | bool | `false` | 使用 Gazebo (Ignition/Fortress) 的 gz_ros2_control 插件 |
| `simulation_controllers` | string | `""` | 仿真时控制器 yaml 路径（传给 Gazebo 插件） |
| `real_hardware_plugin` | string | `alfa_robot_hardware/AlfaRobotHW` | 真实硬件时的 SystemInterface 插件名 |

### 2. 关节与 ros2_control 接口（与 alfa_robot_hardware 一致）

- **位置控制（position）**  
  turn, updown, leftarmbase, leftjoint1–4, rightarmbase, rightjoint1–4：均提供 `position` 命令 + `position`/`velocity`/`acceleration` 状态；限位在 `alfa_robot_macro.ros2_control.xacro` 中定义（如 turn ±π，updown [-0.15, 1.0]，armbase ±0.3 等）。
- **速度控制（velocity）**  
  `left back`、`left forward`、`right back`、`right forward`：仅 `velocity` 命令 + `position`/`velocity`/`acceleration` 状态。

关节名、类型与 `alfa_robot_hardware` 中 `isCanControlledJoint` / `isVelocityControlledJoint` 等保持一致，便于同一套描述既用于可视化又用于真实/仿真控制。

### 3. Launch 参数（对外接口）

- **view_alfa_robot.launch.py**：`description_package`（默认 `alfa_robot_description`）、`prefix`（默认 `""`）。
- **alfa_robot.launch.xml**：`description_package`、`robot_name`、`prefix`、`use_mock_hardware`、`mock_sensor_commands`、`launch_rviz`；内部通过 `$(command xacro ...)` 生成 `robot_description`，并启动 `robot_state_publisher` 与可选的 `rviz2`。

### 4. 文件与资源路径

- 网格：`package://alfa_robot_description/meshes/alfa_robot/visual/<link>.STL`、`.../collision/<link>.STL`。
- RViz 配置：`rviz/alfa_robot.rviz`。
- 测试：`test/alfa_robot_test_urdf_xacro.py` 使用安装后的 `urdf/alfa_robot.urdf.xacro`（需传入默认参数），经 xacro 展开后由 `check_urdf` 校验。

---

## 架构详解

### 1. 目录与文件职责

```
alfa_robot_description/
├── config/                    # 预留配置（当前 .gitkeep）
├── launch/
│   ├── alfa_robot.launch.xml  # robot_state_publisher + 可选 rviz，xacro 带 use_mock 等参数
│   └── view_alfa_robot.launch.py  # 可视化：joint_state_publisher_gui + robot_state_publisher + rviz2
├── meshes/alfa_robot/
│   ├── collision/            # 碰撞用 STL（base, turn, updown, armbase, joint1-4, wheel 等）
│   └── visual/               # 显示用 STL（含 base_chassis, left/right back/forward 等）
├── rviz/
│   └── alfa_robot.rviz       # RViz 显示配置
├── urdf/
│   ├── alfa_robot.urdf.xacro              # 入口：world + alfa_robot macro + alfa_robot_ros2_control macro
│   ├── alfa_robot/
│   │   ├── alfa_robot_macro.xacro         # 连杆/关节树、惯性、视觉/碰撞几何
│   │   └── alfa_robot_macro.ros2_control.xacro  # ros2_control 与 Gazebo 插件块
│   └── common/
│       ├── inertials.xacro   # 惯性宏（sphere/cylinder/box 等）
│       └── materials.xacro   # 材质（black, grey, white, blue, arm_grey）
└── test/
    └── alfa_robot_test_urdf_xacro.py  # pytest：xacro 展开 + check_urdf 校验
```

### 2. 机器人拓扑（简要）

- **world** → base_link（fixed base_joint，可带 origin 偏移）  
  → turn_link（continuous turn）  
  → updown_link（prismatic updown）  
  → leftarmbase_link / rightarmbase_link（prismatic）  
  → leftjoint1_link / rightjoint1_link（prismatic）  
  → leftjoint2–4_link / rightjoint2–4_link（continuous）；  
  另：base_link → base_chassis_link（fixed）→ 四个轮子连杆（left/right back/forward，continuous）。
- 连杆命名：关节名 + `_link`（如 `turn` → `turn_link`），与 Gazebo 等兼容；网格与惯性在 macro 中按 link 定义。

### 3. ros2_control 与仿真切换

- 在 `alfa_robot_macro.ros2_control.xacro` 中按 xacro 条件选择插件：
  - `use_mock_hardware` → `mock_components/GenericSystem`
  - `sim_gazebo_classic` → `gazebo_ros2_control/GazeboSystem`
  - `sim_gazebo` → `gz_ros2_control/GazeboSimSystem`
  - 否则 → `real_hardware_plugin`（默认 `alfa_robot_hardware/AlfaRobotHW`）
- Gazebo Classic/新 Gazebo 块中通过 `simulation_controllers` 传入控制器 yaml 路径；真实硬件时不在本包内启动 controller_manager，由上层 bringup 启动。

### 4. 安装与测试

- `CMakeLists.txt` 将 `config`、`launch`、`meshes`、`rviz`、`urdf`、`test` 安装到 `share/alfa_robot_description`，并显式安装 `meshes/alfa_robot/collision`，保证 `package://` 解析到碰撞网格。
- 单元测试：`ament_add_pytest_test(test_alfa_robot_urdf_xacro, ...)` 调用 `alfa_robot_test_urdf_xacro.py`，用安装后的 xacro 生成 URDF 并用 `check_urdf` 检查合法性。

---

## 启动与调试终端命令

### 1. 编译

在工作空间根目录（如 `alfa_robot_ws`）下：

```bash
cd ~/alfa_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select alfa_robot_description
source install/setup.bash
```

如需符号链接便于修改 xacro 后不重装：

```bash
colcon build --packages-select alfa_robot_description --symlink-install
source install/setup.bash
```

### 2. 仅可视化（无 controller_manager）

**方式一：带关节滑条的 RViz（推荐调试模型）**

```bash
ros2 launch alfa_robot_description view_alfa_robot.launch.py
```

会启动：`joint_state_publisher_gui`、`robot_state_publisher`、`rviz2`（配置来自 `rviz/alfa_robot.rviz`）。可通过 GUI 拖拽关节或随机配置检查运动学与外观。

**方式二：仅发布 robot_description + RViz**

```bash
ros2 launch alfa_robot_description alfa_robot.launch.xml
```

使用默认参数时，会通过 xacro 生成描述（默认真实硬件插件名，未启动 controller_manager）；需有外部节点发布 `/joint_states` 才能看到机器人动。

带 mock 硬件描述（仅改变 URDF 内插件，仍不启动 controller_manager）：

```bash
ros2 launch alfa_robot_description alfa_robot.launch.xml use_mock_hardware:=true
```

### 3. 多机或自定义前缀

```bash
ros2 launch alfa_robot_description view_alfa_robot.launch.py prefix:=robot1_
```

关节/连杆名会带 `robot1_` 前缀；若与 controller 配合，控制器配置中的关节名也需一致。

### 4. 检查 URDF/xacro 合法性（调试）

先安装并 source，再手动执行 xacro + check_urdf：

```bash
source install/setup.bash
xacro src/alfa_robot_description/urdf/alfa_robot.urdf.xacro > /tmp/alfa.urdf
check_urdf /tmp/alfa.urdf
```

### 5. 单元测试

```bash
cd ~/alfa_robot_ws
source install/setup.bash
colcon test --packages-select alfa_robot_description
colcon test-result --verbose
```

测试会从安装目录读取 `alfa_robot.urdf.xacro`，用 xacro 展开并调用 `check_urdf`，验证描述语法与拓扑正确。

### 6. 与真实硬件 / 控制栈联合启动

本包只提供描述与可选 launch；带 controller_manager 的完整 bringup 通常由其他包（如 `alfa_robot_bringup`）提供，例如：

```bash
ros2 launch alfa_robot_bringup alfa_robot_control.launch.py
```

该 launch 会加载本包的 xacro（含 `real_hardware_plugin=alfa_robot_hardware/AlfaRobotHW`）、启动 `robot_state_publisher`、`controller_manager` 等。具体 launch 名以你工程为准。

---

## 小结

- **接口规范**：xacro 参数（prefix、use_mock_hardware、sim_gazebo、real_hardware_plugin 等）、关节与 ros2_control 的 position/velocity 定义、launch 参数与 `package://` 资源路径。
- **系统版本**：ROS 2 Humble、Ubuntu 22.04，依赖 `robot_state_publisher`、`xacro`、`rviz2`、`joint_state_publisher_gui`。
- **启动/调试**：编译 `colcon build --packages-select alfa_robot_description`；可视化 `view_alfa_robot.launch.py` 或 `alfa_robot.launch.xml`；校验 `xacro` + `check_urdf`；测试 `colcon test --packages-select alfa_robot_description`。
- **架构**：入口 xacro 组合 macro（几何+关节树）与 ros2_control macro，支持真实/Mock/Gazebo 多种插件；mesh 与 rviz 配置独立；测试保证 xacro 展开后 URDF 合法。
