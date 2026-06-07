# alfa_robot_v2_arm_v5

## 1. 功能说明

本仓库为 **Alfa 双臂物流机器人 v5** 仿真与控制模块，负责机械臂运动规划、导纳抓取控制、逆运动学求解及 MuJoCo 仿真验证。

核心脚本 `src/demo_pick_v5.py` 演示完整的双臂六维导纳控制抓取流程：

- **机器人初始化**：躯干（pitch / turn / updown）+ 左臂 6-DOF + 右臂 6-DOF 混合控制位姿初始化
- **逆运动学求解**：通过 `DualArmIKV5` (DLS + 数值雅可比) 分别求解左右臂上方/预抓取位姿
- **平滑路径插值**：三次样条过渡至预抓取位姿（smoothstep 插值）
- **姿态加噪模拟**：在末端姿态上叠加随机扰动，模拟视觉定位误差
- **6-DOF 导纳控制**：
  - 接近阶段（approach）：基于力/力矩雅可比沿 Z 方向逼近纸箱
  - 导纳阶段（admittance）：接触检测触发后，基于质量-阻尼模型(M/D)对力/力矩偏差进行六维柔顺控制
- **EMA 力/力矩滤波**：指数移动平均滤除高频噪声
- **数据记录与可视化**：输出力/力矩/位姿/RPY 时间序列 CSV，生成 4×2 子图 PNG/PDF

## 2. 输入输出

### 输入

| 输入项 | 来源 | 说明 |
| ------- | ---- | ---- |
| `scene_v5.xml` | 本地 MuJoCo 场景文件 | 货柜 + 纸箱 + Alfa 双臂机器人模型 |
| `alfa_robot_v5.xml` | 本地 MuJoCo 模型文件 | 机器人 URDF 的 MuJoCo 描述 |
| `DualArmIKV5` | `src/dual_arm_ik_v5.py` | 双臂逆运动学求解器 (DLS 法) |
| `AlfaRobotInterfaceV5` | `src/alfa_interface_v5.py` | 机器人底层混合控制接口 |
| 力传感器数据 | MuJoCo 接触力 (`mj_contactForce`) | 末端吸盘与纸箱的接触力/力矩 |
| 关节编码器 | MuJoCo 仿真状态 (`d.qpos`, `d.qvel`) | 各关节当前位置/速度 |

### 输出

| 输出项 | 目标 | 说明 |
| ------- | ---- | ---- |
| `output/admittance_left.csv` | 磁盘文件 | 左臂力/力矩/位姿/RPY/导纳修正量时间序列 |
| `output/admittance_right.csv` | 磁盘文件 | 右臂力/力矩/位姿/RPY/导纳修正量时间序列 |
| `output/admittance_data.png` | 磁盘文件 | 4×2 子图（力/力矩/姿态/位置）PNG |
| `output/admittance_data.pdf` | 磁盘文件 | 4×2 子图 PDF |
| MuJoCo viewer | 屏幕显示 | 实时 3D 仿真可视化窗口 |

### CSV 字段说明

| 字段 | 单位 | 说明 |
| ---- | ---- | ---- |
| `t` | s | 仿真时间 |
| `Fx, Fy, Fz` | N | 末端执行器坐标系下的接触力（原始） |
| `Tx, Ty, Tz` | Nm | 末端执行器坐标系下的接触力矩（原始） |
| `X, Y, Z` | m | 末端执行器世界坐标位置 |
| `roll, pitch, yaw` | deg | 末端执行器 RPY 欧拉角 |
| `Fx_f, Fy_f, Fz_f` | N | EMA 滤波后的接触力 |
| `Tx_f, Ty_f, Tz_f` | Nm | EMA 滤波后的接触力矩 |
| `dpX, dpY, dpZ` | m | 导纳控制位置修正量 |
| `drX, drY, drZ` | rad | 导纳控制姿态修正量 |

## 3. 依赖模块

### Python 依赖

| 依赖 | 用途 |
| ---- | ---- |
| `numpy` | 矩阵运算、雅可比求解 |
| `mujoco` | 物理仿真引擎 |
| `scipy` | 空间旋转 (`Rotation`)、逆运动学优化 |
| `matplotlib` | 数据可视化绘图 |
| `collections` | EMA 滤波滑动窗口 (`deque`) |

### 内部模块依赖

| 模块 | 文件 | 说明 |
| ---- | ---- | ---- |
| `DualArmIKV5` | `src/dual_arm_ik_v5.py` | 双臂逆运动学求解器，依赖 `DualArmFKV5` |
| `DualArmFKV5` | `src/dual_arm_fk_v5.py` | 双臂正运动学模型 |
| `AlfaRobotInterfaceV5` | `src/alfa_interface_v5.py` | 机器人混合控制底层接口（关节/驱动器映射） |

### 场景依赖

| 文件 | 说明 |
| ---- | ---- |
| `scene_v5.xml` | MuJoCo 场景定义（货柜 + 自由纸箱 + 机器人） |
| `alfa_robot_v5.xml` | 机器人 MuJoCo 模型（scene_v5.xml 中 include） |

## 4. 代码结构

```
alfa_robot_v2_arm_v5/
├── src/
│   ├── demo_pick_v5.py              # ★ 主演示脚本：双臂六维导纳控制抓取
│   ├── dual_arm_ik_v5.py            # 双臂逆运动学求解器
│   ├── dual_arm_fk_v5.py            # 双臂正运动学
│   ├── alfa_interface_v5.py         # 机器人底层控制接口
│   ├── admittance_control_v5.py     # 导纳控制模块
│   ├── left_arm_ik_v5.py            # 左臂 IK 求解器
│   ├── left_arm_dh_v5.py            # 左臂 DH 参数模型
│   ├── alfa_env_v5.py               # 训练/仿真环境封装
│   ├── demo_v5.py                   # 基础 demo
│   ├── demo_ik_v5.py                # IK 测试 demo
│   ├── demo_dh_v5.py                # DH 参数验证 demo
│   ├── demo_admittance_visual_v5.py # 导纳控制可视化 demo
│   ├── Invers__v5.py                # 逆运动学算法
│   ├── adjust_boxes.py              # 纸箱布局调整工具
│   ├── record_ft_v5.py              # 力/力矩数据录制
│   ├── test_adhesion_v5.py          # 吸附测试
│   ├── test_push_record.py          # 推动测试录制
│   ├── debug_adm_v5.py              # 导纳调试工具
│   └── visualize_collision.py       # 碰撞可视化
├── scene_v5.xml                     # MuJoCo 场景文件
├── alfa_robot_v5.xml                # 机器人 MuJoCo 模型
├── launch/                          # ROS 2 launch 文件
│   ├── display.launch.py
│   └── gazebo.launch.py
├── urdf/                            # URDF 描述文件
├── meshes/                          # STL 网格文件
│   ├── visual/
│   └── collision/
├── config/                          # 参数配置文件
├── rviz/                            # RViz 配置
├── output/                          # 输出数据目录（CSV、PNG、PDF）
├── package.xml                      # ROS 2 package 描述
└── CMakeLists.txt                   # CMake 构建文件
```

## 5. 启动方式

### 直接运行 demo_pick_v5

```bash
cd E:\qyx\alfa_robot_v2_arm_v5
python src/demo_pick_v5.py
```

脚本将自动：
1. 加载 MuJoCo 场景并启动交互式 3D 查看器
2. 执行双臂初始化 → IK 求解 → 路径插值 → 导纳抓取
3. 输出 CSV 和 PNG/PDF 到 `output/` 目录

### 关键参数调整

在 `src/demo_pick_v5.py` 顶部常量区可调整：

```python
DT = 0.002          # 仿真时间步长 (s)
F_TARGET = -5.0     # 目标接触力 (N)，负值表示向工件方向
T_TARGET = np.zeros(3)  # 目标力矩 (Nm)
ADMIT_STEPS = 8000  # 最大导纳步数
```

导纳控制参数（`Adm6D` 类）：

```python
self.M  = [0.08, 0.08, 0.08, 0.02, 0.02, 0.02]  # 虚拟质量
self.D  = [25., 25., 25., 6., 6., 6.]            # 虚拟阻尼
self.vmax = [0.005, 0.005, 0.005, 0.5, 0.5, 0.5] # 最大速度
self.cmax = [0.02, 0.02, 0.01, 0.4, 0.4, 0.4]    # 最大修正量
```

## 6. 测试方式

### ROS 2 编译测试

```bash
colcon build --packages-select alfa_robot_v2_arm_v5
```

### ROS 2 launch 测试

```bash
# 仅显示 URDF 模型
ros2 launch alfa_robot_v2_arm_v5 display.launch.py

# Gazebo 仿真（如有配置）
ros2 launch alfa_robot_v2_arm_v5 gazebo.launch.py
```

### Python 单元测试

```bash
# 测试 IK 求解
cd src && python demo_ik_v5.py

# 测试 DH 参数
cd src && python demo_dh_v5.py

# 测试导纳控制可视化
cd src && python demo_admittance_visual_v5.py
```

### 仿真回归测试

运行 `demo_pick_v5.py` 后检查：
- [ ] MuJoCo viewer 正常启动，无报错退出
- [ ] 双臂均能完成 IK 求解到达预抓取位姿
- [ ] 接触检测触发导纳控制阶段切换（控制台打印 `CONTACT -> ADMIT`）
- [ ] `output/` 目录生成 `admittance_left.csv` 和 `admittance_right.csv`
- [ ] `output/` 目录生成 `admittance_data.png` 和 `admittance_data.pdf`
- [ ] CSV 力/力矩数据在合理范围内（Fz ≈ F_TARGET）
- [ ] 图中 Fz 收敛至目标力附近（蓝色虚线 ± 波动）

## 7. 算法说明

### 6-DOF 导纳控制

```
虚拟质量-阻尼模型:
  M * dv/dt + D * v = F_err

其中:
  F_err  = F_measured - F_target   (6-DOF 力/力矩误差)
  v      = 末端速度 (3平移 + 3旋转)
  dv     = (F_err - D * v) / M * dt
  dp     = v * dt                   (位置修正量)

雅可比映射:
  dq = J^T * (J * J^T + λI)^(-1) * [dp_w, α*dr_w]
```

### EMA 滤波

```
F_filtered[t] = α * F_raw[t] + (1-α) * F_filtered[t-1]
α = 0.1
```

### 接触检测

```python
if Fmag >= 0.3 and len(F_buffer) >= 10:
    phase = 'admit'  # 从 approach 切换到 admittance
```

### 退出条件

- 连续丢失接触 (`lost > 2000` 步)
- 最大导纳步数 (`cs > 6000` 步)
- 用户关闭 MuJoCo viewer

## 8. 负责人

维护人：cxy (chen@126.com)

## 9. 已知问题

- 暂未支持严重遮挡情况下的导纳重试策略
- 纸箱位置预设于 `scene_v5.xml`，非感知模块动态生成
- 导纳参数 (M/D/vmax/cmax) 基于经验调参，未做系统参数辨识
