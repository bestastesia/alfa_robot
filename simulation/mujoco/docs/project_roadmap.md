# ALFA Robot V2 — Sim-to-Real 工程计划书

> 版本: v1.0 | 日期: 2026-05-07
> 目标: 构建完整的 MuJoCo 仿真验证体系，实现 Sim 与 Real 的 1:1 等价代换，为深度学习训练和 Isaac 迁移奠定基础

---

## 一、项目定位与核心目标

### 1.1 当前痛点
- ROS2 + RViz 仅能验证运动学，无法仿真真实物理交互（碰撞、摩擦、抓取、惯性）
- 无真实机器人时，算法闭环验证完全缺失
- 当前仿真参数为"能跑就行"的粗放值，与真实电机特性差距巨大

### 1.2 核心目标
| 编号 | 目标 | 验收标准 |
|------|------|----------|
| G1 | 物理参数标定至真机一致 | 各关节稳态位置误差 < 3mm，力矩误差 < 5% |
| G2 | 传感器仿真闭环 | 视觉/雷达/关节编码器输出与真机对齐，延迟差 < 10ms |
| G3 | 完整工程节拍验证 | 单箱取放全流程 sim 耗时与 real 偏差 < 10% |
| G4 | Gym 接口标准化 | 支持 `reset/step/render`，obs/action/reward 与 Isaac 同构 |
| G5 | Isaac Sim 迁移就绪 | 同一份 URDF + 参数配置可一键在 Isaac 启动 |

---

## 二、现有基础盘点

### 2.1 已完成
- [x] URDF 由 SolidWorks 导出，含完整惯性参数与 STL 网格（18 个连杆）
- [x] MuJoCo XML 机器人模型（alfa_robot.xml），双臂 + 底盘 + 吸盘 + 摄像头/雷达
- [x] 物流场景（scene.xml）：集装箱、18 个可交互货箱、传送带、围栏
- [x] Python 仿真封装层（alfa_env.py / alfa_interface.py）
- [x] 左臂 FK 求解器（符号版 + 数值版）及 FK-仿真对比验证脚本
- [x] ROS2 包结构就绪（display.launch.py / gazebo.launch.py）
- [x] 传感器框架（jointpos/framepos/touch/adhesion）

### 2.2 缺失（本计划要补齐）
- [ ] 电机真实参数（峰值功率/最大速度/力矩曲线/减速比）
- [ ] 底盘差速运动学 + 轮地接触模型
- [ ] 雷达仿真（2D/3D 点云生成）
- [ ] 视觉仿真（深度图/RGB/目标检测）
- [ ] 逆运动学求解器（左右臂）
- [ ] 完整工程节拍（取箱→搬运→放箱→复位）
- [ ] Gym/Gymnasium 标准接口
- [ ] Sim-to-Real 对齐验证套件
- [ ] Isaac Sim 导出适配

---

## 三、工程阶段划分

### Phase 1 — 物理层标定与电机模型（2 周）

**目标**: 让 MuJoCo 中的每一个关节表现与真机电机完全一致

#### 1.1 电机参数采集
需从真机或电机规格书获取以下参数，建立 `motor_spec.yaml`:

```yaml
# 示例: 底盘轮驱动电机
base_drive_motor:
  type: brushless          # 电机类型
  rated_torque: 8.5        # 额定力矩 Nm
  peak_torque: 25.0        # 峰值力矩 Nm
  max_speed: 3000          # 最大转速 RPM
  gear_ratio: 50           # 减速比
  inertia: 0.00015         # 转子惯量 kg·m²
  torque_curve:            # 力矩-速度曲线（分段线性）
    - [0, 25.0]            # [RPM, Nm]
    - [1500, 25.0]
    - [3000, 12.0]

# 示例: 左臂 joint2 旋转关节
leftjoint2_motor:
  type: harmonic_drive     # 谐波减速器
  rated_torque: 15.0
  peak_torque: 40.0
  max_speed: 60            # 输出侧 RPM
  gear_ratio: 100
  inertia: 0.00008
  backlash: 0.001          # 回差 rad
```

需要采集的电机列表:
| 关节 | 预估电机类型 | 关键参数 |
|------|-------------|---------|
| base_x / base_y | 轮驱动无刷 | 峰值力矩、轮径、减速比 |
| base_yaw | 转向舵机 | 峵值力矩、转角范围 |
| turn | 谐波减速 | 减速比、回差 |
| updown | 丝杠步进 | 导程、推力、自锁 |
| plate | 丝杠/皮带 | 导程、推力 |
| rightarmbase | 丝杠/气缸 | 行程、推力 |
| rightjoint1 | 丝杠 | 导程、推力 |
| rightjoint2-4 | 谐波减速 | 减速比、回差、峰值力矩 |
| leftarmbase | 丝杠/气缸 | 行程、推力 |
| leftjoint1 | 丝杠 | 导程、推力 |
| leftjoint2-5 | 谐波减速 | 减速比、回差、峰值力矩 |
| leftjoint6 | 丝杠/皮带 | 导程、推力 |
| 吸盘 | 真空泵 | 负压、流量、响应时间 |

#### 1.2 MuJoCo 执行器模型重构
当前所有执行器统一用 `kp=50000, forcerange=-5000~5000`，这是理想化设定，需替换为:

- **位置控制关节** (旋转臂关节): 用 `<position>` 执行器 + 力矩-速度限幅
  - `kp` = 真实 PD 增益
  - `forcerange` = [−peak_torque, +peak_torque]
  - `velocity` 限幅 = max_speed × 2π/60 / gear_ratio
- **速度控制关节** (底盘轮): 用 `<velocity>` 执行器
- **力矩-速度曲线**: 通过自定义 `ctrl` 回调或 Python 端力矩裁剪实现

#### 1.3 关节动力学精修
- `damping`: 从统一 3000 改为根据电机 + 减速器摩擦计算的真实阻尼
- `armature`: 设为转子惯量 × 减速比²（反映到关节侧的等效惯量）
- `frictionloss`: 库仑摩擦项，从规格书或实验获取
- 谐波减速器回差: 在 Python 控制层通过死区模拟

#### 1.4 底盘运动学重构
当前底盘用 `slide` 关节模拟平面运动，需改为:

```
方案 A (推荐): 差速驱动模型
  - 左右各一个 hinge 轮关节
  - 地面接触用 MuJoCo 的 friction + condim 实现
  - base_x/base_y/base_yaw 变为被动力学结果（非直接驱动）

方案 B: 阿克曼转向模型（如果底盘是舵轮结构）
  - 前轮转向角 + 后轮驱动
```

需要确认真机底盘构型后决定。底盘模型替换后需重新标定:
- 轮径 / 轮距 / 质量分布
- 滑移/打滑模型参数
- 最大线速度 / 角速度 / 加速度

---

### Phase 2 — 传感器仿真闭环（2 周）

**目标**: 仿真环境中能获取与真机等价的传感器数据流

#### 2.1 关节编码器
当前已有 `jointpos` / `jointvel` 传感器，需增加:
- **分辨率**: 真机编码器位数（如 14-bit → 0.022° 分辨率），对输出做量化
- **噪声模型**: 高斯噪声 σ = 分辨率/2
- **通信延迟**: 模拟 CAN/EtherCAT 总线延迟（典型 1-5ms）

```python
class JointEncoderSim:
    def __init__(self, resolution_bits=14, noise_std=0.001, latency_ms=2):
        self.resolution = 2 * math.pi / (2 ** resolution_bits)
        self.noise_std = noise_std
        self.latency_steps = int(latency_ms / (sim_dt * 1000))
        self.buffer = deque(maxlen=self.latency_steps + 1)

    def read(self, true_qpos):
        quantized = round(true_qpos / self.resolution) * self.resolution
        noisy = quantized + np.random.normal(0, self.noise_std)
        self.buffer.append(noisy)
        return self.buffer[0]  # 返回延迟前的值
```

#### 2.2 力/力矩传感器
在吸盘末端和底盘支撑点添加:
- `<sensor><force name="s_right_ee_force" site="right_ee"/></sensor>`
- `<sensor><torque name="s_right_ee_torque" site="right_ee"/></sensor>`
- 六轴力传感器噪声模型 + 过采样滤波

#### 2.3 2D/3D 雷达仿真
当前 `lidar_link` 仅为几何体，需实现:

```python
class LidarSim:
    def __init__(self, model, data, lidar_site="lidar_center",
                 num_beams=720, fov_deg=270, max_range=25.0):
        ...

    def scan(self):
        """基于 MuJoCo ray-casting 生成 2D 激光扫描"""
        ranges = np.full(self.num_beams, self.max_range)
        for i, angle in enumerate(self.beam_angles):
            origin = self.lidar_pos
            direction = R.from_euler('z', angle).apply(self.lidar_forward)
            # MuJoCo ray intersection
            dist = mujoco.mj_ray(self.model, self.data, origin, direction)
            if dist >= 0:
                ranges[i] = dist + np.random.normal(0, self.noise_std)
        return ranges
```

关键参数需标定:
- 真机雷达型号 → 角分辨率 / 测距范围 / 测距精度 / 扫描频率
- 点云密度、遮挡/镜面反射退化模型

#### 2.4 视觉仿真
当前 `robot_camera` 已定义但未使用，需激活:

- **RGB 渲染**: 使用 `mujoco.Renderer` 离屏渲染，匹配真机摄像头分辨率/FOV
- **深度图**: 从 renderer 获取 z-buffer，转换为真实距离
- **目标检测标注**: 同步生成 2D bounding box（基于场景中 box body 的投影）
- **摄像头噪声**: 暗角/运动模糊/曝光延迟模拟

```python
class CameraSim:
    def __init__(self, model, data, camera_name="robot_camera",
                 resolution=(640, 480)):
        self.renderer = mujoco.Renderer(model, *resolution)
        self.camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)

    def capture(self, data):
        self.renderer.update_scene(data, camera=self.camera_id)
        rgb = self.renderer.render()           # (H, W, 3) uint8
        depth = self.renderer.render(depth=True)  # 深度图
        return rgb, depth
```

#### 2.5 吸盘状态传感器
- 负压值: 对 `adhesion` 执行器状态做延迟 + 噪声
- 真空泄漏检测: 力传感器阈值判定

---

### Phase 3 — 运动学与控制层（2 周）

**目标**: 实现完整的逆运动学求解 + 轨迹规划 + 吸盘抓取控制

#### 3.1 右臂 FK/IK
当前仅完成左臂 FK，需补齐:
- 右臂 FK 求解器（5 DOF，结构更简单）
- 右臂 IK: 解析法或阻尼最小二乘数值法
- 左臂 IK: 7 DOF 冗余，用伪逆 + 零空间优化

#### 3.2 IK 求解器统一接口
```python
class ArmIKSolver:
    def solve(self, target_pose_4x4, arm="left",
              current_q=None,     # 当前关节角（用于选解）
              constraints=None):  # 关节限位 / 障碍物回避
        ...
        return q_solution, success
```

#### 3.3 轨迹规划
- 关节空间梯形速度规划（考虑各关节最大速度/加速度限制）
- 笛卡尔空间直线插值（通过 IK 实时重解）
- 避障轨迹：在 MuJoCo 中做碰撞检测，RRT*/PRM 采样

#### 3.4 底盘导航控制器
- 目标点跟踪（纯追踪 / MPC）
- 路径规划（A* / Dijkstra on occupancy grid）
- 与雷达仿真闭环，实现 SLAM 验证

#### 3.5 吸盘抓取策略
- 接近 → 对准 → 贴合 → 吸附 → 提起 → 搬运 → 放下 → 释放
- 每步的状态机 + 传感器判定（touch/force/负压）

---

### Phase 4 — 工程节拍与任务编排（2 周）

**目标**: 在仿真中完成完整的物流作业循环，并测量节拍时间

#### 4.1 单箱作业节拍
```
[定位] → [底盘导航至泊位] → [臂伸出取箱] → [吸盘吸附]
→ [缩回搬运] → [底盘导航至目标位] → [臂伸出放箱] → [吸盘释放]
→ [臂复位] → [底盘复位]
```

每个环节需记录:
| 指标 | 说明 |
|------|------|
| 耗时 (s) | 各阶段耗时 + 总耗时 |
| 峰值力矩 (Nm) | 各关节在周期内的峰值 |
| 峰值功率 (W) | 力矩 × 角速度 |
| 能耗 (J) | 功率对时间积分 |
| 关节最大速度 (rad/s) | 是否超限 |
| 碰撞次数 | 理想为 0 |
| 定位精度 (mm) | 最终位姿与目标的偏差 |

#### 4.2 多箱连续作业
- 3 行 × 2 列 × 3 层 = 18 箱全部搬运至传送带
- 优化作业顺序（贪心/TSP）
- 双臂协调：左臂和右臂并行搬运不同箱体

#### 4.3 极限工况测试
- 满载（18 箱最重负载）下底盘最大加速度
- 臂满伸状态下的关节力矩裕度
- 连续运行 100 个循环的温升/衰减模拟
- 碰撞恢复：突发碰撞后能否恢复作业

#### 4.4 节拍看板
```python
class CycleMetrics:
    """一个完整作业周期的度量记录"""
    def __init__(self):
        self.phase_times = {}     # 各阶段耗时
        self.joint_peak_torque = {}  # 各关节峰值力矩
        self.joint_peak_power = {}   # 各关节峰值功率
        self.total_energy = 0.0      # 总能耗
        self.collision_count = 0     # 碰撞次数
        self.final_pose_error = 0.0  # 终态位姿误差

    def summary(self):
        return {
            "total_time": sum(self.phase_times.values()),
            "max_torque_ratio": max(t / limit for t, limit in ...),
            "energy_per_cycle": self.total_energy,
            "safety_margin": ...,  # 最小力矩裕度
        }
```

---

### Phase 5 — Sim-to-Real 对齐验证体系（1.5 周）

**目标**: 建立系统化的验证流程，确保 sim 结果可信任地预测 real

#### 5.1 对齐验证矩阵

| 验证项 | Sim 数据源 | Real 数据源 | 对齐方法 | 容差 |
|--------|-----------|------------|---------|------|
| 关节稳态位置 | jointpos 传感器 | 编码器读数 | 相同指令 → 对比终态 | 3mm / 0.5° |
| 关节阶跃响应 | qpos(t) 时间序列 | 编码器录波 | 上升时间/超调量 | 10% |
| 关节力矩 | actuator force | 驱动器电流 × Kt | 稳态力矩对比 | 5% |
| 底盘轨迹 | base_x(t), base_y(t) | 轮式里程计 | 相同速度指令 → 对比轨迹 | 50mm |
| 吸盘保持力 | touch + force sensor | 真空表 + 测力计 | 相同负压 → 对比保持力 | 10% |
| 雷达点云 | LidarSim.scan() | 真机激光扫描 | 同场景 → 点云配准误差 | 20mm |
| 视觉目标检测 | CameraSim + detector | 真机摄像头 + detector | 相同场景 → mAP 对比 | 5% |
| 完整节拍耗时 | CycleMetrics | 真机计时 | 多循环平均耗时 | 10% |

#### 5.2 Domain Randomization 参数
为弥合 sim-real gap，在训练时对以下参数做随机化:
- 质量偏差: ±10%
- 摩擦系数: ±20%
- 关节阻尼: ±15%
- 传感器噪声等级: 基线 × [0.5, 2.0]
- 执行器延迟: [1ms, 10ms]
- 观测延迟: [1ms, 20ms]

```python
class DomainRandomizer:
    def __init__(self, model, param_ranges):
        self.model = model
        self.param_ranges = param_ranges

    def randomize(self):
        """每次 reset 时随机化物理参数"""
        for body_id, (mass_nominal, ratio) in self.param_ranges["mass"].items():
            self.model.body_mass[body_id] = mass_nominal * np.random.uniform(*ratio)
        # ... 摩擦、阻尼等类似
```

#### 5.3 系统辨识闭环
```
真机数据 → 参数辨识 → 更新 MuJoCo 参数 → Sim 验证 → 迭代
```

---

### Phase 6 — Gym 接口标准化（1 周）

**目标**: 封装为标准 Gymnasium 环境，支持 RL 训练

#### 6.1 环境接口
```python
import gymnasium as gym

class AlfaRobotEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array", "depth"]}

    def __init__(self, task="pick_and_place",
                 domain_rand=False, sim_dt=0.002, frame_skip=10):
        ...

    def reset(self, seed=None, options=None):
        ...
        return obs, info

    def step(self, action):
        ...
        return obs, reward, terminated, truncated, info

    @property
    def observation_space(self):
        # 关节角 + 关节速度 + 末端位姿 + 力传感 + 雷达 + 视觉特征
        return gym.spaces.Dict({...})

    @property
    def action_space(self):
        # 关节目标位置/速度 (连续) 或 离散指令
        return gym.spaces.Box(...)
```

#### 6.2 Reward 设计
| 任务 | Reward 组成 |
|------|-----------|
| 取箱 | 接近奖励 + 吸附成功 + 提起成功 + 碰撞惩罚 + 超时惩罚 |
| 搬运 | 路径跟踪奖励 + 到达目标 + 稳定性惩罚 |
| 放箱 | 对准奖励 + 释放成功 + 定位精度 |
| 完整节拍 | 子任务奖励加权 + 能耗惩罚 + 时间惩罚 |

#### 6.3 观测空间设计
```python
obs = {
    "joint_pos":     np.ndarray,  # (18,) 关节位置
    "joint_vel":     np.ndarray,  # (18,) 关节速度
    "ee_pose_left":  np.ndarray,  # (7,) 左臂末端 xyz + quat(w,x,y,z)
    "ee_pose_right": np.ndarray,  # (7,) 右臂末端
    "ee_force_left": np.ndarray,  # (6,) 六轴力/力矩
    "ee_force_right":np.ndarray,  # (6,)
    "suction_state": np.ndarray,  # (2,) 吸盘状态 [0=off, 1=on]
    "lidar_2d":      np.ndarray,  # (720,) 2D 激光扫描
    "camera_rgb":    np.ndarray,  # (H,W,3) 或压缩特征
    "box_positions": np.ndarray,  # (N,3) 场景中箱体位置（如果可观测）
}
```

---

### Phase 7 — Isaac Sim 迁移适配（1.5 周）

**目标**: 同一场景可在 Isaac Sim 中运行，支持 GPU 并行训练

#### 7.1 URDF/USD 转换
- 当前 URDF 已就绪，Isaac Sim 可直接加载
- 需补充: `.usd` 格式转换（Isaac 的原生格式性能更优）
- 碰撞网格简化: 当前 STL 精度过高，Isaac 需要 convex decomposition

#### 7.2 参数配置统一
```yaml
# config/robot_params.yaml — 同时服务 MuJoCo 和 Isaac
robot:
  joints:
    leftjoint2:
      type: revolute
      damping: 12.5         # 来自标定
      friction: 0.3
      actuator:
        type: position
        kp: 15000           # 来自真机PD参数
        peak_torque: 40.0
        max_velocity: 6.28  # rad/s
```

编写转换器: `yaml → MuJoCo XML` 和 `yaml → Isaac Sim config`

#### 7.3 Isaac 环境适配
- 使用 `OmniIsaacGymEnvs` 框架
- 传感器适配: Isaac 的 lidar/camera API 封装为与 MuJoCo 端同构的接口
- RL 策略在 MuJoCo 验证 → Isaac 大规模训练 → MuJoCo 精细验证 → 真机部署

#### 7.4 迁移验证
- MuJoCo 和 Isaac 对同一初始条件，轨迹偏差 < 5%
- Isaac 1000 并行环境的采样效率基准测试

---

## 四、项目目录结构规划

```
robot_v2/
├── config/
│   ├── robot_params.yaml          # 统一参数配置（电机/传感器/控制）
│   ├── motor_spec.yaml            # 电机规格书参数
│   ├── sensor_spec.yaml           # 传感器规格书参数
│   └── domain_rand.yaml           # Domain randomization 范围
│
├── mujoco/
│   ├── alfa_robot.xml             # 机器人模型（Phase 1 标定后）
│   ├── scene_logistics.xml        # 物流场景
│   ├── scene_pickplace.xml        # 取放专项场景
│   └── scene_navigation.xml       # 导航专项场景
│
├── sensors/
│   ├── joint_encoder.py           # 编码器仿真
│   ├── force_torque_sensor.py     # 六轴力传感器仿真
│   ├── lidar_sim.py               # 2D/3D 雷达仿真
│   ├── camera_sim.py              # 视觉仿真（RGB + Depth）
│   └── suction_sensor.py          # 吸盘状态传感器
│
├── kinematics/
│   ├── left_arm_fk.py             # 左臂 FK
│   ├── right_arm_fk.py            # 右臂 FK
│   ├── left_arm_ik.py             # 左臂 IK
│   ├── right_arm_ik.py            # 右臂 IK
│   └── jacobian.py                # 数值雅可比
│
├── control/
│   ├── trajectory_planner.py      # 轨迹规划
│   ├── navigation_controller.py  # 底盘导航
│   ├── arm_controller.py          # 臂运动控制
│   ├── suction_controller.py      # 吸盘抓取控制
│   └── task_sequencer.py          # 任务编排状态机
│
├── env/
│   ├── alfa_env.py                # 基础仿真环境
│   ├── alfa_gym_env.py            # Gymnasium 标准接口
│   ├── alfa_pickplace_env.py      # 取放专项环境
│   └── domain_randomizer.py       # Domain randomization
│
├── validation/
│   ├── sim2real_benchmark.py      # Sim-Real 对齐测试
│   ├── cycle_metrics.py           # 节拍度量
│   ├── motor_calibration.py       # 电机标定脚本
│   └── reports/                   # 测试报告输出
│
├── isaac/                         # Isaac Sim 适配层
│   ├── convert_urdf_to_usd.py
│   ├── alfa_isaac_env.py
│   └── config/
│
├── alfa_robot_arm/                # ROS2 包（已有）
├── alfa_robot_new_arm/            # ROS2 包（已有）
└── docs/
    └── sim2real_validation_report.md
```

---

## 五、电机参数采集清单

以下表格需逐项填入真机数据，这是 Phase 1 的前提条件:

| 关节名 | 类型 | 电机型号 | 额定力矩(Nm) | 峰值力矩(Nm) | 最大转速(RPM) | 减速比 | 转子惯量(kg·m²) | 编码器分辨率(bit) | 驱动器带宽(Hz) |
|--------|------|---------|-------------|-------------|-------------|-------|----------------|-----------------|-------------|
| base_x | slide | - | - | - | - | - | - | - | - |
| base_y | slide | - | - | - | - | - | - | - | - |
| base_yaw | hinge | - | - | - | - | - | - | - | - |
| turn | hinge | - | - | - | - | - | - | - | - |
| updown | slide | - | - | - | - | - | - | - | - |
| plate | slide | - | - | - | - | - | - | - | - |
| rightarmbase | slide | - | - | - | - | - | - | - | - |
| rightjoint1 | slide | - | - | - | - | - | - | - | - |
| rightjoint2 | hinge | - | - | - | - | - | - | - | - |
| rightjoint3 | hinge | - | - | - | - | - | - | - | - |
| rightjoint4 | hinge | - | - | - | - | - | - | - | - |
| leftarmbase | slide | - | - | - | - | - | - | - | - |
| leftjoint1 | slide | - | - | - | - | - | - | - | - |
| leftjoint2 | hinge | - | - | - | - | - | - | - | - |
| leftjoint3 | hinge | - | - | - | - | - | - | - | - |
| leftjoint4 | hinge | - | - | - | - | - | - | - | - |
| leftjoint5 | hinge | - | - | - | - | - | - | - | - |
| leftjoint6 | slide | - | - | - | - | - | - | - | - |

---

## 六、传感器规格采集清单

| 传感器 | 型号 | 关键参数 | 采样率 | 通信接口 |
|--------|------|---------|-------|---------|
| 底盘轮编码器 | - | 分辨率: ? bit | ? Hz | CAN/EtherCAT |
| 臂关节编码器 | - | 分辨率: ? bit | ? Hz | CAN/EtherCAT |
| 2D 激光雷达 | - | 角分辨率: ?°, 距离精度: ?mm, 范围: ?m | ? Hz | Ethernet |
| 3D 激光雷达(如有) | - | 线数: ?, FOV: ? | ? Hz | Ethernet |
| 摄像头 | - | 分辨率: ?×?, FOV: ?°, 帧率: ?fps | ? Hz | GigE/USB |
| 六轴力传感器(臂末端) | - | 量程: Fx? Fz? Mx? | ? Hz | EtherCAT |
| 吸盘负压传感器 | - | 量程: ?kPa | ? Hz | IO |
| IMU(底盘) | - | 加速度/角速度量程 | ? Hz | SPI/CAN |

---

## 七、风险与应对

| 风险 | 影响 | 应对 |
|------|------|------|
| 真机电机参数无法获取 | Phase 1 阻塞 | 先用同类型电机典型值，标注为"估算"，后续替换 |
| MuJoCo 雷达 ray-casting 性能不足 | 实时率下降 | 降采样 / 多线程 ray / 只在需要时扫描 |
| 谐波减速器回差在 MuJoCo 中无法原生建模 | 关节精度 sim-real gap | Python 层做死区补偿，或用 tendon + 自定义约束 |
| Isaac Sim URDF 兼容性问题 | Phase 7 延期 | 提前做小规模 POC 验证，保留 USD 手动调整路径 |
| 底盘构型不确定（差速 vs 舵轮） | Phase 1 底盘模型阻塞 | 优先确认真机底盘构型，两种方案代码并行准备 |

---

## 八、时间线总览

```
Week 1-2  │ Phase 1: 电机参数采集 + MuJoCo 执行器重构 + 底盘模型
Week 3-4  │ Phase 2: 传感器仿真（编码器/力传感/雷达/视觉）
Week 5-6  │ Phase 3: IK 求解 + 轨迹规划 + 吸盘策略 + 底盘导航
Week 7-8  │ Phase 4: 工程节拍 + 多箱作业 + 极限工况测试
Week 9    │ Phase 5: Sim-to-Real 对齐验证 + Domain Randomization
Week 10   │ Phase 6: Gym 接口标准化 + Reward 设计
Week 11-12│ Phase 7: Isaac Sim 适配 + 迁移验证
```

**总计: 约 12 周（3 个月），可根据真机参数到位时间灵活调整 Phase 1 启动点**

---

## 九、里程碑交付物

| 里程碑 | 交付物 | 时间 |
|--------|-------|------|
| M1 | motor_spec.yaml 填充完毕 + 重构后 alfa_robot.xml 通过 FK 对比验证 | Week 2 |
| M2 | 传感器仿真模块全部就绪 + 单帧数据与真机对齐 | Week 4 |
| M3 | 单箱取放全流程在 sim 中闭环运行 | Week 6 |
| M4 | 18 箱连续搬运节拍报告 + 极限工况通过 | Week 8 |
| M5 | Sim-to-Real 对齐报告（所有指标在容差内） | Week 9 |
| M6 | Gym 环境可被 PPO/SAC 等标准算法训练 | Week 10 |
| M7 | Isaac Sim 同场景运行 + MuJoCo-Isaac 一致性验证 | Week 12 |
