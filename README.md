# robot_v7

## 1. 功能说明

本仓库为 **6-DOF 串联机械臂运动学求解与仿真验证**模块，基于 Modified DH（Craig 约定）参数模型，提供正运动学符号推导、解析逆运动学求解及 MuJoCo 物理仿真。

核心功能模块：

| 模块 | 文件 | 功能 |
| ---- | ---- | ---- |
| 正运动学 (FK) | `forward_kinematics.py` | SymPy 符号推导 6-DOF 变换矩阵，输出 LaTeX/Markdown 格式公式 |
| 逆运动学 (IK) | `ik_solver.py` | 解析法求解全部 8 组逆解（2 肩型 × 2 腕型 × 2 肘型），含位姿误差验证 |
| IK 公式推导 | `ik.md` | Modified DH 参数下 θ₁–θ₆ 完整解析公式及中间变量推导 |
| MuJoCo 仿真 | `mujoco_sim.py` | DH→MJCF 模型自动构建，正方形轨迹绘制 + 粉色拖尾可视化 |
| MATLAB 仿真 | `robot_arm_demo.m` | UR10 实体 3D 渲染，沿末端 Z 轴直线运动演示 |

## 2. 输入输出

### 2.1 `forward_kinematics.py`

| 方向 | 项目 | 说明 |
| ---- | ---- | ---- |
| 输入 | DH 参数表 | `dh_params_raw`：6 行 (a, α°, d, θ)，θ₁–θ₆ 为符号变量 |
| 输出 | `forward_kinematics_output.md` | Markdown 格式：DH 表 + 6 个独立变换矩阵 + 总变换矩阵 T₀⁷ + 末端位置 P/P/P |
| 输出 | `forward_kinematics_output.txt` | 纯文本格式：DH 表 + T₀⁷ + 旋转矩阵 R + 末端位置 + r² |
| 输出 | 控制台 | SymPy pretty-print 的 T 矩阵和位置表达式 |

### 2.2 `ik_solver.py`

| 方向 | 项目 | 说明 |
| ---- | ---- | ---- |
| 输入 | `T: np.ndarray` (4×4) | 目标末端齐次变换矩阵（位置 + 姿态） |
| 输出 | `List[np.ndarray]` | 最多 8 组解，每组为 6 元素 numpy 数组 (θ₁–θ₆, rad) |
| 外部依赖 | `ik.md` | 解析公式的数学推导依据 |

### 2.3 `mujoco_sim.py`

| 方向 | 项目 | 说明 |
| ---- | ---- | ---- |
| 输入 | DH 参数 | 硬编码在 `build_model_xml()` 中，与 `ik_solver.py` 一致 |
| 输入 | IK 目标位姿 | 正方形轨迹（XZ 平面，边长 0.12 m，中心 (0.40, 0, 0.30)） |
| 输出 | MuJoCo Viewer | 6-DOF 臂实时 3D 渲染 + 粉色轨迹拖尾 |
| 输出 | `MUJOCO_LOG.TXT` | MuJoCo 仿真运行日志（可能含不稳定警告） |

### 2.4 `robot_arm_demo.m`

| 方向 | 项目 | 说明 |
| ---- | ---- | ---- |
| 输入 | 起点位姿 + 方向 | `P_start` / `R_target` / `travel_distance` |
| 输出 | MATLAB Figure | UR10 实体 3D 渲染 + 直线路径 |

## 3. 依赖模块

### Python 依赖

| 依赖 | 版本建议 | 用途 |
| ---- | -------- | ---- |
| `numpy` | ≥1.21 | 矩阵/向量运算 |
| `sympy` | ≥1.9 | FK 符号推导、LaTeX 输出 |
| `mujoco` | ≥3.1 | 物理仿真引擎 + Viewer |
| `matplotlib` | ≥3.5 | （保留，robot_arm_demo.m 的 Python 对应） |

### MATLAB 依赖

| 依赖 | 用途 |
| ---- | ---- |
| MATLAB R2020b+ | `robot_arm_demo.m` 运行环境 |
| Robotics System Toolbox | UR10 DH 建模（如有内置模型） |

### 内部依赖

```
ik_solver.py ──── 独立模块，不依赖其他本地文件
mujoco_sim.py ─── import ik_solver (IKSolver)
forward_kinematics.py ─── 独立模块，仅依赖 sympy
robot_arm_demo.m ─── 独立脚本
```

## 4. 代码结构

```
robot_v7/
├── forward_kinematics.py           # ★ 正运动学符号推导 (SymPy, Modified DH)
├── ik_solver.py                    # ★ 解析逆运动学求解器 (IKSolver 类)
├── ik.md                           # ★ IK 解析公式数学推导文档
├── mujoco_sim.py                   # ★ MuJoCo 物理仿真 + 正方形轨迹 demo
├── robot_arm_demo.m                # MATLAB 3D 实体渲染直线运动
├── forward_kinematics_output.md    # FK 输出：Markdown 格式 (含 LaTeX 公式)
├── forward_kinematics_output.txt   # FK 输出：纯文本格式
├── MUJOCO_LOG.TXT                  # MuJoCo 仿真运行日志
└── __pycache__/                    # Python 字节码缓存
```

## 5. 编译方式

本项目为纯 Python/MATLAB 脚本，无需编译。如需在 ROS 2 工作空间中使用：

```bash
# 可选：作为 Python 包安装依赖
pip install numpy sympy mujoco matplotlib

# 验证环境
python -c "import mujoco; print(mujoco.__version__)"
```

## 6. 启动方式

### 6.1 正运动学符号计算

```bash
cd E:\qyx\robot_v7
python forward_kinematics.py
```

输出：
- 控制台打印 6 个独立变换矩阵 + 总变换矩阵 T₀⁷ + 末端位置 Px/Py/Pz
- 生成 `forward_kinematics_output.md`（Markdown + LaTeX）
- 生成 `forward_kinematics_output.txt`（纯文本）

### 6.2 逆运动学求解验证

```bash
python ik_solver.py
```

输出：
- 给定测试关节角 (0.5, 0.3, -0.8, 1.2, -0.4, 0.6) rad
- FK 计算目标位姿
- IK 求解全部 8 组解，对比位置误差 ‖Δp‖ 和姿态误差 ‖ΔR‖
- 标注与原始关节角匹配的解（`*** MATCH ***`）

### 6.3 MuJoCo 仿真

```bash
python mujoco_sim.py
```

- 自动从 DH 参数生成 MuJoCo MJCF 模型
- 预计算正方形轨迹 IK → 实时渲染
- 粉色小球留下末端轨迹拖尾（最多 300 点）
- 按 ESC 退出

## 7. 测试方式

### 7.1 IK 正确性测试

```bash
python ik_solver.py
```

验收标准：
- [ ] 至少找到 1 组解
- [ ] 存在一组解使得位置误差 ‖Δp‖ < 1e-3 m
- [ ] 存在一组解使得姿态误差 ‖ΔR‖ < 1e-3
- [ ] 至少有一组解与原始关节角差值 < 1e-4 rad（标注 `*** MATCH ***`）

### 7.2 FK 正确性测试

```bash
python forward_kinematics.py
```

验收标准：
- [ ] 控制台打印 6 个 T_{i-1}_i 矩阵，无异常退出
- [ ] `forward_kinematics_output.md` 和 `.txt` 成功生成
- [ ] 旋转矩阵 R 为正交矩阵（`R @ R.T ≈ I`）

### 7.3 MuJoCo 仿真测试

```bash
python mujoco_sim.py
```

验收标准：
- [ ] MuJoCo Viewer 正常启动，6 色机械臂可见
- [ ] 末端执行器沿正方形轨迹运动（XZ 平面）不抖动
- [ ] 粉色拖尾连续、不断裂
- [ ] 无 NaN/Inf 警告（检查 `MUJOCO_LOG.TXT`）

## 8. DH 参数表

| Joint i | a_{i-1} (mm) | α_{i-1} (°) | d_i (mm) | θ_i |
| ------- | ------------ | ----------- | -------- | --- |
| 1 | 0 | 0 | 114 | θ₁ |
| 2 | 0 | 90 | 0 | θ₂ |
| 3 | 400 | 0 | 0 | θ₃ |
| 4 | 300 | 0 | 165.4 | θ₄ |
| 5 | 0 | 90 | 136 | θ₅ |
| 6 | 0 | -90 | 233.5 | θ₆ |

> **约定**：Modified DH (Craig's Convention)
> 变换顺序：Rot_x(α_{i-1}) → Trans_x(a_{i-1}) → Rot_z(θ_i) → Trans_z(d_i)
> 单位：长度 mm（代码中转换为 m），角度 °（代码中转换为 rad）

## 9. IK 求解器输出

对于给定末端位姿，`IKSolver.solve()` 最多返回 **8 组解**（2³ = 8）：

| 分支 | 变量 | 来源 |
| ---- | ---- | ---- |
| 肩型 | θ₁ ± | `atan2 ± acos(d₄/r_xy)` |
| 腕型 | θ₅ ± | `± acos(...)` |
| 肘型 | θ₃ ± | `± acos(cos_θ₃)` |

奇异位形处理：
- `r_xy < 1e-12`（θ₁ 奇异）→ 返回空列表
- `|d₄/r_xy| > 1`（不可达）→ 跳过该解
- `sin(θ₅) ≈ 0`（腕部奇异）→ θ₆ 设为 0

## 10. 已知问题

- `MUJOCO_LOG.TXT` 记录有 QACC 不稳定警告（DOF 5, t≈0.068s），可能与初始瞬态或数值积分步长有关
- MATLAB 脚本 `robot_arm_demo.m` 中使用硬编码 UR10 参数，与 Python 模块的 DH 表可能不完全一致
- `mujoco_sim.py` 中 IK 求解使用网格搜索 (pitch/yaw) 而非直接姿态匹配，姿态精度有限
- 暂未包含动力学参数（质量/惯量），仿真中为粗略设定

## 11. 负责人

维护人：倪浩
