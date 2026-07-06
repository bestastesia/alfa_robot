# ALFA 力学语义定义 — Pinocchio RNEA 版

## 力学核心：RNEA 递归牛顿-欧拉逆动力学

所有力矩计算基于 Pinocchio RNEA，**非手动矩阵叉乘**。

### 总力矩公式

```
τ_total = τ_gravity + τ_payload
```

其中：

- **τ_gravity** = `pin.rnea(model, data, q, v=0, a=0)`
  - RNEA 在零速度零加速度下的输出 = 各关节维持姿态所需力矩
  - **天然包含所有连杆自身重力**（含被覆盖为 5kg 的电机）
  - 无需手动计算每段臂的重力矩和杠杆

- **τ_payload** = `J_tool^T · [0, 0, -m·g, 0, 0, 0]^T`
  - 吸盘载荷通过 Jacobian 转置映射到关节力矩
  - `J_tool` = 末端 tool0 的 6×N Jacobian (LOCAL_WORLD_ALIGNED)
  - wrench 世界系：Fz = -mg（重力向下）

### 电机 inertial 覆盖

左右臂 current arm links 的 inertial 参数被覆盖为：

| 参数 | 值 |
|------|------|
| mass | 5.0 kg（可通过 `--motor-mass` 调整） |
| com | [0, 0, 0]（几何中心） |
| inertia | 0.4 × m × r² × I₃ (实心球近似, r ≈ 0.05m) |

### 杠杆效应

RNEA 天然处理杠杆计算：每个关节的力矩包含了从该关节到末端所有连杆和载荷的重力 × 力臂。这正是递归牛顿-欧拉算法的核心优势——逐层向外递推，累积每段连杆的重力效应。

## 语义力分量 (T-0025 兼容)

| 分量 | 符号 | 单位 | 定义 |
|------|------|------|------|
| 关节轴力矩 | τ_axis_Nm | Nm | RNEA 输出的关节轴力矩 |
| 关节轴反力 | F_axis_N | N | RNEA 反力在关节轴方向的投影 |
| 法向反力 | F_normal_N | N | 反力在臂平面法向(y)的投影 |
| 侧向反力 | F_side_N | N | 反力剩余分量的大小 |

### 反力来源

- `data.f[joint_id]` = RNEA 内部计算的 6D 关节反力（在 parent frame）
- `.linear` = 3D 力, `.angular` = 3D 力矩
- 投影到世界系语义轴得到各分量

## 与旧 MVP 的关键区别

| 项目 | 旧 MVP | 新 RNEA 版 |
|------|--------|-----------|
| 重力 | 忽略连杆自身重力 | RNEA 自动包含 |
| 杠杆 | 手动 `(tip-joint) × F` | RNEA 递归累积 |
| 载荷力矩 | 手动叉乘近似 | Jacobian 转置精确映射 |
| 力来源 | 仅末端吸盘 | 所有连杆 + 末端载荷 |
| 电机质量 | URDF 原始值 | 默认 5kg, com=[0,0,0] |
