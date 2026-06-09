
# 逆运动学解析解（Modified DH）

## DH 参数表

| i | a_{i-1} | α_{i-1} (°) | d_i | θ_i |
|---|---------|-------------|-----|-----|
| 1 | 0 | 0 | 114 | θ₁ |
| 2 | 0 | 90 | 0 | θ₂ |
| 3 | 400 | 0 | 0 | θ₃ |
| 4 | 300 | 0 | 165.4 | θ₄ |
| 5 | 0 | 90 | 136 | θ₅ |
| 6 | 0 | -90 | 233.5 | θ₆ |

参数名映射：`d₁=114, a₂=400, a₃=300, d₄=165.4, d₅=136, d₆=233.5`

## 符号说明

- `n, o, a` — 末端姿态矩阵的三列（旋转矩阵 R = [**n** **o** **a**]）
- `p_x, p_y, p_z` — 末端位置（即目标矩阵 T 的第四列前三行）
- `c₁ = cosθ₁`, `s₁ = sinθ₁`（其余关节同理）
- **注意**：§3.1 的 `P_{6}` 指**腕心**（= P_ee − d₆·**a**），§3.2 的 `P_{6}` 指**末端**（= P_ee）

---

## 3. 关节角公式（按求解顺序）

### 3.1 关节角 θ₁

> 此处的 P_6 为腕心位置：`P_6 = P_ee − d₆·a`

$$
\theta_1 = \arctan2\left(P_{6y}^0, P_{6x}^0\right) \pm \arccos\left(\frac{d_4}{\sqrt{(P_{6x}^0)^2 + (P_{6y}^0)^2}}\right) + \frac{\pi}{2}
$$

> `±` 产生两个候选解（左/右肩）。若分母为零则奇异。

### 3.2 关节角 θ₅

> 此处的 P_6 为末端位置：`P_6 = P_ee`

$$
\theta_5 = \pm \arccos\left(\frac{P_{6x}^0 \cdot \sin\theta_1 - P_{6y}^0 \cdot \cos\theta_1 - d_4}{d_6}\right)
$$

> `±` 产生两个候选解（腕部翻转）。当 `d6 = 0` 时需特殊处理。

### 3.3 关节角 θ₆

$$
\theta_6 = \arctan2\left(\frac{-o_x \sin\theta_1 + o_y \cos\theta_1}{\sin\theta_5},\; \frac{n_x \sin\theta_1 - n_y \cos\theta_1}{\sin\theta_5}\right)
$$

> 当 `sinθ5 = 0` 时奇异，可任意设定 θ₆（如保持上一时刻值）或从其他约束求解。

### 3.4 关节角 θ₃

$$
\theta_3 = \pm \arccos\left(\frac{(P_{4x}^1)^2 + (P_{4z}^1)^2 - a_2^2 - a_3^2}{2 a_2 a_3}\right)
$$

> `±` 产生两个候选解（肘部上下）。要求 `|右式| ≤ 1`。

### 3.5 关节角 θ₂

$$
\theta_2 = \arctan2\left((a_3 \cos\theta_3 + a_2) P_{4z}^1 - a_3 \sin\theta_3 \, P_{4x}^1,\; (a_3 \cos\theta_3 + a_2) P_{4x}^1 + a_3 \sin\theta_3 \, P_{4z}^1\right)
$$

### 3.6 关节角 θ₄

$$
\theta_4 = \arctan2\left(n_{4z}^1,\; n_{4x}^1\right) - \theta_2 - \theta_3
$$

### 3.7 补充：中间变量

$$
P_{4x}^1 = -d_6 \left( a_x c_1 + a_y s_1  \right) + d_5 \left( o_x c_1 c_6 + o_y s_1 c_6 + n_x c_1 s_6 + n_y s_1 s_6 \right) + p_x c_1 + p_y s_1
$$

$$
P_{4z}^1 = -d_6 a_z + d_5 \left( o_z c_6 + n_z s_6 \right) + p_z - d_1
$$

$$
n_{4x}^1 = -s_5 \left( a_x c_1 + a_y s_1 \right) + c_5 \left( n_x c_1 c_6 + n_y s_1 c_6 - o_x c_1 s_6 - o_y s_1 s_6 \right)
$$

$$
n_{4z}^1 = n_z c_5 c_6 - o_z c_5 s_6 - a_z s_5
$$
