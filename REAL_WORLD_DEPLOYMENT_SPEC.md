# pick_a2c_2 模型真实部署输入规范

本文档描述在真实卸货场景中使用训练好的策略模型时，视觉系统需要提供什么数据、模型需要什么输入、以及两者之间的转换关系。

---

## 1. 坐标约定

- X：容器后壁 → 开门方向（前方）
- Y：容器左右，中点 Y=0，左负右正
- Z：容器底面 Z=0，向上为正
- 原点：容器底面左下角（从开门方向看）

---

## 2. 视觉系统原始输出

每帧需要检测出容器内所有箱子，每个箱子提供以下 **6 个原始测量值**：

| 符号 | 含义 | 单位 | 来源 |
|------|------|------|------|
| `x` | 箱体中心 X 坐标 | m | 视觉 + 深度 |
| `y` | 箱体中心 Y 坐标 | m | 视觉 + 深度 |
| `z` | 箱体中心 Z 坐标（高度） | m | 视觉 + 深度 |
| `l` | 箱体 X 方向长度 | m | 视觉估计 |
| `w` | 箱体 Y 方向宽度 | m | 视觉估计 |
| `h` | 箱体 Z 方向高度 | m | 视觉估计 |

除此之外，还需通过测量获取 4 个**容器常量**（一帧内不变）：

| 符号 | 含义 | 获取方式 |
|------|------|----------|
| `L` | 容器 X 方向内部深度 | 手工测量 |
| `W` | 容器 Y 方向内部宽度 | 手工测量 |
| `H` | 容器 Z 方向内部高度 | 手工测量 |
| `b` | 标准箱参考边长 | 手工测量 |

---

## 3. 模型输入表

模型每个推理步需要以下 5 个张量：

| 输入 | 形状 | 含义 | 符号 |
|------|------|------|------|
| `s_B` | (1, N_B, D) | 所有剩余箱子的特征序列 | — |
| `s_P` | (1, N_P, D) | 可抓取箱子的特征序列 | — |
| `mask_B` | (1, N_B) | s_B 的 padding mask，True=忽略 | — |
| `mask_P` | (1, N_P) | s_P 的 padding mask，True=忽略 | — |
| `positions` | (1, N_P, 3) | 可抓取箱子的世界坐标 (x,y,z) | — |

> N_B = 总剩余箱数，N_P = 可抓取箱数，D = 16（特征维度）。单帧推理时 batch=1。

核心是 D=16 维的单箱特征。对每个箱子 i，需要计算以下 16 个值：

| 索引 | 名称 | 符号 | 含义 |
|------|------|------|------|
| 0 | x_norm | `x/L` | 前后位置归一化 |
| 1 | y_norm | `y/(W/2)` | 左右位置归一化 |
| 2 | z_norm | `z/H` | 高度归一化 |
| 3 | l_norm | `l/b` | 长度归一化 |
| 4 | w_norm | `w/b` | 宽度归一化 |
| 5 | h_norm | `h/b` | 高度归一化 |
| 6 | front_pref | `1 - x/L` | 距门口距离，越小越靠前 |
| 7 | top_clear | `c_top/H` | 顶部净空归一化 |
| 8 | front_clear | `c_front/L` | 前向退出净空归一化 |
| 9 | support | `n_sup/n_z` | 压在上方的箱子数归一化 |
| 10 | is_dropped | `0 或 1` | 是否为散落/不规则箱 |
| 11 | severity | `0~1` | 散落严重度，规则箱=0 |
| 12 | is_pickable | `0 或 1` | 是否可抓取 |
| 13 | dx_last | `(x-x_last)/L` | 距上次抓取的 X 偏移 |
| 14 | dy_last | `(y-y_last)/W` | 距上次抓取的 Y 偏移 |
| 15 | dz_last | `(z-z_last)/H` | 距上次抓取的 Z 偏移 |

---

## 4. 特征计算步骤

### 4.1 散落箱判定（索引 10）

给定一个箱子的 (x, y, z, l, w, h)，判断是否散落：

- 尺寸与标准值 b 偏差较大
- 姿态有明显旋转（偏航角 > 阈值）
- 位置不在规则网格节点上

判定结果：`is_dropped = 0（规则）或 1（散落）`。

### 4.2 散落严重度（索引 11）

仅对散落箱计算，规则箱直接填 0：

```
sev = 0.35*(1 - clip(x/L, 0, 1))
    + 0.30*(1 - clip(z/H, 0, 1))
    + 0.20*min(1, |l - b|/b)
    + 0.15*min(1, |y|/(W/2))
severity = clip(sev, 0, 1)
```

### 4.3 可抓取判定（索引 12）

箱子 i 可抓取需同时满足两个条件：

**条件 A — 顶部净空 TopClear(i)**：
对每个其他箱子 j，如果：
- z_j 底 ≥ z_i 顶 - 0.02，且
- dx < (l_i+l_j)/2 - 0.03，且
- dy < (w_i+w_j)/2 - 0.03

则 TopClear(i)=False。否则为 True。

**条件 B — 前向净空 FrontClear(i)**：
对每个其他箱子 j，如果：
- x_j 前表面 < x_i 前表面（j 比较靠后），跳过
- Y 范围不重叠，跳过
- |z_j - z_i| > max(h_i, h_j) × 0.45，跳过

否则 FrontClear(i)=False。全跳过则为 True。

**is_pickable = TopClear 且 FrontClear ? 1 : 0**

### 4.4 顶部净空值（索引 7）

```
c_top = H - z_i^top  （初始值）
对每个与 i 的 XY 投影重叠的箱子 j：
    c_top = min(c_top, max(0, z_j^bottom - z_i^top))
```

其中：
- `z_i^top = zi + hi/2`（箱 i 顶面）
- `z_j^bottom = zj - hj/2`（箱 j 底面）

### 4.5 前向净空值（索引 8）

箱子 i 前表面 `x_i^front = xi - li/2`。对每个在 i 前方（x 更小）且 YZ 范围重叠的箱子 j，其阻挡面为 `x_j^back = xj + lj/2`。

```
c_front = max(0, x_i^front - max(所有阻挡箱的 x_j^back))
```

### 4.6 支撑负载（索引 9）

箱子 j 站在箱子 i 上当且仅当：
- `|z_j^bottom - z_i^top| < max(0.04, 0.2b)`
- 且 XY 投影重叠

```
n_sup = 满足上述条件的 j 的数量
n_z = ceil(H / b)  （最大理论层数）
```

### 4.7 上次抓取偏移（索引 13-15）

第一帧（无历史）：三值均为 **0**。
后续帧：`(x_last, y_last, z_last)` 为上一帧被抓箱子的平均位置。

```
dx_last = (x - x_last) / L
dy_last = (y - y_last) / W
dz_last = (z - z_last) / H
```

---

## 5. 构建最终输入

```python
# s_B: 所有剩余箱子的特征, shape (N_B, 16)
# s_P: 可抓取箱子的特征, shape (N_P, 16)
# 加 batch 维度后输入模型

s_B_tensor     = s_B.unsqueeze(0)    # (1, N_B, 16)
s_P_tensor     = s_P.unsqueeze(0)    # (1, N_P, 16)
mask_B_tensor  = zeros(1, N_B, bool) # 全 False, 无 padding
mask_P_tensor  = zeros(1, N_P, bool)
positions_t    = s_P_tensor[:, :, :3].clone()  # 世界坐标, 用于碰撞掩码计算

idx_left, idx_right, *_ = net.sample_actions(
    s_B_tensor, s_P_tensor,
    mask_B_tensor, mask_P_tensor,
    ~mask_P_tensor,   # pickable_mask = 非 padding 的位置
    positions_t,
)

# 输出是 pickable 序列索引, 映射回全局 box_id
global_left  = pickable_list[idx_left]
global_right = pickable_list[idx_right]
```

---

## 6. 推理循环

```
while 容器非空:
    1. 视觉检测 → 所有箱子的 (x,y,z,l,w,h)
    2. 判定每个箱子的 is_dropped
    3. 判定每个箱子的 is_pickable
    4. 计算所有 16 维特征
    5. 构建 s_B, s_P, 调用模型
    6. 映射索引 → 全局 box_id
    7. 机械臂执行抓取
    8. 更新 last_pick_position
```

---

## 7. 关键常量汇总

| 常量 | 值 | 用途 |
|------|-----|------|
| top_clearance_margin | 0.02 m | 顶部判定 Z 裕量 |
| top_block_xy_relax | 0.03 m | 顶部判定 XY 放宽 |
| front_clearance_z_ratio | 0.45 | 前向 Z 判定比例 |
| support_z_threshold | max(0.04, 0.2b) | 支撑关系 Z 阈值 |

---

## 8. 特殊情况

| 情况 | 处理 |
|------|------|
| N_P = 0（无箱可抓） | 不调用模型，触发人工干预 |
| N_P = 1（仅一个） | 模型仍可推理，部署时建议仅用一只手 |
| N_B = 0（容器空） | 任务结束 |
| 第一帧无 last_pick | 索引 13-15 填入 0 |
