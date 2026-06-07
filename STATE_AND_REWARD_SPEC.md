# State and Reward Specification

本文档描述当前 `pick_a2c_2` 项目中**实际生效**的状态空间、候选箱（pickable）判定和奖励函数。

对应代码文件：
- `env.py`
- `model.py`
- `a2c_train.py`
- `test_policy_visual.py`

---

## 1. 状态空间总览

当前时刻状态定义为：

\[
s_t = (s_B^t,\; s_P^t)
\]

其中：

- \(s_B^t\)：当前容器内所有剩余箱子
- \(s_P^t\)：当前可抓取候选箱子（pickable）

每个箱子的特征为一个 16 维向量。

设：

- 箱子中心坐标：\((x_i, y_i, z_i)\)
- 箱子尺寸：\((l_i, w_i, h_i)\)
- 容器尺寸：\((L, W, H)\)
- 标准箱尺寸：\(b\)
- 上一次抓取中心：\((x_{last}, y_{last}, z_{last})\)

---

## 2. 状态结构表

| 项目 | 数学定义 | 含义 | 备注 |
|---|---|---|---|
| 全局箱集合 | \(\mathcal{B}_t\) | 当前所有剩余箱子 | 环境整体状态 |
| 候选箱集合 | \(\mathcal{P}_t\) | 当前可抓箱子集合 | 动作边界 |
| 全局状态 | \(s_B^t=\{f_i \mid i\in\mathcal{B}_t\}\) | 所有剩余箱子的特征序列 | 变长 |
| 候选状态 | \(s_P^t=\{f_i \mid i\in\mathcal{P}_t\}\) | 当前可抓候选箱子的特征序列 | 变长 |
| 单箱特征 | \(f_i \in \mathbb{R}^{16}\) | 单个箱子的 16 维描述 | 当前代码版本 |

---

## 3. 单箱 16 维状态特征表

| 维度 | 名称 | 数学公式 | 含义 | 作用 |
|---|---|---|---|---|
| 1 | `x_norm` | \(x_i / L\) | 前后位置归一化 | 判断离门口远近 |
| 2 | `y_norm` | \(y_i / (W/2)\) | 左右位置归一化 | 判断左右分布 |
| 3 | `z_norm` | \(z_i / H\) | 高度归一化 | 判断层高 |
| 4 | `l_norm` | \(l_i / b\) | 长度归一化 | 判断尺寸偏差 |
| 5 | `w_norm` | \(w_i / b\) | 宽度归一化 | 判断尺寸偏差 |
| 6 | `h_norm` | \(h_i / b\) | 高度尺寸归一化 | 判断尺寸偏差 |
| 7 | `front_preference` | \(1 - x_i/L\) | 越靠门口越大 | 显式表达前排优先 |
| 8 | `top_clearance` | \(c_i^{top}/H\) | 顶部净空归一化 | 判断上方是否空 |
| 9 | `front_clearance` | \(c_i^{front}/L\) | 前向净空归一化 | 判断抓后能否退出 |
| 10 | `support_load` | \(n_i^{support}/n_z\) | 结构支撑程度 | 提供结构信息（仅状态，不参与奖励） |
| 11 | `is_dropped` | \(\mathbb{1}[i\text{ is dropped}]\) | 是否散落箱 | 泛化关键标记 |
| 12 | `drop_severity` | \(\text{sev}_i\) | 散落严重度 | 决定优先处理强度 |
| 13 | `is_pickable` | \(\mathbb{1}[i\in\mathcal{P}_t]\) | 当前是否属于可抓候选 | 标识动作边界 |
| 14 | `dx_to_last` | \((x_i-x_{last})/L\) | 与上次抓取位置的 x 差 | 约束连续抓取 |
| 15 | `dy_to_last` | \((y_i-y_{last})/W\) | 与上次抓取位置的 y 差 | 约束连续抓取 |
| 16 | `dz_to_last` | \((z_i-z_{last})/H\) | 与上次抓取位置的 z 差 | 约束连续抓取 |

---

## 4. 状态空间中的辅助量定义

### 4.1 顶部净空 \(c_i^{top}\)

箱子 \(i\) 的顶面高度：

\[
z_i^{top} = z_i + \frac{h_i}{2}
\]

另一个箱子 \(j\) 的底面高度：

\[
z_j^{bottom} = z_j - \frac{h_j}{2}
\]

若箱子 \(j\) 与箱子 \(i\) 在 \(x,y\) 投影上重叠：

\[
|x_i-x_j| < \frac{l_i+l_j}{2}
\]

\[
|y_i-y_j| < \frac{w_i+w_j}{2}
\]

则可能遮挡箱子 \(i\)。

定义遮挡集合：

\[
\mathcal{O}_i = \{j\neq i \mid |x_i-x_j|<\frac{l_i+l_j}{2},\; |y_i-y_j|<\frac{w_i+w_j}{2}\}
\]

顶部净空：

\[
c_i^{top}=
\begin{cases}
H-z_i^{top}, & \mathcal{O}_i=\varnothing \\
\min\limits_{j\in\mathcal{O}_i}(z_j^{bottom}-z_i^{top})_+, & \mathcal{O}_i\neq\varnothing
\end{cases}
\]

其中：

\[
(a)_+ = \max(a,0)
\]

### 4.2 前向净空 \(c_i^{front}\)

箱子 \(i\) 朝门口方向的前表面：

\[
x_i^{front} = x_i - \frac{l_i}{2}
\]

若箱子 \(j\) 满足：

\[
x_j < x_i
\]

\[
|y_i-y_j| < \frac{w_i+w_j}{2}+\delta_f
\]

\[
|z_i-z_j| < \frac{h_i+h_j}{2}+\delta_f
\]

其中 \(\delta_f\) 为前向净空边界裕量，则认为箱子 \(j\) 可能挡住箱子 \(i\) 向门口退出。

定义阻挡集合：

\[
\mathcal{F}_i=\{j\neq i \mid x_j<x_i,\; |y_i-y_j|<\frac{w_i+w_j}{2}+\delta_f,\; |z_i-z_j|<\frac{h_i+h_j}{2}+\delta_f\}
\]

阻挡箱后表面：

\[
x_j^{back}=x_j+\frac{l_j}{2}
\]

前向净空：

\[
c_i^{front}=\max\left(0,\;x_i^{front}-\max\left(0,\max_{j\in \mathcal{F}_i}x_j^{back}\right)\right)
\]

### 4.3 支撑负载 \(n_i^{support}\)

箱子 \(i\) 顶面高度：

\[
z_i^{top}=z_i+\frac{h_i}{2}
\]

箱子 \(j\) 底面高度：

\[
z_j^{bottom}=z_j-\frac{h_j}{2}
\]

如果满足：

\[
|z_j^{bottom}-z_i^{top}| < \epsilon_s
\]

其中：

\[
\epsilon_s=\max(0.04,\;0.2b)
\]

并且投影重叠：

\[
|x_i-x_j| < \frac{l_i+l_j}{2}
\]

\[
|y_i-y_j| < \frac{w_i+w_j}{2}
\]

则认为箱子 \(i\) 支撑箱子 \(j\)。

于是：

\[
n_i^{support}=\sum_{j\neq i}\mathbf{1}\Big(|z_j^{bottom}-z_i^{top}|<\epsilon_s \land |x_i-x_j|<\frac{l_i+l_j}{2} \land |y_i-y_j|<\frac{w_i+w_j}{2}\Big)
\]

### 4.4 散落严重度 \(\text{sev}_i\)

定义四个子项：

\[
s_i^{(x)} = 1 - \text{clip}(x_i/L,0,1)
\]

\[
s_i^{(z)} = 1 - \text{clip}(z_i/H,0,1)
\]

\[
s_i^{(size)}=\min(1,\;|l_i-b|/b)
\]

\[
s_i^{(y)}=\min(1,\;|y_i|/(W/2))
\]

总严重度：

\[
\text{sev}_i =
0.35\, s_i^{(x)}
+0.30\, s_i^{(z)}
+0.20\, s_i^{(size)}
+0.15\, s_i^{(y)}
\]

最后裁剪到 \([0,1]\)。

### 4.5 上次抓取偏移量

若上一轮抓取中心为：

\[
p_{last}=(x_{last},y_{last},z_{last})
\]

则：

\[
dx_i = (x_i-x_{last})/L
\]

\[
dy_i = (y_i-y_{last})/W
\]

\[
dz_i = (z_i-z_{last})/H
\]

若当前没有上一轮抓取位置，则三者取 0。

---

## 5. pickable 判定表（当前版本）

当前可抓集合定义为：

\[
\mathcal{P}_t = \{ i \in \mathcal{B}_t \mid TopClear(i) \land FrontClear(i) \}
\]

注意：
- **当前版本没有 fallback**
- **当前版本不再使用 `SideClear(i)` 参与候选判定**
- **前向 x 条件是严格的：前面有箱子就算前方有障碍**

### 5.1 `TopClear(i)`

若不存在箱子 \(j\neq i\) 同时满足：

\[
z_j^{bottom} \ge z_i^{top}-\delta_t
\]

且顶部间隙：

\[
z_j^{bottom} - z_i^{top} \le \delta_{z}^{top}
\]

并且 \(x/y\) 投影重叠满足：

\[
|x_i-x_j| < \frac{l_i+l_j}{2} - \delta_{xy}^{top}
\]

\[
|y_i-y_j| < \frac{w_i+w_j}{2} - \delta_{xy}^{top}
\]

则：

\[
TopClear(i)=0
\]

否则：

\[
TopClear(i)=1
\]

当前代码默认：

- \(\delta_t = 0.02\)
- \(\delta_{z}^{top} = 0.02\)
- \(\delta_{xy}^{top} = 0.03\)

### 5.2 `FrontClear(i)`

若存在箱子 \(j\neq i\) 同时满足：

#### x 方向（严格前方）
\[
x_j + \frac{l_j}{2} < x_i - \frac{l_i}{2}
\]

#### y 方向（缩窄后的退出通道）
当前目标箱退出通道的 y 区间定义为：

\[
\left[y_i - \max\left(0,\frac{w_i}{2}-\delta_f\right),\;
y_i + \max\left(0,\frac{w_i}{2}-\delta_f\right)\right]
\]

前方箱的 y 区间定义为：

\[
\left[y_j-\frac{w_j}{2},\;y_j+\frac{w_j}{2}\right]
\]

两者有交集时，认为前方箱进入了当前箱子的退出通道。

#### z 方向（同层障碍）
\[
|z_i-z_j| \le 0.45 \cdot \max(h_i,h_j)
\]

若上述三类条件同时成立，则：

\[
FrontClear(i)=0
\]

否则：

\[
FrontClear(i)=1
\]

---

## 6. 奖励函数总表（当前版本）

| 编号 | 名称 | 数学定义 | 正/负 | 作用 |
|---|---|---|---|---|
| R1 | 双抓成功奖励 | 若左右都合法且不同目标，\(+2.0\) | 正 | 鼓励真正双抓 |
| R2 | 双抓失败惩罚 | 否则 \(-4.0\) | 负 | 惩罚无效双抓 |
| R3 | same target 惩罚 | 若左右抓同一箱，\(-10.0\) | 负 | 避免双手抓同一目标 |
| R4 | 相对 x 惩罚 | \(-\alpha_x \sum_{i\in\mathcal{G}_t} \frac{x_i-x_{min}}{x_{max}-x_{min}}\) | 负 | 相对当前环境前后范围，越靠后越罚 |
| R5 | 高度奖励（加大） | \(0.15\sum_{i\in\mathcal{G}_t} z_i/H\) | 正 | 偏向高处可抓箱，降低塌落风险 |
| R6 | 动作平滑惩罚 | \(-\alpha_m \|p_{cur}-p_{last}\|/S\) | 负 | 避免大跳跃 |
| R7 | 左右手错位惩罚 | 若 \(y_L > y_R\)，则 \(-\alpha_{cross}(y_L-y_R)/W\) | 负 | 鼓励左手抓左边，右手抓右边 |
| R8 | y 方向过近惩罚 | 若 \(dy<y_{th}\)，其中 \(y_{th}=0.10\)，则 \(-\alpha_{xy}(1-dy/y_{th})\) | 负 | 双臂目标在 y 上过近时惩罚（阈值缩小，鼓励左右相邻） |
| R9 | 双臂目标过远惩罚 | 仅以 \(dy>y_{th}\) 触发，其中 \(y_{th}=0.10\)，惩罚量取 \(dx/L + dy/W + dz/H\) | 负 | 双臂目标在 y 上过远时惩罚（触发仅看 dy，三轴加和惩罚） |
| R10 | 散落箱早期奖励 | \(\frac{1}{step}\cdot(0.5+0.5\,sev_i)\) | 正 | 越早抓散落箱，奖励越高 |
| R11 | collapse 惩罚 | \(-2.0 \cdot N_{collapse}\) | 负 | 惩罚塌落 |
| R12 | 移除数量奖励 | \(0.1 \cdot removed\_count\) | 正 | 轻微鼓励效率 |
| R13 | 清空奖励 | 若全部清空，\(+5.0\) | 正 | 任务终局奖励 |
| R14 | 空闲空间奖励 | \(\beta_{free}\sum_{i\in\mathcal{G}_t} \frac{1}{n_i^{near}+1}\)，其中 \(n_i^{near}\) 为以目标箱 i 为中心、半径 \(r_f=1.5b\) 的球体内其他箱子的数量 | 正 | 鼓励优先清理拥挤区域 |

---

## 7. 奖励函数详细公式表

### 7.1 双抓成功奖励

\[
r_{dual-success} =
2.0 \cdot \mathbf{1}[\text{left valid} \land \text{right valid} \land idx_L \neq idx_R]
\]

### 7.2 双抓失败惩罚

\[
r_{dual-fail} =
-4.0 \cdot \mathbf{1}[\neg(\text{left valid} \land \text{right valid} \land idx_L \neq idx_R)]
\]

### 7.3 same target 惩罚

\[
r_{same} =
-10.0 \cdot \mathbf{1}[idx_L = idx_R \land \text{left valid} \land \text{right valid}]
\]

### 7.4 相对 x 惩罚

设当前环境中所有剩余箱子的前后位置为 \(\{x_j\}_{j\in\mathcal{B}_t}\)，则：

\[
x_{min} = \min_{j\in\mathcal{B}_t} x_j,
\qquad
x_{max} = \max_{j\in\mathcal{B}_t} x_j
\]

对每个成功抓到的目标箱 \(i\)，定义相对位置：

\[
\tilde{x}_i = \frac{x_i - x_{min}}{x_{max} - x_{min}}
\]

则：

\[
r_x = -\alpha_x \sum_{i\in\mathcal{G}_t} \tilde{x}_i
\]

当前默认：

\[
\alpha_x = 1.8
\]

### 7.5 高度奖励（已加大）

\[
r_z = 0.15 \sum_{i\in\mathcal{G}_t}\frac{z_i}{H}
\]

### 7.6 动作平滑惩罚

当前抓取中心：

\[
p_{cur} = \frac{1}{|\mathcal{G}_t|}\sum_{i\in\mathcal{G}_t} p_i
\]

场景尺度：

\[
S = \sqrt{L^2+W^2+H^2}
\]

若存在上一轮抓取中心 \(p_{last}\)，则：

\[
r_{motion} = -\alpha_m \frac{\|p_{cur}-p_{last}\|}{S}
\]

若不存在，则：

\[
r_{motion}=0
\]

当前默认：

\[
\alpha_m = 0.35
\]

### 7.7 左右手错位惩罚

若左右抓到两个不同目标，且：

\[
y_L > y_R
\]

则：

\[
r_{cross-hand} = -\alpha_{cross}\frac{y_L-y_R}{W}
\]

否则：

\[
r_{cross-hand}=0
\]

当前默认：

\[
\alpha_{cross}=1.0
\]

### 7.8 y 方向过近惩罚

若左右抓到两个不同目标，定义：

\[
dy = |y_L-y_R|
\]

阈值（已从 0.275 缩小为 0.10，鼓励双臂左右相邻抓取）：

\[
y_{th}=0.10
\]

若：

\[
dy < y_{th}
\]

则：

\[
r_{xy-close} = -\alpha_{xy}\left(1-\frac{dy}{y_{th}}\right)
\]

否则：

\[
r_{xy-close}=0
\]

当前默认：

\[
\alpha_{xy}=1.0
\]

### 7.9 双臂目标过远惩罚（触发仅看 dy，惩罚量含 dx/dy/dz）

若左右抓到两个不同目标，定义：

\[
dx = |x_L-x_R|,
\qquad dy = |y_L-y_R|,
\qquad dz = |z_L-z_R|
\]

阈值（与 yth 一致）：

\[
y_{far}^{th}=0.10
\]

**触发判定**仅看 dy：

若：

\[
dy > y_{far}^{th}
\]

则：

\[
r_{far-pair} = -\alpha_{far} \cdot \left(\frac{dx}{L} + \frac{dy}{W} + \frac{dz}{H}\right)
\]

否则：

\[
r_{far-pair}=0
\]

**备注**：之前版本需要 dx、dy、dz 各自超过独立阈值才触发；当前版本改为**仅以 dy 超过 yth 作为触发条件**，**一旦触发，惩罚量为三轴归一化距离之和**。

当前默认：

\[
\alpha_{far}=0.8
\]

### 7.10 散落箱“越早抓越高”奖励

若抓到的是散落箱：

\[
r_{drop,i} = \frac{1}{t_{step}}\cdot (0.5 + 0.5\,sev_i)
\]

其中：

- \(t_{step}\) 是当前环境步数（从 1 开始）
- \(sev_i\) 是该散落箱的严重度

总和：

\[
r_{drop} = \sum_{i\in\mathcal{G}_t} r_{drop,i}\mathbf{1}[i\text{ dropped}]
\]

### 7.11 collapse 惩罚

设塌落事件数为：

\[
N_{collapse} = \sum_{j\in\mathcal{B}_{t+1}} \mathbf{1}[z_j^{old}-z_j^{new} > \tau_c]
\]

其中当前：

\[
\tau_c = 0.05
\]

则：

\[
r_{collapse} = -2.0 \cdot N_{collapse}
\]

### 7.12 移除数量奖励

\[
removed\_count = |\mathcal{B}_t| - |\mathcal{B}_{t+1}|
\]

\[
r_{removed} = 0.1 \cdot removed\_count
\]

### 7.14 空闲空间奖励（新增）

对每个被抓到的目标箱 i，计算以其中心为球心、半径 \(r_f = 1.5b\) 的球体内**其他箱子**的数量 \(n_i^{near}\)：

\[
n_i^{near} = \sum_{j\in\mathcal{B}_t,\, j\neq i} \mathbf{1}\big[\|p_i-p_j\| < r_f\big]
\]

该箱子的空闲空间奖励为：

\[
r_{free,i} = \frac{1}{n_i^{near}+1}
\]

总空闲空间奖励：

\[
r_{free} = \beta_{free}\sum_{i\in\mathcal{G}_t} r_{free,i}
\]

当前默认：

\[
\beta_{free}=0.3,\qquad r_f = 1.5b
\]

**设计意图**：箱子周围的同伴越少，空闲度越高，奖励越大；这鼓励策略优先清理拥挤区域，提高整体拆箱效率。

### 7.13 清空终局奖励

若：

\[
|\mathcal{B}_{t+1}|=0
\]

则：

\[
r_{finish}=5.0
\]

并令：

\[
done=True
\]

---

## 8. 总 reward 公式

\[
r_t =
r_{dual-success}
+r_{dual-fail}
+r_{same}
+r_x
+r_z
+r_{free}
+r_{motion}
+r_{cross-hand}
+r_{xy-close}
+r_{far-pair}
+r_{drop}
+r_{collapse}
+r_{removed}
+r_{finish}
\]

---

## 9. 测试脚本诊断说明

当前 `test_policy_visual.py` 会：

1. 高亮当前候选箱（绿色）
2. 高亮左右手实际选中目标（蓝色/红色）
3. 打印所有未进入候选集的箱子
4. 对每个被排除箱子，输出：
   - `id`
   - `reason`（`top_blocked` / `front_blocked`）
   - `row / col / layer`
   - `x / y / z`
   - `dropped`
   - 如果有顶部冲突箱，则输出：
     - `top_blocker`
     - `bottom_z`
     - `target_top_z`
     - `z_gap / z_gap_limit`
     - `dx / x_overlap_limit`
     - `dy / y_overlap_limit`
   - 如果有前向冲突箱，则输出：
     - `front_blocker`
     - `x_max / target_front_x`
     - `y_range / target_y`
     - `z_gap / z_limit`

---

## 10. 状态空间—pickable—奖励函数对应总表

| 模块 | 关键量 | 数学对象 | 作用 |
|---|---|---|---|
| 全局状态 | \(s_B^t\) | 所有剩余箱子特征序列 | 提供整体结构信息 |
| 候选状态 | \(s_P^t\) | pickable 候选箱子序列 | 提供动作边界 |
| pickable 判定 | \(\mathcal{P}_t\) | `TopClear ∧ FrontClear` | 决定哪些箱子可作为动作候选 |
| 状态特征 | \(f_i^{(1:16)}\) | 位置、尺寸、净空、支撑、散落、上次抓取偏移等 | 描述单箱价值与抓取上下文 |
| reward 主线 | \(r_t\) | 双抓 + 相对 x 惩罚 + 高度奖励（加大）+ 空闲空间奖励（新增）+ 动作平滑惩罚 + 左右手错位惩罚 + y 过近惩罚（阈值缩小）+ y 过远惩罚（仅 dy）+ 散落箱早期奖励 + collapse 惩罚 | 驱动策略学习 |
