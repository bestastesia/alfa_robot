# pick_a2c_2

这是基于 `pick_a2c` 持续迭代后的双臂卸货 A2C 项目。

当前版本的核心特征：

- **持续式环境**：抓取失败、塌落、局面变乱不会直接结束，只有箱子全部清空才结束。
- **前门留空 + 前部散落箱**：规则堆叠主要放在后部，测试时可强制前部门口出现散落箱。
- **双臂动作输出**：模型每步输出两个抓取目标。
- **当前主奖励**：
  - 双抓成功/失败
  - 相对 x 惩罚（基于当前环境的 xmin/xmax 归一化）
  - 与上次抓取位置差的惩罚（连续抓取）
  - 左右手错位惩罚（左手偏左，右手偏右）
  - y 方向过近惩罚
  - 双臂目标过远惩罚（x/y/z）
  - 散落箱“越早抓越高”奖励
  - collapse 惩罚
- **Transformer + A2C**：使用变长状态序列、Transformer 编码和 A2C 训练。
- **完整训练工程能力**：
  - checkpoint 保存
  - `--resume path`
  - `--resume latest`
  - 训练曲线图输出
  - `curve_history.json`
  - `run_config.json`

---

## 目录中的关键文件

- `env.py`：MuJoCo 卸货环境、pickable 判定、奖励设计
- `model.py`：Transformer Actor-Critic 网络
- `a2c_train.py`：A2C 训练逻辑、日志、曲线、checkpoint 恢复
- `test_policy_visual.py`：加载 `.pt` 模型进行可视化回放与候选诊断
- `STATE_AND_REWARD_SPEC.md`：状态空间、pickable、奖励函数完整说明文档

---

## 关键可调参数（环境）

下面这些参数目前都集中在 `env.py -> ContainerUnpackEnv.__init__()` 中：

### 场景尺寸与生成
- `n_x`：容器前后方向离散列数
- `n_y`：容器左右方向离散列数
- `n_z`：容器高度层数
- `box_size`：标准箱尺寸
- `noise_prob`：散落箱出现概率
- `max_noise_boxes`：散落箱最大数量
- `empty_ratio`：规则堆叠高度图中的稀疏比例

### 物理仿真
- `settle_steps`：reset 后稳定步数
- `sim_steps_per_action`：每次动作后物理推进步数
- `collapse_threshold`：塌落判定阈值

### 候选箱判定
- `top_clearance_margin`：顶部遮挡 z 方向裕量
- `top_block_xy_relax`：顶部遮挡 x/y 投影放宽量
- `front_clearance_margin`：前向通道 y 方向收缩量

### 奖励系数
- `near_x_coef`：相对 x 惩罚系数
- `motion_penalty_coef`：与上次抓取位置差惩罚系数
- `cross_hand_penalty_coef`：左右手错位惩罚系数
- `xy_close_penalty_coef`：双臂 y 方向过近惩罚系数
- `xy_close_y_threshold`：双臂 y 方向过近阈值
- `far_pair_penalty_coef`：双臂目标过远惩罚系数
- `far_pair_x_threshold`：双臂 x 方向过远阈值
- `far_pair_y_threshold`：双臂 y 方向过远阈值
- `far_pair_z_threshold`：双臂 z 方向过远阈值

当前默认值请以 `env.py` 中实际代码为准。

---

## 安装

```bash
pip install -r requirements.txt
```

如果本机未安装 MuJoCo：

```bash
pip install mujoco
```

---

## 训练

### 启动新训练

```bash
python a2c_train.py --device cuda --log_dir ./runs/a2c_unpack_continuous_v1
```

### 从最新 checkpoint 恢复训练

```bash
python a2c_train.py --device cuda --log_dir ./runs/a2c_unpack_continuous_v1 --resume latest
```

### 从指定 checkpoint 恢复训练

```bash
python a2c_train.py --device cuda --log_dir ./runs/a2c_unpack_continuous_v1 --resume ./runs/a2c_unpack_continuous_v1/checkpoint_20000.pt
```

---

## 训练输出

当前训练目录会自动生成：

- `best_model.pt`
- `checkpoint_*.pt`
- `training_curves.png`
- `curve_history.json`
- `run_config.json`

训练图当前关注的主指标包括：

- `Episode Reward`
- `Average X Penalty`
- `Motion Distance Penalty`
- `XY Close Penalty`
- `Dropped Boxes Cleared`
- `Invalid Action Rate`

---

## 可视化测试（主入口）

当前主要测试入口是：

```bash
python test_policy_visual.py --model_path ./runs/a2c_unpack_continuous_v1/best_model.pt --device cuda --steps 30 --sleep 0.5
```

如果你想先在 CPU 上看：

```bash
python test_policy_visual.py --model_path ./runs/a2c_unpack_continuous_v1/best_model.pt --device cpu --steps 30 --sleep 0.5
```

---

## 测试脚本当前行为

`test_policy_visual.py` 现在会：

- 加载 `.pt` 模型
- 支持固定随机种子：
  - `--seed 12345`（默认）
  - 同一 seed 下测试场景可复现
- 默认在测试时使用：
  - `scatter_prob = 1.0`
  - 即测试场景 100% 会出现散落箱
- 可视化当前环境并回放模型抓取动作
- 打印每一步的：
  - `reward`
  - `remaining`
  - `next_pickable`
  - `avg_x_penalty`
  - `motion_penalty`
  - `xy_close_penalty`

---

## 可视化颜色说明

在测试回放中：

- **普通规则箱**：棕色
- **散落箱**：橙色
- **当前候选箱（pickable）**：绿色
- **左手选中目标**：蓝色
- **右手选中目标**：红色
- **左右手选中同一箱**：紫色

---

## 测试控制方式

viewer 模式下：

- 启动后默认**暂停**
- 按 **空格**：继续 / 暂停 切换
- 选中的左右目标会先高亮一帧，再执行抓取

启动后提示：

```text
Playback starts paused. Press Space to continue/pause.
```

---

## 候选箱诊断功能

测试脚本会输出：

- 所有**未进入候选集**的箱子
- 每个箱子的排除原因与几何数据

当前会打印的信息包括：

- `id`
- `reason`
  - `top_blocked`
  - `front_blocked`
- `row / col / layer`
- `x / y / z`
- `dropped`

如果是顶部冲突，还会打印：

- `top_blocker`
- `bottom_z`
- `target_top_z`
- `z_gap / z_gap_limit`
- `dx / x_overlap_limit`
- `dy / y_overlap_limit`

如果是前向冲突，还会打印：

- `front_blocker`
- `x_max / target_front_x`
- `y_range / target_y`
- `z_gap / z_limit`

这部分输出用于诊断：

- 为什么某个箱子没进候选集
- 是被哪个箱子挡住的
- 判定使用了哪些具体比较数值

---

## 导出当前 MuJoCo 场景 XML

如果你想导出当前场景 XML，可以用环境接口：

- `env.export_xml(path)`

---

## 当前版本说明

这个项目最近改动较多，尤其在：

- pickable 判定
- 奖励函数
- 环境持续性
- 训练图与恢复训练
- 测试脚本诊断能力

因此如果你在使用旧的 `.pt` 文件进行回放，请注意：

> 旧模型不一定适配当前环境规则。

如果要真正评估当前设计，建议用当前版本代码重新训练新模型。
