# LM整段关节路径调整＋Fast TimeOpt：1.1工程师交接

日期：2026-11-09。状态：用户认可研究方向；两个阶段的单任务离线实验完成，尚未迁入正式全流程。本文为迁移入口，历史过程及补充数据见同目录 `LM_STRAIGHT_REFINEMENT_20261109.md`。

## 1. 具体新增的操作

```text
原候选选型 / 原LM解算
        ↓
LM输出整段关节路径 q[帧, 15]
        ↓
【新增】64控制点三次样条表示＋整段最多32轮L-BFGS调整
        ↓
现有原生Fast TimeOpt（动力学限制不变）
        ↓
输出有时间戳的关节位置、速度、加速度、jerk
```

只修改LM之后、TimeOpt之前的中间关节路径。两端关节构型固定，中间共享Updown和左右各7轴可以协调改变；TCP/箱体路线及朝向通过软残差保持。不是重新选择吸附/运输终点，不是修改RRT，不是控制器250Hz五次插值，也没有引入五次样条转换。

目标是减少关节路径的局部弯折和高阶变化，使相同jerk/速度/力矩限制下TimeOpt允许更快执行。当前L-BFGS目标不直接包含TimeOpt返回时长，因此更多迭代不保证更快，需用最终时间比较。

## 2. 与PR80的关系

已只读核对PR80，head为 `9872c1ed64d09ce2e39a3696b83fb12f66e3d9a8`：

- https://github.com/kkozia188/alfa_robot/pull/80
- 本次使用的动力学 `generated/pr71_current_dynamics/robot.yaml`、`robot.urdf`，以及显示URDF `generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf`，在核对时与该PR逐字节一致。
- TimeOpt速度、jerk、力矩、负载与限制缩放一致。研究工作区自由段当时仍有不同的圆角/TrajOpt配置，不能称整条研究链与PR80完全一致。
- 本次只新增LM后处理实验，没有修改1.1检出、PR80、默认全流程、驱动或实机。本文不要求撤回PR80已有自由段策略。
- 两个测试对象均是L24/R20，最上排左右两端箱子；不是全任务泛化结果。

正式迁移位置：仅在 `extract` 和 `conveyor_place` 两个LM阶段完成后、现有 `time_path` / Fast TimeOpt之前接入。Home接近、运输RRT、返回Home等其他阶段保持工程师当前实现。

## 3. 实际算法与默认参数

实现：`tools/v3_straight_joint_refine_experiment.py::refine`。

### 输入输出

```python
controls, start, goal, diagnostics = refine(
    checker, rows, count=64, iterations=32,
    validity=validity, box_reference=reference_or_none,
)
```

- `rows`：有限值 `[N,15]`，顺序固定 `updown,left_joint1..7,right_joint1..7`；m/rad。
- `checker`：当前阶段、底盘位、场景、吸附变换和负载模型对应的检查器，不能拿取箱场景直接验证导航后的放箱。
- `validity`：主要向目标函数提供地面及球索引信息；优化器中的碰撞项直接取checker，并不会自动继承validity的场景穿透容差。
- 输出 `controls`：Fast原生三次路径控制数组 `[1,64,15]`；`start/goal` 为 `[1,15]`；最后一个控制槽按现有原生适配约定放置goal。不能把任意均匀采样数组直接冒充原生控制点。
- 起终状态固定，实验最终端点最大数值误差约 `1.2e-7`（旋转rad/Updown m分量混合报告）；TimeOpt后起终速度和加速度为零，未要求jerk为零。

### 表示与优化器

1. `v3_native_timeopt_probe.py::fit_path` 按当前关节路径弧长重采样，以最小二乘拟合原生三次B样条；此过程本身可能减少离散噪声。
2. 曲线两端各三个重复系数固定，中间系数为优化变量；不优化时间变量。
3. 在257个路径进度样本批量计算FK、约束残差、二阶/三阶路径导数。
4. PyTorch L-BFGS：`lr=.5,max_iter=32,max_eval=64,line_search_fn='strong_wolfe',tolerance_grad=1e-8,tolerance_change=1e-10`。
5. 32是最大优化迭代数，不等于32次FK或32次closure；线搜索会额外评估，实际次数需记录。各重复从同一原LM种子独立开始，未复用上一条优化结果。

### 目标函数权重（当前原型，尚未全任务调参）

|项|权重与计算|
|---|---|
|直线横向位置残差|`1e6 * mean(error_m²)`|
|朝向残差|`1e3 * mean(min(||q-qref||², ||q+qref||²))`，四元数正负等价|
|左右进度差|`1e4 * mean((sL-sR)²)`|
|进度倒退、超出[0,1]|各`1e3`平方惩罚|
|相对初始拟合关节路径偏移|权重1的均方差，含m/rad混合，尚未量纲归一化|
|路径二阶导数|`1e-5 * mean(||d²q/ds²||²)`|
|路径三阶导数|`1e-9 * mean(||d³q/ds³||²)`|
|自碰撞、外部障碍、关节边界、固定底盘、地面|当前各`1e6`平方惩罚|

这里的三阶导数是相对路径进度的导数，不是执行时间下的jerk。执行jerk由TimeOpt生成的时间映射决定。三次样条仍可能在节点处有jerk跳变；本方案没有证明“jerk处处连续”。

## 4. 两个阶段的参考不能混用

### 抽离：当前专用分支

`box_reference=None`：TCP的Y/Z保持起点值，朝向保持起点值，进度为 `(start_x-current_x)/.35`，左右共享进度。适用于本次局部坐标−X方向35cm抽离。

这不是任意方向通用实现。正式接入必须从任务参考读取起点、方向、距离，并处理world与footprint变换，不能硬编码`.35`/固定Y-Z后用于所有场景。

### 放箱：箱体直线＋沿程转姿

通过 `box_reference[side] = {start,end,attachment}` 提供同一局部坐标系下的4×4矩阵，`attachment=T_tool_box`。

```text
FK(q) → TCP位姿
TCP位姿 × 固定吸附变换 → 箱体中心/朝向
箱心投影到起终直线 → 进度s
期望箱心 = 起点 + s × 位移
期望箱体朝向 = SLERP(起点朝向, 终点朝向, s)
```

本段包含约45°转姿，TCP自身位置轨迹不必是直线。不能为了套抽离约束而冻结TCP Y/Z或朝向。当前原型仅双臂独立持箱，左右进度协调；不是双手共同持一个物体的精确闭链求解器。

## 5. TimeOpt与原验收口径

|项|原始设置|0.995缩放后|
|---|---|---|
|双臂各轴速度|60°/s|59.7°/s|
|双臂各轴jerk|500rad/s³|497.5rad/s³|
|每臂J1～J7连续力矩|[294,294,178,178,87,52,10]Nm|逐项乘.995|
|双臂加速度|1e6rad/s²|近似不限|
|Updown速度|1e4m/s|近似不限|
|Updown加速度/jerk/力|1e6对应单位|近似不限|
|负载|每箱10kg|进入质量/质心/惯量计算，不是约束阈值|

Fast原生TimeOpt保留32个timing controls、10轮、511个uniform intervals并增加节点侧检查；与64个关节路径控制点是不同概念。TimeOpt启用Graph，新增几何优化器尚未Graph化。

原LM `SequentialPlacementLm` 使用 `scene_penetration_tolerance_m=.001`。本实验事后按同样1mm场景口径检查，没有为通过本次实验新放宽参数。原抽离基线零容差额外诊断也有703/4001帧world拒绝；两种口径必须分别报告，不能宣称本次零容差无碰撞。

代表轨迹4001帧复核：自碰撞、外部场景、关节限位、固定底盘及地面；动力学采样检查速度/jerk/力矩，端点误差另列。它们是离散采样结果，不是连续时间认证。TCP/箱体偏线和朝向偏差是报告项，本次没有新加用户未要求的2mm直线硬门槛。优化中的soft penalty不等于严格约束求解。

## 6. 当前可引用结果

### A. 抽离：统一64控制点轮数扫描

14档0～40轮，每档5次，随机交错、预热后CUDA同步墙钟。每次新建L-BFGS并从相同种子开始。

|轮数|准备＋调整＋热TimeOpt中位数|运动中位数|每次计算＋运动之和的中位数|
|---:|---:|---:|---:|
|0（仅64拟合）|7.1ms|1.956s|1.963s|
|16|166.1ms|1.958s|2.125s|
|24|259.4ms|1.864s|2.123s|
|28|289.4ms|1.567s|1.857s|
|30|323.8ms|1.512s|1.837s|
|32|349.4ms|1.468s|1.811s|
|34|344.8ms|1.450s|1.820s|
|40|441.2ms|1.549s|1.971s|

原128拟合基线运动2.738s。32轮代表最大TCP偏线约.141mm、朝向偏差.017°；14档代表轨迹几何/动力学采样通过。70次TimeOpt success都检查，但只对每档一条代表做4001帧密验，不是70次密验全通过。

32和34轮总和仅差9ms，小于部分墙钟抖动，暂建议32作为统一实验默认，不宣称普适最优。之前单次40轮1.398s/277ms属于早期测量，不混入上述重复测试表。

### B. 运输位→传送带：三组各5次

|方案|运动时间|计算中位数|计算＋运动中位数|
|---|---:|---:|---:|
|原128拟合|3.123s|16.6ms|3.140s|
|仅64拟合|2.298s|约8ms|2.306s|
|64＋32轮|1.702s|290.4ms|1.993s|

32轮新增调整本身中位数282.1ms，准备7.8ms、热TimeOpt约.49ms；最大箱心偏线.0334mm、朝向参考偏差.1249°。三条代表4001帧几何/动力学采样通过，15次TimeOpt success通过。相对原128运动短45.5%，相对同64仅拟合短25.9%；不得把全部收益归于冗余构型调整，降控制点/拟合本身已有收益。

### 计时定义

- 总和中位数来自同一次运行的计算＋运动；不必等于分项中位数相加。
- 不含上游原LM、共用初始化、首次kernel编译/Graph捕获、保存与浏览器渲染。
- 额外4001帧诊断另列：放箱优化组约37ms；抽离轮数扫描热诊断约70～80ms，第一档可能含冷开销。迁移时若把这些检查放入正式关键路径，必须计入端到端延迟。
- 首次TimeOpt调用可能重捕获Graph，不得用表中的约.5ms替代冷启动耗时。
- 单任务、单负载、单硬件实验，未证明其他任务都提速，也未进行实机执行。

## 7. 代码和数据入口

研究根目录：`/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3`。

|相对路径|职责|
|---|---|
|tools/v3_straight_joint_refine_experiment.py|refine核心及最初32/64控制点实验|
|tools/v3_native_timeopt_probe.py|原生三次路径fit_path适配|
|tools/v3_straight_refinement_sweep.py|抽离14档×5次轮数扫描|
|tools/v3_placement_refine_experiment.py|放箱三组×5次，与旋转箱体参考构造|
|tools/v3_fast_three_layers.py|复用原TimeOpt限制、负载模型、采样/计时；非新增算法|
|tools/v3_frozen_shape_compare.py::stage_setup|导航前后场景与底盘坐标设置|
|tools/v3_lm_straight_baseline_viewer.py|两个页面共用只读回放，不参与优化|
|generated/lm_straight_corner_20261109/raw_lm.npz|抽离原LM输入|
|generated/lm_straight_corner_20261109/iteration_sweep.json|权威抽离重复计时、代表验收、最优轮数|
|generated/lm_straight_corner_20261109/refinement_summary.json|早期单次40轮对照，勿与扫描表混算|
|generated/lm_corner_placement_refine_20261109/refinement_summary.json|权威放箱15次计时及三条代表验收|
|generated/lm_corner_placement_refine_20261109/source.json.gz|放箱历史LM来源及场景、目标、附着变换|

大体积generated资产可能被Git忽略，不要以“PR里没有文件”推断实验不存在，也不要复制全部历史generated。本文旁的 `LM_TIMEOPT_REFINEMENT_MANIFEST_20261109.json` 给出关键源码/模型/结果文件绝对路径和SHA256，便于按需同步。

当前页面：抽离 http://localhost:8269/ ，放箱 http://localhost:8270/ 。均为离线结果回放，不是点击重新求解，不发送硬件。关节颜色模式为速度/力矩/jerk的实际规划限值利用率，红色不等于故障或因果证明。

## 8. 复现命令

以下均在当前电脑本地运行，只读输入并写实验输出。会重新计算并覆盖对应实验summary，不会执行机器人。

抽离轮数扫描：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3 && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:tools OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -u tools/v3_straight_refinement_sweep.py
```

放箱三组对照：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3 && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:tools OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -u tools/v3_placement_refine_experiment.py
```

查看当前服务，不另起同端口：

```bash
systemctl --user status alfa-lm-corner-baseline-8269 alfa-lm-placement-refine-8270 --no-pager
```

## 9. 给1.1的集成建议与尚未完成事项

建议先作为独立可开关模块接在两段LM后，保留原路径对照，再做全任务回归；目前不应凭一个案例直接替换所有任务。若拆PR，可先做模块/配置/数据合同，再单独做两阶段接线和全任务验收。

迁移前需要处理：

1. 从tools实验脚本抽出纯核心：不依赖实验ROOT、旧generated快照和私有TimeOpt字段；返回控制点、边界和diagnostics，TimeOpt仍由现有工程入口管理。
2. 把15轴顺序、box_reference坐标系、单位、双臂数量、非零位移、NaN、模型/负载身份和端点误差校验做成显式合同。当前代码假定双臂，不能直接用于单臂。
3. 抽离参考方向/距离参数化；放箱用当前任务实际吸附变换、目标和导航后场景，不能重新加载本实验L24/R20源文件。
4. 当前目标函数是软代价，需接现有正式验收合同；不可把penalty低、TimeOpt成功或“播放完整”当作全项通过。
5. 三次样条仍需检查节点侧jerk、控制器插值后的速度/力矩/jerk及TCP误差。未实现250Hz五次插值，也不承诺插值会消除峰值。
6. 确认是否允许候选不优时保留原路径。建议由工程师与用户明确接受策略；本实验没有自动回退RRT、改端点或静默替换失败候选。
7. 更新/预热资源不等于缓存任务答案。应复用常量basis、检查器、稳定Graph资源，但新任务仍需从当前LM路径重新计算。
8. 在同机同负载下分别统计初始化、准备、几何优化、TimeOpt、验收、端到端时间以及运动时长。下一步才能评估提前退出/自适应轮数，而不是按任务号硬编码轮数。

本文只交接研究方案及证据，没有发布生产版本或代替正式验收。
