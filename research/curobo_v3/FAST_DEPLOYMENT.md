# 当前全流程部署合同（2026-11-10）

本页是本次PR的唯一当前运行说明，基线为已合入的PR80（`alfa_v3_curobo@92bd8cdb`）。
旧README/HANDOFF里的解析抽离、151/191mm TCP、旧cuRobo版本和旧成功率只作历史证据。
本交付为本地理想模型规划、优化和可视化，不发送实机，不包含ROS生产接口或导航规划。

## 1. 入口、输入、输出

- 实时页面：`tools/v3_fast_live_viewer.py --port 8293`；点击才重新计算，不读取历史任务答案。
- 无界面全流程：`tools/v3_fast_three_layers.py --indices 0 38 --output ...`；索引来自
  `FastThreeLayerPlanner.planner.task_options`。当前正式Fast入口只开放双臂箱墙任务，不是任意Pose接口。
- 配置：`config/fast_three_layers.json`；`lm_refinement.enabled=false`仅关闭新增LM后处理，恢复LM128控制点拟合。
  `extract/conveyor_place`使用64控制点、最多32轮L-BFGS；每次从本次LM路径独立计算，不延续上次优化结果。
- 每次输出`NNN_source.json.gz`（实际规划路径、阶段、选型、LM与场景快照）、`NNN_report.json`
  （配置快照/摘要、分层计时、求解标志、偏差、动力学诊断）、六个阶段NPZ（position/velocity/
  acceleration/jerk/torque/times）。m、rad、s、N·m；七轴图速度用°/s，jerk用rad/s³。
- `complete_candidate`表示有完整时序候选，**不是几何验收通过**；`all_trajopt_success`、
  `dynamic_valid`、总体`success`分别报告。失败候选保留显示，异常不偷偷换成旧结果。

## 2. 固定的全流程逻辑

```text
固定Home
 → 多个解析吸附IK（枚举Updown/ψ，原筛选及排序）
 → 预测对应抽离终点（不是对全部候选做完整LM）
 → 多根/多目标RRT选吸附与运输构型组合
 → 仅选中候选做15轴连续LM抽离（Updown自由，携箱40球）
 → 若实际LM末态不同：实际末态RRT连接同一个已选运输构型
 → 验证Home接近和贴近，失败按原链路尝试其他组合
 → 运输位导航到放置停车位（瞬移，非导航路径）
 → 15轴LM放箱，末态关节允许自由求解，不回退RRT放箱
 → 释放箱体、从实际末态RRT回固定Home，底盘瞬移返回
```

轨迹规划策略仍是自研多起终点RRT；Fast提供底层FK/碰撞、LM、TrajOpt与TimeOpt，不把它称为
官方单起终点全流程。预测和实际抽离末态可能相差较大，重连解决接缝，但增加搜索耗时且预测选型未必最优。

轨迹优化：自由段`approach/transport/return_home`只做Fast几何TrajOpt（32控制点、50轮、
形状系数100、Graph）；LM后仅`extract/conveyor_place`做本PR的64控制点最多32轮L-BFGS，
不是自由段TrajOpt，保持两端关节值。最后贴近`contact_approach`仍原128拟合。
时序优化：全部六段分别Fast TimeOpt，段间零速度/零加速度边界，不要求jerk端点为零。

放箱参考必须由本次导航后场景、FK、实际吸附变换和目标生成；箱心走直线、朝向沿进度SLERP，
不能套抽离TCP固定朝向。抽离参考从本次LM的局部方向/距离构造，不硬编码负X35cm到所有任务。
模型、附着、有限值、关节顺序、非零位移合同明确校验；异常报错，无自动RRT回退或端点重选。

## 3. 唯一模型和碰撞口径

模型来源：`generated/zero_model_candidate_20261018/{fixed.yaml,mobile.yaml,robot_rezero_candidate.urdf}`；
动力学：`generated/pr71_current_dynamics/robot.{yaml,urdf}`；Home：`config/rezero_home.json`。
运行准备只在忽略目录`generated/fast_runtime_assets`生成当前检出的绝对路径，不修改冻结输入。
页面和规划使用同一重定位URDF，禁止借用原工程师工作区里的资产。

- 共享Updown＋左右各7轴，固定顺序15轴。mobile18额外包含base_x/y/yaw用于显示/确定性位移，不是18维自由规划。
- 新零位、关节轴/正向及腕部拟合沿用PR80；J7到TCP为+Z135mm，到J6旋转轴234.2mm。
- `base_footprint`无实体；`base_link`固定底盘和立柱使用四盒，定义在
  `generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json`。
  对机器人用球-盒，对世界用盒-盒SAT，移动底盘同步平移/Yaw。
- 原base_link71球槽位保留但半径置负，不再叠加球模型；`arm_carriage`仍20球随Updown移动；
  双臂及头部分支用原球，持箱每箱40球（内接近似，不宣称完全覆盖实体边角）。
- 自由RRT外部查询球半径、固定盒半尺寸乘1.10；IK接触、LM、自碰撞、内部四盒、地面及TrajOpt基准几何不膨胀。
- 世界原容差1mm、自碰撞原口径不变；接触只使用原吸附端帽针对自身目标箱的豁免，不删除环境障碍。
  LM后处理的碰撞项是原型软罚，零激活世界代价与验收1mm口径分别记录。
- 保留搜索节点/边和LM节点检查。用户已取消返回路径密化、0.25°审计、最终全流程及优化后验碰门，
  本PR不恢复；显示完整、TimeOpt成功不代表连续碰撞安全。
- 抽离后处理准备使用**携箱checker**，不能用空载checker优化抽离。

## 4. 动力学与限值

每箱10kg，根据每阶段实际吸附变换加入质量、质心与惯量；抽离/运输/放箱携箱，其余空载。
每臂J1～J7连续扭矩`[294,294,178,178,87,52,10]Nm`，速度60°/s、jerk500rad/s³，
规划缩放0.995。Updown及加速度用PR80的大有限界近似不限，尚非硬件量化参数。
原生TimeOpt固定32个时间控制点、10轮、511 uniform intervals（另有节点侧约束），与64路径控制点不同。
配置/负载/自定义代价不偷偷移除；冷启动、求解、诊断、墙钟、运动时长分别列出。

## 5. 依赖与干净检出准备

必需：Linux、CUDA GPU、Python3.10、g++、Eigen3。实测软件版本及关键源码/模型SHA256在
`config/fast_release_manifest.json`；cuRobo固定`76c2f4b6d325b4e76696939de1faa6c14d409e0a`，
当前运行时`_src`434个源文件与该官方提交逐字节对照一致。**不要安装最新main或套旧本地补丁。**
Torch2.8.0/CUDA12.8、Warp1.17.0、NumPy2.2.6、SciPy1.15.3、Viser1.1.1、yourdfpy0.0.60、
Plotly6.2.0、cuda-core1.2.0是本次实测组合，不等于保证其他组合不兼容。

模型、米制mesh、40球、运输IK原始库及解析桥源码已在Git；`.so`在检出后编译。
前端必须安装`config/visualization_dependency.json`的`alfa-motion-viser`冻结版本：
仓库`http://192.168.100.141:3000/SevenovaHangzhou/robot_motion_control.git`，提交
`479ce2a12db558816d8d7e88434f55dbb1904005`，子目录`observability/viser`。
这是内网前端依赖，不包含在本PR；没有访问权限必须找仓库维护者提供该版本，不能降级成简化模型。
`verify_frontend_package`核对渲染代码与资产清单哈希；不是只看包版本号。

**没有既有环境的新机器**可在当前PR检出内使用下列完整命令建立Python3.10虚拟环境。
前提是已安装系统Python3.10/venv、g++/Eigen3和匹配GPU驱动，能访问官方GitHub/PyPI与上述内网仓库；
不能访问内网时安装会停止，不会自动替换前端资产。依赖清单是本机实测冻结，不是跨硬件兼容性保证。
这是安装说明，本次验证复用匹配环境，没有声称已经在一台全新操作系统上重装测试。

```bash
REPO="$(git rev-parse --show-toplevel)" && \
python3.10 -m venv /mnt/mydisk/ALFA/.venvs/curobo-v2-py310 && \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pip install \
--requirement "$REPO/research/curobo_v3/requirements-fast-tested.txt" && \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pip install \
"nvidia-curobo @ git+https://github.com/NVlabs/curobo.git@76c2f4b6d325b4e76696939de1faa6c14d409e0a" && \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pip install \
"alfa-motion-viser @ git+http://192.168.100.141:3000/SevenovaHangzhou/robot_motion_control.git@479ce2a12db558816d8d7e88434f55dbb1904005#subdirectory=observability/viser"
```

下列本机命令优先使用`PYTHONPATH`中的固定Fast源码；其他机器没有该目录时会使用上面安装的固定包。
无论哪种方式，都必须通过源码SHA和API预检，不能靠导入成功就认定版本一致。

已有匹配Python环境时，在**当前PR检出的任意目录**直接复制以下完整命令。命令自动定位当前Git根，
本机运行时与Python路径明确列出；其他机器可在整段命令顶部调整对应路径，但不能混入旧cuRobo。

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO/research/curobo_v3" && \
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:"$REPO/research/curobo_v3/tools" \
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
tools/prepare_checkout.py --fast-only --compiler g++
```

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO/research/curobo_v3" && \
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:"$REPO/research/curobo_v3/tools" \
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
tools/v3_fast_preflight.py --gpu --output generated/local_fast_preflight.json
```

检查必须显示`ready=true`、`home_valid=true`、`fixed_body_box_world_enabled=true`，
前端和模型/运行时哈希核对成功。版本差异另列`version_differences`，有差异不能冒称完全相同环境。
无GUI诊断可加`--skip-visualization`，**该结果不能声称前端部署完成**。

## 6. 启动、完整测试、可视化

实时页面（本机独立发布检出已使用8293，原8290服务不受影响）：

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO/research/curobo_v3" && \
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:"$REPO/research/curobo_v3/tools" \
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -u \
tools/v3_fast_live_viewer.py --port 8293
```

访问`http://127.0.0.1:8293/`，选任务→点击重新计算→从头播放全流程；可按TimeOpt时间轴拖动、跳阶段、
切左右臂并看七轴v/a/jerk/扭矩。只展示本次返回数据；失败有标记，不读旧任务答案。
机器人、纸箱、集装箱、地面和龙头车使用冻结Viser模型，不拿碰撞球当视觉模型。
龙头车原位由`conveyor_box_targets[*].conveyor_position`更新，不能把放箱点当车体中心；
放箱参考与规划快照坐标一致。`--radius-review`是历史实验回放且依赖未提交结果，不是正式入口，不用于交付。

无GUI完整计算（非预存回放）：

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO/research/curobo_v3" && \
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:"$REPO/research/curobo_v3/tools" \
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -u \
tools/v3_fast_three_layers.py --indices 0 38 --output generated/local_lm_release_run
```

## 7. 验证范围与已知问题

针对性回归命令：

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO" && \
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:"$REPO/research/curobo_v3/tools" \
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pytest -q \
research/curobo_v3/tests/test_lm_refinement.py \
research/curobo_v3/tests/test_fast_release_contract.py \
research/curobo_v3/tests/test_fast_fixed_boxes.py \
research/curobo_v3/tests/test_predicted_selected_lm.py \
research/curobo_v3/tests/test_no_dense_rechecks.py \
research/curobo_v3/tests/test_frozen_lm_extraction.py
```

本次40通过、1跳过；跳过为未在干净检出放入历史大体积候选存档的回放检查，不是本次GPU求解失败。

本次独立检出与PR80基线逐文件对照：固定/移动模型、腕球、40球、四盒、Home及原RRT/LM编排不改；
只新增LM后处理、抽离后处理loaded checker接线、配置、依赖核对和入口部署适配。
干净检出两例都完成全流程候选，12段TimeOpt动力学采样通过；L24/R20全项求解标志通过，
L18/R11原接近TrajOpt失败保留。精确计时、接缝和逐阶段结果见
`generated/lm_refinement_release_20261110/validation.json`。不是全任务成功率，也不是新300箱验收。

继续保留：TrajOpt软间隙兼作约束可能标失败、候选路径可能绕路、冷/换拓扑开销、TimeOpt诊断共享缓冲区
可能触发Graph重捕获、LM软残差不保证严格直线、球模型覆盖缺口、旧模型/回放测试不一致等。
本次不擅自修复这些问题、不放宽验收、不用“动画能播”替代规划验收。

8291的300箱录像、约37分钟原倍速录制及单臂收尾都是历史临时演示，不属于本PR部署功能，
源码位于此前`/tmp`且未正式化；其他工程师按本PR可运行**双臂单任务完整闭环**，
不能宣称已经提供整箱300箱生产调度或实机系统。
