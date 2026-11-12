# MOTION-447 · PR82 空域 XYZ 抽离实验交接

## 结论与边界

**有优化时间的可能性，但典型／稳定收益小（<1.5s），且多处改造、开发成本及在线决策代价大，暂不替换默认链路。** 顶层外侧配对节省中位数0.610s；单个seed11曾节省1.626s，已保留，`<1.5s`不是样本最大值。其余两类箱对反而变慢。

|箱对（0-based）|严格速度配对|基线周期中位数s|XYZ周期中位数s|配对节省中位数|配对收益中位数|
|---|---:|---:|---:|---:|---:|
|顶层外侧L24/R20|7/7|11.342|10.705|+0.610s|+5.32%|
|上层内侧L18/R16|5/7|10.420|11.226|−0.878s|−8.48%|
|上层外侧L19/R15|6/7|10.640|11.121|−0.438s|−4.18%|

正值为变快；配对收益中位数不能由两列时长中位数直接相除。比较完整**候选运动时长**，不是规划计算加速或硬件节拍。基线完整原生/动力学标志18/21，新携箱组合后20/21，剩余共享approach失败保留；速度只统计18组严格有效配对。

- 基线：PR82合入提交`ee6dba3280437ea33002ea87a5f8afb8725eadfb`。2026-11-11实验，3场景×7种子`11 23 37 51 71 89 101`。
- 相同吸附状态、运输终点、其余四阶段和剩余障碍。顶层移除21/22/23；另两例移除上方20/21/23/24，保留中央22。显式箱格场景不冒充原PR合并占位cuboid逐字节相同。
- 尝试过合并段重拟合、整段自由几何TrajOpt、直接接触根RRT、短X控制、上方/上方侧向、整格上方净空；失败与原型输入错误均留档，不计作有效提速。停止维护失败的整段优化入口，历史源码仍在归档。
- 最终59点终点网格：X=0/−0.04/−0.10/−0.18/−0.26m，侧向幅度0/0.10/0.18m朝已移除邻格，Z=0/0.12/0.24/0.44m，排除零位移；按解析IK启发式选短X/上方/上方侧向，补测整格上方净空。原15轴LM、1.10世界膨胀RRT、LM后处理、TrajOpt/TimeOpt保留；真正XYZ候选与短X控制分开统计。
- 候选选择是有界、样本内探索；既不是连续XYZ全局最优，也不是包含原35cm保底的最短时间规划器。仍在抽离/运输接缝停一次。
- 内侧代表seed71抽离1.377→1.376s，运输2.357→3.235s；最大单关节累计转角135.6°→200.2°。外侧seed89抽离只省0.038s、运输多花0.421s，说明时间被转移到后段。
- 先行候选评估加补测额外计算中位数约7.6–8.8s；正式接入需涉及空域决策、候选预测、携箱碰撞、IK/LM连续性、实际末态重连、时间化/停止策略、失败保底与回归，不是改一个35cm常量。
- 保持每箱10kg、60°/s、jerk500rad/s³、原扭矩和0.995缩放。42个选中携箱阶段各4001点动力学及离散40球碰撞采样不等于连续碰撞/实体边角/实机安全认证。本地Torch/NumPy/SciPy版本与PR历史发布版本不同，两方法在同一环境配对重跑；固定Fast源码/模型哈希已通过。

## 代码与证据放哪里

- `tools/benchmark_free_space_extract.py`：只保留最终可复用实验；不修改默认规划器。新输出目录必须不存在，拒绝跨场景/种子复用。
- `tools/summarize_free_space_extract.py`：原始两轮summary重建或从精简report渲染自包含HTML。
- `tools/v3_free_space_extract_viewer.py`：原URDF、原TimeOpt秒表的CPU并排回放；无客户端不推进播放，手选箱对/种子/阶段会关闭自动轮播。
- `tools/check_free_space_extract.py`：无需GPU的精简证据检查、可选ZIP校验及归档原始轨迹复核。
- `evidence/free_space_extract_pr82_20261111/`：21组CSV、精简report、历史验证与归档元数据。约112KiB，不提交轨迹、机器缓存、重复HTML和失败临时目录。
- Linear：[MOTION-447](https://linear.app/sevenova/issue/MOTION-447)。原生文档含全部尝试与数据；Resources有**187566976字节、280项**ZIP。附件ID、SHA256及范围见`evidence/free_space_extract_pr82_20261111/archive.json`，不是即将过期的签名URL。
- ZIP含21组基线六段/选中XYZ两段轨迹、场景source、两轮summary、冻结源码、HTML及验证记录；不含全部未选中临时轨迹。2026-11-12维护时再次核对远端HTTP200、大小和ETag与本地MD5一致，并逐项验证ZIP内SHA清单。
- 本机仅保留一份忽略的ZIP：`generated/free_space_extract/MOTION-447-pr82-xyz-extraction-data.zip`；原始展开目录和浏览器临时profile已清理，8171临时服务已停止。历史文档里的“当前正在运行”仅对应2026-11-11当时状态。

## CPU检查与HTML（不需要归档或GPU）

在本分支检出的任意目录执行；下面使用本机已有Python环境，不安装新依赖：

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO/research/curobo_v3" && \
export CUDA_VISIBLE_DEVICES='' PYTHONPATH="$PWD/tools" \
  OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python tools/benchmark_free_space_extract.py --self-check && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python tools/check_free_space_extract.py && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python tools/summarize_free_space_extract.py \
  --report evidence/free_space_extract_pr82_20261111/report.json \
  --output generated/free_space_extract/report.html
```

## 从归档恢复数据及回放

新机器先从MOTION-447的Resources下载ZIP，存为本检出内`research/curobo_v3/generated/free_space_extract/MOTION-447-pr82-xyz-extraction-data.zip`，再执行下列完整命令。**只展开到忽略的restored目录，不能在仓库根直接解压：ZIP还包含历史工具源码，会覆盖维护后的文件。** ZIP本身按archive.json校验后才解压，不执行归档里的旧脚本。

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO/research/curobo_v3" && \
export CUDA_VISIBLE_DEVICES='' PYTHONPATH="$PWD/tools" \
  OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 && \
DATA="$PWD/generated/free_space_extract" && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python tools/check_free_space_extract.py \
  --archive "$DATA/MOTION-447-pr82-xyz-extraction-data.zip" && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python -m zipfile -e \
  "$DATA/MOTION-447-pr82-xyz-extraction-data.zip" "$DATA/restored" && \
RESULTS="$DATA/restored/research/curobo_v3/generated/free_space_extract" && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python tools/check_free_space_extract.py \
  --results "$RESULTS" && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python -c \
  'from v3_fast_assets import prepare_fast_assets; prepare_fast_assets()' && \
/home/astesia/Sevenova/curobo_v2/.venv/bin/python -u tools/v3_free_space_extract_viewer.py \
  --results "$RESULTS" \
  --report evidence/free_space_extract_pr82_20261111/report.json \
  --host 127.0.0.1 \
  --port 8171
```

浏览器打开`http://127.0.0.1:8171/`，仅离线回放，无硬件指令；若本机GPU正忙，用CPU软件渲染浏览器，不能绕过全局GPU队列占卡。回放完成后Ctrl-C停服务。可加`--self-check`检查21对/126时间线，或`--smoke-test`检查三个完整模型场景。

## 重新做GPU实验（不是本次维护验证）

先按`FAST_DEPLOYMENT.md`准备固定依赖和当前检出资产；不得把Python包导入成功当模型一致。ETA6m来自原主轮计算285.747s加初始化/波动余量，不是超时或保证。整个前台负载一个PATH中的`gpuq` entry，不覆盖锁变量、不嵌套排队；后续规模变化需重新校准ETA。

```bash
REPO="$(git rev-parse --show-toplevel)" && \
cd "$REPO/research/curobo_v3" && \
export CUDA_VISIBLE_DEVICES=0 \
  PYTHONPATH=/home/astesia/Sevenova/curobo_pr82_76c2f4b:"$PWD/tools" \
  OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MAKEFLAGS='-j8 -l12' \
  CMAKE_BUILD_PARALLEL_LEVEL=8 MAX_JOBS=8 TORCHINDUCTOR_COMPILE_THREADS=1 && \
RUN=$(date +%Y%m%d-%H%M%S) && \
gpuq --name 'MOTION-447空域XYZ双箱3场景×7种子复测' --estimate 6m -- \
  /home/astesia/Sevenova/curobo_v2/.venv/bin/python -u \
  tools/benchmark_free_space_extract.py \
  --cases top_outer upper_inner upper_outer \
  --seeds 11 23 37 51 71 89 101 \
  --output "generated/free_space_extract/multiseed-${RUN}"
```

净空补测入口为`--clearance-only --reuse-baselines`，后者指向同场景/种子的首轮目录（归档恢复时指向`restored/research/curobo_v3/generated/free_space_extract/multiseed-20261111`）。原净空补测71.916s；21组暂按2m排队，并按实际机器冷启动调整。维护只删失败原型和不用的诊断导出，最终XYZ求解算法与冻结源码AST对照一致；未冒称维护后重新完成GPU实验。
