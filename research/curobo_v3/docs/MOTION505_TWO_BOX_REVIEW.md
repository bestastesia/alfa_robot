# MOTION-505：双SKU抽取搬运待验收交付

- Linear：MOTION-505，父MOTION-419；In Review，验收人李昊洋，负责人文子轩。
- 基线：PR82 `ee6dba3280437ea33002ea87a5f8afb8725eadfb`；沿用本地专用分支 `feature/pr82-two-box-extraction`，远程交付分支 `motion-505-two-box-extraction`，按用户要求不开PR。不是主干合入或实机验收。
- 原25箱完整覆盖：Y∈[−1.02,1.02]m、Z∈[0,2.04]m。40箱箱心Y±.82/Z .20–1.84m；高箱Y±.7875/Z .291–1.749m；高箱Y/Z交换后Y±.729/Z .2325–1.8075m。连续箱心仍须满足高差≤.60m及完整箱体不重叠。
- 箱心共X=1.1680520510673524m，前距.66m；Z<.79顶吸，等于及以上侧吸；原SKU沿world−X抽离.300/.255m。回中左[40,−46,0,−100,−30,−75,90]°，右逐项相反，updown任务内在URDF[−1,0]m选择。

## 改动归类

| 交付 | 入口/文件 | 作用 |
| --- | --- | --- |
| 箱位与回放 | `tools/v3_box_layout.py`、`v3_box_layout_planner.py` | 实际箱心编辑、默认规划、状态/计时、真实物理ID回放和预览 |
| 合同与附件 | `curobo_core/{box_pick,layout_contract,suction_grasp,contact_reach}.py` | 原尺寸/面/吸盘足迹、固定前距/14角、必要可达性条件 |
| 顺序与换手 | `v3_ordered_layout.py`、`v3_handover_layout.py`、`curobo_core/{ordered_pick,handover,handover_roles}.py` | 物理ID独立于实际手臂，10kg支持切换、真实吸附、donor退离 |
| 公共安全/原生 | `native_spline.py`、`native_transport.py`、`v3_two_box_timing.py`及LM/载荷/扭矩共享适配 | 隔离数值修复、5mm底盘/2mm机器人地面门禁、完整箱体、RNEA、q/v/a/source历史 |
| 现用换手实现 | `tools/experiments/box_layout_20261119/transfer_{fresh,fresh_native,suffix,native,prefix_native}/` | 当前默认入口仍导入这些模块；尽管目录名为experiments，它们是运行依赖，不可按旧实验整目录删除 |
| 回归与独立审计 | `tests/`、`default_handover_regression/source_contact_audit.py` | 配置/身份/几何/数组/吸附检查；审计函数由已验证脚本原样纳入版本控制 |

## 验证与边界

[44例总摘要](two_box_acceptance_20261125/acceptance_current44.json)、[逐例参数/计时/实际吸附](two_box_acceptance_20261125/cases44.json)、[原始输入](two_box_acceptance_20261125/requests/)、[证据SHA](two_box_acceptance_20261125/evidence_receipts.json)。

最终native边界8+原36全新44/44、外部CPU44、首次实际吸附88、当时相关CPU107通过；29同步/8顺序/7换手，1,176,114 native点，31个实际升降值，14角最大误差4.37e−8rad。38例域内、6例更大范围，后者保留且不充作域内覆盖。规划批次墙钟695.763s和2540.943s，44案运动时长合计681.637s；普通混合双箱23.60s规划/6.41s运动。不是父419 AGV12的≤8/10s性能达标证据。

两个真实边界失败及修复：原指定手臂均不可达→保持物理ID与原位、自动交换实际手臂；样条拟合低于原5mm底盘间隙→抽离native硬校验/对应LM引导，加已有≤8 rest子段恢复（实测2段，5.215290/5.000187mm，q/v/a接缝0，总抽离距离不变）。历史6/8与7/8失败及违规整段均保留。

清理后与本轮相关的109项CPU检查再次全部通过（含既有107项和更新后的2项mock检查），日志`two_box_acceptance_20261125/post_cleanup_cpu109.txt`。

提交整理期间扩大检查：可收集CPU集合305 passed、15 failed、19 skipped；15个失败均在隔离PR82源码+同当前资产下复现，无新增失败。另2个旧测试文件因缺少`alfa_motion_viser`无法收集。详见[检查口径](two_box_acceptance_20261125/packaging_validation_20261126.json)。不写整仓测试全绿，不修改旧模型/历史期望去凑通过。

**连续全域成功、CCD、实机、吸附接触力、放箱码垛/释放/外部物流均未验收。**本次是有限离线候选，等待李昊洋检查；父419结论不自动变化。原始GPU报告保持原源SHA，不因整理脚本和文档重签为新执行结果。

## Viser复现

在本机新终端完整复制；已有8295服务运行时直接打开该页面即可，不要重复启动同端口。箱位标签属于原物理箱，实际取箱手臂可自动交换。启动仅CPU界面，点击解算后由内部PATH `gpuq`排队。

```bash
cd /home/astesia/Sevenova/.pr82-two-box-extraction
env \
  MAKEFLAGS='-j8 -l12' \
  CMAKE_BUILD_PARALLEL_LEVEL=8 \
  MAX_JOBS=8 \
  TORCHINDUCTOR_COMPILE_THREADS=8 \
  PYTHONDONTWRITEBYTECODE=1 \
  OPENBLAS_NUM_THREADS=1 \
  OMP_NUM_THREADS=1 \
  WARP_CACHE_PATH=/home/astesia/Sevenova/.pr82-two-box-extraction/research/curobo_v3/generated/default_handover_runtime_cache_20261124/warp \
  TORCHINDUCTOR_CACHE_DIR=/home/astesia/Sevenova/.pr82-two-box-extraction/research/curobo_v3/generated/default_handover_runtime_cache_20261124/inductor \
  TRITON_CACHE_DIR=/home/astesia/Sevenova/.pr82-two-box-extraction/research/curobo_v3/generated/default_handover_runtime_cache_20261124/triton \
  CUDA_CACHE_PATH=/home/astesia/Sevenova/.pr82-two-box-extraction/research/curobo_v3/generated/default_handover_runtime_cache_20261124/cuda \
  TORCH_EXTENSIONS_DIR=/home/astesia/Sevenova/.pr82-two-box-extraction/research/curobo_v3/generated/default_handover_runtime_cache_20261124/torch_extensions \
  /home/astesia/Sevenova/curobo_v2_fast/.venv/bin/python -B -u \
  research/curobo_v3/tools/v3_box_layout.py \
  --host 0.0.0.0 \
  --port 8295 \
  --left-size 40x40x30 \
  --right-size 46.5x25.5x58.2 \
  --front-distance 0.66 \
  --left-center 1.1680520510673524 0.5 1.02 \
  --right-center 1.1680520510673524 -0.5 1.02 \
  --side-min-height 0.79 \
  --max-contacts 16 \
  --estimate 8m \
  --session-idle-seconds 0 \
  --session-seconds 600 \
  --session-max-requests 8 \
  --runs-root /home/astesia/Sevenova/.pr82-two-box-extraction/research/curobo_v3/generated/two_box_layout/viser_acceptance_20261126
```

8m为参考已测最长约400s加初始化/波动的暂定单请求排队估时，不是超时；运动时长从原生轨迹读取，排队等待不计规划墙钟。完整GPU回归也必须用PATH `gpuq`持同一共享锁，最大8编译并行，禁止嵌套队列。

## 证据归档与清理

原始全部实验、失败轨迹、源快照已先压缩归档并校验，再移除工作区不再需要的历史二进制/重复快照；最终44案实际数组、摘要、输入和当前验收页面保留。归档位置与SHA见`evidence_receipts.json`，按需恢复到原`generated/two_box_layout`目录再使用历史诊断CLI；`test_handover.py`所需的16个历史原生数组/几何fixture仍保留本机，独立检出运行该历史集成测试需先恢复归档（不将资产缺失误报算法退化）。当前默认规划不读取旧解。

归档：`/home/astesia/Sevenova/验收归档/motion505-two-box-20261126/two_box_layout-evidence.tar.zst`。

```bash
cd /home/astesia/Sevenova/.pr82-two-box-extraction/research/curobo_v3/generated
tar --zstd -xf /home/astesia/Sevenova/验收归档/motion505-two-box-20261126/two_box_layout-evidence.tar.zst
```

压缩包完整性和SHA已验证。远程Git携带代码、配置、测试及小型摘要/原输入；完整NPZ/源快照保存在本机归档，不把本机归档声称为远程附件。当前仍依赖既有Fast模型/URDF/解析IK资产与环境，按`FAST_DEPLOYMENT.md`准备；未引入新第三方包。
