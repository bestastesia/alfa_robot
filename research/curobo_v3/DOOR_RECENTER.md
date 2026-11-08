# 门扉双箱依次回中

入口：`tools/v3_door_recenter.py`。流程先搬下箱到最新Atlas条件化回中位，再保持下箱、下降并抽离上箱，最后双箱收束后复用普通放箱/释放/Home后缀。

## 合同

- 回中：`placement_task_policy.json` + 2816行批准Atlas + 整机/实际双负载≤1.05m。
- 门扉实际附件通过固定seed局部条件化；保留真实 `tool_to_object`，不替换canonical payload。
- 下箱回中搜索升降＋左臂，右臂解析补偿保持上箱世界位姿；上箱阶段对称保持下箱。
- 接触、抽离、门板、箱墙、实际箱体OBB和工具接触网格始终门禁。
- cubic/polyline/stops共同验收选最短时间律；10kg/箱、0.9倍率RNEA只重定时，不改变q几何。
- 导航为到位状态切换，`navigation_path_validated=false`，不声称连续底盘轨迹。

## 当前验收

当前源码三种子11/29/41完整成功；机械臂时长29.94～30.88s，最终收束各臂额外旋转≤60.29°，包络1.0125m，RNEA峰值≤0.8999。上箱接近因安全端点跨腕分支，额外旋转87.39～97.81°，已低于105°产品门但不是理想30°。

```bash
cd /home/astesia/Sevenova/door-recenter-20261103/runtime-modernized-grasp
/home/astesia/Sevenova/curobo_v2/.venv/bin/python -u tools/v3_door_recenter_viewer.py \
  --result /home/astesia/Sevenova/door-recenter-20261103/runs/modernized-v50-current-s11.json \
  --port 8143
```

验证：`tests-modernization-cpu-v12.log`（75 passed）、`tests-modernization-gpu-v1.log`（1 passed）、三个当前源码 verifier、`rolling-v50-obstacle-left.json`。

## 计算延迟

2026-11-05将上箱预接触改为共享预算/确定性seed bank，下箱回中改为选中终点后的lazy未来下降验证，最终收束按65°门早停。seed 11/29/41端到端计算为36.33/39.41/43.86s；对应上箱预接触4.38/4.41/6.76s，下箱回中2.48/2.74/2.65s。证据见`latency-optimization-summary.json`及三个`latency-v8-*-verification.json`。

## 认证结算缓存

默认运行使用`generated/door_recenter_certified_plan.json.gz`。缓存key绑定关键源码、冻结资产、policy、算法、roll和动力学；payload有digest及失败关闭安全门。当前seed11/29/41内部结算125.6～138.3ms，完整进程2.66～2.84s；缓存失效时回退18.533s冷规划。证据：`settlement-under-10s-audit.json`。

## 无轨迹缓存最终计算口径

正式验收使用常驻`DoorPlanner`，只复用cuRobo checker/solver/RNEA模型等资源，不复用任何轨迹结果；每个请求重新IK、RRT、碰撞、时间化和RNEA。seed11/29/41为9.808/9.913/9.784s，均通过verifier。默认`--plan-cache`关闭。

## 五列产品验收（2026-11-07）

五列入口 `tools/v3_door_five_columns.py` 已按原顺序 `4,0,3,1,2` 完整通过。外/内列采用列对齐底盘到位、机架临时支撑、下箱抽离后立即回中等待、上箱再抓持和双箱共同放置；中心列保留原门扉流程。列0使用列4的严格几何对称变换（底盘y/yaw取反、双臂交换并取反、箱列 `4-column`），随后重新执行RNEA；全门扉帧经GPU检查，保守球模型的43帧边界假阳性再由FCL精确网格复核，最大接触深度0.0671mm，低于既有0.1mm几何容差。

产品证据：`/home/astesia/Sevenova/door-recenter-20261103/five-columns-product-v4/`。五轮结构验证、列0对称验证、92项CPU测试和1项GPU测试均通过；五轮最终附件为空，完成箱15～24，剩余箱0～14。单进程计算墙钟189.978s，轨迹运动总时长176.331s；详细分阶段耗时见 `FIVE_COLUMN_TIMING.md`。Viser运行于 `http://127.0.0.1:8144`；切换列轮次时同步显示该轮独立E2E、运动及阶段耗时。
