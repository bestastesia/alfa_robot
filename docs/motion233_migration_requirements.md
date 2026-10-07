# MOTION-233 迁移要求与当前证据（2026-10-07）

## 基线和提交边界

用户明确指定 `alfa_robot/alfa_v3_curobo@f044bf1`，并再次确认主线保留RRT。2026-10-07只读核对远端，`alfa_v3_curobo`仍为完整SHA `f044bf1463d6ab138bb5aac078056d1e3d17a1b9`。本地候选工作树为`.motion-233-curobo`，分支`motion-233-curobo-f044bf1`。

原PR #43仍是`b72ba30`→`alfa_v3_dev`，Open/Conflicting。mentor的2026-09-28评论明确禁止合并旧模型/旧流程，要求重建替代PR、选择性迁移，不回退模型/planar/执行链。**因此不是强行解决冲突再重放整个旧栈。** 本轮没有push、改PR或写Linear。

## 要求映射

| 要求/来源 | 已有证据 | 当前判定 |
|---|---|---|
| 用户指定最新cuRobo基线及模型 | 远端SHA、祖先关系；生成资产路径归一化和模型/vendor逐文件核验 | 已落在指定基线；不沿用误选的alfa_v3_dev Stage实验结论 |
| mentor统一机械坐标、零位、限位、TCP，不回退右J2/J4与joint origin | 相对f044，模型、vendor、生成模型资产未修改；全墙逐帧URDF限位复核 | 已保留，面内抓取变化不是修改TCP或机械限位 |
| 不删除planar根、底盘TF、Stage Action、轨迹缓存 | `git diff f044bf1 -- ros2_ws`为空 | 源码保留；没有声称新研究入口已接入/验证实机执行 |
| 用户要求RRT主线 | 默认informed_rrt；三种已有RRT函数AST与f044一致；25箱记录全为informed_rrt | 已保留。PRM修复隔离，不是本迁移门禁 |
| 车头净距0.75m、完整搬运序列、单臂收尾及顶吸 | 15轮25箱实跑，实际模型前缘差值、状态继承、面内足迹/附着/目标位姿和只读回放 | 当前研究碰撞口径通过；旧固定中心点没有被冒报可达 |
| 原PR参数化箱墙/缺箱 | WallLayout、快照取接触点、CPU合同；历史3×4/4×3选中双箱实跑 | 接口已迁；不同尺寸仍受冻结负载模型限制，不能说原全部L4场景已迁完 |
| 原PR固定公开种子 | 复用原8数值并保存原manifest；新请求/CLI将seed传入RRT各阶段，8例各一次25箱 | 球模型口径8/8通过；不等于旧OMPL轨迹或位级确定性已等价 |
| 原固定种子C/D精确轨迹对拍 | 新同请求同seed两次预检都成功，但运输阶段轨迹/冗余终点不同 | 尚未达到严格位级重现；不能用“有seed参数”冒认完成 |
| 原逐箱时延、预算、时间参数化、质量指标 | 新入口记录总时间及阶段/失败尝试；当前单轮多为8～12s，仍输出未计时关节帧 | 不能继承旧3s/5s门槛或实机时序结论；未完成 |
| 原完整FCL复核 | 新基线原样保留球模型。CPU对旧默认25箱抽查90个关键状态，发现3处未豁免网格相交，成对复查可复现 | 不可沿用旧模型FCL结论；需要专门处理模型表示差异，未改豁免/阈值 |
| 原Golden算法与参考资产的最终取舍 | 原PR/Git对象和冻结报告仍在；新实现复用导师核心，未移植旧坐标补丁 | 不宣称旧全部算法/预算/执行接口已迁移；需按要求继续收口 |

**整体目标仍未完成。** 已通过的研究序列、种子矩阵和模型对拍只是上述相应项的证据，不替代其它门禁。

## 种子合同

`PlanRequest.search_seed`和CLI`--search-seed`只控制RRT采样，不改IK策略、IK默认随机配置或2秒单段搜索预算。省略该字段保持旧请求ID、旧阶段种子与默认行为；接近阶段使用base+候选索引，搬运使用base，回程使用base+1，全部按uint32取值并写入结果。

`tests/fixtures/OMPL_SEEDS.json`从原PR `b72ba30:tools/v3_scoop_golden_20260921/OMPL_SEEDS.json`逐字节保留。它的旧名称不表示当前使用OMPL，数值集合为104729、104743、104759、104761、104773、104779、104789、104801。原manifest中的时延要求没有被删除，但当前不能宣布满足。

每个矩阵例独立进程、每例持共享GPU锁并检查其它compute PID，例间释放锁。固定清单每例只执行一次；失败要保留，禁止挑成功样本。壁钟预算和缓存中的IK采样历史都可能影响最终路径，因此只固定RRT种子不是完整确定性证明。预检分阶段对比显示起点/吸附/抽离完全一致，运输阶段开始不同；具体确定性收口尚未完成。

## 独立网格诊断的边界

诊断读取`l02-diagnosis/sequence.json`中**此前默认种子**的成功轨迹，并使用同一固定模型的URDF collision meshes、同一状态和研究碰撞豁免。它不是对八个新种子逐帧完成的FCL验收。

| 轮次/阶段 | 网格接触对 | CPU球对最小间隙 |
|---|---|---:|
| 第9轮，抽离终点 | arm_carriage / left_link5 | 约3.07mm |
| 第13轮，抽离终点 | base_link / right_link2 | 约56.89mm |
| 第14轮，携箱转身 | right_link4 / right_link7 | 约3.47mm |

两个独立组织方式的FCL查询都报告网格相交；上述球对仍分离，说明当前模型表示给出了不同判断。OBB这里只是辅助包围盒，不把非凸网格的OBB重叠当作实物碰撞证明。没有新增忽略项、没有缩放网格/球半径，也没有据此更换RRT。最终物理安全结论需要继续核对碰撞表示及完整路径。

## 本轮产物

`/home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/`：
- `matrix-01/matrix-plan.json`、`matrix-status.json`、八份逐seed结果与日志；代码/模型前后哈希一致。
- `default-preflight.json`、`seeded-preflight.json`、`seed-preflight-audit.json`、`warm-repeat-phase-diff.json`。
- `mesh-checkpoints.json`（90点抽查）、`mesh-pair-recheck.json`（3对复查）及CPU-only复现脚本。
- `migration-structure-audit.json`记录RRT函数AST和ROS执行树保留情况。

## 复跑命令

新建独立输出目录，运行完整公开种子清单（脚本逐例持同一flock；已有矩阵目录拒绝覆盖）：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/run_motion233_rrt_seed_matrix.py \
  --runtime-root /home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/runtime/research/curobo_v3 \
  --sdk-repository /home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
  --output-dir /home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/matrix-manual-rerun \
  --case-timeout 600
```

若只是单例复核：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/rrt-seed-contract/runtime/research/curobo_v3
flock /tmp/sevenova-curobo-gpu.lock \
  env PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
  PYTORCH_ALLOC_CONF=expandable_segments:True \
  /home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_plan_cycle.py \
  --sequence \
  --wall-distance 0.75 \
  --search-seed 104729 \
  --output /tmp/motion233-seed-104729-rerun.json
```

主线基线核验器默认只要求Informed RRT；只有显式`--check-optional-planners`才检查其它规划器。可选PRM实验不会阻塞RRT迁移，也不能拿来替代原有安全/性能要求。
