# L02 专项定位与修复（2026-10-07）

## 结论

阻塞不是2秒RRT时间预算：原抓取目标在进入路径规划前就没有合格IK。L-BFGS确实执行到所配迭代上限，但把优化种子和迭代数分别扩大4倍，原目标仍没有合格解。

实际缺陷是**把可吸附箱面过度限制为一个固定抓取点**。同一箱面内存在满足原机械限位的抓取点。现在auto依次尝试原面中心、上半面内点、下半面内点，然后保留原顶吸分支；不是按L02箱号写一个预设姿态。

最后一箱选中了同一正面内Z方向 **+53.75mm** 的点。箱体位置、车头0.75m净距、机器人模型/限位/TCP、2mm/1°IK阈值均未改；移动的是吸附点，不是箱子。**旧面中心目标没有被冒报为可达**。

## 排除过程

证据目录：`/home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/`。

1. **独立CPU FK/IK**：直接解析固定URDF链，与yourdfpy的12个随机状态逐项对拍；再用SciPy带原关节边界求解。原侧吸/顶吸目标的最佳有界结果压在`updown=-1m`和左J2=`-105°`下限，不能满足原阈值。
2. **限位消融仅用于诊断**：临时放宽离线求解器J2边界或升降边界会减小残差；未修改任何模型文件，也未执行这些越界解。相反，仅把合法侧吸点上移5cm，在原边界下就得到精确6D解。
3. **GPU预算对照**：保持同一模型、场景、碰撞和目标，读取全部优化输出，独立按FK误差、全部关节限位和整机碰撞验收，不只看SDK成功标志。

| 优化种子数 | 主优化迭代上限 | 原侧吸目标合格数 | 原顶吸目标合格数 |
|---:|---:|---:|---:|
| 512 | 500 | 0 | 0 |
| 512 | 2000 | 0 | 0 |
| 2048 | 500 | 0 | 0 |
| 2048 | 2000 | 0 | 0 |

原目标各次主优化器都实际执行到该上限。改为合法面内偏移、仍用512/500时，独立位姿/限位/碰撞复核得到**147个合法接触状态**；种子阶段已有解，主L-BFGS没有运行。CPU/GPU数值搜索不是连续空间不可达的数学证明，但不支持“只把路径超时调大即可解决”。

SDK失败项排序还存在诊断陷阱：`_get_result`对失败项加`1e16`，float32会损失原误差排序。因此旧日志中“返回128候选的最小误差”不是全512种子的全局最优。此次诊断读取全部512/2048输出；生产日志已明确标记统计范围，没有修改SDK文件。`budget-summary.json`修正了早期诊断脚本对“主优化器0次”的解释：这是种子成功后早退，不是停滞。

## 修复如何保持几何一致

- 备选点由箱面尺寸、冻结末端接触平面尺寸和5mm边距计算：`(.4/2 - .175/2 - .005)/2 = .05375m`。没有箱号判断，也没有写死关节角。
- 整个吸附平面仍在箱面内；最后一箱实测最小边距约22.5mm。
- 同一偏移同时用于接触目标、`tool_to_box`、40球载荷、缓存键和放置目标换算；保持箱体最终目的位姿不变。
- 新抓取的搬运IK目标也预检完整转身，最终轨迹仍走原碰撞/地面/倾角密检。旧中心方案失败及额外尝试耗时保留在`suction_attempts`中。
- 非auto请求保留原固定点行为。没有提升生产IK预算；仍是512种子/500迭代、最多返回128候选，原RRT每段2秒。

## 验收结果

- CPU测试：35项通过，新增接触足迹/箱体位姿保持、非法偏移拒绝与偏移缓存失效检查。
- L02独立完整周期：通过，13.487s，Home误差0；吸附、抽离、搬运、转身、释放、回位均完成。
- **从头整墙：25/25箱、15/15轮通过，27905帧，144.683s。** 第15轮发生一次合法抓取点重选；本次空载回程回退0次。
- 默认0.9m L24/R20回归：2/2通过，10.382/6.350s。
- 强化CPU验收：逐帧核对原URDF全部18个活动轴限位；逐轮请求、释放事件、末态继承、箱体消耗和首尾连接都需通过。最后一箱同时复核吸附面覆盖、刚性附着、35cm抽离及箱体目的位姿。

这里证明的是**指定f044研究碰撞策略下的一次完整在线规划及其复核**。不是全三角网格/FCL、真空承载或实机认证；也不等于原PR的公开种子矩阵、性能门槛及所有算法迁移均已完成。旧PRM回归问题不在本修复中消失。

## 查看与复跑

8095为本轮整墙只读回放，实时规划禁用，不占用规划GPU。可直接选择第15轮，界面会显示`0.05375m`面内偏移。

```bash
xdg-open http://127.0.0.1:8095
```

若页面退出：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/runtime/research/curobo_v3
CUDA_VISIBLE_DEVICES='' \
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_full_cycle_demo.py \
  --port 8095 \
  --wall-distance 0.75 \
  --sequence \
  --sequence-json /home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/sequence.json \
  --replay-only
```

GPU复跑必须先与其他窗口协调并持同一锁：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/runtime/research/curobo_v3
flock /tmp/sevenova-curobo-gpu.lock \
  env PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
  PYTORCH_ALLOC_CONF=expandable_segments:True \
  /home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_plan_cycle.py \
  --sequence \
  --wall-distance 0.75 \
  --output /tmp/motion233-l02-fixed-wall.json
```

CPU单箱几何复核：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/verify_motion233_top_suction.py \
  --suction-mode side \
  --result /home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/last-box-from-sequence.json \
  --runtime-root /home/astesia/Sevenova/motion233-acceptance/l02-diagnosis/runtime/research/curobo_v3
```

GPU窗口已经自然结束并明确通知协调线程接回。未push、修改PR或写Linear。
