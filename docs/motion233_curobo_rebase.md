> **2026-10-08更新：** 最终PR基线已由历史`f044bf1`前移至`alfa_v3_curobo@4200247`；上游TaskCycle Demo保留，机器人/碰撞模型未变化。以下f044对拍内容是迁移过程证据。
> 最终隔离验证：MOTION-233完整墙25/25、25790帧、146.185s；上游L19/R15与L4/R0 TaskCycle均成功；272文件核验通过。

# MOTION-233：对齐指定 cuRobo 基线（2026-10-06）

> 本页记录最初 `2959ef6` 的冻结基线验收。后续布局增量及当前8095的0.75m页面见 `motion233_curobo_wall_layout.md`；下方原始产物与运行命令仍指向冻结副本。


## 正确基线与替代关系

- 指定目标：`alfa_robot/alfa_v3_curobo@f044bf1463d6ab138bb5aac078056d1e3d17a1b9`，本地分支 `motion-233-curobo-f044bf1`，工作树 `/home/astesia/Sevenova/.motion-233-curobo`。
- 当前入口为 `research/curobo_v3/tools/v3_plan_cycle.py` 和同核心 Viser `v3_full_cycle_demo.py`，不是 ROS/MoveIt Stage。默认模型为 `generated/v3_analytic_071cb95/` 的**解析代理模型**，源 `071cb954d8c626626c41cdeff599bc451a56c117`，并保留 f044 中的命名姿态、18维回放/15维运输、40球负载、完整6D目标与碰撞策略。
- cuRobo SDK 固定官方 `78fd485fa82d9b9a063fb4985e371814587e666a`；本机 Python3.10.12/Torch2.9.1+cu128/Warp1.17.0，导师参考 Torch2.8。不得静默导入旧的 cuRobo SDK。
- 之前 `.motion-233-v322-baseline` 是误选 `alfa_v3_dev` 的实验支线，原样保留；其 Stage 故障和“缺少铲具导致 blocked”判断不用于评价本基线。本次重建在正确上游上，**不重放这些不适用的 Stage/KDL/OMPL 改动**，也不修改另一个窗口的 mentor-adoption/c3 worktree。
- `research/curobo_v3` 的 **255 个跟踪文件**与 f044 原件逐字节一致。为避免提交本机绝对路径，运行副本用 `git archive f044...` 生成到验收目录；仅调用上游已有 `prepare_checkout.py` 重定位路径并单进程编译解析桥。模型 YAML/URDF 在剥除本地路径差异后内容完全一致，工具可再次校验。

## 实跑与可视化证据

证据根目录：`/home/astesia/Sevenova/motion233-acceptance/curobo-f044bf1/`。

| 检查 | 本次结果 |
|---|---|
| 上游源码、模型、具名初态、完整目标Pose、场景、附着物、collision policy 对拍 | 255文件通过；请求除本地内容ID/时间戳外与隔壁成功基线一致 |
| CPU核心 / cuRobo直接导入 | 22测试通过 / 21组导入通过 |
| GPU连接器 / GPU BIT* | 3/3 / 3/3 |
| 常驻cache复用、场景/负载更新失效、过期规划拒绝、headless无Viser | 全部通过 |
| L24/R20 完整周期两次（冷/热） | 成功；10.303/6.498s；最终Home误差0 |
| 六规划器完整周期 | 6/6成功；相邻关节步长≤0.50001°、最终Home误差0 |
| 龙头车Mesh远离/重叠/关闭与缓存隔离 | 通过 |
| 本分支Viser页面点击“计算完整流程并播放” | 真实重算成功，2053帧，10.723s，最终Home误差0 |
| 实际浏览器阶段检查 | 抽离35cm、携箱转身、释放、最终2053/2053帧均确认，截图已保存 |

六规划器耗时：Informed RRT 10.347s、RRT 6.433s、RRTConnect 6.480s、PRM 6.410s、Informed Connect 6.329s、BIT* 4.242s。耗时仅本机证据，不作跨GPU性能承诺。截图采用独立无头Edge软件WebGL；CUDA规划仍在真实GPU运行，没有改动或停止隔壁8090服务。

`rebase-audit.json`、`cycle.json`、`six-planners.json`、`ui-live-cycle.json`、`ui-check.json`、`viser-{home,extract,carry,release,final-home}.png` 和 `SHA256SUMS` 是当前证据。UI数据来自本轮新计算，不拿隔壁JSON冒充本轮结果。

## 严格边界

**这里只验收与用户指定 f044bf1 研究基线等价的迁移，不宣称 MOTION-233 的25箱全部完成或实机/FCL/OBB认证。**保持上游 `exclude_task_objects_before_contact=true`、`defer_payload_until_extract_end=true`、40个内接负载球、89°倾角规则；不是此前完整三角网格FCL口径。未改成0.75m Stage单箱输入，未更改任务组合L24/R20，未添加RT-Control或ROS Action调用。旧黄金25/25泛化/性能验收若继续迁入，应在此正确核心上单独验证，不能靠转换结论冒认完成。没有push、改PR或写Linear。

## 重新运行与查看

重新验证完整周期（使用当前准备完成的运行副本，不动源码/隔壁环境）：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-f044bf1/runtime/research/curobo_v3
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_plan_cycle.py \
  --left-box 24 \
  --right-box 20 \
  --runs 2 \
  --output /home/astesia/Sevenova/motion233-acceptance/curobo-f044bf1/cycle.json
```

打开已运行的本分支页面（8095；隔壁8090不动）：

```bash
xdg-open http://127.0.0.1:8095
```

若页面进程已退出，在新终端启动：

```bash
cd /home/astesia/Sevenova/motion233-acceptance/curobo-f044bf1/runtime/research/curobo_v3
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/v3_full_cycle_demo.py \
  --port 8095 \
  --comparison-json /home/astesia/Sevenova/motion233-acceptance/curobo-f044bf1/six-planners.json
```

再次核验当前候选提交、未改动的导师模型以及与隔壁基线相同的默认输入：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo
PYTHONPATH=/home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
/home/astesia/Sevenova/curobo_v2_ros/bin/python tools/verify_motion233_curobo_rebase.py \
  --source-ref HEAD \
  --artifacts /home/astesia/Sevenova/motion233-acceptance/curobo-layout-v1 \
  --runtime-root /home/astesia/Sevenova/motion233-acceptance/curobo-layout-v1/runtime/research/curobo_v3 \
  --sdk-repository /home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/curobo-main \
  --reference-cycle /home/astesia/Sevenova/golden_curobo_adapter/mentor-repro-20261006/plan-cycle-pinned-sdk.json
```
