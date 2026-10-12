# V3 cuRobo 规划研究

新同事和后续AI先读[完整研究与工程交接](HANDOFF.md)，再阅读本运行说明和当前入口代码。

MOTION-276 的[独立规划核心说明](CORE.md)提供SceneSnapshot/RobotState、版本化场景、
GPU缓存失效、无GUI规划入口和回归命令。现有Viser入口复用同一核心与快照。

正式开发分支：`alfa_v3_curobo`，基于`alfa_v3_dev@d9c330ce`。算法研究跟踪[MOTION-275](https://linear.app/sevenova/issue/MOTION-275)，初步适配证据见已收口的MOTION-238～243。

`tools/`保存已有研究源码；当前完整方案入口是`v3_mixed_placement_demo.py`（我方8140），解析64接触、多根联合搬运、侧吸朝上/顶吸朝下目标及球/cuboid研究合同见[RESEARCH_SYNC.md](RESEARCH_SYNC.md)。旧`v3_full_cycle_demo.py`及初版94任务入口保留作历史对照。`historical_artifacts_manifest.json`记录旧实验路径、大小和SHA256；大型轨迹不导入PR。

当前冻结模型、米制mesh、40球箱体、完整目标Pose、解析源码和主要结果一同保存；description来源`feat/v3-analytic-proxy@071cb954d8c626626c41cdeff599bc451a56c117`。模型是实验代理，不是实机标定。移动配置与固定配置同源，运输使用15维，yaw仅作确定性转身。

## 依赖与运行

当前适配环境：Python3.10、PyTorch2.8+CUDA12.8、Warp1.17.0、pytest8.4.2及官方cuRobo main提交`78fd485fa82d9b9a063fb4985e371814587e666a`；其余依赖包括NumPy/SciPy/trimesh/yourdfpy/viser/PyYAML、g++/Eigen3及CUDA显卡。升级记录、测试和完整命令见[CUROBO_MAIN.md](CUROBO_MAIN.md)。固定提交用于复现，不自动追踪main后续变化。

旧证据基于`4ea77366`；`curobo-v2-local-fixes.patch`仅适用于旧提交。新版官方已包含连接器首碰截断及Warp网格接口修复，不再应用旧算法补丁。回归测试按新版`set_dependencies`初始化，保存在本仓`tests/test_gpu_curobo_compat.py`。没有提交第三方仓库或编译库。

本机完整命令：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/prepare_checkout.py --compiler g++
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/v3_full_cycle_demo.py --port 8090
```

准备命令将冻结路径重定位到当前checkout并编译解析桥；更换机器需提供满足上述依赖的Python环境。`8090`若被旧Demo占用需先停止旧服务或改端口。当前原研究目录继续保留，避免打断用户正在查看的场景。

## 当前流程与验收

cuRobo返回32个接触IK→逐候选固定Updown/ψ解析35cm抽离→首个抽离与Home到接触GPU RRT均成功的候选→携箱到完整6D目标→yaw180→释放箱消失→空载转回→双臂回原Home。三段RRT各使用2秒Anytime预算，没有Shortcut或TrajOpt二次优化。目标X减5cm，朝向严格保留。

当前校正模型首组多次通过，热态完整规划6.44～6.52s、抽离线偏差0.033mm、最终Home误差0；当前模型11/60组批量验收未完成。抽离阶段负载球检测延后至35cm终点；40球是内接近似，不是保守箱体包络，未宣称真实OBB全程验收、动力学/实机/正式Action交付。旧模型结果不能混作当前成功率。

`generated/v3_analytic_071cb95/`保存解析FK/IK一致性、周期及计时JSON；这些包含原路径的历史证据正文保持原样。
