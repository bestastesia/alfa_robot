# MOTION-233 RRT → cuRobo TrajOpt → RNEA 动力学轨迹产品

## 当前合同

当前完整链路为：

1. Mentor 规划器生成接触/放置 IK、RRT 路径和解析笛卡尔接近/抽离段。
2. 对 RRT 阶段执行现有碰撞 Shortcut。
3. `approach/transport/conveyor_place/return_home` 将 Shortcut 路径作为 cuRobo V2 `TrajectoryOptimizer.solve_cspace()` 的 seed。
4. TrajOpt 输出必须重新通过项目 `GpuValidity`、地面、携箱和导数限制；失败时回退现有已验收的 cubic/stops/polyline 路径，不降低整墙成功率。
5. `contact_approach/extract` 保持解析笛卡尔几何，只做 C2/五次时间化，避免破坏吸盘法向和抽离走廊。
6. TrajOpt 输出后冻结 `q`。RNEA 动力学阶段只能调整局部时间，必须满足 `geometry_hash_before == geometry_hash_after` 且 `maximum_position_delta == 0`。
7. 动力学模型包含连杆/吸盘自重、箱体质量/COM/惯量、速度、加速度、科氏/离心项；静态重力超限直接拒绝，动态超限可局部减速。
8. 控制器导出只包含严格递增、rest-to-rest 的运动段，移除零时长 attach/release 和轮次间导航瞬移；输出 `q/qdot/qddot/jerk/RNEA-required-torque/time`。

这仍是代理刚体模型下的研究控制产品，不等于实机闭环力矩安全认证。摩擦、减速器损耗/回差、电气热限制和吸盘柔性仍需台架标定。

## 复现

先重定位工作树模型并编译解析 IK：

```bash
cd /home/astesia/Sevenova/.motion-233-curobo-trajopt
MAKEFLAGS='-j8 -l12' CMAKE_BUILD_PARALLEL_LEVEL=8 \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/prepare_checkout.py --compiler g++
```

生成 10 kg/工作手的完整 TrajOpt + RNEA 产品：

```bash
PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_timed_wall_sequence.py \
  --output-dir /home/astesia/Sevenova/visualizations/mentor-trajopt-dynamics-10kg \
  --payload-mass-kg 10 \
  --torque-limit-scale .9
```

导出控制器轨迹：

```bash
PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_export_controller_trajectory.py \
  --input /home/astesia/Sevenova/visualizations/mentor-trajopt-dynamics-10kg/full-wall.json \
  --output /home/astesia/Sevenova/visualizations/mentor-trajopt-dynamics-10kg/controller-trajectory.json
```

执行产品门禁：

```bash
PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_verify_dynamic_pipeline.py \
  --result /home/astesia/Sevenova/visualizations/mentor-trajopt-dynamics-10kg/full-wall.json \
  --controller /home/astesia/Sevenova/visualizations/mentor-trajopt-dynamics-10kg/controller-trajectory.json
```

## 验证证据

### PR #65 HEAD `a77abd5`（2026-10-26完整产品回归）

- 15/15 轮、25/25 箱成功。
- 60 个 RRT 阶段尝试 TrajOpt：33 段采纳，27 段因求解失败或 Mentor 碰撞复验失败而安全回退。
- 90 个运动阶段全部通过时间和动力学门禁。
- 动力学前后 15/15 轮几何 hash 一致，最大位置变化为 0。
- 速度/加速度/jerk 最大限制比例：0.9851 / 0.9867 / 0.0443。
- 10 kg 峰值持续力矩利用率 67.44%，无动力学重定时、无力矩失败。
- 名义动作时间：基线 897.260 s，新链路 815.870 s。
- 控制器导出 90 个严格递增、rest-to-rest 段，包含逐帧位置、速度、加速度、jerk、RNEA需求力矩和重力力矩（力矩不可直接作为实机指令）。

### 最新基线 `3e0ec15` 完整产品复核（2026-10-27）

- 冻结代理URDF、mobile URDF、机器人配置和Home命名姿态与PR #65 HEAD逐字节一致；这里没有切换运动学模型。
- 第10轮原固定种子`20261013`在新增3mm输送线外部场景间隙策略下，64条最佳密化候选全部因左右附着箱代理球互碰而拒绝，最大穿透约2.97mm；没有环境、地面或关节限位碰撞。
- 保留完整3mm安全策略和所有碰撞门禁，仅在固定主种子失败后以固定备用种子`20261014`重试；第10轮在1.248s内得到并密化验收通过11路点路径。第13、14轮也使用同一确定性备用策略。
- 最新基线完整回归：15/15轮、25/25箱，60段尝试TrajOpt，31段采纳、29段安全回退，90段控制器轨迹，动力学前后几何最大变化0。
- 10kg/手名义动作时间836.127s；完整规划与优化耗时318.599s。
- Core 60项、GPU cuRobo compatibility 3项通过（提交前最终运行）。

## Viser 对比

```bash
PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_timed_task_demo.py \
  --port 8090 \
  --result /home/astesia/Sevenova/visualizations/mentor-rebased-timed/round-*.json \
  --sequence-json \
    /home/astesia/Sevenova/visualizations/mentor-rebased-timed/full-wall.json \
    /home/astesia/Sevenova/visualizations/mentor-trajopt-dynamics-10kg/full-wall.json
```

浏览器打开 `http://127.0.0.1:8090`，使用“回放版本”切换基线和新链路；所选关节会同时显示 `q/qdot`、当前 RNEA 总力矩和重力力矩。
