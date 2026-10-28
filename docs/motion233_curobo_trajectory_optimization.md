# MOTION-288 固定基线轨迹的时间参数化与RNEA动力学约束

## 当前合同

生产链路为：

1. Mentor生成并验收接触/放置IK、RRT稀疏路点、解析接近/抽离段和0.5°密化轨迹。
2. 仅对已验收的稀疏RRT路点执行一次0.5°碰撞Shortcut，形成最终几何基线；不再扫描数百个密化帧。
3. RRT阶段沿基线直边做逐路点停稳时间化；解析接近/抽离段保留既有2mm容差C2时间化并验碰。
4. 默认不运行改变几何的cuRobo TrajOpt；`--enable-trajopt`仅用于实验对比。
5. 时间化结果生成模型、场景、任务、载荷和几何hash证书；RNEA运行时核验证书，不重复整条碰撞复验。
6. RNEA包含自重、载荷质量/COM/惯量、速度、加速度、科氏/离心项。静态重力超限直接拒绝；动态超限优先使用C2局部减速窗，失败时回退整阶段统一减速。
7. 动力学阶段不得改变逐帧`q`，必须满足`geometry_hash_before == geometry_hash_after`和`maximum_position_delta == 0`。
8. 每轮真实端到端计算时间从几何规划开始计到控制器数据完成，硬门禁为5秒；超时结果可回放，但不得标记`controller_ready`。
9. 控制器导出为严格递增、rest-to-rest的`q/qdot/qddot/jerk/RNEA-required-torque/time`，支持局部减速产生的非均匀时间间隔。

这仍是代理刚体模型级研究门禁，不等于实机闭环力矩认证。摩擦、减速器效率/回差、电气热模型和吸盘柔性仍需台架标定。

## 复现

```bash
cd /home/astesia/Sevenova/.motion-233-curobo-trajopt
MAKEFLAGS='-j8 -l12' CMAKE_BUILD_PARALLEL_LEVEL=8 \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/prepare_checkout.py --compiler g++

PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_timed_wall_sequence.py \
  --output-dir /home/astesia/Sevenova/visualizations/motion288-fixed-path-local-retime-20261028-4 \
  --payload-mass-kg 10 \
  --torque-limit-scale .9

PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_export_controller_trajectory.py \
  --input /home/astesia/Sevenova/visualizations/motion288-fixed-path-local-retime-20261028-4/full-wall.json \
  --output /home/astesia/Sevenova/visualizations/motion288-fixed-path-local-retime-20261028-4/controller-trajectory.json

PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_verify_dynamic_pipeline.py \
  --result /home/astesia/Sevenova/visualizations/motion288-fixed-path-local-retime-20261028-4/full-wall.json \
  --controller /home/astesia/Sevenova/visualizations/motion288-fixed-path-local-retime-20261028-4/controller-trajectory.json
```

## 2026-10-28验收证据

- 15/15轮、25/25箱成功；固定原搜索seed，不降低3mm场景余量或碰撞门禁。
- TrajOpt默认关闭：0次尝试；90个控制器运动段全部导出。
- 全墙端到端计算41.820秒，平均2.788秒/轮；最慢一轮3.611秒，15轮全部低于5秒硬门禁。
- 时间参数化平均约100ms/轮，RNEA平均约46ms/轮。
- 动力学前后最大位置变化0；10kg峰值持续力矩利用率67.09%，本次无需触发减速。
- 名义动作时间910.775秒；相对无TrajOpt基线897.260秒增加1.51%，来源是3mm场景余量下更保守的稀疏Shortcut。
- Core测试61项、GPU测试10项通过，产品门禁通过。

## Viser

```bash
PYTHONPATH=research/curobo_v3/tools \
/home/astesia/Sevenova/curobo_v2_ros/bin/python \
  research/curobo_v3/tools/v3_timed_task_demo.py \
  --port 8090 \
  --sequence-json \
    /home/astesia/Sevenova/visualizations/motion288-fixed-path-local-retime-20261028-4/full-wall.json
```
