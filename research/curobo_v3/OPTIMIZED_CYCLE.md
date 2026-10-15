# 8151研究方案分阶段同步

## 第一阶段：等价龙头车与优先接触候选

`PriorityContactPlanner`保留原5cm Updown/10°冗余角网格、FK/碰撞过滤及64候选上限。按可分解Home成本全局最小堆展开双臂组合，256一批检查，补完第64个的并列成本后结束。`valid_unique_count`仅指已检查部分，不代表全量合法组合。

`CuboidConveyorPlanner`先核验当前冻结STL为8角点/12面且体积与bounds一致，再把bounds中心按实例Pose变换成两份等价cuboid；复杂网格拒绝，不静默外包框近似。机器人球、4内部cuboid和负载合同保持。

正式入口`tools/v3_visualized_task_demo.py`消费此阶段类；8151研究服务保持独立。碰撞场景审计兼容原STL和等价cuboid，逐部件核验尺寸、姿态、角点与实例身份，不能只通过数量断言代替几何验收。

复现：`python tools/v3_priority_regression.py --output-dir /tmp/priority_regression`（从research/curobo_v3运行）。四个任务L22/R17、L18/R16、L9/R1、L4/R2比较优先64与全枚举集合，再验证完整周期和独立FK；任一异常即停止。实验原报告是历史对照，不泛化到全94成功率。

本同步不改前端包、关节装配或碰撞拟合；六份本机模型差异仅checkout路径重定位，不提交绝对路径变化。底盘仍为导航到位瞬移，箱体释放消失，未时间参数化或上机。

## 第二阶段：常驻拓扑检查器

`PersistentCollisionPlanner`按机器人/负载/附着link/mobile模型建立常驻检查器；相同拓扑下箱号、停车位、场景对象增删和尺寸变化通过native `update_world`更新，清除内部几何查询缓存，不缓存旧碰撞判定或路径。

启动预热后还原原任务、snapshot、目标及策略。侧吸/顶吸、空载、mobile或单臂负载拓扑不同仍需分别创建。当前只支持≤40 cuboid且无mesh，串行使用、无长期淘汰策略；不宣称生产并发或无限动态世界支持。

复现：`python tools/v3_persistent_regression.py --cycles --output /tmp/persistent/summary.json`。11个实际参数场景×1024状态与fresh检查器逐项比较，再做四任务完整周期。按用户要求，不加入额外人工mesh负例；实际场景没有mesh。

## 第三阶段：按需抽离与旋转软代价

`LazyExtractionPlanner`每个请求重新生成64候选，固定Updown/ψ一次预测35cm终点；GPU目标距离与旋转探针交替排序后，按8/16/32/64逐批执行原完整解析抽离、密化、碰撞和独立FK直线验收。预测不是可行性证明；同请求已经验收的抽离复用，下个请求清空。

运输扩批预算350/450/600/600ms共享约2秒，不是每批2秒，GPU批次可能略超预算；放箱与回Home各有独立预算。扩批重建树，没有跨批树复用。`lazy_extraction.timings_ms/batches`覆盖预测及全部批次，`pipeline_timing_ms`只描述最终批，不能相加冒充互不重叠计时。

旋转额外角、回退和弧偏离仅作软代价与显示，不再因为旧90°/15°/45°阈值硬拒绝；碰撞、限位、末端完整Pose和抽离验收仍硬检查。旧基类缺省策略保持，正式入口必须导入新类。

复现：`python tools/v3_lazy_regression.py --output-dir /tmp/lazy_regression`。验证L18/R22旧旋转阈值案例以及L18/R16、L9/R1完整周期，记录实际抽离数量、各批预算和成本，并独立FK验收。
