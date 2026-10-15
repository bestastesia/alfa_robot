# 8151研究方案分阶段同步

## 第一阶段：等价龙头车与优先接触候选

`PriorityContactPlanner`保留原5cm Updown/10°冗余角网格、FK/碰撞过滤及64候选上限。按可分解Home成本全局最小堆展开双臂组合，256一批检查，补完第64个的并列成本后结束。`valid_unique_count`仅指已检查部分，不代表全量合法组合。

`CuboidConveyorPlanner`先核验当前冻结STL为8角点/12面且体积与bounds一致，再把bounds中心按实例Pose变换成两份等价cuboid；复杂网格拒绝，不静默外包框近似。机器人球、4内部cuboid和负载合同保持。

正式入口`tools/v3_visualized_task_demo.py`消费此阶段类；8151研究服务保持独立。碰撞场景审计兼容原STL和等价cuboid，逐部件核验尺寸、姿态、角点与实例身份，不能只通过数量断言代替几何验收。

复现：`python tools/v3_priority_regression.py --output-dir /tmp/priority_regression`（从research/curobo_v3运行）。四个任务L22/R17、L18/R16、L9/R1、L4/R2比较优先64与全枚举集合，再验证完整周期和独立FK；任一异常即停止。实验原报告是历史对照，不泛化到全94成功率。

本同步不改前端包、关节装配或碰撞拟合；六份本机模型差异仅checkout路径重定位，不提交绝对路径变化。底盘仍为导航到位瞬移，箱体释放消失，未时间参数化或上机。
