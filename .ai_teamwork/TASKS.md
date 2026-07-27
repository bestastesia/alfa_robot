# 当前任务

## 使用方式

- 当前表只放未完成/需确认的任务；已完成和旧方向统一进 `.ai_teamwork/archive/`。
- 开工前只看自己相关任务；如果任务依赖前置项，先确认前置项是否完成。
- 完成后必须追加 `.ai_teamwork/LOG.md`，并由 PM 把任务移出当前表或标记归档。

## 状态

- TODO：待做
- DOING：进行中
- BLOCKED：被依赖卡住
- DONE：完成后应移入归档
- CANCELED：撤销后应移入归档

## 当前任务列表

| ID | 状态 | 负责人/角色 | 任务 | 范围 | 依赖/备注 |
| --- | --- | --- | --- | --- | --- |
| T-0030 | TODO | 运控工程师 | 实现当前机械臂单臂可达空间批量验证 | 复用 `alfa_robot_moveit_config/scripts/nine_orient_reachability.py`；输入 xyz 范围与间隔；输出 CSV/点云 | 依赖 T-0029（已完成并归档）。仅针对当前机械臂当前 URDF/SRDF；每个点跑 9 朝向，记录全部成功/部分失败/失败原因和耗时。下一步需要用户/PM 确定采样范围和步长。 |
| T-0031 | TODO | 运控工程师 | 可达空间点云与机械臂同场景可视化 | RViz Marker/PointCloud2 或 Rerun；必须加载当前机械臂模型 | 依赖 T-0030。验收采用点云可视化，并且必须同时显示当前机械臂外观；成功点和失败点颜色区分，9 朝向失败可按失败数量渐变。 |
| T-0032 | BLOCKED | 运控工程师 | 校验可达性仿真结果可信度 | 抽样点 IK 解、FK 回代、关节限制、碰撞/是否考虑碰撞的说明 | 依赖 T-0030/T-0031。抽查成功点 9 朝向 FK 误差，确认 IK 求解器和 planning group 正确；明确结果是否考虑碰撞。 |

## 已归档完成项

已完成摘要见：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/COMPLETED_SUMMARY.md`
