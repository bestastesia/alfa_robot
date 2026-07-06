# 运控工程化护栏

本文记录当前实验期先落地的低风险护栏，以及暂不立刻重构的高风险项。

## 1. MoveIt config 包职责冻结

当前 `alfa_robot_moveit_config` 已经承担大量运控实验逻辑，这是历史原因，不是理想终态。

短期规则：

- 允许保留现有 `dual_arm_planner_node` 作为实验/MoveIt Adapter。
- 新增算法逻辑不要继续直接堆进 `dual_arm_planner_node.cpp`。
- 已稳定算法优先沉淀到独立 Module，再由节点调用。
- 迁移到 `robot_motion_control` 时，再正式按包拆分。

推荐未来包：

- `robot_motion_ik`：IK 候选、去重、cost scorer。
- `robot_motion_extract`：抽离、横向让位、小步 KDL primitive。
- `robot_motion_scene`：场景建模、碰撞对象、PlanningScene Adapter。
- `robot_motion_planning`：负重规划、轨迹候选选择。
- `robot_motion_runtime`：任务状态机、执行前后状态管理。
- `robot_motion_debug`：Rerun、JSONL、诊断工具。

## 2. 参数入口规则

launch 参数可以用于实验开关，但不应该成为算法版本本身。

短期要求：

- 重要实验必须保存完整启动命令和 snapshot。
- 核心参数逐步进入 YAML/manifest。
- 新实验结果应写入 motion baseline id。

风险说明：如果只保存一条很长的 launch 命令，不保存参数版本，后续很难确认两次实验是否真的可比。

## 3. 碰撞真相源

当前硬判定以 MoveIt PlanningScene/FCL 为唯一真相源。

阶段规则：

- IK 后碰撞检查：MoveIt PlanningScene/FCL。
- 抽离每一步碰撞检查：同源 PlanningScene snapshot。
- 负重规划碰撞检查：MoveIt/FCL。
- 执行前最终检查：MoveIt/FCL。

AABB 和 Rerun 的地位：

- AABB 只作为诊断/粗筛，不默认作为硬失败。
- Rerun 只作为解释和复盘，不作为自动通过条件。

如需恢复 AABB 硬失败，可显式启动：

```bash
ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py \
  enforce_loaded_plan_aabb_clearance:=true
```

## 4. 关节命名和方向契约

短期不大规模重命名关节，避免破坏 MoveIt、执行桥、PLC 和历史数据。

但必须使用唯一校验入口：

```bash
ros2 run alfa_robot_moveit_config check_joint_contract.py
```

真实执行前应使用更严格检查：

```bash
ros2 run alfa_robot_moveit_config check_joint_contract.py --require-real-signs
```

如果该检查失败，不应继续上机执行。

## 5. motion baseline

当前新增 motion baseline manifest：

- `ros2_ws/src/alfa_robot_moveit_config/config/motion_baselines/current_motion_baseline.yaml`

生成当前 baseline JSON：

```bash
ros2 run alfa_robot_moveit_config generate_motion_baseline.py \
  --output /tmp/current_motion_baseline.json
```

只打印 baseline id：

```bash
ros2 run alfa_robot_moveit_config generate_motion_baseline.py --print-id
```

后续目标：关键 benchmark / Rerun / JSONL header 自动写入 `baseline_id`，防止旧模型实验数据误解释新模型问题。

## 6. 当前暂不立刻处理的高风险项

以下问题确认存在，但不建议在当前实验期立刻大改：

- 关节统一改名：风险高，影响 MoveIt/执行/PLC/历史数据。
- 硬件总线和 node id 全配置化：电控方案仍在调试，过早抽象可能反而制造漂移。
- 完整生产生命周期管理：当前 Python 脚本仍定位为测试工具，不作为生产入口。
- 替换 BioIK：短期没有低成本替代，当前继续使用多候选 + 去重 + cost scorer。
- 替换 MoveIt/FCL 碰撞：当前仍以 MoveIt/FCL 为唯一硬判定真相源。
