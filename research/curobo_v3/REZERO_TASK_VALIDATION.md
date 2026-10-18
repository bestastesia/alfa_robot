# 新零位、J6反向与新Home全链路验证

模型与角度合同见ZERO_MODEL_CANDIDATE.md。J6仅反轴，零位不移动；Home左J6=+40°、右=-40°。Updown沿用-0.3m。

六组双臂完整成功：L18/R22侧吸，L7/R5、L9/R1、L4/R5、L8/R7、L4/R2顶吸；约2.58～3.15s/组，预热不计入。包含预接触、3cm接近、吸附、35cm抽离、收束运输、导航到位瞬移、RRT放箱、释放消失、直接回新Home。底盘移动仍由导航模拟到位，不是底盘轨迹控制。

该批使用已提交SingleArmPlanner版本注入，避免工作区其他AI未提交的闲臂优化影响验收。RezeroTaskPlanner直接继承SingleArmPlanner，自己定义顶吸TCP1.2m和放箱箱心1.082m，不再依赖未提交的AdaptiveRaised模块或它的配置。细验重试共享原2秒预算，不放宽碰撞。只验证上述六组双臂，不宣称119组、混合吸附、单臂或实机均通过。

独立审计包括：原球模型/底盘cuboid/箱墙/双龙头车cuboid碰撞、限位、相邻机械臂帧≤0.5°、机械臂段底盘冻结、URDF FK计算世界-X抽离35cm、最终箱子6D目标与新Home。保留原门禁：最后3cm接近排除目标箱体；抽离全段检查空载机器人及负载终点，运输/放箱全段检查负载。没有将其称为抽离全段负载精检。

数据：/tmp/rezero_j6_tasks_20261018/，日志/tmp/rezero_j6_full_cycle.log，解析报告generated/zero_model_candidate_20261018/analytic_validation.json，摘要task_regression.json。旧J6符号的数据/tmp/rezero_tasks_20261018不可配新URDF回放。

## 实时规划界面

在本机既有cuRobo/前端环境运行；新检出先按README运行prepare_checkout.py重定位资产并编译解析库，不提交.so。需要修改路径时，请保留所有参数使用同一模型版本。

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python research/curobo_v3/tools/v3_rezero_task_demo.py \
  --port 8152 \
  --robot-config /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/zero_model_candidate_20261018/fixed.yaml \
  --mobile-robot-config /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/zero_model_candidate_20261018/mobile.yaml \
  --urdf /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf \
  --named-poses /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/zero_model_candidate_20261018/named_poses.yaml
```

## 重算六组并复核

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python research/curobo_v3/tools/v3_rezero_task_smoke.py \
  --output-dir /tmp/rezero_j6_tasks_20261018
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python research/curobo_v3/tools/v3_rezero_task_audit.py \
  --source-dir /tmp/rezero_j6_tasks_20261018
```

生成器负责从旧资产创建新坐标模型；正常规划只读取已冻结的两份新坐标候选库。重生成候选库需要原始研究资产，不能对已转换库再次转换。

本次纯离线。没有修改工控机模型、使能电机或发布实机控制指令。
