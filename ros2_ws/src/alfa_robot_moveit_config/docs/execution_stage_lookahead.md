# 执行阶段预规划说明

## 目标

在双臂箱垛流程执行当前动作时，提前计算后续阶段，减少串行等待时间。

当前实现覆盖两类 lookahead：

- `return_pregrasp`：当前轮 `loaded` 轨迹开始执行后，提前规划 `loaded` 结束到本轮 `return_pregrasp` 的轨迹。
- `next_pick`：当前轮 `loaded` 轨迹开始执行后，提前规划下一轮抓取的 `pregrasp` 状态和 `grasp_ik` 轨迹。

## 开关

默认关闭，测试时通过 launch 参数开启：

```bash
lookahead_return_pregrasp_enabled:=true
lookahead_next_pick_enabled:=true
```

相关文件：

- `src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py`
- `src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`

## 实现要点

- 在 `loaded` 阶段完成规划、正式执行前启动异步 lookahead。
- lookahead 使用独立 `PlanningPipeline`，从当前规划结果推导预测状态，不依赖执行过程中实时关节状态。
- 使用缓存命中前会校验当前状态与预测状态是否一致；不一致则丢弃缓存并回退原串行规划。
- `next_pick` 缓存覆盖下一轮 `pregrasp` 和 `grasp_ik`：
  - `pregrasp` 目前是预测状态到固定预抓取状态的单状态缓存。
  - `grasp_ik` 包含下一轮双臂 IK 选优结果和从预抓取到抓取位的 MoveIt 轨迹。
- 规划结果仍会做场景碰撞复查；缓存不可用时不阻塞主流程。

## 验证命令

启动两轮测试：

```bash
cd /home/alfa/dev5/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p /tmp/dual_arm_next_pick_test/ros_log

ROS_DOMAIN_ID=85 ROS_LOG_DIR=/tmp/dual_arm_next_pick_test/ros_log \
ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py \
  start_move_group:=true \
  execute:=true \
  max_rounds:=2 \
  include_top_suction:=false \
  lookahead_return_pregrasp_enabled:=true \
  lookahead_next_pick_enabled:=true \
  record_trajectories:=false \
  record_jsonl_path:=/tmp/dual_arm_next_pick_test/flow.jsonl \
  planning_time:=4.0 \
  planning_attempts:=4 \
  allowed_start_tolerance:=0.1 \
  velocity_scale:=1.0 \
  acceleration_scale:=1.0 \
  box_front_x:=0.925 \
  scene_y_shift:=-0.4 \
  fixed_updown:=0.3 \
  enable_static_box_obstacles:=false \
  enforce_loaded_plan_aabb_clearance:=false
```

触发服务：

```bash
cd /home/alfa/dev5/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_DOMAIN_ID=85 \
ros2 service call /dual_arm_planner/run_box_stack_flow std_srvs/srv/Trigger '{}'
```

检查日志：

```bash
rg -n "started next_pick|using next_pick|using return_pregrasp|Box-stack flow finished|MoveIt planning failed|ERROR" \
  /tmp/dual_arm_next_pick_test/ros_log/*dual_arm_planner_node*.log
```

## 验收证据

已跑通 `max_rounds=2`，日志显示：

```text
Lookahead planning pipeline ready ... return_pregrasp=true next_pick=true
[round_1_L2_R4/loaded] started next_pick lookahead for round_2_L7_R9
[round_2_L7_R9/pregrasp] using next_pick pregrasp lookahead total_wall=1401.8ms
[round_2_L7_R9/grasp_ik] using next_pick grasp lookahead points=41 total_wall=1401.8ms ik_wall=1310.9ms plan_wall=90.4ms
Box-stack flow finished
```

该次流程时间：

- 开始：`1782912157.993255117`
- 结束：`1782912190.417780356`
- 总耗时：约 `32.42s`

对比测试：

- 串行基线：`lookahead_return_pregrasp_enabled=true`、`lookahead_next_pick_enabled=false`，两轮约 `35.14s`
- 当前方案：`lookahead_return_pregrasp_enabled=true`、`lookahead_next_pick_enabled=true`，两轮约 `34.44s`
- 单次 A/B 净收益约 `0.70s`
- 第二轮 `pregrasp + grasp_ik + grasp plan + grasp execute` 局部从约 `5.99s` 降到约 `4.04s`

全流程收益小于被隐藏的 `1.4s` 计算量，主要因为 OMPL 路径点数和执行时长存在随机波动。

## 已知现象

- `No 3D sensor plugin(s) defined for octomap updates` 是当前 MoveIt 环境未配置 3D 传感器的日志，不影响本次静态障碍测试。
- `Failed to fetch current robot state` 来自测试环境 `/joint_states` 时间戳为 `0.000000`，当前流程依赖 last commanded state 仍可继续执行；实机或更严格仿真应修正 joint state 时间戳。
- `Publisher already registered for provided node name` 是同名 logger/node 注册警告，不影响本次流程。

## Linear 评论建议

```md
进展：执行阶段预规划已接入双臂箱垛流程；`loaded` 执行窗口内可提前计算本轮 return_pregrasp，以及下一轮 pregrasp/grasp IK+路径。
证据：`src/alfa_robot_moveit_config/docs/execution_stage_lookahead.md`；两轮测试命中 `started next_pick`、`using next_pick pregrasp/grasp`，并 `Box-stack flow finished`。
验证：`max_rounds=2`、`lookahead_return_pregrasp_enabled=true`、`lookahead_next_pick_enabled=true` 跑通；下一轮隐藏计算量约 1401.8ms。
遗留：A/B 全流程收益受 OMPL 随机路径与执行时长波动影响；实机前建议修 `/joint_states` 时间戳告警。
```
