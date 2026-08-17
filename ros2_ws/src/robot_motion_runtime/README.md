# robot_motion_runtime

`robot_motion_runtime` 是运控服务化的第一层运行时包，目标是把原来集中在
`dual_arm_planner_node` 周围的长流程拆成可观测的服务图。

## 当前提供的节点

| 节点 | 接口 | 职责 |
| --- | --- | --- |
| `motion_state_source_node` | `/robot_motion/set_state`、`/robot_motion/state` | 生产/仿真统一机器人状态事实源。真实机器人可订阅 `/joint_states`；仿真可先调用 `set_state` 固定初始姿态。 |
| `motion_scene_source_node` | `/robot_motion/set_scene`、`/robot_motion/scene` | 生产/仿真统一场景事实源。仿真可先调用 `set_scene` 固定箱墙、集装箱等碰撞对象；Plan 服务会使用该场景做碰撞检查。 |
| `dual_arm_ik_candidate_service_node` | `/robot_motion/plan_dual_arm_ik` | 调用左右两次 `SolveArmIk`，把目标位姿转成双臂 IK candidate states。 |
| `box_pair_task_adapter_node` | `/robot_motion/run_box_pair_task` | 仅用于旧实验兼容，默认不启动；显式设置 `enable_legacy_box_pair_task_adapter:=true` 才会加载。 |
| `dual_grasp_task_adapter_node` | `/robot_motion/run_dual_grasp_task`、`/robot_motion/task_receipt` | 对外任务入口。整机只传左右箱体正面中心 6D 位姿和执行开关；节点内部识别排数、吸附方式与任务策略，并发布任务回执。 |
| `plan_extract_service_node` | `/robot_motion/plan_extract` | 独立 `PlanExtract` 服务。轻量模式是 deterministic shortcut；完整栈可调用 `/robot_motion/check_collision` 过滤候选轨迹。 |
| `plan_loaded_service_node` | `/robot_motion/plan_loaded` | 独立 `PlanLoaded` 服务。按最近负重姿态族生成 shortcut 轨迹；完整栈可调用碰撞服务过滤候选。 |
| `execute_trajectory_service_node` | `/robot_motion/execute_trajectory` | 把 service 形式的执行请求转成现有 FollowJointTrajectory action，或 dry-run 验证。 |
| `motion_task_orchestrator_node` | `/robot_motion/run_task`、`/robot_motion/run_dual_arm_pose_task` | 编排服务。前者接收 IK candidates；后者接收左右目标位姿并串起 `PlanDualArmIk -> PlanExtract -> PlanLoaded -> ExecuteTrajectory`。 |
| `motion_runtime_dashboard_node` | `http://127.0.0.1:8766` | 运行时前端，展示已启动服务、服务状态、最新机器人/场景事实状态、节点列表和阶段事件。 |

所有服务节点都会发布 JSON 字符串到 `/robot_motion/runtime_status`，前端据此显示每个服务在做什么。

## 启动

### 手动全流程测试：仿真 bringup -> 算法栈 -> ROS 任务 -> Rerun 实时显示

这个流程用于模拟“实体机器已经 bringup，算法只负责接任务并发执行”的状态。

终端 1：启动仿真 bringup。它会发布 `/joint_states`，并提供与生产执行层一致的执行接口：

- action：`/dual_arm_jtc/follow_joint_trajectory`
- topic：`/dual_arm_jtc/joint_trajectory`
- 兼容旧 action：`/alfa_execution/execute_joint_trajectory`

仿真执行器按 250Hz 内部控制周期复现实机 `joint_trajectory_controller` 的 variable-degree spline，
当前带位置和速度的轨迹会使用三次 Hermite 插值；`/joint_states` 默认按 50Hz 发布。

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash

ros2 launch robot_motion_runtime sim_bringup.launch.py \
  initial_updown:=0.0 \
  start_rerun:=true
```

终端 2：启动算法服务栈。它会订阅终端 1 的 `/joint_states` 作为事实状态，并把最终轨迹发给仿真执行器：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash

ros2 launch robot_motion_runtime runtime_full_stack.launch.py \
  subscribe_joint_states:=true \
  execute_forward_action:=true \
  execution_action_name:=/dual_arm_jtc/follow_joint_trajectory \
  execute_wait_for_goal_acceptance:=true \
  execute_wait_for_result:=true \
  execute_resample_before_forward:=true \
  execute_resample_rate_hz:=10.0 \
  task_service_timeout_s:=60.0 \
  plan_check_collision:=true
```

终端 3：发送一次双箱任务。这个接口是给整机/上游调度对接用的正式入口；
核心输入只有左右箱体朝向机器人的正面中心 6D 位姿和执行开关。位姿固定使用
`base_link`，位置单位为米，RPY 单位为弧度，局部 `+Z` 从机器人指向箱内。
算法根据高度映射箱体排数，并在内部生成侧吸或顶吸工具接触位姿。任务状态通过
`/robot_motion/task_receipt` 回执：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash

ros2 service call /robot_motion/run_dual_grasp_task motion_internal_interfaces/srv/RunDualGraspTask "{
  request_id: 'manual_dual_grasp_6_8',
  left: {
    pose_6d: {x: 0.925, y: 0.400, z: 1.197906, roll: 3.14159265359, pitch: -1.57079632679, yaw: 0.0}
  },
  right: {
    pose_6d: {x: 0.925, y: -0.400, z: 1.197906, roll: 3.14159265359, pitch: -1.57079632679, yaw: 0.0}
  },
  execute: true
}"
```

排数识别、吸附坐标换算和完整 C++ 算法边界见
`docs/运控/双抓取6D任务合同与策略.md`。

任务回执监听：

```bash
ros2 topic echo /robot_motion/task_receipt
```

回执状态只面向整机控制，核心状态为 `accepted`、`running`、`succeeded`、`failed`、`cancelled`、`stopped`。
内部 IK/抽离/负重规划调试信息继续留在 `/robot_motion/runtime_status` 和各服务返回值里。

旧箱号任务入口仅保留为显式兼容模式，默认正式栈不会创建该节点。先在启动命令中增加
`enable_legacy_box_pair_task_adapter:=true`，才能使用下面的请求。这个请求会走完整链路：
`RunBoxPairTask -> RunDualArmPoseTask -> PlanDualArmIk -> PlanExtract -> PlanLoaded -> ExecuteTrajectory`。

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash

ros2 service call /robot_motion/run_box_pair_task motion_internal_interfaces/srv/RunBoxPairTask "{
  context: {request_id: 'manual_box_pair_6_8', frame_id: 'base_link', scene_id: 'manual_box_stack', state_id: 'live_joint_states'},
  left_box_id: 6,
  right_box_id: 8,
  left_grasp_mode: 'front',
  right_grasp_mode: 'front',
  box_front_x: 0.925,
  scene_y_shift: -0.4,
  world_to_base_z: 0.202094,
  fixed_updown: 0.3,
  candidate_limit: 8,
  planning_mode: 'shortcut',
  execute: true,
  dry_run: false,
  velocity_scale: 1.0,
  acceleration_scale: 1.0
}"
```

当前已验证的 smoke 是 L6/R8。L1/R3 在 `fixed_updown=0.3` 下目标高度约 1.598m，解析 IK 会判定不可达，不适合作为手动首测任务。
验证时执行器收到的是完整 `当前状态 -> IK/抽离 -> 负重` 轨迹，示例日志为
`points=41 duration=2.000s initial_delta=0.000000 first_updown=0.000 last_updown=0.300`。

可选终端 4：打开服务状态看板：

```bash
xdg-open http://127.0.0.1:8766
```

如果要换真实执行层，先通过 `~/rt-control-current/tools/rt_control_ipc.sh` 完成现场确认，
看到 `READY: rt-control 已启动并完成 /rt/enable。` 后，算法栈默认直接连接生产 action
`/dual_arm_jtc/follow_joint_trajectory`。真实执行层禁止 partial goal，固定顺序为
`right_joint1..right_joint6,left_joint1..left_joint6,turn,updown`；旋转轴单位为 rad，
updown 单位为 m。`/joint_states` 和该 action 都是 ROS/URDF 模型语义，运控不得再做
EtherCAT 方向或零点补偿。`execute_trajectory_service_node` 会按当前反馈补齐未规划轴，
重排为完整 14 轴后发送。

只启动运行时骨架：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash
ros2 launch robot_motion_runtime runtime_services.launch.py
```

启动完整服务图，也就是运行时骨架 + 解析 IK 服务 + 碰撞服务。这个入口会让前端同时看到
`SetRobotMotionScene`、`SolveArmIk`、`PlanDualArmIk`、`RunBoxPairTask`、`PlanExtract`、`PlanLoaded`、`CheckCollision`、`ExecuteTrajectory`、`RunMotionTask`
和 `RunDualArmPoseTask` 的服务状态。
完整栈默认开启 `plan_check_collision:=true`，所以 `PlanExtract/PlanLoaded` 会通过 `/robot_motion/check_collision`
检查生成的候选轨迹：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash
ros2 launch robot_motion_runtime runtime_full_stack.launch.py
```

浏览器打开：

```bash
xdg-open http://127.0.0.1:8766
```

如果当前没有真实 `/joint_states`，先显式固定仿真事实状态：

```bash
ros2 service call /robot_motion/set_state motion_internal_interfaces/srv/SetRobotMotionState "{
  context: {request_id: 'manual_init', frame_id: 'world', scene_id: 'demo', state_id: 'manual:zero'},
  source: 'manual',
  authoritative: true,
  joint_state: {
    name: ['updown','turn','pitch','left_joint1','left_joint2','left_joint3','left_joint4','left_joint5','left_joint6','right_joint1','right_joint2','right_joint3','right_joint4','right_joint5','right_joint6'],
    position: [0.55,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  }
}"
```

如果当前没有真实场景服务，也先显式固定仿真场景。空场景表示只使用 MoveIt 当前 PlanningScene；真实箱墙/集装箱可通过 `scene_objects` 传入：

```bash
ros2 service call /robot_motion/set_scene motion_internal_interfaces/srv/SetRobotMotionScene "{
  context: {request_id: 'manual_scene', frame_id: 'base_link', scene_id: 'manual:empty', state_id: 'manual:zero'},
  source: 'manual',
  authoritative: true,
  scene_objects: [],
  attached_collision_objects: []
}"
```

之后可以直接调用目标位姿任务入口 `/robot_motion/run_dual_arm_pose_task`，也可以手动分段调用
`SolveArmIk -> PlanDualArmIk -> PlanExtract -> PlanLoaded -> ExecuteTrajectory`。

最小 dry-run 任务示例：

```bash
ros2 service call /robot_motion/run_task motion_internal_interfaces/srv/RunMotionTask "{
  context: {request_id: 'demo_task', frame_id: 'world', scene_id: 'demo', state_id: 'manual:zero'},
  seed_state: {name: ['updown','turn','pitch','left_joint1','right_joint1'], position: [0.55,0,0,0,0]},
  ik_candidate_states: [
    {name: ['updown','turn','pitch','left_joint1','right_joint1'], position: [0.3,0,0,0.1,-0.1]}
  ],
  loaded_goal_family: [
    {name: ['updown','turn','pitch','left_joint1','right_joint1'], position: [0.3,0,0,0,0]}
  ],
  candidate_limit: 8,
  planning_mode: 'shortcut',
  execute: true,
  dry_run: true,
  velocity_scale: 1.0,
  acceleration_scale: 1.0
}"
```

目标位姿任务示例：

```bash
ros2 service call /robot_motion/run_dual_arm_pose_task motion_internal_interfaces/srv/RunDualArmPoseTask "{
  context: {request_id: 'pose_task', frame_id: 'base_link', scene_id: 'demo', state_id: 'manual:zero'},
  seed_state: {name: ['updown','turn','pitch','left_joint1','right_joint1'], position: [0.55,0,0,0,0]},
  left_target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.6, y: 0.25, z: 0.8}, orientation: {x: 0.0, y: 0.7071068, z: 0.0, w: 0.7071068}}},
  right_target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.6, y: -0.25, z: 0.8}, orientation: {x: 0.0, y: 0.7071068, z: 0.0, w: 0.7071068}}},
  fixed_updown: 0.55,
  candidate_limit: 8,
  planning_mode: 'shortcut',
  execute: true,
  dry_run: true,
  velocity_scale: 1.0,
  acceleration_scale: 1.0
}"
```

箱号任务示例：

```bash
ros2 service call /robot_motion/run_box_pair_task motion_internal_interfaces/srv/RunBoxPairTask "{
  context: {request_id: 'box_pair_demo', frame_id: 'base_link', scene_id: 'box_stack', state_id: 'manual:zero'},
  seed_state: {name: ['updown','turn','pitch','left_joint1','left_joint2','left_joint3','left_joint4','left_joint5','left_joint6','right_joint1','right_joint2','right_joint3','right_joint4','right_joint5','right_joint6'], position: [0.55,0,0,0,0,0,0,0,0,0,0,0,0,0,0]},
  left_box_id: 1,
  right_box_id: 3,
  left_grasp_mode: 'front',
  right_grasp_mode: 'front',
  box_front_x: 0.925,
  scene_y_shift: -0.4,
  world_to_base_z: 0.202094,
  fixed_updown: 0.55,
  candidate_limit: 8,
  planning_mode: 'shortcut',
  execute: false,
  dry_run: true
}"
```

## 当前边界

- `PlanExtract` 已经独立成服务，完整栈会调用碰撞服务过滤 shortcut 候选；碰撞场景来自请求字段或 `/robot_motion/scene`；但还不是最终 C++ 抽离 rollout。
- `PlanLoaded` 已经独立成服务，完整栈会调用碰撞服务过滤 shortcut 候选；碰撞场景来自请求字段或 `/robot_motion/scene`；RRT/local-RRT 仍需继续从 `loaded_pose_planning` 迁入。
- `RunDualArmPoseTask` 已经把左右目标位姿接进服务链。
- `RunBoxPairTask` 只保留为默认关闭的旧实验 adapter；正式入口只接受左右正面中心 6D 位姿。
- 前端显示 ROS graph 与 runtime_status，不直接控制机器人。

## 本轮验证口径

2026-07-09 验证过的完整 smoke：

- 干净环境重编译：`motion_internal_interfaces`、`alfa_robot_moveit_config`、`robot_motion_runtime` 通过。
- 静态校验：`/usr/bin/python3 -m py_compile src/robot_motion_runtime/robot_motion_runtime/*.py src/robot_motion_runtime/launch/*.py` 通过。
- 前端数据：`node --check docs/system_portal/assets/data.js` 通过。
- 运行验证：隔离 `ROS_DOMAIN_ID=228` 启动 `runtime_full_stack.launch.py subscribe_joint_states:=false execute_forward_action:=false dashboard_port:=8774 ik_root_samples:=360 ik_default_max_solutions:=4 plan_check_collision:=true`。
- 任务验证：依次调用 `/robot_motion/set_state`、`/robot_motion/set_scene`、`/robot_motion/run_box_pair_task`，L1/R3 侧吸 dry-run 成功，返回 `ik=4`、`extract=4`、`loaded=4`。
- 前端验证：`/api/status` 中 `set_state`、`set_scene`、`run_box_pair_task`、`run_dual_arm_pose_task`、`plan_dual_arm_ik`、`solve_arm_ik`、`plan_extract`、`plan_loaded`、`check_collision`、`execute_trajectory` 均 available 且 request_count > 0；`latest_scene.scene_object_count=1`。
