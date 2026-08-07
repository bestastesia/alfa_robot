# Motion 域分阶段运行与联调

## 1. 进程职责

对使用者只有一个 Motion 服务进程。它内部维护长驻 planner、当前任务计划和阶段状态机，负责 IK、碰撞规划、轨迹整形、轨迹下发及阶段回执。

Motion 不控制吸附通路、真空泵和真空阈值，不启动或使能 rt-control。Autonomy 在 Motion 阶段之间直接编排 rt-control。

```text
Autonomy/测试客户端
        |
        | /motion/execute_stage
        v
Motion 服务
  - 阶段状态机
  - 长驻 Planner / PlanningScene
  - 轨迹缓存与实时规划
  - 十四轴轨迹执行
        |
        | /dual_arm_jtc/follow_joint_trajectory
        v
rt-control
        |
        +---- /joint_states ----> Motion

Autonomy -----------------------> rt-control 吸附接口
```

## 2. 公共接口

| ROS 名称 | 类型 | 生产者 → 消费者 |
|---|---|---|
| `/motion/execute_stage` | `alfa_motion_interfaces/action/ExecuteMotionStage` | Autonomy → Motion |
| `/motion/readiness` | `alfa_motion_interfaces/msg/MotionReadiness` | Motion → Autonomy/观测工具 |
| `/dual_arm_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion → rt-control |
| `/joint_states` | `sensor_msgs/msg/JointState` | rt-control → Motion |

Action Goal：

```text
uint8 execution_stage
DualArmPoseTargets targets
```

`DualArmPoseTargets`：

```text
STAGE_TOP_SUCTION=1
STAGE_SIDE_SUCTION=2
STAGE_NO_MOVE=3

uint8 left_stage
geometry_msgs/Pose left_pose
uint8 right_stage
geometry_msgs/Pose right_pose
```

Pose 固定在 `base_link` 下，位置单位米、姿态为归一化四元数。Goal 不包含任务号、箱号、frame、时间戳、关节角、轨迹或 PLC 指令；ROS Action Goal UUID 负责请求身份。

Result 只有 `diagnostic`。成功、失败和取消使用 Action 原生终态；Feedback 只有 `PLANNING/EXECUTING/SETTLING`。

接口源码：

- `ros2_ws/src/alfa_motion_interfaces/action/ExecuteMotionStage.action`
- `ros2_ws/src/alfa_motion_interfaces/msg/DualArmPoseTargets.msg`

## 3. 阶段状态机

阶段必须严格按以下顺序调用：

| 阶段 | Motion 行为 | 成功后 Autonomy 行为 |
|---|---|---|
| `CAMERA_VIEW` | 接收第一对重拍 Pose，执行 turn 对齐、IK、碰撞规划和重拍位轨迹 | 触发感知重拍和精定位 |
| `PREGRASP` | 接收第二对箱体正面中心 Pose，从重拍真实末态计算完整计划并执行到预抓取位 | 请求靠近阶段 |
| `APPROACH` | 执行 5cm 靠近吸附轨迹 | 打开吸附通路并等待真空条件 |
| `PLACE` | 执行抽离、负重过渡、预放置和放置轨迹 | 关闭吸附通路并等待释放条件 |
| `HOME` | 执行放置位到初始位轨迹并清除计划 | 进入下一任务或结束 |

`CAMERA_VIEW/PREGRASP` 必须提供两侧有效目标；`APPROACH/PLACE/HOME` 完全忽略 `targets`。当前完整规划要求双臂都参与，不接受单侧 `NO_MOVE`。

每个 Action 只有在对应轨迹被 rt-control 接受并返回成功后才返回 `SUCCEEDED`。顺序错误、服务忙、场景未知或目标非法时 Goal 被拒绝或返回 `ABORTED`。

## 4. Turn 语义

- 重拍前若实体 `turn` 未到 `-90°`，Motion 先保持另外 13 轴不动，将它转到距离当前角度最近的等价 `-90°`，避免额外整圈旋转。
- IK、碰撞场景和后续规划始终使用虚拟 `turn=0`，算法不感知实体 turn。
- 除专用对齐轨迹外，发送到 rt-control 的每条十四轴轨迹都用最新 `/joint_states` 中的真实 turn 覆盖规划值，因此 turn 静默保持。
- 第二批完整规划的起点来自重拍执行后的真实双臂和 updown 反馈，只有 turn 在传入 planner 前被置零。

## 5. 轨迹缓存

- 当前保存 `0.70～0.75m` 六档距离、五个等高任务共 30 条完整轨迹，距离按厘米向上取整。
- 只有默认横向布局、同排双抓且缓存起点与当前起点一致时命中；重拍末态不同会自动回退实时规划。
- 命中后跳过 IK、抽离和负重规划，但仍按当前速度、加速度和 30Hz 合同重新定时。
- 模型、箱体尺寸、场景、关节合同或算法版本变化后必须重建缓存。

## 6. 原生联调

Mock：

```bash
cd tools/demonstration0720/armmotion
./run_motion_domain.sh --mock
```

另一个终端模拟 Autonomy：

```bash
cd tools/demonstration0720/armmotion
./run_manual_motion_task.sh \
  --recapture-left 0.70 0.40 1.597906 3.1415926 -1.5707963 0 \
  --recapture-right 0.70 -0.40 1.597906 3.1415926 -1.5707963 0 \
  --task B1 --front-distance 0.70 --top-distance 0.70 \
  --interactive --yes-execute
```

实机必须先由 rt-control 自己的受控入口启动并确认 `READY`，再执行：

```bash
./run_motion_domain.sh --hardware
```

测试发送器仅模拟 Autonomy。生产系统直接调用 `/motion/execute_stage`，不启动测试发送器。
`--interactive` 会在每个 Action Goal 前等待回车；靠近后先由外部打开吸附通路并确认真空，放置后先关闭吸附通路并确认释放，再继续下一阶段。

## 7. 当前限制

- `left_stage/right_stage` 已进入并校验公共合同，但现有抓取策略仍主要由箱体正面中心高度推导。
- Gate、安全状态、模型/标定版本强制准入尚未完成。
- Action 取消当前只保证在轨迹段边界收敛，尚未完成生产级中途制动策略。
- 长驻 planner 仍由历史 MoveIt 规划进程承载，是 Motion 内部实现细节，后续可替换而不修改公共 Action。
- Docker 联调说明见 `docker/motion/README.md`。
