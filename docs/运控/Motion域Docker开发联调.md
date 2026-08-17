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
        | /whole_body_jtc/follow_joint_trajectory
        v
rt-control
        |
        +---- /joint_states ----> Motion

Autonomy -----------------------> rt-control 吸附接口
```

## 2. 公共接口

| ROS 名称 | 类型 | 生产者 → 消费者 |
|---|---|---|
| `/motion/execute_stage` | `robot_motion_interfaces/action/ExecuteMotionStage` | Autonomy → Motion |
| `/motion/readiness` | `robot_system_interfaces/msg/DomainReadiness` | Motion → Autonomy/观测工具 |
| `/whole_body_jtc/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Motion → Native rt-control |
| `/joint_states` | `sensor_msgs/msg/JointState` | rt-control → Motion |

Action Goal：

```text
uint8 execution_stage
DualArmPoseTargets targets
```

`DualArmPoseTargets`：

```text
GRASP_MODE_TOP_SUCTION=1
GRASP_MODE_SIDE_SUCTION=2
GRASP_MODE_NO_MOVE=3

uint8 left_grasp_mode
geometry_msgs/Pose left_pose
uint8 right_grasp_mode
geometry_msgs/Pose right_pose
```

Pose 固定在 `base_link` 下，位置单位米、姿态为归一化四元数。侧吸 Pose 表示实际正面吸附面中心，顶吸 Pose 表示实际顶面吸附面中心。Goal 不包含任务号、箱号、frame、时间戳、关节角、轨迹或 PLC 指令；ROS Action Goal UUID 负责请求身份。

Result 包含 `ok`、结构化 `robot_system_interfaces/ErrorInfo error` 和仅供诊断的 `diagnostic`。成功、失败和取消同时使用 Action 原生终态；Feedback 只有 `PLANNING/EXECUTING/SETTLING`。

接口源码：

- `/mnt/mydisk/ALFA/SevenovaHangzhou/robot_interfaces/robot_motion_interfaces/action/ExecuteMotionStage.action`
- `/mnt/mydisk/ALFA/SevenovaHangzhou/robot_interfaces/robot_motion_interfaces/msg/DualArmPoseTargets.msg`

## 3. 阶段状态机

标准流程按以下顺序调用：

| 阶段 | Motion 行为 | 成功后 Autonomy 行为 |
|---|---|---|
| `CAMERA_VIEW` | 接收第一对重拍 Pose，保留输入姿态并执行 IK、碰撞规划和重拍位轨迹 | 触发感知重拍和精定位 |
| `PREGRASP` | 接收第二对实际吸附面中心 Pose，允许 Motion 标准化抓取姿态，从重拍真实末态计算完整计划并执行到预抓取位 | 请求靠近阶段 |
| `APPROACH` | 执行 5cm 靠近吸附轨迹 | 打开吸附通路并等待真空条件 |
| `PLACE` | 执行抽离、负重过渡、预放置和放置轨迹 | 关闭吸附通路并等待释放条件 |
| `HOME` | 执行放置位到初始位轨迹并清除计划 | 进入下一任务或结束 |

允许在 Motion 空闲且尚未建立任务周期时跳过 `CAMERA_VIEW`，直接发送 `PREGRASP`。此时 Motion 从最新 `/joint_states` 对应的真实状态开始计算完整计划；后续仍严格按 `APPROACH → PLACE → HOME` 执行。已有任务周期内不能再次跳过或乱序调用。

`CAMERA_VIEW/PREGRASP` 至少提供一侧有效目标；若恰好一侧为 `NO_MOVE`，Motion 将有效目标按 `Y取负` 镜像给另一臂，并执行对应的双臂降级方案。观察阶段直接求镜像双臂 IK，抓取阶段按镜像后的双臂目标查找缓存。左右同时 `NO_MOVE` 非法。`APPROACH/PLACE/HOME` 完全忽略 `targets`。

每个 Action 只有在对应轨迹被 rt-control 接受并返回成功后才返回 `SUCCEEDED`。顺序错误、服务忙、场景未知或目标非法时 Goal 被拒绝或返回 `ABORTED`。

## 4. Turn 边界

- Motion 不检测 Turn 是否到达某个角度，不规划 Turn，也没有任何主动改变 Turn 的代码入口。
- IK 和碰撞规划内部使用固定虚拟 `turn=0`；Motion 收到基于实时 TF 树计算的
  `base_link` 目标后，先依据当前 Turn 角度与 `base_link→turn` 变换，将左右 Pose
  整体反变换到 Turn=0 的虚拟模型，再进入 IK 和碰撞规划。这个虚拟值不会成为硬件命令。
- Native rt-control 的 `/whole_body_jtc` 禁止 partial goal，因此完整十四轴消息仍必须包含 `turn`。执行适配器只复制最新 `/joint_states` 中的 Turn 反馈，并固定发送零速度、零加速度，使其保持不动。
- Turn 的目标值、运动时机、到位判断和异常处理全部由其他域负责。

## 5. 轨迹缓存

- 当前保存 `0.70～0.75m` 六档距离、五个等高任务共 30 条“预抓取之后”轨迹，距离按厘米向上取整，箱层按吸附面高度匹配。
- 同排双抓可命中；当前 Y 偏差不参与缓存键。单臂目标镜像后按同一规则命中。
- 重拍末态不要求等于缓存起点。命中后先在完整 2.4m 集装箱场景中精确规划“当前重拍末态 → 缓存预抓取状态”，再复用靠近、抽离、放置和返回轨迹。
- 缓存桥接末点必须精确等于缓存首态；不接受 MoveIt 关节容差导致的提前停止。所有轨迹仍按当前速度、加速度和 30Hz 合同重新定时。
- 模型、箱体尺寸、场景、关节合同或算法版本变化后必须重建缓存。

## 6. 原生联调

首次构建先导入锁定版本的中央接口仓库；各机器目录可以不同，但必须使用同一提交：

```bash
cd ros2_ws
vcs import src < src/dependencies.repos
colcon build --packages-select robot_interfaces_qos robot_system_interfaces robot_motion_interfaces
```

Mock：

```bash
cd tools/demonstration0720/armmotion
./run_motion_domain.sh --mock
```

另一个终端模拟 Autonomy：

```bash
cd tools/demonstration0720/armmotion
./run_manual_motion_task.sh \
  --task B1 --front-distance 0.70 --top-distance 0.70 \
  --interactive --yes-execute
```

单臂输入示例（未提供的右臂自动使用 `NO_MOVE`，Motion 内部镜像为双臂降级）：

```bash
./run_manual_motion_task.sh \
  --recapture-left 0.62 0.40 1.237906 3.1415926 -1.5707963 0 \
  --left 0.72 0.40 1.237906 3.1415926 -1.5707963 0 \
  --interactive --yes-execute
```

实机使用工控机 `/home/ar/rt-control-dev` 中的 Native rt-control，不使用
`~/rt-control-current` Docker 发布副本。Motion 不负责启动或使能控制域；先由负责人确认：

```bash
cd ~/rt-control-dev/robot
./tools/rt_control_native.sh status
```

确认 Native rt-control 已运行、使能，并且
`/whole_body_jtc/follow_joint_trajectory` 存在服务端后，再执行：

```bash
cd ~/motion_domain_current
MOTION_HARDWARE_CONFIRM=ENABLE_MOTION_HARDWARE ./run_motion_domain.sh --hardware
```

测试发送器仅模拟 Autonomy。生产系统直接调用 `/motion/execute_stage`，不启动测试发送器。
`--interactive` 会在每个 Action Goal 前等待回车；靠近后先由外部打开吸附通路并确认真空，放置后先关闭吸附通路并确认释放，再继续下一阶段。

### 30组真实 Action 输入

以下命令从已验证的 `0.70～0.75m × 五排` 缓存读取真实吸附面位姿，并输出完整的
`CAMERA_VIEW→PREGRASP→APPROACH→PLACE→HOME` Goal，不会执行机器人：

```bash
cd ~/motion_domain_current
./run_dump_action_examples.sh \
  --output ~/motion_domain_current/data/motion_action_examples_30.json
```

只查看 `0.72m` 第三排对应的五条可直接粘贴的 `ros2 action send_goal` 命令：

```bash
./run_dump_action_examples.sh --distance-cm 72 --row 3 --format commands
```

这些样例使用公共 Action 的真实四元数和吸附模式，不包含测试任务号或箱号。
单臂诊断时增加 `--single-arm left` 或 `--single-arm right`，未发送的一臂会明确写为
`GRASP_MODE_NO_MOVE=3`，便于验证 Motion 内部的 Y 镜像降级。

## 7. 当前限制

- `left_grasp_mode/right_grasp_mode` 已进入并校验公共合同；抓取策略由实际吸附面 Pose 反推出箱层和箱体几何。
- Gate、安全状态、模型/标定版本强制准入尚未完成。
- Action 取消当前只保证在轨迹段边界收敛，尚未完成生产级中途制动策略。
- 长驻 planner 仍由历史 MoveIt 规划进程承载，是 Motion 内部实现细节，后续可替换而不修改公共 Action。
- Docker 联调说明见 `docker/motion/README.md`。
