# alfa_robot_execution_bridge

统一的 ALFA 机器人执行接口包。当前支持 `mock` 和 `ros2_control` 两种后端：前者用于无电机、无 EtherCAT 主栈时调通上层任务编排，后者把统一 action 转发给电控侧已有的 ros2_control 轨迹控制器。

## 接口

- Action server: `/alfa_execution/execute_joint_trajectory`
- Action type: `control_msgs/action/FollowJointTrajectory`
- State topic: `/joint_states`
- Joint order:
  - `left_joint1` ~ `left_joint6`
  - `right_joint1` ~ `right_joint6`
  - `turn`

发送的轨迹必须包含完整 13 个关节，位置单位是 `rad`，目标含义是 ROS 机械零位坐标下的绝对关节位置。

## 轨迹插值语义

`mock` 后端默认按与工控机 EtherCAT 控制器一致的 `250Hz` 控制周期执行，并复现
`joint_trajectory_controller` 的 variable-degree spline 规则：只有位置时线性插值，位置和速度齐全时三次
Hermite 插值，位置、速度和加速度齐全时五次插值。公共实现位于
`alfa_robot_execution_bridge/trajectory_interpolation.py`，数字孪生执行器也复用该实现，避免本地回放与实机采用不同插值口径。

## 运行 mock

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
colcon build --packages-select alfa_robot_execution_bridge --symlink-install
source install/setup.bash
ros2 launch alfa_robot_execution_bridge execution_bridge.launch.py
```

另一个终端发送测试轨迹：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash
ros2 run alfa_robot_execution_bridge send_mock_trajectory --duration-s 4 --amplitude-deg 15
```

## 运行 ros2_control 转发后端

先启动电控侧 EtherCAT 主栈和 ros2_control，使 `/dual_arm_trajectory_controller/follow_joint_trajectory` 可用，然后运行：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
source install/setup.bash
ros2 launch alfa_robot_execution_bridge execution_bridge.launch.py \
  config_file:=install/alfa_robot_execution_bridge/share/alfa_robot_execution_bridge/config/ros2_control_bridge.yaml
```

此时上层仍然只发 `/alfa_execution/execute_joint_trajectory`。

## dual_arm_planner 丐版接入

`dual_arm_planner` 可以不走 MoveIt execute，改为把规划出的 `JointTrajectory` 发送到本包统一 action：

```bash
ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py \
  execution_backend:=alfa_execution_bridge \
  execution_action_name:=/alfa_execution/execute_joint_trajectory
```

该路径会把 MoveIt 里的 `leftjoint*` / `rightjoint*` 映射成执行接口里的 `left_joint*` / `right_joint*`，并默认带上 `turn` 保持当前值。若规划里有未映射且发生变化的轴，例如 `updown`，会默认拒绝执行，避免静默丢轴。

## 方向关系

`alfa_robot_execution_bridge/joints.py` 是当前实机方向标定的唯一真相源，包含 joint 顺序和 `ROS/Rerun -> EtherCAT` 方向映射。

`config/*.yaml` 不再复制 `direction_signs`。默认 `apply_direction_signs=false`，表示电控侧 ros2_control / 硬件层已经处理方向；如果确认下游没有处理方向，再打开该参数，运行时会自动使用 `joints.py` 中的方向表，避免双重翻转和配置漂移。

## updown 四字段命令合同

生产话题保持为 `/canopen/updown_position_controller/commands`，消息固定为：

```text
[position_m, velocity_mps, acceleration_mps2, deceleration_mps2]
```

旧的单元素 `[position_m]` 不再有效。`updown.py` 是运控侧构造和校验该消息的唯一入口；
位置仍先经过 `joints.py` 的逻辑值到电机值换算。`run_jog_to_pose.sh` 可通过
`--updown-speed-mps`、`--updown-acceleration-mps2`、`--updown-deceleration-mps2`
设置 profile，默认均为 `0.05`。

```bash
/home/ar/lhy_dev/run_jog_to_pose.sh \
  --updown-m 0.15 \
  --updown-speed-mps 0.05 \
  --updown-acceleration-mps2 0.05 \
  --updown-deceleration-mps2 0.05 \
  --send
```

全流程执行脚本不再按 10Hz 连续重发 PP 位置点，而是在每个阶段开始时原子下发一次最终位置和
profile 参数；速度根据该阶段位移/时长动态计算，并受 `--max-updown-speed-m-s` 上限约束。

## jog_to_pose PLC IO 联调

工控机联调入口 `/home/ar/lhy_dev/run_jog_to_pose.sh` 支持在运动前或运动成功后调用
`plc_node` 的三路 `SetBool` 服务。三个输出默认都是 `keep`，未传参数时不会修改 PLC 输出。

只测试 PLC，不运动：

```bash
/home/ar/lhy_dev/run_jog_to_pose.sh \
  --plc-only \
  --left-solenoid on \
  --right-solenoid on \
  --vacuum-pump on \
  --send
```

全部关闭：

```bash
/home/ar/lhy_dev/run_jog_to_pose.sh \
  --plc-only \
  --left-solenoid off \
  --right-solenoid off \
  --vacuum-pump off \
  --send
```

在轨迹成功完成后再打开阀和泵：

```bash
/home/ar/lhy_dev/run_jog_to_pose.sh \
  <原有位置参数> \
  --left-solenoid on \
  --right-solenoid on \
  --vacuum-pump on \
  --plc-when after \
  --send
```

`--plc-when before` 表示运动前切换；默认 `after` 只在轨迹 action 成功后切换。
未指定的输出保持不变，脚本不会根据电磁阀状态自动推导真空泵状态。
