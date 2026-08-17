# alfa_robot_execution_bridge

ALFA 轨迹插值、关节合同和历史执行兼容工具包。`/alfa_execution/execute_joint_trajectory`
是旧 13 轴兼容接口，不是当前 rt-control 的生产入口；新代码应直接使用完整 14 轴
`/whole_body_jtc/follow_joint_trajectory`。

## Rerun 滑块示教器

该工具订阅权威 `/joint_states`、`/tf` 和 `/tf_static`，同时打开 Tk 滑块窗口和 Rerun。
Rerun 原位模型严格使用 rt-control 的 `base_footprint → base_link → 本体关节` TF 显示
实体状态，不再用本地 FK 冒充实体状态；横向偏移 2m 的滑块目标仍使用本地 URDF FK。
拖动滑块只更新预览，不发布
`/joint_states`，也不会驱动机器人；只有勾选现场安全确认并点击“执行滑块目标”后，才会
向 `/whole_body_jtc/follow_joint_trajectory` 发送完整14轴五次平滑轨迹。

```bash
cd /mnt/mydisk/ALFA/alfa_robot/ros2_ws
colcon build --packages-select alfa_robot_rerun alfa_robot_execution_bridge --symlink-install
source install/setup.bash
ros2 run alfa_robot_execution_bridge joint_teach_pendant
```

也可以直接运行源码旁的包装脚本：

```bash
ros2_ws/src/alfa_robot_execution_bridge/scripts/run_joint_teach_pendant.sh
```

包装脚本在 SSH 登录且 `DISPLAY` 为空时，会自动选择工控机当前本地 X11 桌面并设置
`XAUTHORITY`，Tk 和 Rerun 窗口仍显示在工控机屏幕上。

安全边界：

- 示教器与 Motion 算法线程可独立启动、同时在线，双方不存在启动依赖；
- rt-control 已有活动轨迹时拒绝发送，避免与算法线程或其他客户端同时控制；
- rt-control 的本体 TF 树不完整时拒绝发送；
- 滑块和 Rerun 只用于预览，工具不做 MoveIt 场景碰撞规划；
- 默认旋转轴限速 `10deg/s`、加速度 `10deg/s^2`，Updown 限速和加速度均为 `0.05m/s`；
- 每次执行结束都会自动撤销界面的安全解锁，下一次必须重新确认。

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

## 历史 ros2_control 转发后端

该入口仍用于回放旧环境，不能连接当前禁止 partial goal 的 rt-control 生产控制器。

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

## 当前 rt-control 方向关系

`alfa_robot_execution_bridge/joints.py` 提供 `RT_CONTROL_JOINT_NAMES` 固定顺序。J6 编码器
零点由 rt-control 硬件配置处理；电机方向校准也已统一下沉到 rt-control，因此 Motion
公共边界所有方向符号均为 `+1`，公共边界偏置保持为 0。
armmotion 和 `jog_to_pose` 的所有14轴位置、速度、加速度及反馈现在都必须显式经过
`model_to_rt_control_*()` / `rt_control_to_model_position()`；禁止
调用方绕过该边界直接拼装控制器语义。

`config/*.yaml` 不再复制 `direction_signs`。默认 `apply_direction_signs=false`，表示电控侧
ros2_control / 硬件层已经处理方向；运行时仍使用 `joints.py` 固定关节顺序和语义，禁止
重新加入 Motion 侧方向翻转，避免双重校准。

## Updown 当前合同

Updown 已并入完整 14 轴 FJT，单位为米。`run_jog_to_pose.sh` 的
`--updown-speed-mps` 与 `--updown-acceleration-mps2` 现在用于生成同步轨迹，
不再发布独立 PP 命令。

```bash
/home/ar/lhy_dev/run_jog_to_pose.sh \
  --updown-m 0.15 \
  --updown-speed-mps 0.05 \
  --updown-acceleration-mps2 0.05 \
  --send
```

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
