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

该路径会把 MoveIt 里的 `left_v5_joint*` / `right_v5_joint*` 映射成执行接口里的 `left_joint*` / `right_joint*`，并默认带上 `turn` 保持当前值。若规划里有未映射且发生变化的轴，例如 `updown`，会默认拒绝执行，避免静默丢轴。

## 方向关系

`config/*.yaml` 中的 `direction_signs` 记录当前实机方向标定关系。默认 `apply_direction_signs=false`，表示电控侧 ros2_control / 硬件层已经处理方向；如果确认下游没有处理方向，再打开该参数，避免双重翻转。
