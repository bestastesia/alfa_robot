# V3.0.4 侧装双臂 Demo

V3.0.4 单臂模型复制为左右两臂，安装在 `0.4m × 0.5m × 0.5m` 立方体的左右侧面。两侧 `joint1` 轴线分别布置在立方体两侧，机械臂零位向外展开。

当前范围只包含模型、mock `ros2_control`、MoveIt 手动规划和 KDL 数值 IK，不包含旧车体、升降轴、实机执行和完整抓取流程。

## 只看模型

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
./build_v3_demo.sh
./run_v3_demo.sh
```

可以在 Joint State Publisher 中分别调节左右七个关节，确认安装方向和外观。

## MoveIt 与 KDL 测试

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
./build_v3_moveit_demo.sh
./run_v3_moveit_demo.sh
```

在 RViz 的 MotionPlanning 面板选择 `left_arm` 或 `right_arm`，拖动末端交互球，然后依次点击 `Plan`、`Execute`。两组均使用 `kdl_kinematics_plugin/KDLKinematicsPlugin`。

双臂共用一个14轴 mock轨迹控制器；单臂执行时允许发送对应侧的7轴部分目标，另一侧保持当前位置。

关节范围依次为：J1 ±180°、J2 ±105°、J3 ±180°、J4 ±135°、J5 ±180°、J6 ±120°、J7 ±180°。

模型 Demo 默认使用 `ROS_DOMAIN_ID=78`，MoveIt Demo 默认使用 `ROS_DOMAIN_ID=79`，避免其他测试节点污染 `/robot_description`、`/joint_states` 和 TF。
