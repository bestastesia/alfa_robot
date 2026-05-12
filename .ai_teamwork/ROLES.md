# 角色速查

只在任务涉及某个岗位时阅读对应小节；如果要长期承担该岗位，请继续读对应工程师文件。

## 工程师长期注意事项文件

- 项目经理：`.ai_teamwork/engineers/project_manager.md`
- Git 操作工程师：`.ai_teamwork/engineers/git_ops.md`
- 运控工程师：`.ai_teamwork/engineers/motion_control.md`
- 机械工程师：`.ai_teamwork/engineers/mechanical.md`
- 仿真学工程师：`.ai_teamwork/engineers/simulation.md`
- 雷达 SLAM 导航工程师：`.ai_teamwork/engineers/slam_navigation.md`
- 电控工程师：`.ai_teamwork/engineers/electrical.md`
- 感知抓取工程师：`.ai_teamwork/engineers/perception_grasp.md`

## 项目经理

负责梳理当前目标、拆小任务、汇总 AI 进展、发现冲突并提醒用户。

## Git 操作工程师

负责分支、提交、冲突、回滚、未归属改动检查。未经用户要求不要主动提交。

## 运控工程师

主要看 `ros2_ws/src/alfa_robot_hardware/`、`ros2_ws/src/alfa_robot_bringup/`、`ros2_ws/src/alfa_robot_moveit_config/`，关注 ros2_control、controller、实机控制、安全停机、joint command/state、MoveIt/RViz 调试链路。

## 机械工程师

主要看 `ros2_ws/src/alfa_robot_description/`、`alfa_robot_v2_arm_v4_new/`、`scripts/dh_workspace/`，关注 URDF/Xacro、mesh、joint/link、关节轴、限位、DH 参数。

## 仿真学工程师

主要看 `simulation/`，以及和仿真相关的 URDF、MoveIt、RViz/Gazebo/MuJoCo 配置，关注实机前验证。

## 雷达 SLAM 导航工程师

主要看 `lidar_ws/src/`、`ros2_ws/src/fast_lio/`、`ros2_ws/src/livox_ros_driver2/`，关注 Livox、Fast-LIO、2D 建图、AMCL/Nav2、TF、地图。

## 电控工程师

主要关注电机协议、CAN/CANopen/ZeroErr/Cylinder、限位、急停、上电下电、安全策略；源码通常和运控工程师共同看 `alfa_robot_hardware`。

## 感知抓取工程师

主要看 `ros2_ws/src/box_perception/`、`ros2_ws/src/box_perception_msgs/`，关注箱体检测、目标位姿、坐标系、感知到 MoveIt 的桥接。
