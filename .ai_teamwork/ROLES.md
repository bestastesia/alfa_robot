# 角色速查

只在任务涉及某个岗位时阅读对应小节；如果要长期承担该岗位，请继续读对应工程师文件。

## 工程师长期注意事项文件

- 项目经理：`.ai_teamwork/engineers/project_manager.md`
- Git 操作工程师：`.ai_teamwork/engineers/git_ops.md`
- 运控工程师：`.ai_teamwork/engineers/motion_control.md`
- 机械工程师：`.ai_teamwork/engineers/mechanical.md`
- 电控工程师：`.ai_teamwork/engineers/electrical.md`

## 项目经理

负责梳理当前目标、拆小任务、汇总 AI 进展、发现冲突并提醒用户。

## Git 操作工程师

负责分支、提交、冲突、回滚、未归属改动检查。未经用户要求不要主动提交。

## 运控工程师

主要看 `robot_motion_interfaces`、`robot_motion_runtime`、`robot_motion_scene_service`、运动算法与规划适配、执行桥和硬件包，关注唯一事实源、规划、执行、安全停机及 joint command/state 一致性。

## 机械工程师

主要看 `ros2_ws/src/alfa_robot_description/`，关注 URDF/Xacro、mesh、joint/link、关节轴、限位和模型版本。

## 电控工程师

主要关注电机协议、CAN/CANopen/ZeroErr/Cylinder、限位、急停、上电下电、安全策略；源码通常和运控工程师共同看 `alfa_robot_hardware`。

## 外部系统协作

感知、导航和高保真仿真不再由本仓库维护。它们必须通过 `robot_motion_interfaces` 的稳定契约接入，不能把实现代码重新塞回运控包。
