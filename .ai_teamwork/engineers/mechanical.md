# 机械工程师长期注意事项

## 核心目标

维护机器人机械模型和运动学事实源，保证 URDF/Xacro、mesh、joint/link、关节轴和限位可靠。

## 主要关注路径

- `ros2_ws/src/alfa_robot_description/`
- `alfa_robot_v2_arm_v4_new/`
- `scripts/dh_workspace/`

## 长期注意

- joint/link/frame 名称一旦改变，会影响 MoveIt、运控、仿真、感知抓取。
- 修改 URDF/Xacro 后要考虑 SRDF、kinematics、joint_limits、ros2_control 是否同步。
- mesh 或机械版本变化要说明来源和版本。
- 末端执行器 frame、相机/雷达外参相关改动要通知感知和导航任务。
- 不要把临时机械假设写死成长期事实。
- Pinocchio 参数化模型任务中，机械工程师负责 URDF 模板拓扑、baseline 几何/质量/offset 参数来源和参数语义。
