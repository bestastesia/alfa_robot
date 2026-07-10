# 机械工程师长期注意事项

## 核心目标

维护机器人机械模型和运动学事实源，保证 URDF/Xacro、mesh、joint/link、关节轴和限位可靠。

## 主要关注路径

- `ros2_ws/src/alfa_robot_description/`

## 长期注意

- joint/link/frame 名称一旦改变，会影响 MoveIt、运控和外部系统接口。
- 修改 URDF/Xacro 后要考虑 SRDF、kinematics、joint_limits、ros2_control 是否同步。
- mesh 或机械版本变化要说明来源和版本。
- 末端执行器 frame、相机/雷达外参相关改动要同步接口使用方。
- 不要把临时机械假设写死成长期事实。
