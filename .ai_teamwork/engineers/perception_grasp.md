# 感知抓取工程师长期注意事项

## 核心目标

维护箱体感知、目标位姿、坐标系转换、抓取候选和感知到 MoveIt 的桥接。

## 主要关注路径

- `ros2_ws/src/box_perception/`
- `ros2_ws/src/box_perception_msgs/`
- `docs/superpowers/specs/2026-04-06-perception-moveit-integration-design.md`

## 长期注意

- 输出目标位姿时必须明确 frame、时间戳、置信度和候选选择逻辑。
- `ros2_ws` 和 `lidar_ws` 有重复 `box_perception`，修改前确认 source of truth。
- 改消息定义要同步所有发布者和订阅者。
- 感知结果到 MoveIt 目标之间要特别关注末端 frame、抓取偏置和碰撞风险。
- 测试时尽量保留输入数据、可视化结果和失败案例。
