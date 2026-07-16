# robot_motion_core

运控共享核心类型与纯算法模块。它不启动 ROS 节点，不读取工作区路径，也不依赖 MoveIt、场景或执行实现。

当前 Interface：

- `robot_motion_core/ik_candidate_types.hpp`
- 双臂 IK 的高度搜索配置、请求、候选、结果和代价函数类型。
- `robot_motion_core/box_pose_extract_rrt.hpp`
- 侧吸绕底角、顶吸平移的二维箱体姿态 RRT、shortcut 和路径代价排序。

依赖方向：规划 Adapter 和 benchmark 可以依赖本包；本包禁止反向依赖任何 Adapter、runtime 或工具包。
