# Pinocchio 参数化机械臂技术栈

## 使用场景

这套工具链用于离线/半自动评估不同机械臂比例、臂长、重量、法兰偏置下的运动学、动力学、可达性和受力表现，不直接替代 ROS2/MoveIt 实机控制链。

## 技术栈职责

- Pinocchio：核心运动学/动力学引擎；读取 URDF，计算 FK、IK 所需 Jacobian、RNEA 逆动力学力矩。
- Jinja2 或 Python 字符串格式化：URDF 模板引擎；动态生成不同臂长、重量、offset 的临时 URDF。
- MeshCat：3D 可视化；结合 `pinocchio.visualize` 在浏览器渲染机器人和受力/方向箭头。
- NumPy / SciPy：矩阵计算和非线性优化；用于基于 Pinocchio 的轻量 IK 求解器、参数扫描、误差评价。
- Matplotlib / Plotly：压测后数据可视化；生成比例对比雷达图、折线图、热力图等。

## 岗位对应

- 机械工程师：负责参数化 URDF 模板、几何拓扑、臂长/offset/重量参数来源、当前机械臂基准参数。
- 仿真学工程师：负责 Pinocchio/MeshCat 工具链、模型加载、可视化、批量实验脚本和结果图。
- 运控工程师：负责 IK/Jacobian/RNEA 指标是否符合后续控制和规划需求，确认 joint 命名与自由度语义。
- Git 操作工程师：负责新工具目录的提交边界，避免提交临时生成 URDF、缓存和大文件。

## 初始原则

- 先做独立工具目录，不直接改 ROS2 主模型。
- 临时生成文件放入可忽略目录，例如 `generated/` 或 `tmp/`。
- 当前机械臂参数作为 baseline，后续比例扫描都相对 baseline 变化。
- 第一阶段验收优先：能生成 URDF、Pinocchio 能加载、MeshCat 能看见模型、参数变化肉眼可见。
