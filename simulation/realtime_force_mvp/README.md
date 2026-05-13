# ALFA 实时力学可视化器 — Pinocchio RNEA 版

基于 Pinocchio RNEA 递归牛顿-欧拉逆动力学算法的实时力学仿真与可视化工具。

## 核心特性

- **Pinocchio RNEA** 计算真实逆动力学力矩，包含所有连杆自身重力
- **各电机默认 5kg** 重心在几何中心，覆盖 URDF 中的 inertial 参数
- **tkinter 滑块 GUI** 实时调整所有 15 个关节角度和吸盘质量
- **MeshCat 3D** 可视化机器人姿态 + 力箭头
- **实时信息面板** 显示所有关节 τ_axis、F_axis/normal/side 和 effort 百分比
- **T-0027 校验模式** 验证物理合理性

## 安装依赖

使用 `alfa` conda 环境（已预装 pinocchio, meshcat, numpy）：

```bash
conda activate alfa
```

## 启动实时可视化

```bash
cd /mnt/mydisk/ALFA/alfa_robot
conda activate alfa
python3 simulation/realtime_force_mvp/scripts/realtime_force_visualizer.py
```

启动后同时打开：
- **tkinter 控制台**：拖动滑块调整关节角度和吸盘质量
- **MeshCat 浏览器**：3D 可视化机器人姿态和力方向

所有力矩数据实时刷新，无需手动输入。

## 启动参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--urdf PATH` | URDF 文件路径 | 自动从 xacro 生成 |
| `--motor-mass KG` | 电机质量 (kg) | 5.0 |
| `--validate` | 运行 T-0027 校验模式 | 否 |
| `--no-meshcat` | 禁用 MeshCat 3D 可视化 | 否 |

## 校验模式 (T-0027)

```bash
python3 simulation/realtime_force_mvp/scripts/realtime_force_visualizer.py --validate
```

输出多组位姿下的 RNEA 力矩对比和物理合理性校验报告。

## 仅终端模式（无 GUI）

```bash
python3 simulation/realtime_force_mvp/scripts/realtime_force_visualizer.py --no-meshcat
```

## 力学计算原理

详见 `FORCE_SEMANTICS_MVP.md`。