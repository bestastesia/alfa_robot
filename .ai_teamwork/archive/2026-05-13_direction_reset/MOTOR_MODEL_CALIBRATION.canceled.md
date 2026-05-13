# 电机 STL 标定记录

本文件用于记录 `motor_model/T电机.stl` 在一个明确三维坐标系中的连接点、旋转点和旋转方向。

## 用户已知描述

- 当前默认所有电机都是 T 型电机。
- T 型电机可以理解为两个圆柱体直角连接。
- 圆柱体直径约 10 cm。
- 圆柱体高度约 15 cm。
- 两个圆柱体重合约 10 cm。
- 每个方向大约露出 5 cm。

## 本轮任务范围

只做单个 T 电机标定，不搭完整机械臂。

最重要的结果不是“看起来像不像”，而是在一个 3D 坐标系中确定：

1. 旋转点：joint origin 在哪里。
2. 旋转方向：joint axis 指向哪里。
3. 连接点：parent 端和 child 端分别在哪里。
4. 连接方向：连杆应该沿哪个方向接出去。
5. STL mesh 相对上述坐标系需要怎样平移/旋转。

## 输出文件

- 坐标数据：`.ai_teamwork/motor_calibration/t_motor_calibration_points.json`
- 2D 投影确认图：`.ai_teamwork/motor_calibration/t_motor_calibration_preview.svg`
- 3D/RViz 预览 URDF：`.ai_teamwork/motor_calibration/t_motor_calibration_preview.urdf`

## STL 文件检查

- 文件：`motor_model/T电机.stl`
- 格式：binary STL
- 三角面数量：1084
- 唯一顶点数量：546
- 单位判断：STL 数值单位按 mm 理解；在 URDF 中使用 `scale="0.001 0.001 0.001"` 转为米。
- 原 STL 包围盒 min（mm）：`[-50.0, -50.0, 25.0]`
- 原 STL 包围盒 max（mm）：`[49.970207, 75.0, 150.0]`
- 原 STL 包围盒尺寸（mm）：`[99.970207, 125.0, 125.0]`
- 原 STL 包围盒中心（mm）：`[-0.0148965, 12.5, 87.5]`
- 几何判断：
  - STL 中有一个竖直圆柱，中心线近似为 `x=0, y=0`，Z 范围约 `25..150 mm`。
  - STL 中有一个横向圆柱，中心线近似为 `x=0, z=100 mm`，Y 范围约 `-50..75 mm`。
  - 两个圆柱中心线交点为 `[0, 0, 100] mm`，适合作为候选 joint origin。

## 候选坐标系定义

坐标系名称：`T_motor_joint_frame_v1_candidate`

- 单位：米。
- 原点：两个圆柱中心线交点，即原 STL 坐标 `[0, 0, 100] mm`。
- X 轴：沿圆柱截面横向半径方向；当前 STL 包围盒约 `[-0.05, 0.05] m`。
- Y 轴：沿横向圆柱中心线；`+Y` 指向较长外露端，候选 child 连接方向。
- Z 轴：沿竖直圆柱中心线；候选 joint axis/旋转方向。

在该候选坐标系下，STL 包围盒为：

- min（m）：`[-0.05, -0.05, -0.075]`
- max（m）：`[0.049970207, 0.075, 0.05]`
- size（m）：`[0.099970207, 0.125, 0.125]`

## 三维标定结果候选 A（推荐先验收）

这个候选把“竖直圆柱中心线”作为电机旋转轴。

- joint origin / 旋转点：`[0.0, 0.0, 0.0]`
- joint axis / 旋转方向：`[0.0, 0.0, 1.0]`
- parent 连接点：`[0.0, 0.0, -0.075]`
- parent 连接方向：`[0.0, 0.0, -1.0]`
- child 连接点：`[0.0, 0.075, 0.0]`
- child 连接方向：`[0.0, 1.0, 0.0]`
- mesh origin xyz：`[0.0, 0.0, -0.1]`
- mesh origin rpy：`[0.0, 0.0, 0.0]`
- mesh scale：`[0.001, 0.001, 0.001]`

URDF 使用方式示例：

```xml
<link name="t_motor_link">
  <visual>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package-or-relative-path/to/T电机.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
</link>
```

其中 `t_motor_link` 的 link frame 就是候选坐标系；父级 joint 的 `<origin>` 应放在真实安装位置，`<axis xyz="0 0 1"/>` 表示绕 T 电机竖直圆柱中心线旋转。

## 候选 B（如果真实输出轴在横向圆柱）

如果用户确认实际旋转轴不是竖直圆柱，而是横向圆柱，则采用：

- joint origin / 旋转点：`[0.0, 0.0, 0.0]`
- joint axis / 旋转方向：`[0.0, 1.0, 0.0]`
- parent 连接点：`[0.0, -0.05, 0.0]`
- parent 连接方向：`[0.0, -1.0, 0.0]`
- child 连接点：`[0.0, 0.0, 0.05]`
- child 连接方向：`[0.0, 0.0, 1.0]`

候选 B 暂不作为默认，因为当前任务需要先在 3D 坐标系中给出可确认候选，真实输出轴需要用户看图确认。

## 可视化要求与使用方式

### 查看 2D 投影确认图

直接打开：

```bash
xdg-open .ai_teamwork/motor_calibration/t_motor_calibration_preview.svg
```

图中：

- 黑点：joint origin / 旋转点。
- 蓝色 `+Z`：候选 A 的 joint axis / 旋转方向。
- 绿色 `+Y`：候选 child 连接方向。
- 紫色点/箭头：parent 连接点和 parent 接入方向。
- 橙色点/箭头：child 连接点和 child 接出方向。

### 在 RViz 看 3D 标定模型

可以先用 `check_urdf` 验证：

```bash
source /opt/ros/humble/setup.bash
check_urdf .ai_teamwork/motor_calibration/t_motor_calibration_preview.urdf
```

然后如果本机有 `urdf_tutorial`，可尝试：

```bash
source /opt/ros/humble/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=/mnt/mydisk/ALFA/alfa_robot/.ai_teamwork/motor_calibration/t_motor_calibration_preview.urdf
```

如果没有 `urdf_tutorial`，也可由后续任务把该 URDF 接到项目自己的 RViz display launch。

## 给用户确认的问题

请用户重点确认候选 A 是否符合实际 T 电机机械含义：

1. 黑点 `[0,0,0]` 是否就是旋转点？
2. 蓝色 `+Z` 箭头是否就是旋转轴方向？
3. 紫色 parent 点 `[0,0,-0.075]` 是否是上一节/安装座接入位置？
4. 橙色 child 点 `[0,0.075,0]` 是否是下一节连杆接出位置？
5. 下一节连杆是否应该沿橙色 `+Y` 方向接出去？
6. 如果旋转轴实际应为横向圆柱，请改用候选 B，并说明 parent/child 分别应接哪一端。
7. 当前 STL 姿态是否还需要整体绕 X/Y/Z 转 90 或 180 度？

## 用户确认结论

待用户确认。
