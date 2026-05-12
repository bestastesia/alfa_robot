# ALFA v5 当前机械臂参数化 baseline

任务：T-0015 提取当前机械臂参数化 baseline  
来源模型：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`  
上游 CAD/URDF 来源：`alfa_robot_arm_v5_1` SolidWorks URDF 导出，已合入 `alfa_robot_description/meshes/alfa_robot_arm_v5/`  
单位：m、kg、kg·m²、rad

## 坐标与镜像约定

- 左臂为 baseline，右臂按 `Y=0` 镜像生成。
- 当前主车体中双臂 parent frame 为 `updown`；Pinocchio 独立模型中暂以 `world` 作为 mount parent。
- 左挂点：`[0.0, 0.20, 0.18]`。
- 右挂点：`[0.0, -0.20, 0.18]`。
- `updown` link 的旧 STL 已隐藏并禁碰撞，但 `updown` prismatic joint 和双臂挂载关系保留。

## 关节拓扑

| Joint | Parent -> Child | xyz | rpy | axis | 来源 |
| --- | --- | --- | --- | --- | --- |
| joint1 | link0 -> link1 | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 1.0]` | ROS2 挂载处新增第 1 轴 |
| joint2 | link1 -> link2 | `[-0.0595, 0.0, 0.0735]` | `[1.5708, 0.0, 1.5708]` | `[0.0, 0.0, -1.0]` | v5_1 joint1 |
| joint3 | link2 -> link3 | `[0.0, 0.6, -0.148]` | `[0.0, 0.0, 3.1416]` | `[0.0, 0.0, 1.0]` | v5_1 joint2 |
| joint4 | link3 -> link4 | `[0.0, -0.412, 0.009]` | `[1.5708, -1.5708, 0.0]` | `[-1.0, 0.0, 0.0]` | v5_1 joint3 |
| joint5 | link4 -> link5 | `[0.0595, -0.074, 0.0]` | `[-1.5708, 0.0, 1.5708]` | `[1.0, 0.0, 0.0]` | v5_1 joint4 |
| joint6 | link5 -> link6 | `[-0.06, -0.074, 0.0]` | `[1.5708, 0.0, 0.0]` | `[0.0, 0.0, -1.0]` | v5_1 joint5 |
| tool0_fixed | link6 -> tool0 | `[0.0, 0.0, 0.08]` | `[0.0, 0.0, 0.0]` | fixed | 当前 ROS2 tool frame placeholder |

## 关键机械参数

| 参数 | 值 | 对应字段 | 备注 |
| --- | ---: | --- | --- |
| 中线到单臂挂点距离 | `0.20` | `mount.left_xyz[1]` | 用户要求约 20 cm |
| 挂载高度 | `0.18` | `mount.left_xyz[2]` | 当前 ROS2 baseline |
| 大臂 Y 向长度 | `0.600` | `joint3.xyz_mirror[1]` | v5_1 joint2 origin |
| 小臂 Y 向长度 | `0.412` | `abs(joint4.xyz_mirror[1])` | v5_1 joint3 origin |
| 肩部 X offset | `-0.0595` | `joint2.xyz[0]` | v5_1 joint1 origin |
| 肩部 Z offset | `0.0735` | `joint2.xyz[2]` | v5_1 joint1 origin |
| 肘部 Z offset | `-0.148` | `joint3.xyz_mirror[2]` | v5_1 joint2 origin |
| wrist4 Z offset | `0.009` | `joint4.xyz_mirror[2]` | v5_1 joint3 origin |
| wrist5 X/Y offset | `0.0595 / -0.074` | `joint5.xyz_mirror` | v5_1 joint4 origin |
| wrist6 X/Y offset | `-0.060 / -0.074` | `joint6.xyz_mirror` | v5_1 joint5 origin |
| tool0 Z offset | `0.08` | `tool0_fixed.xyz[2]` | ROS2 placeholder |

## 质量与惯量来源

- `link1..link6` mass/COM/inertia 来自 v5_1 SolidWorks URDF 导出，并已写入 `configs/v5_baseline_stub.yaml`。
- `link0` 是虚拟安装 frame placeholder，质量/惯量仅用于 Pinocchio 加载，不代表真实结构。
- `tool0` 是虚拟末端 frame placeholder，质量/惯量仅用于 Pinocchio 加载。
- 当前 Jinja2 模板只使用 `inertia_diag`；完整惯量张量已保存在 YAML 的 `full_inertia` 字段，供后续模板升级。

## 当前与主 ROS2 模型的差异

- 参数化工具使用 primitive visual，未加载真实 STL。
- 参数化工具根为 `world`，没有主 ROS2 中的 `base_link -> pitch -> turn -> updown` 底座链。
- 当前 baseline 目标是 Pinocchio FK/Jacobian/IK/RNEA 语义校验，不替代实机 URDF。
