# ALFA Robot IK 求解服务

## 求解模式

| 模式 | 规划组 | 总关节数 | 关节列表 | 求解器 |
|------|--------|----------|----------|--------|
| 左臂 | `left_arm_with_base` | 1+6 | updown, leftjoint1-6 | KDL |
| 右臂 | `right_arm_with_base` | 1+6 | updown, rightjoint1-6 | KDL |
| 双臂 | `dual_arm_with_base` | 1+6+6 | updown, leftjoint1-6, rightjoint1-6 | BioIK |

- 每臂 6 DOF (joint1-6)，updown 为共用升降关节
- 位姿参考系: `base_link`
- 末端链接: 左臂 → `leftjoint6_link`，右臂 → `rightjoint6_link`
- 双臂模式需同时提供两个末端目标位姿

## ROS2 Service 接口

**服务名**: `/ik_solve`
**消息类型**: `alfa_robot_ik_service/srv/IkSolve`

### Request

| 字段 | 类型 | 说明 |
|------|------|------|
| `group_name` | string | 规划组名: `left_arm_with_base` / `right_arm_with_base` / `dual_arm_with_base` |
| `targets` | Pose[] | 目标位姿列表 (单臂1个, 双臂2个，顺序: 先左后右) |
| `timeout` | float64 | 求解超时(秒)，默认2.0 |

### Response

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否求解成功 |
| `joint_names` | string[] | 关节名列表 |
| `joint_values` | float64[] | 关节角/位移值 (与 joint_names 一一对应) |
| `solve_ms` | float64 | 求解耗时(ms) |
| `pos_error` | float64 | 位置误差(m) |
| `ori_error` | float64 | 姿态误差(rad) |

### 示例

```bash
# 左臂 IK
ros2 service call /ik_solve alfa_robot_ik_service/srv/IkSolve \
  "{group_name: 'left_arm_with_base', targets: [{position: {x: 1.0, y: 0.3, z: 1.5}, orientation: {w: 1.0}}], timeout: 2.0}"

# 双臂 IK
ros2 service call /ik_solve alfa_robot_ik_service/srv/IkSolve \
  "{group_name: 'dual_arm_with_base', targets: [{position: {x: 1.0, y: 0.5, z: 1.2}, orientation: {w: 1.0}}, {position: {x: 1.0, y: -0.5, z: 1.2}, orientation: {w: 1.0}}], timeout: 2.0}"
```
