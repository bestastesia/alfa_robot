# ALFA V2/V3/V4 URDF Joint Origin/Axis Check

这些结果来自对应提交的 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.xacro`，用于核对 `scripts/dh_workspace/configs/alfa_v*_urdf_left_arm_with_base.yaml`。

说明：

- `origin rpy` 是关节连接的固定角度；对 prismatic 关节也永久生效。
- prismatic 运动不是改变角度，而是在 `origin rpy` 之后，沿 joint frame 的 `axis` 平移。
- `axis_world_at_zero` 是所有前级关节取 0 时，该关节轴在世界/基坐标下的大致方向。
- v3 与 v4 的左臂链在这三个提交中一致。

## V4 `303f191a0dde617bcd5601cf918eb9b07861abcb`

| Joint | Type | origin xyz | origin rpy | axis local | limit | axis world @ zero |
| --- | --- | --- | --- | --- | --- | --- |
| updown | prismatic | `[-0.23547, 0, 0.57771]` | `[0, 0, 0]` | `[0, 0, 1]` | `[0, 1]` | `[0, 0, 1]` |
| leftjoint1 | prismatic | `[-0.00487, 0.232101, 0.232]` | `[0, 0, 0]` | `[1, 0, 0]` | `[0, 0.42]` | `[1, 0, 0]` |
| leftjoint2 | revolute | `[0.67335, 0.10135, 0.04]` | `[0, 1.5708, 0]` | `[0, 0, 1]` | `[-pi, pi]` | `[1, 0, 0]` |
| leftjoint3 | revolute | `[0.01, 0.0058, 0.098]` | `[-1.5708, -1.5708, 0]` | `[0, 0, 1]` | `[-pi/2, pi/2]` | `[0, 1, 0]` |
| leftjoint4 | revolute | `[0.3418, -0.01, -0.082]` | `[1.5708, 0, 1.5708]` | `[0, 0, 1]` | `[-pi, pi]` | `[1, 0, 0]` |
| leftjoint5 | revolute | `[0.01, -0.0022, 0.098]` | `[-1.5708, -1.5708, 0]` | `[0, 0, 1]` | `[-pi/2, pi/2]` | `[0, 1, 0]` |
| leftjoint6 | prismatic | `[0.13, 0, 0.1435]` | `[1.5708, 0, 0]` | `[1, 0, 0]` | `[0, 0.15]` | `[1, 0, 0]` |
| left_ee_joint | fixed | `[0.05, 0, 0]` | `[0, 0, 0]` | none | none | n/a |

## V3 `9ef92b8ed42f4643db870eaa85c673ce7fd063ac`

V3 左臂链与 V4 一致：`updown, leftjoint1, leftjoint2, leftjoint3, leftjoint4, leftjoint5, leftjoint6, left_ee_joint` 的 `origin xyz/rpy`、`axis`、`limit` 都相同。

## V2 `f84dcbb8acb28a616d954d8b7643b66817b2ce2c`

| Joint | Type | origin xyz | origin rpy | axis local | limit | axis world @ zero |
| --- | --- | --- | --- | --- | --- | --- |
| updown | prismatic | `[-0.23547, 0, 0.45934]` | `[0, 0, 0]` | `[0, 0, 1]` | `[0, 0.95]` | `[0, 0, 1]` |
| leftarmbase | prismatic | `[-0.16145, 0.4695, 0.146]` | `[0, 0, 0]` | `[0, 1, 0]` | `[-0.16, 0]` | `[0, 1, 0]` |
| leftjoint1 | prismatic | `[0.094579, -0.021899, 0.088]` | `[0, 0, 0]` | `[1, 0, 0]` | `[0, 0.45]` | `[1, 0, 0]` |
| leftjoint2 | revolute | `[0.74035, 0.0209, 0.033]` | `[3.14159, -1.5708, 0]` | `[0, 0, 1]` | `[-3.14, 3.14]` | `[1, 0, 0]` |
| leftjoint3 | revolute | `[0, -0.035, 0.11]` | `[0, 3.14159, 0]` | `[0, 1, 0]` | `[-0.3, 3.14]` | `[0, -1, 0]` |
| leftjoint4 | prismatic | `[0.376, 0.026, 0.0023]` | `[1.5708, 0, 0]` | `[1, 0, 0]` | `[0, 0.15]` | `[0, 0, -1]` |
| leftjoint5 | revolute | `[0.092, 0.0018, 0]` | `[-1.5708, 0, 0]` | `[0, 0, 1]` | `[-1.57, 1.57]` | `[-1, 0, 0]` |
| left_ee_joint | fixed | `[0.05, 0, 0]` | `[0, 0, 0]` | none | none | n/a |

## Important Prismatic Joint Fixed Angles

- V4/V3 `updown`: fixed rpy `[0, 0, 0]`, translates along local/world `+Z`.
- V4/V3 `leftjoint1`: fixed rpy `[0, 0, 0]`, translates along local/world `+X` at zero.
- V4/V3 `leftjoint6`: fixed rpy `[1.5708, 0, 0]`, translates along local `+X`; at zero it is also approximately world `+X` after upstream zero transforms.
- V2 `updown`: fixed rpy `[0, 0, 0]`, translates along local/world `+Z`.
- V2 `leftarmbase`: fixed rpy `[0, 0, 0]`, translates along local/world `+Y` at zero.
- V2 `leftjoint1`: fixed rpy `[0, 0, 0]`, translates along local/world `+X` at zero.
- V2 `leftjoint4`: fixed rpy `[1.5708, 0, 0]`, translates along local `+X`; at zero it becomes approximately world `-Z` due to upstream fixed transforms.
