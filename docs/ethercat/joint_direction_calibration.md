# EtherCAT 双臂关节方向标定记录

记录时间：2026-06-25（`right_joint4` 已于 2026-07-17 依据 `run_move_all_joints_abs.sh` 实机验证数据修正）

## 唯一权威来源

方向符号表和 updown logical/physical 换算，现在**同在一个物理文件**里管理：

```text
ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge/joints.py
```

任何脚本、launch 文件、YAML 配置都不允许再手抄这份表或换算公式，必须 `import` 这个模块。本文档只记录结论和标定过程，代码事实以 `joints.py` 为准。

2026-07 曾出现过一次严重实机事故（`run_move_all_joints_abs.sh` 启动剧烈抖动、同一命令不同天停止位置不同），根本原因排查后确认：并非某个 EtherCAT PDO 符号错误，而是这份方向表被手抄了三份（本仓库权威 `joints.py`、工控机部署副本、`move_all_joints_abs.py` 自己内嵌的字典），三份互相失步，且转换是否应用是一个默认可关闭的 flag（`--apply-ethercat-signs`），而不是强制的、唯一位置的转换。这次修复的原则是：

1. 表和换算只允许存在于 `joints.py` 一处；
2. 转换只允许在两个边界点发生，且不可关闭：发送前（`ros_to_ethercat_position()` / `logical_to_physical_updown()`），接收后（`ethercat_to_ros_position()` / `physical_to_logical_updown()`）；
3. `run_move_all_joints_abs.sh` / `move_all_joints_abs.py` 已废弃，替换为 `run_jog_to_pose.sh` / `jog_to_pose.py`（见下文）。

## 结论

当前实机测试确认：为了让机器人实际姿态与 Rerun/上位机目标姿态一致，控制器发送到 EtherCAT 侧的目标角度需要按下表做符号映射。

当前上层统一接口关节顺序为：前 6 个左臂、后 6 个右臂，最后 `turn`。

| 上位机 / Rerun 关节 | 发送到 EtherCAT 控制器的角度 | 方向关系 |
| --- | --- | --- |
| `right_joint1` | `+right_joint1` | 同向 |
| `right_joint2` | `-right_joint2` | 反向 |
| `right_joint3` | `+right_joint3` | 同向 |
| `right_joint4` | `-right_joint4` | 反向 |
| `right_joint5` | `+right_joint5` | 同向 |
| `right_joint6` | `+right_joint6` | 同向 |
| `left_joint1` | `+left_joint1` | 同向 |
| `left_joint2` | `+left_joint2` | 同向 |
| `left_joint3` | `-left_joint3` | 反向 |
| `left_joint4` | `+left_joint4` | 同向 |
| `left_joint5` | `-left_joint5` | 反向 |
| `left_joint6` | `+left_joint6` | 同向 |
| `turn` | `+turn` | 同向 |

换成符号数组，按当前统一执行接口关节顺序：

```text
left_joint1,  left_joint2,  left_joint3,  left_joint4,  left_joint5,  left_joint6,
right_joint1, right_joint2, right_joint3, right_joint4, right_joint5, right_joint6,
turn
```

对应：

```text
+1, +1, -1, +1, -1, +1,
+1, -1, +1, -1, +1, +1,
+1
```

## 2026-07-17 修正记录：`right_joint4`

用 `/home/ar/lhy_dev/run_move_all_joints_abs.sh` 下发实机绝对角度，验证上位机 ROS 语义姿态
`0, -45, 120, -75, 0, 0`（左右臂一致）时：

```bash
run_move_all_joints_abs.sh \
  --right-joint1-deg 0 --right-joint2-deg 45 --right-joint3-deg 120 --right-joint4-deg 75 \
  --right-joint5-deg 0 --right-joint6-deg 0 \
  --left-joint1-deg 0 --left-joint2-deg -45 --left-joint3-deg -120 --left-joint4-deg -75 \
  --left-joint5-deg 0 --left-joint6-deg 0 \
  --turn-deg 90 --updown-m 0.20 --duration-s 8 --send
```

与上位机设置的姿态完全一致。反推：

- `right_joint4`: ROS `-75` → EtherCAT `+75`，方向系数 `-1`（原记录 `+1` 是错的，已修正）。
- `left_joint4`: ROS `-75` → EtherCAT `-75`，方向系数 `+1`（跟原记录一致，未改动）。

其余关节（1/2/3/5/6，左右两侧）在这次验证中跟原表一致，未改动。

## 验证方式

用于 Rerun 预览的目标姿态：

```bash
/usr/bin/python3 scripts/ethercat_trajectory_test/joint_target_pose.py \
  --right-joint1-deg 10 --right-joint2-deg 10 --right-joint3-deg 10 \
  --right-joint4-deg 10 --right-joint5-deg 30 --right-joint6-deg 20 \
  --left-joint1-deg 10 --left-joint2-deg 10 --left-joint3-deg 10 \
  --left-joint4-deg 10 --left-joint5-deg 30 --left-joint6-deg 20 \
  --turn-deg 10 \
  --rerun-save data/ethercat_trajectory_tests/my_target_pose.rrd
```

实机发送时，为了达到同一姿态，使用了如下目标：

```bash
/usr/bin/python3 /tmp/joint_target_pose.py \
  --right-joint1-deg 10 --right-joint2-deg -10 --right-joint3-deg 10 \
  --right-joint4-deg 10 --right-joint5-deg 30 --right-joint6-deg 20 \
  --left-joint1-deg 10 --left-joint2-deg 10 --left-joint3-deg -10 \
  --left-joint4-deg 10 --left-joint5-deg -30 --left-joint6-deg 20 \
  --turn-deg 10 \
  --duration-s 5 \
  --send
```

实测结果：实机姿态与 Rerun 中显示的目标姿态一致。

## updown（升降轴）logical/physical 换算

`updown` 同样存在上位机语义（logical，MoveIt/URDF/joint_limits.yaml 范围 `0.0–0.7` m）与实机物理命令（physical，发到 `/canopen/updown_position_controller/commands` 的原始米数）两套语义，换算关系：

```text
physical = logical + UPDOWN_PHYSICAL_ZERO_OFFSET_M   # offset = 0.08 m
```

即 logical `0.0` m 对应 physical `0.08` m，logical `0.7` m 对应 physical `0.78` m。这组常量和 `logical_to_physical_updown()` / `physical_to_logical_updown()` 函数同样定义在 `joints.py`，与关节方向表在同一个文件里维护。

`0.08` m 这个 offset 来自本仓库既有基线文档（`current_motion_baseline.yaml`），这次只是把它从"写在文档里但从未被任何代码执行"变成"真正会被执行的代码"。offset 数值本身尚未经过这次实机重新测量确认，实机部署后仍需电控工程师用低速单轴测试核实 logical 0 对应的物理高度是否符合预期，才能认为这条换算在当前这台机器上是准确的。

实机侧历史范围曾是 physical `0.0–0.92` m（`ros2_control` 硬件层会静默 clamp 越界命令，不会报错拒绝），与上位机 `0.0–0.7` m 的 logical 范围不是同一回事，不要混用。

## 现在如何在实机上手动点位/示教

`run_move_all_joints_abs.sh` / `move_all_joints_abs.py` 已废弃（脚本顶部已加注释标注），原因：方向表手抄且过时、转换是可关闭 flag、updown 完全没有 logical/physical 换算。替换为：

```bash
/home/ar/lhy_dev/ros2_ws/src/alfa_robot_execution_bridge/scripts/run_jog_to_pose.sh \
  --right-joint1-deg 0 --right-joint2-deg 45 --right-joint3-deg 120 \
  --right-joint4-deg 75 --right-joint5-deg 0 --right-joint6-deg 0 \
  --left-joint1-deg 0 --left-joint2-deg -45 --left-joint3-deg -120 \
  --left-joint4-deg -75 --left-joint5-deg 0 --left-joint6-deg 0 \
  --turn-deg 90 --updown-m 0.20 \
  --duration-s 8 --send \
  [--rerun / --no-rerun]
```

所有角度/`--updown-m` 都是上位机 / MoveIt / Rerun 语义（`--updown-m` 是 logical 米数，范围 `0.0–0.7`，不是发给控制器的物理米数）；方向和 updown 换算在脚本内部强制发生，没有可关闭的选项。默认 dry-run，只有加 `--send` 才真正发送，发送前会要求二次确认。`--rerun` 可选打开实时 Rerun 查看当前姿态和目标姿态（ghost）。

## 后续控制器实现建议

控制器内部应统一接受上位机 / MoveIt / Rerun 语义下的关节目标角度，然后在 EtherCAT 写入前做一次方向映射。

不要封装成散落在测试脚本或任务编排代码里的固定表——直接 `import alfa_robot_execution_bridge.joints`：

```text
ethercat_target_rad[joint] = ros_to_ethercat_position(joint, ros_target_rad[joint])
ethercat_target_updown_m = logical_to_physical_updown(ros_target_updown_m)
```

## 注意事项

- 本结论只描述关节正方向符号关系和 updown 换算关系，不描述零位标定精度本身。
- 当前测试脚本发送时会先读取 `/joint_states` 作为轨迹第一点，避免首点与真实位置差距过大导致控制器拒绝。
- 后续正式控制器仍应单独处理：零位标定、软限位、速度/加速度限制、急停与跟随误差保护。
