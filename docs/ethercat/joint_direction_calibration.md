# EtherCAT 双臂关节方向标定记录

记录时间：2026-06-25

## 结论

当前实机测试确认：为了让机器人实际姿态与 Rerun/上位机目标姿态一致，控制器发送到 EtherCAT 侧的目标角度需要按下表做符号映射。

当前上层统一接口关节顺序为：前 6 个左臂、后 6 个右臂，最后 `turn`。

| 上位机 / Rerun 关节 | 发送到 EtherCAT 控制器的角度 | 方向关系 |
| --- | --- | --- |
| `right_joint1` | `+right_joint1` | 同向 |
| `right_joint2` | `-right_joint2` | 反向 |
| `right_joint3` | `+right_joint3` | 同向 |
| `right_joint4` | `+right_joint4` | 同向 |
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
+1, -1, +1, +1, +1, +1,
+1
```

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

## 后续控制器实现建议

控制器内部应统一接受上位机 / MoveIt / Rerun 语义下的关节目标角度，然后在 EtherCAT 写入前做一次方向映射。

建议封装为固定表，不要散落在测试脚本或任务编排代码里：

```text
ethercat_target_rad[joint] = ros_target_rad[joint] * joint_direction_sign[joint]
```

其中 `joint_direction_sign` 使用本文结论表。

## 注意事项

- 本结论只描述关节正方向符号关系，不描述零位偏置。
- 当前测试脚本发送时会先读取 `/joint_states` 作为轨迹第一点，避免首点与真实位置差距过大导致控制器拒绝。
- 后续正式控制器仍应单独处理：零位标定、软限位、速度/加速度限制、急停与跟随误差保护。
