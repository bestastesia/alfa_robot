# L6/R8 实机方向安全基准

日期：2026-06-30

## 结论

`/home/ar/lhy_dev/run_l6_r8_real.sh` 是当前 L6/R8 实机流程唯一允许入口，必须保持：

- 方向映射开启，不能加入 `--no-real-apply-direction-signs`。
- 负重姿态锁定 `--loaded-preferred-pose-index 0`。
- 慢速执行：`--hz 10`，`--max-joint-speed-deg-s 10`。

当前已验证正确的 direct-real 方向规则：

| 关节 | 软件方向处理 |
| --- | --- |
| left_joint3 | 翻转 |
| left_joint5 | 翻转 |
| right_joint2 | 翻转 |
| 其它 10 个关节 | 不翻转 |

## 唯一真相源

仓库内方向映射唯一真相源是：

```text
ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge/joints.py
```

其中 `ROS_TO_ETHERCAT_SIGN_BY_JOINT` 定义 `ROS/Rerun 语义角度 -> 实机 EtherCAT 指令角度`。

规则：

- 不要在执行脚本、launch、YAML 或临时测试脚本里复制方向表。
- `execute_l6_r8_mock_live.py` 必须从 `alfa_robot_execution_bridge.joints` 导入 joint 顺序和方向转换函数。
- `alfa_robot_execution_bridge/config/*.yaml` 只允许配置是否应用方向映射，不允许复制 `direction_signs`。
- 如果实机方向重新标定，只改 `joints.py`，然后运行 `scripts/safety/check_l6_r8_real_safety.py`。

## 关键风险

这次方向反复出现，不只可能来自 EtherCAT sign，还可能来自 L6/R8 的上层负重姿态族索引。
旧的 `loaded_preferred_pose_index=1` 对应 `[-75, 135, 60]` 肘型；当前实机确认方向使用 `loaded_preferred_pose_index=0`。

## 禁止事项

不要在实机任务流程中临时追加或恢复：

```bash
--no-real-apply-direction-signs
--loaded-preferred-pose-index 1
--hz <更高频率>
--max-joint-speed-deg-s <更高速度>
```

如需诊断方向，使用独立小角度单轴测试脚本，不要改 L6/R8 实机任务脚本。

## 当前安全脚本

```bash
/home/ar/lhy_dev/run_l6_r8_real.sh
/home/ar/lhy_dev/verify_l6_r8_direction_safety.sh
```

`verify_l6_r8_direction_safety.sh` 不会运动机器人，只检查：

- wrapper 没有关闭方向映射。
- wrapper 锁定 `loaded_preferred_pose_index=0`。
- wrapper 锁定 `hz=10`、`max_speed=10deg/s`、`right_first`。
- Python 层禁止 real direct 关闭方向映射。
- planner 与执行脚本都接收同一个负重姿态索引。

## 复测建议

如果未来同事改了 controller/hardware 方向处理、负重姿态族或执行脚本，先运行：

```bash
/home/ar/lhy_dev/verify_l6_r8_direction_safety.sh
```

再用小角度单轴测试确认方向；不要直接跑 L6/R8 全流程验证方向。
