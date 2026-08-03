# 电控工程师长期注意事项

## 核心目标

维护电机、驱动器、CAN/CANopen/ZeroErr/Cylinder、限位、急停和电气安全假设。

## 主要关注路径

- `/mnt/mydisk/ALFA/SevenovaHangzhou/robot_system/rt_control/`
- `docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- `docs/REFACTOR_ARCHITECTURE_NOTES.md`

## 长期注意

- 电机 ID、CANopen node id、方向、减速比、零点、限位都属于高风险信息。
- 协议层改动必须说明参数来源和验证方式。
- 上电、下电、急停、失联、超时处理要优先保证安全。
- 和运控工程师共同确认实机动作逻辑，不要孤立修改硬件驱动。
