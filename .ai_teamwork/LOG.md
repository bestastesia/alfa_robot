# AI 协作日志

这里仅保留当前分支仍需让新 AI 立刻看到的最新交接。长过程和已完成事项已归档。

## 归档索引

- `.ai_teamwork/archive/2026-05-13_direction_reset/LOG.full_history.before_reset.md`
- `.ai_teamwork/archive/2026-05-16_pm_handoff/LOG.before_cleanup.md`
- `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/LOG.before_archive.md`
- `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/COMPLETED_SUMMARY.md`

默认不要读归档；只有追溯历史原因、验收证据、责任边界或恢复旧方案时再查。

## 2026-05-18 项目经理 / Codex / v5_dev 协作文件归档

- 做了什么：切换到 `v5_dev`，将 2026-05-16～2026-05-18 的长日志、已完成摘要和归档前状态备份到 `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/`。
- 改了哪里：精简 `.ai_teamwork/LOG.md`、`.ai_teamwork/TASKS.md`、`.ai_teamwork/NOW.md`；新增本次 archive 目录和 `COMPLETED_SUMMARY.md`。
- 当前保留任务：只保留 T-0030/T-0031/T-0032/T-0036/T-0037 这类仍可能需要处理的任务；已完成的 T-0025/T-0026/T-0027/T-0029/T-0033/T-0034/T-0035/TIM-37 移入归档摘要。
- 留给下个 AI：开工仍先读 `.ai_teamwork/START.md`、`.ai_teamwork/NOW.md`、`.ai_teamwork/TASKS.md`；不要把 archive 里的旧任务当成当前待办。

## 2026-05-24 电控工程师 / Codex / PLC Modbus 测试包设计
- 做了什么：明确 `alfa_robot_plc_driver` 第一版为非 ROS Python CLI 测试包，放 `tools/alfa_robot_plc_driver/`；按左右臂合计 12 个同构 PLC Axis 设计。
- 改了哪里：新增/更新 `CONTEXT.md`，记录双臂 12 轴、PLC Axis、Multi-Axis Move、Stop Policy、Modbus Map Placeholder 等术语。
- 关键约定：地址未知时配置用 `null` 占位；mock/dry-run 可运行，真实 PLC 模式必须拒绝缺失地址；`move-many` 任一轴失败/超时默认普通停止所有参与轴。
- 留给下个 AI：代码写好后必须更新 Linear `MOTION-25`，说明当前进度，并明确后续拿到 InoProShop 最终 Modbus 映射表后应填入配置文件的位置。

## 2026-05-24 电控工程师 / Codex / alfa_robot_plc_driver 原型
- 做了什么：新增非 ROS Python CLI 测试包 `tools/alfa_robot_plc_driver/`，按左右臂合计 12 个同构 PLC Axis 实现 mock/dry-run/真实 PLC 配置校验骨架。
- 改了哪里：新增 `codec/config/transport/axis/driver/mock/cli` 模块、`config/plc_modbus_map.example.yaml`、README 和 pytest；未改 `ros2_ws/src/alfa_robot_hardware`。
- 验证结果：`python3 -m pytest -q tools/alfa_robot_plc_driver/tests` 通过（6 passed）；mock `status` 和 `move-many --targets '1:10,2:-20,7:15'` 可运行。
- 留给下个 AI：真实 PLC 模式必须先把 InoProShop 最终 Modbus 映射填入 `tools/alfa_robot_plc_driver/config/plc_modbus_map.example.yaml` 或复制出的 real YAML，并确认 LREAL/UDINT word order。

## 2026-05-24 电控工程师 / Codex / PLC Modbus 架构图
- 做了什么：新增 `docs/plc_modbus/plc_modbus_architecture.svg`，用图梳理上位机测试包、Modbus 协议边界、PLC AxisCtrl、EtherCAT/伺服执行层之间的关系。
- 改了哪里：新增 `docs/plc_modbus/plc_modbus_architecture.svg`；未生成 PNG，因为当前环境没有 `rsvg-convert` 或 ImageMagick 转换工具。
- 验证结果：已检查 SVG 基本结构和关键内容（`move-many`、`InoProShop`）。
- 留给下个 AI：如果需要 PNG，可在有 `rsvg-convert` 的环境运行 `rsvg-convert -o docs/plc_modbus/plc_modbus_architecture.png docs/plc_modbus/plc_modbus_architecture.svg`。
