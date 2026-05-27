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

## 2026-05-25 电控工程师 / Codex / PLC Driver 代码架构图修订
- 做了什么：新增更准确的 `docs/plc_modbus/plc_modbus_code_architecture.svg`，严格按当前代码调用链绘制：CLI → PlcDriver → PlcAxisClient → Codec/Transport → Modbus 映射 → PLC AxisCtrl。
- 改了哪里：新增 `docs/plc_modbus/plc_modbus_code_architecture.svg`；旧 `plc_modbus_architecture.svg` 可作概览，新图用于解释代码路径。
- 验证结果：已检查 SVG 基本结构和关键调用链文本。
- 留给下个 AI：后续讲代码优先引用新版 `plc_modbus_code_architecture.svg`，避免误解 CLI 直接访问 Codec。

## 2026-05-25 电控顾问 / Codex / 顾问上下文交接整理
- 做了什么：整理本顾问窗口围绕用户电气学习、自搭机械臂、6/12 轴创意构型、电控安全概念的完整交接上下文，便于转交给另一位同事继续对接。
- 改了哪里：仅追加本协作日志；未改代码与工程文件。
- 验证结果：不涉及构建/测试。
- 留给下个 AI：用户希望后续同事延续“电控工程师顾问”身份，重点是结合 ALFA 自研机械臂，通过实物小项目学习电控/电气/伺服/安全，而非系统啃教材。

## 2026-05-25 电控顾问 / Codex / PLC Modbus 工作交接文档
- 做了什么：按用户要求整理可转交给同事的完整上下文，覆盖 PLC/Modbus 非 ROS 测试驱动、Linear MOTION-25/26 背景、用户电控顾问偏好和后续联调注意事项。
- 改了哪里：新增 `.ai_teamwork/handoffs/2026-05-25_plc_modbus_handoff.md`，临时副本位于 `/tmp/handoff-c8Q6Vg.md`。
- 验证结果：已按 handoff 技能要求先创建并读取临时文件，再写入交接内容；不涉及构建/测试。
- 留给下个 AI：接手 PLC 线先读该 handoff，再读 `CONTEXT.md`、`tools/alfa_robot_plc_driver/README.md`、Linear `MOTION-25/MOTION-26`。

## 2026-05-25 机械/仿真顾问 / Codex / 工作交接上下文整理
- 做了什么：按用户要求整理本 Codex 窗口相关工作上下文，覆盖 v5 机械臂历史、撤销的参数化/T 电机方向、实时力学 T-0025/T-0026/T-0027、当前可达性任务和 PLC 旁支背景。
- 改了哪里：新增 `.ai_teamwork/CODEX_WORK_HANDOFF_2026-05-25.md`，并追加本日志。
- 验证结果：不涉及构建；已核对当前 `TASKS/LOG`、归档摘要、实时力学 MVP README 和归档力学语义文档。
- 留给接手同事：优先读交接文档第 3 节实时力学分析和第 4 节当前可达性方向；不要恢复旧 Pinocchio 参数化生成或 T 电机自动拼装路线。

## 2026-05-28 机械/仿真顾问 / Claude / 四分支 v5_6 机械臂迁移

- 做了什么：按用户纠偏撤回先前在当前分支直接套 v5_6 的做法；确认 `motion-5`/`motion-10` 是次新状态、`motion-22`/`motion-26` 是次次新状态；以 `motion-5` 为机器人本体基线，给四个分支安装最新 `alfa_robot_arm_v5_6` 双臂模型。
- 改了哪里：`motion-5`、`motion-10`、`motion-22` 使用独立 worktree；当前工作区为 `motion-26`。四分支均更新 description 的双臂 URDF、ros2_control 机械臂 limits，并新增 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_arm_v5_6/` 右臂原始 mesh 与左臂 Y 镜像 mesh。`motion-22`/`motion-26` 额外同步 `motion-5` 的次新机器人本体文件。
- 验证结果：四个分支均通过 `xacro`、`check_urdf`、`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config --symlink-install`；`motion-5/10/22` worktree 构建时使用主工作区 `pick_ik` underlay。
- 留给下个 AI：不要删除根目录 `alfa_robot_arm_v5_6/` 原始源目录；提交交给 Git 操作工程师处理，任务如下：
  1. 分别审查并提交四个工作区：`.claude/worktrees/motion-5-migrate`、`.claude/worktrees/motion-10-migrate`、`.claude/worktrees/motion-22-migrate`、当前 `motion-26` 工作区。
  2. 每个分支只提交本次迁移相关文件：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`、`ros2_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro`、新增 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_arm_v5_6/`；`motion-22`/`motion-26` 还应包含同步次新机器人本体所需的 `ros2_ws/src/alfa_robot_moveit_config/CMakeLists.txt`、`config/alfa_robot.srdf`、`scripts/x_edge_refine_reachability.py`。
  3. 不要混入根目录 `alfa_robot_arm_v5_6/` 原始源目录、`data/ik_benchmark/`、`ros2_ws/*.csv|*.jsonl|*.rrd`、`CONTEXT.md`、`plc_modbus_test.py` 或无关 `.ai_teamwork` 改动。
  4. 推荐提交标题分别使用 `Refs MOTION-5/10/22/26: 安装最新 v5_6 机械臂模型`；提交前可复用本条日志中的验证命令和结果。
