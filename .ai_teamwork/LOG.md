# AI 协作日志

这里记录当前方向下的最新协作信息。长日志已归档：

- `.ai_teamwork/archive/2026-05-13_direction_reset/LOG.full_history.before_reset.md`
- `.ai_teamwork/archive/2026-05-16_pm_handoff/LOG.before_cleanup.md`

## 2026-05-16 项目经理 / Codex / PM 重新接手与任务清理
- 做了什么：重新梳理当前项目状态，将已完成任务和长日志归档，当前任务表只保留待推进事项。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/NOW.md`、`.ai_teamwork/LOG.md`，新增 `.ai_teamwork/archive/2026-05-16_pm_handoff/`。
- 验证结果：当前待推进任务为 T-0030/T-0031/T-0032/T-0036；已完成 T-0025/T-0026/T-0027/T-0029/T-0033/T-0034/T-0035 只保留摘要；力学语义和 v5 proxy 历史文档已备份进本次归档。
- 留给下个 AI：默认不要读归档；需要追溯旧细节再查 archive。`alfa_robot_v2_arm_v5/` 根目录包疑似可删，但 description 包内 `meshes/alfa_robot_v2_arm_v5/` 仍被当前 URDF 使用，不能删。

## 2026-05-16 仿真工程师 / Codex / 最小 Rerun 机器人可视化案例
- 做了什么：按用户要求先做最基础 Rerun 案例，新建 `alfa_robot_rerun` ROS2 Python 包；读取当前 `alfa_robot_description` xacro，加载 visual mesh，订阅 `/joint_states`，用轻量 URDF FK 更新 Rerun 中各 link 位姿。
- 改了哪里：新增 `ros2_ws/src/alfa_robot_rerun/`，包含 `basic_robot_viewer` 节点、`basic_robot_viewer.launch.py`、`README.md`、`package.xml`、`setup.py/setup.cfg`。
- 验证结果：确认 `/usr/bin/python3` 已有 `rerun-sdk 0.31.4`；`py_compile` 通过；`colcon build --packages-select alfa_robot_rerun --symlink-install` 通过；`ros2 pkg executables` 可见 `basic_robot_viewer`；`spawn:=false` 短时启动成功并订阅 `/joint_states`。
- 留给下个 AI：第一版刻意不接 MoveIt/RViz 交互、不做规划和记录管理；后续若扩展，优先加 planned path/末端 frame/可达点云 logger，保持只读观察层。

## 2026-05-16 仿真工程师 / Codex / Rerun SDK 0.31 时间 API 修复
- 做了什么：修复 `basic_robot_viewer` 在收到 `/joint_states` 后崩溃的问题；Rerun 0.31 没有 `set_time_seconds`，改为 `rr.set_time("ros_time", timestamp=stamp)`。
- 改了哪里：`ros2_ws/src/alfa_robot_rerun/alfa_robot_rerun/basic_robot_viewer.py`。
- 验证结果：`py_compile` 通过；`colcon build --packages-select alfa_robot_rerun --symlink-install` 通过；用 `spawn:=false` 启动并 `ros2 topic pub --once /joint_states` 模拟一条关节状态，节点未再崩溃。
- 留给下个 AI：继续保持最小案例定位；若增加功能，优先加末端坐标系/轨迹显示，不要急着接管 MoveIt 交互。

## 2026-05-16 运控工程师 / Codex / V5 单圈关节限位调整
- 做了什么：按用户提供的 description 可视化角度映射，将左右 v5 六轴全部调整为单圈 `revolute` 限位；未使用 `continuous`。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`、`ros2_ws/src/alfa_robot_moveit_config/config/joint_limits.yaml`、`ros2_ws/src/alfa_robot_moveit_config/config/initial_positions.yaml`，新增 `docs/V5_JOINT_LIMIT_POLICY.md`。
- 换算规则：按旋转方向不变处理，即 `actual_angle = program_angle + offset`；因此 `left_v5_joint3/5` 写为 `180~540°` 初始 `360°`，`right_v5_joint4` 写为 `0~360°` 初始 `180°`，其余按用户映射得到对应单圈区间。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给机械/电控工程师：请重点确认非零程序偏移关节的驱动零点与实际零点关系：`left_v5_joint2/3/5`、`right_v5_joint1/2/4`；当前 MoveIt 和 ros2_control 初始角已按程序侧偏移写入。

## 2026-05-16 运控工程师 / Codex / description 预览初始角同步
- 做了什么：排查 `ros2 launch alfa_robot_description view_alfa_robot.launch.py` 看不到限位调整后初始姿态变化的问题。
- 判断原因：该 launch 实时加载的 xacro 限位已经是新的，但只启动 `joint_state_publisher_gui`，不会读取 MoveIt 的 `initial_positions.yaml`；GUI 对不跨 0 的关节会默认放在区间中点，对跨 0 的关节会默认 0，因此用户期望的 remapped 初始角不会全部生效。
- 改了哪里：`ros2_ws/src/alfa_robot_description/launch/view_alfa_robot.launch.py`，新增 `use_v5_initial_positions` 参数，默认把 v5 remap 后的初始角通过 `zeros.*` 传给 `joint_state_publisher_gui`。
- 验证结果：`/usr/bin/python3 -m py_compile` 通过；`colcon build --packages-select alfa_robot_description` 通过；短启动 launch 成功。
- 使用说明：重新 source `ros2_ws/install/setup.bash` 后启动即可；如需看纯 URDF 默认行为，可加 `use_v5_initial_positions:=false`。

## 2026-05-16 运控工程师 / Codex / 修正 right_v5_joint1 单圈限位
- 做了什么：根据用户在 description 预览中复核结果，修正 `right_v5_joint1` 初始化位姿和滑条范围。
- 改了哪里：`right_v5_joint1` 从此前按偏移换算的 `[-90°, 270°] / init 90°` 改为程序侧直观 `[-180°, 180°] / init 0°`；同步 URDF、MoveIt joint_limits、initial_positions、description preview `zeros.*` 和 `docs/V5_JOINT_LIMIT_POLICY.md`。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给机械/电控工程师：`right_v5_joint1` 现在以程序/GUI 0 位作为初始直观姿态，不再使用 +90° 程序偏移；不要按上一版 `-90:-180` 映射自动改回。

## 2026-05-16 运控工程师 / Codex / 修正 right_v5_joint4 单圈限位
- 做了什么：根据用户指出 rightjoint4 与 rightjoint1 同类问题，将 `right_v5_joint4` 初始化位姿和滑条范围改为 description/GUI 直观语义。
- 改了哪里：`right_v5_joint4` 从此前按偏移换算的 `[0°, 360°] / init 180°` 改为程序侧直观 `[-180°, 180°] / init 0°`；同步 URDF、MoveIt joint_limits、initial_positions、description preview `zeros.*` 和 `docs/V5_JOINT_LIMIT_POLICY.md`。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；`colcon build --packages-select alfa_robot_description alfa_robot_moveit_config` 通过。
- 留给机械/电控工程师：`right_v5_joint4` 现在以程序/GUI 0 位作为初始直观姿态，不再使用 +180° 程序偏移；不要按上一版 `0:-180` 映射自动改回。
