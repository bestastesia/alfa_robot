# AI 协作日志

这里记录每个 AI 最近做了什么，方便其他独立上下文的 AI 快速接上。

## 2026-05-12 项目经理 / Codex / T-0001
- 做了什么：创建 AI 协作入口，并根据用户反馈从“重流程管理”改成“轻量高效协作”。
- 改了哪里：`AGENTS.md`、`CLAUDE.md`、`.ai_teamwork/START.md`、`.ai_teamwork/NOW.md`、`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`、`.ai_teamwork/ROLES.md`。
- 验证结果：入口文件能把 Codex/ClaudeCode 引导到 `.ai_teamwork/START.md`。
- 留给下个 AI：先读 `START.md`、`NOW.md`、`TASKS.md`、`LOG.md`；具体任务等待用户/PM 分配。

## 2026-05-12 项目经理 / Codex / 安排 T-0002
- 做了什么：把“提交 AI 轻量协作机制更改”安排给 Git 操作工程师。
- 改了哪里：更新 `.ai_teamwork/TASKS.md`，追加本日志。
- 验证结果：任务范围已限定为 `AGENTS.md`、`CLAUDE.md`、`.ai_teamwork/`。
- 留给下个 AI：提交时不要带入 `.windsurfrules`、`lidar_ws/log/*` 等未归属改动。

## 2026-05-12 项目经理 / Codex / 补充工程师长期注意事项
- 做了什么：按用户反馈恢复每个工程师自己的长期注意事项 md，但保持轻量。
- 改了哪里：新增 `.ai_teamwork/engineers/`，更新 `START.md`、`ROLES.md`、`README.md`、`TASKS.md`。
- 验证结果：启动仍然只要求先读 `START.md`，具体岗位文件按任务需要再读。
- 留给下个 AI：承担某岗位任务时，先读 `.ai_teamwork/engineers/` 下对应文件。

## 2026-05-12 Git 操作工程师 / Codex / T-0002
- 做了什么：提交 AI 轻量协作机制相关文件，并按任务要求限定提交范围。
- 改了哪里：`AGENTS.md`、`CLAUDE.md`、`.ai_teamwork/`（含 `engineers/`）。
- 验证结果：提交前核对 `git status --short --untracked-files=all`，未纳入 `lidar_ws/log/*` 未归属文件。
- 留给下个 AI：后续开工继续先读 `AGENTS.md` 和 `.ai_teamwork/START.md`。

## 2026-05-12 项目经理 / Codex / 安排 T-0003
- 做了什么：根据用户指令安排“拆卸/移除当前 4 代机械臂配置”任务给机械工程师。
- 改了哪里：更新 `.ai_teamwork/TASKS.md`，追加本日志。
- 验证结果：任务已明确“不删除 STL/mesh 文件”，并提示 `alfa_robot_v2_arm_v4_new/` 已有备份线索。
- 留给下个 AI：先做只读调查，确认哪些 URDF/Xacro/MoveIt/仿真文件引用 4 代机械臂；真正改业务代码前最好先完成 T-0002 协作机制提交。

## 2026-05-12 项目经理 / Codex / 修正 T-0003 看板
- 做了什么：修正任务看板，确认 T-0003 已出现在当前任务列表。
- 改了哪里：`.ai_teamwork/TASKS.md`。
- 验证结果：T-0003 状态为 TODO，负责人为机械工程师。
- 留给下个 AI：机械工程师先只读调查 4 代机械臂引用，不要删除 STL/mesh。

## 2026-05-12 项目经理 / Codex / 安排 T-0004/T-0005
- 做了什么：把“原位置安装六轴机械臂”拆成运控先定义接口、机械后落模型两个任务。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0004 分给运控工程师，T-0005 分给机械工程师且阻塞等待 T-0004。
- 留给下个 AI：运控工程师需要先给出安装基准、joint/link 命名、控制关节数量、ros2_control/MoveIt 约束，再交给机械工程师实施。

## 2026-05-12 运控工程师 / Codex / T-0004 v5 proxy 开发路径
- 做了什么：根据用户给出的第五代 2+1+3 传统六轴机械臂方向，梳理 proxy 模型先行的运控/机械协作路径。
- 改了哪里：仅追加本协作日志。
- 验证结果：未改代码；本轮输出机械工程师任务拆分、参数交付清单和推荐技术栈。
- 留给下个 AI：后续若开始落地，应先建立 v5 参数事实源，再生成 URDF/Xacro proxy、MoveIt 配置和 MuJoCo primitive 模型。

## 2026-05-12 机械工程师 / Codex / T-0003 真拆 4 代机械臂
- 做了什么：从主 URDF 模型树真实移除左右机械臂 link/joint，MoveIt/RViz 现在只加载 `world -> base_link -> pitch -> turn -> updown`。
- 改了哪里：`alfa_robot_description` 主 xacro/ros2_control，`alfa_robot_moveit_config` SRDF/kinematics/controller/limits，`alfa_robot_bringup` 相关控制器和启动链。
- 验证结果：`xacro` + `check_urdf` 通过；MoveItConfigsBuilder 用系统 Python 加载通过，URDF/SRDF 无 `leftjoint/rightjoint/left_arm/right_arm` 残留。
- 留给下个 AI：旧双臂脚本、旧宏和仿真仍保留历史引用；simulation 按用户要求未改，后续上新六轴臂时再统一替换。

## 2026-05-12 运控工程师 / Codex / T-0004 文档落地
- 做了什么：将 v5 proxy 技术路线、运控接口约束、机械工程师交付清单写入协作文档。
- 改了哪里：新增 `.ai_teamwork/V5_PROXY_INTERFACE.md`，更新 `.ai_teamwork/TASKS.md`、`.ai_teamwork/NOW.md`。
- 验证结果：T-0004 已标记 DONE，T-0005 从 BLOCKED 改为 TODO，并指向 v5 proxy 接口文档。
- 留给下个 AI：机械工程师应先按 M-01/M-02 交付六轴运动链和左右臂安装基准，不要先做最终 STL。

## 2026-05-12 Git 操作工程师 / Codex / 补充 AI 协作提交规则
- 做了什么：把“AI 负责提交必须标明对应 AI 协作方”写入 Git 操作工程师长期注意事项。
- 改了哪里：`.ai_teamwork/engineers/git_ops.md`，并追加本交接日志。
- 验证结果：未提交；当前工作区仍有其他工程师的既有未提交改动，未触碰业务代码。
- 留给下个 AI：提交时按实际 AI 身份添加协作 trailer，例如 Codex 使用 `Co-Authored-By: Codex <codex@openai.com>`。

## 2026-05-12 机械工程师 / Codex / T-0005 v5 proxy 双六轴安装
- 做了什么：按用户补充要求安装双六轴 v5 proxy；左右臂挂在 `updown` 两侧，挂点 `x=0, y=±0.32, z=0.18`，即与 `updown` 位于同一 YZ 平面（约定 +Y 为前方）。
- 改了哪里：`alfa_robot.urdf.xacro` 新增 primitive 双六轴链；同步 MoveIt SRDF/kinematics/limits/controllers、ros2_control 和 bringup 控制配置。
- 验证结果：`xacro` + `check_urdf` 通过；MoveItConfigsBuilder 可加载；URDF 树显示 `updown` 下挂 `left_v5_link0/right_v5_link0` 两条 6 轴链。
- 留给下个 AI：当前 proxy 连杆长度和限位为保守占位；后续机械给出准确 T 型电机尺寸、连杆长度、法兰/tool0 偏置后再替换参数。

## 2026-05-12 机械工程师 / Codex / T-0005 安装平面与 T 型 proxy 修正
- 做了什么：按用户纠正将双臂安装从 ZX 平面改为 YZ 平面，挂点为 `x=0, y=±0.32, z=0.18`。
- 改了哪里：重写 v5 proxy arm 可视结构，增加明确安装支架、6 个 T 型电机罐体/夹块和双杆连杆，不再是点安装/单盒子效果。
- 验证结果：待本轮重新运行 `xacro`、`check_urdf` 和 MoveItConfigsBuilder。
- 留给下个 AI：proxy 仍不是最终 CAD；若要“真像最终产品”，需要机械输出 T 型电机外径/厚度、每个关节法兰相对位姿、安装板尺寸、连杆截面和外观 STL。

## 2026-05-12 机械工程师 / Codex / T-0005 T 型电机 STL 模板
- 做了什么：按用户建议将 v5 proxy 中所有 T 型电机 visual 改为复用 `alfa_robot_v2_arm_v4_new/visual/leftjoint4.STL` 外观模板。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro` 的 `v5_t_motor_link`，visual 使用 STL mesh，collision 继续使用圆柱 primitive。
- 验证结果：`xacro` + `check_urdf` 通过；当时生成 URDF 中共有 8 个 `leftjoint4.STL` T 电机 visual；后续已扩展为每侧 6 个、共 12 个；MoveItConfigsBuilder 可加载。
- 留给下个 AI：当前 mesh 姿态按关节轴做了 `z/x/y` 三种 rpy 粗对齐；最终仍需机械确认每个 T 电机法兰相对 joint frame 的精确 xyz/rpy。

## 2026-05-12 机械工程师 / Codex / T-0005 关节间距与圆柱连杆修正
- 做了什么：按用户反馈修正 v5 proxy 自由度分布，J3 肘关节移到上臂末端，J4 腕部入口移到前臂末端，避免 J2/J3 看起来同轴同点。
- 改了哪里：`alfa_robot.urdf.xacro` 中 v5 机械臂链；双杆连杆改为单圆柱连杆，6 个 T 型关节均使用 `leftjoint4.STL` visual 模板。
- 验证结果：`xacro` + `check_urdf` 通过；生成 URDF 每侧 6 个 T 电机 mesh，共 12 个；MoveItConfigsBuilder 可加载。
- 留给下个 AI：J2/J3 目前仍同为 X 轴但已空间分离；若最终机构 J3 轴向不同，需要按机械设计再改 axis/rpy。

## 2026-05-12 机械工程师 / Codex / T-0005 正交 T 电机链与 joint1 零位
- 做了什么：按用户反馈调整 v5 proxy：J1/J2 正交，J3 保留主动 T 并增加 fixed 被动 T 支撑，J4/J5/J6 轴向改为 Y/X/Z 三轴正交。
- 改了哪里：`alfa_robot.urdf.xacro` 中 v5 关节轴、T 电机姿态、J1 origin rpy；J1 通过 `v5_joint1_zero_rpy=0 0 -1.5708` 把原来 `joint1=-90°` 的姿态作为新零位。
- 验证结果：`xacro` + `check_urdf` 通过；MoveItConfigsBuilder 可加载；每侧 6 个主动 T + 1 个被动 T，共 14 个 `leftjoint4.STL` visual。
- 留给下个 AI：当前仍是 proxy；T 电机法兰和支架偏移参考旧 URDF 量级（0.098/0.13/0.1435），最终需机械 CAD 尺寸闭环。

## 2026-05-12 项目经理 / Codex / 安排 T-0006/T-0007/T-0008
- 做了什么：根据用户反馈安排 MoveIt 规划组、右手末端控制球、右手镜像三个任务。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0006 处理 `dual_arm_with_base` 关节组成；T-0007 处理右手末端交互球；T-0008 处理右手对称建模。
- 留给下个 AI：T-0006/T-0007 可先查 MoveIt SRDF/RViz；T-0008 涉及模型改动，注意和六轴替换任务 T-0004/T-0005 的先后关系。

## 2026-05-12 项目经理 / Codex / 修正完成任务归档流程
- 做了什么：把已完成任务从当前任务列表移到已完成任务区，并补充 PM 维护规则。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：当前任务列表只保留 T-0006/T-0007/T-0008；T-0002/T-0003/T-0004/T-0005 进入已完成任务。
- 留给下个 AI：以后 PM 标记 DONE 时，必须同步移动到“已完成任务”，不要只改状态。

## 2026-05-12 机械工程师 / Codex / T-0008 右手模型镜像
- 做了什么：按任务板将 v5 右臂相对左臂做 Y 方向镜像，左臂用户手调参数保持不变。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`，给 `v5_proxy_arm` 增加 `mirror_y`，右臂实例为 `-1`。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；joint2~joint6 的右臂 origin 已相对左臂在 Y=0 镜像。
- 留给下个 AI：当前仍按此前约定统一使用 `leftjoint4.STL` 做 T 电机外观；若需要真实左右外观镜像，可再切换右臂 mesh 或单独调整 mesh rpy。

## 2026-05-12 MoveIt/运控工程师 / Codex / T-0006
- 做了什么：修正 v5 双臂带底座规划组，`dual_v5_arm_with_base` 不再包含 `pitch/turn`，仅包含 `updown + 左右臂 12 关节`。
- 改了哪里：`alfa_robot.srdf`、`ros2_controllers.yaml`、`moveit_controllers.yaml`、`dual_arm_planner_node.cpp`、`dual_arm_pose_planner.py`、`.ai_teamwork/TASKS.md`。
- 验证结果：SRDF 展开为 13 个关节；MoveItConfigsBuilder 加载通过；`colcon build --packages-select alfa_robot_moveit_config` 通过；双臂脚本 py_compile 通过。
- 留给下个 AI：T-0007 可继续检查右手交互球，重点看 SRDF end_effector 的 `parent_link/parent_group`、RViz planning group 选择和右手 tool frame。

## 2026-05-12 MoveIt/机械工程师 / Codex / T-0007
- 做了什么：修复 v5 右手末端交互控制球配置；左右单臂 SRDF group 改为 chain 到 `*_v5_tool0`，end_effector 的 parent_group 绑定到 `*_v5_arm_with_base`。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf`、`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：MoveItConfigsBuilder 加载通过；`colcon build --packages-select alfa_robot_moveit_config` 通过；SRDF 中左右 end_effector 均存在。
- 留给下个 AI：T-0008 继续处理右手镜像建模；若 RViz 仍不显示右手球，下一步应实机启动 RViz 检查 MotionPlanning 面板选中 group 是否为 `right_v5_arm_with_base`。

## 2026-05-12 机械工程师 / Codex / T-0008 真镜像修正
- 做了什么：修正右臂不只是 Y 坐标取反，而是按 Y=0 反射做 joint origin、rpy、axis 的真实镜像。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`；右臂使用 `rightjoint4.STL`，左臂仍使用 `leftjoint4.STL`。
- 验证结果：`xacro` 和 `check_urdf` 通过；脚本校验 joint1~joint6 的位置、姿态矩阵和轴向均满足右臂 = Y 镜像(左臂)。
- 留给下个 AI：若用户继续手调左臂参数，右臂会通过 `mirror_y=-1` 跟随镜像；不要再只做坐标取反。

## 2026-05-12 MoveIt/机械工程师 / Codex / T-0007 dual 组补充
- 做了什么：根据用户指出的旧提交 `9ef92b8...` 可双末端规划状态，补齐 `dual_v5_arm_with_base` 的左右两个 end_effector 注册。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.srdf`、`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：SRDF 中 `parent_group=dual_v5_arm_with_base` 的 end_effector 数量为 2；MoveItConfigsBuilder 加载通过；`colcon build --packages-select alfa_robot_moveit_config` 通过。
- 留给下个 AI：如果 RViz 仍只有一个球，优先实机检查 MotionPlanning Display 是否选中 `dual_v5_arm_with_base` 并刷新 start/goal state；SRDF 已与旧双臂模式等价迁移。

## 2026-05-12 项目经理 / Codex / 安排 T-0009/T-0010
- 做了什么：根据用户确认，安排“RViz 双末端目标位姿实时显示”和“替换新的机械臂 URDF/模型文件”两个任务。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0009 分给 MoveIt/可视化工程师，强调显示 RViz 交互目标位姿；T-0010 分给机械工程师，强调新 URDF 替换和备份当前 v5 proxy/STL。
- 留给下个 AI：T-0009 不是显示当前 TF，而是显示拖拽交互球的目标；T-0010 开始前先确认新 URDF/mesh 的具体来源路径。

## 2026-05-12 项目经理 / Codex / 修正 T-0009 负责人
- 做了什么：用户指出没有 MoveIt/可视化工程师岗位，已把 T-0009 调整为运控工程师负责。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/ROLES.md`、`.ai_teamwork/engineers/motion_control.md`。
- 验证结果：T-0009 当前负责人为运控工程师；MoveIt/RViz 调试链路纳入运控职责，末端 frame 问题由机械协助。
- 留给下个 AI：后续不要再创建不存在的岗位；当前岗位只有 PM、运控、机械、Git、仿真、雷达 SLAM 导航、电控、感知抓取等既有角色。

## 2026-05-12 运控工程师 / Codex / T-0009
- 做了什么：新增 RViz 双末端目标位姿监听节点，实时监听 MoveIt/RViz interactive marker feedback，而不是当前 TF。
- 改了哪里：`rviz_dual_goal_pose_monitor.py`、`rviz_dual_goal_pose_monitor.launch.py`、`CMakeLists.txt`、`package.xml`、`.gitignore`、`.ai_teamwork/TASKS.md`。
- 验证结果：`python3 -m py_compile` 通过；`colcon build --packages-select alfa_robot_moveit_config` 通过；脚本已安装到 `lib/alfa_robot_moveit_config`。
- 留给下个 AI：运行时先启动 MoveIt/RViz，再 `ros2 launch alfa_robot_moveit_config rviz_dual_goal_pose_monitor.launch.py`；若无输出，用 `ros2 topic list | grep feedback` 找真实 feedback topic 后通过 launch 参数覆盖。

## 2026-05-12 机械工程师 / Codex / T-0010 新 v5 URDF 模型替换
- 做了什么：接入 `alfa_robot_arm_v5` 新 SolidWorks 导出模型，主 URDF 改为使用新 `motor1~motor6` mesh 和原始关节参数；保留现有 `left/right_v5_joint1..6`、`left/right_v5_link0..tool0` 接口。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`、新增 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_arm_v5/`、备份 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot_v5_proxy_backup.urdf.xacro`。
- 验证结果：`xacro` 展开通过，`check_urdf` 通过；左右臂新 mesh 各 6 个 visual/collision；右臂 joint origin/rpy/axis 通过 Y 镜像矩阵校验；`colcon build --packages-select alfa_robot_description --symlink-install` 完成。
- 留给下个 AI：新源 URDF 只有 5 个内部关节，当前用挂载处新增 joint1 + motor1~6 组成 6 轴接口；MoveIt/RViz 目标位姿任务 T-0009 由运控处理，本轮未改其脚本/launch。

## 2026-05-12 机械工程师 / Codex / 左臂 mesh 镜像到右臂
- 做了什么：以当前左臂为唯一基准，生成右臂专用 `visual_right` / `collision_right` STL，右臂不再复用同一套左臂 mesh。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`；新增 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_arm_v5/visual_right/` 和 `collision_right/`。
- 验证结果：`xacro`、`check_urdf`、`colcon build --packages-select alfa_robot_description --symlink-install` 均通过；逐三角面校验右侧 STL 为左侧 STL 的 local Y 镜像，且法线/绕序已修正。
- 留给下个 AI：如果继续微调左臂原始 mesh，需要重新生成右臂镜像 STL；仅改 URDF joint 镜像不足以保证外观完全镜像。

## 2026-05-12 运控工程师 / Codex / RViz plan_execute 残影排查
- 做了什么：排查 RViz 点击 Plan & Execute 后出现运动轨迹残影的问题；当前配置中 Planned Path 开启 `Loop Animation`，会持续回放 `/display_planned_path` 轨迹，表现类似残影。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/config/moveit.rviz`，将 MotionPlanning 的 `Planned Path -> Loop Animation` 关闭。
- 验证结果：`moveit.rviz` YAML 解析通过；`colcon build --packages-select alfa_robot_moveit_config` 通过。
- 判断结论：从配置看不是 MoveIt 同时向同一控制器下发多个关节目标；当前 `ros2_controllers.yaml` / `moveit_controllers.yaml` 只启用 `dual_v5_arm_controller`。若现场仍有残影，优先用 `ros2 topic info /joint_states -v` 和 `ros2 topic info /display_planned_path -v` 确认是否手动启动了多个 joint state 或 move_group/RViz 发布者。
- 留给下个 AI：不要把 Planned Path 的 RViz 轨迹回放误判成控制器重复发送；如现场确认 `/joint_states` 有多个 publisher，再检查是否同时启动了 demo、bringup、joint_state_publisher_gui 或多个 move_group。

## 2026-05-12 运控工程师 / Codex / BioIK 碰撞解排查
- 做了什么：排查“BioIK 能求出末端位姿但返回碰撞解，明明存在无碰撞解”的问题。
- 判断结论：这不是 BioIK 单纯求解能力不足，更像当前调用链没有把碰撞有效性约束接入 IK 采样；`dual_arm_planner_node.cpp` 直接 `setFromIK(..., timeout)`，没有传 `GroupStateValidityCallbackFn`，所以 IK 成功只代表末端位姿满足，不代表状态无碰撞。
- 关键差异：`test_moveit_pose_goal.py` 走 `/compute_ik` 且设置 `avoid_collisions=True`；C++ 双臂节点走本地 `RobotState::setFromIK`，默认不检查 PlanningScene 碰撞。
- 建议方向：优先不要怪 BioIK；下一步应给 C++ IK 增加 PlanningScene/碰撞 validity callback，或改为让 MoveIt 规划器处理 pose constraints，而不是先固定一个可能碰撞的 joint target。
- 留给下个 AI：如果要修代码，重点看 `ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp:162`，以及 MoveIt2 `RobotState::setFromIK` 的 `GroupStateValidityCallbackFn` 参数。

## 2026-05-12 运控工程师 / Codex / T-0011 BioIK 碰撞过滤
- 做了什么：自建 T-0011 并修复 C++ 双臂 BioIK 路径，给 `RobotState::setFromIK` 增加 PlanningScene 碰撞有效性回调。
- 改了哪里：`ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp`、`ros2_ws/src/alfa_robot_moveit_config/CMakeLists.txt`、`ros2_ws/src/alfa_robot_moveit_config/package.xml`、`.ai_teamwork/TASKS.md`。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config` 通过。
- 留给下个 AI：现场测试时若原目标仍失败，说明无碰撞解没在当前 timeout/seed 下被采到，可再做多 seed 重试或改成 pose constraints 规划；本轮已避免“明知碰撞还接受 IK 解”。

## 2026-05-12 机械工程师 / Codex / v5_1 新机械臂替换
- 做了什么：将 `/mnt/mydisk/ALFA/alfa_robot/alfa_robot_arm_v5_1` 的 SolidWorks 导出模型替换进 `alfa_robot_description`，并用 v5_1 的 URDF 惯量、关节 origin/rpy/axis 更新主 xacro。
- 改了哪里：更新 `ros2_ws/src/alfa_robot_description/meshes/alfa_robot_arm_v5/` 四套 mesh（visual/collision/visual_right/collision_right）和 `ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`；右臂 mesh 仍由左臂 local Y 镜像生成。
- 验证结果：`xacro`、`check_urdf`、左右镜像矩阵校验通过；`colcon build --packages-select alfa_robot_description --symlink-install` 通过；安装空间 24 个 v5_1 mesh 完整。
- 留给下个 AI：源导出目录 `alfa_robot_arm_v5` 和 `alfa_robot_arm_v5_1` 已按用户要求清理；包内 `alfa_robot_description/meshes/alfa_robot_arm_v5` 是当前事实源。

## 2026-05-12 项目经理 / Codex / 安排 T-0012
- 做了什么：根据用户怀疑“底座/机械臂安装连接件本身已碰撞”，新增机械侧任务 T-0012。
- 任务要求：机械工程师删除或禁用 v5 机械臂安装金属连接件的 collision，必要时也移除 visual，实现机械臂可视为直接虚空连接到底盘/升降结构。
- 约束条件：不得改运控接口，必须保留 `left/right_v5_joint1..6`、`left/right_v5_link0..tool0`、MoveIt SRDF/controller 使用的 joint/link 名称。
- 验证要求：`xacro`、`check_urdf` 通过，并在 MoveIt/RViz 或脚本中确认默认状态不再因安装连接件自碰撞。
- 留给下个 AI：若只删 collision 即可解决，优先保留 visual 方便观察；若 visual 误导调试，再按用户“虚空连接”方案移除 visual。

## 2026-05-12 机械工程师 / Codex / T-0012 安装连接件碰撞禁用
- 做了什么：禁用 v5 左右机械臂首节安装连接件 `left_v5_link1` / `right_v5_link1` 的 collision，保留 visual，机械臂仍通过原 joint/link 接口“虚空连接”。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`；`v5_imported_motor_link` 新增 `include_collision` 参数，实例化 `*_v5_link1` 时设为 `false`。
- 验证结果：`xacro`、`check_urdf` 通过；生成 URDF 中 `*_v5_link0` collision=0，`*_v5_link1` visual=1/collision=0；MoveIt C++ 模型加载显示 `left/right_v5_link1` geoms=0；`colcon build --packages-select alfa_robot_description --symlink-install` 通过。
- 留给下个 AI：若 BioIK 仍报告初始碰撞，下一步应检查 `*_v5_link2` 与 `updown/base` 或左右臂互碰，而不是安装连接件 `link1`。

## 2026-05-12 机械工程师 / Codex / 安装连接件视觉隐藏
- 做了什么：在 T-0012 基础上进一步隐藏左右 `*_v5_link1` 的 visual，使安装连接件在 RViz 中不可见，同时继续无 collision。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`；`v5_imported_motor_link` 新增 `include_visual` 参数，`*_v5_link1` 设置 `include_visual=false include_collision=false`。
- 验证结果：`xacro`、`check_urdf`、`colcon build --packages-select alfa_robot_description --symlink-install` 通过；生成 URDF 中 `left/right_v5_link1` visual=0、collision=0。
- 留给下个 AI：现在可见机械臂从 `*_v5_link2` 开始；接口 link/joint 名称仍完整保留。

## 2026-05-12 机械工程师 / Codex / 恢复第一个电机模型
- 做了什么：纠正上一轮误判，恢复左右 `*_v5_link1` 的 motor1 visual/collision；真正的前一级安装连接件是 `*_v5_link0`，它保持 visual=0/collision=0。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`，移除 `*_v5_link1` 实例上的 `include_visual=false include_collision=false`。
- 验证结果：`xacro`、`check_urdf`、`colcon build --packages-select alfa_robot_description --symlink-install` 通过；生成 URDF 中 `*_v5_link0` 不显示不碰撞，`*_v5_link1` 电机恢复 visual=1/collision=1。
- 留给下个 AI：不要再把 `*_v5_link1` 当连接件删；它是第一个电机，前一级虚空连接是 `*_v5_link0`。

## 2026-05-12 机械工程师 / Codex / 删除 motor1 内部连接子块
- 做了什么：确认 `*_v5_link0` 在 URDF 中已无 visual/collision，用户看到的连接件实际是 `motor1.STL` 内部的独立小连通块；已从 motor1 visual/collision STL 中仅剔除该 48-triangle 小连接块，保留第一个电机主体。
- 改了哪里：`ros2_ws/src/alfa_robot_description/meshes/alfa_robot_arm_v5/visual/motor1.STL`、`collision/motor1.STL`，并重新生成 `visual_right/motor1.STL`、`collision_right/motor1.STL`。
- 验证结果：`xacro`、`check_urdf`、`colcon build --packages-select alfa_robot_description --symlink-install` 通过；motor1 左右 visual/collision 均只剩 1 个连通块。
- 留给下个 AI：第一个电机仍是 `*_v5_link1`；被删除的是 motor1 mesh 内部小连接件，不是 URDF link。

## 2026-05-12 机械工程师 / Codex / 隐藏 updown STL 但保留升降
- 做了什么：根据用户确认，真正需要隐藏/禁碰的是 `updown` link 对应的旧金属 STL，而不是 v5 电机；已移除 `updown` link 的 visual/collision 几何，保留 link、prismatic joint 和双臂挂载。
- 改了哪里：`ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro`，删除 `package://alfa_robot_description/meshes/alfa_robot_v2_arm_v4_new/visual/updown.STL` 和 collision/updown.STL 的引用。
- 验证结果：`xacro`、`check_urdf`、`colcon build --packages-select alfa_robot_description --symlink-install` 通过；生成 URDF 中 `updown` visual=0/collision=0，`updown` 仍为 prismatic 且左右 `*_v5_mount` parent 仍是 `updown`。
- 留给下个 AI：机械臂安装位置不变，仍随 `updown` 升降；不要再通过删除 v5 电机或 motor1 子块解决 updown STL 问题。

## 2026-05-12 项目经理 / Codex / 安排 T-0011/T-0012
- 做了什么：把“机械臂可达范围复印到 3D 空间”拆成运控核心测试和仿真可视化两个任务。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0011 由运控工程师负责 MoveIt IK、手动记录、自动采样；T-0012 由仿真工程师负责同机器人模型坐标系下的 3D 可视化。
- 留给下个 AI：可视化优先考虑 RViz Marker/MarkerArray，因为能和 RobotModel 共用 fixed frame，满足“点云和机器人实体相对位置”需求。

## 2026-05-12 项目经理 / Codex / 修正可达范围任务编号
- 做了什么：发现 T-0011/T-0012 已被其他已完成任务占用，重新用 T-0013/T-0014 登记可达范围测试与可视化任务。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0013 为运控核心测试任务；T-0014 为仿真三维可视化任务；当前任务列表只包含这两个新任务。
- 留给下个 AI：不要复用已有任务编号；可达范围需求是“像复印一样把可达点映射到机器人同坐标系 3D 空间”。

## 2026-05-12 项目经理 / Codex / 补充 PM 任务依赖规则
- 做了什么：按用户要求，将“任务依赖/并行关系必须标注”写入项目经理长期注意事项。
- 改了哪里：`.ai_teamwork/engineers/project_manager.md`、`.ai_teamwork/LOG.md`。
- 验证结果：以后 PM 布置任务时需要写清“依赖 T-XXXX”或“可与 T-XXXX 并行”。
- 留给下个 AI：当前 T-0013/T-0014 不额外调整；从后续新任务开始执行该规则。

## 2026-05-12 运控工程师 / Codex / T-0013 可达范围测试核心
- 做了什么：实现机械臂可达范围测试核心节点，支持手动记录当前末端 TF 位姿和自动区域采样调用 `/compute_ik`。
- 改了哪里：新增 `ros2_ws/src/alfa_robot_moveit_config/scripts/reachability_tester.py`、`ros2_ws/src/alfa_robot_moveit_config/launch/reachability_tester.launch.py`，更新 `CMakeLists.txt` 安装脚本。
- 验证结果：`/usr/bin/python3 -m py_compile` 通过；`colcon build --packages-select alfa_robot_moveit_config` 通过；无 MoveIt 服务环境下 launch 可启动并按预期报 `/compute_ik service not available`。
- 使用示例：启动 MoveIt 后，自动采样 `ros2 launch alfa_robot_moveit_config reachability_tester.launch.py mode:=auto side:=left min_x:=0.0 max_x:=0.8 min_y:=0.0 max_y:=0.6 min_z:=0.1 max_z:=0.8 step:=0.1 output_csv:=/tmp/left_reach.csv`；手动记录 `mode:=manual side:=both output_csv:=/tmp/manual_trace.csv`。
- 留给下个 AI：T-0014 仿真工程师可直接消费 CSV 做 RViz Marker/MarkerArray；当前脚本没有做三维可视化。

## 2026-05-12 运控/仿真工程师 / Codex / 撤销可视化相关任务
- 做了什么：按用户要求撤销本轮所有可视化相关工作，不再保留 T-0014 的 RViz 点云可视化，也撤回对实时 RViz 目标可视化脚本的后续改动。
- 改了哪里：删除未跟踪的 `reachability_rviz_visualizer.py` / `reachability_rviz_visualizer.launch.py`；恢复 `rviz_dual_goal_pose_monitor.py` 和对应 launch 到原有目标位姿监控版本；从任务表移除 T-0014。
- 保留内容：T-0013 可达性测试核心 `reachability_tester.py` / `reachability_tester.launch.py` 仍保留，因为它不是可视化任务，而是 CSV 采样核心。
- 验证结果：`colcon build --packages-select alfa_robot_moveit_config --symlink-install` 通过。
- 留给下个 AI：如果以后重新做可视化，需要用户重新确认展示逻辑；当前不要继续实现可视化。

## 2026-05-12 项目经理 / Codex / 安排 Pinocchio 模块1任务
- 做了什么：记录 Pinocchio/Jinja2/MeshCat/NumPy/SciPy/Matplotlib/Plotly 技术栈，并下发“参数化模型生成器”模块1任务。
- 改了哪里：`.ai_teamwork/PINOCCHIO_STACK.md`、`.ai_teamwork/TASKS.md`、机械/仿真/运控工程师长期注意事项。
- 验证结果：T-0015 提取当前机械臂 baseline；T-0016 搭建参数化 URDF 生成 + Pinocchio 加载 + MeshCat 可视化；T-0017 依赖前两者做运动学语义校验。
- 留给下个 AI：T-0015 与 T-0016 可并行；T-0017 依赖 T-0015/T-0016；第一阶段验收是可修改参数、生成临时 URDF、Pinocchio 加载并 MeshCat 可视化。

## 2026-05-12 仿真工程师 / Codex / T-0016 Pinocchio 参数化模型生成器骨架
- 做了什么：先于机械工程师完成模块1软件骨架，新增独立 `simulation/pinocchio_parametric/`，可从 YAML baseline stub 通过 Jinja2 生成双 v5 机械臂 URDF；支持 XML/check_urdf 检查，安装 Pinocchio 后可加载模型，安装 MeshCat 后可浏览器可视化。
- 改了哪里：新增 `configs/v5_baseline_stub.yaml`、`templates/alfa_v5_parametric.urdf.j2`、`tools/generate_model.py`、`README.md`、`generated/.gitignore` 和自动生成的 `MECHANICAL_HANDOFF.md`。
- 验证结果：`/usr/bin/python3 -m py_compile` 通过；`python3 simulation/pinocchio_parametric/tools/generate_model.py --check` 可生成 URDF 和机械交接，XML sanity 通过；当前环境未安装 Pinocchio，按设计跳过 Pinocchio load；ROS `check_urdf` 成功解析双臂树。
- 留给机械工程师：请优先看 `simulation/pinocchio_parametric/MECHANICAL_HANDOFF.md`，校准 `v5_baseline_stub.yaml` 里的 mount、joint origin/rpy/axis、link mass/COM/inertia，并标明参数来自 URDF、SolidWorks、测量还是估算；工具接口名称已对齐 `left/right_v5_joint1..6` 和 `left/right_v5_tool0`。
- 留给运控工程师：T-0017 仍需等 T-0015 校准后再确认 Pinocchio frame/joint 语义、FK/Jacobian/IK/RNEA 可用性。

## 2026-05-12 Git 操作工程师 / Codex / 中文提交信息规则
- 做了什么：根据用户要求，将“后续提交信息统一用中文”写入 Git 操作工程师长期注意事项。
- 改了哪里：`.ai_teamwork/engineers/git_ops.md`，并追加本交接日志。
- 验证结果：仅更新协作规则文档，未改业务代码。
- 留给下个 AI：后续提交标题和正文都优先用中文，仍保留必要的 `Co-Authored-By` trailer。

## 2026-05-12 Git 操作工程师 / Codex / 中文提交标题细则
- 做了什么：补充提交信息规则，允许 `feat:`/`fix:` 等英文类型前缀，但冒号后的标题和正文必须用中文。
- 改了哪里：`.ai_teamwork/engineers/git_ops.md`，并追加本交接日志。
- 验证结果：仅更新协作规则文档，未改业务代码。
- 留给下个 AI：提交格式示例：`fix: 修复 v5 双臂碰撞过滤`，正文继续中文，保留 `Co-Authored-By` trailer。

## 2026-05-12 机械工程师 / Codex / T-0015 Pinocchio baseline 校准
- 做了什么：基于当前 ROS2 `alfa_robot.urdf.xacro` 和 v5_1 SolidWorks 惯量参数，校准 T-0016 参数化模型 baseline；补充大臂/小臂长度、wrist/flange offset、mount、joint 拓扑、质量/COM/惯量来源。
- 改了哪里：`simulation/pinocchio_parametric/configs/v5_baseline_stub.yaml`、新增 `simulation/pinocchio_parametric/MECHANICAL_BASELINE.md`，并重新生成 `simulation/pinocchio_parametric/MECHANICAL_HANDOFF.md`。
- 验证结果：`/usr/bin/python3 -m py_compile` 通过；`/usr/bin/python3 simulation/pinocchio_parametric/tools/generate_model.py --check` 通过；ROS `check_urdf simulation/pinocchio_parametric/generated/alfa_v5_parametric.urdf` 通过；当前环境未安装 Pinocchio，加载检查按工具设计跳过。
- 留给下个 AI：T-0017 运控可基于已校准 YAML 校验 Pinocchio joint/frame/FK/Jacobian/IK 语义；注意参数化模型根是 `world`，未包含 ROS2 主底座链 `base_link->pitch->turn->updown`。

## 2026-05-12 运控工程师 / Codex / T-0017 Pinocchio 运动学语义校验
- 做了什么：完成参数化模型运动学语义校验工具，覆盖 joint/frame 命名、父子链、mount、origin/rpy、axis、limit、FK、position Jacobian 和简单 position IK smoke test。
- 改了哪里：新增 `simulation/pinocchio_parametric/tools/validate_kinematics.py`；更新 `simulation/pinocchio_parametric/README.md`；生成报告 `simulation/pinocchio_parametric/generated/kinematic_semantics_report.md`。
- 验证结果：`/usr/bin/python3 simulation/pinocchio_parametric/tools/validate_kinematics.py` 通过；左右臂 neutral FK 对称，position Jacobian rank=3，轻量 position IK 收敛；当前环境未安装 Pinocchio，所以 Pinocchio 原生 FK/Jacobian/IK/RNEA 路径已写好但未运行。
- 留给机械工程师：当前运控确认 YAML baseline 与生成 URDF 的运动学语义一致；参数化模型根为 `world`，未含 `base_link->pitch->turn->updown`；后续若改臂长、offset、axis 或 tool0，先更新 `configs/v5_baseline_stub.yaml` 后重跑校验。当前模板仍只使用 `inertia_diag`，高可信动力学需把 `full_inertia` 接入模板。
- 留给下个 AI：若需要真正 Pinocchio 数值验收，请先安装 Pinocchio 到 `/usr/bin/python3` 对应环境，再运行 `validate_kinematics.py --require-pinocchio`。

## 2026-05-12 项目经理 / Codex / 安排 Pinocchio 真实外观补强
- 做了什么：确认 `reachability_tester.py` 是 T-0013 运控工程师完成；Pinocchio 模型外观粗糙的原因是 T-0016 只做了 primitive 骨架，模板里 `visual.use_primitives=true` 用圆柱/球占位，验收没有要求真实 mesh 外观。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：新增 T-0018/T-0019/T-0020，分别由机械、仿真、Git 处理真实 mesh 映射、MeshCat 真实外观加载、生成物提交边界。
- 留给下个 AI：这次目标要对齐 SolidWorks/Matlab 插件式真实机械臂外观，不能继续用圆柱体凑合；T-0018/T-0019 可并行，T-0020 依赖前两者。

## 2026-05-12 机械工程师 / Codex / T-0018 Pinocchio 真实外观 mesh 映射
- 做了什么：为参数化 Pinocchio 模型补齐真实 v5 机械臂外观 mesh 映射，明确 link1..6 到 motor1..6.STL 的左右臂 visual/collision 对应关系，保留 link0/tool0 为 frame-only，并说明右臂 mesh 已预镜像、不要在模板里二次镜像。
- 改了哪里：`simulation/pinocchio_parametric/configs/v5_baseline_stub.yaml` 新增 `mesh_mapping`；新增 `simulation/pinocchio_parametric/MESH_MAPPING.md`；更新 `.ai_teamwork/TASKS.md` 将 T-0018 移入已完成。
- 验证结果：`/usr/bin/python3 -m py_compile simulation/pinocchio_parametric/tools/generate_model.py` 通过；YAML mesh 映射加载和全部 STL 文件存在性检查通过；`/usr/bin/python3 simulation/pinocchio_parametric/tools/generate_model.py --check` 通过；ROS `check_urdf simulation/pinocchio_parametric/generated/alfa_v5_parametric.urdf` 通过。
- 留给仿真工程师：T-0019 可以直接读取 `mesh_mapping` 实现 `visual.use_primitives=false`；Pinocchio 离线加载建议优先生成 repo-relative 或 `file://` mesh 路径，右侧直接使用 `visual_right/collision_right` 目录，不要再加 mirror scale。

## 2026-05-12 仿真工程师 / Codex / T-0019 Pinocchio MeshCat 真实外观
- 做了什么：完成参数化 URDF 的真实 STL 外观接入，默认从 T-0018 `mesh_mapping` 生成左右臂 `motor1..6.STL` visual/collision，并保留 primitive fallback。
- 改了哪里：`configs/v5_baseline_stub.yaml`、`templates/alfa_v5_parametric.urdf.j2`、`tools/generate_model.py`、`README.md`、`MECHANICAL_BASELINE.md`、`.ai_teamwork/TASKS.md`。
- 验证结果：`py_compile` 通过；`generate_model.py --check --visual-mode mesh --mesh-uri-mode package/file` 通过；`--visual-mode primitives` fallback 通过；ROS `check_urdf` 通过；`validate_kinematics.py` 通过。当前环境仍未安装 Pinocchio/MeshCat，真实浏览器渲染需在安装后运行。
- 留给下个 AI：T-0020 已解除依赖，可清理 generated/、临时 URDF、MeshCat 缓存和提交边界；若安装 Pinocchio 后做最终验收，建议先跑 `python3 simulation/pinocchio_parametric/tools/generate_model.py --check --visualize --visual-mode mesh`。

## 2026-05-13 项目经理 / Codex / Pinocchio 参数化外观返工
- 做了什么：根据用户严厉反馈，确认 T-0018/T-0019 方向不满足验收：它们把当前真实机械臂 mesh 接进 Pinocchio，而不是生成“参数变化后仍像真实机械臂”的参数化外观。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/PINOCCHIO_STACK.md`、`.ai_teamwork/engineers/simulation.md`、`.ai_teamwork/LOG.md`。
- 验证结果：暂停 T-0020；新增 T-0021/T-0022/T-0023 返工链路，要求机械先定义真实参数化外观规范，仿真再实现，最后用多组比例 demo 验收。
- 留给下个 AI：不要再用当前整机 mesh 冒充参数化模型；如果做不到，必须明确说明做不到和原因，不能凭感觉乱做。

## 2026-05-13 项目经理 / Codex / 更新 UR 资产使用要求
- 做了什么：根据用户补充，纠正“不能用 mesh”的过度理解；允许工程师研究并合理利用 `universal_robot-noetic-devel` 中 UR STL/DAE/Xacro 资产。
- 改了哪里：`.ai_teamwork/PINOCCHIO_STACK.md`、`.ai_teamwork/TASKS.md`、机械/仿真工程师长期注意事项、`.ai_teamwork/LOG.md`。
- 验证结果：T-0021/T-0022/T-0023 已更新为“合理利用 UR 资产 + 保证大臂/小臂长度参数化变化”的验收口径。
- 留给下个 AI：关键不是禁止 mesh，而是不能只加载固定完整机械臂；最重要验收是 upper_arm_length、forearm_length 改变时生成 URDF 和 MeshCat 外观确实变化。

## 2026-05-13 机械工程师 / Codex / T-0021 参数化真实外观规范
- 做了什么：评估 `universal_robot-noetic-devel/ur_description` 中 UR 系列 STL/DAE/Xacro/YAML 资产，确定采用 UR5 分件关节 mesh + 参数化大臂/小臂 primitive 的 hybrid 方案，避免固定整臂 mesh 冒充参数化外观。
- 改了哪里：新增 `simulation/pinocchio_parametric/PARAMETRIC_APPEARANCE_SPEC.md`；在 `simulation/pinocchio_parametric/configs/v5_baseline_stub.yaml` 新增 `parametric_appearance` 钩子；更新 `.ai_teamwork/TASKS.md`。
- 验证结果：YAML 可加载；`parametric_appearance.fixed_assets` 指向的 UR5 `shoulder/wrist1/wrist2/wrist3` visual/collision 文件均存在；`generate_model.py --check` 和 ROS `check_urdf` 仍通过。
- 留给仿真工程师：T-0022 请实现 `ur5_parametric_hybrid`，核心验收是 link2/link3 的 visual/collision 长度随 `dimensions.upper_arm_y`、`dimensions.forearm_y` 变化；T-0018/T-0019 的固定 `mesh_mapping` 只能作为 baseline 外观，不满足比例扫描验收。

## 2026-05-13 机械工程师 / Codex / T-0021 UR5 资产内置修正
- 做了什么：根据用户提醒，把 T-0021 规范实际依赖的最小 UR5 资产从外部 `universal_robot-noetic-devel` 复制进项目，避免后续删除外部目录后生成器失效。
- 改了哪里：新增 `simulation/pinocchio_parametric/assets/ur5/`，包含 UR5 `shoulder/wrist1/wrist2/wrist3` 的 visual DAE、collision STL、来源 README 和 `ur_description` BSD 许可证；更新 `PARAMETRIC_APPEARANCE_SPEC.md`、`v5_baseline_stub.yaml`、`.ai_teamwork/TASKS.md`。
- 验证结果：`parametric_appearance.asset_root` 已切到 `simulation/pinocchio_parametric/assets/ur5`；全部固定资产路径存在；生成器和 `check_urdf` 待本轮最终复跑确认。
- 留给仿真工程师：T-0022 禁止引用 `universal_robot-noetic-devel`，只能引用 `simulation/pinocchio_parametric/assets/ur5` 或后续项目内新增资产。

## 2026-05-13 仿真工程师 / Codex / T-0022 参数化 Hybrid 外观返工
- 做了什么：按 T-0021 规范完成 `ur5_parametric_hybrid` 外观生成，关节外壳复用项目内置 `assets/ur5`，大臂/小臂 visual/collision 随 `dimensions.upper_arm_y`、`dimensions.forearm_y` 动态变化。
- 改了哪里：`configs/v5_baseline_stub.yaml`、`templates/alfa_v5_parametric.urdf.j2`、`tools/generate_model.py`、`README.md`、`MECHANICAL_BASELINE.md`。
- 验证结果：`py_compile` 通过；`generate_model.py --check --appearance-mode ur5_parametric_hybrid` 通过；默认生成 URDF 只引用 `simulation/pinocchio_parametric/assets/ur5` 和 primitive，不引用外部 `universal_robot-noetic-devel`。
- 留给下个 AI：若用户删除外部 UR 文件夹，T-0022 生成器不受影响；外部目录只在历史日志/规范中作为来源说明出现，不是运行时依赖。

## 2026-05-13 仿真工程师 / Codex / T-0023 参数化外观验收 demo
- 做了什么：新增 `tools/generate_appearance_demos.py`，生成 short/baseline/long 三组 `robot_ratio_*.urdf` 和 `appearance_demo_report.md`，用于复现不同大臂/小臂长度下的外观变化。
- 改了哪里：`simulation/pinocchio_parametric/tools/generate_appearance_demos.py`、`simulation/pinocchio_parametric/generated/appearance_demos/`、`.ai_teamwork/TASKS.md`。
- 验证结果：三组 demo 的 `joint3/joint4` origin 与 `link2/link3` visual/collision cylinder length 同步变化；三组 `check_urdf` 通过；`validate_kinematics.py` 通过。当前环境仍未安装 Pinocchio/MeshCat，所以浏览器截图需后续安装环境后执行。
- 留给 Git 操作工程师：T-0020 已解除阻塞；提交时保留 `assets/ur5` 最小资产和生成脚本，generated 下验收 URDF 是否提交请按用户需要决定，禁止提交整个 `universal_robot-noetic-devel`。

## 2026-05-13 Git 操作工程师 / Codex / 外部 UR 目录提交边界
- 做了什么：按用户要求确认 T-0022 运行时不依赖外部 `universal_robot-noetic-devel`，并在 `.gitignore` 忽略整个外部目录，避免误提交。
- 改了哪里：`.gitignore` 放行 `simulation/pinocchio_parametric/assets/**/*.dae|*.stl|*.STL`，同时忽略 `universal_robot-noetic-devel/`。
- 验证结果：`git check-ignore` 确认外部 UR 文件被忽略，项目内置 UR5 DAE/STL 未被忽略。
- 留给下个 AI：用户删除 `/mnt/mydisk/ALFA/alfa_robot/universal_robot-noetic-devel` 后，参数化外观生成器仍应正常工作；历史文档里保留外部目录仅作为来源说明。

## 2026-05-13 仿真工程师 / Codex / T 型关节外观修正
- 做了什么：根据用户反馈，撤掉 `ur5_parametric_hybrid` 中 link2/link3 的 sphere 端盖，改为 T-motor-like 组合：主连杆 cylinder + 两端横向电机 cylinder + 竖向 hub + bracket。
- 改了哪里：`templates/alfa_v5_parametric.urdf.j2`、`tools/generate_model.py`、`configs/v5_baseline_stub.yaml`、`PARAMETRIC_APPEARANCE_SPEC.md`、`README.md`，并重新生成三组 appearance demo。
- 验证结果：`generate_appearance_demos.py --check` 通过；三组 `check_urdf` 通过；`validate_kinematics.py` 通过；`rg "<sphere" templates/alfa_v5_parametric.urdf.j2 generated/appearance_demos/robot_ratio_*.urdf` 和默认 hybrid URDF 均无结果。
- 留给下个 AI：这次不再用球冒充 T 关节；如果视觉仍不满意，下一步应引入真实 T 电机 mesh 或 CAD 资产，而不是回退 sphere/capsule。

## 2026-05-13 项目经理 / Codex / 撤销 Pinocchio 仿真路线
- 做了什么：根据用户明确指令，撤销当前所有 Pinocchio 相关仿真任务和后续返工路线。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/PINOCCHIO_STACK.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0015/T-0016/T-0017/T-0018/T-0019 标注为已撤销历史；T-0020/T-0021/T-0022/T-0023 从当前任务移入已撤销任务区。
- 留给下个 AI：不要继续推进 Pinocchio 参数化仿真；相关文件暂不删除，除非用户明确要求做清理/回滚任务。

## 2026-05-13 项目经理 / Codex / 清理 Pinocchio 仿真文件
- 做了什么：按用户指令清理/回滚 Pinocchio 仿真路线相关文件。
- 改了哪里：删除 `.ai_teamwork/PINOCCHIO_STACK.md` 和 `simulation/pinocchio_parametric/`；清理机械/仿真/运控长期注意事项里的 Pinocchio 继续推进提示；更新 `.ai_teamwork/TASKS.md`。
- 验证结果：Pinocchio 工具目录已不存在；任务历史仍保留在 `TASKS.md` 和 `LOG.md`，方便追溯为什么撤销。
- 留给下个 AI：不要恢复或继续推进 Pinocchio 方案；如需提交，本次提交应只包含 Pinocchio 清理和协作记录更新，不要带入 `.gitignore`、`lidar_ws/log/*` 等未归属改动。

## 2026-05-13 项目经理 / Codex / Pinocchio 清理补充
- 做了什么：将 Pinocchio 探索期临时引入的外部 `universal_robot-noetic-devel/` 加入 `.gitignore`，避免误提交大体积 vendor 快照。
- 改了哪里：`.gitignore`、`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`。
- 验证结果：不删除用户放入的外部目录，但 Git 不再把它列为待提交文件；Pinocchio 自研工具目录仍已删除。
- 留给下个 AI：如果用户明确要求彻底删除外部 UR 快照，再单独执行 `rm -rf universal_robot-noetic-devel/`。

## 2026-05-13 项目经理 / Codex / 安排 T 电机标定任务
- 做了什么：根据用户要求，将工作范围收敛为单一标定任务，不再一次性布置完整机械臂搭建链路。
- 改了哪里：`.ai_teamwork/TASKS.md`、新增 `.ai_teamwork/MOTOR_MODEL_CALIBRATION.md`、`.ai_teamwork/LOG.md`。
- 验证结果：当前只新增 T-0024，目标是标定 `motor_model/T电机.stl` 的坐标系、旋转轴、连接端和 URDF mesh 放置方式。
- 留给下个 AI：本轮不要搭完整机械臂；先把 T 电机标定清楚并准备让用户确认。

## 2026-05-13 项目经理 / Codex / 强化 T 电机三维标定重点
- 做了什么：根据用户补充，将 T-0024 的核心从“泛泛 STL 标定”改为“在 3D 坐标系中明确连接点、旋转点和旋转方向”。
- 改了哪里：`.ai_teamwork/TASKS.md`、`.ai_teamwork/MOTOR_MODEL_CALIBRATION.md`、`.ai_teamwork/LOG.md`。
- 验证结果：T-0024 验收现在聚焦 joint origin、joint axis、parent/child 连接点、连接方向向量、mesh xyz/rpy 和给用户确认的可视化材料。
- 留给下个 AI：不要开始搭机械臂；先把单个 T 电机的 3D 坐标标定做成用户能确认的图/坐标。

## 2026-05-13 机械工程师 / Codex / T-0024 T 电机三维标定候选
- 做了什么：解析 `motor_model/T电机.stl` 的二进制 STL 顶点和包围盒，确认 STL 单位按 mm；以两个圆柱中心线交点 `[0,0,100] mm` 作为候选坐标原点，给出候选 A：旋转轴为竖直 `+Z`，parent 从 `-Z` 接入，child 从 `+Y` 接出。
- 改了哪里：填写 `.ai_teamwork/MOTOR_MODEL_CALIBRATION.md`；新增 `.ai_teamwork/motor_calibration/t_motor_calibration_points.json`、`t_motor_calibration_preview.svg`、`t_motor_calibration_preview.urdf`；更新 `.ai_teamwork/TASKS.md` 为等待用户确认。
- 验证结果：`check_urdf .ai_teamwork/motor_calibration/t_motor_calibration_preview.urdf` 通过；2D SVG 投影和 JSON 坐标已生成；未搭完整机械臂，未改主 URDF。
- 留给用户/下个 AI：请用户打开 SVG 或 RViz 预览确认黑点是否为旋转点、蓝色 +Z 是否为旋转轴、紫色 parent 与橙色 child 点/方向是否正确；确认后再把 T-0024 标 DONE 并用于机械臂组装。
