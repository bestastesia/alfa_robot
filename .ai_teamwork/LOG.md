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
