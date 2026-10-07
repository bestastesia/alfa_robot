# 当前项目状态

## 一句话概况

ALFA Robot 是 ROS2 双臂工业机器人项目；当前仓库只保留运控、电控、机器人模型及其必要运行适配，历史感知、导航和仿真实现已退出主线。

## 开工入口

- 先读 `.ai_teamwork/START.md`。
- 再读本文件和 `.ai_teamwork/TASKS.md`。
- `.ai_teamwork/archive/` 默认不读；只有追溯旧方案、验收证据、责任边界时再查。

## 当前推进重点

- **RRT主线范围（2026-10-07用户再次确认）：** 默认与25箱验证均为导师原有Informed RRT，未切换PRM。PRM只是上游可选对照项；本次额外修复已撤出候选工作树，仅保存于`motion233-acceptance/prm-refinement/optional-prm-fix.patch`。不要把可选六规划器实验变成本次RRT rebase的必需门禁，也不得未经充分对照和用户确认更换主规划器。

- **2026-10-07：L02专项阻塞已修复，0.75m整墙25/25、15/15轮实跑通过。** 预算扩大无效；根因是抓取点过度固定在面中心，改为有界合法面内选点（最后一箱Z+53.75mm），模型/限位/TCP/箱体位置/放置目的位姿均不变。证据 `motion233-acceptance/l02-diagnosis/`，最新文档 `docs/motion233_l02_contact_diagnosis.md`；8095只读整墙回放。GPU窗口已结束并交回协调线程，后续计算须共享flock。下面24/25节点为历史记录，不代表最新状态；原PRM/全网格/多种子/整体迁移收口尚未因此完成。

- MOTION-233顶吸已接入：auto侧吸失败后检查顶升通道再顶吸，模式贯穿附着球/稳定性/缓存/放置目标/Viser。L03/R01顶吸完整周期通过；整墙auto依旧24/25，L02两种接触IK均失败，不冒称全过。最新证据 `curobo-top-v1` 和 `docs/motion233_curobo_top_suction.md`；8096顶吸、8095整墙均只读回放。GPU队列已结束并通知协调线程，后续必须持有 `/tmp/sevenova-curobo-gpu.lock` 或先协调。

- MOTION-233完整顺序：已取消最高3行截断，接入15轮（10双/5单）、逐轮场景继承、显式空载回程反向复用。0.75m从头实跑**24/25，14/15轮通过**；最后L02接触IK失败（左右及无碰撞诊断均未达2mm阈值），未改距离/模型/容差。8095展示该完整顺序和真实通过前缀，排4/5可选。最新入口/验收见 `docs/motion233_curobo_wall_sequence.md`；旧layout-v1文档是历史节点，不能对拍当前HEAD。

- MOTION-233布局增量：固定f044模型，接入行列/缺箱/平移/净距，默认及稀疏所选双箱通过；8095现为0.75m实际重算页。复验和边界见 `docs/motion233_curobo_wall_layout.md`。严格六规划器实际5/6（PRM搬运碰箱），总门禁未通过；原始与候选对同条失败轨迹判定一致，重跑各2次通过不能覆盖该失败。单臂/全25箱/原固定种子与FCL验收仍未完成。

- 本地 `motion-233-curobo-f044bf1` 按用户纠正基于 **alfa_v3_curobo@f044bf1**，不能回到此前误选的 MoveIt Stage 分支。255文件/模型对拍、L24/R20新计算和Viser验证见 `docs/motion233_curobo_rebase.md`；研究碰撞策略不等同于FCL/OBB全墙验收。

- `alfa_v3_curobo` 当前方向：MoveIt替代的基础阶段1～3（MOTION-276），研究入口在
  `research/curobo_v3`。独立快照/状态与规划核心见该目录`CORE.md`；GPU搜索算法仍跟踪
  MOTION-275。以下旧V3/旧主线信息属于历史上下文，不作为此分支当前模型和入口。

- 当前主线：`v5_dev` 已收口左右箱体正面中心 6D 位姿任务合同；功能分支正在接入 V3 七轴双臂模型。
- V3 模型工作跟踪：Linear `MOTION-94`。当前试验分支已接入 `robot_v3.0.9` 十六自由度整机模型；双臂外观与碰撞网格和 V3.0.8 上游资产逐字节一致。
- V3 基础动作跟踪：Linear `MOTION-154`。分支 `motion-154-v3-dual-arm-simple-motion` 已完成首个40cm箱双臂同步解析笛卡尔平移Demo，待用户交互验收后继续旋转和异构握持任务。
- 正式任务输入只包含 `request_id`、左右正面中心 `pose_6d` 和 `execute`；算法内部按高度容差识别排数、吸附方式和抽离策略，禁止从箱号或外部吸附模式获取帮助。
- 旧 `/robot_motion/run_box_pair_task` 仅保留为显式兼容入口，默认完整栈不启动 `box_pair_task_adapter_node`。
- 当前任务表只保留未完成/需确认事项：T-0030/T-0031/T-0032/T-0037。
- 已完成/已同步 Linear 的长过程已归档到 `.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/`。
- PM 必须持续把完成任务移出当前表，避免后续 AI 误认为仍需处理。
- 架构目标、包职责和调试代码准入规则见 `docs/运控/系统架构与包职责边界.md`。
- 可视化架构入口见 `docs/system_portal/architecture.html`。

## 当前仍需注意

- 当前 V3.0.9 恢复 `updown` 升降和 `head_joint` 旋转自由度，连同左右七轴共16个可动关节；`updown` 逻辑范围为 `-1.0～0.0m`（最低点 `-1.0m`、升高1m后的最高点 `0.0m`），`head_joint=-1.57～1.57rad`。因 V3.0.9 与 V3.0.8 的46个上游 STL 完全相同，继续复用 `ros2_ws/src/alfa_robot_description/meshes/robot_v3_0_8/`，不重复存储。
- V3 默认初始姿态固定为左右臂相同的 `J1～J7=[-90,-90,0,-90,0,0,0]°`，共享 `updown=0m`、`head_joint=0°`；模型查看器、mock ros2_control、MoveIt 初始状态和 SRDF `home` 必须保持一致。
- Linear/Git 关联提交标题优先使用 `Refs MOTION-xx: ...`；只写 `MOTION-xx:` 不稳定。
- 一个 issue 只对创建时的验收目标负责；后续探索/测试应拆新 issue 或放 Backlog，不要让已达标 issue 永远开着。
- `alfa_robot_moveit_config` 已不再编译或包含 `scripts/ik_benchmark/` 的头文件；公共 IK 候选类型已迁入 `robot_motion_core`，Rerun 公共实现已迁入 `alfa_robot_rerun`。
- `dual_arm_planner_node` 仍承载完整候选排序、抽离和负重规划适配；这些实现尚未全部迁入独立 core/planning service。
- 历史 `bio_ik`、仓库内 `alfa_robot_hardware` 和旧 `alfa_robot_bringup` 已退出主线；实机硬件与生命周期由外部 `rt-control` 域负责。
- 当前 V3.0.9 已完成 description、16轴 ros2_control/MoveIt 契约与冗余解析 IK 回归；V3.0.8 的单点、前伸40cm、周围15cm数据仅作为双臂几何参考，恢复整机自由度后的完整抓取流程尚未验收。

## 当前主要模块速查

- 接口契约：`ros2_ws/src/robot_motion_interfaces/`
- 纯算法公共核心：`ros2_ws/src/robot_motion_core/`
- 核心运行时：`ros2_ws/src/robot_motion_runtime/`
- 场景能力：`ros2_ws/src/robot_motion_scene_service/`
- 运动算法与 MoveIt 适配：`ros2_ws/src/alfa_robot_analytic_ik/`、`ros2_ws/src/alfa_robot_moveit_config/`
- 执行适配：`ros2_ws/src/alfa_robot_execution_bridge/`；真实硬件由外部 `rt-control` 域负责
- 模型与启动编排：`ros2_ws/src/alfa_robot_description/`、`ros2_ws/src/robot_motion_runtime/launch/`
- 可视化：`ros2_ws/src/alfa_robot_rerun/`
- 实验资产：`scripts/ik_benchmark/`，仅用于验证和迁移，不应反向成为运行时职责来源。

## 常用追溯入口

- 本次归档摘要：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/COMPLETED_SUMMARY.md`
- 本次归档前完整日志：`.ai_teamwork/archive/2026-05-18_v5_dev_collaboration_cleanup/LOG.before_archive.md`
- 控制层硬编码：`docs/CONTROL_LAYER_HARDCODED_PARAMS.md`
- 控制架构梳理：`docs/REFACTOR_ARCHITECTURE_NOTES.md`
- IK 服务说明：`docs/运控/IK/ik_service.md`
