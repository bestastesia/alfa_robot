window.SYSTEM_PORTAL_DATA = {
  meta: {
    title: "ALFA Robot 运控系统地图",
    subtitle: "从外界交互到 ROS2 包职责的可点击流程导航",
    updated: "2026-07-11",
    branchHint: "当前整理基于 v5_dev 清扫后的运控/电控源码；后续迁移到 robot_motion_control 时沿用相同职责规则。",
    updateRule: "新增包、接口或流程时，优先更新 assets/data.js；页面会自动渲染卡片、流程和状态。"
  },
  externalActors: [
    {
      id: "operator",
      name: "操作员 / 调试工程师",
      role: "启动 launch、触发 benchmark、查看 Rerun/RViz、下发临时任务。",
      interfaces: ["ros2 launch", "ros2 service call", "Rerun .rrd", "RViz"]
    },
    {
      id: "upstream",
      name: "任务上游",
      role: "通过稳定任务契约提供左右末端目标和抓取模式；上游可以是感知、整机任务系统或人工调试端。",
      interfaces: ["RunDualGraspTask", "6D Pose", "front / top_suction"]
    },
    {
      id: "planner",
      name: "运控规划服务",
      role: "把箱子目标转成 IK 候选、抽离轨迹、负重规划和可执行 JointTrajectory。",
      interfaces: ["motion_internal_interfaces", "MoveIt PlanningScene", "JSON/Rerun 记录"]
    },
    {
      id: "execution",
      name: "执行层 / 电控",
      role: "接收关节轨迹，转发到 mock、ros2_control 或真实硬件控制链。",
      interfaces: ["/alfa_execution/execute_joint_trajectory", "/joint_states", "ros2_control"]
    },
    {
      id: "robot",
      name: "机器人本体 / 电机",
      role: "执行关节命令并返回反馈；硬件侧包含总线协议、方向、限位、安全停机等真实约束。",
      interfaces: ["CAN/EtherCAT/电机协议", "关节反馈", "安全状态"]
    },
    {
      id: "world",
      name: "环境 / 箱墙 / 集装箱",
      role: "为规划提供碰撞边界：集装箱、箱墙、当前抓取箱、携带箱和动态开洞区域。",
      interfaces: ["场景几何", "MoveIt CollisionObject", "AttachedCollisionObject"]
    },
    {
      id: "visual",
      name: "可视化与审计",
      role: "展示系统状态、规划轨迹、失败原因和实验结论；目前以 Rerun/RViz/CSV/JSON 为主。",
      interfaces: ["Rerun", "RViz", "CSV", "JSONL", "stage_snapshot.json"]
    }
  ],
  systemFlows: [
    {
      id: "task-to-motion",
      title: "任务到运动主链路",
      summary: "从箱子目标输入开始，经过场景建模、IK、抽离、负重规划，最终交给执行层。",
      stages: [
        { name: "任务输入", owner: "任务上游 / operator", data: "request_id、左右 6D Pose、吸附模式 front/top_suction", output: "稳定任务契约 MotionContext" },
        { name: "场景生成", owner: "robot_motion_scene_service", data: "箱垛几何、集装箱尺寸、抓取箱号", output: "CollisionObject / AttachedBox / 箱墙开洞障碍" },
        { name: "IK 候选", owner: "alfa_robot_moveit_config + alfa_robot_analytic_ik", data: "左右 TCP pose、h 候选、负重姿态先验", output: "去重后的 IK candidate 列表和 cost/rank" },
        { name: "抽离搜索", owner: "extract_planning_pipeline", data: "IK candidate + 携带箱碰撞", output: "抽离成功轨迹 / 失败原因" },
        { name: "负重规划", owner: "loaded_pose_planning", data: "抽离末态、负重姿态族、PlanningScene", output: "shortcut / local RRT / RRT 轨迹" },
        { name: "执行和反馈", owner: "alfa_robot_execution_bridge / bringup / hardware", data: "JointTrajectory", output: "/joint_states、执行结果、安全状态" },
        { name: "记录和回放", owner: "alfa_robot_rerun / scripts", data: "stage_snapshot、trajectory、scene", output: "Rerun、CSV、JSONL、summary" }
      ]
    },
    {
      id: "collision-scene",
      title: "碰撞与场景真相链路",
      summary: "把箱墙、集装箱和附着箱统一转成规划可用的碰撞状态，避免 Rerun 与 MoveIt 口径漂移。",
      stages: [
        { name: "几何事实", owner: "robot_motion_scene_core", data: "箱子尺寸、箱垛列/排、车体相对位置", output: "AABB、面板、开洞墙、携带箱规格" },
        { name: "MoveIt 适配", owner: "MotionSceneAdapter", data: "几何对象列表", output: "PlanningSceneInterface 更新 / 临时 PlanningScene 快照" },
        { name: "轨迹校验", owner: "motion_collision_service_node / dual_arm_planner", data: "RobotState + trajectory + attached boxes", output: "valid、reason、contacts" },
        { name: "审计显示", owner: "Rerun/RViz", data: "同源 scene + robot state", output: "可视化碰撞环境" }
      ]
    },
    {
      id: "ik-extract-loaded",
      title: "IK → 抽离 → 负重算法链路",
      summary: "当前最核心的运控实验链路，目标是把可随机/可枚举的 IK 结果转成可执行的抽箱路径。",
      stages: [
        { name: "h 高度候选", owner: "OptimizedDualIkSolver", data: "侧吸/顶吸高度窗、fixed_updown、box pose", output: "h candidate list" },
        { name: "解析 IK 枚举", owner: "alfa_robot_analytic_ik", data: "单臂 pose + h", output: "左右臂 joint 解" },
        { name: "筛选去重", owner: "IkCandidateSelector", data: "legal candidates + score", output: "按代价排序且去重的 TopN" },
        { name: "抽离 rollout", owner: "ExtractCandidateSolver/Scorer", data: "抽离步长、lift/pitch/y shift 候选", output: "rollout path 或失败原因" },
        { name: "负重目标选择", owner: "LoadedPoseSelector", data: "抽离末态、负重姿态族", output: "最近负重姿态目标" },
        { name: "负重轨迹", owner: "LoadedPosePlanner", data: "shortcut / local RRT / RRTConnect", output: "最终采用轨迹" }
      ]
    },
    {
      id: "digital-twin",
      title: "事实源 / 可视化链路",
      summary: "硬件与仿真只提供观测，RobotMotionState/RobotMotionScene 是规划与任务状态机唯一读取的事实。",
      stages: [
        { name: "机器人模型", owner: "alfa_robot_description", data: "URDF/xacro、mesh、joint limits", output: "robot_description" },
        { name: "观测输入", owner: "hardware / mock executor", data: "/joint_states 或一次性 /robot_motion/set_state", output: "带时间戳的关节观测" },
        { name: "权威状态", owner: "robot_motion_runtime", data: "观测 + source + state_id", output: "唯一 /robot_motion/state" },
        { name: "权威场景", owner: "robot_motion_runtime + robot_motion_scene_service", data: "任务场景更新 + scene_id", output: "唯一 /robot_motion/scene" },
        { name: "Rerun 回放", owner: "alfa_robot_rerun + scripts", data: "RobotState/trajectory/scene JSON", output: ".rrd 动态回放" },
        { name: "RViz/MoveIt", owner: "alfa_robot_moveit_config", data: "PlanningScene + robot_description", output: "规划适配与真实场景显示" }
      ]
    }
  ],
  packages: [
    {
      id: "motion_internal_interfaces",
      name: "motion_internal_interfaces",
      layer: "域内接口契约",
      status: "主线接口",
      maturity: "stable",
      responsibility: "定义 Motion 域内规划服务、轨迹、碰撞、状态和附着箱等跨包消息/服务契约。跨域接口由中央 robot_interfaces 仓库提供。",
      consumes: ["无运行时输入；被其它包编译依赖"],
      produces: ["SolveArmIk.srv", "PlanDualArmIk.srv", "PlanExtract.srv", "PlanLoaded.srv", "CheckCollision.srv", "ExecuteTrajectory.srv", "RunDualArmPoseTask.srv", "RunBoxPairTask.srv", "SetRobotMotionScene.srv", "RobotMotionState.msg", "RobotMotionScene.msg", "MotionPlanCandidate.msg", "AttachedBox.msg"],
      keyFiles: ["ros2_ws/src/motion_internal_interfaces/srv/*.srv", "ros2_ws/src/motion_internal_interfaces/msg/*.msg"],
      statusNotes: ["应优先作为迁移到 robot_motion_control 后的稳定边界。", "接口一旦被外部包使用，字段变化需要版本化或兼容层。"]
    },
    {
      id: "robot_motion_core",
      name: "robot_motion_core",
      layer: "算法核心",
      status: "纯 C++ 公共核心",
      maturity: "active",
      responsibility: "承载不依赖 ROS node、MoveIt 和可视化的运控数据模型与算法接口；当前已统一 IK 候选配置、请求、结果和代价函数类型。",
      consumes: ["Eigen", "纯数值输入"],
      produces: ["IkSolverOptions", "UpdownAwareIkRequest", "UpdownAwareIkCandidate", "UpdownAwareIkResult", "UpdownAwareCostFn"],
      keyFiles: ["ros2_ws/src/robot_motion_core/include/robot_motion_core/ik_candidate_types.hpp", "ros2_ws/src/robot_motion_core/test/test_ik_candidate_types.cpp"],
      statusNotes: ["alfa_robot_moveit_config 与 alfa_robot_benchmarks 现在共同依赖此 Interface，不再互相复制类型。", "候选排序、去重、抽离 rollout 和轨迹评分仍需继续从 MoveIt adapter 迁入。"]
    },
    {
      id: "robot_motion_runtime",
      name: "robot_motion_runtime",
      layer: "运行时服务图",
      status: "服务化起点",
      maturity: "active",
      responsibility: "维护权威 RobotMotionState/RobotMotionScene、任务状态机、能力服务编排和运行状态；不直接实现 IK、碰撞或硬件协议。",
      consumes: ["/joint_states 或 /robot_motion/set_state", "/robot_motion/set_scene 或显式 scene_objects", "箱号任务、左右目标 Pose 或 IK candidate states", "loaded goal family", "JointTrajectory action backend"],
      produces: ["/robot_motion/state", "/robot_motion/scene", "/robot_motion/set_scene", "/robot_motion/run_box_pair_task", "/robot_motion/run_dual_arm_pose_task", "/robot_motion/run_task", "/robot_motion/plan_dual_arm_ik", "/robot_motion/plan_extract", "/robot_motion/plan_loaded", "/robot_motion/execute_trajectory", "/robot_motion/runtime_status", "http://127.0.0.1:8766"],
      keyFiles: ["ros2_ws/src/robot_motion_runtime/README.md", "ros2_ws/src/robot_motion_runtime/launch/runtime_services.launch.py", "ros2_ws/src/robot_motion_runtime/robot_motion_runtime"],
      statusNotes: ["这是从 dual_arm_planner_node 抽脱任务链路的运行时骨架。", "只有本包可以发布权威 `/robot_motion/state` 与 `/robot_motion/scene`。", "算法节点通过 motion_internal_interfaces 接入，不能反向依赖 runtime 私有实现。", "当前仍混有部分临时 Plan 节点；目标是迁入独立 planning service。"]
    },
    {
      id: "robot_motion_scene_service",
      name: "robot_motion_scene_service",
      layer: "场景与碰撞",
      status: "场景能力",
      maturity: "active",
      responsibility: "生成集装箱、箱墙开洞、附着箱、AABB 等几何，并将其同步到 MoveIt PlanningScene。",
      consumes: ["箱子编号", "箱垛几何参数", "抓取 pair", "RobotState", "AttachedBoxSpec"],
      produces: ["CollisionObject", "AttachedCollisionObject", "PlanningScene 快照", "AABB/脱离判断"],
      keyFiles: ["ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_core/scene_geometry.hpp", "ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_scene_adapter.hpp", "ros2_ws/src/robot_motion_scene_service/docs/responsibility.md"],
      statusNotes: ["负责几何事实和碰撞适配，不负责 IK、RRT、抓取顺序或执行。", "场景 revision 必须由权威场景源管理，Rerun 与规划共用同一份数据。"]
    },
    {
      id: "alfa_robot_description",
      name: "alfa_robot_description",
      layer: "机器人模型",
      status: "模型事实源",
      maturity: "active",
      responsibility: "维护当前 URDF/xacro、mesh、ros2_control 标签和模型可视化入口。",
      consumes: ["当前机械臂 mesh", "工具 TCP", "joint limit 相关配置"],
      produces: ["/robot_description", "link/joint/tree", "ros2_control hardware declaration"],
      keyFiles: ["ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro", "ros2_ws/src/alfa_robot_description/meshes/current_robot", "ros2_ws/src/alfa_robot_description/launch/view_alfa_robot.launch.py"],
      statusNotes: ["当前命名已去掉 v5 版本语义。", "不要误删 description 包内 current_robot / 仍被 URDF 引用的 mesh。"]
    },
    {
      id: "alfa_robot_moveit_config",
      name: "alfa_robot_moveit_config",
      layer: "MoveIt 适配",
      status: "待瘦身适配包",
      maturity: "transitional",
      responsibility: "维护 SRDF、规划组、控制器与 MoveIt/FCL adapter；现有全流程编排属于待迁出的历史职责。",
      consumes: ["robot_description", "robot_motion_scene_service", "motion_internal_interfaces", "MoveIt PlanningScene", "箱子目标"],
      produces: ["dual_arm_planner 服务", "motion_collision_service_node", "Rerun/JSON snapshot", "MoveIt planning result", "可执行 JointTrajectory"],
      keyFiles: ["ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/optimized_ik_pipeline.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/extract_planning_pipeline.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/loaded_pose_planning.cpp", "ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py"],
      statusNotes: ["禁止继续新增任务状态机、业务顺序和调试入口。", "dual_arm_planner 现有能力按 core → planning service → runtime 顺序迁出。", "最终只保留 MoveIt 配置、碰撞/规划 adapter 和必要 launch。"]
    },
    {
      id: "alfa_robot_analytic_ik",
      name: "alfa_robot_analytic_ik",
      layer: "IK 算法",
      status: "当前主力 IK",
      maturity: "active",
      responsibility: "提供当前三平行轴机械臂的确定性几何 IK 库，替代大量随机 BioIK 尝试。",
      consumes: ["单臂目标 pose", "固定/候选 updown", "当前 URDF 语义"],
      produces: ["单臂多解 joint candidate", "IK 成功/失败原因"],
      keyFiles: ["ros2_ws/src/alfa_robot_analytic_ik/include", "ros2_ws/src/alfa_robot_analytic_ik/src", "ros2_ws/src/alfa_robot_analytic_ik/test/test_analytic_ik.cpp"],
      statusNotes: ["结构变化后需要重新验证解析假设。", "仍需与碰撞、负重代价和抽离成功率联合评估。"]
    },
    {
      id: "alfa_robot_execution_bridge",
      name: "alfa_robot_execution_bridge",
      layer: "执行适配",
      status: "统一执行入口",
      maturity: "active",
      responsibility: "提供统一轨迹 action，支持 mock 和 ros2_control 转发后端。",
      consumes: ["control_msgs/FollowJointTrajectory", "配置里的 joint 映射与方向策略"],
      produces: ["/alfa_execution/execute_joint_trajectory", "/joint_states", "执行结果"],
      keyFiles: ["ros2_ws/src/alfa_robot_execution_bridge/README.md", "ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge", "ros2_ws/src/alfa_robot_execution_bridge/launch/execution_bridge.launch.py"],
      statusNotes: ["真实硬件方向和 ros2_control 方向必须避免双重翻转。", "mock 后端适合联调，不等于真实执行安全验证。"]
    },
    {
      id: "alfa_robot_rerun",
      name: "alfa_robot_rerun",
      layer: "可视化",
      status: "公共只读适配器",
      maturity: "utility",
      responsibility: "集中维护 URDF 解析、FK、mesh 记录、实时关节状态和离线 JSONL 回放，避免各脚本复制机器人可视化实现。",
      consumes: ["/joint_states", "alfa_robot_description xacro", "JSONL/关节状态序列"],
      produces: ["Rerun viewer", ".rrd recording", "共享 UrdfRobot/FK API"],
      keyFiles: ["ros2_ws/src/alfa_robot_rerun/alfa_robot_rerun/visualize_rerun.py", "ros2_ws/src/alfa_robot_rerun/alfa_robot_rerun/joint_state_viewer_node.py", "ros2_ws/src/alfa_robot_rerun/launch/basic_robot_viewer.launch.py"],
      statusNotes: ["当前只读，不替代 RViz/MoveIt 交互，也不参与算法成功判定。", "benchmark 旧路径只保留兼容包装，不再保存第二份实现。"]
    }
  ],
  nonRosAssets: [
    {
      id: "scripts_ik_benchmark",
      name: "scripts/ik_benchmark",
      role: "IK、可达性和全流程性能实验入口；公共候选类型与 Rerun 实现已迁出。",
      status: "隔离实验资产",
      notes: "正式包不得包含此目录的头文件或动态导入脚本；兼容入口只允许转发到正式包。"
    },
    {
      id: "docs_motion",
      name: "docs/运控",
      role: "运控重构、IK 服务、工程化护栏和工作汇总文档。",
      status: "人类交接资产",
      notes: "本 portal 负责快速导航；详细解释仍引用这些 Markdown。"
    }
  ],
  architecture: {
    headline: {
      kicker: "Motion Intelligence Platform",
      title: "从任务意图到可信执行的双臂运控闭环",
      summary: "以唯一状态与场景事实为基础，把多解 IK、碰撞感知抽离、负重规划、轨迹执行和可观测性组织成可替换、可验证、可迁移的能力链。",
      badges: ["双臂协同", "确定性解析 IK", "阶段化碰撞场景", "统一执行接口", "实时 / 离线可视化"]
    },
    capabilityMetrics: [
      { value: "6", label: "职责层级", note: "契约到装配，依赖单向" },
      { value: "1", label: "状态事实源", note: "硬件、Mock、回放统一汇聚" },
      { value: "5", label: "规划阶段", note: "IK、进场、抽离、让位、负重" },
      { value: "3", label: "可替换适配器", note: "MoveIt、执行后端、可视化" }
    ],
    controlLoop: [
      { id: "intent", label: "任务意图", owner: "上游 / 操作端", detail: "左右 6D Pose、抓取模式、执行策略", contract: "RunDualGraspTask" },
      { id: "runtime", label: "运行时编排", owner: "robot_motion_runtime", detail: "固定状态与场景版本，管理阶段、超时、取消和回执", contract: "request_id / state_id / scene_id" },
      { id: "planning", label: "运动能力服务", owner: "planning + scene service", detail: "多解 IK、候选评分、抽离、负重轨迹、碰撞查询", contract: "SolveIK / PlanExtract / PlanLoaded" },
      { id: "execution", label: "轨迹执行", owner: "execution bridge", detail: "统一 FollowJointTrajectory，适配 Mock、ros2_control 和实机", contract: "/alfa_execution/execute_joint_trajectory" },
      { id: "feedback", label: "状态反馈", owner: "hardware / twin", detail: "关节反馈、执行结果、安全状态回到唯一事实源", contract: "RobotMotionState / TaskReceipt" }
    ],
    truthChain: [
      { title: "模型事实", owner: "alfa_robot_description", content: "URDF、SRDF、TCP、关节限位与碰撞几何" },
      { title: "状态事实", owner: "robot_motion_runtime", content: "当前关节、执行阶段、来源与时间戳" },
      { title: "场景事实", owner: "runtime + scene service", content: "集装箱、箱墙、附着箱与 scene_id" },
      { title: "结果证据", owner: "runtime events + Rerun", content: "候选、拒绝原因、轨迹、耗时与回执" }
    ],
    guardrails: [
      { icon: "01", title: "碰撞一致性", text: "MoveIt/FCL、抽离验证和回放共享同一阶段场景语义。" },
      { icon: "02", title: "接口稳定性", text: "上游只依赖任务契约，不感知 IK、RRT 或硬件实现。" },
      { icon: "03", title: "执行安全", text: "规划不直接碰电机；执行层统一处理取消、超时和后端。" },
      { icon: "04", title: "全链可观测", text: "每阶段保留输入版本、候选数量、失败原因和时间预算。" }
    ],
    principles: [
      {
        title: "唯一事实源",
        rule: "只有 robot_motion_runtime 发布权威 RobotMotionState 与 RobotMotionScene；硬件、mock 和回放只提交观测。",
        prevents: "消除多个 /joint_states 或临时脚本互相覆盖导致的状态跳变。"
      },
      {
        title: "算法与 ROS 分离",
        rule: "IK、候选评分、抽离 rollout、轨迹评价进入纯 core；ROS service 只做消息转换、超时和状态上报。",
        prevents: "避免为了测试算法而启动整套 ROS，也避免算法被锁死在某个 node。"
      },
      {
        title: "适配器不掌管业务",
        rule: "MoveIt、硬件、Rerun 都是 adapter；它们实现能力，不决定抓取顺序、阶段跳转和重试策略。",
        prevents: "防止 alfa_robot_moveit_config 再次长成全流程主包。"
      },
      {
        title: "调试代码单向依赖",
        rule: "工具只能调用公开 interface；任何生产包都不得依赖 benchmark 内 helper 或临时脚本。正式可视化统一依赖 alfa_robot_rerun。",
        prevents: "阻止一次性验证脚本被悄悄融入正式运行链路。"
      }
    ],
    layers: [
      {
        index: "01",
        name: "契约层",
        modules: "motion_internal_interfaces",
        owns: "跨包消息、服务、错误码、request/state/scene id",
        mustNot: "依赖 MoveIt 实现、硬件协议或任务脚本"
      },
      {
        index: "02",
        name: "算法核心层",
        modules: "robot_motion_core / alfa_robot_analytic_ik",
        owns: "IK、多解排序、去重、抽离 rollout、轨迹评分",
        mustNot: "创建 ROS node、读取环境变量、启动进程或写可视化"
      },
      {
        index: "03",
        name: "能力服务层",
        modules: "robot_motion_planning_service（目标） / robot_motion_scene_service",
        owns: "SolveIK、PlanExtract、PlanLoaded、CheckCollision",
        mustNot: "决定任务阶段、直接执行轨迹或成为状态事实源"
      },
      {
        index: "04",
        name: "运行时层",
        modules: "robot_motion_runtime",
        owns: "唯一状态/场景事实、任务状态机、超时、取消、回执",
        mustNot: "包含 IK/RRT/FCL 实现或直接访问电机总线"
      },
      {
        index: "05",
        name: "适配器层",
        modules: "alfa_robot_moveit_config / alfa_robot_execution_bridge / alfa_robot_rerun",
        owns: "MoveIt/FCL、执行后端和只读可视化适配",
        mustNot: "保存业务状态机或产生第二份事实状态"
      },
      {
        index: "06",
        name: "部署层",
        modules: "robot_motion_runtime/launch / docker/motion / 外部 rt-control",
        owns: "选择 adapter、加载参数和启动顺序；硬件生命周期归外部 rt-control",
        mustNot: "计算任务、修改轨迹、包含业务判断"
      }
    ],
    ownership: [
      { concern: "机器人模型", owner: "alfa_robot_description", interface: "robot_description", forbidden: "在 planner/bringup 复制 joint、limit 或 TCP 常量" },
      { concern: "当前机器人状态", owner: "robot_motion_runtime", interface: "/robot_motion/state", forbidden: "规划器直接把任意 /joint_states 当权威状态" },
      { concern: "当前场景", owner: "robot_motion_runtime + robot_motion_scene_service", interface: "/robot_motion/scene + scene_id", forbidden: "Rerun、MoveIt 和算法各建一套障碍" },
      { concern: "IK/抽离/负重规划", owner: "robot_motion_core + planning service", interface: "SolveIK / PlanExtract / PlanLoaded", forbidden: "写进 bringup、dashboard 或执行 bridge" },
      { concern: "MoveIt/FCL 调用", owner: "alfa_robot_moveit_config adapter", interface: "规划与碰撞 adapter", forbidden: "在此包维护任务顺序和重试状态机" },
      { concern: "任务状态机与回执", owner: "robot_motion_runtime", interface: "RunDualGraspTask / TaskReceipt", forbidden: "执行层猜测任务是否完成" },
      { concern: "轨迹执行", owner: "alfa_robot_execution_bridge", interface: "FollowJointTrajectory", forbidden: "重新规划、修正 IK 或发布伪权威状态" },
      { concern: "测试与审计", owner: "robot_motion_tools（目标） / scripts/ik_benchmark", interface: "只调用公开服务并订阅事件", forbidden: "被任何生产包编译或运行依赖" }
    ],
    promotion: [
      { stage: "实验", location: "scripts/ik_benchmark 或 robot_motion_tools", gate: "可以快速迭代，但只能调用稳定 interface。" },
      { stage: "算法候选", location: "独立 pure core + 单测", gate: "去掉 CLI、ROS、文件路径和可视化副作用。" },
      { stage: "能力服务", location: "planning/scene service adapter", gate: "定义超时、取消、错误码、输入输出和回归测试。" },
      { stage: "正式流程", location: "robot_motion_runtime", gate: "任务状态机只编排能力服务，不复制算法。" }
    ],
    migration: [
      { phase: "A · 立即约束", result: "清理跨部门旧包；部署层只装配；门户记录唯一官方入口。" },
      { phase: "B · 抽出规划能力", result: "已建立 robot_motion_core 并迁入 IK 候选公共接口；继续迁移排序、去重、rollout 与评分，ROS 节点迁入 planning service。" },
      { phase: "C · 收窄 MoveIt 包", result: "alfa_robot_moveit_config 只保留 SRDF、规划器配置、MoveIt/FCL adapter。" },
      { phase: "D · 建立系统测试", result: "独立 system_tests 通过服务启动、注入 state/scene、运行任务并验证回执，不导入私有实现。" },
      { phase: "E · 迁移新仓库", result: "按上述依赖方向迁入 robot_motion_control，旧仓库只保留历史追溯。" }
    ]
  },
  statusLegend: [
    { key: "stable", label: "稳定接口", color: "green", meaning: "可以作为跨包或跨仓库引用的契约。" },
    { key: "active", label: "主线活跃", color: "blue", meaning: "当前流程正在使用，仍可能随实验调整。" },
    { key: "transitional", label: "过渡主包", color: "orange", meaning: "功能可用但职责偏重，后续需要拆分或迁移。" },
    { key: "lab", label: "实验工具", color: "purple", meaning: "用于 benchmark/验证，不直接代表生产服务。" },
    { key: "external", label: "外部依赖", color: "gray", meaning: "驱动或第三方算法，尽量不要承载业务语义。" },
    { key: "mock-or-early", label: "早期/Mock", color: "red", meaning: "职责或接口还需确认，不应当成稳定事实源。" }
  ],
  recommendedEntrypoints: [
    {
      title: "看系统全貌",
      href: "index.html",
      hint: "先看外界和系统的交互，以及主链路边界。"
    },
    {
      title: "看流程细节",
      href: "flows.html",
      hint: "查看任务到运动、碰撞、IK 抽离、孪生链路。"
    },
    {
      title: "找包职责",
      href: "packages.html",
      hint: "搜索包名，进入包状态和输入输出页面。"
    },
    {
      title: "看目标架构",
      href: "architecture.html",
      hint: "查看唯一事实源、依赖方向、职责矩阵和调试代码晋升规则。"
    }
  ]
};
