window.SYSTEM_PORTAL_DATA = {
  meta: {
    title: "ALFA Robot 运控系统地图",
    subtitle: "从外界交互到 ROS2 包职责的可点击流程导航",
    updated: "2026-07-09",
    branchHint: "当前整理基于 alfa_robot 工作区与当前分支源码；迁移到 robot_motion_control 时应同步更新本数据文件。",
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
      id: "perception",
      name: "感知系统",
      role: "识别箱垛、输出箱子位置和任务目标；当前主流程中大量目标仍可由脚本/benchmark 注入。",
      interfaces: ["box_perception 消息", "任务坐标", "箱子编号"]
    },
    {
      id: "planner",
      name: "运控规划服务",
      role: "把箱子目标转成 IK 候选、抽离轨迹、负重规划和可执行 JointTrajectory。",
      interfaces: ["robot_motion_interfaces", "MoveIt PlanningScene", "JSON/Rerun 记录"]
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
        { name: "任务输入", owner: "box_perception / benchmark / operator", data: "箱子编号、左右目标、吸附模式 front/top_suction、箱墙位置", output: "任务上下文 MotionContext" },
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
      title: "数字孪生 / 可视化链路",
      summary: "当前已有 Rerun、RViz 和 MuJoCo 相关入口，但权威状态源仍在建设中。",
      stages: [
        { name: "机器人模型", owner: "alfa_robot_description", data: "URDF/xacro、mesh、joint limits", output: "robot_description" },
        { name: "状态输入", owner: "robot_motion_runtime / joint_state_broadcaster", data: "/joint_states 或 /robot_motion/set_state", output: "RobotMotionState 或 TF" },
        { name: "Rerun 回放", owner: "alfa_robot_rerun + scripts", data: "RobotState/trajectory/scene JSON", output: ".rrd 动态回放" },
        { name: "RViz/MoveIt", owner: "alfa_robot_moveit_config", data: "PlanningScene + robot_description", output: "真实规划场景显示" },
        { name: "MuJoCo", owner: "simulation/mujoco + bridge scripts", data: "仿真 XML / semantic scene", output: "未来可接入孪生状态源" }
      ]
    }
  ],
  packages: [
    {
      id: "robot_motion_interfaces",
      name: "robot_motion_interfaces",
      layer: "接口契约",
      status: "主线接口",
      maturity: "stable",
      responsibility: "定义运控服务、轨迹、碰撞、状态和附着箱等跨包消息/服务契约。",
      consumes: ["无运行时输入；被其它包编译依赖"],
      produces: ["SolveArmIk.srv", "PlanDualArmIk.srv", "PlanExtract.srv", "PlanLoaded.srv", "CheckCollision.srv", "ExecuteTrajectory.srv", "RunDualArmPoseTask.srv", "RunBoxPairTask.srv", "SetRobotMotionScene.srv", "RobotMotionState.msg", "RobotMotionScene.msg", "MotionPlanCandidate.msg", "AttachedBox.msg"],
      keyFiles: ["ros2_ws/src/robot_motion_interfaces/srv/*.srv", "ros2_ws/src/robot_motion_interfaces/msg/*.msg"],
      statusNotes: ["应优先作为迁移到 robot_motion_control 后的稳定边界。", "接口一旦被外部包使用，字段变化需要版本化或兼容层。"]
    },
    {
      id: "robot_motion_runtime",
      name: "robot_motion_runtime",
      layer: "运行时服务图",
      status: "服务化起点",
      maturity: "active",
      responsibility: "提供权威 RobotMotionState/RobotMotionScene 源、箱号任务 adapter、PlanDualArmIk/PlanExtract/PlanLoaded/ExecuteTrajectory 服务、任务编排服务和运行时前端。",
      consumes: ["/joint_states 或 /robot_motion/set_state", "/robot_motion/set_scene 或显式 scene_objects", "箱号任务、左右目标 Pose 或 IK candidate states", "loaded goal family", "JointTrajectory action backend"],
      produces: ["/robot_motion/state", "/robot_motion/scene", "/robot_motion/set_scene", "/robot_motion/run_box_pair_task", "/robot_motion/run_dual_arm_pose_task", "/robot_motion/run_task", "/robot_motion/plan_dual_arm_ik", "/robot_motion/plan_extract", "/robot_motion/plan_loaded", "/robot_motion/execute_trajectory", "/robot_motion/runtime_status", "http://127.0.0.1:8766"],
      keyFiles: ["ros2_ws/src/robot_motion_runtime/README.md", "ros2_ws/src/robot_motion_runtime/launch/runtime_services.launch.py", "ros2_ws/src/robot_motion_runtime/robot_motion_runtime"],
      statusNotes: ["这是从 dual_arm_planner_node 抽脱任务链路的第一层运行时骨架。", "`runtime_full_stack.launch.py` 会同时启动 runtime、解析 IK 和碰撞服务，并默认让 PlanExtract/PlanLoaded 调用碰撞服务过滤候选。", "箱号任务入口 `/robot_motion/run_box_pair_task` 已接入第一版 5×5 箱墙几何 adapter，内部转发 `/robot_motion/run_dual_arm_pose_task`。", "场景事实入口 `/robot_motion/set_scene` 已接入；PlanExtract/PlanLoaded 会把请求 scene 或 `/robot_motion/scene` 传给碰撞服务。", "当前 PlanExtract/PlanLoaded 仍是 shortcut 候选生成；完整 C++ 抽离 rollout 和 RRT/local-RRT 迁移还未完成。"]
    },
    {
      id: "robot_motion_scene_service",
      name: "robot_motion_scene_service",
      layer: "场景与碰撞",
      status: "核心库",
      maturity: "active",
      responsibility: "生成集装箱、箱墙开洞、附着箱、AABB 等几何，并将其同步到 MoveIt PlanningScene。",
      consumes: ["箱子编号", "箱垛几何参数", "抓取 pair", "RobotState", "AttachedBoxSpec"],
      produces: ["CollisionObject", "AttachedCollisionObject", "PlanningScene 快照", "AABB/脱离判断"],
      keyFiles: ["ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_core/scene_geometry.hpp", "ros2_ws/src/robot_motion_scene_service/include/robot_motion_scene_service/motion_scene_adapter.hpp", "ros2_ws/src/robot_motion_scene_service/docs/responsibility.md"],
      statusNotes: ["当前名字带 service，但本质是 C++ core + MoveIt adapter，不是 ROS service 节点。", "不负责 IK、RRT、抓取顺序或执行。"]
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
      layer: "规划与实验编排",
      status: "过渡主包",
      maturity: "transitional",
      responsibility: "MoveIt 配置、dual_arm_planner、IK/抽离/负重流程装配、碰撞服务、Rerun 快照生成。",
      consumes: ["robot_description", "robot_motion_scene_service", "robot_motion_interfaces", "MoveIt PlanningScene", "箱子目标"],
      produces: ["dual_arm_planner 服务", "motion_collision_service_node", "Rerun/JSON snapshot", "MoveIt planning result", "可执行 JointTrajectory"],
      keyFiles: ["ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/optimized_ik_pipeline.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/extract_planning_pipeline.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/loaded_pose_planning.cpp", "ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py"],
      statusNotes: ["当前仍承担过多业务职责，后续应继续迁移到专门运控包。", "负重规划支持 rrt/shortcut；shortcut 已增加局部 RRT 修补分支。", "部分脚本仍是 benchmark/demo 性质，不应视作长期服务。"]
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
      id: "alfa_robot_bringup",
      name: "alfa_robot_bringup",
      layer: "启动装配",
      status: "运行入口",
      maturity: "active",
      responsibility: "装配 robot_state_publisher、ros2_control、controller spawner、实机/测试启动脚本。",
      consumes: ["robot_description", "controller yaml", "hardware plugin 参数"],
      produces: ["controller_manager", "joint_state_broadcaster", "controller command topics"],
      keyFiles: ["ros2_ws/src/alfa_robot_bringup/launch/alfa_robot.launch.py", "ros2_ws/src/alfa_robot_bringup/launch/moveit_real_execute.launch.py", "ros2_ws/src/alfa_robot_bringup/config"],
      statusNotes: ["启动顺序和 controller 名称是硬约束。", "部分 launch 是测试入口，不能全部视为生产入口。"]
    },
    {
      id: "alfa_robot_hardware",
      name: "alfa_robot_hardware",
      layer: "硬件控制",
      status: "实机底座",
      maturity: "active-risky",
      responsibility: "作为 ros2_control SystemInterface 连接真实电机总线、读写关节状态和命令。",
      consumes: ["controller command interface", "hardware parameters", "CAN/CANopen/RMD 反馈"],
      produces: ["state interface", "电机命令", "硬件生命周期状态"],
      keyFiles: ["ros2_ws/src/alfa_robot_hardware/src", "docs/CONTROL_LAYER_HARDCODED_PARAMS.md", "docs/ethercat/joint_direction_calibration.md"],
      statusNotes: ["总线节点、方向、减速比、安全停机仍有硬编码风险。", "重构必须保留首帧命令初始化、turn 软件零点等行为。"]
    },
    {
      id: "alfa_robot_rerun",
      name: "alfa_robot_rerun",
      layer: "可视化",
      status: "轻量观察器",
      maturity: "utility",
      responsibility: "订阅 /joint_states，用 URDF FK 在 Rerun 中显示机器人；也作为后续可视化基础。",
      consumes: ["/joint_states", "alfa_robot_description xacro"],
      produces: ["Rerun viewer", ".rrd recording"],
      keyFiles: ["ros2_ws/src/alfa_robot_rerun/README.md", "ros2_ws/src/alfa_robot_rerun/launch/basic_robot_viewer.launch.py"],
      statusNotes: ["当前只读，不替代 RViz/MoveIt 交互。", "没有规划场景、碰撞和感知 overlay。"]
    },
    {
      id: "alfa_robot_benchmarks",
      name: "alfa_robot_benchmarks",
      layer: "实验与回归",
      status: "实验工具",
      maturity: "lab",
      responsibility: "运行 IK、并行求解、箱垛抓取、updown solver 等 benchmark 和分析脚本。",
      consumes: ["MoveIt 配置", "IK 参数", "测试范围", "实验 JSON/CSV"],
      produces: ["benchmark 可执行", "CSV/JSONL", "图表", "Rerun"],
      keyFiles: ["ros2_ws/src/alfa_robot_benchmarks/src/updown_solver_comparison_benchmark.cpp", "ros2_ws/src/alfa_robot_benchmarks/scripts", "data/ik_benchmark"],
      statusNotes: ["不应把 benchmark 代码直接当作生产状态机。", "很多历史数据位于 data/，提交时需区分验证产物和源码。"]
    },
    {
      id: "box_perception",
      name: "box_perception",
      layer: "感知",
      status: "感知入口",
      maturity: "active",
      responsibility: "面向箱子识别/卸货场景的视觉感知节点。",
      consumes: ["相机/图像/点云输入", "感知配置"],
      produces: ["箱子检测结果", "box_perception_msgs"],
      keyFiles: ["ros2_ws/src/box_perception/launch/perception.launch.py", "ros2_ws/src/box_perception/box_perception"],
      statusNotes: ["当前运控 benchmark 中常用脚本目标代替真实感知输出。", "与任务编排的稳定接口仍需收敛。"]
    },
    {
      id: "box_perception_msgs",
      name: "box_perception_msgs",
      layer: "感知接口",
      status: "消息包",
      maturity: "active",
      responsibility: "定义箱子感知自定义消息。",
      consumes: ["无运行时输入"],
      produces: ["感知消息类型"],
      keyFiles: ["ros2_ws/src/box_perception_msgs/msg", "ros2_ws/src/box_perception_msgs/CMakeLists.txt"],
      statusNotes: ["应与 robot_motion_interfaces 明确边界：感知输出 vs 运控任务契约。"]
    },
    {
      id: "bio_ik",
      name: "bio_ik",
      layer: "第三方 IK",
      status: "保留依赖",
      maturity: "legacy-support",
      responsibility: "提供 BioIK MoveIt 插件，主要用于历史双末端随机 IK 对比和兼容。",
      consumes: ["MoveIt IK request", "随机种子/timeout"],
      produces: ["IK solution"],
      keyFiles: ["ros2_ws/src/bio_ik/README.md", "ros2_ws/src/bio_ik"],
      statusNotes: ["随机性强，已不是当前稳定流程唯一能力来源。", "保留用于对照和历史实验。"]
    },
    {
      id: "fast_lio",
      name: "fast_lio",
      layer: "雷达定位",
      status: "导航/建图依赖",
      maturity: "external",
      responsibility: "LiDAR-Inertial odometry/mapping，提供雷达建图定位能力。",
      consumes: ["Livox/点云/IMU"],
      produces: ["里程计", "地图", "点云"],
      keyFiles: ["ros2_ws/src/fast_lio/launch/mapping.launch.py", "ros2_ws/src/fast_lio/src"],
      statusNotes: ["与当前抽箱运控链路相对独立。", "运行成本和参数标定需要导航侧维护。"]
    },
    {
      id: "livox_ros_driver2",
      name: "livox_ros_driver2",
      layer: "雷达驱动",
      status: "外部驱动",
      maturity: "external",
      responsibility: "Livox 3D LiDAR ROS2 驱动。",
      consumes: ["Livox 硬件数据"],
      produces: ["Livox 点云/IMU topic"],
      keyFiles: ["ros2_ws/src/livox_ros_driver2", "ros2_ws/src/livox_ros_driver2/launch"],
      statusNotes: ["属于外设驱动，不应混入运控算法职责。"]
    },
    {
      id: "alfa_robot_twist_mux",
      name: "alfa_robot_twist_mux",
      layer: "底盘/急停辅助",
      status: "待梳理",
      maturity: "mock-or-early",
      responsibility: "底盘 twist mux、teleop 和急停相关辅助入口。",
      consumes: ["teleop/cmd_vel/estop 输入"],
      produces: ["mux 后 twist / 安全辅助状态"],
      keyFiles: ["ros2_ws/src/alfa_robot_twist_mux/launch/bringup.launch.py", "ros2_ws/src/alfa_robot_twist_mux/readme.md"],
      statusNotes: ["package 描述仍为 TODO，职责需要进一步工程化确认。"]
    }
  ],
  nonRosAssets: [
    {
      id: "simulation_mujoco",
      name: "simulation/mujoco",
      role: "MuJoCo 仿真资产、生成脚本和同步 bridge 的来源之一。",
      status: "实验孪生资产",
      notes: "当前还不是权威数字孪生服务；后续要和 RobotMotionState、MotionSceneService 对齐。"
    },
    {
      id: "scripts_ik_benchmark",
      name: "scripts/ik_benchmark",
      role: "历史 IK demo、range grid、可达性与实验脚本集合。",
      status: "历史/实验入口",
      notes: "继续使用前要确认加载的是当前安装后的 MoveIt 配置和当前机械臂命名。"
    },
    {
      id: "docs_motion",
      name: "docs/运控",
      role: "运控重构、IK 服务、工程化护栏和工作汇总文档。",
      status: "人类交接资产",
      notes: "本 portal 负责快速导航；详细解释仍引用这些 Markdown。"
    }
  ],
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
    }
  ]
};
