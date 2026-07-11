// 架构评审数据源（2026-07-11）
// 由深度架构评审生成：17 个映射代理逐包/横切审读 + 对抗复核（30 项确认 / 10 项部分成立 / 0 项被驳回）。
// 页面：architecture.html / target_architecture.html / refactor_plan.html（渲染逻辑在 app.js）。
// 问题关闭或推进后请更新对应条目的 verified / severity / 描述。
window.ARCHITECTURE_REVIEW_DATA = {
  meta: {
    reviewSubtitle: "基于 package.xml、CMakeLists、launch 与源码逐行核实的包边界评审：257 条原始发现，去重 171 条，40 条高危经独立对抗复核。",
    targetSubtitle: "目标不是重写，而是把已经存在的正确方向（robot_motion_* 服务化）走完：让每个包回到自己的名字。",
    planSubtitle: "按批次增量迁移，每批独立可回滚；P0 是纯缺陷修复，不动架构。"
  },

  verdict: {
    headline: "结论：配置包变成了上帝包；系统当前没有一条端到端可用的执行链；但依赖无环，增量修复在拓扑上无阻塞。",
    points: [
      {
        title: "上帝包已经形成",
        body: "alfa_robot_moveit_config 名义上是 MoveIt Setup Assistant 自动生成的配置包（package.xml 仍这样自称），实际持有约 15,800 行 C++（dual_arm_planner_node 单文件 4,525 行）、2 个导出库、3 个运行时服务节点、rosidl 接口、MuJoCo 桥和 24 个安装脚本，并自带完整 ros2_control 启动。今天所有三条活的启动路径都以它为重心。"
      },
      {
        title: "执行链现状为零",
        body: "两条执行路径都断：v5 runtime 用 leftjointN 命名，执行桥契约是 left_jointN，goal 被拒收；唯一跑过实机的 execute_l6_r8 脚本链在 MOTION-59 改名后映射函数退化为恒等函数，任务段 12 个臂关节静默冻结且能通过桥校验。仓库没有任何 CI，两处断裂均无检测手段。"
      },
      {
        title: "修复在拓扑上无阻塞",
        body: "复核确认 moveit_config 与 robot_motion_* 之间没有循环依赖，只是单向倒置（runtime 依赖 config 包，因为两个服务节点住错了地方）。robot_motion_interfaces / scene_service / runtime 的分层方向是对的，把节点搬出去即可解开，无需重写。"
      }
    ]
  },

  categories: [
    { key: "confirmed-architectural-problem", label: "确认的架构问题", tone: "red", meaning: "有文件级证据、经复核确认的结构性问题。" },
    { key: "suspected-architectural-problem", label: "疑似架构问题", tone: "orange", meaning: "证据指向问题但仍需运行时验证。" },
    { key: "code-defect", label: "代码缺陷", tone: "red", meaning: "会产生错误行为的具体 bug。" },
    { key: "hidden-coupling", label: "隐性耦合", tone: "orange", meaning: "靠字符串/路径/布局约定维系的跨包契约。" },
    { key: "responsibility-violation", label: "职责越界", tone: "orange", meaning: "代码放在与其职责不符的包里。" },
    { key: "improper-dependency", label: "不当依赖", tone: "orange", meaning: "方向错误、未声明或逃逸出工作区的依赖。" },
    { key: "duplicated-implementation", label: "重复实现", tone: "purple", meaning: "同一事实存在两份以上实现且可漂移。" },
    { key: "debug-leak", label: "调试代码泄漏", tone: "red", meaning: "试车/调试/mock 代码进入生产路径。" },
    { key: "reasonable-compromise", label: "合理妥协", tone: "green", meaning: "虽不优雅但当前合理的临时选择。" },
    { key: "unnecessary-complexity", label: "多余复杂度", tone: "gray", meaning: "可直接删除或简化的部分。" },
    { key: "process-gap", label: "流程缺口", tone: "blue", meaning: "CI/文档/部署清单层面的系统性缺口。" }
  ],

  findings: [
    // ===== A. 上帝包与包边界 =====
    {
      id: "A-01", category: "confirmed-architectural-problem", severity: "critical", verified: "CONFIRMED",
      title: "moveit_config 吸收了整个运控运行时",
      packages: ["ros2_ws/src/alfa_robot_moveit_config"],
      evidence: "package.xml:6-8 仍自称 'automatically generated'；CMakeLists.txt:45-227 构建并导出 alfa_robot_motion_core / alfa_robot_motion_scene_adapter 两个库和 3 个可执行；src/ 共 15,788 行 C++，dual_arm_planner_node.cpp 4,525 行，内含 plan_and_execute / run_box_stack_flow / extract_monitor 等任务级服务。",
      problem: "配置包的契约是可由 Setup Assistant 重新生成的声明式配置；现在重新生成会摧毁运行时，包名与内容完全脱节。",
      consequence: "改任何规划逻辑都要重建『配置包』；robot_motion_runtime 被迫 exec_depend 一个配置包；新工程师按包名找不到规划器。",
      recommendation: "新建 robot_motion_planning 包，把 src/、include/、msg/、srv/ 与 planner/IK/碰撞 launch 整体迁出；moveit_config 只留 SRDF/YAML/move_group/rviz。",
      risk: "中高：迁移期间需要冻结服务名与 launch 参数名；建议放在 P3 批次整体搬移。"
    },
    {
      id: "A-02", category: "improper-dependency", severity: "high", verified: "CONFIRMED",
      title: "依赖倒置：runtime 层依赖配置包获取自己的 /robot_motion 服务",
      packages: ["ros2_ws/src/robot_motion_runtime", "ros2_ws/src/alfa_robot_moveit_config"],
      evidence: "robot_motion_runtime/package.xml:21 exec_depend alfa_robot_moveit_config；runtime_full_stack.launch.py:41、49 include 配置包的 analytic_arm_ik_service.launch.py 与 motion_collision_service.launch.py——因为这两个服务节点物理上住在配置包里。",
      problem: "v5 运行时（当前主线）无法脱离旧 monolith 部署，依赖闭包被拖入完整 MoveIt 栈、warehouse_ros_mongo、rviz 插件。",
      consequence: "runtime 无法独立 CI/部署；配置包任何构建故障都阻塞运行时。",
      recommendation: "motion_collision_service_node 移入 robot_motion_scene_service（让包名终于名副其实）；analytic_arm_ik_service_node 移入 alfa_robot_analytic_ik 侧的薄服务包；然后删除该 exec_depend。",
      risk: "低：服务名不变，只挪节点与 launch。"
    },
    {
      id: "A-03", category: "responsibility-violation", severity: "high", verified: "PARTIAL",
      title: "dual_arm_planner.launch.py 自带完整系统装配（第二个 bringup）",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py"],
      evidence: "launch:311-330 自行启动 move_group + rsp + 静态 TF + controller_manager/ros2_control_node（加载本包 config/ros2_controllers.yaml）+ spawner，共约 130 个 launch 参数。复核修正：bringup 并非『被它取代』——bringup 自身早已 bit-rot（见 B-02），而是启动职责整体迁走后无人回收。",
      problem: "启动/控制器归属从装配层静默迁入规划配置包，与 bringup 的 YAML 形成两个平行控制器宇宙。",
      consequence: "每个启动改动都落进一个 330 行、混合规划参数与系统装配的文件；控制器改名要同步 3 处。",
      recommendation: "把 311-330 行抽为 bringup 的 robot_system.launch.py；dual_arm_planner.launch.py 退化为仅启动节点并声明前置条件。",
      risk: "中：是当前活入口，需要一次带回归 smoke 的原子切换。"
    },
    {
      id: "A-04", category: "improper-dependency", severity: "critical", verified: "PARTIAL",
      title: "生产构建经 ../../../ 逃逸到工作区外的 scripts/ik_benchmark 并转售其头文件",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/CMakeLists.txt", "scripts/ik_benchmark"],
      evidence: "CMakeLists.txt:43 set(IK_BENCHMARK_ROOT .../../../scripts/ik_benchmark)；:88/:127 加入 include 路径；:230 把外部头文件当作本包 API 安装；:258-261 ctest 引用 ../../../scripts/safety/check_l6_r8_real_safety.py。导出头文件 optimized_ik_pipeline.hpp:4 等直接 #include ik_benchmark/*.h。复核补充：scripts/ik_benchmark 本身是个叫 alfa_robot_benchmarks 的完整 ament 包，与 moveit_config 互相 exec_depend。",
      problem: "包不再自包含：colcon 依赖图对这条边完全失明，构建正确性取决于仓库目录形状。",
      consequence: "release/tarball/其他 checkout 布局下 configure 失败；--packages-select 静默用陈旧头文件；生产 IK 数据模型被 benchmark 目录拥有。",
      recommendation: "把 parallel_updown_aware_ik_solver 等生产依赖的求解器提升为工作区内真包（如 alfa_robot_ik_pipeline），双方 find_package；删除 IK_BENCHMARK_ROOT 与外部 install 规则。",
      risk: "中：纯构建结构改动，行为不变，但要一次干净全量构建验证。"
    },
    {
      id: "A-05", category: "code-defect", severity: "high", verified: "CONFIRMED",
      title: "alfa_robot_benchmarks 是指向开发机绝对路径的悬空符号链接",
      packages: ["ros2_ws/src/alfa_robot_benchmarks"],
      evidence: "git ls-tree 显示 120000 符号链接，内容为绝对路径 /mnt/mydisk/ALFA/alfa_robot/scripts/ik_benchmark；在本 checkout 上 ls 直接失败（悬空）。",
      problem: "该『包』只在原开发机的确切挂载路径下存在，其他机器上从工作区静默消失。",
      consequence: "benchmark 可执行在新机器/CI 上不可构建；掩盖了 A-04 中生产头文件的真实归属。",
      recommendation: "删除符号链接，把 scripts/ik_benchmark 物理迁入 ros2_ws/src/（与 A-04 一并处理）。",
      risk: "低。"
    },
    {
      id: "A-06", category: "responsibility-violation", severity: "medium", verified: "",
      title: "rosidl 接口生成在配置包里而不是接口包",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/msg", "ros2_ws/src/robot_motion_interfaces"],
      evidence: "CMakeLists.txt:34-39 在配置包内 rosidl_generate_interfaces：SemanticCargo.msg、SemanticScene.msg、ConfigureExtractMonitor.srv。",
      problem: "配置包同时成为接口包，消费语义场景消息的任何节点都要依赖整个 moveit_config。",
      consequence: "接口演进与配置演进互相绑架。",
      recommendation: "三个接口文件迁入 robot_motion_interfaces（与 RobotMotionState 同层）。",
      risk: "低：需要同步更新 import/include 与 CMake 依赖。"
    },
    {
      id: "A-07", category: "responsibility-violation", severity: "medium", verified: "",
      title: "MuJoCo 数字孪生桥住在配置包里，靠 parents[4] 爬回仓库根",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/scripts", "simulation/mujoco"],
      evidence: "mujoco_sync_view / mujoco_planning_scene_bridge / mujoco_digital_twin 三个 launch 都以 get_package_share_directory(...).parents[4] 定位 simulation/mujoco/scene.xml；scene.xml 又用源码树相对路径引用 description 的 mesh。",
      problem: "仿真桥只在『install 树恰好嵌在完整仓库 checkout 里』时工作。",
      consequence: "任何独立部署（merge-install、二进制安装）下孪生链路全部失效。",
      recommendation: "建立 alfa_robot_mujoco_sim 包（或把 simulation/mujoco 升级为包），把 XML 资产装进 share/ 并用 ament 索引解析。",
      risk: "低。"
    },

    // ===== B. 启动与配置重复 =====
    {
      id: "B-01", category: "confirmed-architectural-problem", severity: "high", verified: "CONFIRMED",
      title: "同名 dual_arm_controller 在两个包里是两种不兼容定义",
      packages: ["ros2_ws/src/alfa_robot_bringup/config/alfa_robot_moveit_real_controllers.yaml", "ros2_ws/src/alfa_robot_moveit_config/config/ros2_controllers.yaml"],
      evidence: "bringup 实机 YAML：update_rate 200，dual_arm_controller = 15 关节（含 pitch/turn）；moveit_config：update_rate 100，torso_controller(pitch,turn) + dual_arm_controller(13 关节)。moveit_controllers.yaml 声明的是 torso+dual_arm 拆分，而 bringup 只 spawn 一个控制器。",
      problem: "一个控制器名，两份互斥拓扑，横跨两个包，没有单一属主。",
      consequence: "bringup 的实机 MoveIt 启动即使成功，轨迹执行也必然失败（pitch/turn 找不到 action server）；规划成功、执行报『Unable to identify controller』。",
      recommendation: "moveit_config 的 ros2_controllers.yaml + moveit_controllers.yaml 作为控制器拓扑唯一属主（P4 后随启动收口移交 bringup）；bringup 的 YAML 收缩为只补充 all_position_controller 的实机 overlay。",
      risk: "中：需要在 mock 栈上做一次完整规划→执行回归。"
    },
    {
      id: "B-02", category: "confirmed-architectural-problem", severity: "high", verified: "PARTIAL",
      title: "bringup 已 bit-rot：引用不存在的可执行、未安装的脚本、被静默丢弃的参数",
      packages: ["ros2_ws/src/alfa_robot_bringup"],
      evidence: "moveit_real_execute.launch.py:178-182 引用 executable=path_node.py，moveit_real_hardware_test.launch.py:160-181 引用 trajectory_executor 与 path——三者在 moveit_config 的构建产物中均不存在；auto_grasp.launch.py 启动的 auto_grasp_node.py 从未被 CMake 安装；两个实机 launch 传给 move_group.launch.py 的 robot_description 参数被 MSA 生成的 launch 静默丢弃；全工作区没有任何包依赖 bringup。复核修正：NOW.md 并未声明 bringup 弃用，它 bit-rot 是事实但『官方弃用』说法无文档支撑。",
      problem: "名义上的启动装配包与现实系统脱钩超过一个模型代际。",
      consequence: "按包名走正门启动的人得到的是坏门；auto 标志一开 launch 直接 executable-not-found。",
      recommendation: "P4 批次：删除/归档死 launch 与死 YAML，把 A-03 抽出的 robot_system.launch.py 落进来，让 bringup 重新成为唯一装配入口。",
      risk: "低（删除死物）到中（复活为唯一入口需要回归）。"
    },
    {
      id: "B-03", category: "duplicated-implementation", severity: "medium", verified: "",
      title: "配置包内的第二份 ros2_control 定义已死，但基线清单仍把它当权威",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/config/alfa_robot.ros2_control.xacro"],
      evidence: "该文件硬编码 mock_components/GenericSystem + mock_sensor_commands=true，定义与 description 宏同名的 15 关节 ros2_control；全工作区唯一引用者是 config/motion_baselines/current_motion_baseline.yaml（把它的哈希当作『当前模型』记录）。",
      problem: "死配置以权威姿态存在，是典型的漂移陷阱。",
      consequence: "基线校验校的是一个不再被加载的文件；下一个人可能改错文件。",
      recommendation: "删除该 xacro 与包内 initial_positions.yaml 死副本；baseline 清单指向 description 的宏。",
      risk: "低。"
    },
    {
      id: "B-04", category: "hidden-coupling", severity: "medium", verified: "",
      title: "核心配置事实无单一来源：updown 行程 0.92 vs 0.99、初始位姿四处、限位三处",
      packages: ["ros2_ws/src/alfa_robot_description", "ros2_ws/src/alfa_robot_moveit_config"],
      evidence: "横切配置代理核实：URDF 声明 updown 0.92 m，planner 默认值与 baseline 清单是 0.99 m；关节列表在 URDF、死 xacro、AlfaRobotHW C++、6 份控制器 YAML、7+ 个 Python 脚本中重复声明。",
      problem: "同一物理事实的副本之间已经出现有运行后果的分歧。",
      consequence: "规划器可能规划出超出实际行程的 updown 目标。",
      recommendation: "限位/初始姿态归 description 单源；其余消费方读 URDF 或共享 YAML（P2 批次契约单一化的一部分）。",
      risk: "中：要先确认 0.92 与 0.99 哪个是当前机器的真值。"
    },
    {
      id: "B-05", category: "debug-leak", severity: "medium", verified: "",
      title: "mock 在三层同时是默认值",
      packages: ["ros2_ws/src/alfa_robot_description/urdf/alfa_robot.urdf.xacro", "ros2_ws/src/alfa_robot_execution_bridge"],
      evidence: "description 顶层 xacro use_mock_hardware 默认 true；execution_bridge_node 的 mode 参数默认 mock；bridge 默认 launch 配置也是 mock。",
      problem: "『默认安全』合理，但三层叠加意味着没有一处显式声明当前运行的是仿真还是实机。",
      consequence: "误以为在驱动实机而实际在 mock（或相反）的空间很大；bridge 的 mock 后端还以执行层命名发布 /joint_states，与模型命名冲突。",
      recommendation: "保留 mock 默认，但 bridge 的 mode 改为必填参数并在 mock 模式打醒目横幅；launch 全部显式传递。",
      risk: "低。"
    },

    // ===== C. 契约断裂与隐性耦合 =====
    {
      id: "C-01", category: "confirmed-architectural-problem", severity: "critical", verified: "CONFIRMED",
      title: "系统当前没有一条端到端可用的执行链",
      packages: ["ros2_ws/src/robot_motion_runtime", "ros2_ws/src/alfa_robot_execution_bridge", "ros2_ws/src/alfa_robot_moveit_config/scripts"],
      evidence: "综合复核（critic 代理专项核实）：(1) v5 主线 runtime 轨迹用 leftjointN（common.py:15-31，与 SRDF 一致），bridge 契约是 left_jointN（joints.py:18-31），execution_bridge_node.py:181-183 拒收 goal；runtime_full_stack.launch.py 甚至不启动 bridge。(2) 唯一驱动过实机的 execute_l6_r8 链因 C-02 静默不动臂。",
      problem: "两种命名约定（leftjointN vs left_jointN）从未有单一属主，靠字符串前缀手术在包间翻译。",
      consequence: "『能跑通全流程』的认知与现实不符；任何执行相关验收结论都建立在 dry-run 上。",
      recommendation: "声明单一执行契约属主：短期修复 C-02 恒等映射；中期让 bridge 接受规范 URDF 命名并独占 EtherCAT 改名；runtime_full_stack 补上 bridge 启动。",
      risk: "高关注度低技术风险：修复本身小，但必须配套契约单测防止再断。"
    },
    {
      id: "C-02", category: "code-defect", severity: "critical", verified: "CONFIRMED",
      title: "改名回归：joint 名映射退化为恒等函数，L6/R8 任务轨迹 12 个臂关节静默冻结",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py"],
      evidence: "execute_l6_r8_mock_live.py:92-99：moveit_to_execution_name 只匹配 left_joint*/right_joint* 前缀并原样返回（恒等）；当前模型命名是 leftjointN（无下划线，URDF:153、SRDF:42），所以所有臂关节返回 None 被丢弃。:117-126 回退到上一位置——生成 N 个相同点的『轨迹』。2026-07-06 MOTION-59 改名（commit 354607c）引入。",
      problem: "安全关键的名字翻译在 Python 里重复实现了 C++ ExecutionTrajectoryAdapter 已有的契约，副本在改名时腐烂，且能通过 bridge 校验（合法 goal，只是不动）。",
      consequence: "mock 与实机的 L6/R8 全流程自 07-06 起任务段零臂动作，无任何报错。",
      recommendation: "立即修复映射（leftjointN ↔ left_jointN）+ 单测断言映射非恒等且覆盖 12 关节；随后删除 Python 副本，从执行桥导入单一契约。",
      risk: "极低：一处函数修复 + 测试。P0 批次第一项。"
    },
    {
      id: "C-03", category: "duplicated-implementation", severity: "high", verified: "CONFIRMED",
      title: "13 关节执行契约在 3 个包、2 种语言、至少 4 处独立硬编码",
      packages: ["ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge/joints.py", "ros2_ws/src/alfa_robot_moveit_config/src/execution_trajectory_adapter.cpp"],
      evidence: "joints.py:18-48 自称 single source of truth；execution_trajectory_adapter.cpp:17-57 用 C++ 重写同一列表与映射；execute_l6_r8_mock_live.py:92-107 第三份（已腐烂，见 C-02）；scripts/safety 的 AST 检查是第四份的只读镜像。",
      problem: "Python 模块无法充当 C++ 消费者的事实源，契约天然分叉，而分叉已经造成一次生产级断裂。",
      consequence: "未来任何关节增减（LOG 里已预告 updown 接入执行）要在 2 种语言 4+ 处同步修改。",
      recommendation: "契约上移到 robot_motion_interfaces：单份 YAML + 生成/加载的 C++ 头与 Python 模块，配对称性单测。",
      risk: "中：涉及三个包的小改动，但正是防止 C-02 复发的根治。"
    },
    {
      id: "C-04", category: "hidden-coupling", severity: "high", verified: "CONFIRMED",
      title: "服务/action 名与 runtime 状态协议只以复制的字符串存在",
      packages: ["ros2_ws/src/alfa_robot_moveit_config", "ros2_ws/src/robot_motion_runtime"],
      evidence: "/robot_motion/solve_arm_ik 等名字在 ≥4 处重复（C++ 节点、两个包的 launch、Python 客户端）；/alfa_execution/execute_joint_trajectory 同样；/robot_motion/runtime_status 是 JSON-in-String 协议，由 C++ ostringstream 与 Python dict 各自手写一份 schema。",
      problem: "节点图靠字符串巧合联通；schema 只隐式存在于两份实现里。",
      consequence: "单方面改名导致服务静默失联（客户端永远等待）；两份 JSON 实现漂移会打断 dashboard。",
      recommendation: "规范名集中到 robot_motion_interfaces（names.hpp/names.py），runtime_status 升级为 RuntimeStatus.msg。",
      risk: "低。"
    },
    {
      id: "C-05", category: "hidden-coupling", severity: "high", verified: "CONFIRMED",
      title: "关节/链路名散布在『通用』运动库里，且 silent-skip 把改名从硬错误变成静默错误",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/src/extract_monitor_state.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/extract_planning_pipeline.cpp"],
      evidence: "extract_monitor_state.cpp:112-127 以字符串拼接 leftjoint+i 构造状态并用 has_variable() 静默跳过缺名；extract_planning_pipeline 硬编码 updown/base links/left_tool0 等；analytic_arm_ik_service_node.cpp:79-87 同时容忍三套别名（含已死的 left_v5_joint*）。",
      problem: "模型改名可以编译通过并『正常运行』，但 IK 种子悄悄归零、touch links 悄悄失配。",
      consequence: "MOTION-59 这类模型迁移会静默降级碰撞检查与规划质量，事后极难归因。",
      recommendation: "名字全部走 SRDF 组/参数注入；缺名一律 fail loudly；删除 v5 死别名。",
      risk: "低-中：改成硬失败可能暴露既有隐性错配，正是目的。"
    },
    {
      id: "C-06", category: "duplicated-implementation", severity: "high", verified: "CONFIRMED",
      title: "箱墙几何与携带箱语义 C++/Python 双实现，碰撞语义已经漂移",
      packages: ["ros2_ws/src/robot_motion_scene_service/src/motion_core/task_geometry.cpp", "ros2_ws/src/robot_motion_runtime/robot_motion_runtime/box_pair_task_adapter_node.py"],
      evidence: "box_pair_task_adapter_node.py:50-64 是 task_geometry.cpp:16-34 的逐行 Python 移植（同一 5×5 网格、同一 z=0.2+0.4*row）；attached box 生成也复制了（相同尺寸常量），但 Python 侧 touch_links 为空——MoveIt 只豁免 attach link，携带箱与 leftjoint4/5/6 的接触会被误报为碰撞。",
      problem: "scene_service README 声明『真实实现只保留在本包内』，而新主线绕开它重新硬编码。",
      consequence: "改一处箱墙尺寸另一处静默失真：机器人抓错格或误报/漏报碰撞。",
      recommendation: "几何单源化：runtime 通过 SetRobotMotionScene/扩展的接口拿姿态而不是公式；或以共享 YAML + 双语言黄金值测试锁定。",
      risk: "中。"
    },
    {
      id: "C-07", category: "hidden-coupling", severity: "high", verified: "PARTIAL",
      title: "安装的执行器在运行时依赖源码树形状：sys.path 注入 + importlib 加载仓库根脚本 + 硬编码 install 布局",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py"],
      evidence: ":48-56 sys.path.insert 执行桥『源码』目录并导入 joints；:237-245 importlib 硬加载 REPO_ROOT/scripts/ik_benchmark/scripts/visualize_rerun.py（缺失即抛错）；:465 以 {ROS_WS}/install/... 字面路径取 params 文件（merge-install 即断）。复核修正：bridge 导入在纯 install 环境会回落到已安装包成功，真正硬断点是 visualize_rerun 与 params 路径。",
      problem: "运行行为取决于 checkout 而不是构建产物；源码与安装版本可静默分叉。",
      consequence: "工控机上无 src/ 的部署直接崩；有 src/ 的部署可能执行陈旧的方向表。",
      recommendation: "get_package_share_directory 解析 params；rerun 可视化改为已安装模块的可选依赖；声明 exec_depend。",
      risk: "低。"
    },

    // ===== D. 调试/试车代码泄漏 =====
    {
      id: "D-01", category: "code-defect", severity: "critical", verified: "CONFIRMED",
      title: "被安装、被 launch 引用的 mujoco_digital_twin.py 从未进入 git——干净克隆无法构建核心包",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/CMakeLists.txt", ".gitignore"],
      evidence: "CMakeLists.txt:346 install(PROGRAMS scripts/mujoco_digital_twin.py)；launch/mujoco_digital_twin.launch.py:42 引用之；文件不在工作树也不在任何 git 历史（唯一踪迹是已删除的 ros2_tmp 树，commit cd82083）。根因：.gitignore:188 整目录忽略 scripts/* + 手工白名单漏了它。",
      problem: "版本控制与构建系统对『什么是生产代码』意见相左；install(PROGRAMS) 缺文件使 CMake install 步骤直接失败。",
      consequence: "任何新 checkout / CI 无法 colcon build 运控核心包；这个状态能存活本身证明没有干净构建验证。",
      recommendation: "从 cd82083 恢复该脚本（git show 可取回）或删除 install 规则与 launch；替换整目录 ignore+白名单模式；加最小 CI（见 H-01）。",
      risk: "极低。P0 第一批。"
    },
    {
      id: "D-02", category: "debug-leak", severity: "high", verified: "CONFIRMED",
      title: "开发机绝对路径 /mnt/mydisk/ALFA/... 是 C++ 参数默认值与主 launch 默认值",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp", "ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py"],
      evidence: "dual_arm_planner_node.cpp:299-304、409-411 三个记录/快照/CSV 路径默认指向 /mnt/mydisk/...；launch:48,50,152-155 复制同样字面量；run_extract_live_benchmark.py:34 直接 REPO_ROOT=Path('/mnt/mydisk/ALFA/alfa_robot')。复核细化：无 /mnt/mydisk 的机器上 motion_flow_recorder 的 create_directories 抛 filesystem_error，被 main 的 catch-all 接住后 FATAL 退出——节点默认参数下根本起不来。",
      problem: "一台开发机的磁盘布局被编译进生产节点默认值。",
      consequence: "新机器默认启动即挂；试车数据管道（快照→Rerun 回放）在任何其他主机上静默断裂。",
      recommendation: "默认空字符串=禁用记录，或统一 data_root 参数/ALFA_ROBOT_ROOT 环境变量派生；launch 不再复制默认值。",
      risk: "低。"
    },
    {
      id: "D-03", category: "debug-leak", severity: "high", verified: "CONFIRMED",
      title: "实机执行入口是 benchmark 脚本的 5 行 fork，住在被 .gitignore 声明为临时的目录",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_real_live.py"],
      evidence: "execute_l6_r8_real_live.py 全部内容是 from execute_l6_r8_mock_live import main; main('real')；mock 脚本含交互 input() 确认门（:681,697）、环境变量安全豁免 ALFA_ALLOW_UNSAFE_DIRECTION_OVERRIDE（:566-575）、硬编码实机控制器 action 名（:578-583）；docs/ethercat/REAL_DIRECTION_SAFETY.md 把它钦定为唯一实机入口。",
      problem: "带 Rerun 仪表的一次性 benchmark 长成了生产执行器：安全联锁、方向表、重采样全部活在 CI 不跑的操作脚本里。",
      consequence: "C-02 就是代价的具象：实机路径断了而没有任何测试变红。",
      recommendation: "执行路径提升为 robot_motion_runtime / execution_bridge 的正式节点+库（可测试），Rerun 观察改可选；操作入口保留薄 CLI。",
      risk: "中：涉及实机安全语义搬迁，需在 P5 配合试车窗口。"
    },
    {
      id: "D-04", category: "debug-leak", severity: "high", verified: "CONFIRMED",
      title: "试车监控/快照/回放与 benchmark CSV 机器被编译进生产规划节点",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/src/extract_monitor_json.cpp", "ros2_ws/src/alfa_robot_moveit_config/src/extract_planning_pipeline.cpp"],
      evidence: "extract monitor 每阶段向磁盘写带操作员中文标签的 JSON 快照（dual_arm_planner_node.cpp:3477-3498）；ExtractBenchmarkRunner/CsvWriter（extract_planning_pipeline.hpp:490-594）内嵌于生产库；~40 个 extract_benchmark_*/record_* 参数暴露在生产节点参数面上；snapshot_path 可经服务调用写任意路径。",
      problem: "调试数据采集与真机规划执行焊死在同一节点，没有构建开关或工具边界。",
      consequence: "每次生产运行都付快照 I/O；错误指向的 snapshot_path 可写节点用户可及的任何位置。",
      recommendation: "监控+benchmark 层拆出为独立库/工具（或 commissioning 参数默认关）；生产库只留 ranked-candidate API。",
      risk: "中：P3 拆包时一并处理最省。"
    },
    {
      id: "D-05", category: "code-defect", severity: "medium", verified: "",
      title: "process_lifecycle.py 会 pkill 主机上任何匹配 move_group 的进程",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/scripts/process_lifecycle.py"],
      evidence: "process_lifecycle.py:29-33,189-200 机器级 pkill 模式匹配；被已安装的 extract_stage_monitor_console.py:161 等调用。",
      problem: "『清理陈旧进程』工具的杀伤范围是整机而不是自己启动的进程组。",
      consequence: "同主机上其他工程师/其他机器人栈的 move_group 被无差别击杀。",
      recommendation: "改为跟踪自己 spawn 的 PID/进程组；工具迁入 tools 包。",
      risk: "低。"
    },
    {
      id: "D-06", category: "debug-leak", severity: "high", verified: "CONFIRMED",
      title: "box_perception 在 YOLO 权重缺失时静默生成随机假箱，而权重不在仓库里",
      packages: ["ros2_ws/src/box_perception/box_perception/segmentor.py"],
      evidence: "segmentor.py:28-35 模型加载失败仅 warnings.warn 后回落 stub；:83-116 _infer_stub 用 RandomState 生成 1-3 个随机 mask、置信度 0.5-0.99；config/params.yaml 指向的 models/box_seg.pt 不存在于仓库。",
      problem: "测试桩接线在生产推理路径上且无 ROS 可见的失败信号；新装机保证触发。",
      consequence: "按文档跑通的感知→auto_grasp 管线可能对随机幻觉箱体发起真实运动。",
      recommendation: "加载失败即 abort；stub 移到显式 use_stub_segmentor 参数并以 ERROR 级公告；权重供给写进部署文档。",
      risk: "低。"
    },

    // ===== E. 运行时代码缺陷 =====
    {
      id: "E-01", category: "code-defect", severity: "high", verified: "CONFIRMED",
      title: "ExecuteTrajectory 门面静默丢弃 velocity_scale/acceleration_scale，上游发固定 1 秒轨迹",
      packages: ["ros2_ws/src/robot_motion_runtime/robot_motion_runtime/execute_trajectory_service_node.py"],
      evidence: "srv 声明并由 orchestrator 填充两个缩放字段（motion_task_orchestrator_node.py:211-212）；execute_trajectory_service_node.py:47-97 从不读取它们；common.py 的插值轨迹时长固定 1s，与运动幅度无关。",
      problem: "声明了的安全相关契约字段被接受后丢弃。",
      consequence: "命名修复（C-01）落地后，第一次真实执行将以『钦定 1 秒』的全速轨迹运动——暴力动作/驱动器报警。",
      recommendation: "实现时间重缩放，或在实现前显式拒绝 scale != 1.0 的请求（fail loudly）；轨迹时长按关节限速计算。",
      risk: "低，但必须在打通执行链之前完成——列入 P0/P2 的执行链联锁。"
    },
    {
      id: "E-02", category: "code-defect", severity: "high", verified: "PARTIAL",
      title: "嵌套同步服务链超时预算倒挂且无取消：超时报错后编排器仍可能执行轨迹",
      packages: ["ros2_ws/src/robot_motion_runtime/robot_motion_runtime/box_pair_task_adapter_node.py", "ros2_ws/src/robot_motion_runtime/robot_motion_runtime/motion_task_orchestrator_node.py"],
      evidence: "adapter 外层超时 10s（:214,258-277），内层 orchestrator 最多 4×10s 顺序调用 + ExecuteTrajectory，且超时后无取消路径；orchestrator 在 PlanLoaded 后无条件继续执行（:206-215）。复核细化：当前因 C-01 命名断裂 goal 会被拒，物理双动作需命名修好后才可能，但重复 ExecuteTrajectory 调用今天即可复现。",
      problem: "超时预算不分层、被放弃的请求继续运行，阻塞式 call 模式复制进 5 个节点。",
      consequence: "操作员看到 TimeoutError 重试，第一条链在后台继续走到执行——修好 C-01 后即是双重运动隐患。",
      recommendation: "外层预算 ≥ 内层总和或在 MotionContext 传递 deadline；ExecuteTrajectory 前检查 deadline；call helper 上移 common.py 单份实现。",
      risk: "低。"
    },
    {
      id: "E-03", category: "code-defect", severity: "high", verified: "",
      title: "planner 节点在互斥回调组内阻塞等待 action 结果，alfa_execution_bridge 后端会 5s 超时假死",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp"],
      evidence: "dual_arm_planner_node.cpp:515-516 action client 与 :603-663 的 Trigger 服务共用节点默认互斥回调组；execute_with_alfa_execution_bridge(:3183) 在服务回调内轮询 future(:3234-3250)——goal-response 回调无法进入。",
      problem: "同一互斥组内的自阻塞：经典 sync-over-async 死锁模式。",
      consequence: "以 alfa_execution_bridge 为后端调用 plan_and_execute 时每阶段烧满 execution_action_wait_timeout_s 后报『failed to send goal』，即使桥完全健康。",
      recommendation: "action client 与长服务放入 Reentrant/独立回调组，或执行改异步状态机；补一条驱动真实后端的集成测试。",
      risk: "低。（本条在复核容量上限外，标注待复核。）"
    },
    {
      id: "E-04", category: "code-defect", severity: "high", verified: "CONFIRMED",
      title: "ros2_control 宏把 leftjoint1/rightjoint1 命令区间钳到 [-0.0, 0.0]，与 URDF/MoveIt ±2.356 矛盾",
      packages: ["ros2_ws/src/alfa_robot_description/urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro"],
      evidence: "宏 :113 与 :119 对两关节写 lower=-0.0 upper=0.0，而 URDF 与 joint_limits.yaml 均为 ±2.35619449；该宏服务 mock/gazebo/real 全部模式；MOTION-59 迁移引入，无注释解释。",
      problem: "规划层认为可动 ±135°，控制层命令接口写死 0——两层各说各话。",
      consequence: "MoveIt 可以规划穿过 joint1 的轨迹，控制器/硬件层要么钳制要么拒绝——静默的轨迹失真。",
      recommendation: "运行时验证一次实际行为，然后要么恢复 ±2.356，要么把『joint1 禁用』写成显式注释并同步 SRDF/joint_limits，让两层一致。",
      risk: "低-中：需要先弄清是笔误还是有意锁轴。"
    },
    {
      id: "E-05", category: "code-defect", severity: "high", verified: "CONFIRMED",
      title: "已安装的调试工具 extract_failed_attempts_rerun.py 每次调用必 NameError",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/scripts/extract_failed_attempts_rerun.py"],
      evidence: ":204 使用 process_lifecycle.configure_ros_domain 但全文件没有 import process_lifecycle（姊妹脚本都有）；经 CMakeLists.txt:355 安装。",
      problem: "死于到达的代码被当作产品安装，证明安装脚本没有任何冒烟导入检查。",
      consequence: "『回放全部失败抽离候选』的调试工作流不可用，操作员按文档必撞 traceback。",
      recommendation: "补一行 import；CI 对每个安装脚本做 byte-compile/import 冒烟（H-01 的一部分）。",
      risk: "极低。P0。"
    },

    // ===== F. 硬件层（注意：当前无活 launch 以实机模式到达 AlfaRobotHW，以下均为潜伏问题） =====
    {
      id: "F-01", category: "confirmed-architectural-problem", severity: "critical", verified: "CONFIRMED",
      title: "硬件插件与 URDF 描述的是两台不同的机器（潜伏：当前无活入口到达实机模式）",
      packages: ["ros2_ws/src/alfa_robot_hardware/src/alfa_robot_hardware.cpp", "ros2_ws/src/alfa_robot_description/urdf"],
      evidence: "buildJoints()（alfa_robot_hardware.cpp:332-385）硬编码构造 turn/leftjoint2-5/rightjoint2-4/updown/leftjoint1/rightjoint1/plate，从不读取 info_.joints；URDF 声明 15 关节含 pitch、leftjoint6、rightjoint5/6，且没有 plate。critic 专项核实：use_mock_hardware:=false 只出现在 bringup 的死 launch 里——该插件当前从任何活路径不可达，真正的实机执行走仓库外的 EtherCAT 工控机栈。",
      problem: "『模型单一事实源』广告着一个绑定不同物理机器的实机模式；四份关节副本中唯一编译进二进制的一份漂移最远。",
      consequence: "任何人复活 bringup 实机路径都会撞上控制器激活失败 + 幽灵 plate 关节；同时仓库对真正在跑实机的 EtherCAT 栈零覆盖。",
      recommendation: "短期把 real_hardware_plugin 默认改为显式哨兵防误触；中期 buildJoints 改为 info_.joints 数据驱动 + 每关节 hardware_parameters；把外部 EtherCAT 栈的契约（关节序/方向）以文档+校验形式钉进仓库。",
      risk: "高（涉及实机），但当前紧迫度因不可达而降级——列入 P6 实机窗口。"
    },
    {
      id: "F-02", category: "confirmed-architectural-problem", severity: "high", verified: "CONFIRMED",
      title: "read()/write() 全程实时违规：spin_some、100ms 阻塞总线 I/O、usleep、堆分配（潜伏）",
      packages: ["ros2_ws/src/alfa_robot_hardware/src/alfa_robot_hardware.cpp", "ros2_ws/src/alfa_robot_hardware/src/driver"],
      evidence: "read() 开头 rclcpp::spin_some(estop_node_)（:247-250）；ZeroerrDriver::readPositions 每节点 sendCommandAndWait 默认 100ms 超时 + usleep(200)；Cylinder 10ms 轮询；控制路径上有 dynamic_cast 与日志。",
      problem: "200Hz 控制环的周期无上界；一个失联节点可拖住全机所有总线。",
      consequence: "上实机后表现为全局抖动与不可控周期，且难以归因。",
      recommendation: "总线 I/O 移到每总线工作线程 + 双缓冲交换；estop 用独立 executor 线程；P6 与 F-01 同窗处理。",
      risk: "高（实机验证成本），当前潜伏。"
    },
    {
      id: "F-03", category: "code-defect", severity: "high", verified: "CONFIRMED",
      title: "急停子系统竞态且半接线：无锁并发访问驱动、导出的 estop 状态恒为 0、清除不重新使能",
      packages: ["ros2_ws/src/alfa_robot_hardware/src/alfa_robot_hardware.cpp"],
      evidence: "CAN 监视线程（:497-534）与 RT 环并发调用同一驱动的 socket 读写与容器（全包 grep 无一处 std::mutex）；emergency_stop state interface 从未被写入；clearEmergencyStop 不重新使能电机。",
      problem: "安全特性以未定义行为的方式挂在 RT 插件上，可观测状态是谎言，恢复语义未定义。",
      consequence: "急停恰好在最需要可靠性时可能损坏驱动状态；上层看到的急停状态永远『正常』。",
      recommendation: "监视线程只置原子标志；实际停机在 write() 内单线程执行；补状态写入与恢复流程定义。",
      risk: "中（代码改动小），验证需实机窗口。"
    },
    {
      id: "F-04", category: "code-defect", severity: "high", verified: "CONFIRMED",
      title: "ZeroerrJoint 方向符号只用于 write 不用于 read——命令与状态互为镜像",
      packages: ["ros2_ws/src/alfa_robot_hardware/src/joint/zeroerr_joint.cpp"],
      evidence: "read()（:58）不乘 config_.sign；write()（:80-81）乘 sign=-1（leftjoint2/3 配置为 -1）。命令 +0.3 rad，反馈回来 -0.3 rad。",
      problem: "read 不是 write 的逆变换。",
      consequence: "JTC 的目标/路径容差在这两个关节上必然报警中止；/joint_states 显示镜像运动。",
      recommendation: "read 应用相同逆变换（先在实机上确认哪个物理方向为正）。",
      risk: "低改动、实机验证。"
    },
    {
      id: "F-05", category: "responsibility-violation", severity: "high", verified: "CONFIRMED",
      title: "activate 即无命令运动：cylinder 自动归零、ZeroErr 驱向硬编码编码器零点",
      packages: ["ros2_ws/src/alfa_robot_hardware/src/joint/cylinder_joint.cpp", "ros2_ws/src/alfa_robot_hardware/src/joint/zeroerr_joint.cpp"],
      evidence: "CylinderJoint::activate 直接 position_command_=0.0 且 write 无 first-read 保护；ZeroerrJoint 初始计数取自配置常量 262144 而非实读。其他关节类型有首帧保护，这两类没有。",
      problem: "归零是试车/启动策略，不是 hardware_interface 的职责，且实现不一致。",
      consequence: "在任意姿态激活插件，左臂 2/3 关节和升降轴会自行运动——物理碰撞隐患。",
      recommendation: "补首帧命令初始化保护；homing 改为显式服务触发操作。",
      risk: "中：涉及实机安全语义。"
    },
    {
      id: "F-06", category: "code-defect", severity: "high", verified: "",
      title: "安全停机等待环读的是冻结缓存，永远烧满 5 秒超时",
      packages: ["ros2_ws/src/alfa_robot_hardware/src/joint/rmd_joint.cpp"],
      evidence: "moveToSafePosition 循环 read(kDt)/write(kDt)，但 joint 级 read 只取驱动缓存，缓存只在总线批量读时更新——循环从不触发总线读，收敛判断对着陈旧值。",
      problem: "关节层重构后 moveToSafePosition 仍假设 read() 做硬件 I/O。",
      consequence: "use_safe_shutdown=true（xacro 默认）下每次停机阻塞 5s/关节并报 all_ok=false。",
      recommendation: "循环内刷新驱动缓存，或把安全停机时序上移到已有批量读的 AlfaRobotHW 层。（复核容量上限外，标注待复核。）",
      risk: "低。"
    },

    // ===== G. 感知与周边 =====
    {
      id: "G-01", category: "duplicated-implementation", severity: "high", verified: "",
      title: "auto_grasp 双拷贝携带两套不同的手眼标定外参（z 平移差 9cm）",
      packages: ["ros2_ws/src/alfa_robot_bringup/scripts/auto_grasp_node.py", "scripts/auto_grasp.py"],
      evidence: "两份几乎相同的实现（均注明『移植自 trans.py』）：bringup 版 T_RADAR_TO_BASE z=0.817，仓库根版 z=0.727；均以 numpy 字面量硬编码雷达→基座外参并 subprocess 启动 launch。",
      problem: "物理标定结果存在两份且数值不同，没有任何标记指明哪份是现行的。",
      consequence: "9cm 的抓取目标分歧在实机上是抓到与撞上的差别；跑哪份全凭运气。（复核容量上限外，标注待复核。）",
      recommendation: "只保留一份（感知/应用包，不在 bringup）；外参进标定 YAML 或以静态 TF 发布；删除另一份。",
      risk: "低改动；需确认现行外参真值。"
    },
    {
      id: "G-02", category: "hidden-coupling", severity: "medium", verified: "",
      title: "box_perception 发布错误 frame_id 并内联手眼矩阵而非走 TF",
      packages: ["ros2_ws/src/box_perception/box_perception/perception_node.py"],
      evidence: "检测结果 header 的 frame_id 与实际相机/雷达系不符；雷达→基座变换以内联矩阵存在而不是 TF 树。",
      problem: "感知→运控坐标语义靠硬编码矩阵而不是系统级 TF。",
      consequence: "任何外参更新要改代码；下游按 header 变换会得到错的位姿。",
      recommendation: "修正 frame_id，外参走 static TF/标定文件。",
      risk: "低。"
    },
    {
      id: "G-03", category: "improper-dependency", severity: "medium", verified: "",
      title: "bio_ik 以 vendored 形式每次全量参与编译，而生产已不加载它",
      packages: ["ros2_ws/src/bio_ik"],
      evidence: "kinematics.yaml 为空（解析 IK 走独立服务）；bio_ik 仍作为工作区包每次编译。",
      problem: "保留对照用途合理，但不该在默认构建路径上。",
      consequence: "全量构建时间与依赖面无谓增加。",
      recommendation: "迁入 dependency.repos 或 COLCON_IGNORE，按需拉取。",
      risk: "极低。"
    },

    // ===== H. 流程 / CI / 文档 =====
    {
      id: "H-01", category: "process-gap", severity: "high", verified: "",
      title: "整个仓库没有 CI：所有守门（安全 ctest、契约 pytest、xacro 测试）只在开发机手工跑",
      packages: [".github", "ros2_ws"],
      evidence: "critic 专项核实：仓库根无 .github/workflows、无 gitlab-ci、无 Jenkinsfile；唯一的 workflow 是 description 包内死的 sphinx 检查（GitHub 只识别仓库根 workflows）。",
      problem: "D-01（构建断裂）与 C-02（执行链断裂）都属于最小 CI 一次构建/一个单测就能拦下的回归。",
      consequence: "任何守护声明都是善意谎言；每次合入的验证质量取决于个人。",
      recommendation: "最小仓库根 CI：colcon build（干净容器）+ execution_bridge 契约测试 + 安全 ctest；删除/搬迁死 sphinx workflow。",
      risk: "极低。P0。"
    },
    {
      id: "H-02", category: "process-gap", severity: "high", verified: "",
      title: "安全门 check_l6_r8_real_safety.py 恰好没有钉住 joint 名映射——正是 C-02 穿过的洞",
      packages: ["scripts/safety/check_l6_r8_real_safety.py"],
      evidence: ":71-135 以 AST 钉住方向符号表、负重姿态族、导入与 CLI 旗标、planner 默认值——唯独不覆盖 moveit_to_execution_name / execution_to_moveit_name。MOTION-59 改名顺利通过安全门。",
      problem: "守门内容与真实风险面错位。",
      consequence: "给了『有安全门』的错误安心感。",
      recommendation: "把文本钉扎升级为可执行单测：断言映射对 SRDF 全部臂关节双射且非恒等；随 C-03 契约上移一并迁移。",
      risk: "极低。P0。"
    },
    {
      id: "H-03", category: "duplicated-implementation", severity: "medium", verified: "",
      title: "AI 协作的 ground-truth 控制层文档描述的是已删除的 v2 架构",
      packages: ["docs/REFACTOR_ARCHITECTURE_NOTES.md", "docs/CONTROL_LAYER_HARDCODED_PARAMS.md"],
      evidence: "两份文档仍在讲 CanBus/RMD/CANopen、3:1 armbase 补偿、traj_log 服务、四轮 velocity 关节与 /home/kzoia 路径——这些在当前代码里都已不存在；NOW.md 把它们列为追溯入口。",
      problem: "新开工的 AI/工程师按入口文档建立的是上一代机器的心智模型。",
      consequence: "排障与重构决策建立在过时事实上。",
      recommendation: "按本次评审结论重写两份文档（或显式标注『v2 历史』移入 archive）。",
      risk: "无。"
    },
    {
      id: "H-04", category: "process-gap", severity: "medium", verified: "",
      title: "部署清单 dependency.repos 缺了评审发现的大半真实依赖；雷达两包本地 fork 无人评审",
      packages: ["ros2_ws/src/dependency.repos", "ros2_ws/src/fast_lio", "ros2_ws/src/livox_ros_driver2"],
      evidence: "critic 核实：清单缺 Livox-SDK2、rerun-sdk、mujoco、YOLO 权重、ik_benchmark；fast_lio 携带手改的 'mid360 copy.yaml'，livox 配置硬编码机器人网络 IP——149 个文件的两个 vendored 包在本次评审覆盖之外。",
      problem: "『新机器怎么跑起来』没有可信答案。",
      consequence: "新部署靠考古；雷达栈的本地魔改随升级丢失。",
      recommendation: "以本次新机审读结论重生成 dependency.repos + bootstrap 文档；雷达两包补一次对上游 diff 审读，机器 IP 外提为站点配置。",
      risk: "低。"
    },

    // ===== R. 合理妥协（正面确认，防止误改） =====
    {
      id: "R-01", category: "reasonable-compromise", severity: "low", verified: "",
      title: "空 kinematics.yaml、thin-wrapper URDF、mock 控制器容差覆盖是有意为之，应保留",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/config"],
      evidence: "kinematics.yaml 为空是因为解析 IK 走独立服务（by design）；包内 alfa_robot.urdf.xacro 只是 include description 的薄包装（URDF 分叉已在早期治愈）。",
      problem: "无。列出以免后续重构误判为缺陷。",
      consequence: "拆包时这三件+SRDF/joint_limits/ompl/pilz 就是 moveit_config 的全部合法遗产。",
      recommendation: "保持现状。",
      risk: "无。"
    },
    {
      id: "R-02", category: "reasonable-compromise", severity: "low", verified: "",
      title: "execution_bridge 本身是设计正确的接缝；ROS_DOMAIN_ID 隔离与方向覆盖环境门是合理的临时防御",
      packages: ["ros2_ws/src/alfa_robot_execution_bridge"],
      evidence: "包边界、action 契约、mock/ros2_control 双后端的结构是对的；问题在细节（C-01 命名、静默丢轴与 README 承诺相反、goal 发送无超时）而非包本身。",
      problem: "无。",
      consequence: "它应当成为执行边界的终局属主，修边界缺陷而不是绕开它。",
      recommendation: "保留并强化：把拒收未映射关节的校验从客户端移进桥内。",
      risk: "无。"
    },
    {
      id: "R-03", category: "reasonable-compromise", severity: "low", verified: "",
      title: "motion_core 前向兼容 shim 头是未完成迁移的正确中间态——应当完成而不是回退",
      packages: ["ros2_ws/src/alfa_robot_moveit_config/include/alfa_robot_moveit_config/motion_core"],
      evidence: "scene_geometry/task_geometry 已迁至 robot_motion_scene_service，配置包留转发头兼容 ~5 处旧 include；pose_math 尚滞留。",
      problem: "半程状态本身不是错，是方向正确的证据。",
      consequence: "P3 拆包时顺手完成：迁 pose_math、改余下 include、删转发头。",
      recommendation: "按既定方向收尾。",
      risk: "低。"
    }
  ],

  currentGraph: {
    width: 980,
    height: 640,
    caption: "现状：三条活入口全部汇聚到配置包；红色虚线为倒置/逃逸依赖。复核确认无循环依赖——是单向倒置，可增量解开。",
    legend: [
      { label: "正常依赖", tone: "blue" },
      { label: "倒置/逃逸依赖", tone: "red" },
      { label: "漂移中的耦合", tone: "orange" },
      { label: "休眠/不可达", tone: "gray" }
    ],
    nodes: [
      { id: "op1", label: "入口A: runtime_full_stack", sub: "v5 服务图主线", x: 30, y: 20, w: 220, h: 52, tone: "ok" },
      { id: "op2", label: "入口B: dual_arm_planner.launch", sub: "规划/回放（自带控制栈）", x: 380, y: 20, w: 240, h: 52, tone: "warn" },
      { id: "op3", label: "入口C: execute_l6_r8_*_live", sub: "唯一实机脚本链（已断）", x: 720, y: 20, w: 230, h: 52, tone: "bad" },
      { id: "runtime", label: "robot_motion_runtime", sub: "9 个 rclpy 服务节点", x: 30, y: 130, w: 220, h: 52, tone: "ok" },
      { id: "moveit", label: "alfa_robot_moveit_config", sub: "上帝包: 15.8k行C++ + 24脚本 + 配置 + 启动", x: 330, y: 150, w: 340, h: 60, tone: "bad" },
      { id: "bridge", label: "alfa_robot_execution_bridge", sub: "执行边界 (left_jointN 契约)", x: 720, y: 150, w: 230, h: 52, tone: "warn" },
      { id: "scene", label: "robot_motion_scene_service", sub: "场景/碰撞 C++ 库", x: 90, y: 300, w: 230, h: 48, tone: "ok" },
      { id: "ifaces", label: "robot_motion_interfaces", sub: "服务/消息契约（叶）", x: 30, y: 400, w: 220, h: 48, tone: "good" },
      { id: "analytic", label: "alfa_robot_analytic_ik", sub: "解析 IK 库", x: 360, y: 300, w: 190, h: 48, tone: "ok" },
      { id: "desc", label: "alfa_robot_description", sub: "URDF/mesh 模型源", x: 590, y: 300, w: 200, h: 48, tone: "ok" },
      { id: "ikbench", label: "scripts/ik_benchmark", sub: "工作区外·被生产编译引用", x: 330, y: 420, w: 230, h: 48, tone: "bad" },
      { id: "sim", label: "simulation/mujoco", sub: "工作区外·parents[4] 爬取", x: 600, y: 420, w: 210, h: 48, tone: "bad" },
      { id: "bringup", label: "alfa_robot_bringup", sub: "bit-rot：引用已删除的可执行", x: 30, y: 540, w: 230, h: 48, tone: "dim" },
      { id: "hardware", label: "alfa_robot_hardware", sub: "描述另一台机器·无活入口", x: 330, y: 540, w: 240, h: 48, tone: "dim" },
      { id: "ethercat", label: "外部 EtherCAT 工控机栈", sub: "真实实机执行·仓库外零覆盖", x: 660, y: 540, w: 250, h: 48, tone: "dim" }
    ],
    edges: [
      { from: "op1", to: "runtime", kind: "ok" },
      { from: "op2", to: "moveit", kind: "ok" },
      { from: "op3", to: "moveit", kind: "warn", label: "subprocess 启动 planner 栈" },
      { from: "runtime", to: "moveit", kind: "bad", label: "exec_depend + launch include（倒置）" },
      { from: "runtime", to: "ifaces", kind: "ok" },
      { from: "runtime", to: "bridge", kind: "warn", label: "契约命名不匹配·goal 被拒" },
      { from: "moveit", to: "scene", kind: "ok" },
      { from: "moveit", to: "analytic", kind: "ok" },
      { from: "moveit", to: "desc", kind: "ok" },
      { from: "moveit", to: "ifaces", kind: "ok" },
      { from: "moveit", to: "ikbench", kind: "bad", label: "../../../ CMake 逃逸" },
      { from: "moveit", to: "sim", kind: "bad", label: "parents[4] 爬根" },
      { from: "moveit", to: "bridge", kind: "bad", label: "sys.path 注入源码树" },
      { from: "bringup", to: "moveit", kind: "warn", label: "controllers.yaml 默认取自配置包" },
      { from: "desc", to: "hardware", kind: "dim", label: "plugin 名引用" },
      { from: "bridge", to: "ethercat", kind: "dim", label: "转发（契约仅存在于口头）" }
    ]
  },

  targetGraph: {
    width: 980,
    height: 660,
    caption: "目标：依赖只允许自上而下。moveit_config 变回纯配置叶子；工具与仿真隔离在侧翼，只允许消费服务接口。",
    legend: [
      { label: "允许的依赖方向", tone: "blue" },
      { label: "新建/收编的包", tone: "green" },
      { label: "隔离区（不得被生产依赖）", tone: "purple" }
    ],
    nodes: [
      { id: "t-bringup", label: "alfa_robot_bringup", sub: "唯一启动装配层（L5）", x: 340, y: 20, w: 300, h: 52, tone: "good" },
      { id: "t-runtime", label: "robot_motion_runtime", sub: "服务图 + 任务编排（L4）", x: 100, y: 130, w: 240, h: 52, tone: "ok" },
      { id: "t-planning", label: "robot_motion_planning（新）", sub: "planner/extract/loaded/IK/碰撞服务", x: 400, y: 130, w: 280, h: 52, tone: "good" },
      { id: "t-bridge", label: "alfa_robot_execution_bridge", sub: "执行边界·独占命名翻译（L4）", x: 720, y: 130, w: 240, h: 52, tone: "ok" },
      { id: "t-moveit", label: "alfa_robot_moveit_config", sub: "纯配置：SRDF/YAML/move_group（叶）", x: 400, y: 250, w: 280, h: 52, tone: "good" },
      { id: "t-scene", label: "robot_motion_scene_service", sub: "场景/碰撞库+碰撞服务节点（L2）", x: 100, y: 360, w: 250, h: 52, tone: "ok" },
      { id: "t-ik", label: "alfa_robot_analytic_ik + ik_pipeline（收编）", sub: "IK 算法库（L2）", x: 400, y: 360, w: 280, h: 52, tone: "good" },
      { id: "t-hw", label: "alfa_robot_hardware", sub: "ros2_control 插件·数据驱动关节（L3）", x: 720, y: 360, w: 240, h: 52, tone: "ok" },
      { id: "t-desc", label: "alfa_robot_description", sub: "模型/限位/初始姿态唯一源（L1）", x: 100, y: 470, w: 250, h: 52, tone: "ok" },
      { id: "t-ifaces", label: "robot_motion_interfaces", sub: "消息/服务/命名契约唯一属主（L0）", x: 560, y: 470, w: 270, h: 52, tone: "good" },
      { id: "t-tools", label: "alfa_robot_motion_tools（新）", sub: "试车/监控/rerun/reachability", x: 60, y: 580, w: 260, h: 48, tone: "warn" },
      { id: "t-sim", label: "alfa_robot_mujoco_sim（新）", sub: "MuJoCo 桥+场景资产", x: 380, y: 580, w: 240, h: 48, tone: "warn" },
      { id: "t-bench", label: "alfa_robot_benchmarks（真包化）", sub: "benchmark/实验", x: 680, y: 580, w: 250, h: 48, tone: "warn" }
    ],
    edges: [
      { from: "t-bringup", to: "t-runtime", kind: "ok", label: "launch" },
      { from: "t-bringup", to: "t-planning", kind: "ok", label: "launch" },
      { from: "t-bringup", to: "t-bridge", kind: "ok", label: "launch" },
      { from: "t-runtime", to: "t-planning", kind: "ok", label: "service 调用（经接口）" },
      { from: "t-runtime", to: "t-ifaces", kind: "ok" },
      { from: "t-planning", to: "t-moveit", kind: "ok", label: "读配置" },
      { from: "t-planning", to: "t-scene", kind: "ok" },
      { from: "t-planning", to: "t-ik", kind: "ok" },
      { from: "t-planning", to: "t-ifaces", kind: "ok" },
      { from: "t-bridge", to: "t-ifaces", kind: "ok" },
      { from: "t-moveit", to: "t-desc", kind: "ok", label: "xacro" },
      { from: "t-scene", to: "t-ifaces", kind: "ok" },
      { from: "t-hw", to: "t-desc", kind: "ok", label: "hardware_parameters" },
      { from: "t-tools", to: "t-ifaces", kind: "dim", label: "只许依赖接口" },
      { from: "t-sim", to: "t-ifaces", kind: "dim", label: "只许依赖接口" },
      { from: "t-bench", to: "t-ik", kind: "dim", label: "find_package" }
    ]
  },

  godPackage: {
    name: "alfa_robot_moveit_config",
    statLine: "92 个源文件 / 约 24,000 行（其中 C++ 15,788 行）/ 17 个 launch / 24 个安装脚本 / 2 个导出库 / 3 个 rosidl 接口——逐项判定去向：",
    groups: [
      {
        title: "合法遗产：留下",
        verdict: "保留", tone: "green",
        note: "拆完之后配置包的全部内容——恰好就是 MoveIt Setup Assistant 能重新生成的东西。",
        items: ["config/alfa_robot.srdf", "config/joint_limits.yaml", "config/ompl_planning.yaml", "config/pilz_cartesian_limits.yaml", "config/moveit_controllers.yaml", "config/kinematics.yaml（有意为空）", "config/moveit.rviz", "8 个 MSA 标准 launch 短件（demo/move_group/rsp/...）"]
      },
      {
        title: "C++ 规划运行时",
        verdict: "→ robot_motion_planning", tone: "red",
        note: "完整的运动规划应用层——4,525 行的 mega-node 加两个导出库。",
        items: ["src/dual_arm_planner_node.cpp（4,525 行）", "src/extract_planning_pipeline.cpp（1,845 行）", "src/loaded_pose_planning.cpp", "src/optimized_ik_pipeline.cpp", "src/extract_monitor_*（试车监控随迁后再拆）", "src/box_stack_flow_orchestrator.cpp", "src/motion_core/ + motion_scene_adapter 库", "launch/dual_arm_planner.launch.py（去掉自带控制栈后）"]
      },
      {
        title: "/robot_motion 服务节点",
        verdict: "→ scene_service / IK 服务包", tone: "red",
        note: "runtime 对配置包 exec_depend 的唯一原因。",
        items: ["src/analytic_arm_ik_service_node.cpp（零 MoveIt 代码）", "src/motion_collision_service_node.cpp", "scripts/robot_motion_state_source.py（删：runtime 已有超集实现）", "launch/analytic_arm_ik_service.launch.py", "launch/motion_collision_service.launch.py"]
      },
      {
        title: "rosidl 接口",
        verdict: "→ robot_motion_interfaces", tone: "orange",
        note: "配置包不该同时是接口包。",
        items: ["msg/SemanticCargo.msg", "msg/SemanticScene.msg", "srv/ConfigureExtractMonitor.srv"]
      },
      {
        title: "MuJoCo 仿真桥",
        verdict: "→ alfa_robot_mujoco_sim", tone: "orange",
        note: "与 simulation/mujoco 资产合体为真正的仿真包，消灭 parents[4] 爬根。",
        items: ["scripts/mujoco_sync_bridge.py", "scripts/mujoco_planning_scene_bridge.py", "scripts/mujoco_joint_demo_commander.py", "scripts/semantic_scene_to_planning_scene.py + semantic_scene_utils.py", "scripts/mujoco_digital_twin.py（先从 git 历史抢救或删除）", "3 个 mujoco_*.launch.py", "config/mujoco_initial_positions.yaml"]
      },
      {
        title: "试车/调试/benchmark 工具",
        verdict: "→ alfa_robot_motion_tools", tone: "purple",
        note: "生产包不再 install 它们；执行逻辑本体上移 runtime/bridge，工具只留薄壳。",
        items: ["scripts/execute_l6_r8_mock_live.py / execute_l6_r8_real_live.py", "scripts/run_extract_live_benchmark.py", "scripts/extract_stage_monitor_console.py（1,280 行脚本兼库）", "scripts/extract_*_rerun.py / extract_startup_stability_smoke.py", "scripts/reachability_tester.py + nine_orient / x_edge 变体", "scripts/rviz_dual_goal_pose_monitor.py", "scripts/process_lifecycle.py（修 pkill 范围）", "scripts/motion_contracts/"]
      },
      {
        title: "死代码",
        verdict: "删除", tone: "gray",
        note: "git 历史可找回，不需要陪葬在生产包里。",
        items: ["config/alfa_robot.ros2_control.xacro（死的第二份 ros2_control）", "config/initial_positions.yaml（死副本）", "scripts/pick_place_demo.py / dual_arm_pose_planner.py / test_left_arm_full_planning.py（未安装的旧 demo）", "launch/robot_motion_state_source.launch.py（双份状态源）"]
      }
    ]
  },

  responsibilityMatrix: [
    { pkg: "alfa_robot_description", today: "URDF/mesh 模型源（但限位/初始姿态有旁支副本）", target: "模型、限位、初始姿态、关节命名的唯一事实源；ros2_control 参数全部 xacro 化", change: "强化", tone: "blue" },
    { pkg: "alfa_robot_moveit_config", today: "上帝包：配置+规划运行时+服务+仿真桥+试车工具+启动", target: "纯 MoveIt 配置叶子包，0 行 C++/Python", change: "大瘦身", tone: "red" },
    { pkg: "robot_motion_planning（新）", today: "——（散落在配置包内）", target: "规划应用层：planner 节点、extract/loaded 管线、IK pipeline", change: "新建", tone: "green" },
    { pkg: "robot_motion_runtime", today: "v5 服务图（依赖配置包、执行链断裂）", target: "服务图+编排；契约经 interfaces；不依赖任何配置包", change: "解耦", tone: "blue" },
    { pkg: "robot_motion_scene_service", today: "场景/碰撞 C++ 库（名不副实：没有 service）", target: "场景库 + motion_collision_service 节点（名副其实）", change: "补齐", tone: "green" },
    { pkg: "robot_motion_interfaces", today: "消息/服务契约（但命名契约散落各处）", target: "消息 + 服务名 + 关节命名契约（YAML→生成 C++/Python）唯一属主", change: "扩权", tone: "green" },
    { pkg: "alfa_robot_execution_bridge", today: "执行边界（契约与 runtime 断裂、被 sys.path 穿透）", target: "执行边界终局属主：独占 MoveIt↔执行命名翻译与方向表", change: "强化", tone: "blue" },
    { pkg: "alfa_robot_bringup", today: "bit-rot：死 launch、死 YAML、无人依赖", target: "唯一系统装配层：robot_system.launch.py（rsp+ros2_control+spawner+move_group）", change: "复活", tone: "orange" },
    { pkg: "alfa_robot_hardware", today: "描述另一代机器；RT 违规；无活入口（潜伏）", target: "数据驱动关节表的 ros2_control 插件，或按 EtherCAT 契约重写", change: "隔离→重修", tone: "orange" },
    { pkg: "alfa_robot_analytic_ik", today: "干净的 IK 库（但服务节点住在配置包）", target: "IK 库 + 收编 ik_benchmark 的生产求解器（ik_pipeline）", change: "扩容", tone: "blue" },
    { pkg: "alfa_robot_motion_tools（新）", today: "——（散落在配置包 scripts/）", target: "试车/监控/rerun/reachability 工具包，只依赖服务接口", change: "新建", tone: "green" },
    { pkg: "alfa_robot_mujoco_sim（新）", today: "——（桥在配置包，资产在仓库根）", target: "MuJoCo 桥 + 场景资产的完整仿真包", change: "新建", tone: "green" },
    { pkg: "alfa_robot_benchmarks", today: "指向 /mnt/mydisk 的悬空符号链接", target: "scripts/ik_benchmark 物理迁入后的真实 benchmark 包", change: "真包化", tone: "orange" },
    { pkg: "box_perception", today: "感知节点（stub 假检测静默回落、frame 语义错）", target: "感知包：加载失败 fail loudly，外参走 TF", change: "修边界", tone: "blue" },
    { pkg: "alfa_robot_rerun / twist_mux / bio_ik", today: "可视化工具 / 无主孤岛 / vendored 全量编译", target: "rerun 保持只读工具；twist_mux 认领归属；bio_ik 移出默认构建", change: "小整理", tone: "gray" }
  ],

  dependencyRules: [
    { rule: "依赖只能向下", detail: "装配(bringup) → 服务(runtime/planning/bridge) → 算法库(scene/ik) → 模型(description) → 契约(interfaces)。任何向上箭头都是架构回归，评审即拒。" },
    { rule: "moveit_config 是叶子", detail: "任何包不得为获取节点/脚本而依赖它；它自身保持 0 行可执行代码。需要它配置的包在启动期读取，不在构建期链接。" },
    { rule: "契约只住 interfaces", detail: "服务/action 名、关节命名映射、runtime 状态 schema 一律在 robot_motion_interfaces 定义（消息或生成的常量），任何第二份手写副本都要配等价性测试。" },
    { rule: "禁止逃逸工作区", detail: "ROS 包内禁止 ../../../、parents[n]、sys.path 注入其他包源码树、指向 checkout 布局的绝对/相对路径。资产进 share/，代码进包，路径走 ament 索引。" },
    { rule: "bringup 独占系统装配", detail: "ros2_control_node、controller spawner、rsp、move_group 的组合只出现在 bringup 的 launch；controller 拓扑 YAML 单一属主，其他包最多提供 overlay。" },
    { rule: "工具隔离于生产", detail: "试车/监控/benchmark/可视化住 tools/benchmarks/sim 包；生产包的 install 列表里不出现 demo/test/mock/monitor 名字的文件。" },
    { rule: "mock 必须显式", detail: "mock/仿真模式必须由启动方显式传参，节点在 mock 模式下打印无法忽视的公告；不允许三层默认叠加。" },
    { rule: "每个契约有守门", detail: "跨包字符串/数值契约（关节名、几何常量、方向表）必须有 CI 里跑的等价性/双射性测试；文本 AST 钉扎只作过渡。" }
  ],

  relocations: [
    { what: "mujoco_digital_twin.py 抢救或删除（含 install 规则+launch）", from: "git 历史 cd82083 / CMakeLists:346", to: "修复", tone: "red", reason: "干净克隆无法构建（D-01）", order: "P0" },
    { what: "moveit_to_execution_name 恒等回归修复 + 双射单测", from: "moveit_config/scripts/execute_l6_r8_mock_live.py:92-107", to: "修复", tone: "red", reason: "实机链静默不动臂（C-02）", order: "P0" },
    { what: "extract_failed_attempts_rerun.py 缺 import / velocity_scale 丢弃 / joint1 [0,0] 钳位判定", from: "各处（E-05/E-01/E-04）", to: "修复", tone: "red", reason: "已验证的具体缺陷", order: "P0" },
    { what: "最小 CI：干净 colcon build + 契约测试 + 安全 ctest", from: "（不存在）", to: "新建", tone: "green", reason: "D-01/C-02 类回归的唯一系统性拦截（H-01）", order: "P0" },
    { what: "scripts/ik_benchmark 全部内容", from: "仓库根（经悬空符号链接+../../../ 引用）", to: "ros2_ws 真包", tone: "orange", reason: "生产头文件的真实归属（A-04/A-05）", order: "P1" },
    { what: "/mnt/mydisk 与 /opt/ros 硬编码默认值", from: "dual_arm_planner_node.cpp、launch、3 个脚本", to: "data_root 参数", tone: "orange", reason: "新机器启动即挂（D-02）", order: "P1" },
    { what: "check_l6_r8_real_safety.py", from: "scripts/safety（经 ../../../ ctest）", to: "包内测试", tone: "orange", reason: "构建自包含 + 补 joint 映射钉扎（H-02）", order: "P1" },
    { what: "关节命名契约（YAML→C++/Python 生成）与服务名常量", from: "4+ 处硬编码（joints.py、adapter.cpp、脚本）", to: "robot_motion_interfaces", tone: "green", reason: "C-01/C-03/C-04 的根治", order: "P2" },
    { what: "SemanticCargo/SemanticScene/ConfigureExtractMonitor", from: "moveit_config/msg,srv", to: "robot_motion_interfaces", tone: "green", reason: "配置包不当接口包（A-06）", order: "P2" },
    { what: "箱墙/携带箱几何（含 touch_links 语义）", from: "C++/Python 双实现", to: "单源+黄金值测试", tone: "green", reason: "已漂移的物理事实（C-06）", order: "P2" },
    { what: "dual_arm_planner_node + motion 库 + extract/loaded 管线", from: "alfa_robot_moveit_config/src", to: "robot_motion_planning", tone: "purple", reason: "上帝包解体主刀（A-01）", order: "P3" },
    { what: "analytic_arm_ik_service_node / motion_collision_service_node", from: "alfa_robot_moveit_config/src", to: "IK 服务包 / scene_service", tone: "purple", reason: "解开 runtime→config 倒置（A-02）", order: "P3" },
    { what: "系统装配段（rsp+ros2_control+spawner+move_group）", from: "dual_arm_planner.launch.py:311-330", to: "bringup/robot_system.launch.py", tone: "blue", reason: "启动归位（A-03/B-02）；同时删 bringup 死物", order: "P4" },
    { what: "控制器拓扑 YAML（终结同名异义）", from: "bringup+moveit_config 六份", to: "单一属主+overlay", tone: "blue", reason: "B-01", order: "P4" },
    { what: "execute_l6_r8_*、extract 控制台、reachability、rerun 回放、process_lifecycle", from: "moveit_config/scripts", to: "alfa_robot_motion_tools", tone: "purple", reason: "试车工具出生产包（D-03/D-04/D-05）；执行逻辑本体上移 runtime/bridge", order: "P5" },
    { what: "MuJoCo 桥三脚本 + launch + simulation/mujoco 资产", from: "moveit_config + 仓库根", to: "alfa_robot_mujoco_sim", tone: "purple", reason: "A-07", order: "P5" },
    { what: "AlfaRobotHW 关节表数据驱动化 + estop/RT/方向修复", from: "硬编码 buildJoints 等", to: "URDF hardware_parameters", tone: "gray", reason: "F-01~F-06（潜伏，需实机窗口）", order: "P6" },
    { what: "auto_grasp 双拷贝合一 + 手眼外参进标定文件", from: "bringup/scripts + 仓库根 scripts", to: "感知/应用包", tone: "gray", reason: "9cm 标定分歧（G-01）", order: "P6" }
  ],

  guardrails: [
    { title: "外部契约冻结", body: "迁移全程不改：/robot_motion/* 与 /dual_arm_planner/* 服务名、/alfa_execution/execute_joint_trajectory action、关节名、控制器名、launch 参数名。搬家=换包名，不换接口。" },
    { title: "每批一个可回滚 PR", body: "每个批次独立分支+独立验证；任何一批失败整批回滚，不允许跨批次的半成品状态过夜合入主线。" },
    { title: "先修缺陷再动架构", body: "P0 的四个缺陷修复（构建断裂、恒等映射、velocity_scale、joint1 钳位判定）必须先于任何搬迁落地——在断裂的执行链上做重构等于盲飞。" },
    { title: "硬件层先隔离后重修", body: "AlfaRobotHW 当前不可达：在 P6 实机窗口之前只做『防误触』（real 插件默认哨兵化），不做行为修改；实机语义（首帧保护、安全停机、方向）修改必须在实机旁验证。" },
    { title: "干净构建是每批的准入", body: "每批合入前在干净容器里 colcon build + colcon test 全绿（P0 建的 CI 负责执行）；这是 D-01 类问题的永久疫苗。" },
    { title: "验证口径写进批次", body: "沿用 runtime_full_stack 的 smoke 口径（set_state/set_scene/run_box_pair_task dry-run 计数）+ 每批附加断言；执行链打通后增加 mock 端到端『轨迹确实运动』断言（防 C-02 类静默冻结）。" }
  ],

  phases: [
    {
      title: "P0 · 止血：修复已验证的缺陷（不动架构）",
      riskLabel: "风险：极低", riskTone: "green",
      goal: "让仓库回到『能从干净克隆构建、执行链断点已知且被测试锁定』的状态。",
      steps: [
        "从 git 历史（cd82083）恢复 mujoco_digital_twin.py 或删除其 install 规则与 launch（D-01）",
        "修复 moveit_to_execution_name/execution_to_moveit_name 恒等回归，附双射+非恒等单测（C-02）",
        "extract_failed_attempts_rerun.py 补 import（E-05）；ExecuteTrajectory 对 scale!=1.0 显式拒绝或实现重缩放（E-01）",
        "查明并处置 leftjoint1/rightjoint1 [-0.0,0.0] 钳位：恢复 ±2.356 或显式注释+同步 SRDF（E-04）",
        "建立仓库根最小 CI：干净容器 colcon build + execution_bridge 契约测试 + 安全 ctest；把 joint 名映射加入安全门（H-01/H-02）"
      ],
      verification: ["干净容器 colcon build 全绿", "新增契约单测在 CI 跑通", "mock 端到端 L6/R8 回放：任务段臂关节位置确实变化（对 C-02 的行为级断言）"],
      exit: "CI 常绿；任何人可以从零 checkout 构建整个工作区。"
    },
    {
      title: "P1 · 收编工作区外的生产依赖",
      riskLabel: "风险：低", riskTone: "green",
      goal: "让每个 ROS 包自包含：删除一切 ../../../、悬空符号链接与开发机绝对路径。",
      steps: [
        "scripts/ik_benchmark 物理迁入 ros2_ws/src（生产求解器头文件提为 alfa_robot_ik_pipeline 库，benchmark 部分成为真 alfa_robot_benchmarks 包）；删除悬空符号链接（A-04/A-05）",
        "moveit_config 删除 IK_BENCHMARK_ROOT、外部头 install、../../../ ctest；改为 find_package",
        "/mnt/mydisk 与 /opt/ros 默认值全部替换为 data_root 参数/环境派生，C++ 默认空=禁用记录（D-02）",
        "check_l6_r8_real_safety.py 移入包内测试目录（H-02）",
        "dependency.repos 按新机审读结论重生成 + bootstrap 文档（H-04）"
      ],
      verification: ["把仓库 checkout 到任意路径名下重新构建通过", "--packages-select alfa_robot_moveit_config 单包构建通过", "grep 全工作区无 /mnt/mydisk、无 parents[4]、无 ../../../"],
      exit: "colcon 依赖图完整反映真实依赖；单包可独立构建。"
    },
    {
      title: "P2 · 契约单一化",
      riskLabel: "风险：低-中", riskTone: "blue",
      goal: "把靠字符串巧合维系的跨包契约收进 robot_motion_interfaces，让 C-02 类回归在编译/CI 期死亡。",
      steps: [
        "关节命名契约：单份 YAML + 生成/加载的 names.py 与 names.hpp；joints.py、execution_trajectory_adapter.cpp、脚本全部改为消费方（C-03）",
        "服务/action 名常量进 interfaces；runtime_status 从 JSON-in-String 升级为 RuntimeStatus.msg（C-04）",
        "SemanticScene 等三接口迁入 interfaces（A-06）",
        "箱墙/携带箱几何单源化 + C++/Python 黄金值对拍测试；修 touch_links 空缺（C-06）",
        "删除双份 robot_motion_state_source、死 ros2_control xacro 与死 initial_positions（B-03）；updown 0.92/0.99 归一（B-04）",
        "bridge 内落地『拒收未映射关节』校验，mock 模式醒目公告（B-05/R-02）"
      ],
      verification: ["契约等价性测试入 CI", "runtime_full_stack smoke：ik/extract/loaded 计数不回退", "打通 runtime→bridge 命名后，mock 全链 run_box_pair_task execute=true 真实运动"],
      exit: "执行链端到端可用（mock）；同一事实在工作区内只有一个属主。"
    },
    {
      title: "P3 · 上帝包解体",
      riskLabel: "风险：中", riskTone: "orange",
      goal: "moveit_config 归零代码：规划运行时迁入 robot_motion_planning，服务节点迁到名副其实的包。",
      steps: [
        "新建 robot_motion_planning：整体移入 dual_arm_planner_node、motion 库、extract/loaded/优化 IK 管线与对应 launch/测试（A-01）",
        "motion_collision_service_node → robot_motion_scene_service；analytic_arm_ik_service_node → IK 服务包（A-02）",
        "runtime 的 exec_depend 改指新包；删除对 moveit_config 的依赖",
        "完成 motion_core 半程迁移：pose_math 落位、删转发头（R-03）",
        "试车监控/benchmark 机器留在新包但以参数默认关闭（D-04 的过渡），P5 再拆"
      ],
      verification: ["moveit_config 包内 0 行 C++/Python（CI 断言）", "服务名与行为回归：dry-run 与 P2 基线一致", "MSA 重新打开配置包不再有毁伤面"],
      exit: "依赖图与目标图一致：config 是叶子，runtime 不再依赖它。"
    },
    {
      title: "P4 · 启动收口",
      riskLabel: "风险：中", riskTone: "orange",
      goal: "bringup 复活为唯一系统装配层，控制器拓扑单一属主。",
      steps: [
        "dual_arm_planner.launch.py 的 311-330 装配段抽为 bringup/robot_system.launch.py（A-03）",
        "删除 bringup 死物：moveit_real_execute/hardware_test、path_node 等幽灵引用、死 YAML、__pycache__、debug_mesh_path（B-02/D-07）",
        "控制器拓扑收敛：ros2_controllers.yaml+moveit_controllers.yaml 单一属主（随装配层走），bringup 侧只留 all_position overlay；终结 dual_arm_controller 同名异义（B-01）",
        "runtime_full_stack 由 bringup 汇编或显式声明前置"
      ],
      verification: ["唯一入口清单：bringup 两个（系统/仿真）+ tools 若干（显式标注非生产）", "MoveIt 规划→执行 mock 回归通过", "controller 名 grep 全工作区唯一定义"],
      exit: "任何人按包名走正门能启动正确的系统。"
    },
    {
      title: "P5 · 工具与仿真隔离",
      riskLabel: "风险：低", riskTone: "blue",
      goal: "试车/调试/仿真代码离开生产 install 面。",
      steps: [
        "alfa_robot_motion_tools 建包：extract 控制台（转正为 python 模块）、rerun 回放、reachability、process_lifecycle（修 pkill 范围 D-05）",
        "execute_l6_r8 执行逻辑本体（快照→轨迹重采样→发送监控）上移 runtime/bridge 为被测库；工具包只留薄 CLI（D-03）",
        "MuJoCo 桥+资产合体为 alfa_robot_mujoco_sim（A-07）；修 sync_bridge TypeError",
        "extract 监控/benchmark 从 planning 包拆出或参数门禁默认关（D-04 收尾）",
        "box_perception stub 改显式参数 fail-loudly（D-06）；bio_ik 移出默认构建（G-03）"
      ],
      verification: ["生产包 install 列表 grep 无 demo/test/mock/monitor", "工具包依赖面=interfaces（+可选 rerun/mujoco pip）", "孪生链路在独立部署（无源码树）下运行"],
      exit: "生产与工具的边界由包边界而不是口头约定维持。"
    },
    {
      title: "P6 · 硬件层对齐（实机窗口）",
      riskLabel: "风险：高·需实机", riskTone: "red",
      goal: "决断并修复 hardware 包与真实机器的关系；把仓库外的 EtherCAT 执行栈契约钉进仓库。",
      steps: [
        "决策：AlfaRobotHW 迁移到当前机器（buildJoints 数据驱动 + URDF hardware_parameters，F-01）或正式退役、以 EtherCAT 栈为准",
        "实机语义修复并在机旁验证：首帧命令保护/禁自动 homing（F-05）、ZeroErr read/write 方向对称（F-04）、安全停机缓存刷新（F-06）、estop 并发模型与状态接口（F-03）",
        "RT 违规治理：总线 I/O 移出 read()/write()（F-02）",
        "EtherCAT 工控机栈的关节序/方向/控制器契约以文档+在库校验固化；auto_grasp 外参合一（G-01）",
        "重写 docs/CONTROL_LAYER_HARDCODED_PARAMS.md 与 REFACTOR_ARCHITECTURE_NOTES.md 为当前架构（H-03）"
      ],
      verification: ["沿用 REFACTOR_ARCHITECTURE_NOTES 的回归口径更新版：接口数量、首帧行为、停机行为、方向一致性逐项对照", "急停注入测试", "200Hz 周期抖动测量"],
      exit: "仓库描述的机器 = 实际在跑的机器；实机路径重新可信。"
    }
  ]
};
