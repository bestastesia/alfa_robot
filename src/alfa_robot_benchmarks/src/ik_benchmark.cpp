/**
 * ik_benchmark.cpp — 双臂 IK 求解器性能对比工具
 *
 * 不依赖 ROS 节点运行，直接加载机器人模型和 IK 插件进行离线测试。
 *
 * 测试流程：
 *   1. 随机生成关节角（joint6 固定为 0，避免 prismatic 关节干扰）
 *   2. 正向运动学计算双末端目标位姿
 *   3. 从 home 姿态出发，调用 IK 求解器
 *   4. 对解进行正向运动学验证，计算位置/姿态误差
 *   5. 统计：总数 / 失败 / 误差超阈值 / 误差达标
 *
 * 用法：
 *   source install/setup.bash
 *   ./install/alfa_robot_benchmarks/lib/alfa_robot_benchmarks/ik_benchmark
 *   ./install/.../ik_benchmark --samples 500 --timeout 2.0 --pos-thresh 0.005
 *   ./install/.../ik_benchmark --free-joint6   # 允许 joint6 随机（默认固定为 0）
 *   ./install/.../ik_benchmark --jsonl result.jsonl  # 输出 JSONL 供 3D 回放
 */

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_loader.hpp>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

// ── CLI 参数 ───────────────────────────────────────────────────────────────

struct CliArgs {
    int    samples     = 200;
    double timeout     = 2.0;    // 每次 IK 调用的超时（秒）
    double pos_thresh  = 0.005;  // 位置误差阈值（米），5mm
    double ori_thresh  = 0.01;   // 姿态误差阈值（弧度），约 0.6°
    bool   verbose     = false;
    bool   free_joint6 = false;  // 允许 joint6 随机（默认固定为 0）
    std::string jsonl_path;      // JSONL 输出路径（供 3D 回放）
};

static CliArgs parse_args(int argc, char** argv)
{
    CliArgs a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--samples"    && i + 1 < argc) a.samples    = std::stoi(argv[++i]);
        if (s == "--timeout"    && i + 1 < argc) a.timeout    = std::stod(argv[++i]);
        if (s == "--pos-thresh" && i + 1 < argc) a.pos_thresh = std::stod(argv[++i]);
        if (s == "--ori-thresh" && i + 1 < argc) a.ori_thresh = std::stod(argv[++i]);
        if (s == "--jsonl"       && i + 1 < argc) a.jsonl_path  = argv[++i];
        if (s == "--verbose")                    a.verbose     = true;
        if (s == "--free-joint6")                a.free_joint6 = true;
    }
    return a;
}

// ── 误差计算 ───────────────────────────────────────────────────────────────

// 位置误差：两点欧氏距离（米）
static double pos_err(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
    return (a.translation() - b.translation()).norm();
}

// 姿态误差：SO(3) 测地线角度（弧度）
static double ori_err(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
    Eigen::Quaterniond qa(a.linear()), qb(b.linear());
    double dot = std::abs(qa.dot(qb));
    return 2.0 * std::acos(std::min(dot, 1.0));
}

// ── 单次求解结果 ───────────────────────────────────────────────────────────

enum class Status { kSuccess, kLargeError, kFailed };

struct SampleResult {
    int    index;
    Status status;
    double solve_ms;
    double pos_error;  // 双臂中较大的那个
    double ori_error;
};

// ── 汇总统计 ───────────────────────────────────────────────────────────────

struct Summary {
    std::string name;
    int total = 0, success = 0, large_error = 0, failed = 0;
    double sum_ms = 0, max_ms = 0;
    double sum_pos = 0, max_pos = 0;
    double sum_ori = 0, max_ori = 0;

    void add(const SampleResult& r)
    {
        ++total;
        sum_ms += r.solve_ms;
        max_ms = std::max(max_ms, r.solve_ms);

        if (r.status == Status::kFailed) {
            ++failed;
        } else {
            sum_pos += r.pos_error;
            max_pos = std::max(max_pos, r.pos_error);
            sum_ori += r.ori_error;
            max_ori = std::max(max_ori, r.ori_error);
            if (r.status == Status::kSuccess) ++success;
            else                              ++large_error;
        }
    }

    void print() const
    {
        int solved = success + large_error;
        double avg_ms  = total  > 0 ? sum_ms  / total  : 0.0;
        double avg_pos = solved > 0 ? sum_pos / solved  : 0.0;
        double avg_ori = solved > 0 ? sum_ori / solved  : 0.0;
        double pct_ok  = total  > 0 ? 100.0 * success / total : 0.0;

        std::cout
            << "\n=== " << name << " ===\n"
            << "  总样本数     : " << total << "\n"
            << "  成功（达标） : " << success
            << "  (" << std::fixed << std::setprecision(1) << pct_ok << "%)\n"
            << "  成功（误差大）: " << large_error << "\n"
            << "  失败         : " << failed << "\n"
            << "  平均耗时     : " << std::fixed << std::setprecision(2) << avg_ms << " ms\n"
            << "  最大耗时     : " << max_ms << " ms\n"
            << "  平均位置误差 : " << std::scientific << std::setprecision(3) << avg_pos << " m\n"
            << "  最大位置误差 : " << max_pos << " m\n"
            << "  平均姿态误差 : " << avg_ori << " rad"
            << "  (" << std::fixed << std::setprecision(2) << avg_ori * 180.0 / M_PI << " deg)\n"
            << "  最大姿态误差 : " << std::scientific << std::setprecision(3) << max_ori << " rad"
            << "  (" << std::fixed << std::setprecision(2) << max_ori * 180.0 / M_PI << " deg)\n";
    }
};

// ── 测试用例 ───────────────────────────────────────────────────────────────

struct TestCase {
    std::vector<double>  true_joints;   // 生成 target 时的真实关节值（已知解）
    Eigen::Isometry3d    left_target;
    Eigen::Isometry3d    right_target;
};

/**
 * 生成测试用例：随机关节角 → FK → 双末端目标位姿（base_link 坐标系）。
 *
 * leftjoint6 / rightjoint6 是 prismatic 吸盘伸缩关节（范围 0-0.15m），
 * 不参与末端位姿的空间定位，固定为 0 以专注测试旋转关节的 IK 求解能力。
 * target 转换到 base_link 坐标系，与 IK 求解器的参考系一致。
 */
static std::vector<TestCase> generate_test_cases(
    const moveit::core::RobotModelConstPtr& model,
    const moveit::core::JointModelGroup*    jmg,
    const std::string&                      left_tip,
    const std::string&                      right_tip,
    int                                     count,
    random_numbers::RandomNumberGenerator&  rng,
    bool                                    free_joint6)
{
    std::vector<TestCase> cases;
    cases.reserve(count);

    moveit::core::RobotState state(model);
    state.setToDefaultValues();

    // 找出 joint6 在组关节列表中的索引，生成后将其重置为 0
    const auto& joint_names = jmg->getActiveJointModelNames();
    std::vector<size_t> joint6_indices;
    for (size_t k = 0; k < joint_names.size(); ++k) {
        const auto& n = joint_names[k];
        if (n == "leftjoint6" || n == "rightjoint6") {
            joint6_indices.push_back(k);
        }
    }

    // 预取组内各关节的 position bounds（用于 setToRandomPositions 后强制 clamp）
    const auto& jmodels = jmg->getActiveJointModels();
    std::vector<double> lo_bounds, hi_bounds;
    std::vector<bool>   bounded;
    lo_bounds.reserve(jmodels.size());
    hi_bounds.reserve(jmodels.size());
    bounded.reserve(jmodels.size());
    for (auto* jm : jmodels) {
        const auto& b = jm->getVariableBounds()[0];
        lo_bounds.push_back(b.min_position_);
        hi_bounds.push_back(b.max_position_);
        bounded.push_back(b.position_bounded_);
    }

    for (int i = 0; i < count; ++i) {
        state.setToRandomPositions(jmg, rng);

        // 强制 clamp 到 URDF 关节范围（修复 setToRandomPositions 未正确限制 revolute 关节的问题）
        std::vector<double> jv_tmp;
        state.copyJointGroupPositions(jmg, jv_tmp);
        for (size_t k = 0; k < jv_tmp.size(); ++k) {
            if (bounded[k]) {
                jv_tmp[k] = std::clamp(jv_tmp[k], lo_bounds[k], hi_bounds[k]);
            }
        }
        state.setJointGroupPositions(jmg, jv_tmp);

        // 将 prismatic joint6 固定为 0（吸盘伸缩不参与位姿求解）
        // 除非 --free-joint6 指定允许随机
        if (!free_joint6) {
            for (size_t k : joint6_indices) {
                state.setJointPositions(joint_names[k], {0.0});
            }
        }
        state.update();

        // target 转换到 base_link 坐标系，与 IK 求解器的参考系一致
        const Eigen::Isometry3d T_base_inv =
            state.getGlobalLinkTransform("base_link").inverse();

        std::vector<double> jv;
        state.copyJointGroupPositions(jmg, jv);

        cases.push_back({
            jv,
            T_base_inv * state.getGlobalLinkTransform(left_tip),
            T_base_inv * state.getGlobalLinkTransform(right_tip)
        });
    }

    return cases;
}

// ── 将 Eigen::Isometry3d 转为 geometry_msgs::msg::Pose ────────────────────

static geometry_msgs::msg::Pose to_pose_msg(const Eigen::Isometry3d& tf)
{
    geometry_msgs::msg::Pose p;
    p.position.x = tf.translation().x();
    p.position.y = tf.translation().y();
    p.position.z = tf.translation().z();
    Eigen::Quaterniond q(tf.linear());
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}

// ── JSONL 输出辅助 ────────────────────────────────────────────────────────

// 手动拼接 JSON 数组：[v0, v1, v2, ...]
static std::string json_array(const std::vector<double>& v)
{
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i > 0) os << ", ";
        os << std::setprecision(8) << v[i];
    }
    os << "]";
    return os.str();
}

// 手动拼接 JSON 数组：[x, y, z] 从 Isometry3d 的平移部分
static std::string json_pos(const Eigen::Isometry3d& tf)
{
    std::ostringstream os;
    os << std::setprecision(8)
       << "[" << tf.translation().x()
       << ", " << tf.translation().y()
       << ", " << tf.translation().z() << "]";
    return os.str();
}

// ── 主程序 ─────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    // IK 插件内部会创建 rclcpp::Node 读取参数，必须先 init
    rclcpp::init(argc, argv);

    auto args = parse_args(argc, argv);

    std::cout
        << "╔══════════════════════════════════════════════════╗\n"
        << "║      ALFA Robot IK Solver Benchmark              ║\n"
        << "╚══════════════════════════════════════════════════╝\n\n"
        << "样本数       : " << args.samples    << "\n"
        << "超时         : " << args.timeout    << " s\n"
        << "位置阈值     : " << args.pos_thresh << " m\n"
        << "姿态阈值     : " << args.ori_thresh << " rad\n"
        << "joint6       : " << (args.free_joint6 ? "随机（free）" : "固定为 0") << "\n\n";

    // ── 1. 加载机器人模型（离线，不依赖 ROS 参数服务器）─────────────────

    std::string desc_share   = ament_index_cpp::get_package_share_directory("alfa_robot_description");
    std::string moveit_share = ament_index_cpp::get_package_share_directory("alfa_robot_moveit_config");

    // 优先使用预生成的 .urdf；若不存在则自动调用 xacro 生成
    std::string urdf_path = desc_share + "/urdf/alfa_robot/alfa_robot.urdf";
    {
        std::ifstream test(urdf_path);
        if (!test.good()) {
            std::string xacro_src = desc_share + "/urdf/alfa_robot.urdf.xacro";
            std::cout << "未找到预生成 URDF，正在运行 xacro...\n";
            int ret = std::system(("xacro " + xacro_src + " > /tmp/_alfa_bench.urdf 2>/dev/null").c_str());
            if (ret != 0) {
                std::cerr << "错误：xacro 失败（是否已 source install/setup.bash？）\n";
                rclcpp::shutdown();
                return 1;
            }
            urdf_path = "/tmp/_alfa_bench.urdf";
        }
    }

    std::string srdf_path = moveit_share + "/config/alfa_robot.srdf";

    std::cout << "URDF : " << urdf_path << "\n";
    std::cout << "SRDF : " << srdf_path << "\n\n";

    // 读取文件内容
    auto read_file = [](const std::string& path) -> std::string {
        std::ifstream f(path);
        if (!f.good()) throw std::runtime_error("无法读取文件: " + path);
        return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    };

    std::string urdf_xml = read_file(urdf_path);
    std::string srdf_xml = read_file(srdf_path);


    // 解析 URDF
    auto urdf_model = urdf::parseURDF(urdf_xml);
    if (!urdf_model) {
        std::cerr << "错误：URDF 解析失败\n";
        rclcpp::shutdown();
        return 1;
    }

    // 解析 SRDF（RobotModel 构造函数需要 shared_ptr<const srdf::Model>）
    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, srdf_xml);

    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);

    // 获取规划组
    const std::string group_name = "dual_arm_with_base";
    const auto* jmg = robot_model->getJointModelGroup(group_name);
    if (!jmg) {
        std::cerr << "错误：SRDF 中未找到规划组 '" << group_name << "'\n";
        rclcpp::shutdown();
        return 1;
    }

    // 末端执行器链接（与 SRDF end_effector 定义一致）
    const std::string left_tip  = "leftjoint6_link";
    const std::string right_tip = "rightjoint6_link";

    std::cout << "规划组     : " << group_name << "\n";
    std::cout << "活动关节数 : " << jmg->getActiveJointModels().size() << "\n";
    std::cout << "左末端链接 : " << left_tip  << "\n";
    std::cout << "右末端链接 : " << right_tip << "\n\n";

    // ── 2. 加载 IK 插件 ────────────────────────────────────────────────────

    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
        "moveit_core", "kinematics::KinematicsBase");

    struct SolverDef {
        std::string display_name;
        std::string plugin_id;
    };

    std::vector<SolverDef> solver_defs = {
        {"pick_ik (global memetic)", "pick_ik/PickIkPlugin"},
        {"bio_ik",                   "bio_ik/BioIKKinematicsPlugin"},
    };

    // 预检：确认插件可加载
    std::cout << "检查插件可用性：\n";
    for (auto& sd : solver_defs) {
        try {
            loader.createSharedInstance(sd.plugin_id);
            std::cout << "  [OK]   " << sd.plugin_id << "\n";
        } catch (const pluginlib::PluginlibException& e) {
            std::cout << "  [SKIP] " << sd.plugin_id << " — " << e.what() << "\n";
            sd.plugin_id.clear();  // 标记为不可用
        }
    }
    std::cout << "\n";

    // ── 3. 生成测试用例 ────────────────────────────────────────────────────

    random_numbers::RandomNumberGenerator rng(42);  // 固定种子，保证可复现

    std::cout << "生成 " << args.samples << " 个测试用例（随机关节角 + FK）...\n";
    auto cases = generate_test_cases(robot_model, jmg, left_tip, right_tip, args.samples, rng, args.free_joint6);
    std::cout << "完成。\n\n";

    // ── 4. JSONL 输出文件（空文件，后续逐行追加）──────────────────────────

    if (!args.jsonl_path.empty()) {
        std::ofstream f(args.jsonl_path, std::ios::trunc);
        // 写入文件头行：元信息，供 Python 脚本识别
        std::ofstream jf(args.jsonl_path, std::ios::app);
        jf << "{\"header\":true,\"samples\":" << args.samples
           << ",\"group\":\"" << group_name
           << "\",\"left_tip\":\"" << left_tip
           << "\",\"right_tip\":\"" << right_tip
           << "\",\"joint6_free\":" << (args.free_joint6 ? "true" : "false")
           << "}\n";
    }

    // ── 5. 逐求解器跑基准测试 ──────────────────────────────────────────────

    std::vector<Summary> all_summaries;

    for (const auto& sd : solver_defs) {
        if (sd.plugin_id.empty()) continue;


        // 为每个求解器创建独立实例
        auto solver = loader.createSharedInstance(sd.plugin_id);

        // 创建最小化 rclcpp::Node 用于参数传递（不 spin，不参与 ROS 通信）
        auto node = std::make_shared<rclcpp::Node>(
            "_ik_bench_node",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

        // 参数命名空间：robot_description_kinematics.<group_name>
        // 这是 MoveIt 运行时读取 kinematics.yaml 的标准路径
        const std::string ns = "robot_description_kinematics." + group_name;

        node->declare_parameter(ns + ".kinematics_solver",                   sd.plugin_id);
        node->declare_parameter(ns + ".kinematics_solver_search_resolution", 0.005);
        node->declare_parameter(ns + ".kinematics_solver_timeout",           args.timeout);
        node->declare_parameter(ns + ".kinematics_solver_attempts",          10);

        if (sd.plugin_id.find("pick_ik") != std::string::npos) {
            node->declare_parameter(ns + ".mode",                        std::string("global"));
            node->declare_parameter(ns + ".position_threshold",          0.001);
            node->declare_parameter(ns + ".orientation_threshold",       0.01);
            node->declare_parameter(ns + ".position_scale",              1.0);
            node->declare_parameter(ns + ".rotation_scale",              0.5);
            node->declare_parameter(ns + ".minimal_displacement_weight", 0.001);
            node->declare_parameter(ns + ".memetic_num_threads",         4);
            node->declare_parameter(ns + ".memetic_population_size",     32);
            node->declare_parameter(ns + ".memetic_max_generations",     200);
            node->declare_parameter(ns + ".memetic_elite_size",          8);
            node->declare_parameter(ns + ".cost_threshold",              0.1);
            node->declare_parameter(ns + ".fix_unspecified_end_effectors", true);
        }

        if (sd.plugin_id.find("bio_ik") != std::string::npos) {
            node->declare_parameter(ns + ".bio_ik_max_computation_time", args.timeout);
        }

        // 初始化求解器：绑定机器人模型、规划组、末端链接
        bool init_ok = solver->initialize(
            node, *robot_model, group_name, "base_link",
            {left_tip, right_tip}, 0.005);

        if (!init_ok) {
            std::cerr << "错误：求解器初始化失败 — " << sd.plugin_id << "\n";
            continue;
        }

        std::cout << "── 测试 " << sd.display_name << " ──\n";

        Summary summary;
        summary.name = sd.display_name;

        std::vector<SampleResult> results;
        results.reserve(cases.size());

        moveit::core::RobotState ik_state(robot_model);

        // 每个求解器打开一次 JSONL 文件，追加写入
        std::ofstream jsonl_file;
        if (!args.jsonl_path.empty()) {
            jsonl_file.open(args.jsonl_path, std::ios::app);
        }

        for (int i = 0; i < static_cast<int>(cases.size()); ++i) {
            const auto& tc = cases[i];

            // 种子：home 姿态（全零），测试 solver 的全局搜索能力
            ik_state.setToDefaultValues();
            ik_state.update();
            std::vector<double> seed;
            ik_state.copyJointGroupPositions(jmg, seed);

            // 构造双末端目标位姿（顺序必须与 tip_frames 一致）
            std::vector<geometry_msgs::msg::Pose> ik_poses = {
                to_pose_msg(tc.left_target),
                to_pose_msg(tc.right_target)
            };

            // 调用 IK 求解
            std::vector<double> solution;
            std::vector<double> consistency_limits;  // 空 = 不限制一致性
            moveit_msgs::msg::MoveItErrorCodes error_code;
            kinematics::KinematicsQueryOptions options;

            auto t0 = std::chrono::high_resolution_clock::now();
            bool ok = solver->searchPositionIK(
                ik_poses, seed, args.timeout,
                consistency_limits, solution,
                kinematics::KinematicsBase::IKCallbackFn(),
                error_code, options);
            auto t1 = std::chrono::high_resolution_clock::now();

            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            SampleResult r;
            r.index     = i;
            r.solve_ms  = ms;
            r.pos_error = 0.0;
            r.ori_error = 0.0;

            // FK 验证结果（用于 JSONL 输出）
            Eigen::Isometry3d left_actual_tf = Eigen::Isometry3d::Identity();

            if (!ok) {
                r.status = Status::kFailed;
            } else {
                // FK 验证：在 base_link 坐标系中比较
                ik_state.setJointGroupPositions(jmg, solution);
                ik_state.update();

                const Eigen::Isometry3d T_base_inv =
                    ik_state.getGlobalLinkTransform("base_link").inverse();

                Eigen::Isometry3d left_solved  = T_base_inv * ik_state.getGlobalLinkTransform(left_tip);
                Eigen::Isometry3d right_solved = T_base_inv * ik_state.getGlobalLinkTransform(right_tip);

                double pe = std::max(
                    pos_err(tc.left_target,  left_solved),
                    pos_err(tc.right_target, right_solved));
                double oe = std::max(
                    ori_err(tc.left_target,  left_solved),
                    ori_err(tc.right_target, right_solved));

                r.pos_error = pe;
                r.ori_error = oe;
                r.status = (pe <= args.pos_thresh && oe <= args.ori_thresh)
                           ? Status::kSuccess : Status::kLargeError;

                left_actual_tf = left_solved;
            }

            summary.add(r);
            results.push_back(r);

            // ── JSONL 逐行写入 ─────────────────────────────────────────────
            if (jsonl_file.is_open()) {
                const char* status_str = (r.status == Status::kSuccess)    ? "success" :
                                         (r.status == Status::kLargeError) ? "large_error" : "failed";

                jsonl_file << "{"
                    << "\"solver\":\"" << sd.display_name << "\""
                    << ",\"index\":" << i
                    << ",\"status\":\"" << status_str << "\""
                    << ",\"solve_ms\":" << std::setprecision(4) << ms
                    << ",\"pos_error\":" << std::setprecision(8) << r.pos_error
                    << ",\"target_joints\":" << json_array(tc.true_joints)
                    << ",\"solved_joints\":" << (ok ? json_array(solution) : "[]")
                    << ",\"left_target_pos\":" << json_pos(tc.left_target)
                    << ",\"left_actual_pos\":" << (ok ? json_pos(left_actual_tf) : "[0, 0, 0]")
                    << "}\n";
            }

            if (args.verbose) {
                const char* tag = (r.status == Status::kSuccess)    ? "OK  " :
                                  (r.status == Status::kLargeError) ? "ERR " : "FAIL";
                std::cout << "  [" << std::setw(4) << i << "] "
                          << tag << "  "
                          << std::fixed << std::setprecision(1) << ms << " ms";
                if (ok) {
                    std::cout << "  pos=" << std::scientific << std::setprecision(2)
                              << r.pos_error
                              << "  ori=" << r.ori_error;
                }
                std::cout << "\n";
            }
        }

        summary.print();
        all_summaries.push_back(summary);
    }

    // ── 6. 横向对比汇总 ────────────────────────────────────────────────────

    if (all_summaries.size() > 1) {
        std::cout << "\n╔══════════════════════════════════════════════════╗\n";
        std::cout << "║                  横向对比                        ║\n";
        std::cout << "╚══════════════════════════════════════════════════╝\n";
        std::cout << std::left
                  << std::setw(28) << "求解器"
                  << std::setw(10) << "成功率"
                  << std::setw(12) << "平均耗时"
                  << std::setw(14) << "平均位置误差"
                  << "\n"
                  << std::string(64, '-') << "\n";
        for (const auto& s : all_summaries) {
            double pct = s.total > 0 ? 100.0 * s.success / s.total : 0.0;
            double avg_ms  = s.total > 0 ? s.sum_ms / s.total : 0.0;
            int solved = s.success + s.large_error;
            double avg_pos = solved > 0 ? s.sum_pos / solved : 0.0;
            std::cout << std::left  << std::setw(28) << s.name
                      << std::right << std::setw(7)  << std::fixed << std::setprecision(1) << pct << "%  "
                      << std::setw(8) << std::setprecision(1) << avg_ms << " ms  "
                      << std::scientific << std::setprecision(2) << avg_pos << " m\n";
        }
    }

    std::cout << "\n基准测试完成。\n";

    rclcpp::shutdown();
    return 0;
}
