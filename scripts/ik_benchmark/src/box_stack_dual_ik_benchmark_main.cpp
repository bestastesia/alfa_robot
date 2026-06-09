#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using ik_benchmark::ParallelUpdownAwareIkSolver;
using ik_benchmark::UpdownAwareIkConfig;
using ik_benchmark::UpdownAwareIkRequest;

namespace {

struct BoxSpec {
    int id = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct PickPair {
    int round = 0;
    int left_box = 0;
    int right_box = 0;
    bool top_suction = false;
};

constexpr double kWorldToBaseZ = 0.202094;

std::vector<double> degSeed(const std::vector<double>& degrees)
{
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (double degree : degrees) radians.push_back(degree * M_PI / 180.0);
    return radians;
}

Eigen::Isometry3d makePose(double x, double y, double z, const Eigen::Quaterniond& orientation)
{
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(x, y, z);
    Eigen::Quaterniond q = orientation;
    q.normalize();
    tf.linear() = q.toRotationMatrix();
    return tf;
}

Eigen::Quaterniond forwardXOrientation()
{
    return Eigen::Quaterniond(0.7071, 0.0, 0.7071, 0.0); // tool +Z -> world +X
}

Eigen::Quaterniond topSuctionOrientation()
{
    return Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0); // tool +Z -> world -Z
}

Eigen::Isometry3d poseForwardX(double x, double y, double z)
{
    return makePose(x, y, z - kWorldToBaseZ, forwardXOrientation());
}

Eigen::Isometry3d poseTopSuction(double x, double y, double z)
{
    return makePose(x + 0.15, y, z + 0.2 - kWorldToBaseZ, topSuctionOrientation());
}

BoxSpec targetBox(const BoxSpec& box, bool top_suction)
{
    return top_suction
        ? BoxSpec{box.id, box.x + 0.15, box.y, box.z + 0.2 - kWorldToBaseZ}
        : BoxSpec{box.id, box.x, box.y, box.z - kWorldToBaseZ};
}

BoxSpec targetBoxWorld(const BoxSpec& box, bool top_suction)
{
    return top_suction ? BoxSpec{box.id, box.x + 0.15, box.y, box.z + 0.2} : box;
}

std::map<int, BoxSpec> makeBoxes(double x_offset)
{
    const double base_x = 0.375 + x_offset;
    const std::vector<std::vector<std::pair<int, double>>> rows_top_to_bottom = {
        {{1, 0.6}, {3, 0.2}, {2, -0.2}, {4, -0.6}},
        {{5, 0.6}, {7, 0.2}, {6, -0.2}, {8, -0.6}},
        {{9, 0.6}, {11, 0.2}, {10, -0.2}, {12, -0.6}},
        {{13, 0.6}, {15, 0.2}, {14, -0.2}, {16, -0.6}},
        {{17, 0.6}, {19, 0.2}, {18, -0.2}, {20, -0.6}},
    };

    std::map<int, BoxSpec> boxes;
    for (size_t row = 0; row < rows_top_to_bottom.size(); ++row) {
        const double z = 0.2 + 0.4 * static_cast<double>(rows_top_to_bottom.size() - 1 - row);
        for (const auto& [id, y] : rows_top_to_bottom[row]) {
            boxes[id] = BoxSpec{id, base_x, y, z};
        }
    }
    return boxes;
}

std::vector<PickPair> makePickPairs()
{
    return {
        {1, 1, 2, false},
        {2, 3, 4, false},
        {3, 5, 6, false},
        {4, 7, 8, false},
        {5, 9, 10, false},
        {6, 11, 12, false},
        {7, 13, 14, false},
        {8, 15, 16, false},
        {9, 17, 18, true},
        {10, 19, 20, true},
    };
}

std::vector<double> fullValues(double updown, const std::vector<double>& left, const std::vector<double>& right)
{
    std::vector<double> values;
    values.reserve(13);
    values.push_back(updown);
    values.insert(values.end(), left.begin(), left.end());
    values.insert(values.end(), right.begin(), right.end());
    return values;
}

nlohmann::json vecJson(const std::vector<double>& values)
{
    nlohmann::json out = nlohmann::json::array();
    for (double value : values) out.push_back(value);
    return out;
}

nlohmann::json poseJson(const BoxSpec& box)
{
    return {box.x, box.y, box.z};
}

std::vector<double> makeHomeArmSeed()
{
    return degSeed({0, 15, 135, 0, 60, 0});
}

std::vector<double> makeHomeFullSeed()
{
    return fullValues(0.45, makeHomeArmSeed(), makeHomeArmSeed());
}

UpdownAwareIkConfig makeBioIkConfig()
{
    UpdownAwareIkConfig config;
    config.fixed_group = "dual_v5_arm";
    config.free_group = "dual_v5_arm_with_base";
    config.solver_plugin = "bio_ik/BioIKKinematicsPlugin";
    config.base_frame = "base_link";
    config.left_tip = "left_v5_tool0";
    config.right_tip = "right_v5_tool0";
    config.tool0_offset = 0.0;
    config.gripper_z_reach_lower = 0.45 - kWorldToBaseZ;
    config.gripper_z_reach_upper = 1.25 - kWorldToBaseZ;
    config.top_suction_z_reach_lower = 0.3 - kWorldToBaseZ;
    config.top_suction_z_reach_upper = 0.45 - kWorldToBaseZ;
    config.h_lower = 0.0;
    config.h_upper = 0.99;
    config.h_search_mode = UpdownAwareIkConfig::HSearchMode::FixedDiscrete;
    config.h_search_margin = 0.2;
    config.h_step = 0.1;
    config.h_candidate_count = 16;
    config.seed_count = 32;
    config.workers = 16;
    config.timeout = 0.01;
    config.try_target_orders = false;
    config.use_reversed_target_order = true;
    config.check_tip_error = true;
    config.position_tolerance = 0.02;
    config.top_suction_position_tolerance = 0.04;
    config.orientation_tolerance = 0.05;
    config.top_suction_orientation_tolerance = 7.0 * M_PI / 180.0;
    config.check_collision = true;
    config.enforce_arm_base_collisions = true;
    config.reject_swapped_tips = true;
    config.fallback_enabled = false;
    return config;
}


nlohmann::json rejectionSummaryJson(const std::vector<ik_benchmark::UpdownAwareIkCandidate>& candidates)
{
    std::map<std::string, size_t> counts;
    for (const auto& candidate : candidates) {
        if (!candidate.legal) {
            counts[candidate.rejection_reason.empty() ? "unknown" : candidate.rejection_reason]++;
        }
    }
    nlohmann::json out = nlohmann::json::object();
    for (const auto& [reason, count] : counts) out[reason] = count;
    return out;
}

const ik_benchmark::UpdownAwareIkCandidate* bestRejectedCandidate(
    const std::vector<ik_benchmark::UpdownAwareIkCandidate>& candidates)
{
    const ik_benchmark::UpdownAwareIkCandidate* best = nullptr;
    for (const auto& candidate : candidates) {
        if (candidate.full_joint_values.empty() || candidate.full_joint_names.empty()) continue;
        if (best == nullptr || candidate.direct_pos_error < best->direct_pos_error) {
            best = &candidate;
        }
    }
    return best;
}

void printHelp()
{
    std::cout << "box_stack_dual_ik_benchmark\n"
              << "  --solver-mode optimized_bioik   currently supported solver mode\n"
              << "  --x-offset <m>                  box x = 0.375 + x_offset, default 0.3\n"
              << "  --output <path>                 JSONL output path\n"
              << "  --timeout <sec>                 per-candidate BioIK timeout, default 0.02\n"
              << "  --workers <n>                   parallel workers, default 8\n"
              << "  --seed-count <n>                seeds per h candidate, default 32\n"
              << "  --h-candidates <n>              h candidate count, default 8\n";
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string solver_mode = "optimized_bioik";
    double x_offset = 0.3;
    std::string output = "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/box_stack_dual_ik/box_stack_dual_ik_bioik.jsonl";
    UpdownAwareIkConfig config = makeBioIkConfig();

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--solver-mode" && i + 1 < argc) solver_mode = argv[++i];
        else if (arg == "--x-offset" && i + 1 < argc) x_offset = std::stod(argv[++i]);
        else if ((arg == "--output" || arg == "--jsonl") && i + 1 < argc) output = argv[++i];
        else if (arg == "--timeout" && i + 1 < argc) config.timeout = std::stod(argv[++i]);
        else if (arg == "--workers" && i + 1 < argc) config.workers = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--seed-count" && i + 1 < argc) config.seed_count = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--h-candidates" && i + 1 < argc) config.h_candidate_count = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--help" || arg == "-h") { printHelp(); rclcpp::shutdown(); return 0; }
    }

    if (solver_mode != "optimized_bioik") {
        std::cerr << "Unsupported --solver-mode for now: " << solver_mode << "\n";
        rclcpp::shutdown();
        return 2;
    }

    std::filesystem::path output_path(output);
    if (!output_path.parent_path().empty()) std::filesystem::create_directories(output_path.parent_path());
    std::ofstream ofs(output);
    if (!ofs) {
        std::cerr << "Failed to open output: " << output << "\n";
        rclcpp::shutdown();
        return 2;
    }

    const auto boxes = makeBoxes(x_offset);
    const auto pairs = makePickPairs();
    const auto home_arm_seed = makeHomeArmSeed();
    const auto home_full_seed = makeHomeFullSeed();
    const auto loaded_arm = degSeed({0, 5, 145, 0, 120, 0});
    const auto place_arm = degSeed({0, -90, -90, 0, -90, 180});

    ParallelUpdownAwareIkSolver solver(config);

    nlohmann::json header = {
        {"type", "header"},
        {"schema", "box_stack_dual_ik_benchmark_v1"},
        {"solver_mode", solver_mode},
        {"box_front_face_x", 0.375 + x_offset},
        {"box_x", 0.375 + x_offset},
        {"x_offset", x_offset},
        {"box_depth_x", 0.3},
        {"box_face_size_yz", {0.4, 0.4}},
        {"grasp_point_semantics", "front-face center facing robot; box volume extends +X by 0.3m"},
        {"rounds", pairs.size()},
        {"grasp_orientation", "front: tool +Z toward world +X; top_suction: tool +Z toward world -Z"},
        {"home_updown", 0.45},
        {"home_arm_degrees", {0, 15, 135, 0, 60, 0}},
        {"loaded_arm_degrees", {0, 5, 145, 0, 120, 0}},
        {"place_arm_degrees", {0, -90, -90, 0, -90, 180}},
        {"h_planner", {{"gripper_z_reach_window", {config.gripper_z_reach_lower, config.gripper_z_reach_upper}}, {"top_suction_z_reach_window", {config.top_suction_z_reach_lower, config.top_suction_z_reach_upper}}, {"physical_limits", {config.h_lower, config.h_upper}}}},
        {"candidate_budget", {{"h_candidates", config.h_candidate_count}, {"seed_count", config.seed_count}, {"max_trials", config.h_candidate_count * config.seed_count}}},
        {"workers", config.workers},
        {"timeout", config.timeout},
        {"tip_error_tolerance", { {"front_position", config.position_tolerance}, {"top_suction_position", config.top_suction_position_tolerance}, {"front_orientation", config.orientation_tolerance}, {"top_suction_orientation", config.top_suction_orientation_tolerance} }},
        {"check_collision", config.check_collision}
    };
    ofs << header.dump() << "\n";

    size_t success_rounds = 0;
    const double retry_h_jitter = 0.05;
    std::cout << "=== Box Stack Dual IK Benchmark ===\n"
              << "  solver_mode=" << solver_mode << " box_x=" << (0.375 + x_offset)
              << " rounds=" << pairs.size() << " output=" << output << "\n"
              << "  home seed: updown=0.45, arms=0/15/135/0/60/0 deg\n\n";

    for (const auto& pair : pairs) {
        const BoxSpec& left_box_raw = boxes.at(pair.left_box);
        const BoxSpec& right_box_raw = boxes.at(pair.right_box);
        const BoxSpec left_box = targetBox(left_box_raw, pair.top_suction);
        const BoxSpec right_box = targetBox(right_box_raw, pair.top_suction);
        const BoxSpec left_box_world = targetBoxWorld(left_box_raw, pair.top_suction);
        const BoxSpec right_box_world = targetBoxWorld(right_box_raw, pair.top_suction);

        UpdownAwareIkRequest request;
        request.left_target = pair.top_suction
            ? poseTopSuction(left_box_raw.x, left_box_raw.y, left_box_raw.z)
            : poseForwardX(left_box_raw.x, left_box_raw.y, left_box_raw.z);
        request.right_target = pair.top_suction
            ? poseTopSuction(right_box_raw.x, right_box_raw.y, right_box_raw.z)
            : poseForwardX(right_box_raw.x, right_box_raw.y, right_box_raw.z);
        request.current_h = 0.45;
        request.grasp_mode = pair.top_suction
            ? UpdownAwareIkRequest::GraspMode::TopSuction
            : UpdownAwareIkRequest::GraspMode::Front;
        request.current_arm_joints = fullValues(0.0, home_arm_seed, home_arm_seed);
        request.current_arm_joints.erase(request.current_arm_joints.begin());
        request.current_full_joints = home_full_seed;

        auto result = solver.solve(request);
        const auto initial_result = result;
        bool retry_used = false;
        double retry_current_h = request.current_h;
        double retry_wall_ms = 0.0;
        double retry_sum_solve_ms = 0.0;
        size_t retry_trial_count = 0;
        if (!result.success) {
            retry_used = true;
            retry_current_h = std::min(config.h_upper, request.current_h + retry_h_jitter);
            UpdownAwareIkRequest retry_request = request;
            retry_request.current_h = retry_current_h;
            auto retry_result = solver.solve(retry_request);
            retry_wall_ms = retry_result.wall_ms;
            retry_sum_solve_ms = retry_result.sum_solve_ms;
            retry_trial_count = retry_result.trial_count;
            const auto* initial_best_rejected = bestRejectedCandidate(result.candidates);
            const auto* retry_best_rejected = bestRejectedCandidate(retry_result.candidates);
            const double initial_error = result.success
                ? result.selected.direct_pos_error
                : (initial_best_rejected ? initial_best_rejected->direct_pos_error : std::numeric_limits<double>::infinity());
            const double retry_error = retry_result.success
                ? retry_result.selected.direct_pos_error
                : (retry_best_rejected ? retry_best_rejected->direct_pos_error : std::numeric_limits<double>::infinity());
            if ((retry_result.success && !result.success) ||
                (retry_result.success == result.success && retry_error < initial_error)) {
                result = std::move(retry_result);
            }
        }
        if (result.success) ++success_rounds;
        const double task_wall_ms = initial_result.wall_ms + retry_wall_ms;
        const double task_sum_solve_ms = initial_result.sum_solve_ms + retry_sum_solve_ms;
        const size_t task_trial_count = initial_result.trial_count + retry_trial_count;

        const auto* best_rejected = bestRejectedCandidate(result.candidates);

        nlohmann::json record = {
            {"type", "round"},
            {"round", pair.round},
            {"solver_mode", solver_mode},
            {"left_box", pair.left_box},
            {"right_box", pair.right_box},
            {"grasp_mode", pair.top_suction ? "top_suction" : "front"},
            {"left_target", poseJson(left_box)},
            {"right_target", poseJson(right_box)},
            {"left_target_world", poseJson(left_box_world)},
            {"right_target_world", poseJson(right_box_world)},
            {"target_orientation", pair.top_suction ? "tool +Z toward world -Z" : "tool +Z toward world +X"},
            {"home_updown", 0.45},
            {"home_arm_joint_values", vecJson(home_arm_seed)},
            {"home_full_joint_values", vecJson(home_full_seed)},
            {"pregrasp_updown", 0.45},
            {"pregrasp_left_joint_values", vecJson(home_arm_seed)},
            {"pregrasp_right_joint_values", vecJson(home_arm_seed)},
            {"loaded_updown", 0.45},
            {"loaded_left_joint_values", vecJson(loaded_arm)},
            {"loaded_right_joint_values", vecJson(loaded_arm)},
            {"place_updown", 0.45},
            {"place_left_joint_values", vecJson(place_arm)},
            {"place_right_joint_values", vecJson(place_arm)},
            {"success", result.success},
            {"retry_used", retry_used},
            {"retry_h_jitter", retry_used ? retry_h_jitter : 0.0},
            {"retry_current_h", retry_current_h},
            {"front_retry_used", retry_used && !pair.top_suction},
            {"top_suction_retry_used", retry_used && pair.top_suction},
            {"initial_success", initial_result.success},
            {"initial_h_center", initial_result.h_center},
            {"initial_h_candidates", initial_result.h_candidates},
            {"initial_trial_count", initial_result.trial_count},
            {"initial_legal_count", initial_result.legal_count},
            {"initial_wall_ms", initial_result.wall_ms},
            {"initial_sum_solve_ms", initial_result.sum_solve_ms},
            {"initial_rejection_summary", rejectionSummaryJson(initial_result.candidates)},
            {"retry_trial_count", retry_trial_count},
            {"retry_wall_ms", retry_wall_ms},
            {"retry_sum_solve_ms", retry_sum_solve_ms},
            {"adopted_wall_ms", result.wall_ms},
            {"adopted_sum_solve_ms", result.sum_solve_ms},
            {"task_wall_ms", task_wall_ms},
            {"task_sum_solve_ms", task_sum_solve_ms},
            {"task_trial_count", task_trial_count},
            {"fallback_used", result.fallback_used},
            {"solver_path", result.solver_path},
            {"failure_reason", result.failure_reason},
            {"h_interval_lower", result.h_interval_lower},
            {"h_interval_upper", result.h_interval_upper},
            {"h_interval_reachable", result.range_reachable},
            {"h_center", result.h_center},
            {"h_candidates", result.h_candidates},
            {"trial_count", result.trial_count},
            {"legal_count", result.legal_count},
            {"timeout_like_count", result.timeout_like_count},
            {"rejection_summary", rejectionSummaryJson(result.candidates)},
            {"wall_ms", result.wall_ms},
            {"sum_solve_ms", result.sum_solve_ms},
            {"selected_h", result.success ? result.selected.h : 0.45},
            {"selected_score", result.success ? result.selected.score : std::numeric_limits<double>::infinity()},
            {"selected_h_index", result.success ? result.selected.h_index : 0},
            {"selected_seed_index", result.success ? result.selected.seed_index : 0},
            {"selected_solver_path", result.success ? result.selected.solver_path : ""},
            {"selected_target_order", result.success ? result.selected.target_order : ""},
            {"selected_direct_pos_error", result.success ? result.selected.direct_pos_error : 0.0},
            {"selected_direct_ori_error", result.success ? result.selected.direct_ori_error : 0.0},
            {"selected_collision_free", result.success ? result.selected.collision_free : false},
            {"selected_collision_pairs", result.success ? result.selected.collision_pairs : std::vector<std::string>{}},
            {"selected_joint_names", result.success ? result.selected.full_joint_names : std::vector<std::string>{}},
            {"selected_joint_values", result.success ? result.selected.full_joint_values : std::vector<double>{}},
            {"best_rejected_available", best_rejected != nullptr},
            {"best_rejected_reason", best_rejected ? best_rejected->rejection_reason : ""},
            {"best_rejected_direct_pos_error", best_rejected ? best_rejected->direct_pos_error : 0.0},
            {"best_rejected_direct_ori_error", best_rejected ? best_rejected->direct_ori_error : 0.0},
            {"best_rejected_h", best_rejected ? best_rejected->h : 0.45},
            {"best_rejected_h_index", best_rejected ? best_rejected->h_index : 0},
            {"best_rejected_seed_index", best_rejected ? best_rejected->seed_index : 0},
            {"best_rejected_joint_names", best_rejected ? best_rejected->full_joint_names : std::vector<std::string>{}},
            {"best_rejected_joint_values", best_rejected ? best_rejected->full_joint_values : std::vector<double>{}}
        };
        ofs << record.dump() << "\n";

        std::cout << "round " << pair.round << " boxes " << pair.left_box << "+" << pair.right_box
                  << " mode=" << (pair.top_suction ? "top_suction" : "front")
                  << " success=" << (result.success ? "yes" : "no")
                  << " h=[" << std::fixed << std::setprecision(2) << result.h_interval_lower << "," << result.h_interval_upper << "]"
                  << " selected_h=" << (result.success ? result.selected.h : 0.45)
                  << " legal=" << result.legal_count << "/" << result.trial_count
                  << " wall=" << std::setprecision(1) << result.wall_ms << "ms"
                  << " task_wall=" << task_wall_ms << "ms";
        if (retry_used) {
            std::cout << " retry_h=" << retry_current_h
                      << " initial_legal=" << initial_result.legal_count << "/" << initial_result.trial_count;
        }
        if (!result.success) {
            std::cout << " reason=" << result.failure_reason << " reject=" << rejectionSummaryJson(result.candidates).dump();
        }
        std::cout << "\n";
    }

    nlohmann::json summary = {
        {"type", "summary"},
        {"rounds", pairs.size()},
        {"success_rounds", success_rounds},
        {"failed_rounds", pairs.size() - success_rounds},
        {"solver_mode", solver_mode}
    };
    ofs << summary.dump() << "\n";

    std::cout << "\nSummary: " << success_rounds << "/" << pairs.size() << " success\n"
              << "Results saved to: \"" << output << "\"\n";

    rclcpp::shutdown();
    return success_rounds == pairs.size() ? 0 : 2;
}
