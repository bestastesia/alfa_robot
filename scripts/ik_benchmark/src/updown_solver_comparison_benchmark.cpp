#include "ik_benchmark/ik_solver.h"
#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

using ik_benchmark::IkResult;
using ik_benchmark::IkSolver;
using ik_benchmark::IkSolverOptions;
using ik_benchmark::ParallelUpdownAwareIkSolver;
using ik_benchmark::UpdownAwareIkConfig;
using ik_benchmark::UpdownAwareIkRequest;

namespace {

struct PoseSpec {
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
};

struct PickPoint { PoseSpec left; PoseSpec right; };
struct Stage { std::string name; PoseSpec left; PoseSpec right; };

Eigen::Isometry3d toIsometry(const PoseSpec& pose)
{
    Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
    q.normalize();
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    tf.linear() = q.toRotationMatrix();
    return tf;
}

nlohmann::json poseJson(const PoseSpec& pose)
{
    return {{"position", {pose.x, pose.y, pose.z}}, {"orientation", {pose.qx, pose.qy, pose.qz, pose.qw}}};
}

std::vector<PickPoint> pickPoints()
{
    const double qx = 0.0, qy = 0.7071, qz = 0.0, qw = 0.7071;
    return {
        {{0.6, 0.5, 1.5, qx, qy, qz, qw}, {0.6, -0.5, 1.5, qx, qy, qz, qw}},
        {{0.6, 0.3, 1.5, qx, qy, qz, qw}, {0.6, -0.3, 1.5, qx, qy, qz, qw}},
        {{0.6, 0.0, 1.5, qx, qy, qz, qw}, {0.6, 0.0, 1.1, qx, qy, qz, qw}},
    };
}

std::vector<Stage> makeStages(size_t round, const PickPoint& point, double approach_offset, double place_safe_z)
{
    const std::string prefix = "round_" + std::to_string(round + 1);
    return {
        {prefix + "/safe",
         {0.3, 0.3, point.left.z, point.left.qx, point.left.qy, point.left.qz, point.left.qw},
         {0.3, -0.3, point.right.z, point.right.qx, point.right.qy, point.right.qz, point.right.qw}},
        {prefix + "/approach",
         {point.left.x - approach_offset, point.left.y, point.left.z, point.left.qx, point.left.qy, point.left.qz, point.left.qw},
         {point.right.x - approach_offset, point.right.y, point.right.z, point.right.qx, point.right.qy, point.right.qz, point.right.qw}},
        {prefix + "/grasp", point.left, point.right},
        {prefix + "/retreat",
         {point.left.x - approach_offset, point.left.y, point.left.z, point.left.qx, point.left.qy, point.left.qz, point.left.qw},
         {point.right.x - approach_offset, point.right.y, point.right.z, point.right.qx, point.right.qy, point.right.qz, point.right.qw}},
        {prefix + "/place_safe",
         {0.6, 0.2, place_safe_z, -0.5, 0.5, 0.5, 0.5},
         {0.6, -0.2, place_safe_z, -0.5, 0.5, 0.5, 0.5}},
    };
}

Eigen::Isometry3d compensateTool0(const PoseSpec& pose, double tool0_offset)
{
    return toIsometry(pose) * Eigen::Translation3d(0.0, 0.0, -tool0_offset);
}

double posError(const PoseSpec& target, const Eigen::Isometry3d& actual)
{
    return (toIsometry(target).translation() - actual.translation()).norm();
}

double oriError(const PoseSpec& target, const Eigen::Isometry3d& actual)
{
    Eigen::AngleAxisd aa(toIsometry(target).linear().transpose() * actual.linear());
    return aa.angle();
}

double extractUpdown(const std::vector<std::string>& names, const std::vector<double>& values, double fallback)
{
    for (size_t i = 0; i < names.size() && i < values.size(); ++i) {
        if (names[i] == "updown") return values[i];
    }
    return fallback;
}

std::vector<double> perturbSeed(const std::vector<std::string>& names,
                                std::vector<double> seed,
                                size_t attempt,
                                double revolute_noise,
                                double h_noise,
                                double h_lower,
                                double h_upper)
{
    if (seed.size() != names.size()) seed.assign(names.size(), 0.0);
    std::mt19937 rng(static_cast<uint32_t>(0xBAD5EEDu + attempt * 7919u));
    std::normal_distribution<double> revolute(0.0, revolute_noise);
    std::normal_distribution<double> prismatic(0.0, h_noise);
    for (size_t i = 0; i < seed.size(); ++i) {
        if (names[i] == "updown") seed[i] = std::min(std::max(seed[i] + prismatic(rng), h_lower), h_upper);
        else if (names[i].find("pitch") == std::string::npos) seed[i] += revolute(rng);
    }
    return seed;
}

std::vector<double> armSeedFromFull(const std::vector<std::string>& arm_names,
                                    const IkResult& result)
{
    std::unordered_map<std::string, double> values;
    for (size_t i = 0; i < result.joint_names.size() && i < result.joint_values.size(); ++i) {
        values[result.joint_names[i]] = result.joint_values[i];
    }
    std::vector<double> seed(arm_names.size(), 0.0);
    for (size_t i = 0; i < arm_names.size(); ++i) {
        if (auto it = values.find(arm_names[i]); it != values.end()) seed[i] = it->second;
    }
    return seed;
}

bool tipBindingOk(const Stage& stage, const std::vector<Eigen::Isometry3d>& poses, double pos_tol, double ori_tol, nlohmann::json& record)
{
    if (poses.size() < 2) {
        record["rejection_reason"] = "fk_missing_dual_tip";
        return false;
    }
    const double direct_pos = std::max(posError(stage.left, poses[0]), posError(stage.right, poses[1]));
    const double swapped_pos = std::max(posError(stage.left, poses[1]), posError(stage.right, poses[0]));
    const double direct_ori = std::max(oriError(stage.left, poses[0]), oriError(stage.right, poses[1]));
    record["direct_pos_error"] = direct_pos;
    record["direct_ori_error"] = direct_ori;
    record["swapped_pos_error"] = swapped_pos;
    if (swapped_pos + 1e-4 < direct_pos) {
        record["rejection_reason"] = "tip_order_error";
        return false;
    }
    if (direct_pos > pos_tol || direct_ori > ori_tol) {
        record["rejection_reason"] = "tip_error_too_large";
        return false;
    }
    return true;
}

nlohmann::json runUnlimitedStage(IkSolver& ik,
                                 const Stage& stage,
                                 double current_h,
                                 std::vector<double>& full_seed,
                                 double timeout,
                                 size_t seed_attempts,
                                 double seed_noise,
                                 double h_step,
                                 double h_lower,
                                 double h_upper,
                                 double tool0_offset,
                                 double pos_tol,
                                 double ori_tol)
{
    nlohmann::json record;
    record["strategy"] = "unlimited_bioik_until_collision_free";
    record["current_h"] = current_h;
    record["attempts"] = nlohmann::json::array();
    bool success = false;
    IkResult selected;
    double selected_h = current_h;
    std::vector<double> selected_seed = full_seed;

    for (size_t attempt = 0; attempt < seed_attempts && !success; ++attempt) {
        auto seed = attempt == 0 ? full_seed : perturbSeed(ik.variableNames(), full_seed, attempt, seed_noise, h_step, h_lower, h_upper);
        for (size_t order = 0; order < 2 && !success; ++order) {
            const bool swapped_order = order == 1;
            IkResult result = swapped_order
                ? ik.solveDual(compensateTool0(stage.right, tool0_offset), compensateTool0(stage.left, tool0_offset), seed, timeout)
                : ik.solveDual(compensateTool0(stage.left, tool0_offset), compensateTool0(stage.right, tool0_offset), seed, timeout);
            nlohmann::json attempt_json;
            attempt_json["attempt"] = attempt;
            attempt_json["target_order"] = swapped_order ? "swapped" : "normal";
            attempt_json["success_raw"] = result.success;
            attempt_json["solve_ms"] = result.solve_ms;
            attempt_json["timeout_like"] = result.solve_ms >= timeout * 1000.0 * 0.9;
            attempt_json["joint_names"] = result.joint_names;
            bool ok = false;
            if (!result.joint_values.empty()) {
                auto poses = ik.fk(result.joint_values);
                ok = tipBindingOk(stage, poses, pos_tol, ori_tol, attempt_json);
                std::vector<std::string> collision_pairs;
                const bool collision_free = ik.isNamedStateCollisionFree(result.joint_names, result.joint_values, &collision_pairs);
                attempt_json["collision_free"] = collision_free;
                attempt_json["collision_pairs"] = collision_pairs;
                ok = ok && collision_free;
            } else {
                attempt_json["rejection_reason"] = "ik_failed_empty_solution";
            }
            record["attempts"].push_back(attempt_json);
            if (ok) {
                selected = result;
                selected_seed = seed;
                selected_h = extractUpdown(result.joint_names, result.joint_values, current_h);
                success = true;
            }
        }
    }

    record["success"] = success;
    record["selected_h"] = selected_h;
    record["updown_delta"] = std::abs(selected_h - current_h);
    record["attempt_count"] = record["attempts"].size();
    record["total_solve_ms"] = 0.0;
    for (const auto& a : record["attempts"]) record["total_solve_ms"] = record["total_solve_ms"].get<double>() + a.value("solve_ms", 0.0);
    if (success) {
        full_seed = selected_seed;
    }
    return record;
}

nlohmann::json runLookupStage(IkSolver& arm_ik,
                              IkSolver& fallback_ik,
                              ParallelUpdownAwareIkSolver& solver,
                              const Stage& stage,
                              double current_h,
                              std::vector<double>& arm_seed,
                              std::vector<double>& fallback_seed,
                              size_t seed_attempts,
                              size_t fallback_attempts)
{
    UpdownAwareIkRequest request;
    request.left_target = toIsometry(stage.left);
    request.right_target = toIsometry(stage.right);
    request.current_h = current_h;
    request.current_arm_joints = arm_seed;
    request.current_full_joints = fallback_seed;
    auto result = solver.solve(request);

    nlohmann::json record;
    record["strategy"] = "lookup_like_fixed_h_with_fallback";
    record["success"] = result.success;
    record["fallback_used"] = result.fallback_used;
    record["solver_path"] = result.solver_path;
    record["current_h"] = current_h;
    record["selected_h"] = result.success ? result.selected.h : current_h;
    record["updown_delta"] = result.success ? std::abs(result.selected.h - current_h) : 0.0;
    record["h_interval"] = {{"lower", result.h_interval_lower}, {"upper", result.h_interval_upper}, {"reachable", result.range_reachable}};
    record["h_candidates"] = result.h_candidates;
    record["trial_count"] = result.trial_count;
    record["legal_count"] = result.legal_count;
    record["timeout_like_count"] = result.timeout_like_count;
    record["wall_ms"] = result.wall_ms;
    record["sum_solve_ms"] = result.sum_solve_ms;
    if (result.success) {
        arm_seed = armSeedFromFull(arm_ik.variableNames(), {true, true, result.selected.collision_free, 0, result.selected.joint_names, result.selected.joint_values, result.selected.collision_pairs, result.selected.solve_ms, result.selected.direct_pos_error, result.selected.direct_ori_error});
        fallback_seed = result.selected.full_joint_values;
        record["selected_score"] = result.selected.score;
        record["direct_pos_error"] = result.selected.direct_pos_error;
        record["direct_ori_error"] = result.selected.direct_ori_error;
        record["collision_free"] = result.selected.collision_free;
        record["collision_pairs"] = result.selected.collision_pairs;
    } else {
        record["failure_reason"] = result.failure_reason;
    }
    (void)fallback_ik;
    (void)seed_attempts;
    (void)fallback_attempts;
    return record;
}

nlohmann::json runNewSolverStage(ParallelUpdownAwareIkSolver& solver,
                                 const Stage& stage,
                                 double current_h,
                                 std::vector<double>& arm_seed,
                                 std::vector<double>& full_seed)
{
    UpdownAwareIkRequest request;
    request.left_target = toIsometry(stage.left);
    request.right_target = toIsometry(stage.right);
    request.current_h = current_h;
    request.current_arm_joints = arm_seed;
    request.current_full_joints = full_seed;
    auto result = solver.solve(request);

    nlohmann::json record;
    record["strategy"] = "parallel_updown_aware_solver";
    record["success"] = result.success;
    record["fallback_used"] = result.fallback_used;
    record["solver_path"] = result.solver_path;
    record["current_h"] = current_h;
    record["selected_h"] = result.success ? result.selected.h : current_h;
    record["updown_delta"] = result.success ? std::abs(result.selected.h - current_h) : 0.0;
    record["h_interval"] = {{"lower", result.h_interval_lower}, {"upper", result.h_interval_upper}, {"reachable", result.range_reachable}};
    record["h_candidates"] = result.h_candidates;
    record["trial_count"] = result.trial_count;
    record["legal_count"] = result.legal_count;
    record["timeout_like_count"] = result.timeout_like_count;
    record["wall_ms"] = result.wall_ms;
    record["sum_solve_ms"] = result.sum_solve_ms;
    if (result.success) {
        arm_seed = result.selected.joint_values;
        full_seed = result.selected.full_joint_values;
        record["selected_score"] = result.selected.score;
        record["direct_pos_error"] = result.selected.direct_pos_error;
        record["direct_ori_error"] = result.selected.direct_ori_error;
        record["collision_free"] = result.selected.collision_free;
        record["collision_pairs"] = result.selected.collision_pairs;
    } else {
        record["failure_reason"] = result.failure_reason;
    }
    return record;
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::filesystem::path output = std::filesystem::path("/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/updown_solver_comparison.jsonl");
    size_t rounds = 3;
    double timeout = 0.5;
    size_t seed_attempts = 12;
    double approach_offset = 0.1;
    double place_safe_z = 0.85;
    double tool0_offset = 0.1;
    size_t workers = 8;
    size_t max_stages = 0;
    double fallback_timeout = 2.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--output" || arg == "--jsonl") && i + 1 < argc) output = argv[++i];
        else if (arg == "--rounds" && i + 1 < argc) rounds = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--timeout" && i + 1 < argc) timeout = std::stod(argv[++i]);
        else if (arg == "--seed-attempts" && i + 1 < argc) seed_attempts = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--workers" && i + 1 < argc) workers = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--max-stages" && i + 1 < argc) max_stages = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--fallback-timeout" && i + 1 < argc) fallback_timeout = std::stod(argv[++i]);
    }

    std::filesystem::create_directories(output.parent_path());
    std::ofstream ofs(output);
    if (!ofs.good()) {
        std::cerr << "Cannot write: " << output << "\n";
        rclcpp::shutdown();
        return 1;
    }

    IkSolverOptions options;
    options.base_frame = "base_link";
    options.tip_link = "left_v5_tool0";
    options.tip_link2 = "right_v5_tool0";
    options.reject_collisions = false;

    IkSolver unlimited_ik("dual_v5_arm_with_base", "bio_ik/BioIKKinematicsPlugin", timeout, false, options);
    IkSolver lookup_arm_ik("dual_v5_arm", "bio_ik/BioIKKinematicsPlugin", timeout, false, options);
    IkSolver lookup_fallback_ik("dual_v5_arm_with_base", "bio_ik/BioIKKinematicsPlugin", fallback_timeout, false, options);

    UpdownAwareIkConfig lookup_config;
    lookup_config.workers = 1;
    lookup_config.timeout = timeout;
    lookup_config.seed_count = 8;
    lookup_config.h_candidate_count = 15;
    lookup_config.h_step = 0.1;
    lookup_config.tool0_offset = tool0_offset;
    lookup_config.check_collision = true;
    lookup_config.fallback_enabled = true;
    lookup_config.fallback_timeout = fallback_timeout;
    lookup_config.fallback_seed_count = 12;

    UpdownAwareIkConfig experiment_config = lookup_config;
    experiment_config.workers = workers;
    experiment_config.seed_count = 4;
    experiment_config.h_candidate_count = 5;
    experiment_config.check_collision = true;
    experiment_config.fallback_seed_count = 12;

    ParallelUpdownAwareIkSolver lookup_solver(lookup_config);
    ParallelUpdownAwareIkSolver experiment_solver(experiment_config);

    nlohmann::json header;
    header["type"] = "header";
    header["benchmark"] = "updown_solver_comparison";
    header["output"] = output.string();
    header["rounds"] = rounds;
    header["timeout"] = timeout;
    header["seed_attempts"] = seed_attempts;
    header["experiment_workers"] = workers;
    header["max_stages"] = max_stages;
    header["fallback_timeout"] = fallback_timeout;
    header["place_safe_z"] = place_safe_z;
    ofs << header.dump() << "\n";

    const auto points = pickPoints();
    const size_t n_rounds = std::min(rounds, points.size());
    std::map<std::string, double> total_updown;
    std::map<std::string, double> total_wall;
    std::map<std::string, size_t> success_count;
    std::map<std::string, size_t> stage_count;

    std::vector<double> unlimited_seed(unlimited_ik.variableNames().size(), 0.0);
    std::vector<double> lookup_arm_seed(lookup_arm_ik.variableNames().size(), 0.0);
    std::vector<double> lookup_full_seed(lookup_fallback_ik.variableNames().size(), 0.0);
    std::vector<double> exp_arm_seed(experiment_solver.fixedVariableNames().size(), 0.0);
    std::vector<double> exp_full_seed(experiment_solver.freeVariableNames().size(), 0.0);
    double unlimited_h = 0.0, lookup_h = 0.0, exp_h = 0.0;

    size_t emitted_stages = 0;
    for (size_t round = 0; round < n_rounds; ++round) {
        for (const auto& stage : makeStages(round, points[round], approach_offset, place_safe_z)) {
            if (max_stages > 0 && emitted_stages >= max_stages) {
                break;
            }
            ++emitted_stages;
            std::vector<nlohmann::json> records;
            records.push_back(runUnlimitedStage(unlimited_ik, stage, unlimited_h, unlimited_seed,
                                                timeout, seed_attempts, 0.35, 0.1, 0.0, 0.99,
                                                tool0_offset, 0.02, 0.05));
            records.push_back(runLookupStage(lookup_arm_ik, lookup_fallback_ik, lookup_solver, stage, lookup_h,
                                             lookup_arm_seed, lookup_full_seed, 8, 12));
            records.push_back(runNewSolverStage(experiment_solver, stage, exp_h, exp_arm_seed, exp_full_seed));

            for (auto& record : records) {
                const std::string strategy = record["strategy"];
                record["type"] = "stage";
                record["round"] = round;
                record["stage"] = stage.name;
                record["target_pose"] = poseJson(stage.left);
                record["target_pose2"] = poseJson(stage.right);
                if (record.value("success", false)) {
                    success_count[strategy]++;
                    if (strategy == "unlimited_bioik_until_collision_free") unlimited_h = record.value("selected_h", unlimited_h);
                    if (strategy == "lookup_like_fixed_h_with_fallback") lookup_h = record.value("selected_h", lookup_h);
                    if (strategy == "parallel_updown_aware_solver") exp_h = record.value("selected_h", exp_h);
                }
                stage_count[strategy]++;
                total_updown[strategy] += record.value("updown_delta", 0.0);
                total_wall[strategy] += record.value("wall_ms", record.value("total_solve_ms", 0.0));
                ofs << record.dump() << "\n";
            }
        }
        if (max_stages > 0 && emitted_stages >= max_stages) {
            break;
        }
    }

    for (const auto& [strategy, count] : stage_count) {
        nlohmann::json summary;
        summary["type"] = "summary";
        summary["strategy"] = strategy;
        summary["stage_count"] = count;
        summary["success_count"] = success_count[strategy];
        summary["total_updown_motion"] = total_updown[strategy];
        summary["total_wall_ms"] = total_wall[strategy];
        ofs << summary.dump() << "\n";
    }

    std::cout << "Results saved to: " << output << "\n";
    rclcpp::shutdown();
    return 0;
}
