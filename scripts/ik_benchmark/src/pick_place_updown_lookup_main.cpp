#include "ik_benchmark/ik_solver.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <nlohmann/json.hpp>
#include <map>
#include <string>
#include <utility>
#include <vector>

using ik_benchmark::IkResult;
using ik_benchmark::IkSolver;
using ik_benchmark::IkSolverOptions;

namespace {

struct PoseSpec {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

struct PickPoint {
    PoseSpec left;
    PoseSpec right;
};

struct Stage {
    std::string name;
    PoseSpec left;
    PoseSpec right;
};

struct ReachSphere {
    double cx = 0.015;
    double cy = 0.3125;
    double cz = 0.6625;
    double radius = 0.815;
};

struct HeightInterval {
    bool reachable = false;
    double lower = 0.0;
    double upper = 0.0;
};

struct HeightPlan {
    bool reachable = false;
    HeightInterval left;
    HeightInterval right;
    HeightInterval combined;
    std::vector<double> candidates;
};

struct LegalSolution {
    IkResult result;
    std::vector<double> seed;
    std::vector<double> full_values;
    nlohmann::json attempt_record;
    double h = 0.0;
    size_t h_index = 0;
    size_t attempt = 0;
    double score = 0.0;
};

struct TrialResult {
    IkResult result;
    nlohmann::json attempt_record;
    std::vector<double> full_values;
    bool swapped_better = false;
};

struct DualTipMatch {
    bool has_dual = false;
    double direct_pos_error = 0.0;
    double swapped_pos_error = 0.0;
    double direct_ori_error = 0.0;
    double swapped_ori_error = 0.0;
    bool swapped_is_better = false;
};

Eigen::Isometry3d toIsometry(const PoseSpec& pose)
{
    Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
    q.normalize();

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    tf.linear() = q.toRotationMatrix();
    return tf;
}

Eigen::Isometry3d compensateTool0OffsetForIk(const PoseSpec& desired_tool0_pose, double tool0_offset)
{
    Eigen::Isometry3d tf = toIsometry(desired_tool0_pose);
    if (std::abs(tool0_offset) > 1e-12) {
        tf = tf * Eigen::Translation3d(0.0, 0.0, -tool0_offset);
    }
    return tf;
}

Eigen::Isometry3d targetInFixedUpdownFrame(const PoseSpec& desired_tool0_pose,
                                           double selected_h,
                                           double tool0_offset)
{
    Eigen::Isometry3d target = compensateTool0OffsetForIk(desired_tool0_pose, tool0_offset);
    target.translation().z() -= selected_h;
    return target;
}

double positionError(const PoseSpec& target, const Eigen::Isometry3d& actual)
{
    return (Eigen::Vector3d(target.x, target.y, target.z) - actual.translation()).norm();
}

double orientationError(const PoseSpec& target, const Eigen::Isometry3d& actual)
{
    Eigen::Quaterniond q(target.qw, target.qx, target.qy, target.qz);
    q.normalize();
    Eigen::AngleAxisd aa(q.toRotationMatrix().transpose() * actual.linear());
    return aa.angle();
}

DualTipMatch dualTipMatch(const PoseSpec& left_target,
                          const PoseSpec& right_target,
                          const std::vector<Eigen::Isometry3d>& actual_poses)
{
    DualTipMatch match;
    if (actual_poses.size() < 2) {
        return match;
    }

    match.has_dual = true;
    match.direct_pos_error = std::max(positionError(left_target, actual_poses[0]),
                                      positionError(right_target, actual_poses[1]));
    match.swapped_pos_error = std::max(positionError(left_target, actual_poses[1]),
                                       positionError(right_target, actual_poses[0]));
    match.direct_ori_error = std::max(orientationError(left_target, actual_poses[0]),
                                      orientationError(right_target, actual_poses[1]));
    match.swapped_ori_error = std::max(orientationError(left_target, actual_poses[1]),
                                       orientationError(right_target, actual_poses[0]));
    match.swapped_is_better = match.swapped_pos_error + 1e-4 < match.direct_pos_error;
    return match;
}

nlohmann::json poseToJson(const PoseSpec& pose)
{
    return {
        {"position", {pose.x, pose.y, pose.z}},
        {"orientation", {pose.qx, pose.qy, pose.qz, pose.qw}}
    };
}

nlohmann::json poseToJson(const Eigen::Isometry3d& pose)
{
    Eigen::Quaterniond q(pose.linear());
    return {
        {"position", {pose.translation().x(), pose.translation().y(), pose.translation().z()}},
        {"orientation", {q.x(), q.y(), q.z(), q.w()}}
    };
}

nlohmann::json dualTipMatchToJson(const DualTipMatch& match)
{
    return {
        {"has_dual", match.has_dual},
        {"direct_pos_error", match.direct_pos_error},
        {"swapped_pos_error", match.swapped_pos_error},
        {"direct_ori_error", match.direct_ori_error},
        {"swapped_ori_error", match.swapped_ori_error},
        {"swapped_is_better", match.swapped_is_better}
    };
}

nlohmann::json resultToJson(const IkResult& result)
{
    return {
        {"success", result.success},
        {"collision_checked", result.collision_checked},
        {"collision_free", result.collision_free},
        {"collision_rejection_count", result.collision_rejection_count},
        {"collision_pairs", result.collision_pairs},
        {"joint_names", result.joint_names},
        {"joint_values", result.joint_values},
        {"solve_ms", result.solve_ms},
        {"pos_error", result.pos_error},
        {"ori_error", result.ori_error}
    };
}

nlohmann::json intervalToJson(const HeightInterval& interval)
{
    return {
        {"reachable", interval.reachable},
        {"lower", interval.lower},
        {"upper", interval.upper}
    };
}

HeightInterval intervalForTarget(const PoseSpec& target,
                                 const ReachSphere& sphere,
                                 double tool0_offset,
                                 double h_lower,
                                 double h_upper,
                                 double margin)
{
    const double radius = std::max(0.0, sphere.radius - margin);
    const double dx = target.x - sphere.cx;
    const double dy = target.y - sphere.cy;
    const double dxy2 = dx * dx + dy * dy;
    const double r2 = radius * radius;
    HeightInterval interval;
    if (dxy2 > r2) {
        return interval;
    }

    const double z_margin = std::sqrt(std::max(0.0, r2 - dxy2));
    const double ik_target_z = target.z - tool0_offset;
    interval.lower = ik_target_z - sphere.cz - z_margin;
    interval.upper = ik_target_z - sphere.cz + z_margin;
    interval.lower = std::max(interval.lower, h_lower);
    interval.upper = std::min(interval.upper, h_upper);
    interval.reachable = interval.lower <= interval.upper;
    return interval;
}

std::vector<double> makeHeightCandidates(const HeightInterval& interval,
                                         double current_h,
                                         double step,
                                         size_t max_candidates)
{
    std::vector<double> candidates;
    if (!interval.reachable) {
        return candidates;
    }

    auto add_unique = [&candidates, max_candidates](double value) {
        if (candidates.size() >= max_candidates) {
            return;
        }
        for (double existing : candidates) {
            if (std::abs(existing - value) < 1e-9) {
                return;
            }
        }
        candidates.push_back(value);
    };

    const double clamped_current = std::min(std::max(current_h, interval.lower), interval.upper);
    add_unique(clamped_current);

    if (step <= 0.0) {
        return candidates;
    }

    for (size_t ring = 1; candidates.size() < max_candidates; ++ring) {
        const double delta = step * static_cast<double>(ring);
        bool added = false;
        const double lower_value = clamped_current - delta;
        const double upper_value = clamped_current + delta;
        if (lower_value >= interval.lower - 1e-9) {
            add_unique(std::max(interval.lower, lower_value));
            added = true;
        }
        if (upper_value <= interval.upper + 1e-9) {
            add_unique(std::min(interval.upper, upper_value));
            added = true;
        }
        if (!added && lower_value < interval.lower && upper_value > interval.upper) {
            break;
        }
    }

    return candidates;
}

HeightPlan planHeights(const PoseSpec& left,
                       const PoseSpec& right,
                       const ReachSphere& left_sphere,
                       const ReachSphere& right_sphere,
                       double current_h,
                       double h_lower,
                       double h_upper,
                       double tool0_offset,
                       double margin,
                       double step,
                       size_t max_candidates)
{
    HeightPlan plan;
    plan.left = intervalForTarget(left, left_sphere, tool0_offset, h_lower, h_upper, margin);
    plan.right = intervalForTarget(right, right_sphere, tool0_offset, h_lower, h_upper, margin);
    plan.combined.lower = std::max(plan.left.lower, plan.right.lower);
    plan.combined.upper = std::min(plan.left.upper, plan.right.upper);
    plan.combined.reachable = plan.left.reachable && plan.right.reachable && plan.combined.lower <= plan.combined.upper;
    plan.reachable = plan.combined.reachable;
    plan.candidates = makeHeightCandidates(plan.combined, current_h, step, max_candidates);
    return plan;
}

std::vector<std::string> fullJointNamesForUpdownLookup(const std::vector<std::string>& arm_joint_names)
{
    std::vector<std::string> names;
    names.reserve(arm_joint_names.size() + 1);
    names.push_back("updown");
    for (const std::string& joint_name : arm_joint_names) {
        names.push_back(joint_name);
    }
    return names;
}

std::vector<double> fullJointValuesForUpdownLookup(double updown,
                                                   const std::vector<double>& arm_joint_values)
{
    std::vector<double> values;
    values.reserve(arm_joint_values.size() + 1);
    values.push_back(updown);
    for (double value : arm_joint_values) {
        values.push_back(value);
    }
    return values;
}

std::vector<double> makeFullSeedFromArmSeed(const std::vector<std::string>& full_variable_names,
                                            double updown,
                                            const std::vector<std::string>& arm_variable_names,
                                            const std::vector<double>& arm_seed)
{
    std::vector<double> full_seed(full_variable_names.size(), 0.0);
    for (size_t i = 0; i < full_variable_names.size(); ++i) {
        if (full_variable_names[i] == "updown") {
            full_seed[i] = updown;
            continue;
        }
        for (size_t j = 0; j < arm_variable_names.size() && j < arm_seed.size(); ++j) {
            if (full_variable_names[i] == arm_variable_names[j]) {
                full_seed[i] = arm_seed[j];
                break;
            }
        }
    }
    return full_seed;
}

std::vector<double> makeArmSeedFromFullResult(const std::vector<std::string>& arm_variable_names,
                                              const IkResult& full_result)
{
    std::vector<double> arm_seed(arm_variable_names.size(), 0.0);
    for (size_t i = 0; i < arm_variable_names.size(); ++i) {
        for (size_t j = 0; j < full_result.joint_names.size() && j < full_result.joint_values.size(); ++j) {
            if (arm_variable_names[i] == full_result.joint_names[j]) {
                arm_seed[i] = full_result.joint_values[j];
                break;
            }
        }
    }
    return arm_seed;
}

double updownFromResult(const IkResult& result, double fallback)
{
    for (size_t i = 0; i < result.joint_names.size() && i < result.joint_values.size(); ++i) {
        if (result.joint_names[i] == "updown") {
            return result.joint_values[i];
        }
    }
    return fallback;
}

std::string localStageName(const std::string& stage_name)
{
    const size_t slash = stage_name.find('/');
    return slash == std::string::npos ? stage_name : stage_name.substr(slash + 1);
}

void addUniqueSeed(std::vector<std::vector<double>>& seeds, const std::vector<double>& seed)
{
    if (seed.empty()) {
        return;
    }
    for (const auto& existing : seeds) {
        if (existing.size() != seed.size()) {
            continue;
        }
        double max_diff = 0.0;
        for (size_t i = 0; i < seed.size(); ++i) {
            max_diff = std::max(max_diff, std::abs(existing[i] - seed[i]));
        }
        if (max_diff < 1e-9) {
            return;
        }
    }
    seeds.push_back(seed);
}

nlohmann::json attemptToJson(size_t attempt,
                             const std::vector<double>& attempt_seed,
                             const IkResult& result);

TrialResult runFixedHeightTrial(IkSolver& ik,
                                const std::vector<std::string>& full_joint_names,
                                const Stage& stage,
                                const Eigen::Isometry3d& left_target,
                                const Eigen::Isometry3d& right_target,
                                const std::vector<double>& attempt_seed,
                                size_t attempt,
                                size_t h_index,
                                double candidate_h,
                                double timeout,
                                bool reject_collisions,
                                const std::string& target_order)
{
    const bool swapped_order = target_order == "swapped";
    IkResult result = swapped_order
        ? ik.solveDual(right_target, left_target, attempt_seed, timeout)
        : ik.solveDual(left_target, right_target, attempt_seed, timeout);

    nlohmann::json attempt_record = attemptToJson(attempt, attempt_seed, result);
    attempt_record["h_candidate_index"] = h_index;
    attempt_record["h"] = candidate_h;
    attempt_record["target_order"] = target_order;
    attempt_record["fixed_updown_ik_target_pose"] = poseToJson(left_target);
    attempt_record["fixed_updown_ik_target_pose2"] = poseToJson(right_target);

    TrialResult trial;
    trial.result = result;
    trial.attempt_record = attempt_record;

    if (!trial.result.joint_values.empty()) {
        trial.full_values = fullJointValuesForUpdownLookup(candidate_h, trial.result.joint_values);
        std::vector<std::string> full_collision_pairs;
        const bool full_collision_free = ik.isNamedStateCollisionFree(
            full_joint_names, trial.full_values, &full_collision_pairs);
        trial.result.collision_checked = true;
        trial.result.collision_free = full_collision_free;
        trial.result.collision_pairs = full_collision_pairs;
        trial.attempt_record["full_state_collision_free"] = full_collision_free;
        if (reject_collisions && !full_collision_free) {
            trial.result.success = false;
            trial.attempt_record["rejection_reason"] = "full_state_collision";
        }

        const auto actual_poses = ik.fkNamed(full_joint_names, trial.full_values);
        const DualTipMatch tip_match = dualTipMatch(stage.left, stage.right, actual_poses);
        trial.swapped_better = tip_match.swapped_is_better;
        trial.attempt_record["tip_target_match"] = dualTipMatchToJson(tip_match);
        if (!actual_poses.empty()) {
            trial.attempt_record["actual_pose"] = poseToJson(actual_poses[0]);
        }
        if (actual_poses.size() > 1) {
            trial.attempt_record["actual_pose2"] = poseToJson(actual_poses[1]);
        }
        if (trial.result.success && tip_match.swapped_is_better) {
            trial.result.success = false;
            trial.attempt_record["rejection_reason"] = "dual_tip_target_swapped";
        }
    }

    trial.attempt_record["result_after_full_state_check"] = resultToJson(trial.result);
    return trial;
}

nlohmann::json attemptToJson(size_t attempt,
                             const std::vector<double>& attempt_seed,
                             const IkResult& result)
{
    nlohmann::json record = resultToJson(result);
    record["attempt"] = attempt;
    record["seed"] = attempt_seed;
    return record;
}

std::vector<double> updateSeedFromResult(const std::vector<std::string>& variable_names,
                                         const std::vector<double>& previous_seed,
                                         const IkResult& result)
{
    std::vector<double> next_seed = previous_seed;
    if (next_seed.size() != variable_names.size()) {
        next_seed.assign(variable_names.size(), 0.0);
    }

    for (size_t joint_i = 0; joint_i < result.joint_names.size(); ++joint_i) {
        for (size_t var_i = 0; var_i < variable_names.size(); ++var_i) {
            if (variable_names[var_i] == result.joint_names[joint_i]) {
                next_seed[var_i] = result.joint_values[joint_i];
                break;
            }
        }
    }
    return next_seed;
}

std::vector<double> makePerturbedSeed(const std::vector<std::string>& variable_names,
                                      const std::vector<double>& base_seed,
                                      size_t attempt_index,
                                      double revolute_noise,
                                      double prismatic_noise)
{
    std::vector<double> seed = base_seed;
    if (seed.size() != variable_names.size()) {
        seed.assign(variable_names.size(), 0.0);
    }

    std::mt19937 rng(static_cast<uint32_t>(0xA17A0000u + attempt_index * 2654435761u));
    for (size_t i = 0; i < seed.size(); ++i) {
        const bool is_prismatic = (variable_names[i] == "updown");
        const double sigma = is_prismatic ? prismatic_noise : revolute_noise;
        if (sigma <= 0.0) {
            continue;
        }
        std::normal_distribution<double> dist(0.0, sigma);
        seed[i] += dist(rng);
        if (is_prismatic) {
            seed[i] = std::max(0.0, seed[i]);
        }
    }
    return seed;
}

std::vector<Stage> makeRoundStages(size_t round_index, const PickPoint& point, double approach_offset)
{
    const PoseSpec& left_pick = point.left;
    const PoseSpec& right_pick = point.right;

    Stage safe;
    safe.name = "safe";
    safe.left = {0.3, 0.3, left_pick.z, left_pick.qx, left_pick.qy, left_pick.qz, left_pick.qw};
    safe.right = {0.3, -0.3, right_pick.z, right_pick.qx, right_pick.qy, right_pick.qz, right_pick.qw};

    Stage approach;
    approach.name = "approach";
    approach.left = {left_pick.x - approach_offset, left_pick.y, left_pick.z,
                     left_pick.qx, left_pick.qy, left_pick.qz, left_pick.qw};
    approach.right = {right_pick.x - approach_offset, right_pick.y, right_pick.z,
                      right_pick.qx, right_pick.qy, right_pick.qz, right_pick.qw};

    Stage grasp;
    grasp.name = "grasp";
    grasp.left = left_pick;
    grasp.right = right_pick;

    Stage retreat = approach;
    retreat.name = "retreat";

    Stage place_safe;
    place_safe.name = "place_safe";
    place_safe.left = {0.6, 0.2, 0.6, left_pick.qx, left_pick.qy, left_pick.qz, left_pick.qw};
    place_safe.right = {0.6, -0.2, 0.6, right_pick.qx, right_pick.qy, right_pick.qz, right_pick.qw};

    for (Stage* stage : {&safe, &approach, &grasp, &retreat, &place_safe}) {
        stage->name = "round_" + std::to_string(round_index + 1) + "/" + stage->name;
    }

    return {safe, approach, grasp, retreat, place_safe};
}

void printHelp()
{
    std::cout << "Usage: pick_place_updown_lookup [options]\n"
              << "  --group <name>              MoveIt arm-only group (default: dual_arm)\n"
              << "  --solver <plugin>           IK solver plugin (default: bio_ik/BioIKKinematicsPlugin)\n"
              << "  --timeout <s>               IK timeout per IK attempt (default: 2.0)\n"
              << "  --start <index>             Start round, 0-based (default: 0)\n"
              << "  --rounds <n>                Number of rounds (default: all active PICK_POINTS)\n"
              << "  --approach-offset <m>       Pick approach offset along -x (default: 0.1)\n"
              << "  --tool0-offset <m>          Fixed local +Z offset from link6 to tool0; compensated before IK (default: 0.1)\n"
              << "  --seed-attempts <n>         Seed attempts per h candidate (default: 12)\n"
              << "  --seed-noise <rad>          Revolute joint seed perturbation stddev (default: 0.6)\n"
              << "  --h-lower <m>               Updown lower limit (default: 0.0)\n"
              << "  --h-upper <m>               Updown upper limit (default: 0.99)\n"
              << "  --h-step <m>                Candidate spacing around nearest h (default: 0.02)\n"
              << "  --h-candidates <n>          Max h candidates per stage (default: 15)\n"
              << "  --solution-candidates <n>   Number of legal IK solutions to collect before selecting (default: 5)\n"
              << "  --sphere-margin <m>         Shrink reachability sphere radius (default: 0.0)\n"
              << "  --fallback-timeout <s>      Timeout per baseline fallback IK attempt (default: max(timeout, 2.0))\n"
              << "  --fallback-seed-attempts <n> Seed attempts for baseline fallback (default: max(seed-attempts, 12))\n"
              << "  --no-baseline-fallback      Disable dual_arm_with_base fallback when lookup fails\n"
              << "  --allow-collision-solutions Keep full-state collision solutions (diagnostic only)\n"
              << "  --output <path>             JSONL output path (default: /tmp/pick_place_updown_lookup.jsonl)\n";
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string group = "dual_arm";
    std::string solver = "bio_ik/BioIKKinematicsPlugin";
    std::string output = "/tmp/pick_place_updown_lookup.jsonl";
    double timeout = 2.0;
    double approach_offset = 0.1;
    double tool0_offset = 0.1;
    size_t seed_attempts = 12;
    double seed_noise = 0.6;
    bool reject_collisions = true;
    size_t start = 0;
    size_t rounds = 0;
    double h_lower = 0.0;
    double h_upper = 0.99;
    double h_step = 0.02;
    size_t h_candidate_limit = 15;
    size_t solution_candidate_limit = 5;
    double sphere_margin = 0.0;
    bool baseline_fallback = true;
    double fallback_timeout = -1.0;
    size_t fallback_seed_attempts = 0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--group" && i + 1 < argc) group = argv[++i];
        else if (arg == "--solver" && i + 1 < argc) solver = argv[++i];
        else if (arg == "--timeout" && i + 1 < argc) timeout = std::stod(argv[++i]);
        else if (arg == "--start" && i + 1 < argc) start = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--rounds" && i + 1 < argc) rounds = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--approach-offset" && i + 1 < argc) approach_offset = std::stod(argv[++i]);
        else if (arg == "--tool0-offset" && i + 1 < argc) tool0_offset = std::stod(argv[++i]);
        else if (arg == "--seed-attempts" && i + 1 < argc) seed_attempts = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--seed-noise" && i + 1 < argc) seed_noise = std::stod(argv[++i]);
        else if (arg == "--h-lower" && i + 1 < argc) h_lower = std::stod(argv[++i]);
        else if (arg == "--h-upper" && i + 1 < argc) h_upper = std::stod(argv[++i]);
        else if (arg == "--h-step" && i + 1 < argc) h_step = std::stod(argv[++i]);
        else if (arg == "--h-candidates" && i + 1 < argc) h_candidate_limit = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--solution-candidates" && i + 1 < argc) solution_candidate_limit = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--sphere-margin" && i + 1 < argc) sphere_margin = std::stod(argv[++i]);
        else if (arg == "--fallback-timeout" && i + 1 < argc) fallback_timeout = std::stod(argv[++i]);
        else if (arg == "--fallback-seed-attempts" && i + 1 < argc) fallback_seed_attempts = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--no-baseline-fallback") baseline_fallback = false;
        else if (arg == "--allow-collision-solutions") reject_collisions = false;
        else if ((arg == "--output" || arg == "--jsonl") && i + 1 < argc) output = argv[++i];
        else if (arg == "--help" || arg == "-h") {
            printHelp();
            rclcpp::shutdown();
            return 0;
        }
    }

    const double gqx = 0.0;
    const double gqy = 0.7071;
    const double gqz = 0.0;
    const double gqw = 0.7071;

    const std::vector<PickPoint> pick_points = {
        {{0.6, 0.5, 1.5, gqx, gqy, gqz, gqw}, {0.6, -0.5, 1.5, gqx, gqy, gqz, gqw}},
        {{0.6, 0.3, 1.5, gqx, gqy, gqz, gqw}, {0.6, -0.3, 1.5, gqx, gqy, gqz, gqw}},
        {{0.6, 0.0, 1.5, gqx, gqy, gqz, gqw}, {0.6, 0.0, 1.1, gqx, gqy, gqz, gqw}},
    };

    if (start >= pick_points.size()) {
        std::cerr << "--start is outside active PICK_POINTS size: " << pick_points.size() << "\n";
        rclcpp::shutdown();
        return 1;
    }
    if (rounds == 0 || start + rounds > pick_points.size()) {
        rounds = pick_points.size() - start;
    }
    if (fallback_timeout <= 0.0) {
        fallback_timeout = std::max(timeout, 2.0);
    }
    if (fallback_seed_attempts == 0) {
        fallback_seed_attempts = std::max<size_t>(seed_attempts, 12);
    }

    ReachSphere left_sphere;
    ReachSphere right_sphere;
    right_sphere.cy = -0.3125;

    IkSolverOptions options;
    options.base_frame = "base_link";
    options.tip_link = "left_tool0";
    options.tip_link2 = "right_tool0";
    options.reject_collisions = false;

    IkSolver ik(group, solver, timeout, false, options);
    if (!ik.isDualArm()) {
        std::cerr << "pick_place_updown_lookup requires a dual-arm group, got: " << group << "\n";
        rclcpp::shutdown();
        return 1;
    }

    IkSolver fallback_ik("dual_arm_with_base", solver, fallback_timeout, false, options);

    std::ofstream ofs(output);
    if (!ofs.good()) {
        std::cerr << "Cannot write to " << output << "\n";
        rclcpp::shutdown();
        return 1;
    }

    std::vector<double> seed(ik.variableNames().size(), 0.0);
    double current_h = std::min(std::max(0.0, h_lower), h_upper);
    size_t stage_index = 0;
    size_t successes = 0;
    double total_ms = 0.0;
    double total_updown_motion = 0.0;
    bool stop_after_failure = false;
    std::vector<double> last_successful_fallback_seed;
    std::map<std::string, std::vector<double>> last_successful_stage_seed;

    nlohmann::json header;
    header["header"] = true;
    header["benchmark"] = "pick_place_updown_lookup";
    header["group"] = group;
    header["solver"] = solver;
    header["tip_link"] = options.tip_link;
    header["tip_link2"] = options.tip_link2;
    header["base_frame"] = options.base_frame;
    header["start"] = start;
    header["rounds"] = rounds;
    header["approach_offset"] = approach_offset;
    header["tool0_offset_compensation"] = tool0_offset;
    header["seed_attempts_per_h"] = seed_attempts;
    header["seed_noise"] = seed_noise;
    header["h_lower"] = h_lower;
    header["h_upper"] = h_upper;
    header["h_step"] = h_step;
    header["h_candidate_limit"] = h_candidate_limit;
    header["solution_candidate_limit"] = solution_candidate_limit;
    header["sphere_margin"] = sphere_margin;
    header["baseline_fallback"] = baseline_fallback;
    header["fallback_timeout"] = fallback_timeout;
    header["fallback_seed_attempts"] = fallback_seed_attempts;
    header["left_reach_sphere"] = {{"center", {left_sphere.cx, left_sphere.cy, left_sphere.cz}}, {"radius", left_sphere.radius}};
    header["right_reach_sphere"] = {{"center", {right_sphere.cx, right_sphere.cy, right_sphere.cz}}, {"radius", right_sphere.radius}};
    header["flow"] = {"safe", "approach", "grasp", "retreat", "place_safe"};
    header["place_safe_left"] = poseToJson(PoseSpec{0.6, 0.2, 0.6, 0.0, 0.0, 0.0, 1.0});
    header["place_safe_right"] = poseToJson(PoseSpec{0.6, -0.2, 0.6, 0.0, 0.0, 0.0, 1.0});
    header["seed_policy"] = "for each h candidate: attempt 0 uses previous successful arm solution; later attempts use deterministic perturbed seeds";
    header["collision_policy"] = reject_collisions ? "full named robot state collisions are rejected after arm-only IK" : "full named robot state collisions are recorded but allowed";
    ofs << header.dump() << "\n";

    std::cout << "=== Pick Place Updown Lookup ===\n"
              << "  Group:   " << group << "\n"
              << "  Solver:  " << solver << "\n"
              << "  Timeout: " << timeout << "s per IK attempt\n"
              << "  Tool0 offset compensation: " << tool0_offset << "m\n"
              << "  Full-state collision reject: " << (reject_collisions ? "yes" : "no (diagnostic)") << "\n"
              << "  Baseline fallback: " << (baseline_fallback ? "yes" : "no") << "\n"
              << "  Fallback timeout/attempts: " << fallback_timeout << "s / " << fallback_seed_attempts << "\n"
              << "  Seed attempts per h: " << seed_attempts << "\n"
              << "  Legal solutions to collect: " << solution_candidate_limit << "\n"
              << "  h range/step/candidates: [" << h_lower << ", " << h_upper << "] / " << h_step << " / " << h_candidate_limit << "\n"
              << "  Rounds:  " << rounds << " from " << start << "\n"
              << "  Output:  " << output << "\n\n";

    const std::vector<std::string> full_joint_names = fullJointNamesForUpdownLookup(ik.variableNames());

    for (size_t round_i = start; round_i < start + rounds; ++round_i) {
        const auto stages = makeRoundStages(round_i, pick_points[round_i], approach_offset);
        for (const Stage& stage : stages) {
            nlohmann::json record;
            record["sample"] = stage_index;
            record["round"] = round_i;
            record["stage"] = stage.name;
            record["group"] = group;
            record["solver"] = solver;
            record["seed_joints"] = seed;
            record["seed_joint_names"] = ik.variableNames();
            record["current_updown"] = current_h;
            record["target_pose"] = poseToJson(stage.left);
            record["target_pose2"] = poseToJson(stage.right);
            record["world_ik_target_pose"] = poseToJson(compensateTool0OffsetForIk(stage.left, tool0_offset));
            record["world_ik_target_pose2"] = poseToJson(compensateTool0OffsetForIk(stage.right, tool0_offset));

            const HeightPlan height_plan = planHeights(
                stage.left, stage.right, left_sphere, right_sphere, current_h,
                h_lower, h_upper, tool0_offset, sphere_margin, h_step, h_candidate_limit);
            record["h_left_interval"] = intervalToJson(height_plan.left);
            record["h_right_interval"] = intervalToJson(height_plan.right);
            record["h_interval"] = intervalToJson(height_plan.combined);
            record["h_candidates"] = height_plan.candidates;
            record["h_reachable"] = height_plan.reachable;
            record["attempts"] = nlohmann::json::array();

            bool found_success = false;
            IkResult selected_result;
            std::vector<double> selected_seed = seed;
            std::vector<double> selected_full_values;
            double selected_h = current_h;
            size_t selected_h_index = 0;
            size_t selected_seed_attempt = 0;
            nlohmann::json selected_attempt_record;
            std::vector<LegalSolution> legal_solutions;

            std::cout << std::setw(2) << stage_index << "  " << stage.name << "  ";

            if (!height_plan.reachable || height_plan.candidates.empty()) {
                selected_result.success = false;
                selected_attempt_record["failure_reason"] = "reachability_interval_empty";
            } else {
                const size_t target_legal_count = std::max<size_t>(1, solution_candidate_limit);
                for (size_t h_index = 0; h_index < height_plan.candidates.size(); ++h_index) {
                    if (legal_solutions.size() >= target_legal_count) {
                        break;
                    }
                    const double candidate_h = height_plan.candidates[h_index];
                    const Eigen::Isometry3d left_target = targetInFixedUpdownFrame(stage.left, candidate_h, tool0_offset);
                    const Eigen::Isometry3d right_target = targetInFixedUpdownFrame(stage.right, candidate_h, tool0_offset);

                    for (size_t attempt = 0; attempt < std::max<size_t>(1, seed_attempts); ++attempt) {
                        if (legal_solutions.size() >= target_legal_count) {
                            break;
                        }
                        std::vector<double> attempt_seed = attempt == 0
                            ? seed
                            : makePerturbedSeed(ik.variableNames(), seed, attempt + h_index * seed_attempts, seed_noise, 0.0);

                        std::vector<TrialResult> trials;
                        trials.push_back(runFixedHeightTrial(
                            ik, full_joint_names, stage, left_target, right_target,
                            attempt_seed, attempt, h_index, candidate_h, timeout,
                            reject_collisions, "normal"));
                        if (trials.back().swapped_better) {
                            trials.push_back(runFixedHeightTrial(
                                ik, full_joint_names, stage, left_target, right_target,
                                attempt_seed, attempt, h_index, candidate_h, timeout,
                                reject_collisions, "swapped"));
                        }

                        for (TrialResult& trial : trials) {
                            record["attempts"].push_back(trial.attempt_record);

                            if (selected_attempt_record.is_null() || trial.result.collision_rejection_count < selected_result.collision_rejection_count) {
                                selected_result = trial.result;
                                selected_seed = attempt_seed;
                                selected_full_values = trial.full_values;
                                selected_h = candidate_h;
                                selected_h_index = h_index;
                                selected_seed_attempt = attempt;
                                selected_attempt_record = trial.attempt_record;
                            }

                            if (trial.result.success && trial.result.collision_free && !trial.swapped_better) {
                                const double h_delta = std::abs(candidate_h - current_h);
                                const double score = h_delta + 0.001 * trial.result.solve_ms + 0.01 * static_cast<double>(attempt);
                                LegalSolution legal;
                                legal.result = trial.result;
                                legal.seed = attempt_seed;
                                legal.full_values = trial.full_values;
                                legal.attempt_record = trial.attempt_record;
                                legal.h = candidate_h;
                                legal.h_index = h_index;
                                legal.attempt = attempt;
                                legal.score = score;
                                legal_solutions.push_back(legal);
                                record["attempts"].back()["legal_candidate"] = true;
                                record["attempts"].back()["selection_score"] = score;
                            }
                        }
                    }
                }
            }

            if (!legal_solutions.empty()) {
                auto best = std::min_element(
                    legal_solutions.begin(), legal_solutions.end(),
                    [](const LegalSolution& a, const LegalSolution& b) {
                        return a.score < b.score;
                    });
                found_success = true;
                selected_result = best->result;
                selected_seed = best->seed;
                selected_full_values = best->full_values;
                selected_h = best->h;
                selected_h_index = best->h_index;
                selected_seed_attempt = best->attempt;
                selected_attempt_record = best->attempt_record;
                selected_attempt_record["selected"] = true;
                selected_attempt_record["selection_score"] = best->score;
            }

            nlohmann::json legal_candidates_json = nlohmann::json::array();
            for (size_t i = 0; i < legal_solutions.size(); ++i) {
                legal_candidates_json.push_back({
                    {"index", i},
                    {"h", legal_solutions[i].h},
                    {"h_candidate_index", legal_solutions[i].h_index},
                    {"attempt", legal_solutions[i].attempt},
                    {"score", legal_solutions[i].score},
                    {"solve_ms", legal_solutions[i].result.solve_ms},
                    {"collision_free", legal_solutions[i].result.collision_free}
                });
            }
            record["legal_solution_count"] = legal_solutions.size();
            record["legal_candidates"] = legal_candidates_json;

            record["selected_h"] = selected_h;
            record["selected_h_index"] = selected_h_index;
            record["selected_attempt"] = selected_seed_attempt;
            record["selected_attempt_seed"] = selected_seed;
            record["selected_attempt_record"] = selected_attempt_record;
            record["updown_delta"] = std::abs(selected_h - current_h);
            record["result"] = resultToJson(selected_result);

            if (found_success) {
                auto actual_poses = ik.fkNamed(full_joint_names, selected_full_values);
                const DualTipMatch tip_match = dualTipMatch(stage.left, stage.right, actual_poses);
                record["tip_target_match"] = dualTipMatchToJson(tip_match);
                if (tip_match.swapped_is_better) {
                    selected_result.success = false;
                    nlohmann::json rejected_result = resultToJson(selected_result);
                    rejected_result["rejection_reason"] = "dual_tip_target_swapped";
                    rejected_result["tip_target_match"] = dualTipMatchToJson(tip_match);
                    record["result"] = rejected_result;
                    record["actual_pose"] = poseToJson(actual_poses[0]);
                    if (actual_poses.size() > 1) {
                        record["actual_pose2"] = poseToJson(actual_poses[1]);
                    }
                    record["next_seed_joints"] = seed;
                    record["next_updown"] = current_h;
                    stop_after_failure = true;
                    std::cout << "FAIL  swapped-tip target binding suspected"
                              << " direct=" << tip_match.direct_pos_error
                              << " swapped=" << tip_match.swapped_pos_error << "\n";
                    ofs << record.dump() << "\n";
                    stage_index++;
                    break;
                }

                successes++;
                total_ms += selected_result.solve_ms;
                total_updown_motion += std::abs(selected_h - current_h);
                record["actual_pose"] = poseToJson(actual_poses[0]);
                const double tool_pos_error = positionError(stage.left, actual_poses[0]);
                const double tool_ori_error = orientationError(stage.left, actual_poses[0]);
                double max_tool_pos_error = tool_pos_error;
                double max_tool_ori_error = tool_ori_error;
                record["tool_pos_error"] = tool_pos_error;
                record["tool_ori_error"] = tool_ori_error;
                if (actual_poses.size() > 1) {
                    record["actual_pose2"] = poseToJson(actual_poses[1]);
                    const double tool_pos_error2 = positionError(stage.right, actual_poses[1]);
                    const double tool_ori_error2 = orientationError(stage.right, actual_poses[1]);
                    max_tool_pos_error = std::max(max_tool_pos_error, tool_pos_error2);
                    max_tool_ori_error = std::max(max_tool_ori_error, tool_ori_error2);
                    record["tool_pos_error2"] = tool_pos_error2;
                    record["tool_ori_error2"] = tool_ori_error2;
                }

                nlohmann::json public_result = resultToJson(selected_result);
                public_result["joint_names"] = full_joint_names;
                public_result["joint_values"] = selected_full_values;
                public_result["pos_error"] = max_tool_pos_error;
                public_result["ori_error"] = max_tool_ori_error;
                record["result"] = public_result;

                seed = updateSeedFromResult(ik.variableNames(), seed, selected_result);
                current_h = selected_h;
                last_successful_stage_seed[localStageName(stage.name)] = selected_full_values;
                record["next_seed_joints"] = seed;
                record["next_updown"] = current_h;

                std::cout << "OK    " << selected_result.solve_ms << "ms  "
                          << "h=" << selected_h << "  "
                          << "h_idx=" << selected_h_index << "  "
                          << "attempt=" << selected_seed_attempt << "  "
                          << "tool_pos_err=" << max_tool_pos_error << "  "
                          << "tool_ori_err=" << max_tool_ori_error << "\n";
            } else {
                nlohmann::json failed_result = resultToJson(selected_result);
                if (!height_plan.reachable || height_plan.candidates.empty()) {
                    failed_result["rejection_reason"] = "reachability_interval_empty";
                } else if (!selected_attempt_record.is_null() && selected_attempt_record.contains("rejection_reason")) {
                    failed_result["rejection_reason"] = selected_attempt_record["rejection_reason"];
                } else {
                    failed_result["rejection_reason"] = "ik_failed_for_all_h_candidates";
                }
                if (!selected_full_values.empty()) {
                    failed_result["joint_names"] = full_joint_names;
                    failed_result["joint_values"] = selected_full_values;
                }
                record["lookup_result"] = failed_result;
                record["fallback_used"] = false;
                record["fallback_reason"] = failed_result.value("rejection_reason", "lookup_failed");

                if (baseline_fallback) {
                    nlohmann::json fallback_attempts = nlohmann::json::array();
                    IkResult fallback_result;
                    std::vector<double> fallback_seed;
                    size_t fallback_selected_attempt = 0;
                    const size_t attempts = std::max<size_t>(1, fallback_seed_attempts);
                    const std::vector<double> current_full_seed = makeFullSeedFromArmSeed(
                        fallback_ik.variableNames(), current_h, ik.variableNames(), seed);
                    std::vector<std::vector<double>> seed_queue;
                    addUniqueSeed(seed_queue, current_full_seed);
                    addUniqueSeed(seed_queue, std::vector<double>(fallback_ik.variableNames().size(), 0.0));
                    addUniqueSeed(seed_queue, last_successful_fallback_seed);
                    const auto stage_seed_it = last_successful_stage_seed.find(localStageName(stage.name));
                    if (stage_seed_it != last_successful_stage_seed.end()) {
                        addUniqueSeed(seed_queue, stage_seed_it->second);
                    }

                    for (size_t base_i = 0; base_i < seed_queue.size() && seed_queue.size() < attempts; ++base_i) {
                        const std::vector<double> base_seed = seed_queue[base_i];
                        for (size_t noise_i = 1; seed_queue.size() < attempts; ++noise_i) {
                            const double revolute_sigma = seed_noise * (1.0 + 0.25 * static_cast<double>(noise_i / 4));
                            const double prismatic_sigma = h_step * (1.0 + 0.5 * static_cast<double>(noise_i / 4));
                            addUniqueSeed(
                                seed_queue,
                                makePerturbedSeed(
                                    fallback_ik.variableNames(),
                                    base_seed,
                                    100000 + stage_index * attempts + base_i * 1000 + noise_i,
                                    revolute_sigma,
                                    prismatic_sigma));
                        }
                    }

                    for (size_t attempt = 0; attempt < seed_queue.size(); ++attempt) {
                        std::vector<double> attempt_seed = seed_queue[attempt];
                        for (size_t order_i = 0; order_i < 2; ++order_i) {
                            const bool swapped_order = order_i == 1;
                            IkResult candidate = swapped_order
                                ? fallback_ik.solveDual(
                                      compensateTool0OffsetForIk(stage.right, tool0_offset),
                                      compensateTool0OffsetForIk(stage.left, tool0_offset),
                                      attempt_seed,
                                      fallback_timeout)
                                : fallback_ik.solveDual(
                                      compensateTool0OffsetForIk(stage.left, tool0_offset),
                                      compensateTool0OffsetForIk(stage.right, tool0_offset),
                                      attempt_seed,
                                      fallback_timeout);
                            nlohmann::json attempt_record = attemptToJson(attempt, attempt_seed, candidate);
                            attempt_record["target_order"] = swapped_order ? "swapped" : "normal";
                            bool swapped_better = false;
                            if (!candidate.joint_values.empty()) {
                                const auto actual_poses = fallback_ik.fk(candidate.joint_values);
                                const DualTipMatch tip_match = dualTipMatch(stage.left, stage.right, actual_poses);
                                swapped_better = tip_match.swapped_is_better;
                                attempt_record["tip_target_match"] = dualTipMatchToJson(tip_match);
                                if (!actual_poses.empty()) {
                                    attempt_record["actual_pose"] = poseToJson(actual_poses[0]);
                                }
                                if (actual_poses.size() > 1) {
                                    attempt_record["actual_pose2"] = poseToJson(actual_poses[1]);
                                }
                                if (candidate.success && swapped_better) {
                                    candidate.success = false;
                                    attempt_record["rejection_reason"] = "fallback_dual_tip_target_swapped";
                                }
                            }
                            fallback_attempts.push_back(attempt_record);
                            if (attempt == 0 && order_i == 0) {
                                fallback_result = candidate;
                                fallback_seed = attempt_seed;
                                fallback_selected_attempt = attempt;
                            } else if ((candidate.success && !swapped_better) || candidate.collision_rejection_count < fallback_result.collision_rejection_count) {
                                fallback_result = candidate;
                                fallback_seed = attempt_seed;
                                fallback_selected_attempt = attempt;
                            }
                            if (candidate.success && !swapped_better) {
                                fallback_attempts.back()["selected"] = true;
                                break;
                            }
                            if (!swapped_better) {
                                break;
                            }
                        }
                        if (fallback_result.success) {
                            break;
                        }
                    }


                    record["fallback_used"] = true;
                    record["fallback_group"] = "dual_arm_with_base";
                    record["fallback_attempts"] = fallback_attempts;
                    record["fallback_selected_attempt"] = fallback_selected_attempt;
                    record["fallback_selected_seed"] = fallback_seed;
                    record["solver_path"] = fallback_result.success ? "baseline_fallback" : "lookup_failed_fallback_failed";

                    if (fallback_result.success) {
                        auto actual_poses = fallback_ik.fk(fallback_result.joint_values);
                        const DualTipMatch tip_match = dualTipMatch(stage.left, stage.right, actual_poses);
                        record["tip_target_match"] = dualTipMatchToJson(tip_match);
                        if (tip_match.swapped_is_better) {
                            fallback_result.success = false;
                            failed_result = resultToJson(fallback_result);
                            failed_result["rejection_reason"] = "fallback_dual_tip_target_swapped";
                            failed_result["tip_target_match"] = dualTipMatchToJson(tip_match);
                            record["result"] = failed_result;
                            record["next_seed_joints"] = seed;
                            record["next_updown"] = current_h;
                            stop_after_failure = true;
                            std::cout << "FAIL  fallback swapped-tip target binding suspected\n";
                        } else {
                            successes++;
                            total_ms += fallback_result.solve_ms;
                            const double fallback_h = updownFromResult(fallback_result, current_h);
                            total_updown_motion += std::abs(fallback_h - current_h);
                            record["selected_h"] = fallback_h;
                            record["updown_delta"] = std::abs(fallback_h - current_h);
                            record["actual_pose"] = poseToJson(actual_poses[0]);
                            const double tool_pos_error = positionError(stage.left, actual_poses[0]);
                            const double tool_ori_error = orientationError(stage.left, actual_poses[0]);
                            double max_tool_pos_error = tool_pos_error;
                            double max_tool_ori_error = tool_ori_error;
                            record["tool_pos_error"] = tool_pos_error;
                            record["tool_ori_error"] = tool_ori_error;
                            if (actual_poses.size() > 1) {
                                record["actual_pose2"] = poseToJson(actual_poses[1]);
                                const double tool_pos_error2 = positionError(stage.right, actual_poses[1]);
                                const double tool_ori_error2 = orientationError(stage.right, actual_poses[1]);
                                max_tool_pos_error = std::max(max_tool_pos_error, tool_pos_error2);
                                max_tool_ori_error = std::max(max_tool_ori_error, tool_ori_error2);
                                record["tool_pos_error2"] = tool_pos_error2;
                                record["tool_ori_error2"] = tool_ori_error2;
                            }
                            nlohmann::json public_result = resultToJson(fallback_result);
                            public_result["pos_error"] = max_tool_pos_error;
                            public_result["ori_error"] = max_tool_ori_error;
                            record["result"] = public_result;
                            seed = makeArmSeedFromFullResult(ik.variableNames(), fallback_result);
                            current_h = fallback_h;
                            last_successful_fallback_seed = fallback_result.joint_values;
                            last_successful_stage_seed[localStageName(stage.name)] = fallback_result.joint_values;
                            record["next_seed_joints"] = seed;
                            record["next_updown"] = current_h;
                            std::cout << "OK(FB) " << fallback_result.solve_ms << "ms  "
                                      << "h=" << fallback_h << "  "
                                      << "attempt=" << fallback_selected_attempt << "  "
                                      << "tool_pos_err=" << max_tool_pos_error << "  "
                                      << "tool_ori_err=" << max_tool_ori_error << "\n";
                        }
                    } else {
                        nlohmann::json fallback_failed = resultToJson(fallback_result);
                        fallback_failed["rejection_reason"] = "fallback_failed";
                        fallback_failed["lookup_rejection_reason"] = record["fallback_reason"];
                        record["result"] = fallback_failed;
                        record["next_seed_joints"] = seed;
                        record["next_updown"] = current_h;
                        stop_after_failure = true;
                        std::cout << "FAIL  fallback_failed\n";
                    }
                } else {
                    record["solver_path"] = "updown_lookup_failed";
                    record["result"] = failed_result;
                    record["next_seed_joints"] = seed;
                    record["next_updown"] = current_h;
                    stop_after_failure = true;
                    std::cout << "FAIL";
                    if (failed_result.contains("rejection_reason")) {
                        std::cout << "  " << failed_result["rejection_reason"].get<std::string>();
                    }
                    std::cout << "\n";
                }
            }

            ofs << record.dump() << "\n";
            stage_index++;

            if (stop_after_failure) {
                break;
            }
        }
        if (stop_after_failure) {
            break;
        }
    }

    ofs.close();

    std::cout << "\n=== Summary ===\n"
              << "  Success: " << successes << "/" << stage_index
              << " (" << (stage_index == 0 ? 0.0 : 100.0 * static_cast<double>(successes) / static_cast<double>(stage_index)) << "%)\n";
    if (successes > 0) {
        std::cout << "  Avg solve time: " << total_ms / static_cast<double>(successes) << "ms\n";
    }
    std::cout << "  Total updown motion: " << total_updown_motion << "m\n";
    std::cout << "  Results saved to: " << output << "\n";

    rclcpp::shutdown();
    return stop_after_failure ? 2 : 0;
}
