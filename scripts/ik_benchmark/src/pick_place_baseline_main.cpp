#include "ik_benchmark/ik_solver.h"

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <nlohmann/json.hpp>
#include <string>
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

struct AttemptResult {
    size_t attempt_index = 0;
    std::vector<double> seed;
    IkResult result;
    nlohmann::json attempts = nlohmann::json::array();
};

struct DualTipMatch {
    bool has_dual = false;
    double direct_pos_error = 0.0;
    double swapped_pos_error = 0.0;
    double direct_ori_error = 0.0;
    double swapped_ori_error = 0.0;
    bool swapped_is_better = false;
};

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

AttemptResult solveWithSeedAttempts(IkSolver& ik,
                                    const Eigen::Isometry3d& left_target,
                                    const Eigen::Isometry3d& right_target,
                                    const PoseSpec& left_tool0_target,
                                    const PoseSpec& right_tool0_target,
                                    const std::vector<double>& current_seed,
                                    const std::vector<double>& historical_seed,
                                    size_t seed_attempts,
                                    double seed_noise,
                                    double updown_seed_noise,
                                    double timeout)
{
    AttemptResult selected;
    selected.seed = current_seed;

    const size_t attempts = std::max<size_t>(1, seed_attempts);
    std::vector<std::vector<double>> seed_queue;
    addUniqueSeed(seed_queue, current_seed);
    addUniqueSeed(seed_queue, std::vector<double>(ik.variableNames().size(), 0.0));
    addUniqueSeed(seed_queue, historical_seed);
    for (size_t base_i = 0; base_i < seed_queue.size() && seed_queue.size() < attempts; ++base_i) {
        const std::vector<double> base_seed = seed_queue[base_i];
        for (size_t noise_i = 1; seed_queue.size() < attempts; ++noise_i) {
            const double revolute_sigma = seed_noise * (1.0 + 0.25 * static_cast<double>(noise_i / 4));
            const double prismatic_sigma = updown_seed_noise * (1.0 + 0.5 * static_cast<double>(noise_i / 4));
            addUniqueSeed(
                seed_queue,
                makePerturbedSeed(ik.variableNames(), base_seed, noise_i + base_i * 1000,
                                  revolute_sigma, prismatic_sigma));
        }
    }

    for (size_t attempt = 0; attempt < seed_queue.size(); ++attempt) {
        std::vector<double> attempt_seed = seed_queue[attempt];
        for (size_t order_i = 0; order_i < 2; ++order_i) {
            const bool swapped_order = order_i == 1;
            IkResult result = swapped_order
                ? ik.solveDual(right_target, left_target, attempt_seed, timeout)
                : ik.solveDual(left_target, right_target, attempt_seed, timeout);
            nlohmann::json attempt_record = attemptToJson(attempt, attempt_seed, result);
            attempt_record["target_order"] = swapped_order ? "swapped" : "normal";
            bool swapped_better = false;
            if (!result.joint_values.empty()) {
                const auto actual_poses = ik.fk(result.joint_values);
                const DualTipMatch match = dualTipMatch(left_tool0_target, right_tool0_target, actual_poses);
                swapped_better = match.swapped_is_better;
                attempt_record["tip_target_match"] = dualTipMatchToJson(match);
                if (!actual_poses.empty()) {
                    attempt_record["actual_pose"] = poseToJson(actual_poses[0]);
                }
                if (actual_poses.size() > 1) {
                    attempt_record["actual_pose2"] = poseToJson(actual_poses[1]);
                }
                if (result.success && swapped_better) {
                    result.success = false;
                    attempt_record["rejection_reason"] = "dual_tip_target_swapped";
                }
            }
            selected.attempts.push_back(attempt_record);
            if ((attempt == 0 && order_i == 0) || result.success || result.collision_rejection_count < selected.result.collision_rejection_count) {
                selected.attempt_index = attempt;
                selected.seed = attempt_seed;
                selected.result = result;
            }
            if (result.success && !swapped_better) {
                selected.attempt_index = attempt;
                selected.seed = attempt_seed;
                selected.result = result;
                selected.attempts.back()["selected"] = true;
                return selected;
            }
            if (!swapped_better) {
                break;
            }
        }
    }

    return selected;
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
    std::cout << "Usage: pick_place_baseline [options]\n"
              << "  --group <name>              MoveIt group (default: dual_arm_with_base)\n"
              << "  --solver <plugin>           IK solver plugin (default: bio_ik/BioIKKinematicsPlugin)\n"
              << "  --timeout <s>               IK timeout per stage (default: 2.0)\n"
              << "  --start <index>             Start round, 0-based (default: 0)\n"
              << "  --rounds <n>                Number of rounds (default: all active PICK_POINTS)\n"
              << "  --approach-offset <m>       Pick approach offset along -x (default: 0.1)\n"
              << "  --tool0-offset <m>          Fixed local +Z offset from link6 to tool0; compensated before IK (default: 0.1)\n"
              << "  --seed-attempts <n>         Try previous state first, then n-1 deterministic perturbed seeds (default: 12)\n"
              << "  --seed-noise <rad>          Revolute joint seed perturbation stddev (default: 0.6)\n"
              << "  --updown-seed-noise <m>     Updown seed perturbation stddev (default: 0.08)\n"
              << "  --allow-collision-solutions Keep IK solutions even when PlanningScene reports collision (diagnostic only)\n"
              << "  --output <path>             JSONL output path (default: /tmp/pick_place_baseline.jsonl)\n";
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string group = "dual_arm_with_base";
    std::string solver = "bio_ik/BioIKKinematicsPlugin";
    std::string output = "/tmp/pick_place_baseline.jsonl";
    double timeout = 2.0;
    double approach_offset = 0.1;
    double tool0_offset = 0.1;
    size_t seed_attempts = 12;
    double seed_noise = 0.6;
    double updown_seed_noise = 0.08;
    bool reject_collisions = true;
    size_t start = 0;
    size_t rounds = 0;

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
        else if (arg == "--updown-seed-noise" && i + 1 < argc) updown_seed_noise = std::stod(argv[++i]);
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

    IkSolverOptions options;
    options.base_frame = "base_link";
    options.tip_link = "left_tool0";
    options.tip_link2 = "right_tool0";
    options.reject_collisions = reject_collisions;

    IkSolver ik(group, solver, timeout, false, options);
    if (!ik.isDualArm()) {
        std::cerr << "pick_place_baseline requires a dual-arm group, got: " << group << "\n";
        rclcpp::shutdown();
        return 1;
    }

    std::ofstream ofs(output);
    if (!ofs.good()) {
        std::cerr << "Cannot write to " << output << "\n";
        rclcpp::shutdown();
        return 1;
    }

    std::vector<double> seed(ik.variableNames().size(), 0.0);
    size_t stage_index = 0;
    size_t successes = 0;
    double total_ms = 0.0;
    bool stop_after_failure = false;
    std::map<std::string, std::vector<double>> last_successful_stage_seed;

    nlohmann::json header;
    header["header"] = true;
    header["benchmark"] = "pick_place_baseline";
    header["group"] = group;
    header["solver"] = solver;
    header["tip_link"] = options.tip_link;
    header["tip_link2"] = options.tip_link2;
    header["base_frame"] = options.base_frame;
    header["start"] = start;
    header["rounds"] = rounds;
    header["approach_offset"] = approach_offset;
    header["tool0_offset_compensation"] = tool0_offset;
    header["seed_attempts"] = seed_attempts;
    header["seed_noise"] = seed_noise;
    header["updown_seed_noise"] = updown_seed_noise;
    header["flow"] = {"safe", "approach", "grasp", "retreat", "place_safe"};
    header["place_safe_left"] = poseToJson(PoseSpec{0.6, 0.2, 0.6, 0.0, 0.0, 0.0, 1.0});
    header["place_safe_right"] = poseToJson(PoseSpec{0.6, -0.2, 0.6, 0.0, 0.0, 0.0, 1.0});
    header["seed_policy"] = "attempt 0 uses previous successful stage solution; later attempts use deterministic perturbed seeds";
    ofs << header.dump() << "\n";

    std::cout << "=== Pick Place Baseline ===\n"
              << "  Group:   " << group << "\n"
              << "  Solver:  " << solver << "\n"
              << "  Timeout: " << timeout << "s\n"
              << "  Tool0 offset compensation: " << tool0_offset << "m\n"
              << "  Reject collisions: " << (reject_collisions ? "yes" : "no (diagnostic)") << "\n"
              << "  Seed attempts: " << seed_attempts << "\n"
              << "  Rounds:  " << rounds << " from " << start << "\n"
              << "  Output:  " << output << "\n\n";

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
            record["target_pose"] = poseToJson(stage.left);
            record["target_pose2"] = poseToJson(stage.right);
            record["ik_target_pose"] = poseToJson(compensateTool0OffsetForIk(stage.left, tool0_offset));
            record["ik_target_pose2"] = poseToJson(compensateTool0OffsetForIk(stage.right, tool0_offset));

            AttemptResult attempt_result = solveWithSeedAttempts(
                ik,
                compensateTool0OffsetForIk(stage.left, tool0_offset),
                compensateTool0OffsetForIk(stage.right, tool0_offset),
                stage.left,
                stage.right,
                seed,
                last_successful_stage_seed.count(localStageName(stage.name))
                    ? last_successful_stage_seed[localStageName(stage.name)]
                    : std::vector<double>{},
                seed_attempts,
                seed_noise,
                updown_seed_noise,
                timeout);
            IkResult result = attempt_result.result;
            record["selected_attempt"] = attempt_result.attempt_index;
            record["selected_attempt_seed"] = attempt_result.seed;
            record["attempts"] = attempt_result.attempts;
            record["ik_result_raw"] = resultToJson(result);

            std::cout << std::setw(2) << stage_index << "  " << stage.name << "  ";
            if (result.success) {
                auto actual_poses = ik.fk(result.joint_values);
                const DualTipMatch tip_match = dualTipMatch(stage.left, stage.right, actual_poses);
                record["tip_target_match"] = dualTipMatchToJson(tip_match);
                if (tip_match.swapped_is_better) {
                    result.success = false;
                    nlohmann::json rejected_result = resultToJson(result);
                    rejected_result["rejection_reason"] = "dual_tip_target_swapped";
                    rejected_result["tip_target_match"] = dualTipMatchToJson(tip_match);
                    record["result"] = rejected_result;
                    record["actual_pose"] = poseToJson(actual_poses[0]);
                    if (actual_poses.size() > 1) {
                        record["actual_pose2"] = poseToJson(actual_poses[1]);
                    }
                    record["next_seed_joints"] = seed;
                    stop_after_failure = true;
                    std::cout << "FAIL  swapped-tip target binding suspected"
                              << " direct=" << tip_match.direct_pos_error
                              << " swapped=" << tip_match.swapped_pos_error << "\n";
                    ofs << record.dump() << "\n";
                    stage_index++;
                    break;
                }

                successes++;
                total_ms += result.solve_ms;
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
                nlohmann::json public_result = resultToJson(result);
                public_result["pos_error"] = max_tool_pos_error;
                public_result["ori_error"] = max_tool_ori_error;
                record["result"] = public_result;
                seed = updateSeedFromResult(ik.variableNames(), seed, result);
                last_successful_stage_seed[localStageName(stage.name)] = result.joint_values;
                record["next_seed_joints"] = seed;
                std::cout << "OK    " << result.solve_ms << "ms  "
                          << "attempt=" << attempt_result.attempt_index << "  "
                          << "tool_pos_err=" << max_tool_pos_error << "  "
                          << "tool_ori_err=" << max_tool_ori_error << "\n";
            } else {
                record["result"] = resultToJson(result);
                record["next_seed_joints"] = seed;
                stop_after_failure = true;
                std::cout << "FAIL\n";
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
    std::cout << "  Results saved to: " << output << "\n";

    rclcpp::shutdown();
    return stop_after_failure ? 2 : 0;
}
