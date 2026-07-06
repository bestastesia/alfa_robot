#include "ik_benchmark/ik_solver.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
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

struct Episode {
    std::string id;
    std::string profile;
    std::string difficulty;
    std::vector<Stage> stages;
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

struct DualTipMatch {
    bool has_dual = false;
    double direct_left_pos_error = 0.0;
    double direct_right_pos_error = 0.0;
    double direct_pos_error = 0.0;
    double swapped_left_pos_error = 0.0;
    double swapped_right_pos_error = 0.0;
    double swapped_pos_error = 0.0;
    double direct_ori_error = 0.0;
    double swapped_ori_error = 0.0;
    bool swapped_is_better = false;
};

struct TrialSpec {
    size_t trial_index = 0;
    size_t h_index = 0;
    size_t seed_index = 0;
    double h = 0.0;
    std::vector<double> seed;
};

struct TrialResult {
    size_t trial_index = 0;
    size_t h_index = 0;
    size_t seed_index = 0;
    double h = 0.0;
    bool legal = false;
    bool timeout_like = false;
    bool collision_free = false;
    bool swapped = false;
    double solve_ms = 0.0;
    std::string target_order;
    double direct_left_pos_error = 0.0;
    double direct_right_pos_error = 0.0;
    double direct_pos_error = 0.0;
    double direct_ori_error = 0.0;
    double swapped_left_pos_error = 0.0;
    double swapped_right_pos_error = 0.0;
    double swapped_pos_error = 0.0;
    double score = std::numeric_limits<double>::infinity();
    std::string rejection_reason;
    std::vector<double> joint_values;
    std::vector<double> full_values;
    std::vector<std::string> collision_pairs;
};

struct StageResult {
    std::string episode_id;
    std::string difficulty;
    std::string stage_name;
    size_t stage_index = 0;
    size_t workers = 1;
    bool reachable = false;
    bool success = false;
    double previous_h = 0.0;
    double selected_h = 0.0;
    double updown_delta = 0.0;
    double wall_ms = 0.0;
    double sum_solve_ms = 0.0;
    size_t trial_count = 0;
    size_t legal_count = 0;
    size_t timeout_like_count = 0;
    size_t swapped_count = 0;
    HeightPlan height_plan;
    TrialResult selected;
};

struct RunSummary {
    std::string difficulty;
    size_t workers = 1;
    size_t episode_count = 0;
    size_t stage_count = 0;
    size_t success_stage_count = 0;
    size_t failed_stage_count = 0;
    size_t range_out_stage_count = 0;
    size_t trial_count = 0;
    size_t legal_count = 0;
    size_t timeout_like_count = 0;
    size_t swapped_rejected_count = 0;
    double init_ms = 0.0;
    double wall_ms = 0.0;
    double sum_solve_ms = 0.0;
    double total_updown_motion = 0.0;
};

struct DatasetBuildStats {
    size_t candidate_episodes = 0;
    size_t range_reachable_episodes = 0;
    size_t range_unreachable_episodes = 0;
    size_t reachable_episodes = 0;
    size_t ik_failed_episodes = 0;
    size_t total_unreachable_episodes = 0;
    size_t success_episodes = 0;
    size_t mixed_episodes = 0;
    size_t mostly_fail_episodes = 0;
    size_t unreachable_dataset_episodes = 0;
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
    match.direct_left_pos_error = positionError(left_target, actual_poses[0]);
    match.direct_right_pos_error = positionError(right_target, actual_poses[1]);
    match.direct_pos_error = std::max(match.direct_left_pos_error, match.direct_right_pos_error);
    match.swapped_left_pos_error = positionError(left_target, actual_poses[1]);
    match.swapped_right_pos_error = positionError(right_target, actual_poses[0]);
    match.swapped_pos_error = std::max(match.swapped_left_pos_error, match.swapped_right_pos_error);
    match.direct_ori_error = std::max(orientationError(left_target, actual_poses[0]),
                                      orientationError(right_target, actual_poses[1]));
    match.swapped_ori_error = std::max(orientationError(left_target, actual_poses[1]),
                                       orientationError(right_target, actual_poses[0]));
    match.swapped_is_better = match.swapped_pos_error + 1e-4 < match.direct_pos_error;
    return match;
}

nlohmann::json intervalToJson(const HeightInterval& interval)
{
    return {{"reachable", interval.reachable}, {"lower", interval.lower}, {"upper", interval.upper}};
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
    interval.lower = std::max(h_lower, ik_target_z - sphere.cz - z_margin);
    interval.upper = std::min(h_upper, ik_target_z - sphere.cz + z_margin);
    interval.reachable = interval.lower <= interval.upper;
    return interval;
}

std::vector<double> makeHeightCandidates(const HeightInterval& interval,
                                         double current_h,
                                         double step,
                                         size_t max_candidates)
{
    std::vector<double> candidates;
    if (!interval.reachable || max_candidates == 0) {
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

    const double best_h = std::min(std::max(current_h, interval.lower), interval.upper);
    add_unique(best_h);
    if (step <= 0.0) {
        return candidates;
    }

    for (size_t ring = 1; candidates.size() < max_candidates; ++ring) {
        const double delta = step * static_cast<double>(ring);
        bool added = false;
        const double lower_value = best_h - delta;
        const double upper_value = best_h + delta;
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

std::vector<double> makePerturbedSeed(const std::vector<std::string>& variable_names,
                                      const std::vector<double>& base_seed,
                                      size_t attempt_index,
                                      double revolute_noise)
{
    std::vector<double> seed = base_seed;
    if (seed.size() != variable_names.size()) {
        seed.assign(variable_names.size(), 0.0);
    }
    std::mt19937 rng(static_cast<uint32_t>(0xC0FFEEu + 7919u * attempt_index));
    std::normal_distribution<double> revolute_dist(0.0, revolute_noise);
    for (size_t i = 0; i < seed.size(); ++i) {
        const std::string& name = variable_names[i];
        if (name == "updown" || name.find("pitch") != std::string::npos) {
            continue;
        }
        seed[i] += revolute_dist(rng);
    }
    return seed;
}

std::vector<TrialSpec> makeTrials(const HeightPlan& height_plan,
                                  const std::vector<std::string>& variable_names,
                                  const std::vector<double>& base_seed,
                                  size_t seed_attempts,
                                  double seed_noise);

TrialResult solveTrial(IkSolver& ik,
                       const std::vector<std::string>& full_joint_names,
                       const Stage& stage,
                       const TrialSpec& trial,
                       double current_h,
                       double tool0_offset,
                       double timeout,
                       bool reject_collisions,
                       double pos_threshold,
                       double ori_threshold);

std::vector<Stage> makeRoundStages(const std::string& prefix,
                                  const PickPoint& point,
                                  double approach_offset,
                                  double place_safe_z)
{
    const PoseSpec& left_pick = point.left;
    const PoseSpec& right_pick = point.right;
    return {
        {prefix + "/safe",
         {0.3, 0.3, left_pick.z, left_pick.qx, left_pick.qy, left_pick.qz, left_pick.qw},
         {0.3, -0.3, right_pick.z, right_pick.qx, right_pick.qy, right_pick.qz, right_pick.qw}},
        {prefix + "/approach",
         {left_pick.x - approach_offset, left_pick.y, left_pick.z, left_pick.qx, left_pick.qy, left_pick.qz, left_pick.qw},
         {right_pick.x - approach_offset, right_pick.y, right_pick.z, right_pick.qx, right_pick.qy, right_pick.qz, right_pick.qw}},
        {prefix + "/grasp", left_pick, right_pick},
        {prefix + "/retreat",
         {left_pick.x - approach_offset, left_pick.y, left_pick.z, left_pick.qx, left_pick.qy, left_pick.qz, left_pick.qw},
         {right_pick.x - approach_offset, right_pick.y, right_pick.z, right_pick.qx, right_pick.qy, right_pick.qz, right_pick.qw}},
        {prefix + "/place_safe",
         {0.6, 0.2, place_safe_z, -0.5, 0.5, 0.5, 0.5},
         {0.6, -0.2, place_safe_z, -0.5, 0.5, 0.5, 0.5}}
    };
}

std::vector<PickPoint> seedPickPoints()
{
    const double qx = 0.0;
    const double qy = 0.7071;
    const double qz = 0.0;
    const double qw = 0.7071;
    return {
        {{0.6, 0.5, 1.5, qx, qy, qz, qw}, {0.6, -0.5, 1.5, qx, qy, qz, qw}},
        {{0.6, 0.3, 1.5, qx, qy, qz, qw}, {0.6, -0.3, 1.5, qx, qy, qz, qw}},
        {{0.6, 0.0, 1.5, qx, qy, qz, qw}, {0.6, 0.0, 1.1, qx, qy, qz, qw}},
    };
}

PickPoint offsetPickPoint(const PickPoint& point,
                          double x_offset,
                          double symmetric_y_offset,
                          double z_offset)
{
    PickPoint shifted = point;
    shifted.left.x += x_offset;
    shifted.right.x += x_offset;
    shifted.left.y += symmetric_y_offset;
    shifted.right.y -= symmetric_y_offset;
    shifted.left.z += z_offset;
    shifted.right.z += z_offset;
    return shifted;
}

std::vector<PickPoint> expandedPickPoints(bool base_only)
{
    const auto seeds = seedPickPoints();
    if (base_only) {
        return seeds;
    }
    const std::vector<std::tuple<double, double, double>> offsets = {
        {0.0, 0.0, 0.0}, {-0.05, 0.0, 0.0}, {0.0, 0.05, 0.0}, {0.0, -0.05, 0.0}, {-0.05, 0.0, 0.05},
    };
    std::vector<PickPoint> points;
    for (const auto& seed : seeds) {
        for (const auto& [x_offset, y_offset, z_offset] : offsets) {
            points.push_back(offsetPickPoint(seed, x_offset, y_offset, z_offset));
        }
    }
    return points;
}

std::vector<Stage> makeProfileStages(const std::string& prefix,
                                     const PickPoint& point,
                                     double approach_offset,
                                     double grasp_z_offset,
                                     double global_x_offset,
                                     double place_safe_z)
{
    PickPoint shifted = point;
    shifted.left.x += global_x_offset;
    shifted.right.x += global_x_offset;

    PickPoint grasp_point = shifted;
    grasp_point.left.z = std::max(0.05, point.left.z + grasp_z_offset);
    grasp_point.right.z = std::max(0.05, point.right.z + grasp_z_offset);

    std::vector<Stage> stages = makeRoundStages(prefix, grasp_point, approach_offset, place_safe_z);
    if (!stages.empty()) {
        stages[0].left = {0.3 + global_x_offset, 0.3, point.left.z, point.left.qx, point.left.qy, point.left.qz, point.left.qw};
        stages[0].right = {0.3 + global_x_offset, -0.3, point.right.z, point.right.qx, point.right.qy, point.right.qz, point.right.qw};
    }
    if (stages.size() >= 5 && std::abs(global_x_offset) > 1e-12) {
        stages[4].left.x += global_x_offset;
        stages[4].right.x += global_x_offset;
    }
    return stages;
}

bool isRangeReachableEpisode(const Episode& episode,
                             const ReachSphere& left_sphere,
                             const ReachSphere& right_sphere,
                             double h_lower,
                             double h_upper,
                             double tool0_offset,
                             double sphere_margin,
                             double h_step,
                             size_t h_candidate_limit)
{
    double current_h = std::min(std::max(0.0, h_lower), h_upper);
    for (const auto& stage : episode.stages) {
        const HeightPlan plan = planHeights(stage.left, stage.right, left_sphere, right_sphere,
                                           current_h, h_lower, h_upper, tool0_offset,
                                           sphere_margin, h_step, h_candidate_limit);
        if (!plan.reachable || plan.candidates.empty()) {
            return false;
        }
        current_h = plan.candidates.front();
    }
    return true;
}

bool isIkSuccessfulEpisode(const Episode& episode,
                           IkSolver& ik,
                           const std::vector<std::string>& full_joint_names,
                           const std::vector<std::string>& variable_names,
                           const ReachSphere& left_sphere,
                           const ReachSphere& right_sphere,
                           double h_lower,
                           double h_upper,
                           double tool0_offset,
                           double sphere_margin,
                           double h_step,
                           size_t h_candidate_limit,
                           size_t seed_attempts,
                           double seed_noise,
                           double timeout,
                           bool reject_collisions,
                           double pos_threshold,
                           double ori_threshold)
{
    double current_h = std::min(std::max(0.0, h_lower), h_upper);
    std::vector<double> seed(variable_names.size(), 0.0);
    for (const auto& stage : episode.stages) {
        const HeightPlan height_plan = planHeights(stage.left, stage.right, left_sphere, right_sphere,
                                                  current_h, h_lower, h_upper, tool0_offset,
                                                  sphere_margin, h_step, h_candidate_limit);
        if (!height_plan.reachable || height_plan.candidates.empty()) {
            return false;
        }
        const std::vector<TrialSpec> trials = makeTrials(height_plan, variable_names, seed, seed_attempts, seed_noise);
        TrialResult best;
        bool found = false;
        for (const auto& trial : trials) {
            TrialResult result = solveTrial(ik, full_joint_names, stage, trial, current_h, tool0_offset,
                                            timeout, reject_collisions, pos_threshold, ori_threshold);
            if (!result.legal) {
                continue;
            }
            if (!found || result.score < best.score) {
                best = std::move(result);
                found = true;
            }
        }
        if (!found) {
            return false;
        }
        current_h = best.h;
        seed = best.joint_values;
    }
    return true;
}

std::vector<Episode> candidateEpisodes(double approach_offset,
                                       double place_safe_z,
                                       bool base_only_dataset)
{
    std::vector<Episode> episodes;
    const auto points = expandedPickPoints(base_only_dataset);
    const std::vector<std::pair<std::string, double>> z_profiles = {
        {"orig", 0.0},
        {"z_minus_0p4", -0.4},
        {"z_minus_0p8", -0.8},
    };
    const std::vector<std::pair<std::string, double>> x_profiles = {
        {"nominal", 0.0},
        {"x_plus_0p5", 0.5},
        {"x_minus_0p5", -0.5},
        {"x_plus_0p8", 0.8},
    };

    for (const auto& [z_name, z_offset] : z_profiles) {
        for (const auto& [x_name, x_offset] : x_profiles) {
            for (size_t i = 0; i < points.size(); ++i) {
                Episode episode;
                episode.id = z_name + "/" + x_name + "/round_" + std::to_string(i + 1);
                episode.profile = z_name + "/" + x_name;
                episode.difficulty = "candidate";
                episode.stages = makeProfileStages(episode.id, points[i], approach_offset, z_offset, x_offset, place_safe_z);
                episodes.push_back(std::move(episode));
            }
        }
    }
    return episodes;
}

Episode relabelEpisode(const Episode& source, const std::string& difficulty, size_t sample_index)
{
    Episode episode = source;
    episode.difficulty = difficulty;
    episode.id = difficulty + "/" + episode.id + "/sample_" + std::to_string(sample_index + 1);
    for (size_t stage_index = 0; stage_index < episode.stages.size(); ++stage_index) {
        const auto slash = episode.stages[stage_index].name.find_last_of('/');
        const std::string suffix = slash == std::string::npos ? episode.stages[stage_index].name : episode.stages[stage_index].name.substr(slash + 1);
        episode.stages[stage_index].name = episode.id + "/" + suffix;
    }
    return episode;
}

void appendWithDifficulty(std::vector<Episode>& out,
                          const std::vector<Episode>& source,
                          const std::string& difficulty,
                          size_t count,
                          size_t offset = 0)
{
    if (source.empty() || count == 0) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        out.push_back(relabelEpisode(source[(offset + i) % source.size()], difficulty, i));
    }
}

void appendFailuresWithDifficulty(std::vector<Episode>& out,
                                  const std::vector<Episode>& ik_failed_pool,
                                  const std::vector<Episode>& range_unreachable_pool,
                                  const std::string& difficulty,
                                  size_t count,
                                  size_t ik_offset = 0,
                                  size_t range_offset = 0)
{
    size_t appended = 0;
    if (!ik_failed_pool.empty()) {
        const size_t ik_count = std::min(count, ik_failed_pool.size());
        for (size_t i = 0; i < ik_count; ++i) {
            out.push_back(relabelEpisode(ik_failed_pool[(ik_offset + i) % ik_failed_pool.size()], difficulty, appended++));
        }
    }
    while (appended < count && !range_unreachable_pool.empty()) {
        const size_t i = appended;
        out.push_back(relabelEpisode(range_unreachable_pool[(range_offset + i) % range_unreachable_pool.size()], difficulty, appended));
        ++appended;
    }
}

std::vector<Episode> makeEpisodes(const std::string& profile,
                                  double approach_offset,
                                  double place_safe_z,
                                  bool base_only_dataset,
                                  double h_lower,
                                  double h_upper,
                                  double tool0_offset,
                                  double sphere_margin,
                                  double h_step,
                                  size_t h_candidate_limit,
                                  size_t seed_attempts,
                                  double seed_noise,
                                  double timeout,
                                  bool reject_collisions,
                                  double pos_threshold,
                                  double ori_threshold,
                                  const std::string& dataset_prefilter,
                                  IkSolver* prefilter_ik,
                                  const std::vector<std::string>& prefilter_full_joint_names,
                                  DatasetBuildStats* stats)
{
    const auto candidates = candidateEpisodes(approach_offset, place_safe_z, base_only_dataset);
    ReachSphere left_sphere;
    ReachSphere right_sphere;
    right_sphere.cy = -0.3125;

    std::vector<Episode> reachable_pool;
    std::vector<Episode> ik_failed_pool;
    std::vector<Episode> range_unreachable_pool;
    size_t range_reachable_count = 0;
    std::vector<std::string> variable_names;
    if (dataset_prefilter == "ik" && prefilter_ik) {
        variable_names = prefilter_ik->variableNames();
    }
    for (const auto& episode : candidates) {
        const bool range_reachable = isRangeReachableEpisode(episode, left_sphere, right_sphere,
                                                            h_lower, h_upper, tool0_offset,
                                                            sphere_margin, h_step, h_candidate_limit);
        if (!range_reachable) {
            range_unreachable_pool.push_back(episode);
            continue;
        }
        ++range_reachable_count;
        if (dataset_prefilter == "ik" && prefilter_ik) {
            const bool ik_success = isIkSuccessfulEpisode(episode, *prefilter_ik, prefilter_full_joint_names, variable_names,
                                                          left_sphere, right_sphere, h_lower, h_upper, tool0_offset,
                                                          sphere_margin, h_step, h_candidate_limit, seed_attempts, seed_noise,
                                                          timeout, reject_collisions, pos_threshold, ori_threshold);
            if (ik_success) {
                reachable_pool.push_back(episode);
            } else {
                ik_failed_pool.push_back(episode);
            }
        } else {
            reachable_pool.push_back(episode);
        }
    }

    if (stats) {
        stats->candidate_episodes = candidates.size();
        stats->range_reachable_episodes = range_reachable_count;
        stats->range_unreachable_episodes = range_unreachable_pool.size();
        stats->reachable_episodes = reachable_pool.size();
        stats->ik_failed_episodes = ik_failed_pool.size();
        stats->total_unreachable_episodes = ik_failed_pool.size() + range_unreachable_pool.size();
    }

    const size_t success_count = base_only_dataset ? std::min<size_t>(3, reachable_pool.size()) : std::min<size_t>(15, reachable_pool.size());
    const size_t mixed_count = success_count;
    const size_t mostly_fail_count = success_count;
    const size_t fail_pool_size = ik_failed_pool.size() + range_unreachable_pool.size();
    const size_t unreachable_count = base_only_dataset ? std::min<size_t>(9, fail_pool_size) : std::min<size_t>(45, fail_pool_size);

    std::vector<Episode> episodes;
    if (profile == "success" || profile == "all") {
        appendWithDifficulty(episodes, reachable_pool, "success", success_count, 0);
        if (stats) stats->success_episodes = success_count;
    }
    if (profile == "mixed" || profile == "all") {
        const size_t reachable_count = mixed_count - mixed_count / 3;
        const size_t fail_count = mixed_count - reachable_count;
        appendWithDifficulty(episodes, reachable_pool, "mixed", reachable_count, 3);
        appendFailuresWithDifficulty(episodes, ik_failed_pool, range_unreachable_pool, "mixed", fail_count, 0, 0);
        if (stats) stats->mixed_episodes = reachable_count + fail_count;
    }
    if (profile == "mostly_fail" || profile == "all") {
        const size_t reachable_count = mostly_fail_count / 4;
        const size_t fail_count = mostly_fail_count - reachable_count;
        appendWithDifficulty(episodes, reachable_pool, "mostly_fail", reachable_count, 7);
        appendFailuresWithDifficulty(episodes, ik_failed_pool, range_unreachable_pool, "mostly_fail", fail_count, 2, 5);
        if (stats) stats->mostly_fail_episodes = reachable_count + fail_count;
    }
    if (profile == "unreachable" || profile == "all") {
        appendFailuresWithDifficulty(episodes, range_unreachable_pool, ik_failed_pool, "unreachable", unreachable_count, 0, 0);
        if (stats) stats->unreachable_dataset_episodes = unreachable_count;
    }
    return episodes;
}

std::vector<size_t> parseWorkersList(const std::string& text)
{
    std::vector<size_t> values;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) values.push_back(std::max<size_t>(1, static_cast<size_t>(std::stoul(item))));
    }
    if (values.empty()) values.push_back(1);
    return values;
}

std::vector<TrialSpec> makeTrials(const HeightPlan& height_plan,
                                  const std::vector<std::string>& variable_names,
                                  const std::vector<double>& base_seed,
                                  size_t seed_attempts,
                                  double seed_noise);

TrialResult solveTrial(IkSolver& ik,
                       const std::vector<std::string>& full_joint_names,
                       const Stage& stage,
                       const TrialSpec& trial,
                       double current_h,
                       double tool0_offset,
                       double timeout,
                       bool reject_collisions,
                       double pos_threshold,
                       double ori_threshold)
{
    TrialResult out;
    out.trial_index = trial.trial_index;
    out.h_index = trial.h_index;
    out.seed_index = trial.seed_index;
    out.h = trial.h;

    const Eigen::Isometry3d left_target = targetInFixedUpdownFrame(stage.left, trial.h, tool0_offset);
    const Eigen::Isometry3d right_target = targetInFixedUpdownFrame(stage.right, trial.h, tool0_offset);

    std::vector<TrialResult> order_results;
    order_results.reserve(2);
    for (size_t order_index = 0; order_index < 2; ++order_index) {
        TrialResult candidate = out;
        candidate.target_order = (order_index == 0) ? "normal" : "swapped";
        IkResult result = (order_index == 0)
            ? ik.solveDual(left_target, right_target, trial.seed, timeout)
            : ik.solveDual(right_target, left_target, trial.seed, timeout);
        candidate.solve_ms = result.solve_ms;
        candidate.timeout_like = result.solve_ms >= timeout * 1000.0 * 0.90;
        if (result.joint_values.empty()) {
            candidate.rejection_reason = "ik_failed_empty_solution";
            order_results.push_back(std::move(candidate));
            continue;
        }

        candidate.joint_values = result.joint_values;
        candidate.full_values = fullJointValuesForUpdownLookup(trial.h, result.joint_values);
        std::vector<std::string> full_collision_pairs;
        candidate.collision_free = ik.isNamedStateCollisionFree(full_joint_names, candidate.full_values, &full_collision_pairs);
        candidate.collision_pairs = full_collision_pairs;
        if (reject_collisions && !candidate.collision_free) {
            candidate.rejection_reason = "full_state_collision";
            order_results.push_back(std::move(candidate));
            continue;
        }

        const auto actual_poses = ik.fkNamed(full_joint_names, candidate.full_values);
        const DualTipMatch match = dualTipMatch(stage.left, stage.right, actual_poses);
        candidate.direct_left_pos_error = match.direct_left_pos_error;
        candidate.direct_right_pos_error = match.direct_right_pos_error;
        candidate.direct_pos_error = match.direct_pos_error;
        candidate.direct_ori_error = match.direct_ori_error;
        candidate.swapped_left_pos_error = match.swapped_left_pos_error;
        candidate.swapped_right_pos_error = match.swapped_right_pos_error;
        candidate.swapped_pos_error = match.swapped_pos_error;
        candidate.swapped = match.swapped_is_better;
        if (match.swapped_is_better) {
            candidate.rejection_reason = "tip_order_error";
            order_results.push_back(std::move(candidate));
            continue;
        }
        if (match.direct_pos_error > pos_threshold || match.direct_ori_error > ori_threshold) {
            candidate.rejection_reason = "tip_error_too_large";
            order_results.push_back(std::move(candidate));
            continue;
        }

        candidate.legal = true;
        candidate.score = std::abs(trial.h - current_h) + 0.001 * candidate.solve_ms +
                          0.01 * static_cast<double>(trial.seed_index) +
                          0.0001 * static_cast<double>(order_index);
        order_results.push_back(std::move(candidate));
    }

    auto best = std::min_element(order_results.begin(), order_results.end(), [](const TrialResult& a, const TrialResult& b) {
        if (a.legal != b.legal) return a.legal > b.legal;
        if (a.swapped != b.swapped) return !a.swapped;
        if (a.direct_pos_error != b.direct_pos_error) return a.direct_pos_error < b.direct_pos_error;
        return a.solve_ms < b.solve_ms;
    });
    if (best == order_results.end()) {
        out.rejection_reason = "no_order_result";
        return out;
    }
    return *best;
}

std::vector<TrialResult> runTrials(const std::vector<TrialSpec>& trials,
                                   const Stage& stage,
                                   std::vector<std::unique_ptr<IkSolver>>& solvers,
                                   const std::vector<std::vector<std::string>>& full_joint_names_by_worker,
                                   double current_h,
                                   double tool0_offset,
                                   double timeout,
                                   bool reject_collisions,
                                   double pos_threshold,
                                   double ori_threshold)
{
    std::vector<TrialResult> results(trials.size());
    const size_t workers = std::max<size_t>(1, solvers.size());
    if (workers == 1) {
        for (size_t i = 0; i < trials.size(); ++i) {
            results[i] = solveTrial(*solvers[0], full_joint_names_by_worker[0], stage, trials[i], current_h,
                                    tool0_offset, timeout, reject_collisions, pos_threshold, ori_threshold);
        }
        return results;
    }

    std::atomic<size_t> next_index{0};
    std::vector<std::thread> threads;
    threads.reserve(workers);
    for (size_t worker = 0; worker < workers; ++worker) {
        threads.emplace_back([&, worker]() {
            while (true) {
                const size_t index = next_index.fetch_add(1);
                if (index >= trials.size()) break;
                results[index] = solveTrial(*solvers[worker], full_joint_names_by_worker[worker], stage, trials[index], current_h,
                                            tool0_offset, timeout, reject_collisions, pos_threshold, ori_threshold);
            }
        });
    }
    for (auto& thread : threads) thread.join();
    return results;
}

std::vector<TrialSpec> makeTrials(const HeightPlan& height_plan,
                                  const std::vector<std::string>& variable_names,
                                  const std::vector<double>& base_seed,
                                  size_t seed_attempts,
                                  double seed_noise)
{
    std::vector<TrialSpec> trials;
    size_t trial_index = 0;
    for (size_t h_index = 0; h_index < height_plan.candidates.size(); ++h_index) {
        for (size_t seed_index = 0; seed_index < std::max<size_t>(1, seed_attempts); ++seed_index) {
            TrialSpec trial;
            trial.trial_index = trial_index++;
            trial.h_index = h_index;
            trial.seed_index = seed_index;
            trial.h = height_plan.candidates[h_index];
            trial.seed = seed_index == 0
                ? base_seed
                : makePerturbedSeed(variable_names, base_seed, seed_index + h_index * std::max<size_t>(1, seed_attempts), seed_noise);
            trials.push_back(std::move(trial));
        }
    }
    return trials;
}

nlohmann::json trialToJson(const TrialResult& trial)
{
    return {
        {"trial_index", trial.trial_index},
        {"h", trial.h},
        {"h_index", trial.h_index},
        {"seed_index", trial.seed_index},
        {"legal", trial.legal},
        {"timeout_like", trial.timeout_like},
        {"collision_free", trial.collision_free},
        {"swapped", trial.swapped},
        {"target_order", trial.target_order},
        {"solve_ms", trial.solve_ms},
        {"direct_left_pos_error", trial.direct_left_pos_error},
        {"direct_right_pos_error", trial.direct_right_pos_error},
        {"direct_pos_error", trial.direct_pos_error},
        {"direct_ori_error", trial.direct_ori_error},
        {"swapped_left_pos_error", trial.swapped_left_pos_error},
        {"swapped_right_pos_error", trial.swapped_right_pos_error},
        {"swapped_pos_error", trial.swapped_pos_error},
        {"score", std::isfinite(trial.score) ? trial.score : -1.0},
        {"rejection_reason", trial.rejection_reason},
        {"collision_pairs", trial.collision_pairs}
    };
}

nlohmann::json stageToJson(const StageResult& stage)
{
    return {
        {"type", "stage"},
        {"episode_id", stage.episode_id},
        {"difficulty", stage.difficulty},
        {"stage", stage.stage_name},
        {"stage_index", stage.stage_index},
        {"workers", stage.workers},
        {"reachable", stage.reachable},
        {"success", stage.success},
        {"previous_h", stage.previous_h},
        {"selected_h", stage.selected_h},
        {"updown_delta", stage.updown_delta},
        {"wall_ms", stage.wall_ms},
        {"sum_solve_ms", stage.sum_solve_ms},
        {"trial_count", stage.trial_count},
        {"legal_count", stage.legal_count},
        {"timeout_like_count", stage.timeout_like_count},
        {"swapped_count", stage.swapped_count},
        {"h_interval", intervalToJson(stage.height_plan.combined)},
        {"h_candidates", stage.height_plan.candidates},
        {"selected_trial", trialToJson(stage.selected)}
    };
}

nlohmann::json summaryToJson(const RunSummary& summary, double baseline_wall_ms)
{
    return {
        {"type", "summary"},
        {"difficulty", summary.difficulty},
        {"workers", summary.workers},
        {"episode_count", summary.episode_count},
        {"stage_count", summary.stage_count},
        {"success_stage_count", summary.success_stage_count},
        {"failed_stage_count", summary.failed_stage_count},
        {"range_out_stage_count", summary.range_out_stage_count},
        {"trial_count", summary.trial_count},
        {"legal_count", summary.legal_count},
        {"timeout_like_count", summary.timeout_like_count},
        {"swapped_rejected_count", summary.swapped_rejected_count},
        {"init_ms", summary.init_ms},
        {"wall_ms", summary.wall_ms},
        {"sum_solve_ms", summary.sum_solve_ms},
        {"total_updown_motion", summary.total_updown_motion},
        {"speedup_vs_workers_1", baseline_wall_ms > 0.0 ? baseline_wall_ms / summary.wall_ms : 1.0}
    };
}

void printHelp()
{
    std::cout << "parallel_ik_benchmark: staged episode benchmark for MOTION-30\n"
              << "  --profile <success|mixed|mostly_fail|unreachable|all>\n"
              << "  --workers-list <csv>        Worker counts, default 1,2,4,8\n"
              << "  --timeout <sec>             IK timeout per trial, default 0.2\n"
              << "  --h-candidates <n>          H candidates per stage, default 8\n"
              << "  --seed-attempts <n>         Seeds per h, default 4\n"
              << "  --h-step <m>                H expansion step around minimum-motion h, default 0.1\n"
              << "  --solution-candidates <n>   Reserved for compatibility; all trials are evaluated\n"
              << "  --place-safe-z <m>          Place-safe z height, default 0.85\n"
              << "  --base-only-dataset         Use only original 3 PICK_POINTS\n"
              << "  --dataset-prefilter <sphere|ik> Dataset pool prefilter, default sphere\n"
              << "  --allow-collision-solutions Do not reject full-state collision results\n"
              << "  --include-candidates        Emit per-trial JSON\n"
              << "  --output <path>             JSONL output path\n";
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string group = "dual_arm";
    std::string solver = "bio_ik/BioIKKinematicsPlugin";
    std::string profile = "all";
    std::string workers_list_text = "1,2,4,8";
    std::string output = "/tmp/parallel_ik_benchmark.jsonl";
    double timeout = 0.2;
    double tool0_offset = 0.1;
    double approach_offset = 0.1;
    double place_safe_z = 0.85;
    double seed_noise = 0.35;
    double h_lower = 0.0;
    double h_upper = 0.99;
    double h_step = 0.1;
    double sphere_margin = 0.0;
    double pos_threshold = 0.02;
    double ori_threshold = 0.05;
    size_t h_candidate_limit = 8;
    size_t seed_attempts = 4;
    size_t limit_episodes = 0;
    bool reject_collisions = true;
    bool include_candidates = false;
    bool base_only_dataset = false;
    std::string dataset_prefilter = "sphere";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--group" && i + 1 < argc) group = argv[++i];
        else if (arg == "--solver" && i + 1 < argc) solver = argv[++i];
        else if (arg == "--profile" && i + 1 < argc) profile = argv[++i];
        else if (arg == "--workers-list" && i + 1 < argc) workers_list_text = argv[++i];
        else if (arg == "--timeout" && i + 1 < argc) timeout = std::stod(argv[++i]);
        else if (arg == "--tool0-offset" && i + 1 < argc) tool0_offset = std::stod(argv[++i]);
        else if (arg == "--approach-offset" && i + 1 < argc) approach_offset = std::stod(argv[++i]);
        else if (arg == "--place-safe-z" && i + 1 < argc) place_safe_z = std::stod(argv[++i]);
        else if (arg == "--seed-noise" && i + 1 < argc) seed_noise = std::stod(argv[++i]);
        else if (arg == "--h-lower" && i + 1 < argc) h_lower = std::stod(argv[++i]);
        else if (arg == "--h-upper" && i + 1 < argc) h_upper = std::stod(argv[++i]);
        else if (arg == "--h-step" && i + 1 < argc) h_step = std::stod(argv[++i]);
        else if (arg == "--sphere-margin" && i + 1 < argc) sphere_margin = std::stod(argv[++i]);
        else if (arg == "--pos-threshold" && i + 1 < argc) pos_threshold = std::stod(argv[++i]);
        else if (arg == "--ori-threshold" && i + 1 < argc) ori_threshold = std::stod(argv[++i]);
        else if (arg == "--h-candidates" && i + 1 < argc) h_candidate_limit = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--seed-attempts" && i + 1 < argc) seed_attempts = static_cast<size_t>(std::stoul(argv[++i]));
        else if ((arg == "--limit-cases" || arg == "--limit-episodes") && i + 1 < argc) limit_episodes = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--allow-collision-solutions") reject_collisions = false;
        else if (arg == "--include-candidates") include_candidates = true;
        else if (arg == "--base-only-dataset") base_only_dataset = true;
        else if (arg == "--dataset-prefilter" && i + 1 < argc) dataset_prefilter = argv[++i];
        else if ((arg == "--output" || arg == "--jsonl") && i + 1 < argc) output = argv[++i];
        else if (arg == "--solution-candidates" && i + 1 < argc) ++i;
        else if (arg == "--help" || arg == "-h") {
            printHelp();
            rclcpp::shutdown();
            return 0;
        }
    }

    IkSolverOptions options;
    options.base_frame = "base_link";
    options.tip_link = "left_tool0";
    options.tip_link2 = "right_tool0";
    options.reject_collisions = false;

    std::unique_ptr<IkSolver> prefilter_ik;
    std::vector<std::string> prefilter_full_joint_names;
    if (dataset_prefilter == "ik") {
        prefilter_ik = std::make_unique<IkSolver>(group, solver, timeout, false, options);
        prefilter_full_joint_names = fullJointNamesForUpdownLookup(prefilter_ik->variableNames());
    }

    DatasetBuildStats dataset_stats;
    const auto all_episodes = makeEpisodes(profile, approach_offset, place_safe_z, base_only_dataset,
                                           h_lower, h_upper, tool0_offset, sphere_margin,
                                           h_step, h_candidate_limit, seed_attempts, seed_noise, timeout,
                                           reject_collisions, pos_threshold, ori_threshold, dataset_prefilter,
                                           prefilter_ik.get(), prefilter_full_joint_names, &dataset_stats);
    std::map<std::string, std::vector<Episode>> episodes_by_difficulty;
    for (const auto& episode : all_episodes) {
        auto& bucket = episodes_by_difficulty[episode.difficulty];
        if (limit_episodes == 0 || bucket.size() < limit_episodes) {
            bucket.push_back(episode);
        }
    }

    const std::vector<size_t> workers_list = parseWorkersList(workers_list_text);
    std::ofstream ofs(output);
    if (!ofs.good()) {
        std::cerr << "Cannot write to " << output << "\n";
        rclcpp::shutdown();
        return 1;
    }

    nlohmann::json header;
    header["type"] = "header";
    header["benchmark"] = "parallel_ik_benchmark";
    header["mode"] = "staged_episode";
    header["group"] = group;
    header["solver"] = solver;
    header["profile"] = profile;
    header["workers_list"] = workers_list;
    header["timeout"] = timeout;
    header["h_candidate_limit"] = h_candidate_limit;
    header["seed_attempts"] = seed_attempts;
    header["h_step"] = h_step;
    header["tool0_offset_compensation"] = tool0_offset;
    header["place_safe_z"] = place_safe_z;
    header["base_only_dataset"] = base_only_dataset;
    header["dataset_policy"] = "prefiltered_reachability_pools";
    header["dataset_prefilter"] = dataset_prefilter;
    header["dataset_candidate_episodes"] = dataset_stats.candidate_episodes;
    header["dataset_range_reachable_episodes"] = dataset_stats.range_reachable_episodes;
    header["dataset_range_unreachable_episodes"] = dataset_stats.range_unreachable_episodes;
    header["dataset_reachable_episodes"] = dataset_stats.reachable_episodes;
    header["dataset_ik_failed_episodes"] = dataset_stats.ik_failed_episodes;
    header["dataset_total_unreachable_episodes"] = dataset_stats.total_unreachable_episodes;
    header["dataset_success_episodes"] = dataset_stats.success_episodes;
    header["dataset_mixed_episodes"] = dataset_stats.mixed_episodes;
    header["dataset_mostly_fail_episodes"] = dataset_stats.mostly_fail_episodes;
    header["dataset_unreachable_profile_episodes"] = dataset_stats.unreachable_dataset_episodes;
    header["collision_rejection"] = reject_collisions;
    header["tip_order_policy"] = "demo_order_with_fk_direct_check";
    ofs << header.dump() << "\n";

    std::cout << "=== Parallel IK Benchmark (staged episode) ===\n"
              << "  Profile: " << profile << "\n"
              << "  Timeout: " << timeout << "s\n"
              << "  h candidates / seeds: " << h_candidate_limit << " / " << seed_attempts << "\n"
              << "  h step: " << h_step << "\n"
              << "  Workers: " << workers_list_text << "\n"
              << "  Collision reject: " << (reject_collisions ? "yes" : "no") << "\n"
              << "  Output: " << output << "\n\n";

    for (const auto& [difficulty, episodes] : episodes_by_difficulty) {
        nlohmann::json dataset_record;
        dataset_record["type"] = "dataset";
        dataset_record["difficulty"] = difficulty;
        dataset_record["episode_count"] = episodes.size();
        dataset_record["stage_count"] = episodes.size() * 5;
        dataset_record["episodes"] = nlohmann::json::array();
        for (const auto& episode : episodes) {
            dataset_record["episodes"].push_back({{"id", episode.id}, {"profile", episode.profile}, {"stage_count", episode.stages.size()}});
        }
        ofs << dataset_record.dump() << "\n";

        std::cout << "Dataset " << difficulty << ": episodes=" << episodes.size()
                  << " stages=" << episodes.size() * 5 << "\n";

        double baseline_wall_ms = 0.0;
        for (size_t workers : workers_list) {
            const auto init_t0 = std::chrono::steady_clock::now();
            std::vector<std::unique_ptr<IkSolver>> solvers;
            std::vector<std::vector<std::string>> full_joint_names_by_worker;
            solvers.reserve(workers);
            for (size_t worker = 0; worker < workers; ++worker) {
                solvers.push_back(std::make_unique<IkSolver>(group, solver, timeout, false, options));
                full_joint_names_by_worker.push_back(fullJointNamesForUpdownLookup(solvers.back()->variableNames()));
            }
            const auto init_t1 = std::chrono::steady_clock::now();
            const double init_ms = std::chrono::duration<double, std::milli>(init_t1 - init_t0).count();

            ReachSphere left_sphere;
            ReachSphere right_sphere;
            right_sphere.cy = -0.3125;
            RunSummary summary;
            summary.difficulty = difficulty;
            summary.workers = workers;
            summary.episode_count = episodes.size();
            summary.init_ms = init_ms;

            const auto run_t0 = std::chrono::steady_clock::now();
            for (const auto& episode : episodes) {
                double current_h = std::min(std::max(0.0, h_lower), h_upper);
                std::vector<double> seed(solvers.front()->variableNames().size(), 0.0);
                for (size_t stage_index = 0; stage_index < episode.stages.size(); ++stage_index) {
                    const Stage& stage = episode.stages[stage_index];
                    StageResult stage_result;
                    stage_result.episode_id = episode.id;
                    stage_result.difficulty = difficulty;
                    stage_result.stage_name = stage.name;
                    stage_result.stage_index = stage_index;
                    stage_result.workers = workers;
                    stage_result.previous_h = current_h;
                    stage_result.selected_h = current_h;
                    stage_result.height_plan = planHeights(stage.left, stage.right, left_sphere, right_sphere,
                                                           current_h, h_lower, h_upper, tool0_offset,
                                                           sphere_margin, h_step, h_candidate_limit);
                    stage_result.reachable = stage_result.height_plan.reachable;
                    summary.stage_count++;
                    if (!stage_result.reachable || stage_result.height_plan.candidates.empty()) {
                        stage_result.success = false;
                        summary.failed_stage_count++;
                        summary.range_out_stage_count++;
                        ofs << stageToJson(stage_result).dump() << "\n";
                        continue;
                    }

                    const std::vector<TrialSpec> trials = makeTrials(stage_result.height_plan,
                                                                     solvers.front()->variableNames(),
                                                                     seed, seed_attempts, seed_noise);
                    const auto stage_t0 = std::chrono::steady_clock::now();
                    std::vector<TrialResult> trial_results = runTrials(trials, stage, solvers, full_joint_names_by_worker,
                                                                       current_h, tool0_offset, timeout,
                                                                       reject_collisions, pos_threshold, ori_threshold);
                    const auto stage_t1 = std::chrono::steady_clock::now();
                    stage_result.wall_ms = std::chrono::duration<double, std::milli>(stage_t1 - stage_t0).count();
                    stage_result.trial_count = trial_results.size();

                    for (const auto& trial : trial_results) {
                        stage_result.sum_solve_ms += trial.solve_ms;
                        if (trial.legal) stage_result.legal_count++;
                        if (trial.timeout_like) stage_result.timeout_like_count++;
                        if (trial.swapped) stage_result.swapped_count++;
                    }

                    auto best = std::min_element(trial_results.begin(), trial_results.end(), [](const TrialResult& a, const TrialResult& b) {
                        if (a.legal != b.legal) return a.legal > b.legal;
                        return a.score < b.score;
                    });
                    if (best != trial_results.end()) {
                        stage_result.selected = *best;
                    }

                    stage_result.success = best != trial_results.end() && best->legal;
                    if (stage_result.success) {
                        stage_result.selected_h = best->h;
                        stage_result.updown_delta = std::abs(best->h - current_h);
                        current_h = best->h;
                        seed = best->joint_values;
                        summary.success_stage_count++;
                        summary.total_updown_motion += stage_result.updown_delta;
                    } else {
                        summary.failed_stage_count++;
                    }
                    summary.trial_count += stage_result.trial_count;
                    summary.legal_count += stage_result.legal_count;
                    summary.timeout_like_count += stage_result.timeout_like_count;
                    summary.swapped_rejected_count += stage_result.swapped_count;
                    summary.sum_solve_ms += stage_result.sum_solve_ms;
                    ofs << stageToJson(stage_result).dump() << "\n";
                    if (include_candidates) {
                        for (const auto& trial : trial_results) {
                            nlohmann::json trial_json = trialToJson(trial);
                            trial_json["type"] = "trial";
                            trial_json["episode_id"] = episode.id;
                            trial_json["difficulty"] = difficulty;
                            trial_json["stage"] = stage.name;
                            trial_json["workers"] = workers;
                            ofs << trial_json.dump() << "\n";
                        }
                    }
                }
            }
            const auto run_t1 = std::chrono::steady_clock::now();
            summary.wall_ms = std::chrono::duration<double, std::milli>(run_t1 - run_t0).count();
            if (workers == 1 || baseline_wall_ms <= 0.0) baseline_wall_ms = summary.wall_ms;
            ofs << summaryToJson(summary, baseline_wall_ms).dump() << "\n";

            std::cout << "  workers=" << std::setw(2) << workers
                      << " init=" << std::fixed << std::setprecision(1) << summary.init_ms << "ms"
                      << " wall=" << summary.wall_ms << "ms"
                      << " speedup=" << std::setprecision(2) << (baseline_wall_ms / summary.wall_ms) << "x"
                      << " success_stages=" << summary.success_stage_count << "/" << summary.stage_count
                      << " trials=" << summary.trial_count
                      << " legal=" << summary.legal_count
                      << " timeout=" << summary.timeout_like_count
                      << " updown=" << summary.total_updown_motion << "\n";
        }
        std::cout << "\n";
    }

    std::cout << "Results saved to: " << output << "\n";
    rclcpp::shutdown();
    return 0;
}
