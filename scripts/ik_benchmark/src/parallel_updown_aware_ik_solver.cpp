#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace ik_benchmark {
namespace {

std::vector<double> zeros(size_t n)
{
    return std::vector<double>(n, 0.0);
}

} // namespace

ParallelUpdownAwareIkSolver::ParallelUpdownAwareIkSolver(UpdownAwareIkConfig config,
                                                         UpdownAwareCostFn custom_cost)
    : config_(std::move(config)), custom_cost_(std::move(custom_cost))
{
    if (config_.workers == 0) {
        config_.workers = 1;
    }
    if (config_.solver_options.base_frame.empty()) {
        config_.solver_options.base_frame = config_.base_frame;
    }
    if (config_.solver_options.tip_link.empty()) {
        config_.solver_options.tip_link = config_.left_tip;
    }
    if (config_.solver_options.tip_link2.empty()) {
        config_.solver_options.tip_link2 = config_.right_tip;
    }
    config_.solver_options.reject_collisions = false;

    fixed_solvers_.reserve(config_.workers);
    free_solvers_.reserve(config_.workers);
    for (size_t i = 0; i < config_.workers; ++i) {
        fixed_solvers_.push_back(std::make_unique<IkSolver>(
            config_.fixed_group, config_.solver_plugin, config_.timeout, false, config_.solver_options));
        free_solvers_.push_back(std::make_unique<IkSolver>(
            config_.free_group, config_.solver_plugin, config_.fallback_timeout, false, config_.solver_options));
    }
}

const std::vector<std::string>& ParallelUpdownAwareIkSolver::fixedVariableNames() const
{
    return fixed_solvers_.front()->variableNames();
}

const std::vector<std::string>& ParallelUpdownAwareIkSolver::freeVariableNames() const
{
    return free_solvers_.front()->variableNames();
}

UpdownAwareIkResult ParallelUpdownAwareIkSolver::solve(const UpdownAwareIkRequest& request)
{
    const auto t0 = std::chrono::steady_clock::now();
    UpdownAwareIkResult result;
    const HeightPlan plan = planHeight(request);
    result.range_reachable = plan.reachable;
    result.h_interval_lower = plan.combined.lower;
    result.h_interval_upper = plan.combined.upper;
    result.h_center = plan.h_center;
    result.h_candidates = plan.candidates;

    std::vector<UpdownAwareIkCandidate> candidates;
    if (plan.reachable && std::abs(plan.h_center - request.current_h) <= config_.max_updown_delta) {
        candidates = executeTrials(makeNormalTrials(request, plan), request, plan, false);
    } else {
        result.failure_reason = !plan.reachable ? "h_interval_unreachable" : "h_center_exceeds_motion_limit";
    }

    result.candidates = candidates;
    sortAndSelect(result, request);

    if (!result.success && config_.fallback_enabled && shouldUseFallback(plan, candidates)) {
        result.fallback_used = true;
        auto fallback_candidates = executeTrials(makeFallbackTrials(request, plan), request, plan, true);
        result.candidates.insert(result.candidates.end(), fallback_candidates.begin(), fallback_candidates.end());
        sortAndSelect(result, request);
        if (!result.success && result.failure_reason.empty()) {
            result.failure_reason = "fallback_failed";
        }
    }

    for (const auto& candidate : result.candidates) {
        result.trial_count++;
        result.sum_solve_ms += candidate.solve_ms;
        if (candidate.legal) result.legal_count++;
        if (candidate.timeout_like) result.timeout_like_count++;
        if (candidate.swapped) result.swapped_rejected_count++;
    }

    if (result.success) {
        result.failure_reason.clear();
        result.solver_path = result.selected.solver_path;
    } else if (result.failure_reason.empty()) {
        result.failure_reason = result.range_reachable ? "no_legal_solution" : "h_interval_unreachable";
    }

    const auto t1 = std::chrono::steady_clock::now();
    result.wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

ParallelUpdownAwareIkSolver::HeightInterval ParallelUpdownAwareIkSolver::intervalForTarget(
    const Eigen::Isometry3d& target, const ReachSphereConfig& sphere) const
{
    const double radius = std::max(0.0, sphere.radius - config_.sphere_margin);
    const double dx = target.translation().x() - sphere.cx;
    const double dy = target.translation().y() - sphere.cy;
    const double dxy2 = dx * dx + dy * dy;
    const double r2 = radius * radius;
    HeightInterval interval;
    if (dxy2 > r2) {
        return interval;
    }
    const double z_margin = std::sqrt(std::max(0.0, r2 - dxy2));
    const double ik_target_z = target.translation().z() - config_.tool0_offset;
    interval.lower = std::max(config_.h_lower, ik_target_z - sphere.cz - z_margin);
    interval.upper = std::min(config_.h_upper, ik_target_z - sphere.cz + z_margin);
    interval.reachable = interval.lower <= interval.upper;
    return interval;
}

ParallelUpdownAwareIkSolver::HeightPlan ParallelUpdownAwareIkSolver::planHeight(
    const UpdownAwareIkRequest& request) const
{
    HeightPlan plan;
    plan.left = intervalForTarget(request.left_target, config_.left_reach_sphere);
    plan.right = intervalForTarget(request.right_target, config_.right_reach_sphere);
    plan.combined.lower = std::max(plan.left.lower, plan.right.lower);
    plan.combined.upper = std::min(plan.left.upper, plan.right.upper);
    plan.combined.reachable = plan.left.reachable && plan.right.reachable &&
                              plan.combined.lower <= plan.combined.upper;
    plan.reachable = plan.combined.reachable;
    if (!plan.reachable) {
        return plan;
    }
    plan.h_center = std::min(std::max(request.current_h, plan.combined.lower), plan.combined.upper);
    plan.candidates = makeFixedHCandidates(plan.combined, plan.h_center);
    return plan;
}

std::vector<double> ParallelUpdownAwareIkSolver::makeFixedHCandidates(
    const HeightInterval& interval, double h_center) const
{
    std::vector<double> candidates;
    if (!interval.reachable || config_.h_candidate_count == 0) {
        return candidates;
    }
    auto add = [&](double value) {
        if (candidates.size() >= config_.h_candidate_count) return;
        const double clamped = std::min(std::max(value, interval.lower), interval.upper);
        for (double existing : candidates) {
            if (std::abs(existing - clamped) < 1e-9) return;
        }
        candidates.push_back(clamped);
    };
    add(h_center);
    if (config_.h_step <= 0.0) {
        return candidates;
    }
    for (size_t ring = 1; candidates.size() < config_.h_candidate_count; ++ring) {
        const double delta = config_.h_step * static_cast<double>(ring);
        bool added = false;
        if (h_center - delta >= interval.lower - 1e-9) {
            add(h_center - delta);
            added = true;
        }
        if (h_center + delta <= interval.upper + 1e-9) {
            add(h_center + delta);
            added = true;
        }
        if (!added && h_center - delta < interval.lower && h_center + delta > interval.upper) {
            break;
        }
    }
    return candidates;
}

std::vector<ParallelUpdownAwareIkSolver::TrialSpec> ParallelUpdownAwareIkSolver::makeNormalTrials(
    const UpdownAwareIkRequest& request, const HeightPlan& plan) const
{
    std::vector<TrialSpec> trials;
    const auto& fixed_names = fixedVariableNames();
    std::vector<double> base_seed = request.current_arm_joints.empty()
        ? zeros(fixed_names.size())
        : request.current_arm_joints;
    if (base_seed.size() != fixed_names.size()) {
        base_seed = zeros(fixed_names.size());
    }

    if (config_.h_search_mode == UpdownAwareIkConfig::HSearchMode::ContinuousRange) {
        const double range_lower = std::max(config_.h_lower, plan.h_center - config_.h_search_margin);
        const double range_upper = std::min(config_.h_upper, plan.h_center + config_.h_search_margin);
        if (request.current_h >= plan.combined.lower - 1e-9 && request.current_h <= plan.combined.upper + 1e-9) {
            TrialSpec fixed_current;
            fixed_current.free_updown = false;
            fixed_current.h = request.current_h;
            fixed_current.h_range_lower = request.current_h;
            fixed_current.h_range_upper = request.current_h;
            fixed_current.h_index = 0;
            fixed_current.seed_index = 0;
            fixed_current.solver_path = "fixed_current_h";
            fixed_current.seed = base_seed;
            trials.push_back(std::move(fixed_current));
        }
        for (size_t i = 0; i < std::max<size_t>(1, config_.seed_count); ++i) {
            TrialSpec trial;
            trial.free_updown = true;
            trial.h = plan.h_center;
            trial.h_range_lower = range_lower;
            trial.h_range_upper = range_upper;
            trial.h_index = 0;
            trial.seed_index = i;
            trial.solver_path = "continuous_h_range";
            const double seed_h = range_lower + (range_upper - range_lower) *
                (static_cast<double>(i + 1) / static_cast<double>(std::max<size_t>(2, config_.seed_count + 1)));
            trial.seed = makeFullSeedFromArmSeed(seed_h, base_seed);
            if (i > 0) {
                trial.seed = makePerturbedSeed(freeVariableNames(), trial.seed, i, config_.seed_noise, config_.h_search_margin);
                if (!trial.seed.empty()) {
                    for (size_t k = 0; k < freeVariableNames().size(); ++k) {
                        if (freeVariableNames()[k] == "updown") {
                            trial.seed[k] = std::min(std::max(trial.seed[k], range_lower), range_upper);
                        }
                    }
                }
            }
            trials.push_back(std::move(trial));
        }
        return trials;
    }

    for (size_t h_index = 0; h_index < plan.candidates.size(); ++h_index) {
        for (size_t seed_index = 0; seed_index < std::max<size_t>(1, config_.seed_count); ++seed_index) {
            TrialSpec trial;
            trial.free_updown = false;
            trial.h = plan.candidates[h_index];
            trial.h_range_lower = trial.h;
            trial.h_range_upper = trial.h;
            trial.h_index = h_index;
            trial.seed_index = seed_index;
            trial.solver_path = h_index == 0 && std::abs(trial.h - request.current_h) < 1e-9
                ? "fixed_current_h"
                : "fixed_h_candidates";
            trial.seed = seed_index == 0
                ? base_seed
                : makePerturbedSeed(fixed_names, base_seed, seed_index + h_index * config_.seed_count,
                                    config_.seed_noise, 0.0);
            trials.push_back(std::move(trial));
        }
    }
    return trials;
}

std::vector<ParallelUpdownAwareIkSolver::TrialSpec> ParallelUpdownAwareIkSolver::makeFallbackTrials(
    const UpdownAwareIkRequest& request, const HeightPlan& plan) const
{
    std::vector<TrialSpec> trials;
    std::vector<double> base_seed = !request.current_full_joints.empty()
        ? request.current_full_joints
        : makeFullSeedFromArmSeed(request.current_h, request.current_arm_joints);
    if (base_seed.size() != freeVariableNames().size()) {
        base_seed = makeFullSeedFromArmSeed(request.current_h, request.current_arm_joints);
    }
    const size_t attempts = std::max<size_t>(1, config_.fallback_seed_count);
    for (size_t i = 0; i < attempts; ++i) {
        TrialSpec trial;
        trial.free_updown = true;
        trial.h = plan.reachable ? plan.h_center : request.current_h;
        trial.h_range_lower = config_.h_lower;
        trial.h_range_upper = config_.h_upper;
        trial.seed_index = i;
        trial.solver_path = "release_updown_fallback";
        trial.seed = i == 0
            ? base_seed
            : makePerturbedSeed(freeVariableNames(), base_seed, 100000 + i,
                                config_.seed_noise, std::max(config_.h_step, config_.h_search_margin));
        trials.push_back(std::move(trial));
    }
    return trials;
}

std::vector<UpdownAwareIkCandidate> ParallelUpdownAwareIkSolver::executeTrials(
    const std::vector<TrialSpec>& trials,
    const UpdownAwareIkRequest& request,
    const HeightPlan& plan,
    bool fallback)
{
    std::vector<UpdownAwareIkCandidate> results(trials.size() * (config_.try_target_orders ? 2 : 1));
    std::atomic_size_t next{0};
    const size_t order_count = config_.try_target_orders ? 2 : 1;
    const size_t worker_count = std::max<size_t>(1, config_.workers);
    std::vector<std::thread> threads;
    threads.reserve(worker_count);

    for (size_t worker = 0; worker < worker_count; ++worker) {
        threads.emplace_back([&, worker]() {
            while (true) {
                const size_t item = next.fetch_add(1);
                if (item >= results.size()) break;
                const size_t trial_index = item / order_count;
                const size_t order_index = item % order_count;
                IkSolver& solver = trials[trial_index].free_updown
                    ? *free_solvers_[worker % free_solvers_.size()]
                    : *fixed_solvers_[worker % fixed_solvers_.size()];
                results[item] = solveTrial(solver, trials[trial_index], request, plan, order_index == 1, fallback);
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }
    return results;
}

UpdownAwareIkCandidate ParallelUpdownAwareIkSolver::solveTrial(
    IkSolver& solver,
    const TrialSpec& trial,
    const UpdownAwareIkRequest& request,
    const HeightPlan& plan,
    bool swapped_order,
    bool fallback) const
{
    UpdownAwareIkCandidate out;
    out.h = trial.h;
    out.h_center = plan.h_center;
    out.solver_path = fallback ? "release_updown_fallback" : trial.solver_path;
    out.target_order = swapped_order ? "swapped" : "normal";

    const Eigen::Isometry3d left_target = trial.free_updown
        ? compensateTool0(request.left_target)
        : fixedTarget(request.left_target, trial.h);
    const Eigen::Isometry3d right_target = trial.free_updown
        ? compensateTool0(request.right_target)
        : fixedTarget(request.right_target, trial.h);

    const double timeout = fallback ? config_.fallback_timeout : config_.timeout;
    IkResult result = swapped_order
        ? solver.solveDual(right_target, left_target, trial.seed, timeout)
        : solver.solveDual(left_target, right_target, trial.seed, timeout);
    out.solve_ms = result.solve_ms;
    out.timeout_like = result.solve_ms >= timeout * 1000.0 * 0.9;
    out.joint_names = result.joint_names;
    out.joint_values = result.joint_values;

    if (result.joint_values.empty()) {
        out.rejection_reason = "ik_failed_empty_solution";
        return out;
    }

    std::vector<Eigen::Isometry3d> actual_poses;
    if (trial.free_updown) {
        out.full_joint_names = freeVariableNames();
        out.full_joint_values = result.joint_values;
        out.h = extractUpdown(out.full_joint_names, out.full_joint_values, request.current_h);
        actual_poses = solver.fk(result.joint_values);
        if (out.h < trial.h_range_lower - 1e-9 || out.h > trial.h_range_upper + 1e-9) {
            out.rejection_reason = "updown_out_of_search_range";
            return out;
        }
    } else {
        out.full_joint_names = fullJointNamesForFixedGroup();
        out.full_joint_values = fullJointValuesForFixedGroup(trial.h, result.joint_values);
        actual_poses = solver.fkNamed(out.full_joint_names, out.full_joint_values);
    }

    if (actual_poses.size() < 2) {
        out.rejection_reason = "fk_missing_dual_tip";
        return out;
    }

    const double direct_left = positionError(request.left_target, actual_poses[0]);
    const double direct_right = positionError(request.right_target, actual_poses[1]);
    out.direct_pos_error = std::max(direct_left, direct_right);
    const double swapped_left = positionError(request.left_target, actual_poses[1]);
    const double swapped_right = positionError(request.right_target, actual_poses[0]);
    out.swapped_pos_error = std::max(swapped_left, swapped_right);
    out.direct_ori_error = std::max(orientationError(request.left_target, actual_poses[0]),
                                    orientationError(request.right_target, actual_poses[1]));
    out.swapped = out.swapped_pos_error + 1e-4 < out.direct_pos_error;
    if (config_.reject_swapped_tips && out.swapped) {
        out.rejection_reason = "tip_order_error";
        return out;
    }
    if (config_.check_tip_error &&
        (out.direct_pos_error > config_.position_tolerance || out.direct_ori_error > config_.orientation_tolerance)) {
        out.rejection_reason = "tip_error_too_large";
        return out;
    }

    out.collision_free = true;
    if (config_.check_collision) {
        out.collision_free = solver.isNamedStateCollisionFree(out.full_joint_names, out.full_joint_values, &out.collision_pairs);
        if (!out.collision_free) {
            out.rejection_reason = "full_state_collision";
            return out;
        }
    } else {
        solver.isNamedStateCollisionFree(out.full_joint_names, out.full_joint_values, &out.collision_pairs);
        out.collision_free = out.collision_pairs.empty();
    }

    out.updown_delta = std::abs(out.h - request.current_h);
    out.joint_delta = jointDelta(out, request);
    out.score = scoreCandidate(out, request);
    out.legal = true;
    return out;
}

Eigen::Isometry3d ParallelUpdownAwareIkSolver::compensateTool0(const Eigen::Isometry3d& target) const
{
    return target * Eigen::Translation3d(0.0, 0.0, -config_.tool0_offset);
}

Eigen::Isometry3d ParallelUpdownAwareIkSolver::fixedTarget(const Eigen::Isometry3d& target, double h) const
{
    Eigen::Isometry3d fixed = compensateTool0(target);
    fixed.translation().z() -= h;
    return fixed;
}

std::vector<double> ParallelUpdownAwareIkSolver::makePerturbedSeed(
    const std::vector<std::string>& variable_names,
    const std::vector<double>& base_seed,
    size_t attempt_index,
    double revolute_noise,
    double prismatic_noise) const
{
    std::vector<double> seed = base_seed;
    if (seed.size() != variable_names.size()) {
        seed.assign(variable_names.size(), 0.0);
    }
    std::mt19937 rng(static_cast<uint32_t>(0xC0FFEEu + 7919u * attempt_index));
    std::normal_distribution<double> revolute_dist(0.0, revolute_noise);
    std::normal_distribution<double> prismatic_dist(0.0, prismatic_noise);
    for (size_t i = 0; i < seed.size(); ++i) {
        if (variable_names[i] == "updown") {
            seed[i] = std::min(std::max(seed[i] + prismatic_dist(rng), config_.h_lower), config_.h_upper);
        } else if (variable_names[i].find("pitch") == std::string::npos) {
            seed[i] += revolute_dist(rng);
        }
    }
    return seed;
}

std::vector<double> ParallelUpdownAwareIkSolver::makeFullSeedFromArmSeed(double h, const std::vector<double>& arm_seed) const
{
    const auto& free_names = freeVariableNames();
    const auto& fixed_names = fixedVariableNames();
    std::unordered_map<std::string, double> arm_values;
    for (size_t i = 0; i < fixed_names.size() && i < arm_seed.size(); ++i) {
        arm_values[fixed_names[i]] = arm_seed[i];
    }
    std::vector<double> full(free_names.size(), 0.0);
    for (size_t i = 0; i < free_names.size(); ++i) {
        if (free_names[i] == "updown") {
            full[i] = h;
        } else if (auto it = arm_values.find(free_names[i]); it != arm_values.end()) {
            full[i] = it->second;
        }
    }
    return full;
}

std::vector<std::string> ParallelUpdownAwareIkSolver::fullJointNamesForFixedGroup() const
{
    std::vector<std::string> names;
    names.reserve(fixedVariableNames().size() + 1);
    names.push_back("updown");
    for (const auto& name : fixedVariableNames()) {
        names.push_back(name);
    }
    return names;
}

std::vector<double> ParallelUpdownAwareIkSolver::fullJointValuesForFixedGroup(
    double h, const std::vector<double>& arm_values) const
{
    std::vector<double> values;
    values.reserve(arm_values.size() + 1);
    values.push_back(h);
    values.insert(values.end(), arm_values.begin(), arm_values.end());
    return values;
}

double ParallelUpdownAwareIkSolver::extractUpdown(
    const std::vector<std::string>& names, const std::vector<double>& values, double fallback) const
{
    for (size_t i = 0; i < names.size() && i < values.size(); ++i) {
        if (names[i] == "updown") {
            return values[i];
        }
    }
    return fallback;
}

double ParallelUpdownAwareIkSolver::positionError(
    const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const
{
    return (target.translation() - actual.translation()).norm();
}

double ParallelUpdownAwareIkSolver::orientationError(
    const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const
{
    Eigen::AngleAxisd aa(target.linear().transpose() * actual.linear());
    return aa.angle();
}

double ParallelUpdownAwareIkSolver::jointDelta(
    const UpdownAwareIkCandidate& candidate, const UpdownAwareIkRequest& request) const
{
    if (request.current_arm_joints.empty()) {
        return 0.0;
    }
    std::unordered_map<std::string, double> current;
    const auto& fixed_names = fixedVariableNames();
    for (size_t i = 0; i < fixed_names.size() && i < request.current_arm_joints.size(); ++i) {
        current[fixed_names[i]] = request.current_arm_joints[i];
    }
    double sum = 0.0;
    size_t count = 0;
    for (size_t i = 0; i < candidate.full_joint_names.size() && i < candidate.full_joint_values.size(); ++i) {
        if (candidate.full_joint_names[i] == "updown") {
            continue;
        }
        if (auto it = current.find(candidate.full_joint_names[i]); it != current.end()) {
            const double diff = candidate.full_joint_values[i] - it->second;
            sum += diff * diff;
            ++count;
        }
    }
    return count == 0 ? 0.0 : std::sqrt(sum);
}

double ParallelUpdownAwareIkSolver::jointValue(
    const UpdownAwareIkCandidate& candidate, const std::string& name, double fallback) const
{
    for (size_t i = 0; i < candidate.full_joint_names.size() && i < candidate.full_joint_values.size(); ++i) {
        if (candidate.full_joint_names[i] == name) {
            return candidate.full_joint_values[i];
        }
    }
    return fallback;
}

double ParallelUpdownAwareIkSolver::armTorqueProxy(
    const UpdownAwareIkCandidate& candidate, const std::string& prefix) const
{
    const bool is_left = prefix == "left";
    const double q2 = jointValue(candidate, prefix + "_v5_joint2");
    const double q3 = jointValue(candidate, prefix + "_v5_joint3");
    const double q2_zero = is_left ? config_.left_joint2_horizontal_angle : config_.right_joint2_horizontal_angle;
    const double q3_zero = is_left ? config_.left_joint3_horizontal_angle : config_.right_joint3_horizontal_angle;

    const double shoulder_angle = q2 - q2_zero;
    const double elbow_angle = q2 + q3 - q2_zero - q3_zero;

    const double link2_moment = config_.link2_mass_proxy * 0.5 * config_.link2_length * std::abs(std::cos(shoulder_angle));
    const double link3_shoulder_moment = config_.link3_mass_proxy *
        (config_.link2_length * std::abs(std::cos(shoulder_angle)) +
         0.5 * config_.link3_length * std::abs(std::cos(elbow_angle)));
    const double payload_shoulder_moment = config_.payload_mass_proxy *
        (config_.link2_length * std::abs(std::cos(shoulder_angle)) +
         config_.link3_length * std::abs(std::cos(elbow_angle)));
    const double joint2_proxy = link2_moment + link3_shoulder_moment + payload_shoulder_moment;

    const double link3_elbow_moment = config_.link3_mass_proxy * 0.5 * config_.link3_length * std::abs(std::cos(elbow_angle));
    const double payload_elbow_moment = config_.payload_mass_proxy * config_.link3_length * std::abs(std::cos(elbow_angle));
    const double joint3_proxy = link3_elbow_moment + payload_elbow_moment;

    return config_.cost_joint2_torque * joint2_proxy + config_.cost_joint3_torque * joint3_proxy;
}

double ParallelUpdownAwareIkSolver::jointLeverProxy(
    const UpdownAwareIkCandidate& candidate, const std::string& prefix, int joint_index) const
{
    const bool is_left = prefix == "left";
    const double q2 = jointValue(candidate, prefix + "_v5_joint2");
    const double q3 = jointValue(candidate, prefix + "_v5_joint3");
    const double q2_zero = is_left ? config_.left_joint2_horizontal_angle : config_.right_joint2_horizontal_angle;
    const double q3_zero = is_left ? config_.left_joint3_horizontal_angle : config_.right_joint3_horizontal_angle;
    const double shoulder_angle = q2 - q2_zero;
    const double elbow_angle = q2 + q3 - q2_zero - q3_zero;

    if (joint_index == 2) {
        return config_.link2_length * std::abs(std::cos(shoulder_angle)) +
               config_.link3_length * std::abs(std::cos(elbow_angle));
    }
    if (joint_index == 3) {
        return config_.link3_length * std::abs(std::cos(elbow_angle));
    }
    return 0.0;
}

double ParallelUpdownAwareIkSolver::scoreCandidate(
    const UpdownAwareIkCandidate& candidate, const UpdownAwareIkRequest& request) const
{
    if (custom_cost_) {
        return custom_cost_(candidate, request);
    }

    double score = 0.0;
    if (candidate.updown_delta <= config_.updown_static_epsilon) {
        score -= config_.cost_updown_static_bonus;
    }
    if (candidate.updown_delta <= config_.updown_small_motion_threshold) {
        score -= config_.cost_updown_within_0p1_bonus;
    } else {
        score += config_.cost_updown_over_0p1_distance *
                 (candidate.updown_delta - config_.updown_small_motion_threshold);
    }

    score += armTorqueProxy(candidate, "left");
    score += armTorqueProxy(candidate, "right");
    score += config_.cost_solve_ms * candidate.solve_ms;
    return score;
}

void ParallelUpdownAwareIkSolver::sortAndSelect(
    UpdownAwareIkResult& result, const UpdownAwareIkRequest&) const
{
    auto best = std::min_element(result.candidates.begin(), result.candidates.end(),
        [](const UpdownAwareIkCandidate& a, const UpdownAwareIkCandidate& b) {
            if (a.legal != b.legal) return a.legal > b.legal;
            return a.score < b.score;
        });
    if (best != result.candidates.end()) {
        result.selected = *best;
        result.success = best->legal;
    }
}

bool ParallelUpdownAwareIkSolver::shouldUseFallback(
    const HeightPlan& plan, const std::vector<UpdownAwareIkCandidate>& candidates) const
{
    if (!config_.fallback_enabled) {
        return false;
    }
    if (!plan.reachable || candidates.empty()) {
        return true;
    }
    return std::none_of(candidates.begin(), candidates.end(), [](const UpdownAwareIkCandidate& c) { return c.legal; });
}

} // namespace ik_benchmark
