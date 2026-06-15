#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace ik_benchmark {
namespace {

std::vector<double> zeros(size_t n)
{
    return std::vector<double>(n, 0.0);
}

size_t nextAttemptIndex(size_t h_index, size_t seed_index, size_t seed_count, size_t repeat_index = 0)
{
    const size_t safe_seed_count = std::max<size_t>(1, seed_count);
    return seed_index + h_index * safe_seed_count + repeat_index * 1000003u;
}

template <typename Trial>
std::string trialKey(const std::vector<std::string>& names, const Trial& trial)
{
    std::ostringstream out;
    out.precision(17);
    out << (trial.free_updown ? "free" : "fixed") << '|'
        << trial.h_range_lower << '|' << trial.h_range_upper << '|';
    for (size_t i = 0; i < names.size() && i < trial.seed.size(); ++i) {
        out << names[i] << '=' << trial.seed[i] << ';';
    }
    return out.str();
}

template <typename Trial>
void pushUniqueTrial(std::vector<Trial>& trials,
                     const std::vector<std::string>& names,
                     Trial trial)
{
    const std::string key = trialKey(names, trial);
    for (const auto& existing : trials) {
        if (trialKey(names, existing) == key) return;
    }
    trials.push_back(std::move(trial));
}

double angularDistance(double a, double b)
{
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

} // namespace

ParallelUpdownAwareIkSolver::ParallelUpdownAwareIkSolver(UpdownAwareIkConfig config,
                                                         UpdownAwareCostFn custom_cost)
    : config_(std::move(config)), custom_cost_(std::move(custom_cost))
{
    if (config_.workers == 0) {
        config_.workers = 1;
    }
    IkSolverOptions fixed_options = config_.solver_options;
    IkSolverOptions free_options = config_.solver_options;
    fixed_options.base_frame = "updown";
    free_options.base_frame = config_.base_frame;
    if (fixed_options.tip_link.empty()) {
        fixed_options.tip_link = config_.left_tip;
    }
    if (fixed_options.tip_link2.empty()) {
        fixed_options.tip_link2 = config_.right_tip;
    }
    if (free_options.tip_link.empty()) {
        free_options.tip_link = config_.left_tip;
    }
    if (free_options.tip_link2.empty()) {
        free_options.tip_link2 = config_.right_tip;
    }
    fixed_options.reject_collisions = false;
    fixed_options.enforce_arm_base_collisions = config_.enforce_arm_base_collisions;
    free_options.reject_collisions = false;
    free_options.enforce_arm_base_collisions = config_.enforce_arm_base_collisions;

    fixed_solvers_.reserve(config_.workers);
    free_solvers_.reserve(config_.workers);
    for (size_t i = 0; i < config_.workers; ++i) {
        fixed_solvers_.push_back(std::make_unique<IkSolver>(
            config_.fixed_group, config_.solver_plugin, config_.timeout, false, fixed_options));
        free_solvers_.push_back(std::make_unique<IkSolver>(
            config_.free_group, config_.solver_plugin, config_.fallback_timeout, false, free_options));
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
        const size_t fallback_rounds = std::max<size_t>(1, config_.fallback_rounds);
        for (size_t round_index = 0; round_index < fallback_rounds && !result.success; ++round_index) {
            auto fallback_candidates = executeTrials(makeFallbackTrials(request, plan, round_index), request, plan, true);
            result.candidates.insert(result.candidates.end(), fallback_candidates.begin(), fallback_candidates.end());
            sortAndSelect(result, request);
        }
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
    const Eigen::Isometry3d& target, UpdownAwareIkRequest::GraspMode grasp_mode) const
{
    HeightInterval interval;
    const bool top_suction = grasp_mode == UpdownAwareIkRequest::GraspMode::TopSuction;
    const double configured_lower = top_suction ? config_.top_suction_z_reach_lower : config_.gripper_z_reach_lower;
    const double configured_upper = top_suction ? config_.top_suction_z_reach_upper : config_.gripper_z_reach_upper;
    const double reach_lower = std::min(configured_lower, configured_upper);
    const double reach_upper = std::max(configured_lower, configured_upper);
    if (reach_upper < reach_lower) {
        return interval;
    }

    const double target_z = target.translation().z();
    interval.lower = std::max(config_.h_lower, target_z - reach_upper);
    interval.upper = std::min(config_.h_upper, target_z - reach_lower);
    interval.reachable = interval.lower <= interval.upper;
    return interval;
}

ParallelUpdownAwareIkSolver::HeightPlan ParallelUpdownAwareIkSolver::planHeight(
    const UpdownAwareIkRequest& request) const
{
    HeightPlan plan;
    plan.left = intervalForTarget(request.left_target, request.grasp_mode);
    plan.right = intervalForTarget(request.right_target, request.grasp_mode);
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

    auto add_unique = [&](double value) {
        if (candidates.size() >= config_.h_candidate_count) return;
        const double clamped = std::min(std::max(value, interval.lower), interval.upper);
        for (double existing : candidates) {
            if (std::abs(existing - clamped) < 1e-9) return;
        }
        candidates.push_back(clamped);
    };

    add_unique(h_center);

    const double margin = std::max(0.0, config_.h_search_margin);
    if (margin <= 0.0 || config_.h_candidate_count == 1) {
        return candidates;
    }

    double window_lower = h_center - margin;
    double window_upper = h_center + margin;
    if (window_lower < interval.lower) {
        window_upper = std::min(interval.upper, window_upper + (interval.lower - window_lower));
        window_lower = interval.lower;
    }
    if (window_upper > interval.upper) {
        window_lower = std::max(interval.lower, window_lower - (window_upper - interval.upper));
        window_upper = interval.upper;
    }

    if (window_upper <= window_lower + 1e-12) {
        return candidates;
    }

    add_unique(window_upper);
    add_unique(window_lower);

    std::vector<double> window_samples;
    window_samples.reserve(config_.h_candidate_count);
    const size_t sample_count = config_.h_candidate_count;
    for (size_t i = 0; i < sample_count; ++i) {
        const double ratio = sample_count == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(sample_count - 1);
        window_samples.push_back(window_lower + ratio * (window_upper - window_lower));
    }
    std::sort(window_samples.begin(), window_samples.end(), [h_center](double lhs, double rhs) {
        const double lhs_distance = std::abs(lhs - h_center);
        const double rhs_distance = std::abs(rhs - h_center);
        if (std::abs(lhs_distance - rhs_distance) > 1e-12) {
            return lhs_distance < rhs_distance;
        }
        return lhs < rhs;
    });
    for (double sample : window_samples) {
        add_unique(sample);
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
        const double range_width = std::min(2.0 * config_.h_search_margin, plan.combined.upper - plan.combined.lower);
        double range_lower = plan.h_center - 0.5 * range_width;
        double range_upper = plan.h_center + 0.5 * range_width;
        if (range_lower < plan.combined.lower) {
            range_upper = std::min(plan.combined.upper, range_upper + (plan.combined.lower - range_lower));
            range_lower = plan.combined.lower;
        }
        if (range_upper > plan.combined.upper) {
            range_lower = std::max(plan.combined.lower, range_lower - (range_upper - plan.combined.upper));
            range_upper = plan.combined.upper;
        }
        const size_t target_trial_count = std::max<size_t>(1, config_.h_candidate_count) * std::max<size_t>(1, config_.seed_count);
        const size_t fixed_current_count = std::min(std::max<size_t>(1, config_.seed_count), target_trial_count);
        size_t seed_index = 0;

        const bool has_fixed_current = request.current_h >= plan.combined.lower - 1e-9 && request.current_h <= plan.combined.upper + 1e-9;
        if (has_fixed_current) {
            for (size_t i = 0; i < fixed_current_count; ++i) {
                TrialSpec fixed_current;
                fixed_current.free_updown = false;
                fixed_current.h = request.current_h;
                fixed_current.h_range_lower = request.current_h;
                fixed_current.h_range_upper = request.current_h;
                fixed_current.h_index = 0;
                fixed_current.seed_index = seed_index++;
                fixed_current.solver_path = "fixed_current_h";
                fixed_current.seed = i == 0
                    ? base_seed
                    : makePerturbedSeed(fixed_names, base_seed, 50000 + i, config_.seed_noise, 0.0);
                pushUniqueTrial(trials, fixed_names, std::move(fixed_current));
            }
        }

        const size_t continuous_trials = target_trial_count > seed_index ? target_trial_count - seed_index : 0;
        for (size_t i = 0; i < continuous_trials; ++i) {
            TrialSpec trial;
            trial.free_updown = true;
            trial.h = plan.h_center;
            trial.h_range_lower = range_lower;
            trial.h_range_upper = range_upper;
            trial.h_index = 1;
            trial.seed_index = seed_index++;
            trial.solver_path = "continuous_h_range";
            const double seed_h = std::min(std::max(plan.h_center, range_lower), range_upper);
            trial.seed = makeFullSeedFromArmSeed(seed_h, base_seed);
            trial.seed = makePerturbedSeed(freeVariableNames(), trial.seed, 60000 + i,
                                           i == 0 ? 0.0 : config_.seed_noise,
                                           i == 0 ? 0.0 : config_.h_search_margin);
            for (size_t k = 0; k < freeVariableNames().size() && k < trial.seed.size(); ++k) {
                if (freeVariableNames()[k] == "updown") {
                    trial.seed[k] = std::min(std::max(trial.seed[k], range_lower), range_upper);
                }
            }
            pushUniqueTrial(trials, freeVariableNames(), std::move(trial));
        }
        return trials;
    }

    const size_t seed_count = std::max<size_t>(1, config_.seed_count);
    const size_t target_trial_count = std::max<size_t>(1, config_.h_candidate_count) * seed_count;
    size_t trial_count = 0;
    size_t repeat_index = 0;
    while (!plan.candidates.empty() && trial_count < target_trial_count) {
        for (size_t h_index = 0; h_index < plan.candidates.size() && trial_count < target_trial_count; ++h_index) {
            for (size_t seed_index = 0; seed_index < seed_count && trial_count < target_trial_count; ++seed_index) {
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
                trial.seed = seed_index == 0 && repeat_index == 0
                    ? base_seed
                    : makePerturbedSeed(fixed_names, base_seed, nextAttemptIndex(h_index, seed_index, seed_count, repeat_index),
                                        config_.seed_noise, 0.0);
                const size_t before = trials.size();
                pushUniqueTrial(trials, fixed_names, std::move(trial));
                if (trials.size() > before) ++trial_count;
            }
        }
        ++repeat_index;
    }
    return trials;
}

std::vector<std::vector<double>> ParallelUpdownAwareIkSolver::makeFallbackSeedFamilies(
    const UpdownAwareIkRequest& request, size_t round_index) const
{
    const auto& free_names = freeVariableNames();
    std::vector<std::vector<double>> families;
    auto add_unique_family = [&](const std::vector<double>& seed) {
        if (seed.size() != free_names.size()) return;
        for (const auto& existing : families) {
            if (existing.size() != seed.size()) continue;
            bool same = true;
            for (size_t i = 0; i < seed.size(); ++i) {
                if (std::abs(existing[i] - seed[i]) > 1e-12) {
                    same = false;
                    break;
                }
            }
            if (same) return;
        }
        families.push_back(seed);
    };

    std::vector<double> last_success_seed = !request.current_full_joints.empty()
        ? request.current_full_joints
        : makeFullSeedFromArmSeed(request.current_h, request.current_arm_joints);
    if (last_success_seed.size() != free_names.size()) {
        last_success_seed = makeFullSeedFromArmSeed(request.current_h, request.current_arm_joints);
    }
    add_unique_family(last_success_seed);

    std::vector<double> home_seed(free_names.size(), 0.0);
    for (size_t i = 0; i < free_names.size(); ++i) {
        if (free_names[i] == "updown") {
            home_seed[i] = std::min(std::max(request.current_h, config_.h_lower), config_.h_upper);
        }
    }
    add_unique_family(home_seed);

    const size_t random_family_count = std::max<size_t>(1, config_.fallback_random_family_count);
    for (size_t family_index = 0; family_index < random_family_count; ++family_index) {
        const auto& base = family_index % 2 == 0 ? last_success_seed : home_seed;
        add_unique_family(makePerturbedSeed(free_names, base,
                                            200000 + round_index * 1009 + family_index,
                                            config_.fallback_seed_noise,
                                            config_.fallback_updown_noise));
    }
    return families;
}

std::vector<ParallelUpdownAwareIkSolver::TrialSpec> ParallelUpdownAwareIkSolver::makeFallbackTrials(
    const UpdownAwareIkRequest& request, const HeightPlan& plan, size_t round_index) const
{
    std::vector<TrialSpec> trials;
    const auto seed_families = makeFallbackSeedFamilies(request, round_index);
    const size_t target_trial_count = std::max<size_t>(1, config_.fallback_seed_count);
    size_t seed_index = 0;
    for (size_t attempt = 0; seed_index < target_trial_count; ++attempt) {
        for (size_t family_index = 0; family_index < seed_families.size() && seed_index < target_trial_count; ++family_index) {
            const auto& family_seed = seed_families[family_index];
            TrialSpec trial;
            trial.free_updown = true;
            trial.h = plan.reachable ? plan.h_center : request.current_h;
            trial.h_range_lower = config_.h_lower;
            trial.h_range_upper = config_.h_upper;
            trial.h_index = round_index;
            trial.seed_index = seed_index;
            trial.solver_path = "global_free_h_fallback";
            trial.seed = attempt == 0
                ? family_seed
                : makePerturbedSeed(freeVariableNames(), family_seed,
                                    300000 + round_index * 100003 + family_index * target_trial_count + attempt,
                                    config_.fallback_seed_noise,
                                    config_.fallback_updown_noise);
            const size_t before = trials.size();
            pushUniqueTrial(trials, freeVariableNames(), std::move(trial));
            if (trials.size() > before) ++seed_index;
        }
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
                const bool swapped_order = config_.try_target_orders
                    ? order_index == 1
                    : config_.use_reversed_target_order;
                const TrialSpec& trial = trials[trial_index];
                IkSolver& solver = trial.free_updown
                    ? *free_solvers_[worker % free_solvers_.size()]
                    : *fixed_solvers_[worker % fixed_solvers_.size()];
                results[item] = solveTrial(solver, trial, request, plan, swapped_order, fallback);
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
    out.h_range_lower = trial.h_range_lower;
    out.h_range_upper = trial.h_range_upper;
    out.h_index = trial.h_index;
    out.seed_index = trial.seed_index;
    out.solver_path = trial.solver_path;
    out.target_order = swapped_order ? "swapped" : "normal";

    const Eigen::Isometry3d left_target = trial.free_updown
        ? compensateTool0(request.left_target)
        : fixedTarget(request.left_target, trial.h);
    const Eigen::Isometry3d right_target = trial.free_updown
        ? compensateTool0(request.right_target)
        : fixedTarget(request.right_target, trial.h);

    const double timeout = fallback ? config_.fallback_timeout : config_.timeout;
    const std::vector<double> solve_seed = trial.seed;
    IkResult result = swapped_order
        ? solver.solveDual(right_target, left_target, solve_seed, timeout)
        : solver.solveDual(left_target, right_target, solve_seed, timeout);
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
    } else {
        out.full_joint_names = fullJointNamesForFixedGroup();
        out.full_joint_values = fullJointValuesForFixedGroup(trial.h, result.joint_values);
        out.h = trial.h;
    }
    actual_poses = solver.fk(result.joint_values);
    if (!trial.free_updown) {
        const Eigen::Isometry3d updown_in_base = updownTransformInBase(trial.h);
        for (auto& actual_pose : actual_poses) {
            actual_pose = updown_in_base * actual_pose;
        }
    }
    if (out.h < trial.h_range_lower - 1e-9 || out.h > trial.h_range_upper + 1e-9) {
        out.rejection_reason = "updown_out_of_search_range";
        return out;
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
    const bool top_suction = request.grasp_mode == UpdownAwareIkRequest::GraspMode::TopSuction;
    out.direct_ori_error = top_suction
        ? std::max(toolAxisError(request.left_target, actual_poses[0]),
                   toolAxisError(request.right_target, actual_poses[1]))
        : std::max(orientationError(request.left_target, actual_poses[0]),
                   orientationError(request.right_target, actual_poses[1]));
    out.swapped = out.swapped_pos_error + 1e-4 < out.direct_pos_error;
    if (config_.reject_swapped_tips && out.swapped) {
        out.rejection_reason = "tip_order_error";
        return out;
    }
    const double position_tolerance = request.grasp_mode == UpdownAwareIkRequest::GraspMode::TopSuction
        ? config_.top_suction_position_tolerance
        : config_.position_tolerance;
    const double orientation_tolerance = request.grasp_mode == UpdownAwareIkRequest::GraspMode::TopSuction
        ? config_.top_suction_orientation_tolerance
        : config_.orientation_tolerance;
    if (config_.check_tip_error &&
        (out.direct_pos_error > position_tolerance || out.direct_ori_error > orientation_tolerance)) {
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

Eigen::Isometry3d ParallelUpdownAwareIkSolver::updownTransformInBase(double h) const
{
    return free_solvers_.front()->linkTransformNamed("updown", {"updown"}, {h});
}

Eigen::Isometry3d ParallelUpdownAwareIkSolver::fixedTarget(
    const Eigen::Isometry3d& target, double h) const
{
    return updownTransformInBase(h).inverse() * compensateTool0(target);
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

double ParallelUpdownAwareIkSolver::toolAxisError(
    const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const
{
    const Eigen::Vector3d target_axis = target.linear().col(2).normalized();
    const Eigen::Vector3d actual_axis = actual.linear().col(2).normalized();
    const double dot = std::clamp(target_axis.dot(actual_axis), -1.0, 1.0);
    return std::acos(dot);
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

double ParallelUpdownAwareIkSolver::loadedPoseDistance(
    const UpdownAwareIkCandidate& candidate,
    const std::string& prefix,
    const std::vector<double>& pose) const
{
    if (pose.size() < 6) {
        return 0.0;
    }
    double squared_sum = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        const std::string joint_name = prefix + "_v5_joint" + std::to_string(i + 1);
        const double diff = angularDistance(jointValue(candidate, joint_name), pose[i]);
        squared_sum += diff * diff;
    }
    return std::sqrt(squared_sum);
}

double ParallelUpdownAwareIkSolver::loadedPoseFamilyMinDistance(
    const UpdownAwareIkCandidate& candidate,
    const std::string& prefix,
    const std::vector<std::vector<double>>& family) const
{
    if (family.empty()) {
        return 0.0;
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& pose : family) {
        if (pose.size() < 6) {
            continue;
        }
        best = std::min(best, loadedPoseDistance(candidate, prefix, pose));
    }
    return std::isfinite(best) ? best : 0.0;
}

double ParallelUpdownAwareIkSolver::loadedPosePreferredDistance(
    const UpdownAwareIkCandidate& candidate,
    const std::string& prefix,
    const std::vector<std::vector<double>>& family,
    size_t preferred_index) const
{
    if (preferred_index >= family.size() || family[preferred_index].size() < 6) {
        return 0.0;
    }
    return loadedPoseDistance(candidate, prefix, family[preferred_index]);
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
    if (config_.cost_loaded_family_distance != 0.0) {
        score += config_.cost_loaded_family_distance *
                 (loadedPoseFamilyMinDistance(candidate, "left", config_.left_loaded_pose_family) +
                  loadedPoseFamilyMinDistance(candidate, "right", config_.right_loaded_pose_family));
    }
    if (config_.cost_loaded_preferred_distance != 0.0) {
        score += config_.cost_loaded_preferred_distance *
                 (loadedPosePreferredDistance(candidate, "left", config_.left_loaded_pose_family,
                                              config_.left_preferred_loaded_pose_index) +
                  loadedPosePreferredDistance(candidate, "right", config_.right_loaded_pose_family,
                                              config_.right_preferred_loaded_pose_index));
    }
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
