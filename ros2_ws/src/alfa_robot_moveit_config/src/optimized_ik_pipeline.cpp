#include "alfa_robot_moveit_config/optimized_ik_pipeline.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace alfa_robot::motion
{
namespace
{

bool robot_state_has_variable(
  const moveit::core::RobotState& state,
  const std::string& name)
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
}

double wrapped_angle_delta(double lhs, double rhs)
{
  double delta = std::fmod(lhs - rhs + M_PI, 2.0 * M_PI);
  if (delta < 0.0) {
    delta += 2.0 * M_PI;
  }
  return std::abs(delta - M_PI);
}

std::optional<double> candidate_joint_value(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const std::string& joint_name)
{
  for (size_t i = 0; i < candidate.full_joint_names.size() && i < candidate.full_joint_values.size(); ++i) {
    if (candidate.full_joint_names[i] == joint_name) {
      return candidate.full_joint_values[i];
    }
  }
  return std::nullopt;
}

const std::vector<std::string>& fixed_variable_names()
{
  static const std::vector<std::string> names = {
    "left_joint1", "left_joint2", "left_joint3",
    "left_joint4", "left_joint5", "left_joint6",
    "right_joint1", "right_joint2", "right_joint3",
    "right_joint4", "right_joint5", "right_joint6",
  };
  return names;
}

const std::vector<std::string>& fixed_full_variable_names()
{
  static const std::vector<std::string> names = {
    "updown",
    "left_joint1", "left_joint2", "left_joint3",
    "left_joint4", "left_joint5", "left_joint6",
    "right_joint1", "right_joint2", "right_joint3",
    "right_joint4", "right_joint5", "right_joint6",
  };
  return names;
}

std::string arm_joint_name(const std::string& side, size_t index)
{
  return side + "_joint" + std::to_string(index + 1);
}

std::array<double, 6> arm_seed_from_state(
  const moveit::core::RobotState& state,
  const std::string& side)
{
  std::array<double, 6> seed{};
  for (size_t i = 0; i < seed.size(); ++i) {
    seed[i] = state.getVariablePosition(arm_joint_name(side, i));
  }
  return seed;
}

struct AnalyticHeightPlan
{
  bool reachable = false;
  double lower = 0.0;
  double upper = 0.0;
  double center = 0.0;
  std::vector<double> candidates;
};

struct ToolTargetVariant
{
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  std::string label;
};

std::vector<ToolTargetVariant> tool_target_variants(
  const Eigen::Isometry3d& target,
  bool allow_roll_pi)
{
  std::vector<ToolTargetVariant> variants{{target, "normal"}};
  if (allow_roll_pi) {
    variants.push_back({rotate_about_tool_z(target, M_PI), "roll_pi"});
  }
  return variants;
}

std::pair<double, double> target_h_interval(
  const Eigen::Isometry3d& target,
  robot_motion::core::UpdownAwareIkRequest::GraspMode grasp_mode,
  const robot_motion::core::UpdownAwareIkConfig& config)
{
  const double lower_reach = grasp_mode == robot_motion::core::UpdownAwareIkRequest::GraspMode::TopSuction
    ? config.top_suction_z_reach_lower
    : config.gripper_z_reach_lower;
  const double upper_reach = grasp_mode == robot_motion::core::UpdownAwareIkRequest::GraspMode::TopSuction
    ? config.top_suction_z_reach_upper
    : config.gripper_z_reach_upper;
  return {
    std::max(config.h_lower, target.translation().z() - upper_reach),
    std::min(config.h_upper, target.translation().z() - lower_reach),
  };
}

void push_unique_h(std::vector<double>* values, double value)
{
  if (!values) return;
  for (double existing : *values) {
    if (std::abs(existing - value) < 1e-9) {
      return;
    }
  }
  values->push_back(value);
}

AnalyticHeightPlan plan_height(
  const robot_motion::core::UpdownAwareIkRequest& request,
  robot_motion::core::UpdownAwareIkRequest::GraspMode left_grasp_mode,
  robot_motion::core::UpdownAwareIkRequest::GraspMode right_grasp_mode,
  const robot_motion::core::UpdownAwareIkConfig& config)
{
  if (config.full_h_range_scan) {
    AnalyticHeightPlan plan;
    plan.lower = config.h_lower;
    plan.upper = config.h_upper;
    plan.center = std::clamp(request.current_h, plan.lower, plan.upper);
    plan.reachable = plan.lower <= plan.upper + 1e-9;
    if (!plan.reachable) {
      return plan;
    }
    const double step = std::max(1e-6, std::abs(config.h_step));
    for (double h = plan.lower; h <= plan.upper + 1e-9; h += step) {
      push_unique_h(&plan.candidates, std::min(h, plan.upper));
    }
    if (plan.candidates.empty() || std::abs(plan.candidates.back() - plan.upper) > 1e-9) {
      push_unique_h(&plan.candidates, plan.upper);
    }
    return plan;
  }

  const auto left = target_h_interval(request.left_target, left_grasp_mode, config);
  const auto right = target_h_interval(request.right_target, right_grasp_mode, config);
  AnalyticHeightPlan plan;
  plan.lower = std::max(left.first, right.first);
  plan.upper = std::min(left.second, right.second);
  plan.reachable = plan.lower <= plan.upper + 1e-9;
  if (!plan.reachable) {
    return plan;
  }

  plan.center = std::clamp(request.current_h, plan.lower, plan.upper);
  const double lower = std::max(plan.lower, plan.center - std::abs(config.h_search_margin));
  const double upper = std::min(plan.upper, plan.center + std::abs(config.h_search_margin));
  const size_t count = std::max<size_t>(1, config.h_candidate_count);
  push_unique_h(&plan.candidates, plan.center);
  if (count == 1 || std::abs(upper - lower) < 1e-9) {
    std::sort(plan.candidates.begin(), plan.candidates.end());
    return plan;
  }

  for (size_t i = 0; i < count; ++i) {
    const double ratio = count == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(count - 1);
    push_unique_h(&plan.candidates, lower + (upper - lower) * ratio);
  }
  std::sort(
    plan.candidates.begin(), plan.candidates.end(),
    [&](double lhs, double rhs) {
      const double lhs_delta = std::abs(lhs - plan.center);
      const double rhs_delta = std::abs(rhs - plan.center);
      if (lhs_delta != rhs_delta) return lhs_delta < rhs_delta;
      return lhs < rhs;
    });
  if (plan.candidates.size() > count) {
    plan.candidates.resize(count);
  }
  return plan;
}

double loaded_pose_distance(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const std::string& side,
  const std::vector<double>& pose)
{
  if (pose.size() < 6) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const auto value = candidate_joint_value(candidate, arm_joint_name(side, i));
    if (!value) return 0.0;
    sum += wrapped_angle_delta(*value, pose[i]);
  }
  return sum;
}

double loaded_pose_family_distance(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const std::string& side,
  const std::vector<std::vector<double>>& family)
{
  if (family.empty()) return 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (const auto& pose : family) {
    best = std::min(best, loaded_pose_distance(candidate, side, pose));
  }
  return std::isfinite(best) ? best : 0.0;
}

double preferred_loaded_pose_distance(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const std::string& side,
  const std::vector<std::vector<double>>& family,
  size_t preferred_index)
{
  if (family.empty()) return 0.0;
  const size_t index = std::min(preferred_index, family.size() - 1);
  return loaded_pose_distance(candidate, side, family[index]);
}

double full_joint_delta(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const std::vector<double>& current_full_joints)
{
  if (current_full_joints.size() < candidate.full_joint_values.size()) {
    return 0.0;
  }
  double sum = 0.0;
  for (size_t i = 0; i < candidate.full_joint_values.size(); ++i) {
    const double delta = i == 0
      ? std::abs(candidate.full_joint_values[i] - current_full_joints[i])
      : wrapped_angle_delta(candidate.full_joint_values[i], current_full_joints[i]);
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

double score_analytic_candidate(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const robot_motion::core::UpdownAwareIkRequest& request,
  const robot_motion::core::UpdownAwareIkConfig& config)
{
  double score = 0.0;
  if (config.cost_updown_enabled) {
    const double updown_delta = std::abs(candidate.h - request.current_h);
    if (updown_delta <= config.updown_static_epsilon) {
      score -= config.cost_updown_static_bonus;
    } else if (updown_delta <= config.updown_small_motion_threshold) {
      score -= config.cost_updown_within_0p1_bonus;
    } else {
      score += config.cost_updown_over_0p1_distance *
               (updown_delta - config.updown_small_motion_threshold);
    }
  }
  score += 0.05 * candidate.joint_delta;
  score += config.cost_joint_limit_margin * candidate.joint_limit_margin_cost;
  score += config.cost_loaded_family_distance * (
    loaded_pose_family_distance(candidate, "left", config.left_loaded_pose_family) +
    loaded_pose_family_distance(candidate, "right", config.right_loaded_pose_family));
  score += config.cost_loaded_preferred_distance * (
    preferred_loaded_pose_distance(
      candidate, "left", config.left_loaded_pose_family, config.left_preferred_loaded_pose_index) +
    preferred_loaded_pose_distance(
      candidate, "right", config.right_loaded_pose_family, config.right_preferred_loaded_pose_index));
  return score;
}

robot_motion::core::UpdownAwareIkCandidate make_rejected_candidate(
  double h,
  double h_center,
  double h_lower,
  double h_upper,
  size_t h_index,
  const std::string& reason)
{
  robot_motion::core::UpdownAwareIkCandidate candidate;
  candidate.h = h;
  candidate.h_center = h_center;
  candidate.h_range_lower = h_lower;
  candidate.h_range_upper = h_upper;
  candidate.h_index = h_index;
  candidate.solver_path = "analytic_fixed_h";
  candidate.target_order = "normal";
  candidate.rejection_reason = reason;
  candidate.collision_free = true;
  return candidate;
}

Eigen::Isometry3d tip_pose_in_base_link(
  const moveit::core::RobotState& state,
  const std::string& tip_link)
{
  return state.getGlobalLinkTransform("base_link").inverse() *
         state.getGlobalLinkTransform(tip_link);
}

}  // namespace

double joint_limit_margin_cost(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const moveit::core::RobotModel& robot_model,
  const std::vector<double>& joint_weights,
  double free_ratio)
{
  const double clamped_free_ratio = std::clamp(free_ratio, 0.0, 0.95);
  const auto& variable_names = robot_model.getVariableNames();
  double cost = 0.0;
  for (const auto side : {std::string("left"), std::string("right")}) {
    for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
      const std::string name = arm_joint_name(side, joint_index);
      const auto value = candidate_joint_value(candidate, name);
      if (!value || std::find(variable_names.begin(), variable_names.end(), name) == variable_names.end()) continue;
      const auto& bounds = robot_model.getVariableBounds(name);
      if (!bounds.position_bounded_) continue;
      const double half_range = 0.5 * (bounds.max_position_ - bounds.min_position_);
      if (half_range <= 1e-9) continue;
      const double center = 0.5 * (bounds.max_position_ + bounds.min_position_);
      const double normalized = std::abs(*value - center) / half_range;
      if (normalized <= clamped_free_ratio) continue;
      const double excess = (normalized - clamped_free_ratio) / (1.0 - clamped_free_ratio);
      const double barrier = excess * excess / std::max(1e-3, 1.0 - normalized);
      const double weight = joint_index < joint_weights.size() ? joint_weights[joint_index] : 1.0;
      cost += weight * barrier;
    }
  }
  return cost;
}

double positive_joint_angle_penalty(double joint_angle_rad, double weight)
{
  return std::max(0.0, weight) * std::max(0.0, joint_angle_rad);
}

OptimizedDualIkSolver::OptimizedDualIkSolver(OptimizedDualIkSolverConfig config)
: config_(std::move(config))
{}

bool OptimizedDualIkSolver::ready() const
{
  return config_.ik_config && config_.robot_model;
}

OptimizedDualIkSolveResult OptimizedDualIkSolver::solve(
  const OptimizedDualIkSolveRequest& request,
  const std::string& stage_kind) const
{
  OptimizedDualIkSolveResult output;
  if (!ready()) {
    output.failure_reason = "optimized IK solver is not initialized";
    return output;
  }
  if (!request.seed_state) {
    output.failure_reason = "seed_state is null";
    return output;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto& ik_config = *config_.ik_config;
  const bool left_top_suction = request.top_suction || request.left_top_suction;
  const bool right_top_suction = request.top_suction || request.right_top_suction;
  const auto left_grasp_mode = left_top_suction
    ? robot_motion::core::UpdownAwareIkRequest::GraspMode::TopSuction
    : robot_motion::core::UpdownAwareIkRequest::GraspMode::Front;
  const auto right_grasp_mode = right_top_suction
    ? robot_motion::core::UpdownAwareIkRequest::GraspMode::TopSuction
    : robot_motion::core::UpdownAwareIkRequest::GraspMode::Front;

  robot_motion::core::UpdownAwareIkRequest ik_request;
  ik_request.left_target = pose_to_eigen(request.left_pose);
  ik_request.right_target = pose_to_eigen(request.right_pose);
  ik_request.current_h = currentUpdown(*request.seed_state);
  ik_request.grasp_mode = (left_top_suction && right_top_suction)
    ? robot_motion::core::UpdownAwareIkRequest::GraspMode::TopSuction
    : robot_motion::core::UpdownAwareIkRequest::GraspMode::Front;
  ik_request.current_arm_joints = stateValues(*request.seed_state, fixed_variable_names());
  ik_request.current_full_joints = stateValues(*request.seed_state, fixed_full_variable_names());

  output.ik_result.solver_path = "analytic_fixed_h";
  const AnalyticHeightPlan height_plan = plan_height(ik_request, left_grasp_mode, right_grasp_mode, ik_config);
  output.ik_result.range_reachable = height_plan.reachable;
  output.ik_result.h_interval_lower = height_plan.lower;
  output.ik_result.h_interval_upper = height_plan.upper;
  output.ik_result.h_center = height_plan.center;
  output.ik_result.h_candidates = height_plan.candidates;
  if (!height_plan.reachable) {
    output.ik_result.failure_reason = "h_interval_unreachable";
    output.ik_result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  } else {
    const auto left_seed = arm_seed_from_state(*request.seed_state, "left");
    const auto right_seed = arm_seed_from_state(*request.seed_state, "right");
    const auto left_target_variants = tool_target_variants(
      ik_request.left_target,
      request.allow_front_tool_roll_pi_symmetry && !left_top_suction);
    const auto right_target_variants = tool_target_variants(
      ik_request.right_target,
      request.allow_front_tool_roll_pi_symmetry && !right_top_suction);
    const double left_position_tolerance = left_top_suction
      ? ik_config.top_suction_position_tolerance
      : ik_config.position_tolerance;
    const double right_position_tolerance = right_top_suction
      ? ik_config.top_suction_position_tolerance
      : ik_config.position_tolerance;
    const double left_orientation_tolerance = left_top_suction
      ? ik_config.top_suction_orientation_tolerance
      : ik_config.orientation_tolerance;
    const double right_orientation_tolerance = right_top_suction
      ? ik_config.top_suction_orientation_tolerance
      : ik_config.orientation_tolerance;
    size_t seed_index = 0;
    for (size_t h_index = 0; h_index < height_plan.candidates.size(); ++h_index) {
      const double h = height_plan.candidates[h_index];
      using ArmSolutions = std::vector<alfa_robot::analytic_ik::ArmAnalyticIkSolution>;
      std::vector<std::pair<const ToolTargetVariant*, ArmSolutions>> left_variant_solutions;
      std::vector<std::pair<const ToolTargetVariant*, ArmSolutions>> right_variant_solutions;
      for (const auto& variant : left_target_variants) {
        auto solutions = analytic_solver_.solveInBaseLink(
          alfa_robot::analytic_ik::ArmSide::Left,
          variant.target,
          h,
          left_seed,
          std::max(1e-5, left_position_tolerance),
          std::max(1e-5, left_orientation_tolerance),
          config_.analytic_root_samples);
        if (!solutions.empty()) {
          left_variant_solutions.emplace_back(&variant, std::move(solutions));
        }
      }
      for (const auto& variant : right_target_variants) {
        auto solutions = analytic_solver_.solveInBaseLink(
          alfa_robot::analytic_ik::ArmSide::Right,
          variant.target,
          h,
          right_seed,
          std::max(1e-5, right_position_tolerance),
          std::max(1e-5, right_orientation_tolerance),
          config_.analytic_root_samples);
        if (!solutions.empty()) {
          right_variant_solutions.emplace_back(&variant, std::move(solutions));
        }
      }
      if (left_variant_solutions.empty() || right_variant_solutions.empty()) {
        output.ik_result.candidates.push_back(make_rejected_candidate(
          h, height_plan.center, height_plan.lower, height_plan.upper, h_index,
          left_variant_solutions.empty() ? "left_analytic_no_solution" : "right_analytic_no_solution"));
        continue;
      }
      for (const auto& [left_variant, left_solutions] : left_variant_solutions) {
        for (const auto& [right_variant, right_solutions] : right_variant_solutions) {
          for (const auto& left_solution : left_solutions) {
            for (const auto& right_solution : right_solutions) {
              robot_motion::core::UpdownAwareIkCandidate candidate;
              candidate.legal = true;
              candidate.collision_free = true;
              candidate.solver_path = "analytic_fixed_h";
              candidate.target_order = left_variant->label == "normal" && right_variant->label == "normal"
                ? "normal"
                : "left_" + left_variant->label + "_right_" + right_variant->label;
              candidate.h = h;
              candidate.h_center = height_plan.center;
              candidate.h_range_lower = height_plan.lower;
              candidate.h_range_upper = height_plan.upper;
              candidate.h_index = h_index;
              candidate.seed_index = seed_index++;
              candidate.updown_delta = std::abs(h - ik_request.current_h);
              candidate.joint_names = fixed_variable_names();
              candidate.joint_values.reserve(candidate.joint_names.size());
              for (double value : left_solution.joints) candidate.joint_values.push_back(value);
              for (double value : right_solution.joints) candidate.joint_values.push_back(value);
              candidate.full_joint_names = fixed_full_variable_names();
              candidate.full_joint_values.reserve(candidate.full_joint_names.size());
              candidate.full_joint_values.push_back(h);
              candidate.full_joint_values.insert(
                candidate.full_joint_values.end(),
                candidate.joint_values.begin(),
                candidate.joint_values.end());
              const auto candidate_state =
                robot_state_from_ik_candidate(*request.seed_state, candidate, config_.enforce_bounds_group);
              const Eigen::Isometry3d left_actual = tip_pose_in_base_link(candidate_state, "left_tool0");
              const Eigen::Isometry3d right_actual = tip_pose_in_base_link(candidate_state, "right_tool0");
              const double left_pos_error = pose_position_error(left_variant->target, left_actual);
              const double right_pos_error = pose_position_error(right_variant->target, right_actual);
              const double left_ori_error = pose_orientation_error(left_variant->target, left_actual);
              const double right_ori_error = pose_orientation_error(right_variant->target, right_actual);
              candidate.direct_pos_error = std::max(left_pos_error, right_pos_error);
              candidate.direct_ori_error = std::max(left_ori_error, right_ori_error);
              if (left_pos_error > left_position_tolerance) {
                candidate.legal = false;
                candidate.rejection_reason = "left_analytic_moveit_fk_position_error";
              } else if (right_pos_error > right_position_tolerance) {
                candidate.legal = false;
                candidate.rejection_reason = "right_analytic_moveit_fk_position_error";
              } else if (left_ori_error > left_orientation_tolerance) {
                candidate.legal = false;
                candidate.rejection_reason = "left_analytic_moveit_fk_orientation_error";
              } else if (right_ori_error > right_orientation_tolerance) {
                candidate.legal = false;
                candidate.rejection_reason = "right_analytic_moveit_fk_orientation_error";
              }
              candidate.joint_delta = full_joint_delta(candidate, ik_request.current_full_joints);
              candidate.joint_limit_margin_cost = joint_limit_margin_cost(
                candidate,
                *config_.robot_model,
                ik_config.joint_limit_weights,
                ik_config.joint_limit_free_ratio);
              if (request.capture_pre_score_candidates && candidate.legal) {
                output.ik_result.pre_score_candidates.push_back(candidate);
              }
              candidate.score = score_analytic_candidate(candidate, ik_request, ik_config);
              output.ik_result.candidates.push_back(std::move(candidate));
            }
          }
        }
      }
    }

    output.ik_result.trial_count = output.ik_result.candidates.size();
    output.ik_result.legal_count = static_cast<size_t>(std::count_if(
      output.ik_result.candidates.begin(), output.ik_result.candidates.end(),
      [](const auto& candidate) { return candidate.legal; }));
    output.ik_result.sum_solve_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    output.ik_result.wall_ms = output.ik_result.sum_solve_ms;
    std::sort(
      output.ik_result.candidates.begin(), output.ik_result.candidates.end(),
      [](const auto& lhs, const auto& rhs) {
        if (lhs.legal != rhs.legal) return lhs.legal > rhs.legal;
        if (lhs.score != rhs.score) return lhs.score < rhs.score;
        if (lhs.h_index != rhs.h_index) return lhs.h_index < rhs.h_index;
        return lhs.seed_index < rhs.seed_index;
      });
    const auto selected = std::find_if(
      output.ik_result.candidates.begin(), output.ik_result.candidates.end(),
      [](const auto& candidate) { return candidate.legal; });
    if (selected != output.ik_result.candidates.end()) {
      output.ik_result.success = true;
      output.ik_result.selected = *selected;
    } else {
      output.ik_result.failure_reason = "no_legal_analytic_solution";
    }
  }
  if (!output.ik_result.success) {
    const auto rejection_counts = ik_candidate_rejection_counts_json(output.ik_result);
    double best_pos_error = std::numeric_limits<double>::infinity();
    double best_ori_error = std::numeric_limits<double>::infinity();
    std::string best_reason;
    size_t best_h_index = 0;
    size_t best_seed_index = 0;
    double best_h = 0.0;
    for (const auto& candidate : output.ik_result.candidates) {
      const double combined_error = candidate.direct_pos_error + candidate.direct_ori_error;
      const double best_combined_error = best_pos_error + best_ori_error;
      if (combined_error < best_combined_error) {
        best_pos_error = candidate.direct_pos_error;
        best_ori_error = candidate.direct_ori_error;
        best_reason = candidate.rejection_reason;
        best_h_index = candidate.h_index;
        best_seed_index = candidate.seed_index;
        best_h = candidate.h;
      }
    }
    std::ostringstream oss;
    oss << "optimized IK failed reason=" << output.ik_result.failure_reason
        << " trials=" << output.ik_result.trial_count
        << " legal=" << output.ik_result.legal_count
        << " wall_ms=" << output.ik_result.wall_ms
        << " h_interval=[" << output.ik_result.h_interval_lower << ","
        << output.ik_result.h_interval_upper << "]"
        << " h_candidates=" << vector_json(output.ik_result.h_candidates).dump()
        << " reject=" << rejection_counts.dump()
        << " best_pos=" << best_pos_error
        << " best_ori=" << best_ori_error
        << " best_reason=" << best_reason
        << " best_h=" << best_h
        << " best_h_index=" << best_h_index
        << " best_seed_index=" << best_seed_index;
    output.failure_reason = oss.str();
    return output;
  }

  output.goal_state = std::make_shared<moveit::core::RobotState>(*request.seed_state);
  for (size_t i = 0; i < output.ik_result.selected.full_joint_names.size() &&
                     i < output.ik_result.selected.full_joint_values.size(); ++i) {
    const auto& name = output.ik_result.selected.full_joint_names[i];
    if (isRobotVariable(name)) {
      output.goal_state->setVariablePosition(name, output.ik_result.selected.full_joint_values[i]);
    }
  }
  if (config_.enforce_bounds_group) {
    output.goal_state->enforceBounds(config_.enforce_bounds_group);
  } else {
    output.goal_state->enforceBounds();
  }
  output.goal_state->update();

  const std::string left_grasp_mode_text = left_top_suction ? "top_suction" : "front";
  const std::string right_grasp_mode_text = right_top_suction ? "top_suction" : "front";
  const std::string grasp_mode = left_grasp_mode_text == right_grasp_mode_text
    ? left_grasp_mode_text
    : "mixed";
  output.extra = {
    {"stage_kind", stage_kind},
    {"grasp_mode", grasp_mode},
    {"left_grasp_mode", left_grasp_mode_text},
    {"right_grasp_mode", right_grasp_mode_text},
    {"left_target", pose_json(request.left_pose)},
    {"right_target", pose_json(request.right_pose)},
    {"ik", resultJson(output.ik_result, grasp_mode)}
  };
  output.success = true;
  return output;
}

std::vector<double> OptimizedDualIkSolver::stateValues(
  const moveit::core::RobotState& state,
  const std::vector<std::string>& names) const
{
  std::vector<double> values;
  values.reserve(names.size());
  for (const auto& name : names) {
    values.push_back(isRobotVariable(name) ? state.getVariablePosition(name) : 0.0);
  }
  return values;
}

double OptimizedDualIkSolver::currentUpdown(const moveit::core::RobotState& state) const
{
  return isRobotVariable("updown") ? state.getVariablePosition("updown") : config_.fallback_updown;
}

nlohmann::json ik_candidate_rejection_counts_json(
  const robot_motion::core::UpdownAwareIkResult& result)
{
  std::map<std::string, size_t> counts;
  for (const auto& candidate : result.candidates) {
    if (candidate.legal) {
      counts["legal"]++;
    } else if (!candidate.rejection_reason.empty()) {
      counts[candidate.rejection_reason]++;
    } else {
      counts["unknown"]++;
    }
  }
  nlohmann::json out = nlohmann::json::object();
  for (const auto& [reason, count] : counts) {
    out[reason] = count;
  }
  return out;
}

nlohmann::json OptimizedDualIkSolver::resultJson(
  const robot_motion::core::UpdownAwareIkResult& result,
  const std::string&) const
{
  return {
    {"strategy", "analytic_three_parallel_fixed_h_cost_scorer"},
    {"success", result.success},
    {"fallback_used", result.fallback_used},
    {"failure_reason", result.failure_reason},
    {"trial_count", result.trial_count},
    {"legal_count", result.legal_count},
    {"timeout_like_count", result.timeout_like_count},
    {"wall_ms", result.wall_ms},
    {"sum_solve_ms", result.sum_solve_ms},
    {"h_interval", {{"lower", result.h_interval_lower}, {"upper", result.h_interval_upper}, {"center", result.h_center}}},
    {"h_candidates", vector_json(result.h_candidates)},
    {"candidate_rejection_counts", ik_candidate_rejection_counts_json(result)},
    {"selected", {
      {"h", result.selected.h},
      {"h_index", result.selected.h_index},
      {"seed_index", result.selected.seed_index},
      {"score", result.selected.score},
      {"solver_path", result.selected.solver_path},
      {"target_order", result.selected.target_order},
      {"direct_pos_error", result.selected.direct_pos_error},
      {"direct_ori_error", result.selected.direct_ori_error},
      {"updown_delta", result.selected.updown_delta},
      {"joint_delta", result.selected.joint_delta},
      {"joint_limit_margin_cost", result.selected.joint_limit_margin_cost},
      {"collision_free", result.selected.collision_free},
      {"collision_pairs", result.selected.collision_pairs},
      {"joint_names", result.selected.full_joint_names},
      {"joint_values", result.selected.full_joint_values}
    }}
  };
}

bool OptimizedDualIkSolver::isRobotVariable(const std::string& name) const
{
  if (!config_.robot_model) {
    return false;
  }
  const auto& variable_names = config_.robot_model->getVariableNames();
  return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
}

moveit::core::RobotState robot_state_from_ik_candidate(
  const moveit::core::RobotState& seed_state,
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const moveit::core::JointModelGroup* enforce_bounds_group)
{
  moveit::core::RobotState state(seed_state);
  for (size_t i = 0; i < candidate.full_joint_names.size() && i < candidate.full_joint_values.size(); ++i) {
    const auto& name = candidate.full_joint_names[i];
    if (robot_state_has_variable(state, name)) {
      state.setVariablePosition(name, candidate.full_joint_values[i]);
    }
  }
  if (enforce_bounds_group) {
    state.enforceBounds(enforce_bounds_group);
  } else {
    state.enforceBounds();
  }
  state.update();
  return state;
}

IkCandidateSelector::IkCandidateSelector(IkCandidateSelectorConfig config)
: config_(std::move(config))
{}

bool IkCandidateSelector::similar(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const robot_motion::core::UpdownAwareIkCandidate& kept) const
{
  if (config_.h_threshold >= 0.0 &&
      std::abs(candidate.h - kept.h) > config_.h_threshold) {
    return false;
  }
  if (config_.joint_threshold <= 0.0) {
    return false;
  }

  static const std::array<const char*, 12> arm_joints = {
    "left_joint1", "left_joint2", "left_joint3",
    "left_joint4", "left_joint5", "left_joint6",
    "right_joint1", "right_joint2", "right_joint3",
    "right_joint4", "right_joint5", "right_joint6",
  };

  size_t compared = 0;
  for (const auto* joint_name : arm_joints) {
    const auto lhs = candidate_joint_value(candidate, joint_name);
    const auto rhs = candidate_joint_value(kept, joint_name);
    if (!lhs || !rhs) {
      continue;
    }
    ++compared;
    if (wrapped_angle_delta(*lhs, *rhs) > config_.joint_threshold) {
      return false;
    }
  }
  return compared > 0;
}

std::vector<robot_motion::core::UpdownAwareIkCandidate> IkCandidateSelector::select(
  const std::vector<robot_motion::core::UpdownAwareIkCandidate>& sorted_legal_candidates,
  IkCandidateSelectionStats* stats) const
{
  IkCandidateSelectionStats local_stats;
  local_stats.enabled = config_.dedup_enabled && config_.joint_threshold > 0.0;
  local_stats.input_count = sorted_legal_candidates.size();

  std::vector<robot_motion::core::UpdownAwareIkCandidate> selected;
  if (local_stats.enabled) {
    const auto start = std::chrono::steady_clock::now();
    selected.reserve(sorted_legal_candidates.size());
    for (const auto& candidate : sorted_legal_candidates) {
      bool duplicate = false;
      for (const auto& kept : selected) {
        if (similar(candidate, kept)) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        selected.push_back(candidate);
      }
    }
    local_stats.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  } else {
    selected = sorted_legal_candidates;
  }

  local_stats.unique_count = selected.size();
  local_stats.removed_count = local_stats.input_count > local_stats.unique_count
    ? local_stats.input_count - local_stats.unique_count
    : 0;
  if (config_.candidate_limit > 0 && selected.size() > config_.candidate_limit) {
    selected.resize(config_.candidate_limit);
  }
  local_stats.selected_count = selected.size();
  if (stats) {
    *stats = local_stats;
  }
  return selected;
}

std::vector<robot_motion::core::UpdownAwareIkCandidate> IkCandidateSelector::selectLegalFromResult(
  const robot_motion::core::UpdownAwareIkResult& result,
  IkCandidateSelectionStats* stats) const
{
  std::vector<robot_motion::core::UpdownAwareIkCandidate> legal_candidates;
  legal_candidates.reserve(result.candidates.size());
  for (const auto& candidate : result.candidates) {
    if (candidate.legal) {
      legal_candidates.push_back(candidate);
    }
  }
  std::sort(legal_candidates.begin(), legal_candidates.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.score != rhs.score) return lhs.score < rhs.score;
              if (lhs.h_index != rhs.h_index) return lhs.h_index < rhs.h_index;
              return lhs.seed_index < rhs.seed_index;
            });
  return select(legal_candidates, stats);
}

}  // namespace alfa_robot::motion
