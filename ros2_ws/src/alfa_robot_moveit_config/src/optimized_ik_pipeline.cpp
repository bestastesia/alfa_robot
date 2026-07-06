#include "alfa_robot_moveit_config/optimized_ik_pipeline.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
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
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
  const std::string& joint_name)
{
  for (size_t i = 0; i < candidate.full_joint_names.size() && i < candidate.full_joint_values.size(); ++i) {
    if (candidate.full_joint_names[i] == joint_name) {
      return candidate.full_joint_values[i];
    }
  }
  return std::nullopt;
}

}  // namespace

OptimizedDualIkSolver::OptimizedDualIkSolver(OptimizedDualIkSolverConfig config)
: config_(std::move(config))
{}

bool OptimizedDualIkSolver::ready() const
{
  return config_.solver && config_.robot_model;
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

  ik_benchmark::UpdownAwareIkRequest ik_request;
  ik_request.left_target = pose_to_eigen(request.left_pose);
  ik_request.right_target = pose_to_eigen(request.right_pose);
  ik_request.current_h = currentUpdown(*request.seed_state);
  ik_request.grasp_mode = request.top_suction
    ? ik_benchmark::UpdownAwareIkRequest::GraspMode::TopSuction
    : ik_benchmark::UpdownAwareIkRequest::GraspMode::Front;
  ik_request.current_arm_joints = stateValues(*request.seed_state, config_.solver->fixedVariableNames());
  ik_request.current_full_joints = stateValues(*request.seed_state, config_.solver->fixedFullVariableNames());

  output.ik_result = config_.solver->solve(ik_request);
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

  const std::string grasp_mode = request.top_suction ? "top_suction" : "front";
  output.extra = {
    {"stage_kind", stage_kind},
    {"grasp_mode", grasp_mode},
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
  const ik_benchmark::UpdownAwareIkResult& result)
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
  const ik_benchmark::UpdownAwareIkResult& result,
  const std::string&) const
{
  return {
    {"strategy", "fixed_discrete_h_multi_seed_cost_scorer"},
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
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
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
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
  const ik_benchmark::UpdownAwareIkCandidate& kept) const
{
  if (config_.h_threshold >= 0.0 &&
      std::abs(candidate.h - kept.h) > config_.h_threshold) {
    return false;
  }
  if (config_.joint_threshold <= 0.0) {
    return false;
  }

  static const std::array<const char*, 12> arm_joints = {
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
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

std::vector<ik_benchmark::UpdownAwareIkCandidate> IkCandidateSelector::select(
  const std::vector<ik_benchmark::UpdownAwareIkCandidate>& sorted_legal_candidates,
  IkCandidateSelectionStats* stats) const
{
  IkCandidateSelectionStats local_stats;
  local_stats.enabled = config_.dedup_enabled && config_.joint_threshold > 0.0;
  local_stats.input_count = sorted_legal_candidates.size();

  std::vector<ik_benchmark::UpdownAwareIkCandidate> selected;
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

std::vector<ik_benchmark::UpdownAwareIkCandidate> IkCandidateSelector::selectLegalFromResult(
  const ik_benchmark::UpdownAwareIkResult& result,
  IkCandidateSelectionStats* stats) const
{
  std::vector<ik_benchmark::UpdownAwareIkCandidate> legal_candidates;
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
