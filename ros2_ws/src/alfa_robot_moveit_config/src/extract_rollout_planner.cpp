#include "alfa_robot_moveit_config/extract_rollout_planner.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

namespace alfa_robot::motion
{

ExtractRolloutPlanner::ExtractRolloutPlanner(ExtractRolloutPlannerConfig config)
: config_(std::move(config))
{}

double ExtractRolloutPlanner::currentUpdown(const moveit::core::RobotState& state) const
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  if (std::find(variable_names.begin(), variable_names.end(), "updown") == variable_names.end()) {
    return 0.0;
  }
  return state.getVariablePosition("updown");
}

const std::string& ExtractRolloutPlanner::tipForSide(const std::string& side) const
{
  return side == "left" ? config_.left_tip : config_.right_tip;
}

const moveit::core::JointModelGroup* ExtractRolloutPlanner::groupForSide(const std::string& side) const
{
  return side == "left" ? config_.left_arm_group : config_.right_arm_group;
}

double ExtractRolloutPlanner::currentPitchUpRad(
  const std::string& side,
  const moveit::core::RobotState& state) const
{
  const Eigen::Vector3d tool_normal =
    state.getGlobalLinkTransform(tipForSide(side)).linear() * Eigen::Vector3d::UnitZ();
  const double x = std::max(0.0, tool_normal.x());
  const double z = tool_normal.z();
  return std::max(0.0, std::atan2(z, x));
}

bool ExtractRolloutPlanner::solveCandidate(
  const std::string& side,
  const moveit::core::RobotState& current_state,
  const geometry_msgs::msg::Pose& target_pose,
  size_t step_index,
  size_t candidate_index,
  double retreat_x,
  double retreat_delta_x,
  double lift_z,
  double lift_delta_z,
  double pitch_up_rad,
  double pitch_delta_rad,
  double min_allowed_tip_z,
  const AttachedBoxSpec& carried_box,
  int box_id,
  ExtractCandidate* out) const
{
  if (!config_.candidate_solver) return false;
  const bool solved = config_.candidate_solver->solve(
    {
      side,
      &current_state,
      target_pose,
      step_index,
      candidate_index,
      retreat_x,
      retreat_delta_x,
      lift_z,
      lift_delta_z,
      pitch_up_rad,
      pitch_delta_rad,
      min_allowed_tip_z,
      currentUpdown(current_state),
    },
    out);
  if (!solved || !out || !out->state) return false;

  bool detached = false;
  std::string reason;
  if (!config_.single_clear_callback ||
      !config_.single_clear_callback(*out->state, carried_box, box_id, &detached, &reason)) {
    out->rejection_reason = reason.empty() ? "single_clear_callback_failed" : reason;
    return false;
  }

  out->state_valid = true;
  out->carried_clear = true;
  out->detached_from_neighbors = detached;
  return true;
}

std::vector<ExtractCandidate> ExtractRolloutPlanner::makeCandidatesForSide(
  const std::string& side,
  const moveit::core::RobotState& current_state,
  const BoxSpec& source_box,
  const AttachedBoxSpec& carried_box,
  int box_id,
  size_t step,
  double last_retreat_x,
  double current_lift_z,
  double min_allowed_tip_z) const
{
  std::vector<ExtractCandidate> candidates;
  if (!config_.motion_planner) return candidates;
  const double current_pitch = currentPitchUpRad(side, current_state);

  for (const auto& layer : config_.motion_planner->layers(
         source_box, box_id, current_pitch, last_retreat_x, current_lift_z)) {
    std::vector<ExtractCandidate> layer_candidates;
    for (const auto& command : layer.commands) {
      const auto target_pose = make_pose(
        command.shifted_box.x, command.shifted_box.y, command.shifted_box.z,
        pitch_up_orientation(command.pitch_up_rad));

      ExtractCandidate candidate;
      solveCandidate(side, current_state, target_pose, step, command.candidate_index,
                     command.retreat_x, command.retreat_delta_x,
                     command.lift_z, command.lift_delta_z,
                     command.pitch_up_rad, command.pitch_delta_rad,
                     min_allowed_tip_z, carried_box, box_id, &candidate);
      layer_candidates.push_back(candidate);
    }

    const bool layer_has_valid = std::any_of(
      layer_candidates.begin(), layer_candidates.end(),
      [](const ExtractCandidate& candidate) { return candidate.state_valid; });
    candidates.insert(candidates.end(), layer_candidates.begin(), layer_candidates.end());
    if (layer_has_valid) {
      break;
    }
  }

  return candidates;
}

std::vector<ExtractCandidate> ExtractRolloutPlanner::topValidCandidates(
  const std::string& side,
  const std::vector<ExtractCandidate>& candidates,
  const moveit::core::RobotState& current_state,
  double last_retreat_x) const
{
  std::vector<ExtractCandidate> valid;
  for (const auto& candidate : candidates) {
    if (candidate.state_valid && candidate.state) {
      valid.push_back(candidate);
    }
  }
  std::sort(valid.begin(), valid.end(),
            [&](const ExtractCandidate& a, const ExtractCandidate& b) {
              return config_.candidate_scorer->score(side, a, current_state, last_retreat_x) <
                     config_.candidate_scorer->score(side, b, current_state, last_retreat_x);
            });
  if (valid.size() > config_.top_valid_limit) {
    valid.resize(config_.top_valid_limit);
  }
  return valid;
}

void ExtractRolloutPlanner::copyArmState(
  const std::string& side,
  const moveit::core::RobotState& from,
  moveit::core::RobotState& to) const
{
  const auto* group = groupForSide(side);
  if (!group) return;
  for (const auto& name : group->getVariableNames()) {
    if (std::find(to.getRobotModel()->getVariableNames().begin(), to.getRobotModel()->getVariableNames().end(), name) !=
        to.getRobotModel()->getVariableNames().end()) {
      to.setVariablePosition(name, from.getVariablePosition(name));
    }
  }
}

ArmExtractPath ExtractRolloutPlanner::rolloutArm(
  const std::string& side,
  const moveit::core::RobotState& start_state,
  const BoxSpec& source_box,
  const AttachedBoxSpec& carried_box,
  int box_id) const
{
  ArmExtractPath path;
  moveit::core::RobotState current_state(start_state);
  double last_retreat_x = 0.0;
  double current_lift_z = 0.0;
  double min_allowed_tip_z = current_state.getGlobalLinkTransform(tipForSide(side)).translation().z();
  const size_t max_steps = config_.motion_planner ? config_.motion_planner->maxStepCount() : 0;

  path.states.push_back(std::make_shared<moveit::core::RobotState>(current_state));
  bool detached_seen = false;
  size_t extra_steps_after_detached = 0;
  for (size_t step = 1; step <= max_steps; ++step) {
    const auto candidates =
      makeCandidatesForSide(side, current_state, source_box, carried_box, box_id,
                            step, last_retreat_x, current_lift_z, min_allowed_tip_z);
    auto best_it = std::min_element(
      candidates.begin(), candidates.end(),
      [&](const ExtractCandidate& a, const ExtractCandidate& b) {
        return config_.candidate_scorer->score(side, a, current_state, last_retreat_x) <
               config_.candidate_scorer->score(side, b, current_state, last_retreat_x);
      });
    if (best_it == candidates.end() || !best_it->state_valid || !best_it->state) {
      ++path.failed_steps;
      std::map<std::string, size_t> rejection_counts;
      for (const auto& candidate : candidates) {
        const std::string key = candidate.rejection_reason.empty() ? "unknown" : candidate.rejection_reason;
        rejection_counts[key]++;
      }
      if (!rejection_counts.empty()) {
        size_t best_count = 0;
        for (const auto& [reason, count] : rejection_counts) {
          if (count > best_count) {
            best_count = count;
            path.failure_reason = side + ":" + reason;
          }
        }
      } else {
        path.failure_reason = side + ":no_valid_candidate";
      }
      if (detached_seen && path.success) {
        path.failure_reason.clear();
      }
      break;
    }

    current_state = *best_it->state;
    min_allowed_tip_z =
      std::max(min_allowed_tip_z, current_state.getGlobalLinkTransform(tipForSide(side)).translation().z());
    last_retreat_x = best_it->retreat_x;
    current_lift_z = best_it->lift_z;
    path.final_retreat_x = best_it->retreat_x;
    path.final_lift_z = best_it->lift_z;
    path.final_pitch_deg = best_it->pitch_up_rad * 180.0 / M_PI;
    path.selected_candidates.push_back(*best_it);
    path.states.push_back(std::make_shared<moveit::core::RobotState>(current_state));
    ++path.accepted_steps;

    if (best_it->detached_from_neighbors) {
      detached_seen = true;
      path.success = true;
      path.failure_reason.clear();
    }
    if (detached_seen) {
      ++extra_steps_after_detached;
    }
    if (detached_seen && extra_steps_after_detached > config_.success_extra_steps) {
      path.success = true;
      path.failure_reason.clear();
      break;
    }
  }

  if (!path.success && path.failure_reason.empty()) {
    path.failure_reason = side + ":reached_max_retreat_without_neighbor_detachment";
  }
  return path;
}

moveit::core::RobotState ExtractRolloutPlanner::combineAsyncArmStates(
  const moveit::core::RobotState& base_state,
  const ArmExtractPath& left_path,
  const ArmExtractPath& right_path,
  size_t left_index,
  size_t right_index) const
{
  moveit::core::RobotState state(base_state);
  const auto& left_state = *left_path.states[std::min(left_index, left_path.states.size() - 1)];
  const auto& right_state = *right_path.states[std::min(right_index, right_path.states.size() - 1)];
  copyArmState("left", left_state, state);
  copyArmState("right", right_state, state);
  state.setVariablePosition("updown", currentUpdown(base_state));
  state.enforceBounds(config_.joint_group);
  state.update();
  return state;
}

bool ExtractRolloutPlanner::validateAsyncPath(
  const moveit::core::RobotState& start_state,
  const ArmExtractPath& left_path,
  const ArmExtractPath& right_path,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  std::vector<moveit::core::RobotStatePtr>* combined_states,
  std::string* reason) const
{
  if (!left_path.success) {
    if (reason) *reason = left_path.failure_reason.empty() ? "left_async_extract_failed" : left_path.failure_reason;
    return false;
  }
  if (!right_path.success) {
    if (reason) *reason = right_path.failure_reason.empty() ? "right_async_extract_failed" : right_path.failure_reason;
    return false;
  }
  if (left_path.states.empty() || right_path.states.empty()) {
    if (reason) *reason = "empty_async_extract_path";
    return false;
  }

  const size_t step_count = std::max(left_path.states.size(), right_path.states.size());
  if (combined_states) {
    combined_states->clear();
    combined_states->reserve(step_count);
  }
  for (size_t step = 0; step < step_count; ++step) {
    auto state = std::make_shared<moveit::core::RobotState>(
      combineAsyncArmStates(start_state, left_path, right_path, step, step));
    bool left_detached = false;
    bool right_detached = false;
    std::string state_reason;
    if (!config_.dual_clear_callback ||
        !config_.dual_clear_callback(*state, left_box, left_box_id, right_box, right_box_id,
                                     &left_detached, &right_detached, &state_reason)) {
      if (reason) *reason = "async_combined_step_" + std::to_string(step) + ":" + state_reason;
      return false;
    }
    if (combined_states) {
      combined_states->push_back(state);
    }
  }
  return true;
}

DualExtractStepCandidate ExtractRolloutPlanner::selectDualStepCandidate(
  const moveit::core::RobotState& current_state,
  const std::vector<ExtractCandidate>& left_candidates,
  const std::vector<ExtractCandidate>& right_candidates,
  double left_last_retreat_x,
  double right_last_retreat_x,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id) const
{
  DualExtractStepCandidate best;
  const auto left_valid = topValidCandidates("left", left_candidates, current_state, left_last_retreat_x);
  const auto right_valid = topValidCandidates("right", right_candidates, current_state, right_last_retreat_x);
  if (left_valid.empty() || right_valid.empty()) {
    best.rejection_reason = left_valid.empty() ? "no_valid_left_extract_candidate" : "no_valid_right_extract_candidate";
    return best;
  }

  for (const auto& left_candidate : left_valid) {
    for (const auto& right_candidate : right_valid) {
      auto state = std::make_shared<moveit::core::RobotState>(current_state);
      copyArmState("left", *left_candidate.state, *state);
      copyArmState("right", *right_candidate.state, *state);
      state->setVariablePosition("updown", currentUpdown(current_state));
      state->enforceBounds(config_.joint_group);
      state->update();

      bool left_detached = false;
      bool right_detached = false;
      std::string reason;
      if (!config_.dual_clear_callback ||
          !config_.dual_clear_callback(*state, left_box, left_box_id, right_box, right_box_id,
                                       &left_detached, &right_detached, &reason)) {
        if (best.rejection_reason.empty()) {
          best.rejection_reason = reason.empty() ? "dual_clear_callback_failed" : reason;
        }
        continue;
      }

      const double score =
        config_.candidate_scorer->score("left", left_candidate, current_state, left_last_retreat_x) +
        config_.candidate_scorer->score("right", right_candidate, current_state, right_last_retreat_x) +
        0.2 * std::abs(left_candidate.retreat_x - right_candidate.retreat_x) +
        0.2 * std::abs(left_candidate.lift_z - right_candidate.lift_z);
      if (!best.state_valid || score < best.score) {
        best.left = left_candidate;
        best.right = right_candidate;
        best.state = state;
        best.state_valid = true;
        best.left_detached = left_detached;
        best.right_detached = right_detached;
        best.score = score;
        best.rejection_reason.clear();
      }
    }
  }
  if (!best.state_valid && best.rejection_reason.empty()) {
    best.rejection_reason = "no_collision_free_dual_extract_candidate";
  }
  return best;
}

ExtractRolloutTiming ExtractRolloutPlanner::rolloutLeft(
  const moveit::core::RobotState& start_state,
  const BoxSpec& source_box,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  size_t candidate_order,
  size_t h_index,
  size_t seed_index,
  double h,
  double ik_score,
  double ik_solve_ms,
  const ExtractRecordStepCallback& record_step) const
{
  ExtractRolloutTiming timing;
  timing.candidate_order = candidate_order;
  timing.h_index = h_index;
  timing.seed_index = seed_index;
  timing.h = h;
  timing.ik_score = ik_score;
  timing.ik_solve_ms = ik_solve_ms;

  const auto t0 = std::chrono::steady_clock::now();
  moveit::core::RobotState current_state(start_state);
  double last_retreat_x = 0.0;
  double current_lift_z = 0.0;
  double min_allowed_tip_z = current_state.getGlobalLinkTransform(config_.left_tip).translation().z();
  const size_t max_steps = config_.motion_planner ? config_.motion_planner->maxStepCount() : 0;

  if (record_step) {
    nlohmann::json extra = {
      {"stage_kind", "left_extract_all_legal_ik_start"},
      {"candidate_order", candidate_order},
      {"h_index", h_index},
      {"seed_index", seed_index},
      {"h", h},
      {"ik_score", ik_score},
      {"ik_solve_ms", ik_solve_ms},
      {"accepted", true},
      {"step", 0},
      {"retreat_x", 0.0},
      {"lift_z", 0.0},
      {"pitch_up_deg", 0.0},
      {"left_tip_z", current_state.getGlobalLinkTransform(config_.left_tip).translation().z()},
      {"tool_normal_z", (current_state.getGlobalLinkTransform(config_.left_tip).linear() * Eigen::Vector3d::UnitZ()).z()},
      {"detached_from_neighbors", false}
    };
    record_step(0, current_state, extra);
  }

  for (size_t step = 1; step <= max_steps; ++step) {
    std::vector<ExtractCandidate> candidates =
      makeCandidatesForSide("left", current_state, source_box, left_box, left_box_id,
                            step, last_retreat_x, current_lift_z, min_allowed_tip_z);

    auto best_it = std::min_element(candidates.begin(), candidates.end(), [&](const ExtractCandidate& a, const ExtractCandidate& b) {
      return config_.candidate_scorer->score("left", a, current_state, last_retreat_x) <
             config_.candidate_scorer->score("left", b, current_state, last_retreat_x);
    });

    if (best_it == candidates.end() || !best_it->state_valid) {
      ++timing.failed_steps;
      std::map<std::string, size_t> rejection_counts;
      for (const auto& candidate : candidates) {
        const std::string key = candidate.rejection_reason.empty() ? "unknown" : candidate.rejection_reason;
        rejection_counts[key]++;
      }
      if (!rejection_counts.empty()) {
        timing.failure_reason = rejection_counts.begin()->first;
        size_t best_count = 0;
        for (const auto& [reason, count] : rejection_counts) {
          if (count > best_count) {
            best_count = count;
            timing.failure_reason = reason;
          }
        }
      } else {
        timing.failure_reason = "no_valid_candidate";
      }
      if (record_step) {
        nlohmann::json rejection_json = nlohmann::json::object();
        for (const auto& [reason, count] : rejection_counts) {
          rejection_json[reason] = count;
        }
        nlohmann::json extra = {
          {"stage_kind", "left_extract_all_legal_ik_failed_step"},
          {"candidate_order", candidate_order},
          {"h_index", h_index},
          {"seed_index", seed_index},
          {"h", h},
          {"ik_score", ik_score},
          {"ik_solve_ms", ik_solve_ms},
          {"step", step},
          {"retreat_x", last_retreat_x},
          {"accepted", false},
          {"candidate_count", candidates.size()},
          {"rejection_counts", rejection_json},
          {"failure_reason", timing.failure_reason}
        };
        record_step(step, current_state, extra);
      }
      if (config_.fail_fast) break;
      continue;
    }

    const double selected_score = config_.candidate_scorer->score("left", *best_it, current_state, last_retreat_x);
    const double selected_joint_delta = config_.candidate_scorer->armJointDelta("left", current_state, *best_it->state);
    const double selected_tip_position_delta = config_.candidate_scorer->tipPositionDelta("left", current_state, *best_it->state);
    const double selected_tip_orientation_delta = config_.candidate_scorer->tipOrientationDelta("left", current_state, *best_it->state);

    current_state = *best_it->state;
    min_allowed_tip_z = std::max(min_allowed_tip_z, current_state.getGlobalLinkTransform(config_.left_tip).translation().z());
    last_retreat_x = best_it->retreat_x;
    current_lift_z = best_it->lift_z;
    timing.final_retreat_x = best_it->retreat_x;
    timing.final_lift_z = best_it->lift_z;
    timing.final_pitch_deg = best_it->pitch_up_rad * 180.0 / M_PI;
    ++timing.accepted_steps;

    if (record_step) {
      nlohmann::json extra = {
        {"stage_kind", "left_extract_all_legal_ik_step"},
        {"candidate_order", candidate_order},
        {"h_index", h_index},
        {"seed_index", seed_index},
        {"h", h},
        {"ik_score", ik_score},
        {"ik_solve_ms", ik_solve_ms},
        {"step", step},
        {"candidate_index", best_it->candidate_index},
        {"retreat_x", best_it->retreat_x},
        {"retreat_delta_x", best_it->retreat_delta_x},
        {"lift_z", best_it->lift_z},
        {"lift_delta_z", best_it->lift_delta_z},
        {"pitch_up_deg", best_it->pitch_up_rad * 180.0 / M_PI},
        {"pitch_delta_deg", best_it->pitch_delta_rad * 180.0 / M_PI},
        {"left_tip_z", current_state.getGlobalLinkTransform(config_.left_tip).translation().z()},
        {"tool_normal_z", (current_state.getGlobalLinkTransform(config_.left_tip).linear() * Eigen::Vector3d::UnitZ()).z()},
        {"detached_from_neighbors", best_it->detached_from_neighbors},
        {"candidate_count", candidates.size()},
        {"score", selected_score},
        {"joint_delta", selected_joint_delta},
        {"tip_position_delta", selected_tip_position_delta},
        {"tip_orientation_delta", selected_tip_orientation_delta},
        {"accepted", true}
      };
      record_step(step, current_state, extra);
    }

    if (best_it->detached_from_neighbors) {
      timing.success = true;
      timing.failure_reason.clear();
      timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
      break;
    }
  }

  if (!timing.success && timing.failure_reason.empty()) {
    timing.failure_reason = "reached_max_retreat_without_neighbor_detachment";
  }
  const auto t1 = std::chrono::steady_clock::now();
  timing.rollout_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return timing;
}

ExtractRolloutTiming ExtractRolloutPlanner::rolloutDual(
  const moveit::core::RobotState& start_state,
  const BoxSpec& left_source_box,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const BoxSpec& right_source_box,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  size_t candidate_order,
  size_t h_index,
  size_t seed_index,
  double h,
  double ik_score,
  double ik_solve_ms,
  const ExtractRecordStepCallback& record_step) const
{
  ExtractRolloutTiming timing;
  timing.candidate_order = candidate_order;
  timing.h_index = h_index;
  timing.seed_index = seed_index;
  timing.h = h;
  timing.ik_score = ik_score;
  timing.ik_solve_ms = ik_solve_ms;

  const auto t0 = std::chrono::steady_clock::now();
  moveit::core::RobotState current_state(start_state);
  if (config_.dual_async) {
    const auto left_path = rolloutArm("left", start_state, left_source_box, left_box, left_box_id);
    const auto right_path = rolloutArm("right", start_state, right_source_box, right_box, right_box_id);
    timing.accepted_steps = left_path.accepted_steps + right_path.accepted_steps;
    timing.failed_steps = left_path.failed_steps + right_path.failed_steps;
    timing.final_retreat_x = left_path.final_retreat_x;
    timing.final_lift_z = left_path.final_lift_z;
    timing.final_pitch_deg = left_path.final_pitch_deg;
    timing.right_final_retreat_x = right_path.final_retreat_x;
    timing.right_final_lift_z = right_path.final_lift_z;
    timing.right_final_pitch_deg = right_path.final_pitch_deg;

    std::vector<moveit::core::RobotStatePtr> combined_states;
    std::string async_reason;
    const bool async_valid = validateAsyncPath(
      start_state, left_path, right_path, left_box, left_box_id, right_box, right_box_id,
      &combined_states, &async_reason);
    if (!async_valid) {
      timing.failure_reason = async_reason;
    } else {
      timing.success = true;
      timing.failure_reason.clear();
      timing.final_state = combined_states.empty() ? std::make_shared<moveit::core::RobotState>(start_state) : combined_states.back();
    }

    if (record_step) {
      nlohmann::json start_extra = {
        {"stage_kind", "dual_extract_async_start"},
        {"candidate_order", candidate_order},
        {"h_index", h_index},
        {"seed_index", seed_index},
        {"h", h},
        {"ik_score", ik_score},
        {"ik_solve_ms", ik_solve_ms},
        {"accepted", true},
        {"step", 0},
        {"left_path_success", left_path.success},
        {"right_path_success", right_path.success},
        {"left_path_steps", left_path.accepted_steps},
        {"right_path_steps", right_path.accepted_steps},
        {"async_valid", async_valid},
        {"failure_reason", timing.failure_reason}
      };
      record_step(0, start_state, start_extra);
      const size_t recorded_step_count = std::max(
        combined_states.size(),
        std::max(left_path.states.empty() ? 0 : left_path.states.size() - 1,
                 right_path.states.empty() ? 0 : right_path.states.size() - 1));
      for (size_t step = 1; step <= recorded_step_count; ++step) {
        const auto left_index = std::min(step, left_path.selected_candidates.size());
        const auto right_index = std::min(step, right_path.selected_candidates.size());
        auto display_state = combined_states.empty() || step > combined_states.size()
          ? std::make_shared<moveit::core::RobotState>(
              combineAsyncArmStates(start_state, left_path, right_path, left_index, right_index))
          : combined_states[step - 1];
        nlohmann::json extra = {
          {"stage_kind", "dual_extract_async_step"},
          {"candidate_order", candidate_order},
          {"h_index", h_index},
          {"seed_index", seed_index},
          {"h", h},
          {"ik_score", ik_score},
          {"ik_solve_ms", ik_solve_ms},
          {"step", step},
          {"accepted", true},
          {"left_path_success", left_path.success},
          {"right_path_success", right_path.success},
          {"left_path_steps", left_path.accepted_steps},
          {"right_path_steps", right_path.accepted_steps},
          {"async_valid", async_valid},
          {"failure_reason", timing.failure_reason}
        };
        if (left_index > 0 && left_index <= left_path.selected_candidates.size()) {
          const auto& left = left_path.selected_candidates[left_index - 1];
          extra["left_retreat_x"] = left.retreat_x;
          extra["left_lift_z"] = left.lift_z;
          extra["left_pitch_up_deg"] = left.pitch_up_rad * 180.0 / M_PI;
          extra["left_detached_from_neighbors"] = left.detached_from_neighbors;
        }
        if (right_index > 0 && right_index <= right_path.selected_candidates.size()) {
          const auto& right = right_path.selected_candidates[right_index - 1];
          extra["right_retreat_x"] = right.retreat_x;
          extra["right_lift_z"] = right.lift_z;
          extra["right_pitch_up_deg"] = right.pitch_up_rad * 180.0 / M_PI;
          extra["right_detached_from_neighbors"] = right.detached_from_neighbors;
        }
        record_step(step, *display_state, extra);
      }
    }

    const auto t1 = std::chrono::steady_clock::now();
    timing.rollout_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return timing;
  }

  double left_last_retreat_x = 0.0;
  double right_last_retreat_x = 0.0;
  double left_lift_z = 0.0;
  double right_lift_z = 0.0;
  double left_min_tip_z = current_state.getGlobalLinkTransform(config_.left_tip).translation().z();
  double right_min_tip_z = current_state.getGlobalLinkTransform(config_.right_tip).translation().z();
  const size_t max_steps = config_.motion_planner ? config_.motion_planner->maxStepCount() : 0;

  if (record_step) {
    nlohmann::json extra = {
      {"stage_kind", "dual_extract_all_legal_ik_start"},
      {"candidate_order", candidate_order},
      {"h_index", h_index},
      {"seed_index", seed_index},
      {"h", h},
      {"ik_score", ik_score},
      {"ik_solve_ms", ik_solve_ms},
      {"accepted", true},
      {"step", 0},
      {"left_retreat_x", 0.0},
      {"right_retreat_x", 0.0},
      {"left_lift_z", 0.0},
      {"right_lift_z", 0.0},
      {"left_detached_from_neighbors", false},
      {"right_detached_from_neighbors", false}
    };
    record_step(0, current_state, extra);
  }

  for (size_t step = 1; step <= max_steps; ++step) {
    const auto left_candidates =
      makeCandidatesForSide("left", current_state, left_source_box, left_box, left_box_id,
                            step, left_last_retreat_x, left_lift_z, left_min_tip_z);
    const auto right_candidates =
      makeCandidatesForSide("right", current_state, right_source_box, right_box, right_box_id,
                            step, right_last_retreat_x, right_lift_z, right_min_tip_z);
    const auto best = selectDualStepCandidate(
      current_state,
      left_candidates,
      right_candidates,
      left_last_retreat_x,
      right_last_retreat_x,
      left_box,
      left_box_id,
      right_box,
      right_box_id);

    if (!best.state_valid || !best.state) {
      ++timing.failed_steps;
      timing.failure_reason = best.rejection_reason.empty() ? "no_valid_dual_extract_candidate" : best.rejection_reason;
      if (record_step) {
        nlohmann::json extra = {
          {"stage_kind", "dual_extract_all_legal_ik_failed_step"},
          {"candidate_order", candidate_order},
          {"h_index", h_index},
          {"seed_index", seed_index},
          {"h", h},
          {"ik_score", ik_score},
          {"ik_solve_ms", ik_solve_ms},
          {"step", step},
          {"accepted", false},
          {"left_candidate_count", left_candidates.size()},
          {"right_candidate_count", right_candidates.size()},
          {"failure_reason", timing.failure_reason}
        };
        record_step(step, current_state, extra);
      }
      if (config_.fail_fast) break;
      continue;
    }

    current_state = *best.state;
    left_min_tip_z = std::max(left_min_tip_z, current_state.getGlobalLinkTransform(config_.left_tip).translation().z());
    right_min_tip_z = std::max(right_min_tip_z, current_state.getGlobalLinkTransform(config_.right_tip).translation().z());
    left_last_retreat_x = best.left.retreat_x;
    right_last_retreat_x = best.right.retreat_x;
    left_lift_z = best.left.lift_z;
    right_lift_z = best.right.lift_z;
    timing.final_retreat_x = best.left.retreat_x;
    timing.final_lift_z = best.left.lift_z;
    timing.final_pitch_deg = best.left.pitch_up_rad * 180.0 / M_PI;
    timing.right_final_retreat_x = best.right.retreat_x;
    timing.right_final_lift_z = best.right.lift_z;
    timing.right_final_pitch_deg = best.right.pitch_up_rad * 180.0 / M_PI;
    ++timing.accepted_steps;

    if (record_step) {
      nlohmann::json extra = {
        {"stage_kind", "dual_extract_all_legal_ik_step"},
        {"candidate_order", candidate_order},
        {"h_index", h_index},
        {"seed_index", seed_index},
        {"h", h},
        {"ik_score", ik_score},
        {"ik_solve_ms", ik_solve_ms},
        {"step", step},
        {"left_candidate_index", best.left.candidate_index},
        {"right_candidate_index", best.right.candidate_index},
        {"left_retreat_x", best.left.retreat_x},
        {"right_retreat_x", best.right.retreat_x},
        {"left_retreat_delta_x", best.left.retreat_delta_x},
        {"right_retreat_delta_x", best.right.retreat_delta_x},
        {"left_lift_z", best.left.lift_z},
        {"right_lift_z", best.right.lift_z},
        {"left_pitch_up_deg", best.left.pitch_up_rad * 180.0 / M_PI},
        {"right_pitch_up_deg", best.right.pitch_up_rad * 180.0 / M_PI},
        {"left_detached_from_neighbors", best.left_detached},
        {"right_detached_from_neighbors", best.right_detached},
        {"left_candidate_count", left_candidates.size()},
        {"right_candidate_count", right_candidates.size()},
        {"score", best.score},
        {"accepted", true}
      };
      record_step(step, current_state, extra);
    }

    if (best.left_detached && best.right_detached) {
      timing.success = true;
      timing.failure_reason.clear();
      timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
      break;
    }
  }

  if (!timing.success && timing.failure_reason.empty()) {
    timing.failure_reason = "reached_max_retreat_without_dual_neighbor_detachment";
  }
  const auto t1 = std::chrono::steady_clock::now();
  timing.rollout_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return timing;
}

}  // namespace alfa_robot::motion
