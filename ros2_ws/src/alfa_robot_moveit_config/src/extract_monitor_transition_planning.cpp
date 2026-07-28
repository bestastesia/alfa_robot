#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"

#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include <moveit/robot_state/conversions.h>
#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

namespace
{

using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
using Clock = std::chrono::steady_clock;

constexpr double kMaxStageJointSpeedRadS = 20.0 * M_PI / 180.0;

double elapsed_ms(const Clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
}

bool transition_cancelled(const ExtractMonitorTransitionPlanner& planner)
{
  return planner.is_cancelled && planner.is_cancelled();
}

bool reject_if_cancelled(
  const ExtractMonitorTransitionPlanner& planner,
  std::string* reason)
{
  if (!transition_cancelled(planner)) {
    return false;
  }
  if (reason) {
    *reason = "loaded_plan_cancelled_after_first_success";
  }
  return true;
}

double point_time_s(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

moveit::core::RobotState state_from_point(
  const moveit::core::RobotState& reference,
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  size_t point_index)
{
  moveit::core::RobotState state(reference);
  const auto& point = trajectory.points[point_index];
  for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i) {
    state.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
  }
  state.update(true);
  return state;
}

std::vector<moveit::core::RobotState> plan_states(
  const Plan& plan,
  const moveit::core::RobotState& fallback_start,
  const moveit::core::RobotState& fallback_goal)
{
  const auto& trajectory = plan.trajectory_.joint_trajectory;
  if (trajectory.points.size() < 2 || trajectory.joint_names.empty()) {
    return {fallback_start, fallback_goal};
  }

  std::vector<moveit::core::RobotState> states;
  states.reserve(trajectory.points.size());
  for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
    states.push_back(state_from_point(fallback_start, trajectory, point_index));
  }
  return states;
}

void append_segment(Plan& combined, const Plan& segment)
{
  const auto& input = segment.trajectory_.joint_trajectory;
  if (input.points.empty()) {
    return;
  }

  auto& output = combined.trajectory_.joint_trajectory;
  if (output.joint_names.empty()) {
    output.joint_names = input.joint_names;
  }

  const double output_offset = output.points.empty()
    ? 0.0
    : point_time_s(output.points.back());
  const double input_offset = point_time_s(input.points.front());
  const size_t start_index = output.points.empty() ? 0 : 1;
  for (size_t point_index = start_index; point_index < input.points.size(); ++point_index) {
    auto point = input.points[point_index];
    const double local_time = std::max(0.0, point_time_s(point) - input_offset);
    point.time_from_start = rclcpp::Duration::from_seconds(output_offset + local_time);
    output.points.push_back(std::move(point));
  }
}

bool make_valid_interpolation(
  const ExtractMonitorTransitionPlanner& planner,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  Plan* plan,
  std::string* reason)
{
  if (reject_if_cancelled(planner, reason)) {
    return false;
  }
  if (!planner.make_interpolated_plan || !planner.densify_plan || !planner.validate_plan) {
    if (reason) {
      *reason = "transition_planner_missing_basic_adapter";
    }
    return false;
  }

  Plan candidate = retime_plan_by_max_joint_speed(
    planner.densify_plan(planner.make_interpolated_plan(start_state, goal_state, 1.0)),
    start_state.getRobotModel(),
    kMaxStageJointSpeedRadS);
  std::string validation_reason;
  if (!planner.validate_plan(candidate, start_state, &validation_reason)) {
    if (reason) {
      *reason = validation_reason;
    }
    return false;
  }
  if (plan) {
    *plan = std::move(candidate);
  }
  if (reason) {
    reason->clear();
  }
  return true;
}

bool make_valid_local_rrt(
  const ExtractMonitorTransitionPlanner& planner,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  Plan* plan,
  std::string* reason)
{
  if (reject_if_cancelled(planner, reason)) {
    return false;
  }
  if (!planner.direct_plan || !planner.densify_plan || !planner.validate_plan) {
    if (reason) {
      *reason = "transition_planner_missing_local_rrt_adapter";
    }
    return false;
  }

  Plan candidate;
  std::string direct_reason;
  if (!planner.direct_plan(start_state, goal_state, &candidate, &direct_reason)) {
    if (reason) {
      *reason = direct_reason.empty() ? "local_rrt_direct_plan_failed" : direct_reason;
    }
    return false;
  }

  if (planner.shortcut_plan) {
    std::string shortcut_reason;
    candidate = planner.shortcut_plan(candidate, start_state, &shortcut_reason);
  }
  candidate = retime_plan_by_max_joint_speed(
    planner.densify_plan(candidate),
    start_state.getRobotModel(),
    kMaxStageJointSpeedRadS);

  std::string validation_reason;
  if (!planner.validate_plan(candidate, start_state, &validation_reason)) {
    if (reason) {
      *reason = validation_reason.empty() ? "local_rrt_validation_failed" : validation_reason;
    }
    return false;
  }

  if (plan) {
    *plan = std::move(candidate);
  }
  if (reason) {
    reason->clear();
  }
  return true;
}

bool trajectory_matches_endpoints(
  const Plan& plan,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  std::string* reason)
{
  const auto& trajectory = plan.trajectory_.joint_trajectory;
  if (trajectory.points.size() < 2 || trajectory.joint_names.empty()) {
    if (reason) *reason = "local_backend_empty_trajectory";
    return false;
  }
  constexpr double kEndpointTolerance = 1e-4;
  const auto& model_variables = start_state.getRobotModel()->getVariableNames();
  for (size_t joint_index = 0; joint_index < trajectory.joint_names.size(); ++joint_index) {
    const auto& name = trajectory.joint_names[joint_index];
    if (std::find(model_variables.begin(), model_variables.end(), name) == model_variables.end()) {
      if (reason) *reason = "local_backend_unknown_joint:" + name;
      return false;
    }
    const auto& first = trajectory.points.front();
    const auto& last = trajectory.points.back();
    if (joint_index >= first.positions.size() || joint_index >= last.positions.size()) {
      if (reason) *reason = "local_backend_position_count_mismatch";
      return false;
    }
    const double start_error = std::abs(
      first.positions[joint_index] - start_state.getVariablePosition(name));
    const double goal_error = std::abs(
      last.positions[joint_index] - goal_state.getVariablePosition(name));
    if (start_error > kEndpointTolerance || goal_error > kEndpointTolerance) {
      if (reason) *reason = "local_backend_endpoint_mismatch:" + name;
      return false;
    }
  }
  if (reason) reason->clear();
  return true;
}

bool make_valid_preferred_local_plan(
  const ExtractMonitorTransitionPlanner& planner,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  Plan* plan,
  ExtractMonitorLocalPlanMetrics* metrics,
  double* validation_ms,
  std::string* reason)
{
  if (reject_if_cancelled(planner, reason)) {
    return false;
  }
  if (!planner.preferred_local_plan || !planner.densify_plan || !planner.validate_plan) {
    if (reason) *reason = "transition_planner_missing_preferred_local_adapter";
    return false;
  }

  Plan candidate;
  std::string backend_reason;
  if (!planner.preferred_local_plan(
        start_state, goal_state, &candidate, metrics, &backend_reason)) {
    if (reason) {
      *reason = backend_reason.empty() ? "local_curobo_plan_failed" : backend_reason;
    }
    return false;
  }
  candidate = planner.densify_plan(candidate);
  std::string contract_reason;
  if (!trajectory_matches_endpoints(candidate, start_state, goal_state, &contract_reason)) {
    if (reason) *reason = contract_reason;
    return false;
  }

  std::string validation_reason;
  const auto validation_started = Clock::now();
  const bool valid = planner.validate_plan(candidate, start_state, &validation_reason);
  if (validation_ms) *validation_ms += elapsed_ms(validation_started);
  if (!valid) {
    if (reason) {
      *reason = validation_reason.empty()
        ? "local_curobo_fcl_rejected"
        : "local_curobo_fcl_rejected:" + validation_reason;
    }
    return false;
  }

  if (plan) *plan = std::move(candidate);
  if (reason) reason->clear();
  return true;
}

double variable_delta(const std::string& name, double from, double to)
{
  if (name == "updown") {
    return to - from;
  }
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double state_distance(
  const moveit::core::RobotState& a,
  const moveit::core::RobotState& b,
  const std::vector<std::string>& variable_names)
{
  double squared = 0.0;
  for (const auto& name : variable_names) {
    const double delta = variable_delta(
      name, a.getVariablePosition(name), b.getVariablePosition(name));
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

moveit::core::RobotState steer_state(
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to,
  const std::vector<std::string>& variable_names,
  double max_step)
{
  moveit::core::RobotState out(from);
  const double distance = state_distance(from, to, variable_names);
  const double ratio = distance > max_step && distance > 1e-9 ? max_step / distance : 1.0;
  for (const auto& name : variable_names) {
    const double value = from.getVariablePosition(name) +
      variable_delta(name, from.getVariablePosition(name), to.getVariablePosition(name)) * ratio;
    out.setVariablePosition(name, value);
  }
  out.enforceBounds();
  out.update(true);
  return out;
}

size_t nearest_state_index(
  const std::vector<moveit::core::RobotState>& tree,
  const moveit::core::RobotState& target,
  const std::vector<std::string>& variable_names)
{
  size_t best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree.size(); ++i) {
    const double distance = state_distance(tree[i], target, variable_names);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

bool is_arm_joint_variable(const std::string& name)
{
  return name.rfind("left_joint", 0) == 0 || name.rfind("right_joint", 0) == 0;
}

std::string arm_side_for_variable(const std::string& name)
{
  if (name.rfind("left_joint", 0) == 0) return "left";
  if (name.rfind("right_joint", 0) == 0) return "right";
  return "";
}

std::vector<std::string> custom_rrt_variable_names(
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const std::vector<std::string>& candidate_names,
  std::string* reason)
{
  std::vector<std::string> variables;
  std::string side;
  std::vector<std::string> rejected_non_arm_deltas;
  for (const auto& name : candidate_names) {
    const double delta = std::abs(variable_delta(
      name, start_state.getVariablePosition(name), goal_state.getVariablePosition(name)));
    if (delta < 1e-6) {
      continue;
    }
    if (!is_arm_joint_variable(name)) {
      rejected_non_arm_deltas.push_back(name);
      continue;
    }
    const std::string current_side = arm_side_for_variable(name);
    if (!side.empty() && current_side != side) {
      if (reason) {
        *reason = "custom_local_rrt_multi_arm_segment_not_supported";
      }
      return {};
    }
    side = current_side;
    variables.push_back(name);
  }

  if (!rejected_non_arm_deltas.empty()) {
    if (reason) {
      *reason = "custom_local_rrt_non_arm_delta_not_supported:";
      for (const auto& name : rejected_non_arm_deltas) {
        *reason += " " + name;
      }
    }
    return {};
  }
  if (variables.empty() && reason) {
    *reason = "custom_local_rrt_no_active_arm_variables";
  }
  return variables;
}

std::vector<moveit::core::RobotState> shortcut_state_path(
  const ExtractMonitorTransitionPlanner& planner,
  const std::vector<moveit::core::RobotState>& states)
{
  if (states.size() <= 2) {
    return states;
  }

  std::vector<moveit::core::RobotState> shortened;
  shortened.reserve(states.size());
  shortened.push_back(states.front());

  size_t current = 0;
  while (current + 1 < states.size()) {
    if (transition_cancelled(planner)) {
      return {};
    }
    bool advanced = false;
    for (size_t target = states.size() - 1; target > current; --target) {
      Plan segment;
      std::string segment_reason;
      if (make_valid_interpolation(planner, states[current], states[target], &segment, &segment_reason)) {
        shortened.push_back(states[target]);
        current = target;
        advanced = true;
        break;
      }
    }
    if (!advanced) {
      shortened.push_back(states[current + 1]);
      ++current;
    }
  }

  return shortened;
}

bool custom_local_rrt_bridge(
  const ExtractMonitorTransitionPlanner& planner,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const std::vector<std::string>& variable_names,
  Plan* repaired_plan,
  std::string* reason)
{
  if (reject_if_cancelled(planner, reason)) {
    return false;
  }
  if (variable_names.empty()) {
    if (reason) *reason = "custom_local_rrt_missing_variables";
    return false;
  }

  std::string variable_filter_reason;
  const std::vector<std::string> active_variable_names =
    custom_rrt_variable_names(start_state, goal_state, variable_names, &variable_filter_reason);
  if (active_variable_names.empty()) {
    if (reason) *reason = variable_filter_reason;
    return false;
  }

  Plan direct_segment;
  std::string direct_reason;
  if (make_valid_interpolation(planner, start_state, goal_state, &direct_segment, &direct_reason)) {
    if (repaired_plan) *repaired_plan = std::move(direct_segment);
    if (reason) reason->clear();
    return true;
  }

  struct Tree
  {
    std::vector<moveit::core::RobotState> states;
    std::vector<int> parents;
  };

  auto add_node = [](Tree& tree, const moveit::core::RobotState& state, int parent) {
    tree.states.push_back(state);
    tree.parents.push_back(parent);
    return tree.states.size() - 1;
  };

  auto path_to_root = [](const Tree& tree, size_t index) {
    std::vector<size_t> path;
    while (true) {
      path.push_back(index);
      if (tree.parents[index] < 0) break;
      index = static_cast<size_t>(tree.parents[index]);
    }
    std::reverse(path.begin(), path.end());
    return path;
  };

  Tree start_tree;
  Tree goal_tree;
  add_node(start_tree, start_state, -1);
  add_node(goal_tree, goal_state, -1);

  std::mt19937 rng(0xA11F2026u);
  std::uniform_real_distribution<double> blend_distribution(0.0, 1.0);
  std::uniform_real_distribution<double> joint_offset_distribution(-0.55, 0.55);
  constexpr size_t kMaxIterations = 900;
  constexpr double kStep = 0.22;
  constexpr double kConnectDistance = 0.25;

  std::string last_reason = direct_reason;
  for (size_t iter = 0; iter < kMaxIterations; ++iter) {
    if (reject_if_cancelled(planner, reason)) {
      return false;
    }
    moveit::core::RobotState sample(start_state);
    const double blend = (iter % 5 == 0) ? 1.0 : blend_distribution(rng);
    for (const auto& name : active_variable_names) {
      const double base = start_state.getVariablePosition(name) +
        variable_delta(name, start_state.getVariablePosition(name), goal_state.getVariablePosition(name)) * blend;
      const double offset = joint_offset_distribution(rng);
      sample.setVariablePosition(name, base + offset);
    }
    sample.enforceBounds();
    sample.update(true);

    Tree& active = (iter % 2 == 0) ? start_tree : goal_tree;
    Tree& other = (iter % 2 == 0) ? goal_tree : start_tree;
    const bool active_is_start = iter % 2 == 0;
    const size_t nearest = nearest_state_index(active.states, sample, active_variable_names);
    const auto next = steer_state(active.states[nearest], sample, active_variable_names, kStep);

    Plan edge;
    std::string edge_reason;
    if (!make_valid_interpolation(planner, active.states[nearest], next, &edge, &edge_reason)) {
      last_reason = edge_reason;
      continue;
    }
    const size_t next_index = add_node(active, next, static_cast<int>(nearest));

    const size_t other_nearest = nearest_state_index(other.states, next, active_variable_names);
    if (state_distance(active.states[next_index], other.states[other_nearest], active_variable_names) > kConnectDistance) {
      continue;
    }
    Plan bridge;
    std::string bridge_reason;
    if (!make_valid_interpolation(
          planner, active.states[next_index], other.states[other_nearest], &bridge, &bridge_reason)) {
      last_reason = bridge_reason;
      continue;
    }

    const auto active_path = path_to_root(active, next_index);
    const auto other_path = path_to_root(other, other_nearest);
    std::vector<moveit::core::RobotState> ordered_states;
    if (active_is_start) {
      for (const size_t index : active_path) ordered_states.push_back(active.states[index]);
      for (auto it = other_path.rbegin(); it != other_path.rend(); ++it) {
        ordered_states.push_back(other.states[*it]);
      }
    } else {
      for (const size_t index : other_path) ordered_states.push_back(other.states[index]);
      for (auto it = active_path.rbegin(); it != active_path.rend(); ++it) {
        ordered_states.push_back(active.states[*it]);
      }
    }

    ordered_states = shortcut_state_path(planner, ordered_states);
    if (ordered_states.empty() && transition_cancelled(planner)) {
      if (reason) *reason = "loaded_plan_cancelled_after_first_success";
      return false;
    }

    Plan combined;
    moveit::core::robotStateToRobotStateMsg(start_state, combined.start_state_, true);
    for (size_t i = 1; i < ordered_states.size(); ++i) {
      Plan segment;
      std::string segment_reason;
      if (!make_valid_interpolation(
            planner, ordered_states[i - 1], ordered_states[i], &segment, &segment_reason)) {
        last_reason = segment_reason;
        combined.trajectory_.joint_trajectory.points.clear();
        break;
      }
      append_segment(combined, segment);
    }
    if (!combined.trajectory_.joint_trajectory.points.empty()) {
      if (repaired_plan) *repaired_plan = std::move(combined);
      if (reason) {
        std::ostringstream stream;
        stream << "custom_local_rrt_iterations=" << iter + 1;
        *reason = stream.str();
      }
      return true;
    }
  }

  if (reason) {
    *reason = "custom_local_rrt_failed";
    if (!last_reason.empty()) *reason += ": " + last_reason;
  }
  return false;
}

bool state_is_valid(
  const ExtractMonitorTransitionPlanner& planner,
  const moveit::core::RobotState& state,
  std::string* reason)
{
  Plan stationary;
  if (!make_valid_interpolation(planner, state, state, &stationary, reason)) {
    return false;
  }
  return true;
}

struct LocalRepairStats
{
  size_t patched_segments = 0;
  size_t preferred_calls = 0;
  size_t preferred_segments = 0;
  size_t fallback_segments = 0;
  size_t window_from = std::numeric_limits<size_t>::max();
  size_t window_to = std::numeric_limits<size_t>::max();
  double service_ms = 0.0;
  double solve_ms = 0.0;
  double queue_ms = 0.0;
  double endpoint_check_ms = 0.0;
  double graph_ms = 0.0;
  double trajopt_ms = 0.0;
  double interpolation_ms = 0.0;
  double conversion_ms = 0.0;
  double stitch_ms = 0.0;
  double fcl_validation_ms = 0.0;
};

bool repair_with_local_rrt(
  const ExtractMonitorTransitionPlanner& planner,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const Plan& reference_plan,
  Plan* repaired_plan,
  LocalRepairStats* output_stats,
  std::string* reason)
{
  if (reject_if_cancelled(planner, reason)) {
    return false;
  }
  LocalRepairStats local_stats;
  LocalRepairStats& stats = output_stats ? *output_stats : local_stats;
  stats = LocalRepairStats{};
  const auto states = plan_states(reference_plan, start_state, goal_state);
  if (states.size() < 3) {
    if (reason) {
      *reason = "local_rrt_reference_path_too_short";
    }
    return false;
  }

  const size_t local_window = std::max<size_t>(1, planner.local_window_points);
  const size_t patch_backtrack = planner.local_boundary_backoff_points;
  const size_t patch_forward = planner.local_boundary_backoff_points;
  const size_t max_segments = std::max<size_t>(1, planner.local_max_segments);

  size_t patched_segments = 0;
  std::vector<Plan> accepted_segments;
  std::vector<size_t> accepted_ends{0};
  std::vector<std::string> patch_notes;
  auto current_index = [&]() {
    return accepted_ends.back();
  };
  auto append_accepted = [&](Plan segment, size_t end_index) {
    accepted_segments.push_back(std::move(segment));
    accepted_ends.push_back(end_index);
  };
  auto rollback_prefix_count = [&](size_t patch_start) {
    size_t keep_count = accepted_ends.size();
    while (keep_count > 1 && accepted_ends[keep_count - 1] > patch_start) {
      --keep_count;
    }
    return keep_count;
  };

  while (current_index() + 1 < states.size()) {
    if (reject_if_cancelled(planner, reason)) {
      return false;
    }
    const size_t current = current_index();
    const size_t max_to = std::min(states.size() - 1, current + local_window);

    bool advanced = false;
    for (size_t to = max_to; to > current; --to) {
      Plan segment;
      std::string segment_reason;
      if (make_valid_interpolation(planner, states[current], states[to], &segment, &segment_reason)) {
        append_accepted(std::move(segment), to);
        advanced = true;
        break;
      }
    }
    if (advanced) {
      continue;
    }

    std::string last_rrt_reason;
    size_t first_safe_target = states.size();
    for (size_t to = current + 1; to < states.size(); ++to) {
      if (reject_if_cancelled(planner, reason)) {
        return false;
      }
      std::string state_reason;
      if (state_is_valid(planner, states[to], &state_reason)) {
        first_safe_target = to;
        break;
      } else {
        last_rrt_reason = state_reason;
      }
    }
    if (first_safe_target == states.size()) {
      if (reason) {
        std::ostringstream stream;
        stream << "local_rrt_failed_at_waypoint_" << current
               << ": no_safe_target_after_collision";
        if (!last_rrt_reason.empty()) {
          stream << "; " << last_rrt_reason;
        }
        *reason = stream.str();
      }
      return false;
    }

    if (patched_segments >= max_segments) {
      if (reason) {
        *reason = "local_repair_max_segments_exceeded_at_waypoint_" +
          std::to_string(current);
      }
      return false;
    }

    const size_t patch_start = current > patch_backtrack ? current - patch_backtrack : 0;
    size_t patch_target = std::min(states.size() - 1, first_safe_target + patch_forward);
    while (patch_target > first_safe_target) {
      std::string target_reason;
      if (state_is_valid(planner, states[patch_target], &target_reason)) {
        break;
      }
      last_rrt_reason = target_reason;
      --patch_target;
    }
    if (patch_target <= patch_start) {
      if (reason) {
        std::ostringstream stream;
        stream << "local_rrt_failed_at_waypoint_" << current
               << ": patch_target_not_after_patch_start "
               << patch_start << "->" << patch_target;
        *reason = stream.str();
      }
      return false;
    }

    Plan segment;
    std::string segment_reason;
    bool solved = false;
    size_t solved_patch_start = patch_start;
    size_t solved_patch_target = patch_target;
    std::string solver_tag;
    std::string preferred_failure;
    if (planner.preferred_local_plan) {
      auto try_preferred = [&](size_t from, size_t to) {
        if (reject_if_cancelled(planner, &segment_reason)) {
          return false;
        }
        if (stats.preferred_calls >= planner.local_max_calls) {
          segment_reason = "local_curobo_max_calls_exceeded";
          return false;
        }
        ExtractMonitorLocalPlanMetrics metrics;
        ++stats.preferred_calls;
        const bool valid = make_valid_preferred_local_plan(
          planner,
          states[from],
          states[to],
          &segment,
          &metrics,
          &stats.fcl_validation_ms,
          &segment_reason);
        stats.service_ms += metrics.roundtrip_ms;
        stats.solve_ms += metrics.backend_solve_ms;
        stats.queue_ms += metrics.backend_queue_ms;
        stats.endpoint_check_ms += metrics.backend_endpoint_check_ms;
        stats.graph_ms += metrics.backend_graph_ms;
        stats.trajopt_ms += metrics.backend_trajopt_ms;
        stats.interpolation_ms += metrics.backend_interpolation_ms;
        stats.conversion_ms += metrics.response_conversion_ms;
        return valid;
      };

      solved = try_preferred(patch_start, patch_target);
      if (!solved &&
          segment_reason.find("curobo_start_state_in_collision_or_bounds") !=
            std::string::npos) {
        const size_t max_extra_backoff = std::min(
          patch_start, planner.local_boundary_backoff_points);
        for (size_t backoff = 1;
             backoff <= max_extra_backoff &&
             stats.preferred_calls < planner.local_max_calls;
             ++backoff) {
          if (reject_if_cancelled(planner, &segment_reason)) {
            break;
          }
          const size_t from = patch_start - backoff;
          solved_patch_start = from;
          if (try_preferred(from, patch_target)) {
            solved = true;
            break;
          }
          if (segment_reason.find("curobo_start_state_in_collision_or_bounds") ==
              std::string::npos) {
            break;
          }
        }
      }
      if (!solved &&
          segment_reason.find("curobo_goal_state_in_collision_or_bounds") !=
            std::string::npos) {
        for (size_t to = patch_target + 1;
             to < states.size() && stats.preferred_calls < planner.local_max_calls;
             ++to) {
          if (reject_if_cancelled(planner, &segment_reason)) {
            break;
          }
          std::string target_reason;
          if (!state_is_valid(planner, states[to], &target_reason)) continue;
          solved_patch_target = to;
          if (try_preferred(solved_patch_start, to)) {
            solved = true;
            break;
          }
          if (segment_reason.find("curobo_goal_state_in_collision_or_bounds") ==
              std::string::npos) {
            break;
          }
        }
      }
      if (solved) {
        ++stats.preferred_segments;
        solver_tag = planner.preferred_local_backend;
      } else {
        preferred_failure = segment_reason;
      }
    }

    const bool fallback_allowed = !planner.preferred_local_plan || planner.fallback_to_direct;
    if (!solved && fallback_allowed && !transition_cancelled(planner)) {
      solved = custom_local_rrt_bridge(
        planner,
        states[patch_start],
        states[patch_target],
        reference_plan.trajectory_.joint_trajectory.joint_names,
        &segment,
        &segment_reason);
      solver_tag = "custom_rrt";
      if (!solved && !transition_cancelled(planner)) {
        solved = make_valid_local_rrt(
          planner, states[patch_start], states[patch_target], &segment, &segment_reason);
        solver_tag = "moveit_rrt";
      }
      if (solved) ++stats.fallback_segments;
      solved_patch_start = patch_start;
      solved_patch_target = patch_target;
    }

    if (solved) {
      const size_t keep_count = rollback_prefix_count(solved_patch_start);
      if (keep_count == 0 || accepted_ends[keep_count - 1] > solved_patch_start) {
        solved = false;
        segment_reason = "local_rrt_failed_invalid_patch_prefix";
      }
      Plan prefix_segment;
      const bool needs_prefix = solved &&
        accepted_ends[keep_count - 1] < solved_patch_start;
      if (needs_prefix) {
        std::string prefix_reason;
        if (!make_valid_interpolation(
            planner,
            states[accepted_ends[keep_count - 1]],
            states[solved_patch_start],
            &prefix_segment,
            &prefix_reason)) {
          solved = false;
          segment_reason = "local_rrt_failed_patch_prefix: " + prefix_reason;
        }
      }
      if (!solved) {
        last_rrt_reason = segment_reason;
      } else {
        accepted_segments.resize(keep_count - 1);
        accepted_ends.resize(keep_count);
        if (needs_prefix) {
          append_accepted(std::move(prefix_segment), solved_patch_start);
        }
        append_accepted(std::move(segment), solved_patch_target);
        ++patched_segments;
        stats.window_from = std::min(stats.window_from, solved_patch_start);
        stats.window_to = stats.window_to == std::numeric_limits<size_t>::max()
          ? solved_patch_target
          : std::max(stats.window_to, solved_patch_target);
        advanced = true;
        patch_notes.push_back(
          solver_tag + ":" + std::to_string(solved_patch_start) + "->" +
          std::to_string(solved_patch_target) + "/" + std::to_string(states.size() - 1));
        if (!preferred_failure.empty()) {
          patch_notes.push_back("curobo_failed:" + preferred_failure);
        }
      }
    } else {
      last_rrt_reason = preferred_failure.empty() ? segment_reason : preferred_failure;
      if (!preferred_failure.empty() && !segment_reason.empty() &&
          preferred_failure != segment_reason) {
        last_rrt_reason += "; fallback_failed:" + segment_reason;
      }
    }

    if (!advanced) {
      if (reason) {
        std::ostringstream stream;
        stream << "local_repair_failed_at_waypoint_" << current;
        if (!last_rrt_reason.empty()) {
          stream << ": " << last_rrt_reason;
        }
        *reason = stream.str();
      }
      return false;
    }
  }

  Plan combined;
  moveit::core::robotStateToRobotStateMsg(start_state, combined.start_state_, true);
  const auto stitch_started = Clock::now();
  for (const auto& segment : accepted_segments) {
    append_segment(combined, segment);
  }
  stats.stitch_ms += elapsed_ms(stitch_started);
  combined.planning_time_ = 0.0;
  if (repaired_plan) {
    *repaired_plan = std::move(combined);
  }
  stats.patched_segments = patched_segments;
  if (reason) {
    std::ostringstream stream;
    stream << "local_repair_segments=" << patched_segments;
    if (!patch_notes.empty()) {
      stream << " patched_to_waypoints=";
      for (size_t i = 0; i < patch_notes.size(); ++i) {
        if (i > 0) {
          stream << ",";
        }
        stream << patch_notes[i];
      }
    }
    *reason = stream.str();
  }
  return true;
}

}  // namespace

ExtractMonitorTransitionPlanResult ExtractMonitorTransitionPlanner::plan(
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state) const
{
  ExtractMonitorTransitionPlanResult result;
  if (reject_if_cancelled(*this, &result.failure_reason)) {
    return result;
  }
  if (!make_interpolated_plan || !densify_plan || !validate_plan) {
    result.failure_reason = "transition_planner_missing_basic_adapter";
    return result;
  }

  result.method = "joint_interpolation";
  result.plan = retime_plan_by_max_joint_speed(
    densify_plan(make_interpolated_plan(start_state, goal_state, 1.0)),
    start_state.getRobotModel(),
    kMaxStageJointSpeedRadS);
  const auto initial_validation_started = Clock::now();
  result.valid = validate_plan(result.plan, start_state, &result.failure_reason);
  result.final_fcl_validation_ms = elapsed_ms(initial_validation_started);
  result.final_fcl_valid = result.valid;
  if (result.valid) {
    result.failure_reason.clear();
    return result;
  }

  if (reject_if_cancelled(*this, &result.failure_reason)) {
    result.valid = false;
    return result;
  }

  const auto& reference_plan = result.plan;
  const bool can_try_local_repair =
    (preferred_local_plan || direct_plan) &&
    reference_plan.trajectory_.joint_trajectory.points.size() >= 2 &&
    !reference_plan.trajectory_.joint_trajectory.joint_names.empty();
  if (can_try_local_repair) {
    const std::string original_failure = result.failure_reason;
    LocalRepairStats repair_stats;
    std::string local_rrt_reason;
    const auto repair_started = Clock::now();
    const bool repaired = repair_with_local_rrt(
      *this, start_state, goal_state, reference_plan,
      &result.plan, &repair_stats, &local_rrt_reason);
    result.local_repair_total_ms = elapsed_ms(repair_started);
    result.local_window_from = repair_stats.window_from;
    result.local_window_to = repair_stats.window_to;
    result.local_preferred_call_count = repair_stats.preferred_calls;
    result.local_preferred_segment_count = repair_stats.preferred_segments;
    result.local_fallback_segment_count = repair_stats.fallback_segments;
    result.local_service_ms = repair_stats.service_ms;
    result.local_solve_ms = repair_stats.solve_ms;
    result.local_queue_ms = repair_stats.queue_ms;
    result.local_endpoint_check_ms = repair_stats.endpoint_check_ms;
    result.local_graph_ms = repair_stats.graph_ms;
    result.local_trajopt_ms = repair_stats.trajopt_ms;
    result.local_interpolation_ms = repair_stats.interpolation_ms;
    result.local_conversion_ms = repair_stats.conversion_ms;
    result.local_stitch_ms = repair_stats.stitch_ms;
    result.local_fcl_validation_ms = repair_stats.fcl_validation_ms;
    result.local_fallback_used = preferred_local_plan && repair_stats.fallback_segments > 0;
    if (repaired) {
      if (repair_stats.preferred_segments > 0 && repair_stats.fallback_segments == 0) {
        result.method = "shortcut_local_curobo";
        result.local_repair_backend = preferred_local_backend;
      } else if (repair_stats.patched_segments > 0) {
        result.method = "shortcut_local_rrt";
        result.local_repair_backend = repair_stats.preferred_segments > 0
          ? preferred_local_backend + "+rrt"
          : "rrt";
      } else {
        result.method = "joint_interpolation";
      }
      result.failure_reason = local_rrt_reason;
      result.plan = retime_plan_by_max_joint_speed(
        densify_plan(result.plan),
        start_state.getRobotModel(),
        kMaxStageJointSpeedRadS);
      std::string validation_reason;
      const auto final_validation_started = Clock::now();
      result.valid = validate_plan(result.plan, start_state, &validation_reason);
      result.final_fcl_validation_ms = elapsed_ms(final_validation_started);
      result.final_fcl_valid = result.valid;
      if (!validation_reason.empty()) {
        result.failure_reason = result.failure_reason.empty()
          ? validation_reason
          : result.failure_reason + "; " + validation_reason;
      }
      return result;
    }

    result.method = preferred_local_plan ? "shortcut_local_curobo" : "shortcut_local_rrt";
    result.local_repair_backend = preferred_local_plan ? preferred_local_backend : "rrt";
    result.valid = false;
    result.failure_reason = original_failure;
    if (!local_rrt_reason.empty()) {
      result.failure_reason = result.failure_reason.empty()
        ? local_rrt_reason
        : result.failure_reason + "; " + local_rrt_reason;
    }
    return result;
  }

  if (!direct_plan) {
    return result;
  }

  result.method = "rrt";
  result.failure_reason.clear();
  if (!direct_plan(start_state, goal_state, &result.plan, &result.failure_reason)) {
    result.valid = false;
    return result;
  }

  if (shortcut_plan) {
    std::string shortcut_reason;
    result.plan = shortcut_plan(result.plan, start_state, &shortcut_reason);
    result.failure_reason = shortcut_reason;
  } else {
    result.failure_reason.clear();
  }
  result.plan = retime_plan_by_max_joint_speed(
    densify_plan(result.plan),
    start_state.getRobotModel(),
    kMaxStageJointSpeedRadS);

  std::string validation_reason;
  const auto final_validation_started = Clock::now();
  result.valid = validate_plan(result.plan, start_state, &validation_reason);
  result.final_fcl_validation_ms = elapsed_ms(final_validation_started);
  result.final_fcl_valid = result.valid;
  if (!result.failure_reason.empty() && !validation_reason.empty()) {
    result.failure_reason += "; " + validation_reason;
  } else if (!validation_reason.empty()) {
    result.failure_reason = validation_reason;
  }
  if (result.valid && result.failure_reason.empty()) {
    return result;
  }
  return result;
}

}  // namespace alfa_robot::motion
