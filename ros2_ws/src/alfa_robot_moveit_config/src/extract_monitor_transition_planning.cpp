#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"

#include <moveit/robot_state/conversions.h>
#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

namespace
{

using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

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
  if (!planner.make_interpolated_plan || !planner.densify_plan || !planner.validate_plan) {
    if (reason) {
      *reason = "transition_planner_missing_basic_adapter";
    }
    return false;
  }

  Plan candidate = planner.densify_plan(
    planner.make_interpolated_plan(start_state, goal_state, 1.0));
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
  candidate = planner.densify_plan(candidate);

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

bool repair_with_local_rrt(
  const ExtractMonitorTransitionPlanner& planner,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const Plan& reference_plan,
  Plan* repaired_plan,
  size_t* local_rrt_segments,
  std::string* reason)
{
  const auto states = plan_states(reference_plan, start_state, goal_state);
  if (states.size() < 3) {
    if (reason) {
      *reason = "local_rrt_reference_path_too_short";
    }
    return false;
  }

  constexpr size_t kLocalWindow = 8;
  Plan combined;
  moveit::core::robotStateToRobotStateMsg(start_state, combined.start_state_, true);

  size_t patched_segments = 0;
  size_t current = 0;
  std::vector<std::string> patch_notes;
  while (current + 1 < states.size()) {
    const size_t max_to = std::min(states.size() - 1, current + kLocalWindow);

    bool advanced = false;
    for (size_t to = max_to; to > current; --to) {
      Plan segment;
      std::string segment_reason;
      if (make_valid_interpolation(planner, states[current], states[to], &segment, &segment_reason)) {
        append_segment(combined, segment);
        current = to;
        advanced = true;
        break;
      }
    }
    if (advanced) {
      continue;
    }

    std::string last_rrt_reason;
    for (size_t to = max_to; to > current; --to) {
      Plan segment;
      std::string segment_reason;
      if (make_valid_local_rrt(planner, states[current], states[to], &segment, &segment_reason)) {
        append_segment(combined, segment);
        ++patched_segments;
        current = to;
        advanced = true;
        patch_notes.push_back(
          std::to_string(current) + "/" + std::to_string(states.size() - 1));
        break;
      }
      last_rrt_reason = segment_reason;
    }

    if (!advanced) {
      if (reason) {
        std::ostringstream stream;
        stream << "local_rrt_failed_at_waypoint_" << current;
        if (!last_rrt_reason.empty()) {
          stream << ": " << last_rrt_reason;
        }
        *reason = stream.str();
      }
      return false;
    }
  }

  combined.planning_time_ = 0.0;
  if (repaired_plan) {
    *repaired_plan = std::move(combined);
  }
  if (local_rrt_segments) {
    *local_rrt_segments = patched_segments;
  }
  if (reason) {
    std::ostringstream stream;
    stream << "local_rrt_segments=" << patched_segments;
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
  if (!make_interpolated_plan || !densify_plan || !validate_plan) {
    result.failure_reason = "transition_planner_missing_basic_adapter";
    return result;
  }

  result.method = "joint_interpolation";
  result.plan = densify_plan(make_interpolated_plan(start_state, goal_state, 1.0));
  result.valid = validate_plan(result.plan, start_state, &result.failure_reason);
  if (result.valid) {
    result.failure_reason.clear();
    return result;
  }

  const auto& reference_plan = result.plan;
  const bool can_try_local_rrt =
    direct_plan &&
    reference_plan.trajectory_.joint_trajectory.points.size() >= 2 &&
    !reference_plan.trajectory_.joint_trajectory.joint_names.empty();
  if (can_try_local_rrt) {
    const std::string original_failure = result.failure_reason;
    size_t local_rrt_segments = 0;
    std::string local_rrt_reason;
    if (repair_with_local_rrt(
          *this, start_state, goal_state, reference_plan,
          &result.plan, &local_rrt_segments, &local_rrt_reason)) {
      result.method = local_rrt_segments > 0 ? "shortcut_local_rrt" : "joint_interpolation";
      result.failure_reason = local_rrt_reason;
      result.plan = densify_plan(result.plan);
      std::string validation_reason;
      result.valid = validate_plan(result.plan, start_state, &validation_reason);
      if (!validation_reason.empty()) {
        result.failure_reason = result.failure_reason.empty()
          ? validation_reason
          : result.failure_reason + "; " + validation_reason;
      }
      return result;
    }

    result.method = "shortcut_local_rrt";
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
  result.plan = densify_plan(result.plan);

  std::string validation_reason;
  result.valid = validate_plan(result.plan, start_state, &validation_reason);
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
