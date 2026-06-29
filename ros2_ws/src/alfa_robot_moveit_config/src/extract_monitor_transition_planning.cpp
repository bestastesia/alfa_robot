#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"

namespace alfa_robot::motion
{

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
