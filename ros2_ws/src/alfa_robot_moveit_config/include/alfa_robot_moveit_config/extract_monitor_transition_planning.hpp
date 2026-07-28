#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <functional>
#include <limits>
#include <string>

namespace alfa_robot::motion
{

struct ExtractMonitorTransitionPlanResult
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool valid = false;
  std::string method;
  std::string failure_reason;
  std::string local_repair_backend;
  size_t local_window_from = std::numeric_limits<size_t>::max();
  size_t local_window_to = std::numeric_limits<size_t>::max();
  size_t local_preferred_call_count = 0;
  size_t local_preferred_segment_count = 0;
  size_t local_fallback_segment_count = 0;
  double local_service_ms = 0.0;
  double local_solve_ms = 0.0;
  double local_queue_ms = 0.0;
  double local_endpoint_check_ms = 0.0;
  double local_graph_ms = 0.0;
  double local_trajopt_ms = 0.0;
  double local_interpolation_ms = 0.0;
  double local_conversion_ms = 0.0;
  double local_stitch_ms = 0.0;
  double local_fcl_validation_ms = 0.0;
  double local_repair_total_ms = 0.0;
  double final_fcl_validation_ms = 0.0;
  bool local_fallback_used = false;
  bool final_fcl_valid = false;
};

struct ExtractMonitorLocalPlanMetrics
{
  std::string planner_method;
  double roundtrip_ms = 0.0;
  double backend_total_ms = 0.0;
  double backend_solve_ms = 0.0;
  double backend_queue_ms = 0.0;
  double backend_endpoint_check_ms = 0.0;
  double backend_graph_ms = 0.0;
  double backend_trajopt_ms = 0.0;
  double backend_interpolation_ms = 0.0;
  double response_conversion_ms = 0.0;
};

using ExtractMonitorMakeJointPlan =
  std::function<moveit::planning_interface::MoveGroupInterface::Plan(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    double duration_s)>;

using ExtractMonitorDensifyPlan =
  std::function<moveit::planning_interface::MoveGroupInterface::Plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan)>;

using ExtractMonitorValidatePlan =
  std::function<bool(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    std::string* reason)>;

using ExtractMonitorDirectPlan =
  std::function<bool(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    moveit::planning_interface::MoveGroupInterface::Plan* plan,
    std::string* reason)>;

using ExtractMonitorPreferredLocalPlan =
  std::function<bool(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    moveit::planning_interface::MoveGroupInterface::Plan* plan,
    ExtractMonitorLocalPlanMetrics* metrics,
    std::string* reason)>;

using ExtractMonitorShortcutPlan =
  std::function<moveit::planning_interface::MoveGroupInterface::Plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    std::string* reason)>;

using ExtractMonitorCancellationCheck = std::function<bool()>;

struct ExtractMonitorTransitionPlanner
{
  ExtractMonitorMakeJointPlan make_interpolated_plan;
  ExtractMonitorDensifyPlan densify_plan;
  ExtractMonitorValidatePlan validate_plan;
  ExtractMonitorPreferredLocalPlan preferred_local_plan;
  ExtractMonitorDirectPlan direct_plan;
  ExtractMonitorShortcutPlan shortcut_plan;
  ExtractMonitorCancellationCheck is_cancelled;
  std::string preferred_local_backend = "curobo";
  size_t local_window_points = 8;
  size_t local_boundary_backoff_points = 5;
  size_t local_max_segments = 4;
  size_t local_max_calls = 8;
  bool fallback_to_direct = true;

  ExtractMonitorTransitionPlanResult plan(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state) const;
};

}  // namespace alfa_robot::motion
