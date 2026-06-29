#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <functional>
#include <string>

namespace alfa_robot::motion
{

struct ExtractMonitorTransitionPlanResult
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool valid = false;
  std::string method;
  std::string failure_reason;
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

using ExtractMonitorShortcutPlan =
  std::function<moveit::planning_interface::MoveGroupInterface::Plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    std::string* reason)>;

struct ExtractMonitorTransitionPlanner
{
  ExtractMonitorMakeJointPlan make_interpolated_plan;
  ExtractMonitorDensifyPlan densify_plan;
  ExtractMonitorValidatePlan validate_plan;
  ExtractMonitorDirectPlan direct_plan;
  ExtractMonitorShortcutPlan shortcut_plan;

  ExtractMonitorTransitionPlanResult plan(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state) const;
};

}  // namespace alfa_robot::motion
