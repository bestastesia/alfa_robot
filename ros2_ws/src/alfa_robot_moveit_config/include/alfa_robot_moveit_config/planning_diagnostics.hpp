#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <string>

namespace alfa_robot::motion
{

std::string scene_collision_reason(
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& state,
  const moveit::core::JointModelGroup* group);

std::string group_bounds_reason(
  const moveit::core::RobotState& state,
  const moveit::core::JointModelGroup* group);

bool clamp_variable_to_bounds_if_near(
  moveit::core::RobotState* state,
  const std::string& variable_name,
  double tolerance);

std::string direct_pipeline_failure_diagnostic(
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const moveit::core::JointModelGroup* group);

}  // namespace alfa_robot::motion
