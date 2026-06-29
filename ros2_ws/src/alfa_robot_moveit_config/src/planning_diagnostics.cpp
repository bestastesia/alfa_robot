#include "alfa_robot_moveit_config/planning_diagnostics.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/joint_model.h>

#include <sstream>

namespace alfa_robot::motion
{

std::string scene_collision_reason(
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& state,
  const moveit::core::JointModelGroup* group)
{
  if (!scene) return "scene_missing";
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.contacts = true;
  request.max_contacts = 5;
  request.max_contacts_per_pair = 1;
  if (group) {
    request.group_name = group->getName();
  }
  scene->checkCollision(request, result, state);
  if (!result.collision) return "";
  std::ostringstream out;
  out << "collision";
  size_t count = 0;
  for (const auto& entry : result.contacts) {
    if (count == 0) {
      out << ":";
    } else {
      out << ",";
    }
    out << entry.first.first << "<->" << entry.first.second;
    ++count;
    if (count >= 3) break;
  }
  return out.str();
}

std::string group_bounds_reason(
  const moveit::core::RobotState& state,
  const moveit::core::JointModelGroup* group)
{
  if (!group) return "bounds_missing_group";
  if (state.satisfiesBounds(group)) return "";
  const auto robot_model = state.getRobotModel();
  std::ostringstream out;
  out << "bounds";
  size_t count = 0;
  for (const auto& name : group->getVariableNames()) {
    const auto& bounds = robot_model->getVariableBounds(name);
    const double value = state.getVariablePosition(name);
    bool bad = false;
    if (bounds.position_bounded_) {
      bad = value < bounds.min_position_ - 1e-9 || value > bounds.max_position_ + 1e-9;
    }
    if (!bad) continue;
    out << (count == 0 ? ":" : ",")
        << name << "=" << value
        << "[" << bounds.min_position_ << "," << bounds.max_position_ << "]";
    ++count;
    if (count >= 4) break;
  }
  return out.str();
}

std::string direct_pipeline_failure_diagnostic(
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const moveit::core::JointModelGroup* group)
{
  if (!group) return "diagnostic=missing_group";
  const auto robot_model = start_state.getRobotModel();

  auto state_status = [&](const char* label, const moveit::core::RobotState& state) {
    std::ostringstream out;
    const std::string bounds = group_bounds_reason(state, group);
    out << label << "_bounds=" << (bounds.empty() ? "ok" : bounds);
    const std::string collision = scene_collision_reason(scene, state, group);
    out << "," << label << "_collision=" << (collision.empty() ? "clear" : collision);
    return out.str();
  };

  std::ostringstream out;
  out << "diagnostic{"
      << state_status("start", start_state) << ";"
      << state_status("goal", goal_state);

  if (!start_state.satisfiesBounds(group) || !goal_state.satisfiesBounds(group)) {
    out << ";line=skipped_bounds}";
    return out.str();
  }
  const std::string start_collision = scene_collision_reason(scene, start_state, group);
  const std::string goal_collision = scene_collision_reason(scene, goal_state, group);
  if (!start_collision.empty() || !goal_collision.empty()) {
    out << ";line=skipped_endpoint_collision}";
    return out.str();
  }

  moveit::core::RobotState probe(start_state);
  const auto& variable_names = group->getVariableNames();
  constexpr int kInterpolationSteps = 50;
  for (int step = 1; step < kInterpolationSteps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(kInterpolationSteps);
    for (const auto& name : variable_names) {
      const double start_value = start_state.getVariablePosition(name);
      const double goal_value = goal_state.getVariablePosition(name);
      const auto* variable_joint = robot_model->getJointOfVariable(name);
      const bool angular_variable =
        variable_joint && variable_joint->getType() != moveit::core::JointModel::PRISMATIC;
      const double delta = angular_variable ?
        shortest_angular_distance(start_value, goal_value) :
        (goal_value - start_value);
      probe.setVariablePosition(name, start_value + delta * t);
    }
    probe.update(true);
    const std::string bounds = group_bounds_reason(probe, group);
    if (!bounds.empty()) {
      out << ";line=first_" << bounds << "@" << step << "/" << kInterpolationSteps << "}";
      return out.str();
    }
    const std::string collision = scene_collision_reason(scene, probe, group);
    if (!collision.empty()) {
      out << ";line=first_" << collision << "@" << step << "/" << kInterpolationSteps << "}";
      return out.str();
    }
  }

  out << ";line=straight_joint_interpolation_clear}";
  return out.str();
}

}  // namespace alfa_robot::motion
