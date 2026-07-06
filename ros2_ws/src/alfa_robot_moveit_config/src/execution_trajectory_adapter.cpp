#include "alfa_robot_moveit_config/execution_trajectory_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>

namespace alfa_robot::motion
{

ExecutionTrajectoryAdapter::ExecutionTrajectoryAdapter(ExecutionTrajectoryAdapterConfig config)
: config_(std::move(config))
{}

std::vector<std::string> ExecutionTrajectoryAdapter::targetJointNames() const
{
  std::vector<std::string> names = {
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
  };
  if (config_.include_turn) {
    names.push_back("turn");
  }
  return names;
}

std::string ExecutionTrajectoryAdapter::alfaToMoveItJointName(const std::string& name) const
{
  if (name.rfind("left_joint", 0) == 0) {
    return "leftjoint" + name.substr(std::string("left_joint").size());
  }
  if (name.rfind("right_joint", 0) == 0) {
    return "rightjoint" + name.substr(std::string("right_joint").size());
  }
  return name;
}

std::string ExecutionTrajectoryAdapter::moveItToAlfaJointName(const std::string& name) const
{
  if (name.rfind("leftjoint", 0) == 0) {
    return "left_joint" + name.substr(std::string("leftjoint").size());
  }
  if (name.rfind("rightjoint", 0) == 0) {
    return "right_joint" + name.substr(std::string("rightjoint").size());
  }
  return name;
}

bool ExecutionTrajectoryAdapter::buildGoal(
  const ExecutionTrajectoryBuildRequest& request,
  FollowJointTrajectory::Goal* goal,
  std::string* reason) const
{
  if (!goal) return false;
  if (!request.source) {
    if (reason) *reason = "source trajectory is null";
    return false;
  }
  const auto& source = *request.source;
  if (source.points.empty()) {
    if (reason) *reason = "source trajectory is empty";
    return false;
  }

  const auto target_names = targetJointNames();
  std::vector<int> source_indices;
  source_indices.reserve(target_names.size());
  for (const auto& target_name : target_names) {
    const auto moveit_name = alfaToMoveItJointName(target_name);
    const auto it = std::find(source.joint_names.begin(), source.joint_names.end(), moveit_name);
    if (it == source.joint_names.end()) {
      if (!config_.allow_hold_missing_target_joints) {
        if (reason) *reason = "missing planned joint " + moveit_name + " for target " + target_name;
        return false;
      }
      source_indices.push_back(-1);
    } else {
      source_indices.push_back(static_cast<int>(std::distance(source.joint_names.begin(), it)));
    }
  }

  if (config_.reject_unmapped_planned_joints) {
    for (const auto& planned_name : source.joint_names) {
      const auto alfa_name = moveItToAlfaJointName(planned_name);
      if (std::find(target_names.begin(), target_names.end(), alfa_name) != target_names.end()) {
        continue;
      }
      if (plannedJointChanges(source, planned_name)) {
        if (reason) {
          *reason = "planned joint " + planned_name +
            " changes but is not mapped to alfa execution target joints";
        }
        return false;
      }
    }
  }

  goal->trajectory = trajectory_msgs::msg::JointTrajectory();
  goal->trajectory.header = source.header;
  goal->trajectory.joint_names = target_names;
  goal->trajectory.points.reserve(source.points.size());

  for (const auto& source_point : source.points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = source_point.time_from_start;
    point.positions.reserve(target_names.size());
    if (!source_point.velocities.empty()) point.velocities.reserve(target_names.size());
    if (!source_point.accelerations.empty()) point.accelerations.reserve(target_names.size());
    if (!source_point.effort.empty()) point.effort.reserve(target_names.size());

    for (size_t i = 0; i < target_names.size(); ++i) {
      const int source_index = source_indices[i];
      if (source_index >= 0) {
        const auto index = static_cast<size_t>(source_index);
        point.positions.push_back(index < source_point.positions.size() ? source_point.positions[index] : 0.0);
        if (!source_point.velocities.empty()) {
          point.velocities.push_back(index < source_point.velocities.size() ? source_point.velocities[index] : 0.0);
        }
        if (!source_point.accelerations.empty()) {
          point.accelerations.push_back(index < source_point.accelerations.size() ? source_point.accelerations[index] : 0.0);
        }
        if (!source_point.effort.empty()) {
          point.effort.push_back(index < source_point.effort.size() ? source_point.effort[index] : 0.0);
        }
      } else {
        const auto moveit_name = alfaToMoveItJointName(target_names[i]);
        const bool has_variable = request.is_robot_variable ? request.is_robot_variable(moveit_name) : false;
        const double hold_position =
          has_variable && request.hold_position ? request.hold_position(moveit_name) : 0.0;
        point.positions.push_back(hold_position);
        if (!source_point.velocities.empty()) point.velocities.push_back(0.0);
        if (!source_point.accelerations.empty()) point.accelerations.push_back(0.0);
        if (!source_point.effort.empty()) point.effort.push_back(0.0);
      }
    }
    goal->trajectory.points.push_back(std::move(point));
  }
  return true;
}

bool ExecutionTrajectoryAdapter::robotStateMatches(const ExecutionStateMatchRequest& request) const
{
  if (!request.is_robot_variable || !request.goal_position || !request.current_position) {
    return false;
  }
  for (const auto& name : request.target_names) {
    if (!request.is_robot_variable(name)) {
      continue;
    }
    const double error = std::abs(request.current_position(name) - request.goal_position(name));
    if (error > request.tolerance) {
      return false;
    }
  }
  return true;
}

bool ExecutionTrajectoryAdapter::jointStateMatches(const ExecutionJointStateMatchRequest& request) const
{
  if (!request.current || !request.is_robot_variable || !request.goal_position) {
    return false;
  }
  const auto& current = *request.current;
  for (const auto& name : request.target_names) {
    auto it = std::find(current.name.begin(), current.name.end(), name);
    if (it == current.name.end()) {
      const auto alfa_name = moveItToAlfaJointName(name);
      it = std::find(current.name.begin(), current.name.end(), alfa_name);
    }
    if (it == current.name.end()) {
      continue;
    }
    const size_t index = static_cast<size_t>(std::distance(current.name.begin(), it));
    if (index >= current.position.size()) {
      continue;
    }
    if (!request.is_robot_variable(name)) {
      continue;
    }
    const double error = std::abs(current.position[index] - request.goal_position(name));
    if (error > request.tolerance) {
      return false;
    }
  }
  return true;
}

bool ExecutionTrajectoryAdapter::plannedJointChanges(
  const trajectory_msgs::msg::JointTrajectory& source,
  const std::string& joint_name)
{
  const auto it = std::find(source.joint_names.begin(), source.joint_names.end(), joint_name);
  if (it == source.joint_names.end()) return false;
  const auto index = static_cast<size_t>(std::distance(source.joint_names.begin(), it));
  std::optional<double> first_value;
  for (const auto& point : source.points) {
    if (index >= point.positions.size()) continue;
    if (!first_value) {
      first_value = point.positions[index];
      continue;
    }
    if (std::abs(point.positions[index] - *first_value) > 1e-6) {
      return true;
    }
  }
  return false;
}

}  // namespace alfa_robot::motion
