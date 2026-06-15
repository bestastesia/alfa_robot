#include "alfa_robot_moveit_config/loaded_pose_selector.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace alfa_robot::motion
{

LoadedPoseSelector::LoadedPoseSelector(LoadedPoseSelectorConfig config)
: config_(std::move(config))
{
  if (config_.left_pose_family.empty()) {
    throw std::invalid_argument("LoadedPoseSelector requires at least one left loaded pose");
  }
  if (config_.right_pose_family.empty()) {
    throw std::invalid_argument("LoadedPoseSelector requires at least one right loaded pose");
  }
  config_.left_preferred_index = std::min(config_.left_preferred_index, config_.left_pose_family.size() - 1);
  config_.right_preferred_index = std::min(config_.right_preferred_index, config_.right_pose_family.size() - 1);
}

bool LoadedPoseSelector::hasVariable(const moveit::core::RobotState& state, const std::string& name) const
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
}

std::string LoadedPoseSelector::jointName(const std::string& side, size_t index)
{
  return side + "_v5_joint" + std::to_string(index + 1);
}

double LoadedPoseSelector::armPoseDistance(
  const moveit::core::RobotState& state,
  const std::string& side,
  const std::vector<double>& pose) const
{
  if (pose.size() < 6) return std::numeric_limits<double>::infinity();
  double squared_sum = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const std::string name = jointName(side, i);
    if (!hasVariable(state, name)) {
      return std::numeric_limits<double>::infinity();
    }
    const double diff = shortest_angular_distance(state.getVariablePosition(name), pose[i]);
    squared_sum += diff * diff;
  }
  return std::sqrt(squared_sum);
}

size_t LoadedPoseSelector::nearestPoseIndex(
  const moveit::core::RobotState& state,
  const std::string& side,
  const std::vector<std::vector<double>>& family,
  double* distance) const
{
  size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < family.size(); ++i) {
    const double candidate_distance = armPoseDistance(state, side, family[i]);
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best_index = i;
    }
  }
  if (distance) {
    *distance = best_distance;
  }
  return best_index;
}

std::array<double, 3> LoadedPoseSelector::distanceMetrics(
  const moveit::core::RobotState& state,
  size_t left_index,
  size_t right_index) const
{
  if (left_index >= config_.left_pose_family.size() || right_index >= config_.right_pose_family.size()) {
    return {
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()
    };
  }
  const auto& left_pose = config_.left_pose_family[left_index];
  const auto& right_pose = config_.right_pose_family[right_index];
  double abs_sum = 0.0;
  double squared_sum = 0.0;
  double max_delta = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const std::array<std::pair<std::string, const std::vector<double>*>, 2> arms = {{
      {jointName("left", i), &left_pose},
      {jointName("right", i), &right_pose},
    }};
    for (const auto& [name, pose] : arms) {
      if (!hasVariable(state, name) || pose->size() <= i) {
        return {
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity()
        };
      }
      const double delta = std::abs(shortest_angular_distance(state.getVariablePosition(name), (*pose)[i]));
      abs_sum += delta;
      squared_sum += delta * delta;
      max_delta = std::max(max_delta, delta);
    }
  }
  return {abs_sum, std::sqrt(squared_sum), max_delta};
}

LoadedPoseSelection LoadedPoseSelector::select(const moveit::core::RobotState& state) const
{
  LoadedPoseSelection selection;
  selection.left_index = nearestPoseIndex(
    state, "left", config_.left_pose_family, &selection.left_distance);
  selection.right_index = nearestPoseIndex(
    state, "right", config_.right_pose_family, &selection.right_distance);
  const auto metrics = distanceMetrics(state, selection.left_index, selection.right_index);
  selection.distance_sum = metrics[0];
  selection.distance_l2 = metrics[1];
  selection.max_joint_delta = metrics[2];
  return selection;
}

moveit::core::RobotState LoadedPoseSelector::makeGoalState(
  const moveit::core::RobotState& start_state,
  LoadedPoseSelection* selection) const
{
  moveit::core::RobotState goal_state(start_state);
  LoadedPoseSelection local_selection = select(start_state);
  if (selection) {
    *selection = local_selection;
  }

  const auto& left_pose = config_.left_pose_family[local_selection.left_index];
  const auto& right_pose = config_.right_pose_family[local_selection.right_index];
  if (hasVariable(goal_state, "updown")) {
    goal_state.setVariablePosition("updown", config_.target_updown);
  }
  for (size_t i = 0; i < left_pose.size(); ++i) {
    goal_state.setVariablePosition(jointName("left", i), left_pose[i]);
  }
  for (size_t i = 0; i < right_pose.size(); ++i) {
    goal_state.setVariablePosition(jointName("right", i), right_pose[i]);
  }
  if (config_.enforce_bounds_group) {
    goal_state.enforceBounds(config_.enforce_bounds_group);
  }
  goal_state.update();
  return goal_state;
}

}  // namespace alfa_robot::motion
