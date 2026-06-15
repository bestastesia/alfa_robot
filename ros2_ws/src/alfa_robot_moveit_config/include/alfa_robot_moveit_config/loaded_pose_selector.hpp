#pragma once

#include <moveit/robot_state/robot_state.h>

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct LoadedPoseSelection
{
  size_t left_index = 0;
  size_t right_index = 0;
  double left_distance = std::numeric_limits<double>::infinity();
  double right_distance = std::numeric_limits<double>::infinity();
  double distance_sum = std::numeric_limits<double>::infinity();
  double distance_l2 = std::numeric_limits<double>::infinity();
  double max_joint_delta = std::numeric_limits<double>::infinity();
};

struct LoadedPoseSelectorConfig
{
  std::vector<std::vector<double>> left_pose_family;
  std::vector<std::vector<double>> right_pose_family;
  size_t left_preferred_index = 0;
  size_t right_preferred_index = 0;
  double target_updown = 0.3;
  const moveit::core::JointModelGroup* enforce_bounds_group = nullptr;
};

class LoadedPoseSelector
{
public:
  explicit LoadedPoseSelector(LoadedPoseSelectorConfig config);

  const LoadedPoseSelectorConfig& config() const { return config_; }
  const std::vector<std::vector<double>>& leftPoseFamily() const { return config_.left_pose_family; }
  const std::vector<std::vector<double>>& rightPoseFamily() const { return config_.right_pose_family; }
  size_t leftPreferredIndex() const { return config_.left_preferred_index; }
  size_t rightPreferredIndex() const { return config_.right_preferred_index; }

  double armPoseDistance(
    const moveit::core::RobotState& state,
    const std::string& side,
    const std::vector<double>& pose) const;

  size_t nearestPoseIndex(
    const moveit::core::RobotState& state,
    const std::string& side,
    const std::vector<std::vector<double>>& family,
    double* distance = nullptr) const;

  LoadedPoseSelection select(const moveit::core::RobotState& state) const;

  std::array<double, 3> distanceMetrics(
    const moveit::core::RobotState& state,
    size_t left_index,
    size_t right_index) const;

  moveit::core::RobotState makeGoalState(
    const moveit::core::RobotState& start_state,
    LoadedPoseSelection* selection = nullptr) const;

private:
  bool hasVariable(const moveit::core::RobotState& state, const std::string& name) const;
  static std::string jointName(const std::string& side, size_t index);

  LoadedPoseSelectorConfig config_;
};

}  // namespace alfa_robot::motion
