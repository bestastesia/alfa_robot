#pragma once

#include "alfa_robot_moveit_config/extract_planner_types.hpp"

#include <moveit/robot_state/robot_state.h>

#include <string>

namespace moveit::core
{
class JointModelGroup;
}

namespace alfa_robot::motion
{

struct ExtractCandidateScorerConfig
{
  const moveit::core::JointModelGroup* left_arm_group = nullptr;
  const moveit::core::JointModelGroup* right_arm_group = nullptr;
  std::string left_tip = "left_v5_tool0";
  std::string right_tip = "right_v5_tool0";
  double step_x = 0.03;
  double lift_weight = 10.0;
  double pitch_weight = 0.02;
  double retreat_continuity_weight = 0.2;
  double joint_delta_weight = 0.6;
  double tip_position_delta_weight = 2.0;
  double tip_orientation_delta_weight = 0.05;
};

class ExtractCandidateScorer
{
public:
  explicit ExtractCandidateScorer(ExtractCandidateScorerConfig config);

  double armJointDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  double tipPositionDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  double tipOrientationDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  double score(
    const std::string& side,
    const ExtractCandidate& candidate,
    const moveit::core::RobotState& current_state,
    double last_retreat_x) const;

private:
  const moveit::core::JointModelGroup* groupForSide(const std::string& side) const;
  const std::string& tipForSide(const std::string& side) const;

  ExtractCandidateScorerConfig config_;
};

}  // namespace alfa_robot::motion
