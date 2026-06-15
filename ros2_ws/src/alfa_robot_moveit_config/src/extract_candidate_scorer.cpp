#include "alfa_robot_moveit_config/extract_candidate_scorer.hpp"
#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <moveit/robot_model/joint_model_group.h>

#include <cmath>
#include <limits>

namespace alfa_robot::motion
{

ExtractCandidateScorer::ExtractCandidateScorer(ExtractCandidateScorerConfig config)
: config_(std::move(config))
{}

const moveit::core::JointModelGroup* ExtractCandidateScorer::groupForSide(const std::string& side) const
{
  return side == "left" ? config_.left_arm_group : config_.right_arm_group;
}

const std::string& ExtractCandidateScorer::tipForSide(const std::string& side) const
{
  return side == "left" ? config_.left_tip : config_.right_tip;
}

double ExtractCandidateScorer::armJointDelta(
  const std::string& side,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to) const
{
  const auto* group = groupForSide(side);
  if (!group) return 0.0;

  double sum = 0.0;
  const auto& names = group->getVariableNames();
  for (const auto& name : names) {
    const double delta = to.getVariablePosition(name) - from.getVariablePosition(name);
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

double ExtractCandidateScorer::tipPositionDelta(
  const std::string& side,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to) const
{
  const auto& tip = tipForSide(side);
  const auto& from_tf = from.getGlobalLinkTransform(tip);
  const auto& to_tf = to.getGlobalLinkTransform(tip);
  return (to_tf.translation() - from_tf.translation()).norm();
}

double ExtractCandidateScorer::tipOrientationDelta(
  const std::string& side,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to) const
{
  const auto& tip = tipForSide(side);
  const auto& from_tf = from.getGlobalLinkTransform(tip);
  const auto& to_tf = to.getGlobalLinkTransform(tip);
  return pose_orientation_error(from_tf, to_tf);
}

double ExtractCandidateScorer::score(
  const std::string& side,
  const ExtractCandidate& candidate,
  const moveit::core::RobotState& current_state,
  double last_retreat_x) const
{
  if (!candidate.state_valid || !candidate.state) {
    return std::numeric_limits<double>::infinity();
  }

  const double retreat_continuity =
    std::abs((candidate.retreat_x - last_retreat_x) - config_.step_x);
  const double joint_delta = armJointDelta(side, current_state, *candidate.state);
  const double tip_position_delta = tipPositionDelta(side, current_state, *candidate.state);
  const double tip_orientation_delta = tipOrientationDelta(side, current_state, *candidate.state);

  return config_.lift_weight * candidate.lift_z +
         config_.pitch_weight * std::abs(candidate.pitch_up_rad) +
         config_.retreat_continuity_weight * retreat_continuity +
         config_.joint_delta_weight * joint_delta +
         config_.tip_position_delta_weight * tip_position_delta +
         config_.tip_orientation_delta_weight * tip_orientation_delta;
}

}  // namespace alfa_robot::motion
