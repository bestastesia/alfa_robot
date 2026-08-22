#include "realtime_6d_pose_pipeline.hpp"

#include <moveit/collision_detection/collision_common.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ik_benchmark::prototype
{
namespace
{

using Clock = std::chrono::steady_clock;

double elapsed_ms(const Clock::time_point& start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::string joint_name(alfa_robot::analytic_ik::ArmSide side, std::size_t index)
{
  const std::string prefix = side == alfa_robot::analytic_ik::ArmSide::Left ? "left" : "right";
  return prefix + "_joint" + std::to_string(index + 1);
}

alfa_robot::analytic_ik::ArmSide opposite_side(alfa_robot::analytic_ik::ArmSide side)
{
  return side == alfa_robot::analytic_ik::ArmSide::Left ?
    alfa_robot::analytic_ik::ArmSide::Right : alfa_robot::analytic_ik::ArmSide::Left;
}

double absolute_angular_delta(double lhs, double rhs)
{
  return std::abs(alfa_robot::analytic_ik::normalizeAngle(lhs - rhs));
}

bool has_variable(const moveit::core::RobotModel& model, const std::string& name)
{
  const auto& names = model.getVariableNames();
  return std::find(names.begin(), names.end(), name) != names.end();
}

}  // namespace

Realtime6dPosePipeline::Realtime6dPosePipeline(
  moveit::core::RobotModelPtr robot_model,
  Realtime6dPoseConfig config,
  const std::array<double, 6>& initial_arm_joints,
  const std::array<double, 6>& initial_other_arm_joints)
: robot_model_(std::move(robot_model)),
  config_(std::move(config)),
  planning_scene_(std::make_shared<planning_scene::PlanningScene>(robot_model_)),
  current_state_(robot_model_)
{
  if (!robot_model_) {
    throw std::invalid_argument("robot_model must not be null");
  }
  joint_model_group_ = robot_model_->getJointModelGroup(config_.group_name);
  if (!joint_model_group_) {
    throw std::invalid_argument("unknown MoveIt group: " + config_.group_name);
  }
  if (!robot_model_->hasLinkModel(config_.tool_link)) {
    throw std::invalid_argument("unknown tool link: " + config_.tool_link);
  }

  current_state_.setToDefaultValues();
  if (has_variable(*robot_model_, "updown")) {
    current_state_.setVariablePosition("updown", config_.fixed_updown);
  }
  setArmJoints(current_state_, initial_arm_joints);
  for (std::size_t i = 0; i < initial_other_arm_joints.size(); ++i) {
    current_state_.setVariablePosition(
      joint_name(opposite_side(config_.side), i), initial_other_arm_joints[i]);
  }
  current_state_.update();
  current_state_.updateCollisionBodyTransforms();
}

std::array<double, 6> Realtime6dPosePipeline::currentArmJoints() const
{
  std::array<double, 6> joints{};
  for (std::size_t i = 0; i < joints.size(); ++i) {
    joints[i] = current_state_.getVariablePosition(joint_name(config_.side, i));
  }
  return joints;
}

void Realtime6dPosePipeline::setArmJoints(
  moveit::core::RobotState& state,
  const std::array<double, 6>& joints) const
{
  for (std::size_t i = 0; i < joints.size(); ++i) {
    state.setVariablePosition(joint_name(config_.side, i), joints[i]);
  }
  if (has_variable(*robot_model_, "updown")) {
    state.setVariablePosition("updown", config_.fixed_updown);
  }
  state.update();
  state.updateCollisionBodyTransforms();
}

Eigen::Isometry3d Realtime6dPosePipeline::currentToolPoseInBase() const
{
  return current_state_.getGlobalLinkTransform("base_link").inverse() *
         current_state_.getGlobalLinkTransform(config_.tool_link);
}

void Realtime6dPosePipeline::synchronizeState(
  const std::array<double, 6>& arm_joints,
  const std::array<double, 6>& other_arm_joints,
  double turn,
  double updown)
{
  config_.fixed_updown = updown;
  setArmJoints(current_state_, arm_joints);
  for (std::size_t i = 0; i < other_arm_joints.size(); ++i) {
    current_state_.setVariablePosition(
      joint_name(opposite_side(config_.side), i), other_arm_joints[i]);
  }
  if (has_variable(*robot_model_, "turn")) {
    current_state_.setVariablePosition("turn", turn);
  }
  if (has_variable(*robot_model_, "updown")) {
    current_state_.setVariablePosition("updown", updown);
  }
  current_state_.update();
  current_state_.updateCollisionBodyTransforms();
}

bool Realtime6dPosePipeline::collisionFree(
  const moveit::core::RobotState& state,
  Realtime6dPoseResult& result) const
{
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult collision;
  request.group_name = config_.group_name;
  request.contacts = false;
  planning_scene_->checkCollision(
    request, collision, state, planning_scene_->getAllowedCollisionMatrix());
  ++result.collision_state_checks;
  return !collision.collision;
}

bool Realtime6dPosePipeline::collisionFreeEdge(
  const std::array<double, 6>& start,
  const std::array<double, 6>& goal,
  Realtime6dPoseResult& result) const
{
  double max_delta = 0.0;
  for (std::size_t i = 0; i < start.size(); ++i) {
    max_delta = std::max(max_delta, absolute_angular_delta(goal[i], start[i]));
  }
  const double step = std::max(1e-6, config_.collision_edge_step_rad);
  const std::size_t samples = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(max_delta / step)));
  moveit::core::RobotState probe(current_state_);
  for (std::size_t sample = 1; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    std::array<double, 6> interpolated{};
    for (std::size_t i = 0; i < start.size(); ++i) {
      const double delta = alfa_robot::analytic_ik::normalizeAngle(goal[i] - start[i]);
      interpolated[i] = alfa_robot::analytic_ik::normalizeAngle(start[i] + ratio * delta);
    }
    setArmJoints(probe, interpolated);
    if (!collisionFree(probe, result)) {
      return false;
    }
  }
  return true;
}

Realtime6dPoseResult Realtime6dPosePipeline::process(
  const Eigen::Isometry3d& target_in_base_link)
{
  Realtime6dPoseResult result;
  const auto total_start = Clock::now();
  const auto seed = currentArmJoints();

  const auto ik_start = Clock::now();
  const auto solutions = analytic_solver_.solveInBaseLink(
    config_.side,
    target_in_base_link,
    config_.fixed_updown,
    seed,
    config_.position_tolerance,
    config_.orientation_tolerance);
  result.timing.ik_ms = elapsed_ms(ik_start);
  result.analytic_solution_count = solutions.size();
  if (solutions.empty()) {
    result.status = "ik_no_solution";
    result.timing.total_ms = elapsed_ms(total_start);
    return result;
  }

  for (const auto& solution : solutions) {
    const auto jump_start = Clock::now();
    double max_delta = 0.0;
    for (std::size_t i = 0; i < seed.size(); ++i) {
      max_delta = std::max(max_delta, absolute_angular_delta(solution.joints[i], seed[i]));
    }
    result.max_joint_delta_rad = std::max(result.max_joint_delta_rad, max_delta);
    result.timing.jump_ms += elapsed_ms(jump_start);
    if (max_delta > config_.jump_threshold_rad) {
      ++result.jump_rejections;
      continue;
    }

    moveit::core::RobotState candidate(current_state_);
    setArmJoints(candidate, solution.joints);
    if (!candidate.satisfiesBounds(joint_model_group_)) {
      ++result.bounds_rejections;
      continue;
    }

    const auto collision_start = Clock::now();
    const bool edge_free = collisionFreeEdge(seed, solution.joints, result);
    result.timing.collision_ms += elapsed_ms(collision_start);
    if (!edge_free) {
      ++result.collision_rejections;
      continue;
    }

    current_state_ = candidate;
    current_state_.update();
    current_state_.updateCollisionBodyTransforms();
    result.accepted = true;
    result.status = "accepted";
    result.joints = solution.joints;
    result.position_error_m = solution.position_error;
    result.orientation_error_rad = solution.orientation_error;
    result.max_joint_delta_rad = max_delta;
    result.actual_pose_in_base = currentToolPoseInBase();
    result.timing.total_ms = elapsed_ms(total_start);
    return result;
  }

  if (result.collision_rejections > 0) {
    result.status = "collision_rejected";
  } else if (result.bounds_rejections > 0) {
    result.status = "bounds_rejected";
  } else {
    result.status = "joint_jump_rejected";
  }
  result.timing.total_ms = elapsed_ms(total_start);
  return result;
}

}  // namespace ik_benchmark::prototype
