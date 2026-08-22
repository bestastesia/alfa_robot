#pragma once

#include "alfa_robot_analytic_ik/analytic_ik.hpp"

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <array>
#include <cstddef>
#include <memory>
#include <string>

namespace ik_benchmark::prototype
{

struct Realtime6dPoseConfig
{
  alfa_robot::analytic_ik::ArmSide side = alfa_robot::analytic_ik::ArmSide::Left;
  std::string group_name = "left_arm";
  std::string tool_link = "left_tool0";
  double fixed_updown = 0.3;
  double jump_threshold_rad = 10.0 * 3.14159265358979323846 / 180.0;
  double collision_edge_step_rad = 2.0 * 3.14159265358979323846 / 180.0;
  double position_tolerance = 1e-4;
  double orientation_tolerance = 1e-4;
};

struct Realtime6dPoseTiming
{
  double ik_ms = 0.0;
  double jump_ms = 0.0;
  double collision_ms = 0.0;
  double total_ms = 0.0;
};

struct Realtime6dPoseResult
{
  bool accepted = false;
  std::string status = "not_run";
  std::array<double, 6> joints{};
  Eigen::Isometry3d actual_pose_in_base = Eigen::Isometry3d::Identity();
  double position_error_m = 0.0;
  double orientation_error_rad = 0.0;
  double max_joint_delta_rad = 0.0;
  std::size_t analytic_solution_count = 0;
  std::size_t jump_rejections = 0;
  std::size_t bounds_rejections = 0;
  std::size_t collision_rejections = 0;
  std::size_t collision_state_checks = 0;
  Realtime6dPoseTiming timing;
};

// PROTOTYPE: pure long-lived solve/filter/check pipeline. No ROS I/O lives here.
class Realtime6dPosePipeline
{
public:
  Realtime6dPosePipeline(
    moveit::core::RobotModelPtr robot_model,
    Realtime6dPoseConfig config,
    const std::array<double, 6>& initial_arm_joints,
    const std::array<double, 6>& initial_other_arm_joints);

  Realtime6dPoseResult process(const Eigen::Isometry3d& target_in_base_link);

  void synchronizeState(
    const std::array<double, 6>& arm_joints,
    const std::array<double, 6>& other_arm_joints,
    double turn,
    double updown);

  Eigen::Isometry3d currentToolPoseInBase() const;
  const moveit::core::RobotState& currentState() const { return current_state_; }
  const Realtime6dPoseConfig& config() const { return config_; }

private:
  bool collisionFree(
    const moveit::core::RobotState& state,
    Realtime6dPoseResult& result) const;
  bool collisionFreeEdge(
    const std::array<double, 6>& start,
    const std::array<double, 6>& goal,
    Realtime6dPoseResult& result) const;
  std::array<double, 6> currentArmJoints() const;
  void setArmJoints(moveit::core::RobotState& state, const std::array<double, 6>& joints) const;

  moveit::core::RobotModelPtr robot_model_;
  Realtime6dPoseConfig config_;
  const moveit::core::JointModelGroup* joint_model_group_ = nullptr;
  planning_scene::PlanningScenePtr planning_scene_;
  moveit::core::RobotState current_state_;
  alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk analytic_solver_;
};

}  // namespace ik_benchmark::prototype
