#pragma once

#include "alfa_robot_moveit_config/extract_planner_types.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <mutex>
#include <string>

namespace moveit::core
{
class JointModelGroup;
}

namespace alfa_robot::motion
{

struct ExtractCandidateSolverConfig
{
  moveit::core::RobotModelConstPtr robot_model;
  const moveit::core::JointModelGroup* joint_group = nullptr;
  const moveit::core::JointModelGroup* left_arm_group = nullptr;
  const moveit::core::JointModelGroup* right_arm_group = nullptr;
  std::string left_tip = "left_v5_tool0";
  std::string right_tip = "right_v5_tool0";
  bool use_independent_kdl = false;
  double kdl_timeout = 0.01;
  double position_tolerance = 0.01;
  double orientation_tolerance = 0.05;
  double max_tip_z_drop = 0.002;
  double min_tool_normal_z = -1e-4;
  double max_joint_delta = 0.0;
  int independent_kdl_max_iterations = 120;
  double independent_kdl_eps = 1e-5;
  int independent_kdl_seed_attempts = 1;
  double independent_kdl_seed_jitter = 8.0 * 3.14159265358979323846 / 180.0;
};

struct ExtractCandidateSolveRequest
{
  std::string side = "left";
  const moveit::core::RobotState* current_state = nullptr;
  geometry_msgs::msg::Pose target_pose;
  size_t step_index = 0;
  size_t candidate_index = 0;
  double retreat_x = 0.0;
  double retreat_delta_x = 0.0;
  double lift_z = 0.0;
  double lift_delta_z = 0.0;
  double pitch_up_rad = 0.0;
  double pitch_delta_rad = 0.0;
  double min_allowed_tip_z = 0.0;
  double fixed_updown = 0.0;
};

class ExtractCandidateSolver
{
public:
  explicit ExtractCandidateSolver(ExtractCandidateSolverConfig config);
  ~ExtractCandidateSolver();

  bool initialize(std::string* error = nullptr);

  bool solve(const ExtractCandidateSolveRequest& request, ExtractCandidate* out) const;

private:
  struct ArmKdlChain;

  bool initArmKdlChain(const std::string& side, ArmKdlChain* out, std::string* error) const;
  bool solveIndependentKdl(
    const std::string& side,
    const ArmKdlChain& chain,
    const moveit::core::RobotState& current_state,
    const Eigen::Isometry3d& target_world,
    double fixed_updown,
    moveit::core::RobotState& state) const;

  const moveit::core::JointModelGroup* groupForSide(const std::string& side) const;
  const std::string& tipForSide(const std::string& side) const;
  double armJointDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  ExtractCandidateSolverConfig config_;
  std::unique_ptr<ArmKdlChain> left_kdl_chain_;
  std::unique_ptr<ArmKdlChain> right_kdl_chain_;
  mutable std::mutex moveit_kdl_mutex_;
};

}  // namespace alfa_robot::motion
