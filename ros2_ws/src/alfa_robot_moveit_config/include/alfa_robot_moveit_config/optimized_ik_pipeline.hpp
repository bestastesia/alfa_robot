#pragma once

#include "alfa_robot_analytic_ik/analytic_ik.hpp"
#include "robot_motion_core/ik_candidate_types.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct OptimizedDualIkSolveRequest
{
  std::string stage_name;
  geometry_msgs::msg::Pose left_pose;
  geometry_msgs::msg::Pose right_pose;
  bool top_suction = false;
  const moveit::core::RobotState* seed_state = nullptr;
  bool left_top_suction = false;
  bool right_top_suction = false;
  bool capture_pre_score_candidates = false;
  bool allow_front_tool_roll_pi_symmetry = false;
};

struct OptimizedDualIkSolveResult
{
  bool success = false;
  std::string failure_reason;
  robot_motion::core::UpdownAwareIkResult ik_result;
  moveit::core::RobotStatePtr goal_state;
  nlohmann::json extra;
};

struct OptimizedDualIkSolverConfig
{
  const robot_motion::core::UpdownAwareIkConfig* ik_config = nullptr;
  const moveit::core::RobotModel* robot_model = nullptr;
  const moveit::core::JointModelGroup* enforce_bounds_group = nullptr;
  double fallback_updown = 0.0;
  size_t analytic_root_samples = 360;
};

nlohmann::json ik_candidate_rejection_counts_json(
  const robot_motion::core::UpdownAwareIkResult& result);

double joint_limit_margin_cost(
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const moveit::core::RobotModel& robot_model,
  const std::vector<double>& joint_weights,
  double free_ratio);

double positive_joint_angle_penalty(double joint_angle_rad, double weight);

class OptimizedDualIkSolver
{
public:
  explicit OptimizedDualIkSolver(OptimizedDualIkSolverConfig config);

  const OptimizedDualIkSolverConfig& config() const { return config_; }

  bool ready() const;

  OptimizedDualIkSolveResult solve(
    const OptimizedDualIkSolveRequest& request,
    const std::string& stage_kind) const;

  std::vector<double> stateValues(
    const moveit::core::RobotState& state,
    const std::vector<std::string>& names) const;

  double currentUpdown(const moveit::core::RobotState& state) const;

  nlohmann::json resultJson(
    const robot_motion::core::UpdownAwareIkResult& result,
    const std::string& grasp_mode) const;

private:
  bool isRobotVariable(const std::string& name) const;

  OptimizedDualIkSolverConfig config_;
  alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk analytic_solver_;
};

struct IkCandidateSelectorConfig
{
  bool dedup_enabled = false;
  double joint_threshold = 1.0 * 3.14159265358979323846 / 180.0;
  double h_threshold = 0.005;
  size_t candidate_limit = 0;
};

struct IkCandidateSelectionStats
{
  bool enabled = false;
  size_t input_count = 0;
  size_t unique_count = 0;
  size_t selected_count = 0;
  size_t removed_count = 0;
  double elapsed_ms = 0.0;
};

moveit::core::RobotState robot_state_from_ik_candidate(
  const moveit::core::RobotState& seed_state,
  const robot_motion::core::UpdownAwareIkCandidate& candidate,
  const moveit::core::JointModelGroup* enforce_bounds_group = nullptr);

class IkCandidateSelector
{
public:
  explicit IkCandidateSelector(IkCandidateSelectorConfig config);

  const IkCandidateSelectorConfig& config() const { return config_; }

  std::vector<robot_motion::core::UpdownAwareIkCandidate> select(
    const std::vector<robot_motion::core::UpdownAwareIkCandidate>& sorted_legal_candidates,
    IkCandidateSelectionStats* stats = nullptr) const;

  std::vector<robot_motion::core::UpdownAwareIkCandidate> selectLegalFromResult(
    const robot_motion::core::UpdownAwareIkResult& result,
    IkCandidateSelectionStats* stats = nullptr) const;

private:
  bool similar(
    const robot_motion::core::UpdownAwareIkCandidate& candidate,
    const robot_motion::core::UpdownAwareIkCandidate& kept) const;

  IkCandidateSelectorConfig config_;
};

}  // namespace alfa_robot::motion
