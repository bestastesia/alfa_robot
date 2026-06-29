#pragma once

#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

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
};

struct OptimizedDualIkSolveResult
{
  bool success = false;
  std::string failure_reason;
  ik_benchmark::UpdownAwareIkResult ik_result;
  moveit::core::RobotStatePtr goal_state;
  nlohmann::json extra;
};

struct OptimizedDualIkSolverConfig
{
  ik_benchmark::ParallelUpdownAwareIkSolver* solver = nullptr;
  const moveit::core::RobotModel* robot_model = nullptr;
  const moveit::core::JointModelGroup* enforce_bounds_group = nullptr;
  double fallback_updown = 0.0;
};

nlohmann::json ik_candidate_rejection_counts_json(
  const ik_benchmark::UpdownAwareIkResult& result);

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
    const ik_benchmark::UpdownAwareIkResult& result,
    const std::string& grasp_mode) const;

private:
  bool isRobotVariable(const std::string& name) const;

  OptimizedDualIkSolverConfig config_;
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
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
  const moveit::core::JointModelGroup* enforce_bounds_group = nullptr);

class IkCandidateSelector
{
public:
  explicit IkCandidateSelector(IkCandidateSelectorConfig config);

  const IkCandidateSelectorConfig& config() const { return config_; }

  std::vector<ik_benchmark::UpdownAwareIkCandidate> select(
    const std::vector<ik_benchmark::UpdownAwareIkCandidate>& sorted_legal_candidates,
    IkCandidateSelectionStats* stats = nullptr) const;

  std::vector<ik_benchmark::UpdownAwareIkCandidate> selectLegalFromResult(
    const ik_benchmark::UpdownAwareIkResult& result,
    IkCandidateSelectionStats* stats = nullptr) const;

private:
  bool similar(
    const ik_benchmark::UpdownAwareIkCandidate& candidate,
    const ik_benchmark::UpdownAwareIkCandidate& kept) const;

  IkCandidateSelectorConfig config_;
};

}  // namespace alfa_robot::motion
