#pragma once

#include "ik_benchmark/parallel_updown_aware_ik_solver.h"
#include "alfa_robot_moveit_config/extract_planner_types.hpp"
#include "alfa_robot_moveit_config/ik_candidate_selector.hpp"
#include "alfa_robot_moveit_config/loaded_pose_planner.hpp"
#include "alfa_robot_moveit_config/motion_core/scene_geometry.hpp"

#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct ExtractBenchmarkRunnerConfig
{
  IkCandidateSelector* candidate_selector = nullptr;
  LoadedPosePlanner* loaded_pose_planner = nullptr;
  rclcpp::Logger logger = rclcpp::get_logger("extract_benchmark_runner");
  bool record_tip_error_ik_candidates = false;
  bool record_rollouts = false;
  bool dual_async = false;
  bool plan_loaded_after_success = false;
  size_t candidate_limit = 0;
  size_t extract_workers = 1;
  double dedup_joint_threshold_rad = 0.0;
  double dedup_h_threshold = 0.0;
  std::string csv_path;
  LoadedPoseBatchPlanOptions loaded_options;
};

struct ExtractBenchmarkRunnerCallbacks
{
  std::function<moveit::core::RobotState(
    const moveit::core::RobotState&,
    const ik_benchmark::UpdownAwareIkCandidate&)> state_from_candidate;
  std::function<ExtractRolloutTiming(
    const moveit::core::RobotState&,
    const AttachedBoxSpec&,
    int,
    size_t,
    const ik_benchmark::UpdownAwareIkCandidate&,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>&)> rollout_left;
  std::function<ExtractRolloutTiming(
    const moveit::core::RobotState&,
    const AttachedBoxSpec&,
    int,
    const AttachedBoxSpec&,
    int,
    size_t,
    const ik_benchmark::UpdownAwareIkCandidate&,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>&)> rollout_dual;
  std::function<void(ExtractRolloutTiming&)> fill_loaded_metrics;
  std::function<void(
    const std::string&,
    const moveit::core::RobotState&,
    const std::vector<AttachedBoxSpec>&,
    const nlohmann::json&)> record_keyframe;
  std::function<void(const std::string&, const moveit::core::RobotState&, const ik_benchmark::UpdownAwareIkResult&, const AttachedBoxSpec&)> record_tip_errors;
  std::function<void(const nlohmann::json&)> record_summary;
  std::function<nlohmann::json(const ik_benchmark::UpdownAwareIkResult&)> rejection_counts_json;
  std::function<bool(const std::string&)> fail;
  std::function<void(const std::string&)> set_last_error;
};

class ExtractBenchmarkRunner
{
public:
  ExtractBenchmarkRunner(
    ExtractBenchmarkRunnerConfig config,
    ExtractBenchmarkRunnerCallbacks callbacks);

  bool runLeft(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box,
    int left_box_id);

  bool runDual(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id);

private:
  std::vector<ik_benchmark::UpdownAwareIkCandidate> sortedLegalCandidates(
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    size_t* original_legal_count,
    IkCandidateSelectionStats* dedup_stats) const;

  bool writeCsv(const std::vector<ExtractRolloutTiming>& timings) const;

  ExtractRolloutTiming runDualCandidate(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const std::vector<ik_benchmark::UpdownAwareIkCandidate>& legal_candidates,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    size_t index,
    bool record_rollout) const;

  ExtractBenchmarkRunnerConfig config_;
  ExtractBenchmarkRunnerCallbacks callbacks_;
};

}  // namespace alfa_robot::motion
