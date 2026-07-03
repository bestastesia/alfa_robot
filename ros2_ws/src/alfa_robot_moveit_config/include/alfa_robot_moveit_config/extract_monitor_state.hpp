#pragma once

#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"
#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"
#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <moveit/robot_state/robot_state.h>

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct ExtractMonitorArmSeed
{
  std::vector<double> left_arm;
  std::vector<double> right_arm;
  double updown = 0.0;
};

struct ExtractMonitorInitialStateRequest
{
  int left_box_id = 0;
  int right_box_id = 0;
  AttachedBoxSpec left_box;
  AttachedBoxSpec right_box;
  moveit::core::RobotModelConstPtr robot_model;
  const moveit::core::JointModelGroup* joint_group = nullptr;
  ExtractMonitorArmSeed grasp_seed;
  ExtractMonitorArmSeed loaded_seed;
};

enum class ExtractMonitorPhase
{
  ReadyForIk,
  ReadyForExtract,
  ReadyForLoaded,
  ReadyForFinal,
  Done,
};

enum class ExtractMonitorStage
{
  Ik,
  Extract,
  Loaded,
  Final,
};

struct ExtractMonitorState
{
  int left_box_id = 0;
  int right_box_id = 0;
  std::string prefix;
  AttachedBoxSpec left_box;
  AttachedBoxSpec right_box;
  moveit::core::RobotStatePtr seed_state;
  moveit::core::RobotStatePtr loaded_start_state;
  ik_benchmark::UpdownAwareIkResult ik_result;
  std::vector<ik_benchmark::UpdownAwareIkCandidate> legal_candidates;
  std::vector<moveit::core::RobotStatePtr> candidate_states;
  std::vector<ExtractRolloutTiming> timings;
  double loaded_plan_batch_wall_ms = 0.0;
  size_t loaded_plan_candidate_count = 0;
  size_t loaded_plan_attempted_count = 0;
  size_t loaded_plan_success_count = 0;
  size_t loaded_parallel_workers = 0;
  size_t loaded_candidate_limit = 0;
  std::map<std::string, size_t> loaded_plan_failure_counts;
};

std::string extract_monitor_prefix(int left_box_id, int right_box_id);

ExtractMonitorState make_extract_monitor_initial_state(
  int left_box_id,
  int right_box_id,
  AttachedBoxSpec left_box,
  AttachedBoxSpec right_box,
  moveit::core::RobotStatePtr seed_state,
  moveit::core::RobotStatePtr loaded_start_state);

ExtractMonitorState make_extract_monitor_initial_state(
  const ExtractMonitorInitialStateRequest& request);

moveit::core::RobotState make_extract_monitor_joint_state(
  const moveit::core::RobotModelConstPtr& robot_model,
  const moveit::core::JointModelGroup* joint_group,
  const ExtractMonitorArmSeed& seed);

using ExtractMonitorCandidateStateBuilder =
  std::function<moveit::core::RobotStatePtr(const ik_benchmark::UpdownAwareIkCandidate&)>;

void populate_extract_monitor_candidate_states(
  ExtractMonitorState& state,
  const ExtractMonitorCandidateStateBuilder& state_builder);

using ExtractMonitorCandidateTask =
  std::function<ExtractRolloutTiming(size_t, const ik_benchmark::UpdownAwareIkCandidate&)>;

size_t extract_monitor_worker_count(size_t candidate_count, size_t requested_worker_count);

size_t run_extract_monitor_candidate_tasks(
  ExtractMonitorState& state,
  size_t requested_worker_count,
  const ExtractMonitorCandidateTask& task);

const ik_benchmark::UpdownAwareIkCandidate* extract_monitor_candidate_for_timing(
  const ExtractMonitorState& state,
  const ExtractRolloutTiming& timing);

moveit::core::RobotStatePtr extract_monitor_candidate_state_for_timing(
  const ExtractMonitorState& state,
  const ExtractRolloutTiming& timing);

struct ExtractMonitorTimingSummary
{
  size_t success_count = 0;
  std::map<std::string, size_t> failure_counts;
  std::vector<size_t> success_indices;
};

ExtractMonitorTimingSummary summarize_extract_monitor_timings(
  const std::vector<ExtractRolloutTiming>& timings);

struct ExtractMonitorLoadedPlanSummary
{
  size_t attempted_count = 0;
  size_t success_count = 0;
  std::map<std::string, size_t> failure_counts;
  std::vector<size_t> attempted_indices;
  std::vector<size_t> success_indices;
};

ExtractMonitorLoadedPlanSummary summarize_loaded_plan_timings(
  const std::vector<ExtractRolloutTiming>& timings,
  const std::vector<size_t>& plan_indices);

std::string extract_monitor_ik_stage_message(
  size_t unique_count,
  size_t legal_count,
  size_t trial_count,
  double elapsed_ms,
  const std::string& snapshot_path);

struct ExtractMonitorIkStageMessageRequest
{
  size_t unique_count = 0;
  size_t legal_count = 0;
  size_t trial_count = 0;
  double elapsed_ms = 0.0;
  std::string snapshot_path;
};

std::string extract_monitor_ik_stage_message(
  const ExtractMonitorIkStageMessageRequest& request);

std::string extract_monitor_extract_stage_message(
  size_t success_count,
  size_t total_count,
  size_t worker_count,
  double elapsed_ms,
  const std::string& snapshot_path);

struct ExtractMonitorExtractStageMessageRequest
{
  size_t success_count = 0;
  size_t total_count = 0;
  size_t worker_count = 0;
  double elapsed_ms = 0.0;
  std::string snapshot_path;
};

std::string extract_monitor_extract_stage_message(
  const ExtractMonitorExtractStageMessageRequest& request);

std::string extract_monitor_loaded_stage_message(
  size_t success_count,
  size_t attempted_count,
  size_t candidate_count,
  double elapsed_ms,
  const std::string& snapshot_path);

struct ExtractMonitorLoadedStageMessageRequest
{
  size_t success_count = 0;
  size_t attempted_count = 0;
  size_t candidate_count = 0;
  double elapsed_ms = 0.0;
  std::string snapshot_path;
};

std::string extract_monitor_loaded_stage_message(
  const ExtractMonitorLoadedStageMessageRequest& request);

std::string extract_monitor_final_stage_message(
  const ExtractRolloutTiming& selected,
  double elapsed_ms,
  const std::string& snapshot_path);

struct ExtractMonitorFinalStageMessageRequest
{
  const ExtractRolloutTiming* selected = nullptr;
  double elapsed_ms = 0.0;
  std::string snapshot_path;
};

std::string extract_monitor_final_stage_message(
  const ExtractMonitorFinalStageMessageRequest& request);

using ExtractMonitorTimingPredicate = std::function<bool(const ExtractRolloutTiming&)>;

ExtractRolloutTiming* select_extract_monitor_final_timing(
  std::vector<ExtractRolloutTiming>& timings,
  const ExtractMonitorTimingPredicate& preferred_predicate);

using ExtractMonitorStageRunner = std::function<bool(std::string*)>;

struct ExtractMonitorStageCallbacks
{
  ExtractMonitorStageRunner ik;
  ExtractMonitorStageRunner extract;
  ExtractMonitorStageRunner loaded;
  ExtractMonitorStageRunner final;
};

struct ExtractMonitorFullRunResult
{
  bool success = false;
  std::string message;
  std::array<double, 4> stage_elapsed_ms{0.0, 0.0, 0.0, 0.0};
  double total_elapsed_ms = 0.0;
};

const char* extract_monitor_stage_failure_label(ExtractMonitorStage stage);

ExtractMonitorStage extract_monitor_stage_for_phase(ExtractMonitorPhase phase);

ExtractMonitorPhase extract_monitor_phase_before_running(ExtractMonitorPhase phase);

ExtractMonitorPhase extract_monitor_next_phase_after(ExtractMonitorStage stage);

bool run_extract_monitor_stage(
  ExtractMonitorStage stage,
  const ExtractMonitorStageCallbacks& callbacks,
  const std::function<double()>& last_stage_ms,
  double* elapsed_ms,
  std::string* message);

ExtractMonitorFullRunResult run_extract_monitor_full_sequence(
  const ExtractMonitorStageCallbacks& callbacks,
  const std::function<double()>& last_stage_ms);

class ExtractMonitorController
{
public:
  ExtractMonitorPhase phase() const { return phase_; }

  void reset() { phase_ = ExtractMonitorPhase::ReadyForIk; }

  bool runNext(
    const ExtractMonitorStageCallbacks& callbacks,
    const std::function<double()>& last_stage_ms,
    std::string* message);

  ExtractMonitorFullRunResult runFull(
    const ExtractMonitorStageCallbacks& callbacks,
    const std::function<double()>& last_stage_ms);

private:
  ExtractMonitorPhase phase_ = ExtractMonitorPhase::ReadyForIk;
};

}  // namespace alfa_robot::motion
