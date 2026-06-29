#pragma once

#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"
#include "alfa_robot_moveit_config/optimized_ik_pipeline.hpp"
#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"
#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>

#include <array>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

nlohmann::json attached_boxes_json(const std::vector<AttachedBoxSpec>& specs);

nlohmann::json container_panels_json(const std::vector<ContainerPanel>& panels);

nlohmann::json static_box_obstacles_json(
  bool enabled,
  int opening_left_box_id,
  int opening_right_box_id,
  double inset,
  const std::vector<StaticBoxObstacle>& boxes);

nlohmann::json container_obstacle_json(
  bool enabled,
  const std::string& frame,
  double length,
  double width,
  double height,
  double center_x,
  double center_y,
  double nominal_center_y,
  double scene_y_shift,
  double floor_z,
  double wall_thickness,
  const std::vector<ContainerPanel>& panels);

nlohmann::json attached_box_config_json(
  bool enabled,
  double depth,
  double width,
  double height);

nlohmann::json robot_state_json(const moveit::core::RobotState& state);

nlohmann::json trajectory_json(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

nlohmann::json extract_monitor_stage_json(
  const std::string& stage_name,
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& attached_boxes,
  const nlohmann::json& static_box_obstacles,
  const nlohmann::json& extra);

nlohmann::json extract_monitor_candidate_json(
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
  size_t display_index,
  const moveit::core::RobotState& state);

nlohmann::json extract_monitor_candidate_records_json(
  const std::vector<ik_benchmark::UpdownAwareIkCandidate>& candidates,
  const std::vector<moveit::core::RobotStatePtr>& candidate_states);

nlohmann::json extract_monitor_timing_json(
  const ExtractRolloutTiming& timing,
  size_t display_index,
  const moveit::core::RobotState& state,
  const std::string& prefix,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles);

using ExtractMonitorTimingRecordState =
  std::function<moveit::core::RobotStatePtr(const ExtractRolloutTiming& timing)>;

nlohmann::json extract_monitor_timing_records_json(
  const std::vector<ExtractRolloutTiming>& timings,
  const std::vector<size_t>& indices,
  const std::string& prefix,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles,
  const ExtractMonitorTimingRecordState& record_state);

struct ExtractMonitorTimingRecordsRequest
{
  const std::vector<ExtractRolloutTiming>* timings = nullptr;
  std::vector<size_t> indices;
  std::string prefix;
  int left_box_id = 0;
  int right_box_id = 0;
  std::vector<std::string> target_names;
  std::vector<AttachedBoxSpec> carried_boxes;
  nlohmann::json static_box_obstacles = nlohmann::json::object();
  ExtractMonitorTimingRecordState record_state;
};

nlohmann::json extract_monitor_timing_records_json(
  const ExtractMonitorTimingRecordsRequest& request);

nlohmann::json extract_monitor_replay_context_json(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id);

nlohmann::json extract_monitor_pre_attach_replay_extra(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  bool valid,
  const std::string& method,
  double transition_ms,
  const std::string& failure_reason);

nlohmann::json extract_monitor_selected_lateral_shift_replay_extra(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  const nlohmann::json& shift_extra);

nlohmann::json extract_monitor_selected_loaded_plan_replay_extra(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id);

nlohmann::json extract_monitor_selected_lateral_shift_replay_stages(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles);

nlohmann::json extract_monitor_selected_loaded_plan_replay_stage(
  const std::string& prefix,
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles);

nlohmann::json extract_monitor_selected_extract_replay_stage(
  const std::string& prefix,
  size_t step,
  size_t candidate_order,
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const moveit::core::RobotState& state,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles,
  const nlohmann::json& extra);

struct ExtractMonitorSelectedExtractReplayStateRequest
{
  std::string prefix;
  size_t step = 0;
  size_t candidate_order = 0;
  const moveit::core::RobotState* state = nullptr;
  int left_box_id = 0;
  int right_box_id = 0;
  std::vector<std::string> target_names;
  std::vector<AttachedBoxSpec> carried_boxes;
  nlohmann::json static_box_obstacles = nlohmann::json::object();
  nlohmann::json extra = nlohmann::json::object();
  double time_from_start_sec = 0.0;
};

nlohmann::json extract_monitor_selected_extract_replay_state_stage(
  const std::string& prefix,
  size_t step,
  size_t candidate_order,
  const moveit::core::RobotState& state,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles,
  const nlohmann::json& extra,
  double time_from_start_sec);

nlohmann::json extract_monitor_selected_extract_replay_state_stage(
  const ExtractMonitorSelectedExtractReplayStateRequest& request);

nlohmann::json failure_counts_json(const std::map<std::string, size_t>& failure_counts);

nlohmann::json extract_monitor_snapshot_base(
  const std::string& phase,
  const std::string& phase_label,
  double elapsed_ms,
  int left_box_id,
  int right_box_id,
  double box_front_x,
  double scene_y_shift);

nlohmann::json extract_monitor_ik_snapshot(
  const std::string& snapshot_path,
  double elapsed_ms,
  int left_box_id,
  int right_box_id,
  double box_front_x,
  double scene_y_shift,
  const ik_benchmark::UpdownAwareIkResult& ik_result,
  const IkCandidateSelectionStats& dedup_stats,
  const nlohmann::json& rejection_counts,
  const nlohmann::json& records);

struct ExtractMonitorIkSnapshotRequest
{
  std::string snapshot_path;
  double elapsed_ms = 0.0;
  int left_box_id = 0;
  int right_box_id = 0;
  double box_front_x = 0.0;
  double scene_y_shift = 0.0;
  const ik_benchmark::UpdownAwareIkResult* ik_result = nullptr;
  IkCandidateSelectionStats dedup_stats;
  nlohmann::json rejection_counts = nlohmann::json::object();
  nlohmann::json records = nlohmann::json::array();
};

nlohmann::json extract_monitor_ik_snapshot(
  const ExtractMonitorIkSnapshotRequest& request);

nlohmann::json extract_monitor_extract_snapshot(
  double elapsed_ms,
  int left_box_id,
  int right_box_id,
  double box_front_x,
  double scene_y_shift,
  size_t input_candidate_count,
  size_t success_count,
  size_t worker_count,
  const std::map<std::string, size_t>& failure_counts,
  const nlohmann::json& records);

struct ExtractMonitorExtractSnapshotRequest
{
  double elapsed_ms = 0.0;
  int left_box_id = 0;
  int right_box_id = 0;
  double box_front_x = 0.0;
  double scene_y_shift = 0.0;
  size_t input_candidate_count = 0;
  size_t success_count = 0;
  size_t worker_count = 0;
  std::map<std::string, size_t> failure_counts;
  nlohmann::json records = nlohmann::json::array();
};

nlohmann::json extract_monitor_extract_snapshot(
  const ExtractMonitorExtractSnapshotRequest& request);

nlohmann::json extract_monitor_loaded_snapshot(
  double elapsed_ms,
  int left_box_id,
  int right_box_id,
  double box_front_x,
  double scene_y_shift,
  size_t extract_success_count,
  size_t attempted_count,
  size_t success_count,
  double loaded_plan_batch_wall_ms,
  size_t loaded_parallel_workers,
  size_t loaded_candidate_limit,
  const std::map<std::string, size_t>& failure_counts,
  const nlohmann::json& records);

struct ExtractMonitorLoadedSnapshotRequest
{
  double elapsed_ms = 0.0;
  int left_box_id = 0;
  int right_box_id = 0;
  double box_front_x = 0.0;
  double scene_y_shift = 0.0;
  size_t extract_success_count = 0;
  size_t attempted_count = 0;
  size_t success_count = 0;
  double loaded_plan_batch_wall_ms = 0.0;
  size_t loaded_parallel_workers = 0;
  size_t loaded_candidate_limit = 0;
  std::map<std::string, size_t> failure_counts;
  nlohmann::json records = nlohmann::json::array();
};

nlohmann::json extract_monitor_loaded_snapshot(
  const ExtractMonitorLoadedSnapshotRequest& request);

nlohmann::json extract_monitor_final_snapshot(
  double elapsed_ms,
  int left_box_id,
  int right_box_id,
  double box_front_x,
  double scene_y_shift,
  const nlohmann::json& record,
  const nlohmann::json& replay_stages);

struct ExtractMonitorFinalSnapshotRequest
{
  double elapsed_ms = 0.0;
  int left_box_id = 0;
  int right_box_id = 0;
  double box_front_x = 0.0;
  double scene_y_shift = 0.0;
  nlohmann::json record = nlohmann::json::object();
  nlohmann::json replay_stages = nlohmann::json::array();
};

nlohmann::json extract_monitor_final_snapshot(
  const ExtractMonitorFinalSnapshotRequest& request);

nlohmann::json extract_monitor_full_selected_snapshot(
  nlohmann::json snapshot,
  double box_front_x,
  double scene_y_shift,
  double total_elapsed_ms,
  const std::array<double, 4>& stage_elapsed_ms);

}  // namespace alfa_robot::motion
