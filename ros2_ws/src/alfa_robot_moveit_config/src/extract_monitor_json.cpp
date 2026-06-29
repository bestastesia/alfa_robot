#include "alfa_robot_moveit_config/extract_monitor_json.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"
#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include <algorithm>

namespace alfa_robot::motion
{

nlohmann::json attached_boxes_json(const std::vector<AttachedBoxSpec>& specs)
{
  nlohmann::json boxes = nlohmann::json::array();
  for (const auto& box : specs) {
    boxes.push_back({
      {"id", box.id},
      {"link_name", box.link_name},
      {"center_in_link", {box.center_in_link[0], box.center_in_link[1], box.center_in_link[2]}},
      {"size", {box.size[0], box.size[1], box.size[2]}},
    });
  }
  return boxes;
}

nlohmann::json container_panels_json(const std::vector<ContainerPanel>& panels)
{
  nlohmann::json out = nlohmann::json::array();
  for (const auto& panel : panels) {
    out.push_back({
      {"id", panel.id},
      {"center", {panel.center[0], panel.center[1], panel.center[2]}},
      {"size", {panel.size[0], panel.size[1], panel.size[2]}},
    });
  }
  return out;
}

nlohmann::json static_box_obstacles_json(
  bool enabled,
  int opening_left_box_id,
  int opening_right_box_id,
  double inset,
  const std::vector<StaticBoxObstacle>& boxes)
{
  nlohmann::json box_json = nlohmann::json::array();
  for (const auto& box : boxes) {
    box_json.push_back({
      {"id", box.id},
      {"center", {box.center[0], box.center[1], box.center[2]}},
      {"size", {box.size[0], box.size[1], box.size[2]}},
    });
  }
  return {
    {"enabled", enabled},
    {"mode", "dynamic_box_wall_with_pair_opening"},
    {"opening_left_box_id", opening_left_box_id},
    {"opening_right_box_id", opening_right_box_id},
    {"inset", inset},
    {"boxes", box_json},
  };
}

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
  const std::vector<ContainerPanel>& panels)
{
  return {
    {"enabled", enabled},
    {"frame", frame},
    {"length", length},
    {"width", width},
    {"height", height},
    {"center_x", center_x},
    {"center_y", center_y},
    {"nominal_center_y", nominal_center_y},
    {"scene_y_shift", scene_y_shift},
    {"floor_z", floor_z},
    {"wall_thickness", wall_thickness},
    {"panels", container_panels_json(panels)},
  };
}

nlohmann::json attached_box_config_json(
  bool enabled,
  double depth,
  double width,
  double height)
{
  return {
    {"enabled", enabled},
    {"depth", depth},
    {"width", width},
    {"height", height},
  };
}

nlohmann::json robot_state_json(const moveit::core::RobotState& state)
{
  const auto& names = state.getRobotModel()->getVariableNames();
  std::vector<double> values;
  values.reserve(names.size());
  for (const auto& name : names) {
    values.push_back(state.getVariablePosition(name));
  }
  return {{"joint_names", names}, {"joint_values", values}, {"joint_map", names_values_json(names, values)}};
}

nlohmann::json trajectory_json(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  const auto& traj = plan.trajectory_.joint_trajectory;
  nlohmann::json points = nlohmann::json::array();
  for (const auto& point : traj.points) {
    points.push_back({
      {"time_from_start_sec", rclcpp::Duration(point.time_from_start).seconds()},
      {"positions", point.positions},
      {"velocities", point.velocities}
    });
  }
  return {
    {"joint_names", traj.joint_names},
    {"point_count", traj.points.size()},
    {"points", points}
  };
}

nlohmann::json extract_monitor_stage_json(
  const std::string& stage_name,
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& attached_boxes,
  const nlohmann::json& static_box_obstacles,
  const nlohmann::json& extra)
{
  return {
    {"type", "stage"},
    {"stage", stage_name},
    {"trajectory", trajectory_json(plan)},
    {"target_names", target_names},
    {"start_state", robot_state_json(start_state)},
    {"goal_state", robot_state_json(goal_state)},
    {"attached_boxes", attached_boxes_json(attached_boxes)},
    {"static_box_obstacles", static_box_obstacles},
    {"extra", extra}
  };
}

nlohmann::json extract_monitor_candidate_json(
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
  size_t display_index,
  const moveit::core::RobotState& state)
{
  return {
    {"display_index", display_index},
    {"h", candidate.h},
    {"h_index", candidate.h_index},
    {"seed_index", candidate.seed_index},
    {"score", candidate.score},
    {"solve_ms", candidate.solve_ms},
    {"solver_path", candidate.solver_path},
    {"target_order", candidate.target_order},
    {"updown_delta", candidate.updown_delta},
    {"joint_delta", candidate.joint_delta},
    {"state", robot_state_json(state)}
  };
}

nlohmann::json extract_monitor_candidate_records_json(
  const std::vector<ik_benchmark::UpdownAwareIkCandidate>& candidates,
  const std::vector<moveit::core::RobotStatePtr>& candidate_states)
{
  nlohmann::json records = nlohmann::json::array();
  const size_t count = std::min(candidates.size(), candidate_states.size());
  for (size_t i = 0; i < count; ++i) {
    if (!candidate_states[i]) {
      continue;
    }
    records.push_back(extract_monitor_candidate_json(candidates[i], records.size(), *candidate_states[i]));
  }
  return records;
}

nlohmann::json extract_monitor_replay_context_json(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id)
{
  return {
    {"candidate_order", timing.candidate_order},
    {"loaded_plan_rank", timing.loaded_plan_rank},
    {"left_box_id", left_box_id},
    {"right_box_id", right_box_id}
  };
}

nlohmann::json extract_monitor_pre_attach_replay_extra(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  bool valid,
  const std::string& method,
  double transition_ms,
  const std::string& failure_reason)
{
  nlohmann::json extra = extract_monitor_replay_context_json(timing, left_box_id, right_box_id);
  extra.update({
    {"stage_kind", "monitor_selected_pre_attach_loaded_to_ik_replay"},
    {"valid", valid},
    {"method", method},
    {"transition_ms", transition_ms},
    {"failure_reason", valid ? "" : failure_reason}
  });
  return extra;
}

nlohmann::json extract_monitor_selected_lateral_shift_replay_extra(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  const nlohmann::json& shift_extra)
{
  nlohmann::json extra = shift_extra;
  extra.update(extract_monitor_replay_context_json(timing, left_box_id, right_box_id));
  extra["loaded_plan_success"] = timing.loaded_plan_success;
  extra["loaded_plan_failure_reason"] = timing.loaded_plan_failure_reason;
  return extra;
}

nlohmann::json extract_monitor_selected_loaded_plan_replay_extra(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id)
{
  nlohmann::json extra = extract_monitor_replay_context_json(timing, left_box_id, right_box_id);
  extra.update({
    {"stage_kind", "monitor_selected_loaded_plan_replay"},
    {"valid", true},
    {"loaded_plan_ms", timing.loaded_plan_ms},
    {"loaded_plan_points", timing.loaded_plan_points},
    {"loaded_plan_trajectory_distance", timing.loaded_plan_trajectory_distance},
    {"moveit_attached_box_count", 2}
  });
  return extra;
}

nlohmann::json extract_monitor_selected_lateral_shift_replay_stages(
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles)
{
  nlohmann::json replay_stages = nlohmann::json::array();
  for (const auto& shift_stage : timing.lateral_shift_replay_stages) {
    if (!shift_stage.start_state || !shift_stage.goal_state) {
      continue;
    }
    replay_stages.push_back(extract_monitor_stage_json(
      shift_stage.stage_name,
      shift_stage.plan,
      *shift_stage.start_state,
      *shift_stage.goal_state,
      target_names,
      carried_boxes,
      static_box_obstacles,
      extract_monitor_selected_lateral_shift_replay_extra(
        timing, left_box_id, right_box_id, shift_stage.extra)));
  }
  return replay_stages;
}

nlohmann::json extract_monitor_selected_loaded_plan_replay_stage(
  const std::string& prefix,
  const ExtractRolloutTiming& timing,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles)
{
  if (!timing.loaded_start_state ||
      !timing.loaded_goal_state ||
      timing.loaded_plan.trajectory_.joint_trajectory.points.empty()) {
    return nlohmann::json();
  }

  return extract_monitor_stage_json(
    prefix + "/selected_loaded_plan",
    timing.loaded_plan,
    *timing.loaded_start_state,
    *timing.loaded_goal_state,
    target_names,
    carried_boxes,
    static_box_obstacles,
    extract_monitor_selected_loaded_plan_replay_extra(timing, left_box_id, right_box_id));
}

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
  const nlohmann::json& extra)
{
  nlohmann::json enriched = extra;
  enriched["stage_kind"] = "monitor_selected_extract_replay";
  enriched["candidate_order"] = candidate_order;
  enriched["left_box_id"] = left_box_id;
  enriched["right_box_id"] = right_box_id;
  return extract_monitor_stage_json(
    prefix + "/selected_extract_step_" + std::to_string(step),
    plan,
    state,
    state,
    target_names,
    carried_boxes,
    static_box_obstacles,
    enriched);
}

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
  double time_from_start_sec)
{
  return extract_monitor_selected_extract_replay_stage(
    prefix,
    step,
    candidate_order,
    single_state_plan(state, target_names, time_from_start_sec),
    state,
    left_box_id,
    right_box_id,
    target_names,
    carried_boxes,
    static_box_obstacles,
    extra);
}

nlohmann::json extract_monitor_selected_extract_replay_state_stage(
  const ExtractMonitorSelectedExtractReplayStateRequest& request)
{
  if (!request.state) {
    return nlohmann::json::object();
  }
  return extract_monitor_selected_extract_replay_state_stage(
    request.prefix,
    request.step,
    request.candidate_order,
    *request.state,
    request.left_box_id,
    request.right_box_id,
    request.target_names,
    request.carried_boxes,
    request.static_box_obstacles,
    request.extra,
    request.time_from_start_sec);
}

nlohmann::json extract_monitor_timing_json(
  const ExtractRolloutTiming& timing,
  size_t display_index,
  const moveit::core::RobotState& state,
  const std::string& prefix,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles)
{
  nlohmann::json replay_stages = nlohmann::json::array();
  for (const auto& stage : timing.rollout_records) {
    replay_stages.push_back(stage);
  }
  for (const auto& shift_stage : timing.lateral_shift_replay_stages) {
    if (!shift_stage.start_state || !shift_stage.goal_state) {
      continue;
    }
    nlohmann::json extra = shift_stage.extra;
    extra.update(extract_monitor_replay_context_json(timing, left_box_id, right_box_id));
    extra["loaded_plan_success"] = timing.loaded_plan_success;
    extra["loaded_plan_failure_reason"] = timing.loaded_plan_failure_reason;
    replay_stages.push_back(extract_monitor_stage_json(
      shift_stage.stage_name,
      shift_stage.plan,
      *shift_stage.start_state,
      *shift_stage.goal_state,
      target_names,
      carried_boxes,
      static_box_obstacles,
      extra));
  }
  if (timing.loaded_start_state && timing.loaded_goal_state &&
      !timing.loaded_plan.trajectory_.joint_trajectory.points.empty()) {
    nlohmann::json extra = extract_monitor_replay_context_json(timing, left_box_id, right_box_id);
    extra.update({
      {"stage_kind", "monitor_loaded_plan_attempt_replay"},
      {"valid", timing.loaded_plan_success},
      {"loaded_plan_success", timing.loaded_plan_success},
      {"loaded_plan_failure_reason", timing.loaded_plan_failure_reason},
      {"loaded_plan_ms", timing.loaded_plan_ms},
      {"loaded_plan_points", timing.loaded_plan_points},
      {"loaded_plan_trajectory_distance", timing.loaded_plan_trajectory_distance}
    });
    replay_stages.push_back(extract_monitor_stage_json(
      prefix + "/candidate_" + std::to_string(timing.candidate_order) + "/loaded_plan_attempt",
      timing.loaded_plan,
      *timing.loaded_start_state,
      *timing.loaded_goal_state,
      target_names,
      carried_boxes,
      static_box_obstacles,
      extra));
  }
  return {
    {"display_index", display_index},
    {"candidate_order", timing.candidate_order},
    {"h", timing.h},
    {"h_index", timing.h_index},
    {"seed_index", timing.seed_index},
    {"ik_score", timing.ik_score},
    {"ik_solve_ms", timing.ik_solve_ms},
    {"rollout_ms", timing.rollout_ms},
    {"interval_ms", timing.interval_ms},
    {"accepted_steps", timing.accepted_steps},
    {"failed_steps", timing.failed_steps},
    {"final_retreat_x", timing.final_retreat_x},
    {"final_lift_z", timing.final_lift_z},
    {"final_pitch_deg", timing.final_pitch_deg},
    {"right_final_retreat_x", timing.right_final_retreat_x},
    {"right_final_lift_z", timing.right_final_lift_z},
    {"right_final_pitch_deg", timing.right_final_pitch_deg},
    {"success", timing.success},
    {"failure_reason", timing.failure_reason},
    {"loaded_plan_attempted", timing.loaded_plan_attempted},
    {"loaded_plan_success", timing.loaded_plan_success},
    {"lateral_shift_attempted", timing.lateral_shift_attempted},
    {"lateral_shift_success", timing.lateral_shift_success},
    {"lateral_shift_ms", timing.lateral_shift_ms},
    {"lateral_shift_reached_distance", timing.lateral_shift_reached_distance},
    {"lateral_shift_points", timing.lateral_shift_points},
    {"loaded_plan_rank", timing.loaded_plan_rank},
    {"loaded_plan_ms", timing.loaded_plan_ms},
    {"loaded_plan_points", timing.loaded_plan_points},
    {"loaded_plan_trajectory_distance", timing.loaded_plan_trajectory_distance},
    {"loaded_plan_failure_reason", timing.loaded_plan_failure_reason},
    {"loaded_pose_distance_sum", timing.loaded_pose_distance_sum},
    {"loaded_pose_distance_l2", timing.loaded_pose_distance_l2},
    {"loaded_pose_max_joint_delta", timing.loaded_pose_max_joint_delta},
    {"loaded_plan_selected", timing.loaded_plan_selected},
    {"state", robot_state_json(state)},
    {"replay_stage_count", replay_stages.size()},
    {"replay_stages", replay_stages}
  };
}

nlohmann::json extract_monitor_timing_records_json(
  const std::vector<ExtractRolloutTiming>& timings,
  const std::vector<size_t>& indices,
  const std::string& prefix,
  int left_box_id,
  int right_box_id,
  const std::vector<std::string>& target_names,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const nlohmann::json& static_box_obstacles,
  const ExtractMonitorTimingRecordState& record_state)
{
  nlohmann::json records = nlohmann::json::array();
  if (!record_state) {
    return records;
  }
  for (const auto index : indices) {
    if (index >= timings.size()) {
      continue;
    }
    const auto state = record_state(timings[index]);
    if (!state) {
      continue;
    }
    records.push_back(extract_monitor_timing_json(
      timings[index],
      records.size(),
      *state,
      prefix,
      left_box_id,
      right_box_id,
      target_names,
      carried_boxes,
      static_box_obstacles));
  }
  return records;
}

nlohmann::json extract_monitor_timing_records_json(
  const ExtractMonitorTimingRecordsRequest& request)
{
  if (!request.timings) {
    return nlohmann::json::array();
  }
  return extract_monitor_timing_records_json(
    *request.timings,
    request.indices,
    request.prefix,
    request.left_box_id,
    request.right_box_id,
    request.target_names,
    request.carried_boxes,
    request.static_box_obstacles,
    request.record_state);
}

nlohmann::json failure_counts_json(const std::map<std::string, size_t>& failure_counts)
{
  nlohmann::json failure_json = nlohmann::json::object();
  for (const auto& [reason, count_value] : failure_counts) {
    failure_json[reason] = count_value;
  }
  return failure_json;
}

nlohmann::json extract_monitor_snapshot_base(
  const std::string& phase,
  const std::string& phase_label,
  double elapsed_ms,
  int left_box_id,
  int right_box_id,
  double box_front_x,
  double scene_y_shift)
{
  return {
    {"type", "extract_monitor_snapshot"},
    {"phase", phase},
    {"phase_label", phase_label},
    {"elapsed_ms", elapsed_ms},
    {"left_box_id", left_box_id},
    {"right_box_id", right_box_id},
    {"box_front_x", box_front_x},
    {"scene_y_shift", scene_y_shift}
  };
}

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
  const nlohmann::json& records)
{
  auto snapshot = extract_monitor_snapshot_base(
    "ik_candidates", "不重复 IK 候选", elapsed_ms, left_box_id, right_box_id, box_front_x, scene_y_shift);
  snapshot["snapshot_path"] = snapshot_path;
  snapshot["ik_trial_count"] = ik_result.trial_count;
  snapshot["ik_legal_count"] = ik_result.legal_count;
  snapshot["ik_wall_ms"] = ik_result.wall_ms;
  snapshot["ik_dedup_enabled"] = dedup_stats.enabled;
  snapshot["ik_dedup_input_count"] = dedup_stats.input_count;
  snapshot["ik_dedup_unique_count"] = dedup_stats.unique_count;
  snapshot["ik_dedup_removed_count"] = dedup_stats.removed_count;
  snapshot["ik_dedup_selected_count"] = dedup_stats.selected_count;
  snapshot["ik_dedup_ms"] = dedup_stats.elapsed_ms;
  snapshot["rejection_counts"] = rejection_counts;
  snapshot["records"] = records;
  return snapshot;
}

nlohmann::json extract_monitor_ik_snapshot(
  const ExtractMonitorIkSnapshotRequest& request)
{
  if (!request.ik_result) {
    return nlohmann::json::object();
  }
  return extract_monitor_ik_snapshot(
    request.snapshot_path,
    request.elapsed_ms,
    request.left_box_id,
    request.right_box_id,
    request.box_front_x,
    request.scene_y_shift,
    *request.ik_result,
    request.dedup_stats,
    request.rejection_counts,
    request.records);
}

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
  const nlohmann::json& records)
{
  auto snapshot = extract_monitor_snapshot_base(
    "extract_successes", "抽离成功候选", elapsed_ms, left_box_id, right_box_id, box_front_x, scene_y_shift);
  snapshot["input_candidate_count"] = input_candidate_count;
  snapshot["success_count"] = success_count;
  snapshot["worker_count"] = worker_count;
  snapshot["failure_counts"] = failure_counts_json(failure_counts);
  snapshot["records"] = records;
  return snapshot;
}

nlohmann::json extract_monitor_extract_snapshot(
  const ExtractMonitorExtractSnapshotRequest& request)
{
  return extract_monitor_extract_snapshot(
    request.elapsed_ms,
    request.left_box_id,
    request.right_box_id,
    request.box_front_x,
    request.scene_y_shift,
    request.input_candidate_count,
    request.success_count,
    request.worker_count,
    request.failure_counts,
    request.records);
}

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
  const nlohmann::json& records)
{
  auto snapshot = extract_monitor_snapshot_base(
    "loaded_plan_successes", "负重规划成功候选", elapsed_ms, left_box_id, right_box_id, box_front_x, scene_y_shift);
  snapshot["extract_success_count"] = extract_success_count;
  snapshot["attempted_count"] = attempted_count;
  snapshot["success_count"] = success_count;
  snapshot["loaded_plan_batch_wall_ms"] = loaded_plan_batch_wall_ms;
  snapshot["loaded_parallel_workers"] = loaded_parallel_workers;
  snapshot["loaded_candidate_limit"] = loaded_candidate_limit;
  snapshot["failure_counts"] = failure_counts_json(failure_counts);
  snapshot["records"] = records;
  return snapshot;
}

nlohmann::json extract_monitor_loaded_snapshot(
  const ExtractMonitorLoadedSnapshotRequest& request)
{
  return extract_monitor_loaded_snapshot(
    request.elapsed_ms,
    request.left_box_id,
    request.right_box_id,
    request.box_front_x,
    request.scene_y_shift,
    request.extract_success_count,
    request.attempted_count,
    request.success_count,
    request.loaded_plan_batch_wall_ms,
    request.loaded_parallel_workers,
    request.loaded_candidate_limit,
    request.failure_counts,
    request.records);
}

nlohmann::json extract_monitor_final_snapshot(
  double elapsed_ms,
  int left_box_id,
  int right_box_id,
  double box_front_x,
  double scene_y_shift,
  const nlohmann::json& record,
  const nlohmann::json& replay_stages)
{
  auto snapshot = extract_monitor_snapshot_base(
    "final_selected", "最终采用方案", elapsed_ms, left_box_id, right_box_id, box_front_x, scene_y_shift);
  snapshot["records"] = nlohmann::json::array({record});
  snapshot["replay_stages"] = replay_stages;
  return snapshot;
}

nlohmann::json extract_monitor_final_snapshot(
  const ExtractMonitorFinalSnapshotRequest& request)
{
  return extract_monitor_final_snapshot(
    request.elapsed_ms,
    request.left_box_id,
    request.right_box_id,
    request.box_front_x,
    request.scene_y_shift,
    request.record,
    request.replay_stages);
}

nlohmann::json extract_monitor_full_selected_snapshot(
  nlohmann::json snapshot,
  double box_front_x,
  double scene_y_shift,
  double total_elapsed_ms,
  const std::array<double, 4>& stage_elapsed_ms)
{
  if (!snapshot.is_object()) {
    snapshot = nlohmann::json::object();
  }
  snapshot["phase"] = "full_selected";
  snapshot["phase_label"] = "完整流程最终采用方案";
  snapshot["box_front_x"] = box_front_x;
  snapshot["scene_y_shift"] = scene_y_shift;
  snapshot["elapsed_ms"] = total_elapsed_ms;
  snapshot["ik_elapsed_ms"] = stage_elapsed_ms[0];
  snapshot["extract_elapsed_ms"] = stage_elapsed_ms[1];
  snapshot["loaded_elapsed_ms"] = stage_elapsed_ms[2];
  snapshot["final_elapsed_ms"] = stage_elapsed_ms[3];
  return snapshot;
}

}  // namespace alfa_robot::motion
