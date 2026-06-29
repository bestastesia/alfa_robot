#include "alfa_robot_moveit_config/extract_monitor_json.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <array>
#include <cassert>
#include <memory>

namespace
{

moveit::core::RobotModelPtr empty_model()
{
  const std::string urdf_xml =
    R"(<robot name="empty_robot"><link name="world"/></robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(<robot name="empty_robot"/>)"));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit::core::RobotModelPtr one_joint_model()
{
  const std::string urdf_xml =
    R"(<robot name="one_joint_robot">
      <link name="world"/>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <parent link="world"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
      </joint>
    </robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(<robot name="one_joint_robot"/>)"));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit::planning_interface::MoveGroupInterface::Plan one_point_plan()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  plan.trajectory_.joint_trajectory.points.push_back(point);
  return plan;
}

}  // namespace

int main()
{
  using alfa_robot::motion::AttachedBoxSpec;
  using alfa_robot::motion::ContainerPanel;
  using alfa_robot::motion::ExtractMonitorExtractSnapshotRequest;
  using alfa_robot::motion::ExtractMonitorFinalSnapshotRequest;
  using alfa_robot::motion::ExtractMonitorIkSnapshotRequest;
  using alfa_robot::motion::ExtractMonitorLoadedSnapshotRequest;
  using alfa_robot::motion::ExtractMonitorSelectedExtractReplayStateRequest;
  using alfa_robot::motion::ExtractMonitorTimingRecordsRequest;
  using alfa_robot::motion::LoadedPoseReplayStage;
  using alfa_robot::motion::StaticBoxObstacle;
  using alfa_robot::motion::attached_boxes_json;
  using alfa_robot::motion::attached_box_config_json;
  using alfa_robot::motion::container_obstacle_json;
  using alfa_robot::motion::container_panels_json;
  using alfa_robot::motion::extract_monitor_candidate_records_json;
  using alfa_robot::motion::extract_monitor_extract_snapshot;
  using alfa_robot::motion::extract_monitor_final_snapshot;
  using alfa_robot::motion::extract_monitor_full_selected_snapshot;
  using alfa_robot::motion::extract_monitor_loaded_snapshot;
  using alfa_robot::motion::extract_monitor_pre_attach_replay_extra;
  using alfa_robot::motion::extract_monitor_replay_context_json;
  using alfa_robot::motion::extract_monitor_selected_lateral_shift_replay_extra;
  using alfa_robot::motion::extract_monitor_selected_lateral_shift_replay_stages;
  using alfa_robot::motion::extract_monitor_selected_extract_replay_stage;
  using alfa_robot::motion::extract_monitor_selected_extract_replay_state_stage;
  using alfa_robot::motion::extract_monitor_selected_loaded_plan_replay_extra;
  using alfa_robot::motion::extract_monitor_selected_loaded_plan_replay_stage;
  using alfa_robot::motion::extract_monitor_snapshot_base;
  using alfa_robot::motion::extract_monitor_timing_records_json;
  using alfa_robot::motion::failure_counts_json;
  using alfa_robot::motion::static_box_obstacles_json;

  AttachedBoxSpec box;
  box.id = "carried_left_box_6";
  box.link_name = "left_v5_tool0";
  box.center_in_link = {0.1, 0.2, 0.3};
  box.size = {0.4, 0.5, 0.6};

  const auto boxes = attached_boxes_json({box});
  assert(boxes.is_array());
  assert(boxes.size() == 1);
  assert(boxes[0].at("id") == "carried_left_box_6");
  assert(boxes[0].at("link_name") == "left_v5_tool0");
  assert(boxes[0].at("center_in_link").size() == 3);
  assert(boxes[0].at("size").size() == 3);

  ContainerPanel panel;
  panel.id = "container_ceiling";
  panel.center = {1.0, 2.0, 3.0};
  panel.size = {4.0, 5.0, 6.0};
  const auto panels = container_panels_json({panel});
  assert(panels.size() == 1);
  assert(panels[0].at("id") == "container_ceiling");
  assert(panels[0].at("center")[2] == 3.0);
  assert(panels[0].at("size")[0] == 4.0);

  const auto container = container_obstacle_json(
    true, "world", 8.0, 2.2, 2.4, 0.9, -0.4, 0.0, -0.4, 0.0, 0.03, {panel});
  assert(container.at("enabled") == true);
  assert(container.at("frame") == "world");
  assert(container.at("center_y") == -0.4);
  assert(container.at("nominal_center_y") == 0.0);
  assert(container.at("panels").size() == 1);

  StaticBoxObstacle obstacle;
  obstacle.id = "box_wall_left";
  obstacle.center = {0.9, 0.4, 0.6};
  obstacle.size = {0.3, 0.4, 0.8};
  const auto static_boxes = static_box_obstacles_json(true, 6, 8, 0.002, {obstacle});
  assert(static_boxes.at("enabled") == true);
  assert(static_boxes.at("mode") == "dynamic_box_wall_with_pair_opening");
  assert(static_boxes.at("opening_left_box_id") == 6);
  assert(static_boxes.at("opening_right_box_id") == 8);
  assert(static_boxes.at("inset") == 0.002);
  assert(static_boxes.at("boxes").size() == 1);

  const auto attached_config = attached_box_config_json(true, 0.3, 0.4, 0.5);
  assert(attached_config.at("enabled") == true);
  assert(attached_config.at("depth") == 0.3);
  assert(attached_config.at("width") == 0.4);
  assert(attached_config.at("height") == 0.5);

  const auto failures = failure_counts_json({{"left_kdl_no_solution", 2}, {"unknown", 1}});
  assert(failures.is_object());
  assert(failures.at("left_kdl_no_solution") == 2);
  assert(failures.at("unknown") == 1);

  const auto snapshot = extract_monitor_snapshot_base(
    "extract_successes", "抽离成功候选", 12.5, 6, 8, 0.925, -0.4);
  assert(snapshot.at("type") == "extract_monitor_snapshot");
  assert(snapshot.at("phase") == "extract_successes");
  assert(snapshot.at("left_box_id") == 6);
  assert(snapshot.at("right_box_id") == 8);
  assert(snapshot.at("box_front_x") == 0.925);

  ik_benchmark::UpdownAwareIkResult ik_result;
  ik_result.trial_count = 512;
  ik_result.legal_count = 128;
  ik_result.wall_ms = 42.5;
  alfa_robot::motion::IkCandidateSelectionStats dedup_stats;
  dedup_stats.enabled = true;
  dedup_stats.input_count = 128;
  dedup_stats.unique_count = 64;
  dedup_stats.removed_count = 64;
  dedup_stats.selected_count = 32;
  dedup_stats.elapsed_ms = 1.5;
  const auto ik_snapshot_from_request = extract_monitor_ik_snapshot(
    ExtractMonitorIkSnapshotRequest{
      "/tmp/snapshot.json",
      11.0,
      6,
      8,
      0.925,
      -0.4,
      &ik_result,
      dedup_stats,
      nlohmann::json{{"tip_error_too_large", 7}},
      nlohmann::json::array({nlohmann::json{{"candidate_index", 0}}})});
  assert(ik_snapshot_from_request.at("phase") == "ik_candidates");
  assert(ik_snapshot_from_request.at("snapshot_path") == "/tmp/snapshot.json");
  assert(ik_snapshot_from_request.at("ik_trial_count") == 512);
  assert(ik_snapshot_from_request.at("ik_legal_count") == 128);
  assert(ik_snapshot_from_request.at("ik_wall_ms") == 42.5);
  assert(ik_snapshot_from_request.at("ik_dedup_enabled") == true);
  assert(ik_snapshot_from_request.at("ik_dedup_unique_count") == 64);
  assert(ik_snapshot_from_request.at("rejection_counts").at("tip_error_too_large") == 7);
  assert(ik_snapshot_from_request.at("records").size() == 1);
  const auto empty_ik_snapshot = extract_monitor_ik_snapshot(ExtractMonitorIkSnapshotRequest{});
  assert(empty_ik_snapshot.is_object());
  assert(empty_ik_snapshot.empty());

  const auto extract_snapshot = extract_monitor_extract_snapshot(
    23.0, 6, 8, 0.925, -0.4, 64, 12, 8, {{"collision", 3}}, nlohmann::json::array());
  assert(extract_snapshot.at("phase") == "extract_successes");
  assert(extract_snapshot.at("input_candidate_count") == 64);
  assert(extract_snapshot.at("success_count") == 12);
  assert(extract_snapshot.at("worker_count") == 8);
  assert(extract_snapshot.at("failure_counts").at("collision") == 3);

  const auto extract_snapshot_from_request = extract_monitor_extract_snapshot(
    ExtractMonitorExtractSnapshotRequest{
      24.0,
      6,
      8,
      0.925,
      -0.4,
      64,
      13,
      16,
      {{"left_kdl_no_solution", 5}},
      nlohmann::json::array({nlohmann::json{{"candidate_order", 12}}})});
  assert(extract_snapshot_from_request.at("phase") == "extract_successes");
  assert(extract_snapshot_from_request.at("input_candidate_count") == 64);
  assert(extract_snapshot_from_request.at("success_count") == 13);
  assert(extract_snapshot_from_request.at("worker_count") == 16);
  assert(extract_snapshot_from_request.at("failure_counts").at("left_kdl_no_solution") == 5);
  assert(extract_snapshot_from_request.at("records").size() == 1);

  const auto loaded_snapshot = extract_monitor_loaded_snapshot(
    34.0, 6, 8, 0.925, -0.4, 12, 4, 1, 30.0, 8, 8, {{"plan_failed", 3}}, nlohmann::json::array());
  assert(loaded_snapshot.at("phase") == "loaded_plan_successes");
  assert(loaded_snapshot.at("extract_success_count") == 12);
  assert(loaded_snapshot.at("attempted_count") == 4);
  assert(loaded_snapshot.at("success_count") == 1);
  assert(loaded_snapshot.at("loaded_parallel_workers") == 8);

  const auto loaded_snapshot_from_request = extract_monitor_loaded_snapshot(
    ExtractMonitorLoadedSnapshotRequest{
      35.0,
      6,
      8,
      0.925,
      -0.4,
      13,
      5,
      2,
      31.0,
      16,
      10,
      {{"direct_pipeline_planning_failed_code_99999", 4}},
      nlohmann::json::array({nlohmann::json{{"loaded_plan_rank", 1}}})});
  assert(loaded_snapshot_from_request.at("phase") == "loaded_plan_successes");
  assert(loaded_snapshot_from_request.at("extract_success_count") == 13);
  assert(loaded_snapshot_from_request.at("attempted_count") == 5);
  assert(loaded_snapshot_from_request.at("success_count") == 2);
  assert(loaded_snapshot_from_request.at("loaded_plan_batch_wall_ms") == 31.0);
  assert(loaded_snapshot_from_request.at("loaded_parallel_workers") == 16);
  assert(loaded_snapshot_from_request.at("loaded_candidate_limit") == 10);
  assert(loaded_snapshot_from_request.at("failure_counts").at("direct_pipeline_planning_failed_code_99999") == 4);
  assert(loaded_snapshot_from_request.at("records").size() == 1);

  alfa_robot::motion::ExtractRolloutTiming timing;
  timing.candidate_order = 12;
  timing.loaded_plan_rank = 3;
  const auto replay_context = extract_monitor_replay_context_json(timing, 6, 8);
  assert(replay_context.at("candidate_order") == 12);
  assert(replay_context.at("loaded_plan_rank") == 3);
  assert(replay_context.at("left_box_id") == 6);
  assert(replay_context.at("right_box_id") == 8);

  const auto pre_attach_ok = extract_monitor_pre_attach_replay_extra(
    timing, 6, 8, true, "joint_interpolation", 5.0, "ignored_failure");
  assert(pre_attach_ok.at("stage_kind") == "monitor_selected_pre_attach_loaded_to_ik_replay");
  assert(pre_attach_ok.at("candidate_order") == 12);
  assert(pre_attach_ok.at("loaded_plan_rank") == 3);
  assert(pre_attach_ok.at("valid") == true);
  assert(pre_attach_ok.at("method") == "joint_interpolation");
  assert(pre_attach_ok.at("transition_ms") == 5.0);
  assert(pre_attach_ok.at("failure_reason") == "");

  const auto pre_attach_failed = extract_monitor_pre_attach_replay_extra(
    timing, 6, 8, false, "rrt", 7.5, "collision");
  assert(pre_attach_failed.at("valid") == false);
  assert(pre_attach_failed.at("method") == "rrt");
  assert(pre_attach_failed.at("failure_reason") == "collision");

  timing.loaded_plan_success = true;
  timing.loaded_plan_failure_reason = "";
  const auto lateral_extra = extract_monitor_selected_lateral_shift_replay_extra(
    timing, 6, 8, nlohmann::json{{"stage_kind", "left_lateral_shift"}, {"valid", true}});
  assert(lateral_extra.at("stage_kind") == "left_lateral_shift");
  assert(lateral_extra.at("valid") == true);
  assert(lateral_extra.at("candidate_order") == 12);
  assert(lateral_extra.at("left_box_id") == 6);
  assert(lateral_extra.at("right_box_id") == 8);
  assert(lateral_extra.at("loaded_plan_success") == true);
  assert(lateral_extra.at("loaded_plan_failure_reason") == "");

  timing.loaded_plan_ms = 31.5;
  timing.loaded_plan_points = 42;
  timing.loaded_plan_trajectory_distance = 2.25;
  const auto selected_loaded_extra = extract_monitor_selected_loaded_plan_replay_extra(timing, 6, 8);
  assert(selected_loaded_extra.at("stage_kind") == "monitor_selected_loaded_plan_replay");
  assert(selected_loaded_extra.at("valid") == true);
  assert(selected_loaded_extra.at("candidate_order") == 12);
  assert(selected_loaded_extra.at("loaded_plan_rank") == 3);
  assert(selected_loaded_extra.at("loaded_plan_ms") == 31.5);
  assert(selected_loaded_extra.at("loaded_plan_points") == 42);
  assert(selected_loaded_extra.at("loaded_plan_trajectory_distance") == 2.25);
  assert(selected_loaded_extra.at("moveit_attached_box_count") == 2);

  const auto model = empty_model();
  auto start_state = std::make_shared<moveit::core::RobotState>(model);
  auto goal_state = std::make_shared<moveit::core::RobotState>(model);
  start_state->setToDefaultValues();
  goal_state->setToDefaultValues();

  ik_benchmark::UpdownAwareIkCandidate candidate;
  candidate.h = 0.3;
  candidate.h_index = 2;
  candidate.seed_index = 5;
  candidate.score = 1.25;
  candidate.solver_path = "fixed_h_candidates";
  candidate.target_order = "normal";
  const auto candidate_records = extract_monitor_candidate_records_json({candidate}, {start_state, nullptr});
  assert(candidate_records.is_array());
  assert(candidate_records.size() == 1);
  assert(candidate_records[0].at("display_index") == 0);
  assert(candidate_records[0].at("h") == 0.3);
  assert(candidate_records[0].at("h_index") == 2);
  assert(candidate_records[0].at("seed_index") == 5);
  assert(candidate_records[0].at("solver_path") == "fixed_h_candidates");
  assert(candidate_records[0].at("state").at("joint_names").size() == 0);

  const auto empty_candidate_records = extract_monitor_candidate_records_json({candidate}, {nullptr});
  assert(empty_candidate_records.empty());

  LoadedPoseReplayStage shift_stage;
  shift_stage.stage_name = "shift_stage";
  shift_stage.plan = one_point_plan();
  shift_stage.start_state = start_state;
  shift_stage.goal_state = goal_state;
  shift_stage.extra = nlohmann::json{{"stage_kind", "shift"}, {"valid", true}};
  timing.lateral_shift_replay_stages = {shift_stage};
  const auto shift_stages = extract_monitor_selected_lateral_shift_replay_stages(
    timing, 6, 8, {}, {box}, nlohmann::json::object());
  assert(shift_stages.size() == 1);
  assert(shift_stages[0].at("stage") == "shift_stage");
  assert(shift_stages[0].at("extra").at("candidate_order") == 12);
  assert(shift_stages[0].at("attached_boxes").size() == 1);

  const auto extract_replay_stage = extract_monitor_selected_extract_replay_stage(
    "extract_monitor_L6_R8",
    5,
    12,
    one_point_plan(),
    *start_state,
    6,
    8,
    {},
    {box},
    nlohmann::json{{"boxes", nlohmann::json::array()}},
    nlohmann::json{{"valid", true}});
  assert(extract_replay_stage.at("stage") == "extract_monitor_L6_R8/selected_extract_step_5");
  assert(extract_replay_stage.at("extra").at("stage_kind") == "monitor_selected_extract_replay");
  assert(extract_replay_stage.at("extra").at("candidate_order") == 12);
  assert(extract_replay_stage.at("extra").at("left_box_id") == 6);
  assert(extract_replay_stage.at("extra").at("right_box_id") == 8);
  assert(extract_replay_stage.at("extra").at("valid") == true);
  assert(extract_replay_stage.at("attached_boxes").size() == 1);
  assert(extract_replay_stage.at("static_box_obstacles").at("boxes").is_array());

  const auto joint_model = one_joint_model();
  moveit::core::RobotState joint_state(joint_model);
  joint_state.setToDefaultValues();
  joint_state.setVariablePosition("joint1", 0.42);
  const auto state_replay_stage = extract_monitor_selected_extract_replay_state_stage(
    "extract_monitor_L6_R8",
    6,
    12,
    joint_state,
    6,
    8,
    {"joint1", "missing_joint"},
    {box},
    nlohmann::json{{"boxes", nlohmann::json::array()}},
    nlohmann::json{{"valid", true}},
    0.6);
  assert(state_replay_stage.at("trajectory").at("joint_names").size() == 2);
  assert(state_replay_stage.at("trajectory").at("points").size() == 1);
  assert(state_replay_stage.at("trajectory").at("points")[0].at("positions")[0] == 0.42);
  assert(state_replay_stage.at("trajectory").at("points")[0].at("positions")[1] == 0.0);

  const auto request_state_replay_stage = extract_monitor_selected_extract_replay_state_stage(
    ExtractMonitorSelectedExtractReplayStateRequest{
      "extract_monitor_L6_R8",
      7,
      13,
      &joint_state,
      6,
      8,
      {"joint1", "missing_joint"},
      {box},
      nlohmann::json{{"boxes", nlohmann::json::array()}},
      nlohmann::json{{"valid", true}, {"source", "request"}},
      0.7});
  assert(request_state_replay_stage.at("stage") == "extract_monitor_L6_R8/selected_extract_step_7");
  assert(request_state_replay_stage.at("extra").at("candidate_order") == 13);
  assert(request_state_replay_stage.at("extra").at("source") == "request");
  assert(request_state_replay_stage.at("trajectory").at("points")[0].at("positions")[0] == 0.42);

  const auto empty_request_state_replay_stage = extract_monitor_selected_extract_replay_state_stage(
    ExtractMonitorSelectedExtractReplayStateRequest{});
  assert(empty_request_state_replay_stage.is_object());
  assert(empty_request_state_replay_stage.empty());

  timing.loaded_start_state = start_state;
  timing.loaded_goal_state = goal_state;
  timing.loaded_plan = one_point_plan();
  std::vector<alfa_robot::motion::ExtractRolloutTiming> timing_list(2);
  timing_list[1] = timing;
  const auto timing_records = extract_monitor_timing_records_json(
    timing_list,
    {1, 99},
    "extract_monitor_L6_R8",
    6,
    8,
    {},
    {box},
    nlohmann::json::object(),
    [start_state](const alfa_robot::motion::ExtractRolloutTiming&) {
      return start_state;
    });
  assert(timing_records.is_array());
  assert(timing_records.size() == 1);
  assert(timing_records[0].at("display_index") == 0);
  assert(timing_records[0].at("candidate_order") == 12);
  assert(timing_records[0].at("loaded_plan_rank") == 3);
  assert(timing_records[0].at("replay_stage_count") == 2);
  assert(timing_records[0].at("replay_stages")[0].at("attached_boxes").size() == 1);

  const auto timing_records_from_request = extract_monitor_timing_records_json(
    ExtractMonitorTimingRecordsRequest{
      &timing_list,
      {1, 99},
      "extract_monitor_L6_R8",
      6,
      8,
      {},
      {box},
      nlohmann::json::object(),
      [start_state](const alfa_robot::motion::ExtractRolloutTiming&) {
        return start_state;
      }});
  assert(timing_records_from_request == timing_records);
  assert(extract_monitor_timing_records_json(ExtractMonitorTimingRecordsRequest{}).empty());

  const auto empty_timing_records = extract_monitor_timing_records_json(
    timing_list,
    {1},
    "extract_monitor_L6_R8",
    6,
    8,
    {},
    {box},
    nlohmann::json::object(),
    [](const alfa_robot::motion::ExtractRolloutTiming&) {
      return moveit::core::RobotStatePtr{};
    });
  assert(empty_timing_records.empty());

  const auto loaded_stage = extract_monitor_selected_loaded_plan_replay_stage(
    "extract_monitor_L6_R8", timing, 6, 8, {}, {box}, nlohmann::json::object());
  assert(loaded_stage.is_object());
  assert(loaded_stage.at("stage") == "extract_monitor_L6_R8/selected_loaded_plan");
  assert(loaded_stage.at("extra").at("stage_kind") == "monitor_selected_loaded_plan_replay");
  assert(loaded_stage.at("extra").at("candidate_order") == 12);
  assert(loaded_stage.at("attached_boxes").size() == 1);

  const auto final_snapshot = extract_monitor_final_snapshot(
    45.0,
    6,
    8,
    0.925,
    -0.4,
    nlohmann::json{{"candidate_order", 12}},
    nlohmann::json::array({nlohmann::json{{"stage", "selected_loaded_plan"}}}));
  assert(final_snapshot.at("phase") == "final_selected");
  assert(final_snapshot.at("phase_label") == "最终采用方案");
  assert(final_snapshot.at("records").size() == 1);
  assert(final_snapshot.at("records")[0].at("candidate_order") == 12);
  assert(final_snapshot.at("replay_stages").size() == 1);
  assert(final_snapshot.at("replay_stages")[0].at("stage") == "selected_loaded_plan");

  const auto final_snapshot_from_request = extract_monitor_final_snapshot(
    ExtractMonitorFinalSnapshotRequest{
      46.0,
      6,
      8,
      0.925,
      -0.4,
      nlohmann::json{{"candidate_order", 13}},
      nlohmann::json::array({nlohmann::json{{"stage", "selected_extract_step"}}})});
  assert(final_snapshot_from_request.at("phase") == "final_selected");
  assert(final_snapshot_from_request.at("elapsed_ms") == 46.0);
  assert(final_snapshot_from_request.at("records")[0].at("candidate_order") == 13);
  assert(final_snapshot_from_request.at("replay_stages")[0].at("stage") == "selected_extract_step");

  const auto full_snapshot = extract_monitor_full_selected_snapshot(
    final_snapshot, 0.925, -0.4, 123.0, std::array<double, 4>{1.0, 2.0, 3.0, 4.0});
  assert(full_snapshot.at("phase") == "full_selected");
  assert(full_snapshot.at("phase_label") == "完整流程最终采用方案");
  assert(full_snapshot.at("box_front_x") == 0.925);
  assert(full_snapshot.at("scene_y_shift") == -0.4);
  assert(full_snapshot.at("elapsed_ms") == 123.0);
  assert(full_snapshot.at("ik_elapsed_ms") == 1.0);
  assert(full_snapshot.at("extract_elapsed_ms") == 2.0);
  assert(full_snapshot.at("loaded_elapsed_ms") == 3.0);
  assert(full_snapshot.at("final_elapsed_ms") == 4.0);
  assert(full_snapshot.at("records").size() == 1);

  return 0;
}
