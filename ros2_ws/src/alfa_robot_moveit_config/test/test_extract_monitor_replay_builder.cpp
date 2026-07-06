#include "alfa_robot_moveit_config/extract_monitor_replay_builder.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
#include <memory>
#include <string>

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
  using alfa_robot::motion::ExtractMonitorReplayBuildRequest;
  using alfa_robot::motion::ExtractMonitorReplayBuilder;
  using alfa_robot::motion::ExtractMonitorState;
  using alfa_robot::motion::LoadedPoseReplayStage;
  using alfa_robot::motion::make_extract_monitor_replay_request;

  const auto model = empty_model();
  auto start_state = std::make_shared<moveit::core::RobotState>(model);
  auto ik_state = std::make_shared<moveit::core::RobotState>(model);
  auto loaded_goal_state = std::make_shared<moveit::core::RobotState>(model);
  start_state->setToDefaultValues();
  ik_state->setToDefaultValues();
  loaded_goal_state->setToDefaultValues();

  AttachedBoxSpec left_box;
  left_box.id = "left_box";
  left_box.link_name = "left_tool0";
  left_box.size = {0.4, 0.4, 0.4};
  AttachedBoxSpec right_box;
  right_box.id = "right_box";
  right_box.link_name = "right_tool0";
  right_box.size = {0.4, 0.4, 0.4};

  alfa_robot::motion::ExtractRolloutTiming timing;
  timing.candidate_order = 7;
  timing.loaded_plan_rank = 2;
  timing.loaded_plan_success = true;
  timing.rollout_records.push_back(nlohmann::json{{"stage", "extract_step"}});

  LoadedPoseReplayStage shift_stage;
  shift_stage.stage_name = "shift_stage";
  shift_stage.plan = one_point_plan();
  shift_stage.start_state = ik_state;
  shift_stage.goal_state = ik_state;
  shift_stage.extra = nlohmann::json{{"stage_kind", "shift"}, {"valid", true}};
  timing.lateral_shift_replay_stages.push_back(shift_stage);

  timing.loaded_start_state = ik_state;
  timing.loaded_goal_state = loaded_goal_state;
  timing.loaded_plan = one_point_plan();

  ExtractMonitorReplayBuilder builder;
  bool ensured = false;
  builder.ensure_extract_replay = [&](auto& selected) {
    ensured = true;
    selected.rollout_records.push_back(nlohmann::json{{"stage", "ensured_extract_step"}});
  };
  builder.transition_planner.make_interpolated_plan = [](const auto&, const auto&, double) {
    return one_point_plan();
  };
  builder.transition_planner.densify_plan = [](const auto& plan) {
    return plan;
  };
  builder.transition_planner.validate_plan = [](const auto&, const auto&, std::string* reason) {
    if (reason) *reason = "";
    return true;
  };
  const auto t0 = std::chrono::steady_clock::time_point{};
  int clock_calls = 0;
  builder.now = [&] {
    return t0 + std::chrono::milliseconds(5 * clock_calls++);
  };

  ExtractMonitorReplayBuildRequest request;
  request.prefix = "extract_monitor_L6_R8";
  request.left_box_id = 6;
  request.right_box_id = 8;
  request.carried_boxes = {left_box, right_box};
  request.loaded_start_state = start_state;
  request.ik_goal_state = ik_state;

  const auto stages = builder.build(timing, request);
  assert(ensured);
  assert(stages.is_array());
  assert(stages.size() == 5);
  assert(stages[0].at("stage") == "extract_monitor_L6_R8/selected_pre_attach_loaded_to_ik");
  assert(stages[0].at("extra").at("stage_kind") == "monitor_selected_pre_attach_loaded_to_ik_replay");
  assert(stages[0].at("extra").at("transition_ms") == 5.0);
  assert(stages[1].at("stage") == "extract_step");
  assert(stages[2].at("stage") == "ensured_extract_step");
  assert(stages[3].at("stage") == "shift_stage");
  assert(stages[3].at("attached_boxes").size() == 2);
  assert(stages[4].at("stage") == "extract_monitor_L6_R8/selected_loaded_plan");
  assert(stages[4].at("extra").at("moveit_attached_box_count") == 2);

  ExtractMonitorReplayBuilder missing_transition_builder;
  missing_transition_builder.ensure_extract_replay = {};
  const auto missing_transition_stages = missing_transition_builder.build(timing, request);
  assert(missing_transition_stages.size() == 5);
  assert(missing_transition_stages[0].at("extra").at("valid") == false);
  assert(missing_transition_stages[0].at("extra").at("failure_reason") == "transition_planner_missing_basic_adapter");

  request.loaded_start_state.reset();
  auto no_pre_attach_timing = timing;
  no_pre_attach_timing.rollout_records.resize(1);
  const auto no_pre_attach_stages = builder.build(no_pre_attach_timing, request);
  assert(no_pre_attach_stages.size() == 4);
  assert(no_pre_attach_stages[0].at("stage") == "extract_step");

  ExtractMonitorState monitor_state;
  monitor_state.prefix = "extract_monitor_L1_R2";
  monitor_state.left_box_id = 1;
  monitor_state.right_box_id = 2;
  monitor_state.left_box = left_box;
  monitor_state.right_box = right_box;
  monitor_state.loaded_start_state = start_state;
  const auto factory_request = make_extract_monitor_replay_request(
    monitor_state,
    {"updown", "leftjoint1"},
    nlohmann::json{{"boxes", nlohmann::json::array()}},
    ik_state);
  assert(factory_request.prefix == "extract_monitor_L1_R2");
  assert(factory_request.left_box_id == 1);
  assert(factory_request.right_box_id == 2);
  assert(factory_request.target_names.size() == 2);
  assert(factory_request.carried_boxes.size() == 2);
  assert(factory_request.loaded_start_state == start_state);
  assert(factory_request.ik_goal_state == ik_state);
  assert(factory_request.static_box_obstacles.at("boxes").is_array());

  return 0;
}
