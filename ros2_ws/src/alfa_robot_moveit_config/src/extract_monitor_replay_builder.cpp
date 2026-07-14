#include "alfa_robot_moveit_config/extract_monitor_replay_builder.hpp"

#include <utility>

namespace alfa_robot::motion
{

ExtractMonitorReplayBuildRequest make_extract_monitor_replay_request(
  const ExtractMonitorState& state,
  const std::vector<std::string>& target_names,
  const nlohmann::json& static_box_obstacles,
  moveit::core::RobotStatePtr ik_goal_state)
{
  ExtractMonitorReplayBuildRequest request;
  request.prefix = state.prefix;
  request.left_box_id = state.left_box_id;
  request.right_box_id = state.right_box_id;
  request.target_names = target_names;
  request.carried_boxes = {state.left_box, state.right_box};
  request.static_box_obstacles = static_box_obstacles;
  request.loaded_start_state = state.loaded_start_state;
  request.ik_goal_state = std::move(ik_goal_state);
  return request;
}

nlohmann::json ExtractMonitorReplayBuilder::build(
  ExtractRolloutTiming& selected,
  const ExtractMonitorReplayBuildRequest& request) const
{
  if (ensure_extract_replay) {
    ensure_extract_replay(selected);
  }

  nlohmann::json replay_stages = nlohmann::json::array();
  if (request.loaded_start_state && request.ik_goal_state) {
    std::string pre_contact_reason;
    const auto pre_contact_state = build_pre_contact_state
      ? build_pre_contact_state(*request.loaded_start_state, *request.ik_goal_state, &pre_contact_reason)
      : nullptr;
    if (pre_contact_state) {
      const auto approach_t0 = now ? now() : std::chrono::steady_clock::now();
      const auto approach = transition_planner.plan(*request.loaded_start_state, *pre_contact_state);
      const auto approach_t1 = now ? now() : std::chrono::steady_clock::now();
      const double approach_ms = std::chrono::duration<double, std::milli>(
        approach_t1 - approach_t0).count();
      auto approach_extra = extract_monitor_pre_attach_replay_extra(
        selected,
        request.left_box_id,
        request.right_box_id,
        approach.valid,
        approach.method,
        approach_ms,
        approach.failure_reason);
      approach_extra["stage_kind"] = "monitor_selected_pre_attach_loaded_to_pre_contact_replay";
      approach_extra["pre_contact_offset_m"] = 0.05;
      replay_stages.push_back(extract_monitor_stage_json(
        request.prefix + "/selected_pre_attach_loaded_to_pre_contact",
        approach.plan,
        *request.loaded_start_state,
        *pre_contact_state,
        request.target_names,
        {},
        request.static_box_obstacles,
        approach_extra));

      const auto contact_t0 = now ? now() : std::chrono::steady_clock::now();
      const auto contact = transition_planner.plan(*pre_contact_state, *request.ik_goal_state);
      const auto contact_t1 = now ? now() : std::chrono::steady_clock::now();
      const double contact_ms = std::chrono::duration<double, std::milli>(
        contact_t1 - contact_t0).count();
      auto contact_extra = extract_monitor_pre_attach_replay_extra(
        selected,
        request.left_box_id,
        request.right_box_id,
        contact.valid,
        contact.method,
        contact_ms,
        contact.failure_reason);
      contact_extra["stage_kind"] = "monitor_selected_pre_attach_pre_contact_to_ik_replay";
      contact_extra["pre_contact_offset_m"] = 0.05;
      replay_stages.push_back(extract_monitor_stage_json(
        request.prefix + "/selected_pre_attach_pre_contact_to_ik",
        contact.plan,
        *pre_contact_state,
        *request.ik_goal_state,
        request.target_names,
        {},
        request.static_box_obstacles,
        contact_extra));
    } else {
      const auto transition_t0 = now ? now() : std::chrono::steady_clock::now();
      const auto transition = transition_planner.plan(*request.loaded_start_state, *request.ik_goal_state);
      const auto transition_t1 = now ? now() : std::chrono::steady_clock::now();
      const double transition_ms = std::chrono::duration<double, std::milli>(
        transition_t1 - transition_t0).count();
      auto extra = extract_monitor_pre_attach_replay_extra(
        selected,
        request.left_box_id,
        request.right_box_id,
        transition.valid,
        transition.method,
        transition_ms,
        transition.failure_reason);
      extra["pre_contact_fallback_reason"] = pre_contact_reason;
      replay_stages.push_back(extract_monitor_stage_json(
        request.prefix + "/selected_pre_attach_loaded_to_ik",
        transition.plan,
        *request.loaded_start_state,
        *request.ik_goal_state,
        request.target_names,
        {},
        request.static_box_obstacles,
        extra));
    }
  }

  for (const auto& stage : selected.rollout_records) {
    replay_stages.push_back(stage);
  }

  const auto shift_replay_stages = extract_monitor_selected_lateral_shift_replay_stages(
    selected,
    request.left_box_id,
    request.right_box_id,
    request.target_names,
    request.carried_boxes,
    request.static_box_obstacles);
  for (const auto& stage : shift_replay_stages) {
    replay_stages.push_back(stage);
  }

  const auto loaded_stage = extract_monitor_selected_loaded_plan_replay_stage(
    request.prefix,
    selected,
    request.left_box_id,
    request.right_box_id,
    request.target_names,
    request.carried_boxes,
    request.static_box_obstacles);
  if (!loaded_stage.is_null()) {
    replay_stages.push_back(loaded_stage);
  }

  return replay_stages;
}

}  // namespace alfa_robot::motion
