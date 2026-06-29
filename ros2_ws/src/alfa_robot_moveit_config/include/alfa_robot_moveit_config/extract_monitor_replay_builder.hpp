#pragma once

#include "alfa_robot_moveit_config/extract_monitor_json.hpp"
#include "alfa_robot_moveit_config/extract_monitor_state.hpp"
#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"

#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct ExtractMonitorReplayBuildRequest
{
  std::string prefix;
  int left_box_id = 0;
  int right_box_id = 0;
  std::vector<std::string> target_names;
  std::vector<AttachedBoxSpec> carried_boxes;
  nlohmann::json static_box_obstacles = nlohmann::json::object();
  moveit::core::RobotStatePtr loaded_start_state;
  moveit::core::RobotStatePtr ik_goal_state;
};

ExtractMonitorReplayBuildRequest make_extract_monitor_replay_request(
  const ExtractMonitorState& state,
  const std::vector<std::string>& target_names,
  const nlohmann::json& static_box_obstacles,
  moveit::core::RobotStatePtr ik_goal_state);

using ExtractMonitorEnsureExtractReplay =
  std::function<void(ExtractRolloutTiming& selected)>;

using ExtractMonitorTransitionClock =
  std::function<std::chrono::steady_clock::time_point()>;

struct ExtractMonitorReplayBuilder
{
  ExtractMonitorTransitionPlanner transition_planner;
  ExtractMonitorEnsureExtractReplay ensure_extract_replay;
  ExtractMonitorTransitionClock now = [] { return std::chrono::steady_clock::now(); };

  nlohmann::json build(
    ExtractRolloutTiming& selected,
    const ExtractMonitorReplayBuildRequest& request) const;
};

}  // namespace alfa_robot::motion
