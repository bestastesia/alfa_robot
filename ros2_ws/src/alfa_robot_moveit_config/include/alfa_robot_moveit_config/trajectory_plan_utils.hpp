#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <string>
#include <vector>

namespace alfa_robot::motion
{

moveit::planning_interface::MoveGroupInterface::Plan single_state_plan(
  const moveit::core::RobotState& state,
  const std::vector<std::string>& target_names,
  double time_from_start_sec);

}  // namespace alfa_robot::motion
