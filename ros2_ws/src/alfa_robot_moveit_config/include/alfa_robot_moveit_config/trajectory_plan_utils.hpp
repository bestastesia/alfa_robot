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

moveit::planning_interface::MoveGroupInterface::Plan retime_plan_by_max_joint_speed(
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const moveit::core::RobotModelConstPtr& robot_model,
  double max_angular_speed_rad_s,
  double sample_rate_hz = 10.0,
  double max_updown_speed_m_s = 0.05);

}  // namespace alfa_robot::motion
