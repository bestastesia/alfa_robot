#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct TipFloorUpdownTarget
{
  double position = 0.0;
  double resulting_min_tip_z = 0.0;
  bool clamped = false;
  bool feasible = false;
};

TipFloorUpdownTarget tip_floor_updown_target(
  double current_updown,
  double current_min_tip_z,
  double required_min_tip_z,
  double min_updown,
  double max_updown);

moveit::planning_interface::MoveGroupInterface::Plan single_state_plan(
  const moveit::core::RobotState& state,
  const std::vector<std::string>& target_names,
  double time_from_start_sec);

double nearest_equivalent_joint_position(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& joint_variable,
  double reference_position,
  double candidate_position);

moveit::planning_interface::MoveGroupInterface::Plan retime_plan_by_max_joint_speed(
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const moveit::core::RobotModelConstPtr& robot_model,
  double max_angular_speed_rad_s,
  double sample_rate_hz = 10.0,
  double max_updown_speed_m_s = 0.05);

}  // namespace alfa_robot::motion
