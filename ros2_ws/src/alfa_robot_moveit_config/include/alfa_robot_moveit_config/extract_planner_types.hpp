#pragma once

#include "alfa_robot_moveit_config/motion_core/task_geometry.hpp"

#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/msg/pose.hpp>

#include <limits>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct ExtractCandidate
{
  size_t step_index = 0;
  size_t candidate_index = 0;
  double retreat_x = 0.0;
  double retreat_delta_x = 0.0;
  double lift_z = 0.0;
  double lift_delta_z = 0.0;
  double pitch_up_rad = 0.0;
  double pitch_delta_rad = 0.0;
  geometry_msgs::msg::Pose target_pose;
  bool ik_success = false;
  bool state_valid = false;
  bool carried_clear = false;
  bool detached_from_neighbors = false;
  std::string rejection_reason;
  moveit::core::RobotStatePtr state;
};

struct DualExtractStepCandidate
{
  ExtractCandidate left;
  ExtractCandidate right;
  moveit::core::RobotStatePtr state;
  bool state_valid = false;
  bool left_detached = false;
  bool right_detached = false;
  double score = std::numeric_limits<double>::infinity();
  std::string rejection_reason;
};

struct ArmExtractPath
{
  bool success = false;
  std::string failure_reason;
  std::vector<moveit::core::RobotStatePtr> states;
  std::vector<ExtractCandidate> selected_candidates;
  double final_retreat_x = 0.0;
  double final_lift_z = 0.0;
  double final_pitch_deg = 0.0;
  size_t accepted_steps = 0;
  size_t failed_steps = 0;
};

struct ExtractRolloutTiming
{
  size_t candidate_order = 0;
  size_t h_index = 0;
  size_t seed_index = 0;
  double h = 0.0;
  double ik_score = 0.0;
  double ik_solve_ms = 0.0;
  double rollout_ms = 0.0;
  double interval_ms = 0.0;
  bool success = false;
  bool loaded_plan_attempted = false;
  bool loaded_plan_success = false;
  double loaded_plan_ms = 0.0;
  size_t loaded_plan_points = 0;
  size_t selected_left_loaded_pose_index = 0;
  size_t selected_right_loaded_pose_index = 0;
  double selected_left_loaded_pose_distance = 0.0;
  double selected_right_loaded_pose_distance = 0.0;
  double loaded_pose_distance_sum = 0.0;
  double loaded_pose_distance_l2 = 0.0;
  double loaded_pose_max_joint_delta = 0.0;
  size_t loaded_plan_rank = 0;
  size_t accepted_steps = 0;
  size_t failed_steps = 0;
  double final_retreat_x = 0.0;
  double final_lift_z = 0.0;
  double final_pitch_deg = 0.0;
  double right_final_retreat_x = 0.0;
  double right_final_lift_z = 0.0;
  double right_final_pitch_deg = 0.0;
  std::string failure_reason;
  std::string loaded_plan_failure_reason;
  moveit::core::RobotStatePtr final_state;
};

}  // namespace alfa_robot::motion
