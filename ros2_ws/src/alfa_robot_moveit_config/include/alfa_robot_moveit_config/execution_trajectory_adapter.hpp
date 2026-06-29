#pragma once

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <functional>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct ExecutionTrajectoryAdapterConfig
{
  bool include_turn = true;
  bool allow_hold_missing_target_joints = true;
  bool reject_unmapped_planned_joints = true;
};

struct ExecutionTrajectoryBuildRequest
{
  const trajectory_msgs::msg::JointTrajectory* source = nullptr;
  std::function<bool(const std::string& moveit_joint_name)> is_robot_variable;
  std::function<double(const std::string& moveit_joint_name)> hold_position;
};

struct ExecutionStateMatchRequest
{
  std::function<bool(const std::string& moveit_joint_name)> is_robot_variable;
  std::function<double(const std::string& moveit_joint_name)> goal_position;
  std::function<double(const std::string& moveit_joint_name)> current_position;
  std::vector<std::string> target_names;
  double tolerance = 0.0;
};

struct ExecutionJointStateMatchRequest
{
  std::function<bool(const std::string& moveit_joint_name)> is_robot_variable;
  std::function<double(const std::string& moveit_joint_name)> goal_position;
  const sensor_msgs::msg::JointState* current = nullptr;
  std::vector<std::string> target_names;
  double tolerance = 0.0;
};

class ExecutionTrajectoryAdapter
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  explicit ExecutionTrajectoryAdapter(ExecutionTrajectoryAdapterConfig config = {});

  const ExecutionTrajectoryAdapterConfig& config() const { return config_; }

  std::vector<std::string> targetJointNames() const;
  std::string alfaToMoveItJointName(const std::string& name) const;
  std::string moveItToAlfaJointName(const std::string& name) const;

  bool buildGoal(
    const ExecutionTrajectoryBuildRequest& request,
    FollowJointTrajectory::Goal* goal,
    std::string* reason) const;

  bool robotStateMatches(const ExecutionStateMatchRequest& request) const;

  bool jointStateMatches(const ExecutionJointStateMatchRequest& request) const;

private:
  static bool plannedJointChanges(
    const trajectory_msgs::msg::JointTrajectory& source,
    const std::string& joint_name);

  ExecutionTrajectoryAdapterConfig config_;
};

}  // namespace alfa_robot::motion
