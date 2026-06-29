#include "alfa_robot_moveit_config/execution_trajectory_adapter.hpp"

#include <cassert>
#include <cmath>
#include <string>

namespace
{

trajectory_msgs::msg::JointTrajectory make_source()
{
  trajectory_msgs::msg::JointTrajectory source;
  source.joint_names = {
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3", "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3", "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
  };
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (size_t i = 0; i < source.joint_names.size(); ++i) {
    point.positions.push_back(static_cast<double>(i + 1));
  }
  source.points.push_back(point);
  return source;
}

}  // namespace

int main()
{
  using alfa_robot::motion::ExecutionTrajectoryAdapter;
  using alfa_robot::motion::ExecutionTrajectoryAdapterConfig;
  using alfa_robot::motion::ExecutionTrajectoryBuildRequest;
  using alfa_robot::motion::ExecutionJointStateMatchRequest;
  using alfa_robot::motion::ExecutionStateMatchRequest;

  ExecutionTrajectoryAdapterConfig config;
  config.include_turn = true;
  ExecutionTrajectoryAdapter adapter(config);

  assert(adapter.alfaToMoveItJointName("left_joint3") == "left_v5_joint3");
  assert(adapter.moveItToAlfaJointName("right_v5_joint6") == "right_joint6");

  auto source = make_source();
  ExecutionTrajectoryAdapter::FollowJointTrajectory::Goal goal;
  std::string reason;
  ExecutionTrajectoryBuildRequest request;
  request.source = &source;
  request.is_robot_variable = [](const std::string& name) { return name == "turn"; };
  request.hold_position = [](const std::string& name) { return name == "turn" ? 0.42 : 0.0; };
  const bool ok = adapter.buildGoal(
    request,
    &goal,
    &reason);
  assert(ok);
  assert(reason.empty());
  assert(goal.trajectory.joint_names.size() == 13);
  assert(goal.trajectory.joint_names.front() == "left_joint1");
  assert(goal.trajectory.joint_names.back() == "turn");
  assert(goal.trajectory.points.size() == 1);
  assert(std::abs(goal.trajectory.points.front().positions.front() - 1.0) < 1e-12);
  assert(std::abs(goal.trajectory.points.front().positions.back() - 0.42) < 1e-12);

  source.joint_names.push_back("updown");
  source.points.front().positions.push_back(0.1);
  trajectory_msgs::msg::JointTrajectoryPoint second = source.points.front();
  second.positions.back() = 0.2;
  source.points.push_back(second);
  reason.clear();
  ExecutionTrajectoryBuildRequest reject_request;
  reject_request.source = &source;
  const bool rejected = adapter.buildGoal(
    reject_request,
    &goal,
    &reason);
  assert(!rejected);
  assert(reason.find("updown") != std::string::npos);

  ExecutionStateMatchRequest state_match;
  state_match.target_names = {"left_v5_joint1", "ignored_joint"};
  state_match.tolerance = 0.05;
  state_match.is_robot_variable = [](const std::string& name) {
    return name == "left_v5_joint1";
  };
  state_match.goal_position = [](const std::string& name) {
    return name == "left_v5_joint1" ? 1.0 : 0.0;
  };
  state_match.current_position = [](const std::string& name) {
    return name == "left_v5_joint1" ? 1.02 : 100.0;
  };
  assert(adapter.robotStateMatches(state_match));
  state_match.current_position = [](const std::string& name) {
    return name == "left_v5_joint1" ? 1.10 : 0.0;
  };
  assert(!adapter.robotStateMatches(state_match));

  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"left_joint1", "right_v5_joint2"};
  joint_state.position = {0.98, -0.51};
  ExecutionJointStateMatchRequest joint_match;
  joint_match.current = &joint_state;
  joint_match.target_names = {"left_v5_joint1", "right_v5_joint2"};
  joint_match.tolerance = 0.05;
  joint_match.is_robot_variable = [](const std::string& name) {
    return name == "left_v5_joint1" || name == "right_v5_joint2";
  };
  joint_match.goal_position = [](const std::string& name) {
    if (name == "left_v5_joint1") return 1.0;
    if (name == "right_v5_joint2") return -0.5;
    return 0.0;
  };
  assert(adapter.jointStateMatches(joint_match));
  joint_state.position[0] = 0.8;
  assert(!adapter.jointStateMatches(joint_match));

  return 0;
}
