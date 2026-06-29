#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include <rclcpp/duration.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <unordered_set>

namespace alfa_robot::motion
{

moveit::planning_interface::MoveGroupInterface::Plan single_state_plan(
  const moveit::core::RobotState& state,
  const std::vector<std::string>& target_names,
  double time_from_start_sec)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = target_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start_sec);
  point.positions.reserve(target_names.size());

  const auto& model_names = state.getRobotModel()->getVariableNames();
  const std::unordered_set<std::string> variable_names(model_names.begin(), model_names.end());
  for (const auto& name : target_names) {
    point.positions.push_back(variable_names.count(name) > 0 ? state.getVariablePosition(name) : 0.0);
  }

  trajectory.points.push_back(point);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_.joint_trajectory = trajectory;
  return plan;
}

}  // namespace alfa_robot::motion
