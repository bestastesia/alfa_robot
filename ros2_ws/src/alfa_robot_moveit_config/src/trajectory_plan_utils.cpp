#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include <moveit/robot_model/joint_model.h>
#include <rclcpp/duration.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_set>

namespace alfa_robot::motion
{

namespace
{

double point_time_s(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

double angular_variable_delta(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& name,
  double from,
  double to)
{
  const auto* joint = robot_model ? robot_model->getJointOfVariable(name) : nullptr;
  const auto* revolute_joint =
    dynamic_cast<const moveit::core::RevoluteJointModel*>(joint);
  if (!revolute_joint) {
    return 0.0;
  }
  return revolute_joint->isContinuous()
    ? std::atan2(std::sin(to - from), std::cos(to - from))
    : (to - from);
}

bool is_angular_variable(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& name)
{
  const auto* joint = robot_model ? robot_model->getJointOfVariable(name) : nullptr;
  return dynamic_cast<const moveit::core::RevoluteJointModel*>(joint) != nullptr;
}

std::vector<double> sample_positions_at_time(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  const moveit::core::RobotModelConstPtr& robot_model,
  double sample_time_s)
{
  const auto& points = trajectory.points;
  if (points.empty()) {
    return {};
  }
  if (sample_time_s <= point_time_s(points.front())) {
    return points.front().positions;
  }
  if (sample_time_s >= point_time_s(points.back())) {
    return points.back().positions;
  }

  size_t upper = 1;
  while (upper < points.size() && point_time_s(points[upper]) < sample_time_s) {
    ++upper;
  }
  upper = std::min(upper, points.size() - 1);
  const size_t lower = upper == 0 ? 0 : upper - 1;
  const double lower_time = point_time_s(points[lower]);
  const double upper_time = point_time_s(points[upper]);
  const double ratio = upper_time > lower_time + 1e-9
    ? std::clamp((sample_time_s - lower_time) / (upper_time - lower_time), 0.0, 1.0)
    : 0.0;

  std::vector<double> positions;
  positions.reserve(trajectory.joint_names.size());
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    if (i >= points[lower].positions.size() || i >= points[upper].positions.size()) {
      positions.push_back(0.0);
      continue;
    }
    const double from = points[lower].positions[i];
    const double to = points[upper].positions[i];
    const double value = is_angular_variable(robot_model, trajectory.joint_names[i])
      ? from + angular_variable_delta(robot_model, trajectory.joint_names[i], from, to) * ratio
      : from + (to - from) * ratio;
    positions.push_back(value);
  }
  return positions;
}

}  // namespace

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

moveit::planning_interface::MoveGroupInterface::Plan retime_plan_by_max_joint_speed(
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const moveit::core::RobotModelConstPtr& robot_model,
  double max_angular_speed_rad_s,
  double sample_rate_hz,
  double max_updown_speed_m_s)
{
  const auto& input = plan.trajectory_.joint_trajectory;
  if (!robot_model || max_angular_speed_rad_s <= 1e-9 ||
      input.points.size() < 2 || input.joint_names.empty()) {
    return plan;
  }

  std::vector<size_t> angular_indices;
  angular_indices.reserve(input.joint_names.size());
  for (size_t i = 0; i < input.joint_names.size(); ++i) {
    if (is_angular_variable(robot_model, input.joint_names[i])) {
      angular_indices.push_back(i);
    }
  }
  if (angular_indices.empty()) {
    return plan;
  }

  auto output = plan;
  auto& points = output.trajectory_.joint_trajectory.points;
  for (auto& point : points) {
    point.velocities.clear();
    point.accelerations.clear();
    point.effort.clear();
  }

  double elapsed_s = 0.0;
  points.front().time_from_start = rclcpp::Duration::from_seconds(0.0);
  for (size_t point_index = 1; point_index < points.size(); ++point_index) {
    const auto& previous = input.points[point_index - 1];
    const auto& current = input.points[point_index];

    double required_s = 0.0;
    for (const size_t joint_index : angular_indices) {
      if (joint_index >= previous.positions.size() ||
          joint_index >= current.positions.size()) {
        continue;
      }
      const double delta = std::abs(angular_variable_delta(
        robot_model,
        input.joint_names[joint_index],
        previous.positions[joint_index],
        current.positions[joint_index]));
      required_s = std::max(required_s, delta / max_angular_speed_rad_s);
    }
    if (max_updown_speed_m_s > 1e-9) {
      const auto updown_it = std::find(
        input.joint_names.begin(), input.joint_names.end(), "updown");
      if (updown_it != input.joint_names.end()) {
        const size_t updown_index = static_cast<size_t>(
          std::distance(input.joint_names.begin(), updown_it));
        if (updown_index < previous.positions.size() &&
            updown_index < current.positions.size()) {
          const double delta = std::abs(
            current.positions[updown_index] - previous.positions[updown_index]);
          required_s = std::max(required_s, delta / max_updown_speed_m_s);
        }
      }
    }

    if (required_s <= 1e-9) {
      required_s = std::max(
        0.0,
        point_time_s(current) - point_time_s(previous));
    }
    elapsed_s += required_s;
    points[point_index].time_from_start = rclcpp::Duration::from_seconds(elapsed_s);
  }

  if (sample_rate_hz <= 1e-9 || elapsed_s <= 1e-9) {
    return output;
  }

  moveit::planning_interface::MoveGroupInterface::Plan sampled = output;
  auto& sampled_trajectory = sampled.trajectory_.joint_trajectory;
  sampled_trajectory.points.clear();
  const double period_s = 1.0 / sample_rate_hz;
  const size_t sample_count = std::max<size_t>(
    2,
    static_cast<size_t>(std::ceil(elapsed_s / period_s)) + 1);
  sampled_trajectory.points.reserve(sample_count);

  for (size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
    const double sample_time_s = sample_index + 1 == sample_count
      ? elapsed_s
      : std::min(elapsed_s, period_s * static_cast<double>(sample_index));
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(sample_time_s);
    point.positions = sample_positions_at_time(
      output.trajectory_.joint_trajectory,
      robot_model,
      sample_time_s);
    sampled_trajectory.points.push_back(std::move(point));
  }

  return sampled;
}

}  // namespace alfa_robot::motion
