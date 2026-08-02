#include "alfa_robot_moveit_config/loaded_pose_planning.hpp"

#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"
#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"
#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <moveit/robot_state/conversions.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace alfa_robot::motion
{

// updown 逻辑/URDF 与电机物理规划范围上限均为 0.7m。负重抬升的
// 目标高度不得超过此上限，否则会命令超出 URDF joint limit 的 updown（原先硬编码 0.99
// 属于旧 [0,0.99] 范围的遗留值）。
constexpr double kUpdownLogicalUpperM = 0.7;

LoadedPoseSelector::LoadedPoseSelector(LoadedPoseSelectorConfig config)
: config_(std::move(config))
{
  if (config_.left_pose_family.empty()) {
    throw std::invalid_argument("LoadedPoseSelector requires at least one left loaded pose");
  }
  if (config_.right_pose_family.empty()) {
    throw std::invalid_argument("LoadedPoseSelector requires at least one right loaded pose");
  }
  config_.left_preferred_index = std::min(config_.left_preferred_index, config_.left_pose_family.size() - 1);
  config_.right_preferred_index = std::min(config_.right_preferred_index, config_.right_pose_family.size() - 1);
}

bool LoadedPoseSelector::hasVariable(const moveit::core::RobotState& state, const std::string& name) const
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
}

std::string LoadedPoseSelector::jointName(const std::string& side, size_t index)
{
  return side + "_joint" + std::to_string(index + 1);
}

double LoadedPoseSelector::armPoseDistance(
  const moveit::core::RobotState& state,
  const std::string& side,
  const std::vector<double>& pose) const
{
  if (pose.size() < 6) return std::numeric_limits<double>::infinity();
  double squared_sum = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const std::string name = jointName(side, i);
    if (!hasVariable(state, name)) {
      return std::numeric_limits<double>::infinity();
    }
    const double diff = shortest_angular_distance(state.getVariablePosition(name), pose[i]);
    squared_sum += diff * diff;
  }
  return std::sqrt(squared_sum);
}

size_t LoadedPoseSelector::nearestPoseIndex(
  const moveit::core::RobotState& state,
  const std::string& side,
  const std::vector<std::vector<double>>& family,
  double* distance) const
{
  size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < family.size(); ++i) {
    const double candidate_distance = armPoseDistance(state, side, family[i]);
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best_index = i;
    }
  }
  if (distance) {
    *distance = best_distance;
  }
  return best_index;
}

std::array<double, 3> LoadedPoseSelector::distanceMetrics(
  const moveit::core::RobotState& state,
  size_t left_index,
  size_t right_index) const
{
  if (left_index >= config_.left_pose_family.size() || right_index >= config_.right_pose_family.size()) {
    return {
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()
    };
  }
  const auto& left_pose = config_.left_pose_family[left_index];
  const auto& right_pose = config_.right_pose_family[right_index];
  double abs_sum = 0.0;
  double squared_sum = 0.0;
  double max_delta = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const std::array<std::pair<std::string, const std::vector<double>*>, 2> arms = {{
      {jointName("left", i), &left_pose},
      {jointName("right", i), &right_pose},
    }};
    for (const auto& [name, pose] : arms) {
      if (!hasVariable(state, name) || pose->size() <= i) {
        return {
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity()
        };
      }
      const double delta = std::abs(shortest_angular_distance(state.getVariablePosition(name), (*pose)[i]));
      abs_sum += delta;
      squared_sum += delta * delta;
      max_delta = std::max(max_delta, delta);
    }
  }
  return {abs_sum, std::sqrt(squared_sum), max_delta};
}

LoadedPoseSelection LoadedPoseSelector::select(const moveit::core::RobotState& state) const
{
  LoadedPoseSelection selection;
  selection.left_index = nearestPoseIndex(
    state, "left", config_.left_pose_family, &selection.left_distance);
  selection.right_index = nearestPoseIndex(
    state, "right", config_.right_pose_family, &selection.right_distance);
  const auto metrics = distanceMetrics(state, selection.left_index, selection.right_index);
  selection.distance_sum = metrics[0];
  selection.distance_l2 = metrics[1];
  selection.max_joint_delta = metrics[2];
  return selection;
}

moveit::core::RobotState LoadedPoseSelector::makeGoalState(
  const moveit::core::RobotState& start_state,
  LoadedPoseSelection* selection) const
{
  moveit::core::RobotState goal_state(start_state);
  LoadedPoseSelection local_selection = select(start_state);
  if (selection) {
    *selection = local_selection;
  }

  const auto& left_pose = config_.left_pose_family[local_selection.left_index];
  const auto& right_pose = config_.right_pose_family[local_selection.right_index];
  if (hasVariable(goal_state, "updown")) {
    const double target_updown = config_.preserve_lower_updown
      ? std::min(start_state.getVariablePosition("updown"), config_.target_updown)
      : config_.target_updown;
    goal_state.setVariablePosition("updown", target_updown);
  }
  for (size_t i = 0; i < left_pose.size(); ++i) {
    goal_state.setVariablePosition(jointName("left", i), left_pose[i]);
  }
  for (size_t i = 0; i < right_pose.size(); ++i) {
    goal_state.setVariablePosition(jointName("right", i), right_pose[i]);
  }
  if (config_.enforce_bounds_group) {
    goal_state.enforceBounds(config_.enforce_bounds_group);
  }
  goal_state.update();
  return goal_state;
}

void LoadedPoseSelector::fillTimingDistanceMetrics(ExtractRolloutTiming& timing) const
{
  if (!timing.final_state) {
    return;
  }

  const auto selection = select(*timing.final_state);
  timing.selected_left_loaded_pose_index = selection.left_index;
  timing.selected_right_loaded_pose_index = selection.right_index;
  timing.selected_left_loaded_pose_distance = selection.left_distance;
  timing.selected_right_loaded_pose_distance = selection.right_distance;
  timing.loaded_pose_distance_sum = selection.distance_sum;
  timing.loaded_pose_distance_l2 = selection.distance_l2;
  timing.loaded_pose_max_joint_delta = selection.max_joint_delta;
}

}  // namespace alfa_robot::motion

#include "robot_motion_scene_service/motion_scene_adapter.hpp"

#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace alfa_robot::motion
{

namespace
{

constexpr double kMaxStageJointSpeedRadS = 20.0 * M_PI / 180.0;

double trajectory_joint_distance(
  const moveit_msgs::msg::RobotTrajectory& trajectory,
  const std::vector<std::string>& target_joint_names)
{
  const auto& joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.size() < 2 || joint_trajectory.joint_names.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  std::vector<size_t> indices;
  indices.reserve(target_joint_names.size());
  for (const auto& target_name : target_joint_names) {
    const auto found = std::find(
      joint_trajectory.joint_names.begin(),
      joint_trajectory.joint_names.end(),
      target_name);
    if (found != joint_trajectory.joint_names.end()) {
      indices.push_back(static_cast<size_t>(
        std::distance(joint_trajectory.joint_names.begin(), found)));
    }
  }
  if (indices.empty()) {
    for (size_t i = 0; i < joint_trajectory.joint_names.size(); ++i) {
      indices.push_back(i);
    }
  }

  double total = 0.0;
  for (size_t point_index = 1; point_index < joint_trajectory.points.size(); ++point_index) {
    const auto& previous = joint_trajectory.points[point_index - 1];
    const auto& current = joint_trajectory.points[point_index];
    for (const size_t index : indices) {
      if (index < previous.positions.size() && index < current.positions.size()) {
        total += std::abs(current.positions[index] - previous.positions[index]);
      }
    }
  }
  return total;
}

std::vector<std::string> touch_links_for_attached_box(const AttachedBoxSpec& box)
{
  std::vector<std::string> links{box.link_name};
  if (box.link_name.rfind("left_", 0) == 0) {
    links.push_back("left_joint6");
    links.push_back("left_joint5");
    links.push_back("left_joint4");
    links.push_back("left_joint3");
  } else if (box.link_name.rfind("right_", 0) == 0) {
    links.push_back("right_joint6");
    links.push_back("right_joint5");
    links.push_back("right_joint4");
    links.push_back("right_joint3");
  }
  return links;
}

void attach_boxes_to_robot_state(
  moveit::core::RobotState& state,
  const std::vector<AttachedBoxSpec>& boxes,
  double collision_padding)
{
  for (const auto& box : boxes) {
    const double size_x = std::max(0.001, box.size[0] + 2.0 * collision_padding);
    const double size_y = std::max(0.001, box.size[1] + 2.0 * collision_padding);
    const double size_z = std::max(0.001, box.size[2] + 2.0 * collision_padding);
    std::vector<shapes::ShapeConstPtr> shapes;
    shapes.push_back(std::make_shared<shapes::Box>(size_x, size_y, size_z));

    EigenSTL::vector_Isometry3d shape_poses;
    Eigen::Isometry3d shape_pose = Eigen::Isometry3d::Identity();
    shape_pose.translation() = Eigen::Vector3d(
      box.center_in_link[0],
      box.center_in_link[1],
      box.center_in_link[2]);
    shape_poses.push_back(shape_pose);

    state.attachBody(
      box.id,
      Eigen::Isometry3d::Identity(),
      shapes,
      shape_poses,
      touch_links_for_attached_box(box),
      box.link_name);
  }
  state.update(true);
}

void append_plan_segment(
  moveit::planning_interface::MoveGroupInterface::Plan& combined,
  const moveit::planning_interface::MoveGroupInterface::Plan& segment)
{
  const auto& input = segment.trajectory_.joint_trajectory;
  if (input.points.empty()) return;
  auto& output = combined.trajectory_.joint_trajectory;
  if (output.joint_names.empty()) output.joint_names = input.joint_names;
  const double output_offset = output.points.empty()
    ? 0.0
    : rclcpp::Duration(output.points.back().time_from_start).seconds();
  const double input_offset = rclcpp::Duration(input.points.front().time_from_start).seconds();
  const size_t start_index = output.points.empty() ? 0 : 1;
  for (size_t i = start_index; i < input.points.size(); ++i) {
    auto point = input.points[i];
    const double local_time = std::max(
      0.0, rclcpp::Duration(point.time_from_start).seconds() - input_offset);
    point.time_from_start = rclcpp::Duration::from_seconds(output_offset + local_time);
    output.points.push_back(std::move(point));
  }
}

geometry_msgs::msg::Pose translated_tip_pose(
  const moveit::core::RobotState& state,
  const std::string& tip_name,
  double delta_x)
{
  const Eigen::Isometry3d& tip = state.getGlobalLinkTransform(tip_name);
  Eigen::Quaterniond orientation(tip.linear());
  orientation.normalize();
  geometry_msgs::msg::Pose pose;
  pose.position.x = tip.translation().x() + delta_x;
  pose.position.y = tip.translation().y();
  pose.position.z = tip.translation().z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

geometry_msgs::msg::Pose tip_position_with_goal_orientation(
  const moveit::core::RobotState& position_state,
  const moveit::core::RobotState& orientation_state,
  const std::string& tip_name)
{
  const Eigen::Isometry3d& position_tip = position_state.getGlobalLinkTransform(tip_name);
  const Eigen::Isometry3d& orientation_tip = orientation_state.getGlobalLinkTransform(tip_name);
  Eigen::Quaterniond orientation(orientation_tip.linear());
  orientation.normalize();
  geometry_msgs::msg::Pose pose;
  pose.position.x = position_tip.translation().x();
  pose.position.y = position_tip.translation().y();
  pose.position.z = position_tip.translation().z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

bool box_uses_top_suction(const AttachedBoxSpec& box)
{
  return box.size[2] > box.size[0];
}

double joint_variable_delta(
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const std::string& name)
{
  const double start = start_state.getVariablePosition(name);
  const double goal = goal_state.getVariablePosition(name);
  const auto* variable_joint = start_state.getRobotModel()->getJointOfVariable(name);
  const auto* revolute_joint =
    dynamic_cast<const moveit::core::RevoluteJointModel*>(variable_joint);
  const bool continuous_variable = revolute_joint && revolute_joint->isContinuous();
  return continuous_variable
    ? std::atan2(std::sin(goal - start), std::cos(goal - start))
    : (goal - start);
}

moveit::planning_interface::MoveGroupInterface::Plan make_interpolated_joint_plan(
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const std::vector<std::string>& joint_names,
  double duration_s)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto& trajectory = plan.trajectory_.joint_trajectory;
  trajectory.joint_names = joint_names;
  double max_delta = 0.0;
  for (const auto& name : trajectory.joint_names) {
    const double delta = joint_variable_delta(start_state, goal_state, name);
    const double limit = name == "updown" ? 0.01 : (5.0 * M_PI / 180.0);
    max_delta = std::max(max_delta, std::abs(delta) / limit);
  }

  const size_t steps = std::max<size_t>(2, static_cast<size_t>(std::ceil(max_delta)) + 1);
  trajectory.points.reserve(steps);
  for (size_t step_index = 0; step_index < steps; ++step_index) {
    const double ratio = steps <= 1 ? 1.0 : static_cast<double>(step_index) / static_cast<double>(steps - 1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s * ratio);
    point.positions.reserve(trajectory.joint_names.size());
    for (const auto& name : trajectory.joint_names) {
      const double start = start_state.getVariablePosition(name);
      point.positions.push_back(start + joint_variable_delta(start_state, goal_state, name) * ratio);
    }
    trajectory.points.push_back(std::move(point));
  }
  moveit::core::robotStateToRobotStateMsg(start_state, plan.start_state_, true);
  plan.planning_time_ = 0.0;
  return plan;
}

double plan_duration_s(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  const auto& points = plan.trajectory_.joint_trajectory.points;
  return points.empty() ? 0.0 : rclcpp::Duration(points.back().time_from_start).seconds();
}

moveit::core::RobotState state_from_plan_at_time(
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const moveit::core::RobotState& reference_state,
  double time_s)
{
  moveit::core::RobotState state(reference_state);
  const auto& trajectory = plan.trajectory_.joint_trajectory;
  const auto& points = trajectory.points;
  if (trajectory.joint_names.empty() || points.empty()) {
    state.update(true);
    return state;
  }

  size_t upper = 0;
  while (upper + 1 < points.size() &&
         rclcpp::Duration(points[upper].time_from_start).seconds() < time_s) {
    ++upper;
  }
  const size_t lower = upper == 0 ? 0 : upper - 1;
  const double lower_t = rclcpp::Duration(points[lower].time_from_start).seconds();
  const double upper_t = rclcpp::Duration(points[upper].time_from_start).seconds();
  const double ratio = upper_t > lower_t + 1e-9
    ? std::clamp((time_s - lower_t) / (upper_t - lower_t), 0.0, 1.0)
    : 0.0;
  for (size_t index = 0; index < trajectory.joint_names.size(); ++index) {
    if (index >= points[lower].positions.size() || index >= points[upper].positions.size()) continue;
    const std::string& name = trajectory.joint_names[index];
    const double from = points[lower].positions[index];
    const double to = points[upper].positions[index];
    const auto* variable_joint = state.getRobotModel()->getJointOfVariable(name);
    const auto* revolute_joint =
      dynamic_cast<const moveit::core::RevoluteJointModel*>(variable_joint);
    const double delta = revolute_joint && revolute_joint->isContinuous()
      ? std::atan2(std::sin(to - from), std::cos(to - from))
      : to - from;
    state.setVariablePosition(name, from + delta * ratio);
  }
  state.update(true);
  return state;
}

void copy_arm_goal(
  const std::string& side,
  const moveit::core::RobotState& goal_state,
  moveit::core::RobotState& state)
{
  for (size_t joint_index = 1; joint_index <= 6; ++joint_index) {
    const std::string joint_name = side + "_joint" + std::to_string(joint_index);
    state.setVariablePosition(joint_name, goal_state.getVariablePosition(joint_name));
  }
  state.enforceBounds();
  state.update(true);
}

moveit::planning_interface::MoveGroupInterface::Plan merge_parallel_arm_plans(
  const moveit::core::RobotState& start_state,
  const moveit::planning_interface::MoveGroupInterface::Plan& left_plan,
  const moveit::planning_interface::MoveGroupInterface::Plan& right_plan,
  const std::vector<std::string>& joint_names)
{
  moveit::planning_interface::MoveGroupInterface::Plan merged;
  auto& trajectory = merged.trajectory_.joint_trajectory;
  trajectory.joint_names = joint_names;
  const double duration_s = std::max(plan_duration_s(left_plan), plan_duration_s(right_plan));
  constexpr double kSampleStepS = 0.05;
  const size_t steps = std::max<size_t>(
    2, static_cast<size_t>(std::ceil(duration_s / kSampleStepS)) + 1);
  trajectory.points.reserve(steps);
  for (size_t step = 0; step < steps; ++step) {
    const double ratio = steps <= 1 ? 1.0 : static_cast<double>(step) / static_cast<double>(steps - 1);
    const double time_s = duration_s * ratio;
    const auto left_state = state_from_plan_at_time(left_plan, start_state, time_s);
    const auto right_state = state_from_plan_at_time(right_plan, start_state, time_s);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(time_s);
    point.positions.reserve(joint_names.size());
    for (const auto& name : joint_names) {
      if (name.rfind("left_joint", 0) == 0) {
        point.positions.push_back(left_state.getVariablePosition(name));
      } else if (name.rfind("right_joint", 0) == 0) {
        point.positions.push_back(right_state.getVariablePosition(name));
      } else {
        point.positions.push_back(start_state.getVariablePosition(name));
      }
    }
    trajectory.points.push_back(std::move(point));
  }
  moveit::core::robotStateToRobotStateMsg(start_state, merged.start_state_, true);
  merged.planning_time_ = std::max(left_plan.planning_time_, right_plan.planning_time_);
  return merged;
}

ExtractMonitorTransitionPlanner make_loaded_transition_planner(
  const LoadedPosePlannerConfig& config,
  const std::string& stage_name,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const LoadedPlanCancellationCheck& is_cancelled)
{
  ExtractMonitorTransitionPlanner repair_planner;
  repair_planner.is_cancelled = is_cancelled;
  repair_planner.make_interpolated_plan =
    [&config](const auto& start, const auto& goal, double duration_s) {
      return make_interpolated_joint_plan(
        start, goal, config.target_joint_names, duration_s);
    };
  repair_planner.densify_plan = [](const auto& candidate) {
    return candidate;
  };
  repair_planner.validate_plan =
    [&config, &carried_boxes](const auto& candidate, const auto& start, std::string* reason) {
    return config.clearance_callback
      ? config.clearance_callback(candidate, start, carried_boxes, reason)
      : true;
  };
  repair_planner.direct_plan =
    [&config, &stage_name, &carried_boxes](
      const auto& start,
      const auto& goal,
      auto* repaired,
      std::string* reason) {
      if (!config.direct_plan_callback) {
        if (reason) {
          *reason = "loaded_direct_plan_callback_not_available";
        }
        return false;
      }
      return config.direct_plan_callback(
        stage_name + "/shortcut_local_rrt_patch", start, goal, carried_boxes, repaired, reason);
    };
  return repair_planner;
}

bool plan_loaded_synchronized_transition(
  const moveit::core::RobotState& loaded_start_state,
  const moveit::core::RobotState& goal_state,
  ExtractMonitorTransitionPlanner& repair_planner,
  moveit::planning_interface::MoveGroupInterface::Plan* plan,
  std::string* reason)
{
  const auto transition = repair_planner.plan(loaded_start_state, goal_state);
  if (plan) *plan = transition.plan;
  if (!transition.valid) {
    if (reason) *reason = "synchronized_loaded_transition_failed: " + transition.failure_reason;
    return false;
  }
  if (reason) reason->clear();
  return true;
}

bool plan_loaded_staged_fallback(
  const moveit::core::RobotState& loaded_start_state,
  const moveit::core::RobotState& goal_state,
  const LoadedPosePlannerConfig& config,
  ExtractMonitorTransitionPlanner& repair_planner,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  moveit::planning_interface::MoveGroupInterface::Plan* plan,
  std::string* reason)
{
  moveit::core::RobotState arm_goal_state(goal_state);
  arm_goal_state.setVariablePosition(
    "updown", loaded_start_state.getVariablePosition("updown"));
  arm_goal_state.enforceBounds();
  arm_goal_state.update(true);

  moveit::core::RobotState left_goal_state(loaded_start_state);
  copy_arm_goal("left", arm_goal_state, left_goal_state);
  moveit::core::RobotState right_goal_state(loaded_start_state);
  copy_arm_goal("right", arm_goal_state, right_goal_state);
  const auto left_transition = repair_planner.plan(loaded_start_state, left_goal_state);
  if (!left_transition.valid) {
    if (reason) *reason = "staged_left_loaded_arm_transition_failed: " + left_transition.failure_reason;
    return false;
  }
  const auto right_transition = repair_planner.plan(loaded_start_state, right_goal_state);
  if (!right_transition.valid) {
    if (reason) *reason = "staged_right_loaded_arm_transition_failed: " + right_transition.failure_reason;
    return false;
  }
  auto combined = merge_parallel_arm_plans(
    loaded_start_state, left_transition.plan, right_transition.plan, config.target_joint_names);
  std::string arm_reason;
  if (config.clearance_callback &&
      !config.clearance_callback(combined, loaded_start_state, carried_boxes, &arm_reason)) {
    if (reason) *reason = "staged_parallel_loaded_arm_collision: " + arm_reason;
    return false;
  }
  const auto updown_transition = repair_planner.plan(arm_goal_state, goal_state);
  if (!updown_transition.valid) {
    if (reason) *reason = "staged_loaded_updown_transition_failed: " + updown_transition.failure_reason;
    return false;
  }
  append_plan_segment(combined, updown_transition.plan);
  std::string combined_reason;
  if (config.clearance_callback &&
      !config.clearance_callback(combined, loaded_start_state, carried_boxes, &combined_reason)) {
    if (reason) *reason = "staged_loaded_plan_collision: " + combined_reason;
    return false;
  }
  if (plan) *plan = std::move(combined);
  if (reason) reason->clear();
  return true;
}

}  // namespace

LoadedPosePlanner::LoadedPosePlanner(LoadedPosePlannerConfig config)
: config_(std::move(config))
{}

double LoadedPosePlanner::currentUpdown(const moveit::core::RobotState& state)
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  if (std::find(variable_names.begin(), variable_names.end(), "updown") == variable_names.end()) {
    return 0.0;
  }
  return state.getVariablePosition("updown");
}

int LoadedPosePlanner::boxColumn(const AttachedBoxSpec& box)
{
  return box_column_from_left(boxId(box));
}

int LoadedPosePlanner::boxId(const AttachedBoxSpec& box)
{
  const auto last_underscore = box.id.find_last_of('_');
  if (last_underscore == std::string::npos || last_underscore + 1 >= box.id.size()) {
    return 0;
  }
  try {
    return std::stoi(box.id.substr(last_underscore + 1));
  } catch (const std::exception&) {
    return 0;
  }
}

bool LoadedPosePlanner::planLateralShift(
  const std::string& stage_name,
  const moveit::core::RobotState& start_state,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  moveit::core::RobotState* shifted_state,
  LoadedPosePlanResult* result)
{
  if (!shifted_state || !result) return false;
  if (!config_.lateral_shift_enabled || config_.lateral_shift_distance <= 1e-6) {
    *shifted_state = start_state;
    return true;
  }

  const AttachedBoxSpec* center_box = nullptr;
  const int shift_column =
    std::max(1, std::min(kBoxStackColumnCount, config_.lateral_shift_column));
  for (const auto& box : carried_boxes) {
    if (boxColumn(box) == shift_column) {
      center_box = &box;
      break;
    }
  }
  if (!center_box) {
    *shifted_state = start_state;
    return true;
  }

  const std::string side = center_box->link_name.rfind("left_", 0) == 0 ? "left" : "right";
  const std::string tip_name = center_box->link_name;
  const std::string group_name = side + "_arm";
  const moveit::core::JointModelGroup* arm_group =
    start_state.getRobotModel()->getJointModelGroup(group_name);
  if (!arm_group) {
    result->failure_reason = "lateral_shift_missing_group_" + group_name;
    return false;
  }

  const double direction_y = side == "left" ? 1.0 : -1.0;
  const double distance = std::abs(config_.lateral_shift_distance);
  const double step = std::max(0.001, std::abs(config_.lateral_shift_step));
  const size_t step_count = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / step)));

  moveit::core::RobotState current_state(start_state);
  const auto t0 = std::chrono::steady_clock::now();
  const auto finish_partial = [&](bool shifted_any) {
    result->lateral_shift_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    if (shifted_any) {
      result->lateral_shift_success = true;
      *shifted_state = current_state;
      return true;
    }
    return false;
  };

  for (size_t step_index = 1; step_index <= step_count; ++step_index) {
    const double shift = std::min(distance, step * static_cast<double>(step_index));
    const double step_delta = shift - result->lateral_shift_reached_distance;
    const Eigen::Isometry3d start_tip = current_state.getGlobalLinkTransform(tip_name);
    Eigen::Isometry3d target_tip = start_tip;
    target_tip.translation().y() += direction_y * step_delta;

    if (!config_.lateral_shift_solver) {
      result->failure_reason = "lateral_shift_solver_not_initialized";
      return finish_partial(result->lateral_shift_reached_distance > 1e-6);
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_tip.translation().x();
    target_pose.position.y = target_tip.translation().y();
    target_pose.position.z = target_tip.translation().z();
    Eigen::Quaterniond q(target_tip.linear());
    q.normalize();
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    ExtractCandidateSolveRequest request;
    request.side = side;
    request.current_state = &current_state;
    request.target_pose = target_pose;
    request.step_index = step_index;
    request.candidate_index = 0;
    request.retreat_x = 0.0;
    request.retreat_delta_x = 0.0;
    request.lift_z = 0.0;
    request.lift_delta_z = 0.0;
    request.pitch_up_rad = 0.0;
    request.pitch_delta_rad = 0.0;
    request.min_allowed_tip_z = start_tip.translation().z();
    request.fixed_updown = currentUpdown(current_state);

    ExtractCandidate candidate;
    if (!config_.lateral_shift_solver->solve(request, &candidate) || !candidate.state) {
      result->failure_reason = "lateral_shift_analytic_failed_step_" + std::to_string(step_index);
      if (!candidate.rejection_reason.empty()) {
        result->failure_reason += ": " + candidate.rejection_reason;
      }
      return finish_partial(result->lateral_shift_reached_distance > 1e-6);
    }

    moveit::core::RobotState target_state(*candidate.state);
    target_state.setVariablePosition("updown", currentUpdown(current_state));
    target_state.enforceBounds(arm_group);
    target_state.update();

    moveit::core::RobotState planning_start(current_state);
    moveit::core::RobotState planning_goal(target_state);
    attach_boxes_to_robot_state(planning_start, carried_boxes, config_.attached_box_collision_padding);
    attach_boxes_to_robot_state(planning_goal, carried_boxes, config_.attached_box_collision_padding);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto& trajectory = plan.trajectory_.joint_trajectory;
    trajectory.joint_names = config_.target_joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    trajectory_msgs::msg::JointTrajectoryPoint goal_point;
    start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    goal_point.time_from_start = rclcpp::Duration::from_seconds(0.1 * static_cast<double>(step_index));
    start_point.positions.reserve(trajectory.joint_names.size());
    goal_point.positions.reserve(trajectory.joint_names.size());
    for (const auto& name : trajectory.joint_names) {
      start_point.positions.push_back(current_state.getVariablePosition(name));
      goal_point.positions.push_back(target_state.getVariablePosition(name));
    }
    trajectory.points.push_back(start_point);
    trajectory.points.push_back(goal_point);
    plan = retime_plan_by_max_joint_speed(
      plan,
      current_state.getRobotModel(),
      kMaxStageJointSpeedRadS);

    result->lateral_shift_attempted = true;
    result->lateral_shift_points += trajectory.points.size();

    std::string clearance_reason;
    const bool clear = config_.clearance_callback
      ? config_.clearance_callback(plan, planning_start, carried_boxes, &clearance_reason)
      : true;
    if (!clear) {
      result->failure_reason =
        "lateral_shift_analytic_collision_step_" + std::to_string(step_index) + ": " + clearance_reason;
      return finish_partial(result->lateral_shift_reached_distance > 1e-6);
    }

    if (config_.record_callback) {
      nlohmann::json extra = {
        {"stage_kind", "post_extract_lateral_shift"},
        {"valid", true},
        {"shift_side", side},
        {"shift_step", step_index},
        {"shift_step_count", step_count},
        {"shift_distance_y", direction_y * shift},
        {"shift_method", "analytic_step"},
        {"shift_step_resolution", step},
        {"shift_joint_delta_limit", config_.max_joint_delta},
        {"target_box", center_box->id},
        {"carried_box_count", carried_boxes.size()}
      };
      LoadedPoseReplayStage replay_stage;
      replay_stage.stage_name = stage_name + "/lateral_shift_step_" + std::to_string(step_index);
      replay_stage.plan = plan;
      replay_stage.start_state = std::make_shared<moveit::core::RobotState>(planning_start);
      replay_stage.goal_state = std::make_shared<moveit::core::RobotState>(planning_goal);
      replay_stage.extra = extra;
      result->lateral_shift_replay_stages.push_back(std::move(replay_stage));
      config_.record_callback(
        stage_name + "/lateral_shift_step_" + std::to_string(step_index),
        plan,
        planning_start,
        planning_goal,
        config_.target_joint_names,
        extra);
    }

    current_state = target_state;
    result->lateral_shift_reached_distance = shift;
  }

  return finish_partial(true);
}

LoadedPosePlanResult LoadedPosePlanner::plan(
  const std::string& stage_name,
  const moveit::core::RobotState& extract_state,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  size_t loaded_plan_rank)
{
  return planInternal(
    stage_name, extract_state, carried_boxes, loaded_plan_rank, true,
    LoadedPlanCancellationCheck{});
}

LoadedPosePlanResult LoadedPosePlanner::planInternal(
  const std::string& stage_name,
  const moveit::core::RobotState& extract_state,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  size_t loaded_plan_rank,
  bool manage_scene_adapter,
  const LoadedPlanCancellationCheck& is_cancelled)
{
  LoadedPosePlanResult result;
  const auto cancelled = [&]() {
    return is_cancelled && is_cancelled();
  };
  if (cancelled()) {
    result.failure_reason = "loaded_plan_cancelled_after_first_success";
    return result;
  }
  if (!config_.move_group && !config_.direct_plan_callback) {
    result.failure_reason = "loaded_move_group_not_initialized";
    return result;
  }
  if (!config_.selector) {
    result.failure_reason = "loaded_pose_selector_not_initialized";
    return result;
  }
  if (!config_.scene_adapter) {
    result.failure_reason = "motion_scene_adapter_not_initialized";
    return result;
  }

  std::vector<AttachedBoxSpec> saved_boxes;
  if (manage_scene_adapter) {
    saved_boxes = config_.scene_adapter->activeAttachedBoxes();
    config_.scene_adapter->setActiveAttachedBoxesForRecordOnly(carried_boxes);
  }

  auto restore_boxes = [&]() {
    if (!manage_scene_adapter) {
      return;
    }
    std::vector<std::string> ids;
    ids.reserve(carried_boxes.size());
    for (const auto& box : carried_boxes) {
      ids.push_back(box.id);
    }
    config_.scene_adapter->removeCarriedBoxIds(ids);
    config_.scene_adapter->setActiveAttachedBoxesForRecordOnly(saved_boxes);
  };

  const auto cancel_result = [&]() {
    result.failure_reason = "loaded_plan_cancelled_after_first_success";
    restore_boxes();
    return result;
  };

  if (manage_scene_adapter &&
      !config_.scene_adapter->applyAttachedBoxState(
        config_.scene_adapter->activeAttachedBoxes(),
        moveit_msgs::msg::CollisionObject::ADD)) {
    result.failure_reason = "failed_to_attach_box_for_loaded_plan";
    restore_boxes();
    return result;
  }

  if (cancelled()) {
    return cancel_result();
  }

  moveit::core::RobotState start_state_with_boxes(extract_state);
  attach_boxes_to_robot_state(
    start_state_with_boxes,
    carried_boxes,
    config_.attached_box_collision_padding);

  moveit::core::RobotState loaded_start_state(start_state_with_boxes);
  moveit::core::RobotState shifted_state(extract_state);
  const bool apply_pre_loaded_lower =
    config_.pre_loaded_lower_updown_delta > 1e-6 &&
    carried_boxes.size() == 2 &&
    boxId(carried_boxes[0]) == config_.pre_loaded_lower_left_box_id &&
    boxId(carried_boxes[1]) == config_.pre_loaded_lower_right_box_id;
  {
    std::lock_guard<std::mutex> lock(record_mutex_);
    if (cancelled()) {
      return cancel_result();
    }
    if (!planLateralShift(stage_name, shifted_state, carried_boxes, &shifted_state, &result)) {
      restore_boxes();
      return result;
    }
  }
  if (cancelled()) {
    return cancel_result();
  }
  if (apply_pre_loaded_lower) {
    const moveit::core::RobotState pre_lower_start(shifted_state);
    shifted_state.setVariablePosition(
      "updown",
      currentUpdown(pre_lower_start) - config_.pre_loaded_lower_updown_delta);
    shifted_state.enforceBounds();
    shifted_state.update();
    if (config_.record_callback) {
      moveit::core::RobotState planning_start(pre_lower_start);
      moveit::core::RobotState planning_goal(shifted_state);
      attach_boxes_to_robot_state(planning_start, carried_boxes, config_.attached_box_collision_padding);
      attach_boxes_to_robot_state(planning_goal, carried_boxes, config_.attached_box_collision_padding);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto& trajectory = plan.trajectory_.joint_trajectory;
      trajectory.joint_names = config_.target_joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint start_point;
      trajectory_msgs::msg::JointTrajectoryPoint goal_point;
      start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
      goal_point.time_from_start = rclcpp::Duration::from_seconds(0.1);
      start_point.positions.reserve(trajectory.joint_names.size());
      goal_point.positions.reserve(trajectory.joint_names.size());
      for (const auto& name : trajectory.joint_names) {
        start_point.positions.push_back(pre_lower_start.getVariablePosition(name));
        goal_point.positions.push_back(shifted_state.getVariablePosition(name));
      }
      trajectory.points.push_back(start_point);
      trajectory.points.push_back(goal_point);
      plan = retime_plan_by_max_joint_speed(
        plan,
        pre_lower_start.getRobotModel(),
        kMaxStageJointSpeedRadS);

      nlohmann::json extra = {
        {"stage_kind", "pre_loaded_lower_updown"},
        {"valid", true},
        {"lower_left_box_id", config_.pre_loaded_lower_left_box_id},
        {"lower_right_box_id", config_.pre_loaded_lower_right_box_id},
        {"updown_delta", -config_.pre_loaded_lower_updown_delta},
        {"start_updown", currentUpdown(pre_lower_start)},
        {"target_updown", currentUpdown(shifted_state)}
      };
      LoadedPoseReplayStage replay_stage;
      replay_stage.stage_name = stage_name + "/pre_loaded_lower_updown";
      replay_stage.plan = plan;
      replay_stage.start_state = std::make_shared<moveit::core::RobotState>(planning_start);
      replay_stage.goal_state = std::make_shared<moveit::core::RobotState>(planning_goal);
      replay_stage.extra = extra;
      result.lateral_shift_replay_stages.push_back(std::move(replay_stage));
      config_.record_callback(
        stage_name + "/pre_loaded_lower_updown",
        plan,
        planning_start,
        planning_goal,
      config_.target_joint_names,
      extra);
    }
  }
  attach_boxes_to_robot_state(
    shifted_state,
    carried_boxes,
    config_.attached_box_collision_padding);
  loaded_start_state = shifted_state;
  const moveit::core::RobotState validation_start_state(loaded_start_state);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto set_result_plan = [&](const moveit::planning_interface::MoveGroupInterface::Plan& segment) {
    plan = retime_plan_by_max_joint_speed(
      segment,
      validation_start_state.getRobotModel(),
      kMaxStageJointSpeedRadS);
    result.plan = plan;
    result.plan_points = plan.trajectory_.joint_trajectory.points.size();
    result.trajectory_joint_distance = trajectory_joint_distance(
      plan.trajectory_, config_.target_joint_names);
    result.start_state = std::make_shared<moveit::core::RobotState>(validation_start_state);
  };

  moveit::core::RobotState goal_state = config_.selector->makeGoalState(
    loaded_start_state, &result.selection);

  const auto t0 = std::chrono::steady_clock::now();
  moveit::core::MoveItErrorCode plan_result = moveit::core::MoveItErrorCode::FAILURE;
  std::string direct_failure_reason;
  if (config_.planning_mode == "shortcut") {
    ExtractMonitorTransitionPlanner repair_planner =
      make_loaded_transition_planner(config_, stage_name, carried_boxes, is_cancelled);
    bool shortcut_ok = plan_loaded_synchronized_transition(
      loaded_start_state,
      goal_state,
      repair_planner,
      &plan,
      &direct_failure_reason);
    if (!shortcut_ok) {
      const auto synchronized_failed_plan = plan;
      const std::string synchronized_failure = direct_failure_reason;
      std::string staged_failure;
      shortcut_ok = plan_loaded_staged_fallback(
        loaded_start_state,
        goal_state,
        config_,
        repair_planner,
        carried_boxes,
        &plan,
        &staged_failure);
      if (!shortcut_ok) {
        plan = synchronized_failed_plan;
        direct_failure_reason = synchronized_failure;
        if (!staged_failure.empty()) {
          direct_failure_reason += "; " + staged_failure;
        }
      }
    }
    plan_result = shortcut_ok
      ? moveit::core::MoveItErrorCode::SUCCESS
      : moveit::core::MoveItErrorCode::FAILURE;
  } else if (config_.direct_plan_callback) {
    const bool direct_ok = config_.direct_plan_callback(
      stage_name, loaded_start_state, goal_state, carried_boxes, &plan, &direct_failure_reason);
    plan_result = direct_ok
      ? moveit::core::MoveItErrorCode::SUCCESS
      : moveit::core::MoveItErrorCode::FAILURE;
  } else {
    config_.move_group->setStartState(loaded_start_state);
    config_.move_group->setJointValueTarget(goal_state);
    plan_result = config_.move_group->plan(plan);
  }
  const auto t1 = std::chrono::steady_clock::now();

  if (cancelled()) {
    return cancel_result();
  }

  result.attempted = true;
  result.plan_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  result.plan_points = plan.trajectory_.joint_trajectory.points.size();
  result.trajectory_joint_distance = trajectory_joint_distance(
    plan.trajectory_, config_.target_joint_names);
  set_result_plan(plan);
  result.goal_state = std::make_shared<moveit::core::RobotState>(goal_state);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    result.failure_reason = direct_failure_reason.empty()
      ? "moveit_planning_failed_code_" + std::to_string(plan_result.val)
      : direct_failure_reason;
    restore_boxes();
    return result;
  }

  std::string carried_collision_reason;
  result.carried_clear = config_.clearance_callback
    ? config_.clearance_callback(plan, validation_start_state, carried_boxes, &carried_collision_reason)
    : true;
  if (!result.carried_clear) {
    result.failure_reason = carried_collision_reason;
    if (config_.planning_mode == "shortcut" && config_.direct_plan_callback) {
      const auto repair_start = std::chrono::steady_clock::now();
      ExtractMonitorTransitionPlanner repair_planner =
        make_loaded_transition_planner(config_, stage_name, carried_boxes, is_cancelled);

      const bool rear_guard_collision =
        carried_collision_reason.find("rear guard") != std::string::npos;
      const ExtractMonitorTransitionPlanResult repaired =
        repair_planner.plan(loaded_start_state, goal_state);
      if (repaired.valid) {
        set_result_plan(repaired.plan);
        result.carried_clear = true;
        carried_collision_reason.clear();
        result.failure_reason.clear();
      } else {
        bool rear_guard_repaired = false;
        std::string rear_guard_failure;
        const bool carried_top_suction = std::any_of(
          carried_boxes.begin(), carried_boxes.end(), box_uses_top_suction);
        const auto& loaded_variable_names = loaded_start_state.getRobotModel()->getVariableNames();
        const bool has_updown_variable = std::find(
          loaded_variable_names.begin(), loaded_variable_names.end(), "updown") !=
          loaded_variable_names.end();
        if (rear_guard_collision && carried_top_suction &&
            has_updown_variable) {
          constexpr double kDeterministicLiftDistance = 0.36;
          const double lifted_updown = std::min(
            kUpdownLogicalUpperM,
            std::max(currentUpdown(loaded_start_state), currentUpdown(goal_state)) +
              kDeterministicLiftDistance);

          moveit::core::RobotState lifted_state(loaded_start_state);
          lifted_state.setVariablePosition("updown", lifted_updown);
          lifted_state.enforceBounds();
          lifted_state.update(true);

          moveit::core::RobotState lifted_goal_state(goal_state);
          lifted_goal_state.setVariablePosition("updown", lifted_updown);
          lifted_goal_state.enforceBounds();
          lifted_goal_state.update(true);

          auto lift_segment = make_interpolated_joint_plan(
            loaded_start_state, lifted_state, config_.target_joint_names, 0.4);
          std::string lift_reason;
          if (config_.clearance_callback &&
              !config_.clearance_callback(lift_segment, loaded_start_state, carried_boxes, &lift_reason)) {
            rear_guard_failure = "rear_guard_top_lift_failed: " + lift_reason;
          } else {
            auto plan_side_order =
              [&](const std::string& first_side, const std::string& second_side,
                  moveit::planning_interface::MoveGroupInterface::Plan* side_plan,
                  std::string* failure_reason) {
                moveit::core::RobotState intermediate(lifted_state);
                for (size_t joint_index = 1; joint_index <= 6; ++joint_index) {
                  const std::string joint_name =
                    first_side + "_joint" + std::to_string(joint_index);
                  intermediate.setVariablePosition(
                    joint_name, lifted_goal_state.getVariablePosition(joint_name));
                }
                intermediate.enforceBounds();
                intermediate.update(true);

                const auto first_transition = repair_planner.plan(lifted_state, intermediate);
                if (!first_transition.valid) {
                  if (failure_reason) {
                    *failure_reason = first_side + "_top_lift_transition_failed: " +
                      first_transition.failure_reason;
                  }
                  return false;
                }

                const auto second_transition = repair_planner.plan(intermediate, lifted_goal_state);
                if (!second_transition.valid) {
                  if (failure_reason) {
                    *failure_reason = second_side + "_top_lift_transition_failed: " +
                      second_transition.failure_reason;
                  }
                  return false;
                }

                auto combined_side_plan = first_transition.plan;
                append_plan_segment(combined_side_plan, second_transition.plan);
                if (side_plan) {
                  *side_plan = std::move(combined_side_plan);
                }
                return true;
              };

            moveit::planning_interface::MoveGroupInterface::Plan arm_transition_plan;
            std::string arm_transition_failure;
            const bool arm_transition_ok =
              plan_side_order("left", "right", &arm_transition_plan, &arm_transition_failure) ||
              plan_side_order("right", "left", &arm_transition_plan, &arm_transition_failure);
            if (!arm_transition_ok) {
              rear_guard_failure = "rear_guard_top_lift_arm_transition_failed: " +
                arm_transition_failure;
            } else {
              auto lower_segment = make_interpolated_joint_plan(
                lifted_goal_state, goal_state, config_.target_joint_names, 0.4);
              std::string lower_reason;
              if (config_.clearance_callback &&
                  !config_.clearance_callback(lower_segment, lifted_goal_state, carried_boxes, &lower_reason)) {
                rear_guard_failure = "rear_guard_top_lift_lower_failed: " + lower_reason;
              } else {
                auto lifted_plan = lift_segment;
                append_plan_segment(lifted_plan, arm_transition_plan);
                append_plan_segment(lifted_plan, lower_segment);
                std::string lifted_reason;
                if (config_.clearance_callback &&
                    !config_.clearance_callback(lifted_plan, loaded_start_state, carried_boxes, &lifted_reason)) {
                  rear_guard_failure = "rear_guard_top_lift_validation_failed: " + lifted_reason;
                } else {
                  set_result_plan(lifted_plan);
                  result.carried_clear = true;
                  carried_collision_reason.clear();
                  result.failure_reason.clear();
                  rear_guard_repaired = true;
                }
              }
            }
          }
        }
        if (rear_guard_collision &&
            !rear_guard_repaired && config_.lateral_shift_solver && carried_boxes.size() == 2) {
          moveit::core::RobotState retreat_state(loaded_start_state);
          moveit::planning_interface::MoveGroupInterface::Plan retreat_prefix;
          moveit::core::robotStateToRobotStateMsg(
            loaded_start_state, retreat_prefix.start_state_, true);
          constexpr double kRetreatStep = 0.02;
          constexpr size_t kRetreatSteps = 20;
          bool retreat_ready = false;
          for (size_t retreat_step = 1; retreat_step <= kRetreatSteps; ++retreat_step) {
            if (cancelled()) {
              return cancel_result();
            }
            moveit::core::RobotState next_state(retreat_state);
            bool solve_ok = true;
            for (const auto& box : carried_boxes) {
              const std::string side = box.link_name.rfind("left_", 0) == 0 ? "left" : "right";
              ExtractCandidateSolveRequest request;
              request.side = side;
              request.current_state = &next_state;
              request.target_pose = translated_tip_pose(next_state, box.link_name, -kRetreatStep);
              request.step_index = retreat_step;
              request.candidate_index = 0;
              request.retreat_x = kRetreatStep * static_cast<double>(retreat_step);
              request.retreat_delta_x = kRetreatStep;
              request.min_allowed_tip_z = next_state.getGlobalLinkTransform(box.link_name).translation().z();
              request.fixed_updown = currentUpdown(next_state);
              request.top_suction = box_uses_top_suction(box);
              ExtractCandidate candidate;
              if (!config_.lateral_shift_solver->solve(request, &candidate) || !candidate.state) {
                solve_ok = false;
                rear_guard_failure = "rear_guard_retreat_ik_failed_step_" +
                  std::to_string(retreat_step) + "_" + side;
                break;
              }
              next_state = *candidate.state;
              next_state.setVariablePosition("updown", currentUpdown(retreat_state));
              next_state.update(true);
            }
            if (!solve_ok) break;

            auto retreat_segment = make_interpolated_joint_plan(
              retreat_state, next_state, config_.target_joint_names, 0.1);
            std::string retreat_clearance_reason;
            if (config_.clearance_callback &&
                !config_.clearance_callback(
                  retreat_segment, retreat_state, carried_boxes, &retreat_clearance_reason)) {
              rear_guard_failure = "rear_guard_retreat_collision_step_" +
                std::to_string(retreat_step) + ": " + retreat_clearance_reason;
              break;
            }
            append_plan_segment(retreat_prefix, retreat_segment);
            retreat_state = next_state;

            moveit::core::RobotState posture_state(retreat_state);
            bool posture_solved = true;
            for (const auto& box : carried_boxes) {
              const std::string side = box.link_name.rfind("left_", 0) == 0 ? "left" : "right";
              ExtractCandidateSolveRequest posture_request;
              posture_request.side = side;
              posture_request.current_state = &posture_state;
              posture_request.target_pose = tip_position_with_goal_orientation(
                posture_state, goal_state, box.link_name);
              posture_request.step_index = retreat_step;
              posture_request.candidate_index = 1;
              posture_request.min_allowed_tip_z =
                posture_state.getGlobalLinkTransform(box.link_name).translation().z();
              posture_request.fixed_updown = currentUpdown(posture_state);
              posture_request.top_suction = box_uses_top_suction(box);
              ExtractCandidate posture_candidate;
              if (!config_.lateral_shift_solver->solve(
                    posture_request, &posture_candidate) || !posture_candidate.state) {
                posture_solved = false;
                break;
              }
              posture_state = *posture_candidate.state;
              posture_state.setVariablePosition("updown", currentUpdown(retreat_state));
              posture_state.update(true);
            }
            if (posture_solved) {
              auto posture_segment = make_interpolated_joint_plan(
                retreat_state, posture_state, config_.target_joint_names, 0.5);
              auto final_segment = make_interpolated_joint_plan(
                posture_state, goal_state, config_.target_joint_names, 0.5);
              std::string posture_reason;
              const bool posture_clear = !config_.clearance_callback ||
                (config_.clearance_callback(posture_segment, retreat_state, carried_boxes, &posture_reason) &&
                 config_.clearance_callback(final_segment, posture_state, carried_boxes, &posture_reason));
              if (posture_clear) {
                append_plan_segment(retreat_prefix, posture_segment);
                append_plan_segment(retreat_prefix, final_segment);
                retreat_ready = true;
                break;
              }
              rear_guard_failure = posture_reason;
            }

            auto remaining_shortcut = make_interpolated_joint_plan(
              retreat_state, goal_state, config_.target_joint_names, 1.0);
            std::string remaining_reason;
            if (!config_.clearance_callback ||
                config_.clearance_callback(remaining_shortcut, retreat_state, carried_boxes, &remaining_reason)) {
              append_plan_segment(retreat_prefix, remaining_shortcut);
              retreat_ready = true;
              break;
            }
            rear_guard_failure = remaining_reason;
          }

          if (!retreat_ready && !retreat_prefix.trajectory_.joint_trajectory.points.empty()) {
            auto try_sequential_arm_transition =
              [&](const std::string& first_side, const std::string& second_side,
                  std::string* failure_reason) {
                moveit::core::RobotState intermediate(retreat_state);
                for (size_t joint_index = 1; joint_index <= 6; ++joint_index) {
                  const std::string joint_name =
                    first_side + "_joint" + std::to_string(joint_index);
                  intermediate.setVariablePosition(
                    joint_name, goal_state.getVariablePosition(joint_name));
                }
                intermediate.update(true);

                const auto first_transition = repair_planner.plan(retreat_state, intermediate);
                if (!first_transition.valid) {
                  if (failure_reason) {
                    *failure_reason = first_side + "_first_transition_failed: " +
                      first_transition.failure_reason;
                  }
                  return false;
                }

                const auto second_transition = repair_planner.plan(intermediate, goal_state);
                if (!second_transition.valid) {
                  if (failure_reason) {
                    *failure_reason = second_side + "_second_transition_failed: " +
                      second_transition.failure_reason;
                  }
                  return false;
                }

                auto sequential_plan = retreat_prefix;
                append_plan_segment(sequential_plan, first_transition.plan);
                append_plan_segment(sequential_plan, second_transition.plan);
                std::string sequential_reason;
                if (config_.clearance_callback &&
                    !config_.clearance_callback(
                      sequential_plan, loaded_start_state, carried_boxes, &sequential_reason)) {
                  if (failure_reason) {
                    *failure_reason = "sequential_transition_validation_failed: " +
                      sequential_reason;
                  }
                  return false;
                }

                retreat_prefix = std::move(sequential_plan);
                return true;
              };

            std::string sequential_failure;
            retreat_ready =
              try_sequential_arm_transition("left", "right", &sequential_failure) ||
              try_sequential_arm_transition("right", "left", &sequential_failure);
            if (!retreat_ready && !sequential_failure.empty()) {
              rear_guard_failure = sequential_failure;
            }
          }

          if (!retreat_ready && !retreat_prefix.trajectory_.joint_trajectory.points.empty()) {
            const auto retreat_repaired = repair_planner.plan(retreat_state, goal_state);
            if (retreat_repaired.valid) {
              append_plan_segment(retreat_prefix, retreat_repaired.plan);
              retreat_ready = true;
            } else {
              rear_guard_failure = retreat_repaired.failure_reason;
            }
          }

          if (retreat_ready) {
            auto combined = retreat_prefix;
            std::string combined_reason;
            if (config_.clearance_callback &&
                !config_.clearance_callback(
                  combined, loaded_start_state, carried_boxes, &combined_reason)) {
              rear_guard_failure = combined_reason;
            } else {
              set_result_plan(combined);
              result.carried_clear = true;
              carried_collision_reason.clear();
              result.failure_reason.clear();
              rear_guard_repaired = true;
            }
          }
        }
        if (!rear_guard_repaired) {
          result.failure_reason = "shortcut_local_rrt_failed";
          if (!repaired.failure_reason.empty()) {
            result.failure_reason += ": " + repaired.failure_reason;
          }
          if (!rear_guard_failure.empty()) {
            result.failure_reason += "; rear_guard_retreat_failed: " + rear_guard_failure;
          }
          carried_collision_reason = result.failure_reason;
        }
      }
      result.plan_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - repair_start).count();
    }
  }

  if (config_.record_callback) {
    const auto& left_family = config_.selector->leftPoseFamily();
    const auto& right_family = config_.selector->rightPoseFamily();
    nlohmann::json extra = {
      {"stage_kind", "post_extract_loaded_plan"},
      {"loaded_planning_mode", config_.planning_mode},
      {"valid", result.carried_clear},
      {"lateral_shift_enabled", config_.lateral_shift_enabled},
      {"lateral_shift_attempted", result.lateral_shift_attempted},
      {"lateral_shift_success", result.lateral_shift_success},
      {"lateral_shift_ms", result.lateral_shift_ms},
      {"lateral_shift_reached_distance", result.lateral_shift_reached_distance},
      {"lateral_shift_points", result.lateral_shift_points},
      {"loaded_plan_ms", result.plan_ms},
      {"loaded_plan_points", result.plan_points},
      {"loaded_plan_trajectory_distance", result.trajectory_joint_distance},
      {"start_updown", currentUpdown(validation_start_state)},
      {"planning_start_updown", currentUpdown(loaded_start_state)},
      {"target_updown", currentUpdown(goal_state)},
      {"carried_box_count", carried_boxes.size()},
      {"moveit_attached_box_count", carried_boxes.size()},
      {"loaded_plan_rank", loaded_plan_rank},
      {"loaded_pose_distance_sum", result.selection.distance_sum},
      {"loaded_pose_distance_l2", result.selection.distance_l2},
      {"loaded_pose_max_joint_delta", result.selection.max_joint_delta},
      {"loaded_pose_max_joint_delta_deg", result.selection.max_joint_delta * 180.0 / M_PI},
      {"selected_left_loaded_pose_index", result.selection.left_index},
      {"selected_right_loaded_pose_index", result.selection.right_index},
      {"selected_left_loaded_pose_distance", result.selection.left_distance},
      {"selected_right_loaded_pose_distance", result.selection.right_distance},
      {"selected_left_loaded_pose_deg", pose_degrees_json(left_family[result.selection.left_index])},
      {"selected_right_loaded_pose_deg", pose_degrees_json(right_family[result.selection.right_index])},
      {"failure_reason", result.carried_clear ? "" : carried_collision_reason}
    };
    std::lock_guard<std::mutex> lock(record_mutex_);
    config_.record_callback(
      stage_name, plan, validation_start_state, goal_state, config_.target_joint_names, extra);
  }

  if (cancelled()) {
    return cancel_result();
  }

  if (!result.carried_clear) {
    restore_boxes();
    return result;
  }

  result.success = true;
  result.failure_reason.clear();
  restore_boxes();
  return result;
}

LoadedPoseBatchPlanResult LoadedPosePlanner::planBatch(
  const std::string& prefix,
  std::vector<ExtractRolloutTiming>& timings,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const LoadedPoseBatchPlanOptions& options)
{
  LoadedPoseBatchPlanResult batch_result;
  if (!options.enabled) {
    return batch_result;
  }

  for (size_t i = 0; i < timings.size(); ++i) {
    if (timings[i].success && timings[i].final_state) {
      batch_result.plan_indices.push_back(i);
    }
  }

  if (options.sort_by_pose_distance) {
    std::sort(batch_result.plan_indices.begin(), batch_result.plan_indices.end(),
              [&](const size_t a, const size_t b) {
                const auto& lhs = timings[a];
                const auto& rhs = timings[b];
                const double lhs_rank =
                  (std::isfinite(lhs.ik_score) ? lhs.ik_score : 0.0) +
                  (std::isfinite(lhs.loaded_pose_distance_sum) ? lhs.loaded_pose_distance_sum :
                    std::numeric_limits<double>::infinity());
                const double rhs_rank =
                  (std::isfinite(rhs.ik_score) ? rhs.ik_score : 0.0) +
                  (std::isfinite(rhs.loaded_pose_distance_sum) ? rhs.loaded_pose_distance_sum :
                    std::numeric_limits<double>::infinity());
                if (lhs_rank != rhs_rank) {
                  return lhs_rank < rhs_rank;
                }
                if (lhs.ik_score != rhs.ik_score) {
                  return lhs.ik_score < rhs.ik_score;
                }
                if (lhs.loaded_pose_distance_sum != rhs.loaded_pose_distance_sum) {
                  return lhs.loaded_pose_distance_sum < rhs.loaded_pose_distance_sum;
                }
                if (lhs.loaded_pose_distance_l2 != rhs.loaded_pose_distance_l2) {
                  return lhs.loaded_pose_distance_l2 < rhs.loaded_pose_distance_l2;
                }
                if (lhs.loaded_pose_max_joint_delta != rhs.loaded_pose_max_joint_delta) {
                  return lhs.loaded_pose_max_joint_delta < rhs.loaded_pose_max_joint_delta;
                }
                return lhs.candidate_order < rhs.candidate_order;
              });
  }

  const size_t loaded_limit = options.candidate_limit > 0
    ? std::min(options.candidate_limit, batch_result.plan_indices.size())
    : batch_result.plan_indices.size();

  const auto start = std::chrono::steady_clock::now();
  const auto apply_result = [&](size_t rank, const LoadedPosePlanResult& result) {
    auto& timing = timings[batch_result.plan_indices[rank]];
    timing.loaded_plan_attempted = result.attempted;
    timing.loaded_plan_success = result.success;
    timing.lateral_shift_attempted = result.lateral_shift_attempted;
    timing.lateral_shift_success = result.lateral_shift_success;
    timing.lateral_shift_ms = result.lateral_shift_ms;
    timing.lateral_shift_reached_distance = result.lateral_shift_reached_distance;
    timing.lateral_shift_points = result.lateral_shift_points;
    timing.loaded_plan_ms = result.plan_ms;
    timing.loaded_plan_points = result.plan_points;
    timing.loaded_plan_trajectory_distance = std::isfinite(result.trajectory_joint_distance)
      ? result.trajectory_joint_distance
      : 0.0;
    timing.selected_left_loaded_pose_index = result.selection.left_index;
    timing.selected_right_loaded_pose_index = result.selection.right_index;
    timing.selected_left_loaded_pose_distance = result.selection.left_distance;
    timing.selected_right_loaded_pose_distance = result.selection.right_distance;
    timing.loaded_pose_distance_sum = result.selection.distance_sum;
    timing.loaded_pose_distance_l2 = result.selection.distance_l2;
    timing.loaded_pose_max_joint_delta = result.selection.max_joint_delta;
    timing.loaded_plan_failure_reason = result.failure_reason;
    timing.lateral_shift_replay_stages = result.lateral_shift_replay_stages;
    if (result.start_state && result.goal_state) {
      timing.loaded_plan = result.plan;
      timing.loaded_start_state = result.start_state;
      timing.loaded_goal_state = result.goal_state;
    }
  };

  const auto set_skipped_after_success = [&](size_t from_rank) {
    for (size_t rest_rank = from_rank; rest_rank < batch_result.plan_indices.size(); ++rest_rank) {
      auto& skipped = timings[batch_result.plan_indices[rest_rank]];
      skipped.loaded_plan_rank = rest_rank + 1;
      if (!skipped.loaded_plan_attempted && skipped.loaded_plan_failure_reason.empty()) {
        skipped.loaded_plan_failure_reason = "loaded_plan_skipped_after_first_success";
      }
    }
  };

  const auto choose_best_success = [&]() {
    size_t best_index = timings.size();
    double best_score = std::numeric_limits<double>::infinity();
    for (const size_t timing_index : batch_result.plan_indices) {
      auto& timing = timings[timing_index];
      timing.loaded_plan_selected = false;
      if (!timing.loaded_plan_success) {
        continue;
      }
      const double trajectory_distance = std::isfinite(timing.loaded_plan_trajectory_distance)
        ? timing.loaded_plan_trajectory_distance
        : std::numeric_limits<double>::infinity();
      const double ik_score = std::isfinite(timing.ik_score)
        ? timing.ik_score
        : 0.0;
      const double score = ik_score + trajectory_distance;
      if (score < best_score ||
          (score == best_score &&
           (best_index >= timings.size() || timing.loaded_plan_rank < timings[best_index].loaded_plan_rank))) {
        best_score = score;
        best_index = timing_index;
      }
    }
    if (best_index < timings.size()) {
      timings[best_index].loaded_plan_selected = true;
    }
  };

  const size_t worker_count = std::max<size_t>(1, std::min(options.parallel_workers, loaded_limit));
  const bool use_parallel = worker_count > 1;
  if (!use_parallel) {
    for (size_t rank = 0; rank < batch_result.plan_indices.size(); ++rank) {
      auto& timing = timings[batch_result.plan_indices[rank]];
      timing.loaded_plan_rank = rank + 1;

      if (rank >= loaded_limit) {
        timing.loaded_plan_failure_reason = "loaded_plan_skipped_by_limit";
        continue;
      }

      const LoadedPosePlanResult result = plan(
        prefix + "/candidate_" + std::to_string(timing.candidate_order) + "/post_extract_loaded",
        *timing.final_state,
        carried_boxes,
        timing.loaded_plan_rank);

      apply_result(rank, result);

      if (options.stop_on_first_success && timing.loaded_plan_success) {
        set_skipped_after_success(rank + 1);
        break;
      }
    }
  } else {
    const auto saved_boxes = config_.scene_adapter->activeAttachedBoxes();
    config_.scene_adapter->setActiveAttachedBoxesForRecordOnly(carried_boxes);
    if (!config_.scene_adapter->applyAttachedBoxState(
          config_.scene_adapter->activeAttachedBoxes(),
          moveit_msgs::msg::CollisionObject::ADD)) {
      for (size_t rank = 0; rank < batch_result.plan_indices.size(); ++rank) {
        auto& timing = timings[batch_result.plan_indices[rank]];
        timing.loaded_plan_rank = rank + 1;
        timing.loaded_plan_failure_reason = rank < loaded_limit
          ? "failed_to_attach_box_for_parallel_loaded_plan"
          : "loaded_plan_skipped_by_limit";
      }
      batch_result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      return batch_result;
    }

    std::vector<LoadedPosePlanResult> results(loaded_limit);
    std::vector<bool> finished(loaded_limit, false);
    std::mutex result_mutex;
    std::atomic<size_t> next_rank{0};
    std::atomic<bool> stop{false};
    std::atomic<size_t> first_success_rank{loaded_limit};

    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    for (size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
      workers.emplace_back([&, worker_index]() {
        (void)worker_index;
        while (true) {
          if (options.stop_on_first_success && stop.load(std::memory_order_relaxed)) {
            break;
          }
          const size_t rank = next_rank.fetch_add(1, std::memory_order_relaxed);
          if (rank >= loaded_limit) {
            break;
          }
          const auto& timing = timings[batch_result.plan_indices[rank]];
          LoadedPosePlanResult result = planInternal(
            prefix + "/candidate_" + std::to_string(timing.candidate_order) + "/post_extract_loaded",
            *timing.final_state,
            carried_boxes,
            rank + 1,
            false,
            [&]() {
              return options.stop_on_first_success && stop.load(std::memory_order_relaxed);
            });
          if (options.stop_on_first_success && result.success) {
            size_t expected = loaded_limit;
            first_success_rank.compare_exchange_strong(
              expected, rank, std::memory_order_relaxed);
            stop.store(true, std::memory_order_relaxed);
          }
          {
            std::lock_guard<std::mutex> lock(result_mutex);
            results[rank] = std::move(result);
            finished[rank] = true;
          }
        }
      });
    }
    for (auto& worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }

    for (size_t rank = 0; rank < batch_result.plan_indices.size(); ++rank) {
      auto& timing = timings[batch_result.plan_indices[rank]];
      timing.loaded_plan_rank = rank + 1;
      if (rank >= loaded_limit) {
        timing.loaded_plan_failure_reason = "loaded_plan_skipped_by_limit";
        continue;
      }
      if (!finished[rank]) {
        timing.loaded_plan_failure_reason = "loaded_plan_skipped_after_first_success";
        continue;
      }
      apply_result(rank, results[rank]);
    }

    if (options.stop_on_first_success && first_success_rank.load(std::memory_order_relaxed) < loaded_limit) {
      const size_t winner_rank = first_success_rank.load(std::memory_order_relaxed);
      for (const size_t timing_index : batch_result.plan_indices) {
        timings[timing_index].loaded_plan_selected = false;
      }
      timings[batch_result.plan_indices[winner_rank]].loaded_plan_selected = true;
      if (winner_rank + 1 < batch_result.plan_indices.size()) {
        set_skipped_after_success(winner_rank + 1);
      }
    } else {
      choose_best_success();
    }

    std::vector<std::string> ids;
    ids.reserve(carried_boxes.size());
    for (const auto& box : carried_boxes) {
      ids.push_back(box.id);
    }
    config_.scene_adapter->removeCarriedBoxIds(ids);
    config_.scene_adapter->setActiveAttachedBoxesForRecordOnly(saved_boxes);
  }
  if (!use_parallel) {
    choose_best_success();
  }
  batch_result.wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  return batch_result;
}

}  // namespace alfa_robot::motion
