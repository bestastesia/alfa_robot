#include "alfa_robot_moveit_config/curobo_segment_client.hpp"

#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <moveit/robot_state/conversions.h>
#include <rclcpp/duration.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace alfa_robot::motion
{

namespace
{

using Clock = std::chrono::steady_clock;

double elapsed_ms(const Clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
}

int parse_box_id(const std::string& id)
{
  size_t begin = id.size();
  while (begin > 0 && std::isdigit(static_cast<unsigned char>(id[begin - 1]))) {
    --begin;
  }
  if (begin == id.size()) return 0;
  try {
    return std::stoi(id.substr(begin));
  } catch (...) {
    return 0;
  }
}

std::string side_from_link(const std::string& link_name)
{
  if (link_name.rfind("left", 0) == 0) return "left";
  if (link_name.rfind("right", 0) == 0) return "right";
  return "";
}

std::vector<std::string> touch_links(const AttachedBoxSpec& box)
{
  const std::string side = side_from_link(box.link_name);
  std::vector<std::string> links{box.link_name};
  if (!side.empty()) {
    links.push_back(side + "_joint6");
    links.push_back(side + "_joint5");
    links.push_back(side + "_joint4");
    links.push_back(side + "_joint3");
  }
  return links;
}

sensor_msgs::msg::JointState to_joint_state(
  const moveit::core::RobotState& state,
  const std::vector<std::string>& joint_names)
{
  sensor_msgs::msg::JointState message;
  message.name = joint_names;
  message.position.reserve(joint_names.size());
  for (const auto& name : joint_names) {
    message.position.push_back(state.getVariablePosition(name));
  }
  return message;
}

robot_motion_interfaces::msg::AttachedBox to_attached_box(const AttachedBoxSpec& box)
{
  robot_motion_interfaces::msg::AttachedBox message;
  message.id = box.id;
  message.box_id = parse_box_id(box.id);
  message.side = side_from_link(box.link_name);
  message.grasp_mode = "explicit_center_in_link";
  message.link_name = box.link_name;
  message.center_in_link.position.x = box.center_in_link[0];
  message.center_in_link.position.y = box.center_in_link[1];
  message.center_in_link.position.z = box.center_in_link[2];
  message.center_in_link.orientation.w = 1.0;
  message.size.x = box.size[0];
  message.size.y = box.size[1];
  message.size.z = box.size[2];
  message.touch_links = touch_links(box);
  return message;
}

bool finite_vector(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

}  // namespace

CuroboSegmentClient::CuroboSegmentClient(
  rclcpp::Node& node, CuroboSegmentClientConfig config)
: node_(node), config_(std::move(config))
{
  callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  client_ = node_.create_client<Service>(
    config_.service_name, rmw_qos_profile_services_default, callback_group_);
}

bool CuroboSegmentClient::serviceReady() const
{
  return client_ && client_->service_is_ready();
}

bool CuroboSegmentClient::plan(
  const std::string& stage_name,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const std::vector<AttachedBoxSpec>& attached_boxes,
  Plan* output_plan,
  ExtractMonitorLocalPlanMetrics* metrics,
  std::string* reason)
{
  if (metrics) *metrics = ExtractMonitorLocalPlanMetrics{};
  if (!client_) {
    if (reason) *reason = "local_curobo_client_not_initialized";
    return false;
  }
  if (!client_->service_is_ready()) {
    if (reason) *reason = "local_curobo_service_unavailable:" + config_.service_name;
    return false;
  }

  const auto joint_names = dual_arm_with_updown_joint_names();
  auto request = std::make_shared<Service::Request>();
  request->context.request_id = stage_name + ":" + std::to_string(++request_counter_);
  request->context.stamp = node_.now();
  request->context.frame_id = config_.frame_id;
  request->context.scene_id = config_.scene_id;
  request->start_state = to_joint_state(start_state, joint_names);
  request->goal_state = to_joint_state(goal_state, joint_names);
  request->attached_boxes.reserve(attached_boxes.size());
  for (const auto& box : attached_boxes) {
    request->attached_boxes.push_back(to_attached_box(box));
  }
  request->force_graph = config_.force_graph;
  request->max_attempts = static_cast<uint32_t>(std::max<size_t>(1, config_.max_attempts));
  request->timeout_s = config_.timeout_s;

  const auto started = Clock::now();
  auto pending = client_->async_send_request(request);
  const auto timeout = std::chrono::duration<double>(std::max(1e-3, config_.timeout_s));
  if (pending.wait_for(timeout) != std::future_status::ready) {
    client_->remove_pending_request(pending);
    const double roundtrip = elapsed_ms(started);
    if (metrics) metrics->roundtrip_ms = roundtrip;
    if (reason) {
      std::ostringstream stream;
      stream << "local_curobo_timeout_after_" << roundtrip << "ms";
      *reason = stream.str();
    }
    return false;
  }

  const auto response = pending.get();
  const double roundtrip = elapsed_ms(started);
  if (metrics) {
    metrics->planner_method = response->planner_method;
    metrics->roundtrip_ms = roundtrip;
    metrics->backend_total_ms = response->total_time_ms;
    metrics->backend_solve_ms = response->solve_time_ms;
    metrics->backend_queue_ms = response->queue_time_ms;
    metrics->backend_endpoint_check_ms = response->endpoint_check_time_ms;
    metrics->backend_graph_ms = response->graph_time_ms;
    metrics->backend_trajopt_ms = response->trajopt_time_ms;
    metrics->backend_interpolation_ms = response->interpolation_time_ms;
  }
  if (!response->success) {
    if (reason) {
      *reason = response->message.empty()
        ? "local_curobo_service_failed"
        : "local_curobo_service_failed:" + response->message;
    }
    return false;
  }
  const auto conversion_started = Clock::now();
  const bool converted = validateAndConvertResponse(
    *response, start_state, goal_state, config_, output_plan, reason);
  if (metrics) metrics->response_conversion_ms = elapsed_ms(conversion_started);
  return converted;
}

bool CuroboSegmentClient::validateAndConvertResponse(
  const Service::Response& response,
  const moveit::core::RobotState& start_state,
  const moveit::core::RobotState& goal_state,
  const CuroboSegmentClientConfig& config,
  Plan* output_plan,
  std::string* reason)
{
  const auto canonical_names = dual_arm_with_updown_joint_names();
  const auto& input = response.trajectory;
  if (input.points.size() < 2 || input.joint_names.empty()) {
    if (reason) *reason = "local_curobo_empty_trajectory";
    return false;
  }
  std::map<std::string, size_t> indices;
  for (size_t i = 0; i < input.joint_names.size(); ++i) {
    if (!indices.emplace(input.joint_names[i], i).second) {
      if (reason) *reason = "local_curobo_duplicate_joint:" + input.joint_names[i];
      return false;
    }
  }
  if (indices.size() != canonical_names.size()) {
    if (reason) *reason = "local_curobo_joint_count_mismatch";
    return false;
  }
  for (const auto& name : canonical_names) {
    if (indices.count(name) == 0) {
      if (reason) *reason = "local_curobo_missing_joint:" + name;
      return false;
    }
  }

  Plan converted;
  moveit::core::robotStateToRobotStateMsg(start_state, converted.start_state_, true);
  converted.planning_time_ = std::max(0.0, response.total_time_ms) / 1000.0;
  auto& output = converted.trajectory_.joint_trajectory;
  output.header = input.header;
  output.joint_names = canonical_names;
  output.points.reserve(input.points.size());

  double previous_time = -std::numeric_limits<double>::infinity();
  const auto* group = start_state.getRobotModel()->getJointModelGroup(config.joint_group);
  if (!group) {
    if (reason) *reason = "local_curobo_unknown_joint_group:" + config.joint_group;
    return false;
  }
  for (size_t point_index = 0; point_index < input.points.size(); ++point_index) {
    const auto& source = input.points[point_index];
    if (source.positions.size() != input.joint_names.size() || !finite_vector(source.positions)) {
      if (reason) *reason = "local_curobo_invalid_positions@" + std::to_string(point_index);
      return false;
    }
    const double time_s = rclcpp::Duration(source.time_from_start).seconds();
    if (!std::isfinite(time_s) || (point_index > 0 && time_s <= previous_time)) {
      if (reason) *reason = "local_curobo_non_monotonic_time@" + std::to_string(point_index);
      return false;
    }
    previous_time = time_s;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = source.time_from_start;
    point.positions.reserve(canonical_names.size());
    const bool copy_velocities = source.velocities.size() == input.joint_names.size();
    const bool copy_accelerations = source.accelerations.size() == input.joint_names.size();
    if ((!source.velocities.empty() && !copy_velocities) ||
        (!source.accelerations.empty() && !copy_accelerations) ||
        (copy_velocities && !finite_vector(source.velocities)) ||
        (copy_accelerations && !finite_vector(source.accelerations))) {
      if (reason) *reason = "local_curobo_invalid_derivatives@" + std::to_string(point_index);
      return false;
    }
    if (copy_velocities) point.velocities.reserve(canonical_names.size());
    if (copy_accelerations) point.accelerations.reserve(canonical_names.size());
    moveit::core::RobotState state(start_state);
    for (const auto& name : canonical_names) {
      const size_t source_index = indices.at(name);
      const double position = source.positions[source_index];
      point.positions.push_back(position);
      state.setVariablePosition(name, position);
      if (copy_velocities) point.velocities.push_back(source.velocities[source_index]);
      if (copy_accelerations) point.accelerations.push_back(source.accelerations[source_index]);
    }
    state.update(true);
    if (!state.satisfiesBounds(group, config.bounds_tolerance)) {
      if (reason) *reason = "local_curobo_joint_bounds_violation@" + std::to_string(point_index);
      return false;
    }
    output.points.push_back(std::move(point));
  }

  for (size_t i = 0; i < canonical_names.size(); ++i) {
    const auto& name = canonical_names[i];
    const double start_error = std::abs(
      output.points.front().positions[i] - start_state.getVariablePosition(name));
    const double goal_error = std::abs(
      output.points.back().positions[i] - goal_state.getVariablePosition(name));
    if (start_error > config.endpoint_tolerance || goal_error > config.endpoint_tolerance) {
      if (reason) *reason = "local_curobo_endpoint_mismatch:" + name;
      return false;
    }
  }

  if (output_plan) *output_plan = std::move(converted);
  if (reason) reason->clear();
  return true;
}

}  // namespace alfa_robot::motion
