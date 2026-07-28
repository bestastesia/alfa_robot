#pragma once

#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"
#include "robot_motion_interfaces/srv/plan_joint_segment.hpp"
#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct CuroboSegmentClientConfig
{
  std::string service_name = "/robot_motion/plan_joint_segment";
  std::string scene_id;
  std::string frame_id = "base_link";
  std::string joint_group = "dual_arm_with_base";
  double timeout_s = 0.5;
  size_t max_attempts = 3;
  bool force_graph = true;
  double endpoint_tolerance = 1e-4;
  double bounds_tolerance = 1e-6;
};

class CuroboSegmentClient
{
public:
  using Service = robot_motion_interfaces::srv::PlanJointSegment;
  using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

  CuroboSegmentClient(rclcpp::Node& node, CuroboSegmentClientConfig config);

  const CuroboSegmentClientConfig& config() const { return config_; }
  bool serviceReady() const;

  bool plan(
    const std::string& stage_name,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const std::vector<AttachedBoxSpec>& attached_boxes,
    Plan* output_plan,
    ExtractMonitorLocalPlanMetrics* metrics,
    std::string* reason);

  static bool validateAndConvertResponse(
    const Service::Response& response,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const CuroboSegmentClientConfig& config,
    Plan* output_plan,
    std::string* reason);

private:
  rclcpp::Node& node_;
  CuroboSegmentClientConfig config_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<Service>::SharedPtr client_;
  std::atomic<uint64_t> request_counter_{0};
};

}  // namespace alfa_robot::motion
