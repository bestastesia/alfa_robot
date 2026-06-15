#pragma once

#include "alfa_robot_moveit_config/extract_planner_types.hpp"
#include "alfa_robot_moveit_config/loaded_pose_selector.hpp"
#include "alfa_robot_moveit_config/motion_core/scene_geometry.hpp"

#include <moveit/move_group_interface/move_group_interface.h>

#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace alfa_robot::motion
{

class MotionSceneAdapter;

struct LoadedPosePlanResult
{
  bool success = false;
  bool attempted = false;
  bool carried_clear = false;
  double plan_ms = 0.0;
  size_t plan_points = 0;
  std::string failure_reason;
  LoadedPoseSelection selection;
};

struct LoadedPoseBatchPlanOptions
{
  bool enabled = false;
  bool sort_by_pose_distance = true;
  bool stop_on_first_success = false;
  size_t candidate_limit = 0;
};

struct LoadedPoseBatchPlanResult
{
  std::vector<size_t> plan_indices;
  double wall_ms = 0.0;
};

using LoadedPlanClearanceCallback = std::function<bool(
  const moveit::planning_interface::MoveGroupInterface::Plan&,
  const moveit::core::RobotState&,
  std::string*)>;

using LoadedPlanRecordCallback = std::function<void(
  const std::string&,
  const moveit::planning_interface::MoveGroupInterface::Plan&,
  const moveit::core::RobotState&,
  const moveit::core::RobotState&,
  const std::vector<std::string>&,
  const nlohmann::json&)>;

struct LoadedPosePlannerConfig
{
  moveit::planning_interface::MoveGroupInterface* move_group = nullptr;
  LoadedPoseSelector* selector = nullptr;
  MotionSceneAdapter* scene_adapter = nullptr;
  std::vector<std::string> target_joint_names;
  LoadedPlanClearanceCallback clearance_callback;
  LoadedPlanRecordCallback record_callback;
};

class LoadedPosePlanner
{
public:
  explicit LoadedPosePlanner(LoadedPosePlannerConfig config);

  LoadedPosePlanResult plan(
    const std::string& stage_name,
    const moveit::core::RobotState& extract_state,
    const std::vector<AttachedBoxSpec>& carried_boxes,
    size_t loaded_plan_rank);

  LoadedPoseBatchPlanResult planBatch(
    const std::string& prefix,
    std::vector<ExtractRolloutTiming>& timings,
    const std::vector<AttachedBoxSpec>& carried_boxes,
    const LoadedPoseBatchPlanOptions& options);

private:
  static double currentUpdown(const moveit::core::RobotState& state);

  LoadedPosePlannerConfig config_;
};

}  // namespace alfa_robot::motion
