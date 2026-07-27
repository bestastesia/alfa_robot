#pragma once

#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace moveit::core
{
class JointModelGroup;
}  // namespace moveit::core

namespace alfa_robot::motion
{

class MotionSceneAdapter;
class ExtractCandidateSolver;
struct ExtractRolloutTiming;

struct LoadedPoseSelection
{
  size_t left_index = 0;
  size_t right_index = 0;
  double left_distance = std::numeric_limits<double>::infinity();
  double right_distance = std::numeric_limits<double>::infinity();
  double distance_sum = std::numeric_limits<double>::infinity();
  double distance_l2 = std::numeric_limits<double>::infinity();
  double max_joint_delta = std::numeric_limits<double>::infinity();
};

struct LoadedPoseSelectorConfig
{
  std::vector<std::vector<double>> left_pose_family;
  std::vector<std::vector<double>> right_pose_family;
  size_t left_preferred_index = 0;
  size_t right_preferred_index = 0;
  double target_updown = 0.3;
  bool preserve_lower_updown = false;
  const moveit::core::JointModelGroup* enforce_bounds_group = nullptr;
};

class LoadedPoseSelector
{
public:
  explicit LoadedPoseSelector(LoadedPoseSelectorConfig config);

  const LoadedPoseSelectorConfig& config() const { return config_; }
  const std::vector<std::vector<double>>& leftPoseFamily() const { return config_.left_pose_family; }
  const std::vector<std::vector<double>>& rightPoseFamily() const { return config_.right_pose_family; }
  size_t leftPreferredIndex() const { return config_.left_preferred_index; }
  size_t rightPreferredIndex() const { return config_.right_preferred_index; }

  double armPoseDistance(
    const moveit::core::RobotState& state,
    const std::string& side,
    const std::vector<double>& pose) const;

  size_t nearestPoseIndex(
    const moveit::core::RobotState& state,
    const std::string& side,
    const std::vector<std::vector<double>>& family,
    double* distance = nullptr) const;

  LoadedPoseSelection select(const moveit::core::RobotState& state) const;

  std::array<double, 3> distanceMetrics(
    const moveit::core::RobotState& state,
    size_t left_index,
    size_t right_index) const;

  moveit::core::RobotState makeGoalState(
    const moveit::core::RobotState& start_state,
    LoadedPoseSelection* selection = nullptr) const;

  void fillTimingDistanceMetrics(ExtractRolloutTiming& timing) const;

private:
  bool hasVariable(const moveit::core::RobotState& state, const std::string& name) const;
  static std::string jointName(const std::string& side, size_t index);

  LoadedPoseSelectorConfig config_;
};

class MotionSceneAdapter;

struct LoadedPoseReplayStage
{
  std::string stage_name;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr start_state;
  moveit::core::RobotStatePtr goal_state;
  nlohmann::json extra;
};

struct LoadedPosePlanResult
{
  bool success = false;
  bool attempted = false;
  bool carried_clear = false;
  bool lateral_shift_attempted = false;
  bool lateral_shift_success = false;
  double lateral_shift_ms = 0.0;
  double lateral_shift_reached_distance = 0.0;
  size_t lateral_shift_points = 0;
  double plan_ms = 0.0;
  size_t plan_points = 0;
  double trajectory_joint_distance = std::numeric_limits<double>::infinity();
  std::string failure_reason;
  LoadedPoseSelection selection;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr start_state;
  moveit::core::RobotStatePtr goal_state;
  std::vector<LoadedPoseReplayStage> lateral_shift_replay_stages;
};

struct LoadedPoseBatchPlanOptions
{
  bool enabled = false;
  bool sort_by_pose_distance = true;
  bool stop_on_first_success = false;
  size_t candidate_limit = 0;
  size_t parallel_workers = 1;
};

struct LoadedPoseBatchPlanResult
{
  std::vector<size_t> plan_indices;
  double wall_ms = 0.0;
};

using LoadedPlanClearanceCallback = std::function<bool(
  const moveit::planning_interface::MoveGroupInterface::Plan&,
  const moveit::core::RobotState&,
  const std::vector<AttachedBoxSpec>&,
  std::string*)>;

using LoadedPlanRecordCallback = std::function<void(
  const std::string&,
  const moveit::planning_interface::MoveGroupInterface::Plan&,
  const moveit::core::RobotState&,
  const moveit::core::RobotState&,
  const std::vector<std::string>&,
  const nlohmann::json&)>;

using LoadedDirectPlanCallback = std::function<bool(
  const std::string&,
  const moveit::core::RobotState&,
  const moveit::core::RobotState&,
  const std::vector<AttachedBoxSpec>&,
  moveit::planning_interface::MoveGroupInterface::Plan*,
  std::string*)>;

using LoadedPlanCancellationCheck = std::function<bool()>;
using LoadedTopSuctionHeightMismatchCallback = std::function<bool()>;

struct LoadedPosePlannerConfig
{
  moveit::planning_interface::MoveGroupInterface* move_group = nullptr;
  LoadedPoseSelector* selector = nullptr;
  MotionSceneAdapter* scene_adapter = nullptr;
  std::vector<std::string> target_joint_names;
  double attached_box_collision_padding = 0.0;
  bool lateral_shift_enabled = false;
  double lateral_shift_distance = 0.4;
  double lateral_shift_step = 0.04;
  int lateral_shift_column = 2;
  int pre_loaded_lower_left_box_id = 0;
  int pre_loaded_lower_right_box_id = 0;
  double pre_loaded_lower_updown_delta = 0.0;
  double fixed_updown = 0.3;
  std::string planning_mode = "rrt";
  double min_tool_normal_z = -1e-4;
  double max_joint_delta = 0.0;
  ExtractCandidateSolver* lateral_shift_solver = nullptr;
  LoadedPlanClearanceCallback clearance_callback;
  LoadedPlanRecordCallback record_callback;
  LoadedDirectPlanCallback direct_plan_callback;
  LoadedTopSuctionHeightMismatchCallback top_suction_height_mismatch_callback;
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
  static int boxColumn(const AttachedBoxSpec& box);
  static int boxId(const AttachedBoxSpec& box);

  LoadedPosePlanResult planInternal(
    const std::string& stage_name,
    const moveit::core::RobotState& extract_state,
    const std::vector<AttachedBoxSpec>& carried_boxes,
    size_t loaded_plan_rank,
    bool manage_scene_adapter,
    const LoadedPlanCancellationCheck& is_cancelled);

  bool planLateralShift(
    const std::string& stage_name,
    const moveit::core::RobotState& start_state,
    const std::vector<AttachedBoxSpec>& carried_boxes,
    moveit::core::RobotState* shifted_state,
    LoadedPosePlanResult* result);

  LoadedPosePlannerConfig config_;
  std::mutex record_mutex_;
};

}  // namespace alfa_robot::motion
