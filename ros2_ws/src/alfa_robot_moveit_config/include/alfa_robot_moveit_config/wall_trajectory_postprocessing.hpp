#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <vector>

namespace alfa_robot::motion
{
struct TrajectoryFrame
{
  std::string stage;
  std::vector<double> joints;
  bool box_attached = false;
  bool box_visible = true;
  size_t scene_index = 0;
  nlohmann::json carried_boxes = nlohmann::json::array();
  double time_from_start_s = 0.0;
};

struct TrajectoryPostprocessOptions
{
  std::string variant = "topk";
  double sample_period = 0.05;
  double empty_velocity_scaling = 0.50;
  double empty_acceleration_scaling = 0.50;
  double loaded_velocity_scaling = 0.25;
  double loaded_acceleration_scaling = 0.25;
  double arm_max_jerk = 2.0;
  double head_max_jerk = 2.0;
  double updown_max_jerk = 0.30;
};

struct TrajectoryPostprocessMetrics
{
  double shortcut_ms = 0.0;
  double chomp_ms = 0.0;
  double totg_ms = 0.0;
  double ruckig_ms = 0.0;
  double execution_duration_s = 0.0;
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  double max_jerk = 0.0;
  bool timing_valid = false;
  std::string effective_variant = "topk";
  std::string optimizer_status = "not_requested";
  std::string fallback_reason;
};

using EdgeValidator = std::function<bool(const TrajectoryFrame&, const TrajectoryFrame&, std::string*)>;
using PathValidator = std::function<bool(const std::vector<TrajectoryFrame>&, std::string*)>;

bool optimizeChompFreeSpace(
  std::vector<TrajectoryFrame>* frames,
  const moveit::core::RobotModelConstPtr& robot_model,
  const planning_scene::PlanningSceneConstPtr& empty_scene,
  const planning_scene::PlanningSceneConstPtr& loaded_scene,
  const std::vector<std::string>& joint_names,
  const std::string& group_name,
  double* wall_ms,
  std::string* reason);

bool postprocessWallTrajectory(
  std::vector<TrajectoryFrame>* frames,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::vector<std::string>& joint_names,
  const TrajectoryPostprocessOptions& options,
  const EdgeValidator& edge_validator,
  const PathValidator& path_validator,
  const std::function<bool(std::vector<TrajectoryFrame>*, double*, std::string*)>& chomp_optimizer,
  TrajectoryPostprocessMetrics* metrics,
  std::string* reason);
}  // namespace alfa_robot::motion
