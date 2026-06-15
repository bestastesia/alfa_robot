#pragma once

#include "alfa_robot_moveit_config/motion_core/task_geometry.hpp"

#include <cstddef>
#include <vector>

namespace alfa_robot::motion
{

struct ExtractMotionDelta
{
  double retreat_ratio = 1.0;
  double lift_ratio = 0.0;
  double pitch_delta_deg = 0.0;
};

struct ExtractMotionCommand
{
  size_t candidate_index = 0;
  double retreat_x = 0.0;
  double retreat_delta_x = 0.0;
  double lift_z = 0.0;
  double lift_delta_z = 0.0;
  double pitch_up_rad = 0.0;
  double pitch_delta_rad = 0.0;
  BoxSpec shifted_box;
};

struct ExtractMotionLayer
{
  double pitch_delta_deg = 0.0;
  std::vector<ExtractMotionCommand> commands;
};

struct ExtractMotionPlannerConfig
{
  double step_x = 0.03;
  double max_x = 0.36;
};

class ExtractMotionPlanner
{
public:
  explicit ExtractMotionPlanner(ExtractMotionPlannerConfig config = {});

  const ExtractMotionPlannerConfig& config() const { return config_; }

  std::vector<ExtractMotionDelta> motionDeltas() const;

  std::vector<double> pitchDeltaDegrees(double current_pitch_rad) const;

  std::vector<ExtractMotionLayer> layers(
    const BoxSpec& source_box,
    int box_id,
    double current_pitch_rad,
    double last_retreat_x,
    double current_lift_z) const;

  size_t maxStepCount() const;

private:
  ExtractMotionPlannerConfig config_;
};

}  // namespace alfa_robot::motion
