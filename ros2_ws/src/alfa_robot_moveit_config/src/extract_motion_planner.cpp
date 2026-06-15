#include "alfa_robot_moveit_config/extract_motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace alfa_robot::motion
{

ExtractMotionPlanner::ExtractMotionPlanner(ExtractMotionPlannerConfig config)
: config_(std::move(config))
{}

std::vector<ExtractMotionDelta> ExtractMotionPlanner::motionDeltas() const
{
  return {
    {1.0, 0.0, 0.0},
    {0.8, 0.6, 0.0},
    {0.6, 0.8, 0.0},
    {0.0, 1.0, 0.0},
  };
}

std::vector<double> ExtractMotionPlanner::pitchDeltaDegrees(double current_pitch_rad) const
{
  const double current_pitch_deg = current_pitch_rad * 180.0 / M_PI;
  std::vector<double> deltas{0.0, 1.0};
  if (current_pitch_deg >= 1.0) {
    deltas.push_back(-1.0);
  }
  deltas.push_back(3.0);
  if (current_pitch_deg >= 3.0) {
    deltas.push_back(-3.0);
  }
  return deltas;
}

std::vector<ExtractMotionLayer> ExtractMotionPlanner::layers(
  const BoxSpec& source_box,
  int box_id,
  double current_pitch_rad,
  double last_retreat_x,
  double current_lift_z) const
{
  std::vector<ExtractMotionLayer> result;
  size_t candidate_index = 0;
  for (double pitch_delta_deg : pitchDeltaDegrees(current_pitch_rad)) {
    ExtractMotionLayer layer;
    layer.pitch_delta_deg = pitch_delta_deg;
    const double pitch_delta = pitch_delta_deg * M_PI / 180.0;
    const double pitch_rad = std::max(0.0, current_pitch_rad + pitch_delta);
    for (const auto& delta : motionDeltas()) {
      const double retreat_delta = std::max(0.0, delta.retreat_ratio * config_.step_x);
      const double retreat_x = std::min(config_.max_x, last_retreat_x + retreat_delta);
      if (retreat_x <= last_retreat_x + 1e-6 && last_retreat_x >= config_.max_x - 1e-6) {
        continue;
      }

      const double lift_delta = std::max(0.0, delta.lift_ratio * config_.step_x);
      const double lift_z = current_lift_z + lift_delta;
      layer.commands.push_back({
        candidate_index,
        retreat_x,
        retreat_x - last_retreat_x,
        lift_z,
        lift_delta,
        pitch_rad,
        pitch_delta,
        BoxSpec{box_id, source_box.x - retreat_x, source_box.y, source_box.z + lift_z},
      });
      ++candidate_index;
    }
    result.push_back(std::move(layer));
  }
  return result;
}

size_t ExtractMotionPlanner::maxStepCount() const
{
  return static_cast<size_t>(
    std::ceil(std::max(0.0, config_.max_x) / std::max(1e-6, 0.6 * config_.step_x))) + 2;
}

}  // namespace alfa_robot::motion
