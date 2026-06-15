#pragma once

#include "alfa_robot_moveit_config/extract_planner_types.hpp"

#include <cstddef>
#include <vector>

namespace alfa_robot::motion
{

struct ExtractBenchmarkSummary
{
  double total_interval_ms = 0.0;
  double total_rollout_ms = 0.0;
  double total_loaded_plan_ms = 0.0;
  double mean_interval_ms = 0.0;
  double mean_rollout_ms = 0.0;
  double mean_loaded_plan_ms = 0.0;
  size_t loaded_plan_attempted_count = 0;
  size_t loaded_plan_success_count = 0;
  size_t loaded_plan_first_success_rank = 0;
  size_t loaded_plan_first_success_candidate_order = 0;
  bool any_success = false;
};

ExtractBenchmarkSummary summarize_extract_timings(
  const std::vector<ExtractRolloutTiming>& timings);

}  // namespace alfa_robot::motion
