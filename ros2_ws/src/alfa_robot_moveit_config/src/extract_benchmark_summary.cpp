#include "alfa_robot_moveit_config/extract_benchmark_summary.hpp"

namespace alfa_robot::motion
{

ExtractBenchmarkSummary summarize_extract_timings(
  const std::vector<ExtractRolloutTiming>& timings)
{
  ExtractBenchmarkSummary summary;
  for (const auto& timing : timings) {
    summary.total_interval_ms += timing.interval_ms;
    summary.total_rollout_ms += timing.rollout_ms;
    summary.any_success = summary.any_success || timing.success;

    if (timing.loaded_plan_attempted) {
      ++summary.loaded_plan_attempted_count;
      summary.total_loaded_plan_ms += timing.loaded_plan_ms;
    }
    if (timing.loaded_plan_success) {
      ++summary.loaded_plan_success_count;
      if (summary.loaded_plan_first_success_rank == 0) {
        summary.loaded_plan_first_success_rank = timing.loaded_plan_rank;
        summary.loaded_plan_first_success_candidate_order = timing.candidate_order;
      }
    }
  }

  summary.mean_interval_ms = timings.size() > 1
    ? summary.total_interval_ms / static_cast<double>(timings.size() - 1)
    : 0.0;
  summary.mean_rollout_ms = !timings.empty()
    ? summary.total_rollout_ms / static_cast<double>(timings.size())
    : 0.0;
  summary.mean_loaded_plan_ms = summary.loaded_plan_attempted_count > 0
    ? summary.total_loaded_plan_ms / static_cast<double>(summary.loaded_plan_attempted_count)
    : 0.0;
  return summary;
}

}  // namespace alfa_robot::motion
