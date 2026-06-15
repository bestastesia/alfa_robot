#include "alfa_robot_moveit_config/extract_benchmark_runner.hpp"

#include "alfa_robot_moveit_config/extract_benchmark_csv_writer.hpp"
#include "alfa_robot_moveit_config/extract_benchmark_summary.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

namespace alfa_robot::motion
{

ExtractBenchmarkRunner::ExtractBenchmarkRunner(
  ExtractBenchmarkRunnerConfig config,
  ExtractBenchmarkRunnerCallbacks callbacks)
: config_(std::move(config)), callbacks_(std::move(callbacks))
{}

std::vector<ik_benchmark::UpdownAwareIkCandidate> ExtractBenchmarkRunner::sortedLegalCandidates(
  const ik_benchmark::UpdownAwareIkResult& ik_result,
  size_t* original_legal_count,
  IkCandidateSelectionStats* dedup_stats) const
{
  std::vector<ik_benchmark::UpdownAwareIkCandidate> legal_candidates;
  for (const auto& candidate : ik_result.candidates) {
    if (candidate.legal) {
      legal_candidates.push_back(candidate);
    }
  }
  std::sort(legal_candidates.begin(), legal_candidates.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.score < rhs.score; });
  if (original_legal_count) {
    *original_legal_count = legal_candidates.size();
  }
  if (config_.candidate_selector) {
    legal_candidates = config_.candidate_selector->select(legal_candidates, dedup_stats);
  } else if (dedup_stats) {
    dedup_stats->input_count = legal_candidates.size();
    dedup_stats->unique_count = legal_candidates.size();
    dedup_stats->selected_count = legal_candidates.size();
  }
  return legal_candidates;
}

bool ExtractBenchmarkRunner::writeCsv(const std::vector<ExtractRolloutTiming>& timings) const
{
  return ExtractBenchmarkCsvWriter::write(config_.csv_path, timings, config_.logger);
}

bool ExtractBenchmarkRunner::runLeft(
  const std::string& prefix,
  const moveit::core::RobotState& seed_state,
  const ik_benchmark::UpdownAwareIkResult& ik_result,
  const AttachedBoxSpec& left_box,
  int left_box_id)
{
  size_t original_legal_count = 0;
  IkCandidateSelectionStats dedup_stats;
  auto legal_candidates = sortedLegalCandidates(ik_result, &original_legal_count, &dedup_stats);
  if (legal_candidates.empty()) {
    return callbacks_.fail ? callbacks_.fail(prefix + "/extract_benchmark: no legal IK candidates") : false;
  }

  if (config_.record_tip_error_ik_candidates && callbacks_.record_tip_errors) {
    callbacks_.record_tip_errors(prefix, seed_state, ik_result, left_box);
  }

  std::vector<ExtractRolloutTiming> timings;
  timings.reserve(legal_candidates.size());
  auto previous_start = std::chrono::steady_clock::now();
  bool any_success = false;
  for (size_t i = 0; i < legal_candidates.size(); ++i) {
    const auto start = std::chrono::steady_clock::now();
    auto state = callbacks_.state_from_candidate(seed_state, legal_candidates[i]);
    std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)> record_step;
    if (config_.record_rollouts && callbacks_.record_keyframe) {
      record_step = [&](size_t step_index, const moveit::core::RobotState& rollout_state, const nlohmann::json& extra) {
        callbacks_.record_keyframe(
          prefix + "/candidate_" + std::to_string(i) + "/step_" + std::to_string(step_index),
          rollout_state,
          std::vector<AttachedBoxSpec>{left_box},
          extra);
      };
    }
    auto timing = callbacks_.rollout_left(
      state, left_box, left_box_id, i, legal_candidates[i], record_step);
    if (timing.success && timing.final_state && callbacks_.fill_loaded_metrics) {
      callbacks_.fill_loaded_metrics(timing);
    }
    timing.interval_ms = std::chrono::duration<double, std::milli>(start - previous_start).count();
    previous_start = start;
    any_success = any_success || timing.success;
    timings.push_back(std::move(timing));
  }
  if (!timings.empty()) {
    timings.front().interval_ms = 0.0;
  }

  LoadedPoseBatchPlanOptions loaded_options = config_.loaded_options;
  loaded_options.stop_on_first_success = false;
  const LoadedPoseBatchPlanResult loaded_batch = config_.loaded_pose_planner
    ? config_.loaded_pose_planner->planBatch(prefix, timings, std::vector<AttachedBoxSpec>{left_box}, loaded_options)
    : LoadedPoseBatchPlanResult{};

  writeCsv(timings);

  const ExtractBenchmarkSummary summary = summarize_extract_timings(timings);
  RCLCPP_INFO(config_.logger,
              "[%s/extract_benchmark] legal_ik=%zu selected=%zu dedup=%s unique=%zu removed=%zu dedup_ms=%.3f success_any=%s loaded_plan=%zu/%zu mean_interval=%.3fms mean_rollout=%.3fms mean_loaded_plan=%.3fms",
              prefix.c_str(), original_legal_count, timings.size(),
              dedup_stats.enabled ? "true" : "false", dedup_stats.unique_count,
              dedup_stats.removed_count, dedup_stats.elapsed_ms,
              any_success ? "true" : "false",
              summary.loaded_plan_success_count, summary.loaded_plan_attempted_count,
              summary.mean_interval_ms, summary.mean_rollout_ms, summary.mean_loaded_plan_ms);

  if (callbacks_.record_summary) {
    callbacks_.record_summary(nlohmann::json({
      {"type", "extract_benchmark_summary"},
      {"legal_ik_count", original_legal_count},
      {"tested_ik_count", legal_candidates.size()},
      {"candidate_limit", config_.candidate_limit},
      {"ik_dedup_enabled", dedup_stats.enabled},
      {"ik_dedup_joint_threshold_deg", config_.dedup_joint_threshold_rad * 180.0 / M_PI},
      {"ik_dedup_h_threshold", config_.dedup_h_threshold},
      {"ik_dedup_input_count", dedup_stats.input_count},
      {"ik_dedup_unique_count", dedup_stats.unique_count},
      {"ik_dedup_removed_count", dedup_stats.removed_count},
      {"ik_dedup_selected_count", dedup_stats.selected_count},
      {"ik_dedup_ms", dedup_stats.elapsed_ms},
      {"success_any", any_success},
      {"mean_interval_ms", summary.mean_interval_ms},
      {"mean_rollout_ms", summary.mean_rollout_ms},
      {"loaded_plan_after_success", config_.plan_loaded_after_success},
      {"loaded_plan_candidate_limit", config_.loaded_options.candidate_limit},
      {"loaded_plan_sort_by_pose_distance", config_.loaded_options.sort_by_pose_distance},
      {"loaded_plan_sorted_success_candidate_count", loaded_batch.plan_indices.size()},
      {"loaded_plan_attempted_count", summary.loaded_plan_attempted_count},
      {"loaded_plan_success_count", summary.loaded_plan_success_count},
      {"mean_loaded_plan_ms", summary.mean_loaded_plan_ms},
      {"csv_path", config_.csv_path},
      {"rollouts_recorded", config_.record_rollouts},
      {"ik_candidate_rejection_counts", callbacks_.rejection_counts_json ? callbacks_.rejection_counts_json(ik_result) : nlohmann::json::object()}
    }));
  }
  return any_success;
}

ExtractRolloutTiming ExtractBenchmarkRunner::runDualCandidate(
  const std::string& prefix,
  const moveit::core::RobotState& seed_state,
  const std::vector<ik_benchmark::UpdownAwareIkCandidate>& legal_candidates,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  size_t index,
  bool record_rollout) const
{
  auto state = callbacks_.state_from_candidate(seed_state, legal_candidates[index]);
  std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)> record_step;
  if (record_rollout && callbacks_.record_keyframe) {
    const std::vector<AttachedBoxSpec> boxes{left_box, right_box};
    record_step = [&, boxes, index](size_t step_index, const moveit::core::RobotState& rollout_state, const nlohmann::json& extra) {
      callbacks_.record_keyframe(
        prefix + "/candidate_" + std::to_string(index) + "/step_" + std::to_string(step_index),
        rollout_state,
        boxes,
        extra);
    };
  }
  auto timing = callbacks_.rollout_dual(
    state, left_box, left_box_id, right_box, right_box_id,
    index, legal_candidates[index], record_step);
  if (timing.success && timing.final_state && callbacks_.fill_loaded_metrics) {
    callbacks_.fill_loaded_metrics(timing);
  }
  return timing;
}

bool ExtractBenchmarkRunner::runDual(
  const std::string& prefix,
  const moveit::core::RobotState& seed_state,
  const ik_benchmark::UpdownAwareIkResult& ik_result,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id)
{
  size_t original_legal_count = 0;
  IkCandidateSelectionStats dedup_stats;
  auto legal_candidates = sortedLegalCandidates(ik_result, &original_legal_count, &dedup_stats);
  if (legal_candidates.empty()) {
    return callbacks_.fail ? callbacks_.fail(prefix + "/dual_extract_benchmark: no legal IK candidates") : false;
  }

  std::vector<ExtractRolloutTiming> timings(legal_candidates.size());
  bool any_success = false;
  const size_t requested_extract_workers = std::max<size_t>(1, config_.extract_workers);
  const size_t used_extract_workers = config_.record_rollouts
    ? 1
    : std::max<size_t>(1, std::min(requested_extract_workers, legal_candidates.size()));
  const auto extract_wall_start = std::chrono::steady_clock::now();
  if (used_extract_workers <= 1) {
    auto previous_start = extract_wall_start;
    for (size_t i = 0; i < legal_candidates.size(); ++i) {
      const auto start = std::chrono::steady_clock::now();
      auto timing = runDualCandidate(
        prefix, seed_state, legal_candidates, left_box, left_box_id, right_box, right_box_id,
        i, config_.record_rollouts);
      timing.interval_ms = std::chrono::duration<double, std::milli>(start - previous_start).count();
      previous_start = start;
      timings[i] = std::move(timing);
    }
    if (!timings.empty()) {
      timings.front().interval_ms = 0.0;
    }
  } else {
    std::atomic<size_t> next_index{0};
    std::vector<std::thread> workers;
    workers.reserve(used_extract_workers);
    for (size_t worker_index = 0; worker_index < used_extract_workers; ++worker_index) {
      workers.emplace_back([&, worker_index]() {
        (void)worker_index;
        while (true) {
          const size_t i = next_index.fetch_add(1);
          if (i >= legal_candidates.size()) {
            break;
          }
          timings[i] = runDualCandidate(
            prefix, seed_state, legal_candidates, left_box, left_box_id, right_box, right_box_id,
            i, false);
        }
      });
    }
    for (auto& worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    for (auto& timing : timings) {
      timing.interval_ms = 0.0;
    }
  }
  const auto extract_wall_end = std::chrono::steady_clock::now();
  const double extract_wall_ms =
    std::chrono::duration<double, std::milli>(extract_wall_end - extract_wall_start).count();
  for (const auto& timing : timings) {
    any_success = any_success || timing.success;
  }

  const LoadedPoseBatchPlanResult loaded_batch = config_.loaded_pose_planner
    ? config_.loaded_pose_planner->planBatch(
        prefix, timings, std::vector<AttachedBoxSpec>{left_box, right_box},
        config_.loaded_options)
    : LoadedPoseBatchPlanResult{};
  const double loaded_plan_wall_ms = loaded_batch.wall_ms;

  writeCsv(timings);

  const ExtractBenchmarkSummary summary = summarize_extract_timings(timings);
  const double task_wall_ms =
    ik_result.wall_ms + dedup_stats.elapsed_ms + extract_wall_ms + loaded_plan_wall_ms;
  const bool benchmark_success = config_.plan_loaded_after_success
    ? summary.loaded_plan_success_count > 0
    : any_success;
  if (!benchmark_success && callbacks_.set_last_error) {
    callbacks_.set_last_error(prefix + "/dual_extract_benchmark: " +
      std::string(any_success ? "loaded_plan_failed" : "extract_failed"));
  }

  RCLCPP_INFO(config_.logger,
              "[%s/dual_extract_benchmark] legal_ik=%zu selected=%zu dedup=%s unique=%zu removed=%zu dedup_ms=%.3f extract=%zu workers wall=%.3fms success_any=%s loaded_plan=%zu/%zu wall=%.3fms task_wall=%.3fms mean_interval=%.3fms mean_rollout=%.3fms mean_loaded_plan=%.3fms",
              prefix.c_str(), original_legal_count, timings.size(),
              dedup_stats.enabled ? "true" : "false", dedup_stats.unique_count,
              dedup_stats.removed_count, dedup_stats.elapsed_ms,
              used_extract_workers, extract_wall_ms,
              any_success ? "true" : "false",
              summary.loaded_plan_success_count, summary.loaded_plan_attempted_count,
              loaded_plan_wall_ms, task_wall_ms,
              summary.mean_interval_ms, summary.mean_rollout_ms, summary.mean_loaded_plan_ms);

  if (callbacks_.record_summary) {
    callbacks_.record_summary(nlohmann::json({
      {"type", "extract_benchmark_summary"},
      {"stage", prefix},
      {"mode", "dual_extract"},
      {"dual_async", config_.dual_async},
      {"legal_ik_count", original_legal_count},
      {"tested_ik_count", legal_candidates.size()},
      {"candidate_limit", config_.candidate_limit},
      {"ik_dedup_enabled", dedup_stats.enabled},
      {"ik_dedup_joint_threshold_deg", config_.dedup_joint_threshold_rad * 180.0 / M_PI},
      {"ik_dedup_h_threshold", config_.dedup_h_threshold},
      {"ik_dedup_input_count", dedup_stats.input_count},
      {"ik_dedup_unique_count", dedup_stats.unique_count},
      {"ik_dedup_removed_count", dedup_stats.removed_count},
      {"ik_dedup_selected_count", dedup_stats.selected_count},
      {"ik_dedup_ms", dedup_stats.elapsed_ms},
      {"ik_wall_ms", ik_result.wall_ms},
      {"extract_parallel_requested_workers", requested_extract_workers},
      {"extract_parallel_used_workers", used_extract_workers},
      {"extract_parallel_enabled", used_extract_workers > 1},
      {"extract_wall_ms", extract_wall_ms},
      {"extract_sum_rollout_ms", summary.total_rollout_ms},
      {"task_success", benchmark_success},
      {"success_any", any_success},
      {"mean_interval_ms", summary.mean_interval_ms},
      {"mean_rollout_ms", summary.mean_rollout_ms},
      {"loaded_plan_after_success", config_.plan_loaded_after_success},
      {"loaded_plan_candidate_limit", config_.loaded_options.candidate_limit},
      {"loaded_plan_sort_by_pose_distance", config_.loaded_options.sort_by_pose_distance},
      {"loaded_plan_stop_on_first_success", config_.loaded_options.stop_on_first_success},
      {"loaded_plan_sorted_success_candidate_count", loaded_batch.plan_indices.size()},
      {"loaded_plan_attempted_count", summary.loaded_plan_attempted_count},
      {"loaded_plan_success_count", summary.loaded_plan_success_count},
      {"loaded_plan_first_success_rank", summary.loaded_plan_first_success_rank},
      {"loaded_plan_first_success_candidate_order", summary.loaded_plan_first_success_candidate_order},
      {"loaded_plan_wall_ms", loaded_plan_wall_ms},
      {"loaded_plan_sum_ms", summary.total_loaded_plan_ms},
      {"mean_loaded_plan_ms", summary.mean_loaded_plan_ms},
      {"task_wall_ms", task_wall_ms},
      {"csv_path", config_.csv_path},
      {"rollouts_recorded", config_.record_rollouts},
      {"ik_candidate_rejection_counts", callbacks_.rejection_counts_json ? callbacks_.rejection_counts_json(ik_result) : nlohmann::json::object()}
    }));
  }
  return benchmark_success;
}

}  // namespace alfa_robot::motion
