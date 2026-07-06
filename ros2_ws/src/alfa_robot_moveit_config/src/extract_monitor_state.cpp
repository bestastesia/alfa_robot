#include "alfa_robot_moveit_config/extract_monitor_state.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>
#include <utility>

namespace alfa_robot::motion
{

namespace
{

ExtractMonitorStageRunner runner_for_stage(
  ExtractMonitorStage stage,
  const ExtractMonitorStageCallbacks& callbacks)
{
  switch (stage) {
    case ExtractMonitorStage::Ik:
      return callbacks.ik;
    case ExtractMonitorStage::Extract:
      return callbacks.extract;
    case ExtractMonitorStage::Loaded:
      return callbacks.loaded;
    case ExtractMonitorStage::Final:
      return callbacks.final;
  }
  return {};
}

size_t stage_index(ExtractMonitorStage stage)
{
  switch (stage) {
    case ExtractMonitorStage::Ik:
      return 0;
    case ExtractMonitorStage::Extract:
      return 1;
    case ExtractMonitorStage::Loaded:
      return 2;
    case ExtractMonitorStage::Final:
      return 3;
  }
  return 0;
}

bool timing_has_loaded_plan(const ExtractRolloutTiming& timing)
{
  return timing.loaded_plan_success && timing.final_state;
}

}  // namespace

std::string extract_monitor_prefix(int left_box_id, int right_box_id)
{
  return "extract_monitor_L" + std::to_string(left_box_id) +
         "_R" + std::to_string(right_box_id);
}

ExtractMonitorState make_extract_monitor_initial_state(
  int left_box_id,
  int right_box_id,
  AttachedBoxSpec left_box,
  AttachedBoxSpec right_box,
  moveit::core::RobotStatePtr seed_state,
  moveit::core::RobotStatePtr loaded_start_state)
{
  ExtractMonitorState state;
  state.left_box_id = left_box_id;
  state.right_box_id = right_box_id;
  state.left_box = std::move(left_box);
  state.right_box = std::move(right_box);
  state.prefix = extract_monitor_prefix(left_box_id, right_box_id);
  state.seed_state = std::move(seed_state);
  state.loaded_start_state = std::move(loaded_start_state);
  return state;
}

ExtractMonitorState make_extract_monitor_initial_state(
  const ExtractMonitorInitialStateRequest& request)
{
  return make_extract_monitor_initial_state(
    request.left_box_id,
    request.right_box_id,
    request.left_box,
    request.right_box,
    std::make_shared<moveit::core::RobotState>(
      make_extract_monitor_joint_state(
        request.robot_model,
        request.joint_group,
        request.grasp_seed)),
    std::make_shared<moveit::core::RobotState>(
      make_extract_monitor_joint_state(
        request.robot_model,
        request.joint_group,
        request.loaded_seed)));
}

moveit::core::RobotState make_extract_monitor_joint_state(
  const moveit::core::RobotModelConstPtr& robot_model,
  const moveit::core::JointModelGroup* joint_group,
  const ExtractMonitorArmSeed& seed)
{
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  const auto& variable_names = robot_model->getVariableNames();
  const auto has_variable = [&](const std::string& name) {
    return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
  };
  for (size_t i = 0; i < seed.left_arm.size(); ++i) {
    const std::string name = "leftjoint" + std::to_string(i + 1);
    if (has_variable(name)) {
      state.setVariablePosition(name, seed.left_arm[i]);
    }
  }
  for (size_t i = 0; i < seed.right_arm.size(); ++i) {
    const std::string name = "rightjoint" + std::to_string(i + 1);
    if (has_variable(name)) {
      state.setVariablePosition(name, seed.right_arm[i]);
    }
  }
  if (has_variable("turn")) {
    state.setVariablePosition("turn", seed.turn);
  }
  if (has_variable("updown")) {
    state.setVariablePosition("updown", seed.updown);
  }
  if (joint_group) {
    state.enforceBounds(joint_group);
  } else {
    state.enforceBounds();
  }
  state.update();
  return state;
}

void populate_extract_monitor_candidate_states(
  ExtractMonitorState& state,
  const ExtractMonitorCandidateStateBuilder& state_builder)
{
  state.candidate_states.clear();
  state.candidate_states.reserve(state.legal_candidates.size());
  if (!state_builder) {
    return;
  }
  for (const auto& candidate : state.legal_candidates) {
    state.candidate_states.push_back(state_builder(candidate));
  }
}

size_t extract_monitor_worker_count(size_t candidate_count, size_t requested_worker_count)
{
  if (candidate_count == 0) {
    return 0;
  }
  return std::max<size_t>(1, std::min(requested_worker_count, candidate_count));
}

size_t run_extract_monitor_candidate_tasks(
  ExtractMonitorState& state,
  size_t requested_worker_count,
  const ExtractMonitorCandidateTask& task)
{
  const size_t count = state.legal_candidates.size();
  state.timings.clear();
  state.timings.resize(count);
  if (count == 0 || !task) {
    return 0;
  }

  const size_t worker_count = extract_monitor_worker_count(count, requested_worker_count);
  std::atomic<size_t> next_index{0};
  std::vector<std::thread> workers;
  workers.reserve(worker_count);
  for (size_t worker = 0; worker < worker_count; ++worker) {
    workers.emplace_back([&]() {
      while (true) {
        const size_t index = next_index.fetch_add(1);
        if (index >= count) {
          break;
        }
        state.timings[index] = task(index, state.legal_candidates[index]);
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }
  return worker_count;
}

const ik_benchmark::UpdownAwareIkCandidate* extract_monitor_candidate_for_timing(
  const ExtractMonitorState& state,
  const ExtractRolloutTiming& timing)
{
  if (timing.candidate_order >= state.legal_candidates.size()) {
    return nullptr;
  }
  return &state.legal_candidates[timing.candidate_order];
}

moveit::core::RobotStatePtr extract_monitor_candidate_state_for_timing(
  const ExtractMonitorState& state,
  const ExtractRolloutTiming& timing)
{
  if (timing.candidate_order >= state.candidate_states.size()) {
    return nullptr;
  }
  return state.candidate_states[timing.candidate_order];
}

ExtractMonitorTimingSummary summarize_extract_monitor_timings(
  const std::vector<ExtractRolloutTiming>& timings)
{
  ExtractMonitorTimingSummary summary;
  for (size_t i = 0; i < timings.size(); ++i) {
    const auto& timing = timings[i];
    if (timing.success && timing.final_state) {
      ++summary.success_count;
      summary.success_indices.push_back(i);
    } else {
      summary.failure_counts[timing.failure_reason.empty() ? "unknown" : timing.failure_reason]++;
    }
  }
  return summary;
}

ExtractMonitorLoadedPlanSummary summarize_loaded_plan_timings(
  const std::vector<ExtractRolloutTiming>& timings,
  const std::vector<size_t>& plan_indices)
{
  ExtractMonitorLoadedPlanSummary summary;
  for (const auto index : plan_indices) {
    if (index >= timings.size()) {
      continue;
    }
    const auto& timing = timings[index];
    if (timing.loaded_plan_attempted) {
      ++summary.attempted_count;
      summary.attempted_indices.push_back(index);
    }
    if (timing.loaded_plan_success && timing.final_state) {
      ++summary.success_count;
      summary.success_indices.push_back(index);
    } else {
      summary.failure_counts[
        timing.loaded_plan_failure_reason.empty() ? "unknown" : timing.loaded_plan_failure_reason]++;
    }
  }
  return summary;
}

std::string extract_monitor_ik_stage_message(
  size_t unique_count,
  size_t legal_count,
  size_t trial_count,
  double elapsed_ms,
  const std::string& snapshot_path)
{
  std::ostringstream out;
  out << "IK阶段完成: unique=" << unique_count
      << " legal=" << legal_count
      << " trials=" << trial_count
      << " elapsed=" << elapsed_ms << "ms snapshot=" << snapshot_path;
  return out.str();
}

std::string extract_monitor_ik_stage_message(
  const ExtractMonitorIkStageMessageRequest& request)
{
  return extract_monitor_ik_stage_message(
    request.unique_count,
    request.legal_count,
    request.trial_count,
    request.elapsed_ms,
    request.snapshot_path);
}

std::string extract_monitor_extract_stage_message(
  size_t success_count,
  size_t total_count,
  size_t worker_count,
  double elapsed_ms,
  const std::string& snapshot_path)
{
  std::ostringstream out;
  out << "抽离阶段完成: success=" << success_count << "/" << total_count
      << " workers=" << worker_count
      << " elapsed=" << elapsed_ms << "ms snapshot=" << snapshot_path;
  return out.str();
}

std::string extract_monitor_extract_stage_message(
  const ExtractMonitorExtractStageMessageRequest& request)
{
  return extract_monitor_extract_stage_message(
    request.success_count,
    request.total_count,
    request.worker_count,
    request.elapsed_ms,
    request.snapshot_path);
}

std::string extract_monitor_loaded_stage_message(
  size_t success_count,
  size_t attempted_count,
  size_t candidate_count,
  double elapsed_ms,
  const std::string& snapshot_path)
{
  std::ostringstream out;
  out << "负重规划阶段完成: success=" << success_count
      << " attempted=" << attempted_count
      << " candidates=" << candidate_count
      << " elapsed=" << elapsed_ms << "ms snapshot=" << snapshot_path;
  return out.str();
}

std::string extract_monitor_loaded_stage_message(
  const ExtractMonitorLoadedStageMessageRequest& request)
{
  return extract_monitor_loaded_stage_message(
    request.success_count,
    request.attempted_count,
    request.candidate_count,
    request.elapsed_ms,
    request.snapshot_path);
}

std::string extract_monitor_final_stage_message(
  const ExtractRolloutTiming& selected,
  double elapsed_ms,
  const std::string& snapshot_path)
{
  std::ostringstream out;
  out << "最终方案已选择: candidate_order=" << selected.candidate_order
      << " loaded_rank=" << selected.loaded_plan_rank
      << " trajectory_distance=" << selected.loaded_plan_trajectory_distance
      << " elapsed=" << elapsed_ms << "ms snapshot=" << snapshot_path;
  return out.str();
}

std::string extract_monitor_final_stage_message(
  const ExtractMonitorFinalStageMessageRequest& request)
{
  if (!request.selected) {
    std::ostringstream out;
    out << "最终方案选择失败: selected=null elapsed="
        << request.elapsed_ms << "ms snapshot=" << request.snapshot_path;
    return out.str();
  }
  return extract_monitor_final_stage_message(
    *request.selected,
    request.elapsed_ms,
    request.snapshot_path);
}

ExtractRolloutTiming* select_extract_monitor_final_timing(
  std::vector<ExtractRolloutTiming>& timings,
  const ExtractMonitorTimingPredicate& preferred_predicate)
{
  if (preferred_predicate) {
    for (auto& timing : timings) {
      if (timing_has_loaded_plan(timing) && preferred_predicate(timing)) {
        return &timing;
      }
    }
  }
  for (auto& timing : timings) {
    if (timing_has_loaded_plan(timing)) {
      return &timing;
    }
  }
  return nullptr;
}

const char* extract_monitor_stage_failure_label(ExtractMonitorStage stage)
{
  switch (stage) {
    case ExtractMonitorStage::Ik:
      return "IK阶段";
    case ExtractMonitorStage::Extract:
      return "抽离阶段";
    case ExtractMonitorStage::Loaded:
      return "负重规划阶段";
    case ExtractMonitorStage::Final:
      return "最终选择阶段";
  }
  return "未知阶段";
}

ExtractMonitorStage extract_monitor_stage_for_phase(ExtractMonitorPhase phase)
{
  switch (phase) {
    case ExtractMonitorPhase::ReadyForIk:
    case ExtractMonitorPhase::Done:
      return ExtractMonitorStage::Ik;
    case ExtractMonitorPhase::ReadyForExtract:
      return ExtractMonitorStage::Extract;
    case ExtractMonitorPhase::ReadyForLoaded:
      return ExtractMonitorStage::Loaded;
    case ExtractMonitorPhase::ReadyForFinal:
      return ExtractMonitorStage::Final;
  }
  return ExtractMonitorStage::Ik;
}

ExtractMonitorPhase extract_monitor_phase_before_running(ExtractMonitorPhase phase)
{
  if (phase == ExtractMonitorPhase::Done) {
    return ExtractMonitorPhase::ReadyForIk;
  }
  return phase;
}

ExtractMonitorPhase extract_monitor_next_phase_after(ExtractMonitorStage stage)
{
  switch (stage) {
    case ExtractMonitorStage::Ik:
      return ExtractMonitorPhase::ReadyForExtract;
    case ExtractMonitorStage::Extract:
      return ExtractMonitorPhase::ReadyForLoaded;
    case ExtractMonitorStage::Loaded:
      return ExtractMonitorPhase::ReadyForFinal;
    case ExtractMonitorStage::Final:
      return ExtractMonitorPhase::Done;
  }
  return ExtractMonitorPhase::ReadyForIk;
}

bool run_extract_monitor_stage(
  ExtractMonitorStage stage,
  const ExtractMonitorStageCallbacks& callbacks,
  const std::function<double()>& last_stage_ms,
  double* elapsed_ms,
  std::string* message)
{
  const auto runner = runner_for_stage(stage, callbacks);
  if (!runner) {
    if (message) {
      *message = "extract monitor: missing stage runner";
    }
    if (elapsed_ms) {
      *elapsed_ms = 0.0;
    }
    return false;
  }
  const bool ok = runner(message);
  if (elapsed_ms) {
    *elapsed_ms = last_stage_ms ? last_stage_ms() : 0.0;
  }
  return ok;
}

ExtractMonitorFullRunResult run_extract_monitor_full_sequence(
  const ExtractMonitorStageCallbacks& callbacks,
  const std::function<double()>& last_stage_ms)
{
  ExtractMonitorFullRunResult result;
  const auto total_start = std::chrono::steady_clock::now();
  constexpr std::array<ExtractMonitorStage, 4> stages{
    ExtractMonitorStage::Ik,
    ExtractMonitorStage::Extract,
    ExtractMonitorStage::Loaded,
    ExtractMonitorStage::Final,
  };

  for (const auto stage : stages) {
    std::string stage_message;
    double elapsed_ms = 0.0;
    if (!run_extract_monitor_stage(stage, callbacks, last_stage_ms, &elapsed_ms, &stage_message)) {
      result.message = std::string("完整流程失败在") + extract_monitor_stage_failure_label(stage) + ": " + stage_message;
      result.success = false;
      return result;
    }
    result.stage_elapsed_ms[stage_index(stage)] = elapsed_ms;
  }

  result.total_elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - total_start).count();
  std::ostringstream out;
  out << "完整流程完成: total=" << result.total_elapsed_ms << "ms"
      << " ik=" << result.stage_elapsed_ms[0] << "ms"
      << " extract=" << result.stage_elapsed_ms[1] << "ms"
      << " loaded=" << result.stage_elapsed_ms[2] << "ms"
      << " final=" << result.stage_elapsed_ms[3] << "ms";
  result.message = out.str();
  result.success = true;
  return result;
}

bool ExtractMonitorController::runNext(
  const ExtractMonitorStageCallbacks& callbacks,
  const std::function<double()>& last_stage_ms,
  std::string* message)
{
  const auto stage = extract_monitor_stage_for_phase(phase_);
  phase_ = extract_monitor_phase_before_running(phase_);
  double elapsed_ms = 0.0;
  const bool ok = run_extract_monitor_stage(stage, callbacks, last_stage_ms, &elapsed_ms, message);
  if (ok) {
    phase_ = extract_monitor_next_phase_after(stage);
  }
  return ok;
}

ExtractMonitorFullRunResult ExtractMonitorController::runFull(
  const ExtractMonitorStageCallbacks& callbacks,
  const std::function<double()>& last_stage_ms)
{
  reset();
  ExtractMonitorFullRunResult result;
  const auto total_start = std::chrono::steady_clock::now();
  constexpr std::array<ExtractMonitorStage, 4> stages{
    ExtractMonitorStage::Ik,
    ExtractMonitorStage::Extract,
    ExtractMonitorStage::Loaded,
    ExtractMonitorStage::Final,
  };

  for (const auto stage : stages) {
    std::string stage_message;
    double elapsed_ms = 0.0;
    if (!run_extract_monitor_stage(stage, callbacks, last_stage_ms, &elapsed_ms, &stage_message)) {
      result.message = std::string("完整流程失败在") + extract_monitor_stage_failure_label(stage) + ": " + stage_message;
      result.success = false;
      return result;
    }
    result.stage_elapsed_ms[stage_index(stage)] = elapsed_ms;
    phase_ = extract_monitor_next_phase_after(stage);
  }

  result.total_elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - total_start).count();
  std::ostringstream out;
  out << "完整流程完成: total=" << result.total_elapsed_ms << "ms"
      << " ik=" << result.stage_elapsed_ms[0] << "ms"
      << " extract=" << result.stage_elapsed_ms[1] << "ms"
      << " loaded=" << result.stage_elapsed_ms[2] << "ms"
      << " final=" << result.stage_elapsed_ms[3] << "ms";
  result.message = out.str();
  result.success = true;
  reset();
  return result;
}

}  // namespace alfa_robot::motion
