#include <alfa_robot_moveit_config/wall_trajectory_postprocessing.hpp>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace alfa_robot::motion
{
namespace
{
constexpr double kEpsilon = 1e-9;
constexpr double kMinimumJointMotion = 1e-6;

bool fail(std::string* reason, std::string message)
{
  if (reason) *reason = std::move(message);
  return false;
}

bool sameMetadata(const TrajectoryFrame& left, const TrajectoryFrame& right)
{
  return left.stage == right.stage && left.box_attached == right.box_attached &&
    left.box_visible == right.box_visible && left.scene_index == right.scene_index &&
    left.carried_boxes == right.carried_boxes;
}

bool sameLifecycle(const TrajectoryFrame& left, const TrajectoryFrame& right)
{
  return left.box_attached == right.box_attached && left.box_visible == right.box_visible &&
    left.scene_index == right.scene_index && left.carried_boxes == right.carried_boxes;
}

bool loaded(const TrajectoryFrame& frame)
{
  if (frame.box_attached) return true;
  if (!frame.carried_boxes.is_array()) return false;
  return std::any_of(frame.carried_boxes.begin(), frame.carried_boxes.end(),
    [](const nlohmann::json& box) { return box.is_object() && box.value("attached", false); });
}

bool sameJoints(const TrajectoryFrame& left, const TrajectoryFrame& right)
{
  return left.joints == right.joints;
}

bool deterministicShortcut(
  const std::vector<TrajectoryFrame>& input,
  const EdgeValidator& edge_validator,
  std::vector<TrajectoryFrame>* output,
  std::string* reason)
{
  output->clear();
  for (size_t begin = 0; begin < input.size();) {
    size_t end = begin + 1;
    while (end < input.size() && sameMetadata(input[begin], input[end])) ++end;

    output->push_back(input[begin]);
    size_t current = begin;
    while (current + 1 < end) {
      size_t candidate = end - 1;
      std::string edge_reason;
      while (candidate > current + 1 &&
          !edge_validator(input[current], input[candidate], &edge_reason))
        candidate = current + (candidate - current + 1) / 2;
      if (candidate == current + 1 &&
          !edge_validator(input[current], input[candidate], &edge_reason)) {
        return fail(reason, "shortcut failed at frame " + std::to_string(current) + " -> " +
          std::to_string(candidate) + (edge_reason.empty() ? "" : ": " + edge_reason));
      }
      output->push_back(input[candidate]);
      current = candidate;
    }
    begin = end;
  }
  return true;
}

bool insertMetadataTransitions(
  const std::vector<TrajectoryFrame>& input,
  std::vector<TrajectoryFrame>* output,
  std::string* reason)
{
  output->clear();
  output->push_back(input.front());
  for (size_t index = 1; index < input.size(); ++index) {
    const auto& previous = output->back();
    const auto& next = input[index];
    if (!sameMetadata(previous, next) && !sameJoints(previous, next)) {
      if (!sameLifecycle(previous, next)) {
        return fail(reason, "lifecycle transition changes joints at frame " +
          std::to_string(index - 1) + " -> " + std::to_string(index));
      }
      TrajectoryFrame transition = next;
      transition.joints = previous.joints;
      output->push_back(std::move(transition));
    }
    output->push_back(next);
  }
  return true;
}

bool validateFrameValues(
  const std::vector<TrajectoryFrame>& frames,
  const std::vector<std::string>& joint_names,
  std::string* reason)
{
  for (size_t index = 0; index < frames.size(); ++index) {
    if (frames[index].joints.size() != joint_names.size())
      return fail(reason, "joint count mismatch at frame " + std::to_string(index));
    if (!std::all_of(frames[index].joints.begin(), frames[index].joints.end(),
        [](double value) { return std::isfinite(value); }))
      return fail(reason, "non-finite joint value at frame " + std::to_string(index));
  }
  return true;
}

bool validateFrames(
  const std::vector<TrajectoryFrame>& frames,
  const std::vector<std::string>& joint_names,
  const EdgeValidator& edge_validator,
  std::string* reason)
{
  if (!validateFrameValues(frames, joint_names, reason)) return false;
  for (size_t index = 1; index < frames.size(); ++index) {
    std::string edge_reason;
    if (!edge_validator(frames[index - 1], frames[index], &edge_reason)) {
      return fail(reason, "edge validation failed at frame " + std::to_string(index - 1) +
        " -> " + std::to_string(index) +
        (edge_reason.empty() ? "" : ": " + edge_reason));
    }
  }
  return true;
}

bool makeLimits(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::vector<std::string>& variables,
  double velocity_scaling,
  double acceleration_scaling,
  const TrajectoryPostprocessOptions& options,
  std::unordered_map<std::string, double>* velocity_limits,
  std::unordered_map<std::string, double>* acceleration_limits,
  std::unordered_map<std::string, double>* jerk_limits,
  std::string* reason)
{
  velocity_limits->clear();
  acceleration_limits->clear();
  jerk_limits->clear();
  for (const auto& name : variables) {
    const auto& bounds = robot_model->getVariableBounds(name);
    const double velocity = std::max(std::abs(bounds.min_velocity_), std::abs(bounds.max_velocity_));
    const double acceleration = std::max(
      std::abs(bounds.min_acceleration_), std::abs(bounds.max_acceleration_));
    if (!bounds.velocity_bounded_ || !std::isfinite(velocity) || velocity <= 0.0)
      return fail(reason, "missing positive velocity limit for " + name);
    if (!bounds.acceleration_bounded_ || !std::isfinite(acceleration) || acceleration <= 0.0)
      return fail(reason, "missing positive acceleration limit for " + name);
    const double jerk = name == "updown" ? options.updown_max_jerk :
      (name == "head_joint" ? options.head_max_jerk : options.arm_max_jerk);
    (*velocity_limits)[name] = velocity * velocity_scaling;
    (*acceleration_limits)[name] = acceleration * acceleration_scaling;
    (*jerk_limits)[name] = jerk;
  }
  return true;
}

struct TimedRun
{
  size_t begin = 0;
  size_t end = 0;
  double start_time = 0.0;
  double duration = 0.0;
  std::shared_ptr<robot_trajectory::RobotTrajectory> trajectory;
};


void accumulateTimedTrajectoryMetrics(
  const robot_trajectory::RobotTrajectory& trajectory,
  TrajectoryPostprocessMetrics* metrics)
{
  const auto& indices = trajectory.getGroup()->getVariableIndexList();
  std::vector<double> previous_acceleration;
  for (size_t waypoint = 0; waypoint < trajectory.getWayPointCount(); ++waypoint) {
    const auto& state = trajectory.getWayPoint(waypoint);
    std::vector<double> acceleration;
    acceleration.reserve(indices.size());
    for (const int variable : indices) {
      metrics->max_velocity = std::max(
        metrics->max_velocity, std::abs(state.getVariableVelocity(variable)));
      const double value = state.getVariableAcceleration(variable);
      metrics->max_acceleration = std::max(metrics->max_acceleration, std::abs(value));
      acceleration.push_back(value);
    }
    if (!previous_acceleration.empty()) {
      const double dt = trajectory.getWayPointDurationFromPrevious(waypoint);
      if (dt > kEpsilon)
        for (size_t joint = 0; joint < acceleration.size(); ++joint)
          metrics->max_jerk = std::max(metrics->max_jerk,
            std::abs((acceleration[joint] - previous_acceleration[joint]) / dt));
    }
    previous_acceleration = std::move(acceleration);
  }
}

bool makeTimedRun(
  const std::vector<TrajectoryFrame>& frames,
  size_t begin,
  size_t end,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::vector<std::string>& joint_names,
  const TrajectoryPostprocessOptions& options,
  double start_time,
  TrajectoryPostprocessMetrics* metrics,
  TimedRun* run,
  std::string* reason)
{
  run->begin = begin;
  run->end = end;
  run->start_time = start_time;

  bool moving = false;
  for (size_t index = begin + 1; index < end; ++index)
    for (size_t joint = 0; joint < joint_names.size(); ++joint)
      moving = moving || std::abs(
        frames[index].joints[joint] - frames[begin].joints[joint]) > kMinimumJointMotion;
  if (!moving) return true;

  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, "whole_body");
  for (size_t index = begin; index < end; ++index) {
    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();
    for (size_t joint = 0; joint < joint_names.size(); ++joint)
      state.setVariablePosition(joint_names[joint], frames[index].joints[joint]);
    state.update(true);
    trajectory->addSuffixWayPoint(state, 0.0);
  }

  std::unordered_map<std::string, double> velocity_limits, acceleration_limits, jerk_limits;
  const double velocity_scaling = loaded(frames[begin]) ?
    options.loaded_velocity_scaling : options.empty_velocity_scaling;
  const double acceleration_scaling = loaded(frames[begin]) ?
    options.loaded_acceleration_scaling : options.empty_acceleration_scaling;
  if (!makeLimits(robot_model, trajectory->getGroup()->getVariableNames(), velocity_scaling,
      acceleration_scaling, options, &velocity_limits, &acceleration_limits, &jerk_limits, reason))
    return false;

  const auto totg_started = std::chrono::steady_clock::now();
  const trajectory_processing::TimeOptimalTrajectoryGeneration totg(
    0.0, options.sample_period, kMinimumJointMotion);
  const bool totg_ok = totg.computeTimeStamps(*trajectory, velocity_limits, acceleration_limits);
  metrics->totg_ms += std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - totg_started).count();
  if (!totg_ok) return fail(reason, "TOTG failed for stage " + frames[begin].stage);

  const auto ruckig_started = std::chrono::steady_clock::now();
  const robot_trajectory::RobotTrajectory totg_trajectory(*trajectory, true);
  bool ruckig_ok = false;
  for (const double time_scale : {1.0, 10.0, 100.0}) {
    *trajectory = robot_trajectory::RobotTrajectory(totg_trajectory, true);
    if (time_scale > 1.0) {
      const auto& indices = trajectory->getGroup()->getVariableIndexList();
      for (size_t index = 0; index < trajectory->getWayPointCount(); ++index) {
        auto state = trajectory->getWayPointPtr(index);
        for (const int variable : indices) {
          state->setVariableVelocity(variable, state->getVariableVelocity(variable) / time_scale);
          state->setVariableAcceleration(
            variable, state->getVariableAcceleration(variable) / (time_scale * time_scale));
        }
        if (index)
          trajectory->setWayPointDurationFromPrevious(
            index, totg_trajectory.getWayPointDurationFromPrevious(index) * time_scale);
        state->update();
      }
    }
    if (trajectory_processing::RuckigSmoothing::applySmoothing(
        *trajectory, velocity_limits, acceleration_limits, jerk_limits)) {
      ruckig_ok = true;
      break;
    }
  }
  metrics->ruckig_ms += std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - ruckig_started).count();
  if (!ruckig_ok) return fail(reason, "Ruckig failed for stage " + frames[begin].stage);

  run->duration = trajectory->getWayPointDurationFromStart(trajectory->getWayPointCount() - 1);
  if (!std::isfinite(run->duration) || run->duration <= 0.0)
    return fail(reason, "non-positive timed duration for stage " + frames[begin].stage);
  accumulateTimedTrajectoryMetrics(*trajectory, metrics);
  run->trajectory = std::move(trajectory);
  return true;
}

void appendFrame(std::vector<TrajectoryFrame>* frames, TrajectoryFrame frame)
{
  if (!frames->empty() && sameMetadata(frames->back(), frame) &&
      sameJoints(frames->back(), frame) &&
      std::abs(frames->back().time_from_start_s - frame.time_from_start_s) <= kEpsilon)
    return;
  frames->push_back(std::move(frame));
}

bool sampleRuns(
  const std::vector<TrajectoryFrame>& frames,
  const std::vector<TimedRun>& runs,
  const std::vector<std::string>& joint_names,
  double sample_period,
  std::vector<TrajectoryFrame>* sampled,
  std::string* reason)
{
  sampled->clear();
  TrajectoryFrame first = frames.front();
  first.time_from_start_s = 0.0;
  sampled->push_back(std::move(first));
  double next_sample_time = sample_period;

  for (const auto& run : runs) {
    TrajectoryFrame run_start = frames[run.begin];
    run_start.time_from_start_s = run.start_time;
    appendFrame(sampled, std::move(run_start));

    const double end_time = run.start_time + run.duration;
    if (run.trajectory) {
      while (next_sample_time < end_time - kEpsilon) {
        if (next_sample_time > run.start_time + kEpsilon) {
          moveit::core::RobotStatePtr state =
            std::make_shared<moveit::core::RobotState>(run.trajectory->getRobotModel());
          if (!run.trajectory->getStateAtDurationFromStart(
              next_sample_time - run.start_time, state) || !state) {
            return fail(reason, "failed to sample stage " + frames[run.begin].stage);
          }
          TrajectoryFrame frame = frames[run.begin];
          frame.joints.clear();
          frame.joints.reserve(joint_names.size());
          for (const auto& name : joint_names)
            frame.joints.push_back(state->getVariablePosition(name));
          frame.time_from_start_s = next_sample_time;
          appendFrame(sampled, std::move(frame));
        }
        next_sample_time += sample_period;
      }
      if (std::abs(next_sample_time - end_time) <= kEpsilon)
        next_sample_time += sample_period;
    }

    TrajectoryFrame run_end = frames[run.end - 1];
    run_end.time_from_start_s = end_time;
    appendFrame(sampled, std::move(run_end));
  }
  return true;
}

}  // namespace

bool postprocessWallTrajectory(
  std::vector<TrajectoryFrame>* frames,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::vector<std::string>& joint_names,
  const TrajectoryPostprocessOptions& options,
  const EdgeValidator& edge_validator,
  const PathValidator& path_validator,
  const std::function<bool(std::vector<TrajectoryFrame>*, double*, std::string*)>& chomp_optimizer,
  TrajectoryPostprocessMetrics* metrics,
  std::string* reason)
{
  if (reason) reason->clear();
  if (!frames || !metrics || frames->empty()) return fail(reason, "empty trajectory");
  *metrics = TrajectoryPostprocessMetrics{};
  if (!std::isfinite(options.sample_period) || options.sample_period <= 0.0)
    return fail(reason, "sample_period must be positive");

  if (options.variant == "topk") {
    for (size_t index = 0; index < frames->size(); ++index)
      (*frames)[index].time_from_start_s = index * options.sample_period;
    metrics->execution_duration_s = frames->back().time_from_start_s;
    return true;
  }
  if (options.variant != "shortcut_ruckig" && options.variant != "chomp_ruckig")
    return fail(reason, "unsupported trajectory variant: " + options.variant);
  if (!robot_model) return fail(reason, "shortcut_ruckig requires a robot model");
  if (!edge_validator) return fail(reason, "shortcut_ruckig requires an edge validator");
  if (joint_names.empty()) return fail(reason, "shortcut_ruckig requires joint names");
  for (const double value : {options.empty_velocity_scaling, options.empty_acceleration_scaling,
      options.loaded_velocity_scaling, options.loaded_acceleration_scaling}) {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
      return fail(reason, "velocity and acceleration scaling must be in (0, 1]");
  }
  for (const double value : {options.arm_max_jerk, options.head_max_jerk, options.updown_max_jerk})
    if (!std::isfinite(value) || value <= 0.0) return fail(reason, "jerk limits must be positive");

  const auto* group = robot_model->getJointModelGroup("whole_body");
  if (!group) return fail(reason, "whole_body planning group is unavailable");
  const std::unordered_set<std::string> provided(joint_names.begin(), joint_names.end());
  const std::unordered_set<std::string> required(
    group->getVariableNames().begin(), group->getVariableNames().end());
  if (provided != required || provided.size() != joint_names.size())
    return fail(reason, "joint_names must exactly match whole_body variables");

  std::vector<TrajectoryFrame> working = *frames;
  if (!validateFrameValues(working, joint_names, reason)) return false;

  const auto shortcut_started = std::chrono::steady_clock::now();
  std::vector<TrajectoryFrame> shortened;
  const bool shortcut_ok = deterministicShortcut(working, edge_validator, &shortened, reason);
  metrics->shortcut_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - shortcut_started).count();
  if (!shortcut_ok) return false;

  if (options.variant == "chomp_ruckig") {
    if (!chomp_optimizer) return fail(reason, "chomp_ruckig requires a CHOMP optimizer");
    auto chomp_candidate = shortened;
    std::string chomp_reason;
    bool chomp_ok = chomp_optimizer(&chomp_candidate, &metrics->chomp_ms, &chomp_reason);
    if (chomp_ok) {
      std::string validation_reason;
      chomp_ok = validateFrames(chomp_candidate, joint_names, edge_validator, &validation_reason);
      if (!chomp_ok) chomp_reason = "invalid CHOMP output: " + validation_reason;
    }
    if (chomp_ok) {
      shortened = std::move(chomp_candidate);
      metrics->optimizer_status = "optimized";
    } else {
      metrics->optimizer_status = "failed_fallback";
      metrics->fallback_reason = "CHOMP failed" +
        (chomp_reason.empty() ? std::string{} : ": " + chomp_reason);
      metrics->effective_variant = "shortcut_ruckig";
    }
  }

  std::vector<TrajectoryFrame> expanded;
  if (!insertMetadataTransitions(shortened, &expanded, reason)) return false;

  std::vector<TimedRun> runs;
  double current_time = 0.0;
  for (size_t begin = 0; begin < expanded.size();) {
    size_t end = begin + 1;
    while (end < expanded.size() && sameMetadata(expanded[begin], expanded[end])) ++end;
    TimedRun run;
    if (!makeTimedRun(expanded, begin, end, robot_model, joint_names, options,
        current_time, metrics, &run, reason))
      return false;
    current_time += run.duration;
    runs.push_back(std::move(run));
    begin = end;
  }

  std::vector<TrajectoryFrame> sampled;
  if (!sampleRuns(expanded, runs, joint_names, options.sample_period, &sampled, reason)) return false;
  // Ruckig interpolation can leave the straight joint-space edges validated above.
  if (path_validator) {
    if (!validateFrameValues(sampled, joint_names, reason) || !path_validator(sampled, reason)) return false;
  } else if (!validateFrames(sampled, joint_names, edge_validator, reason)) return false;
  if (!sameJoints(sampled.front(), frames->front()) || !sameJoints(sampled.back(), frames->back()))
    return fail(reason, "postprocessing changed a trajectory endpoint");

  metrics->execution_duration_s = current_time;
  metrics->timing_valid = true;
  if (metrics->effective_variant == "topk") metrics->effective_variant = options.variant;
  *frames = std::move(sampled);
  return true;
}
}  // namespace alfa_robot::motion
