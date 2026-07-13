#include "robot_motion_core/box_pose_extract_rrt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <utility>

namespace robot_motion::core
{

namespace
{

struct TreeNode
{
  BoxPoseExtractState state;
  size_t parent = 0;
  double cumulative_joint_motion = 0.0;
};

BoxPoseExtractState interpolate(
  const BoxPoseExtractState& from,
  const BoxPoseExtractState& to,
  double ratio)
{
  return {
    from.retreat + (to.retreat - from.retreat) * ratio,
    from.lift + (to.lift - from.lift) * ratio,
    from.pitch + (to.pitch - from.pitch) * ratio,
  };
}

bool intervals_overlap(double lhs_min, double lhs_max, double rhs_min, double rhs_max)
{
  return lhs_min < rhs_max && lhs_max > rhs_min;
}

std::array<std::array<double, 2>, 4> rectangle_corners(
  const BoxPoseExtractState& state,
  const BoxPoseExtractRrtConfig& config)
{
  if (config.mode == BoxPoseExtractMode::TopTranslate) {
    const double center_x = 0.5 * config.box_depth - state.retreat;
    const double center_z = 0.5 * config.box_height + state.lift;
    return {{
      {center_x - 0.5 * config.box_depth, center_z - 0.5 * config.box_height},
      {center_x + 0.5 * config.box_depth, center_z - 0.5 * config.box_height},
      {center_x + 0.5 * config.box_depth, center_z + 0.5 * config.box_height},
      {center_x - 0.5 * config.box_depth, center_z + 0.5 * config.box_height},
    }};
  }

  const double cosine = std::cos(state.pitch);
  const double sine = std::sin(state.pitch);
  const std::array<std::array<double, 2>, 4> local{{
    {0.0, 0.0},
    {config.box_depth, 0.0},
    {config.box_depth, config.box_height},
    {0.0, config.box_height},
  }};
  std::array<std::array<double, 2>, 4> out{};
  for (size_t index = 0; index < local.size(); ++index) {
    out[index] = {
      cosine * local[index][0] - sine * local[index][1] - state.retreat,
      sine * local[index][0] + cosine * local[index][1] + state.lift,
    };
  }
  return out;
}

bool polygon_overlaps_source(
  const std::array<std::array<double, 2>, 4>& corners,
  const BoxPoseExtractRrtConfig& config)
{
  double min_x = corners.front()[0];
  double max_x = corners.front()[0];
  double min_z = corners.front()[1];
  double max_z = corners.front()[1];
  for (const auto& corner : corners) {
    min_x = std::min(min_x, corner[0]);
    max_x = std::max(max_x, corner[0]);
    min_z = std::min(min_z, corner[1]);
    max_z = std::max(max_z, corner[1]);
  }

  const double margin = std::max(0.0, config.separation_margin);
  return intervals_overlap(min_x, max_x, -margin, config.box_depth + margin) &&
         intervals_overlap(min_z, max_z, -margin, config.box_height + margin);
}

BoxPoseExtractState steer(
  const BoxPoseExtractState& from,
  const BoxPoseExtractState& to,
  const BoxPoseExtractRrtConfig& config)
{
  const double retreat_delta = to.retreat - from.retreat;
  const double lift_delta = to.lift - from.lift;
  const double pitch_delta = to.pitch - from.pitch;
  double ratio = 1.0;
  if (std::abs(retreat_delta) > config.step_retreat) {
    ratio = std::min(ratio, config.step_retreat / std::abs(retreat_delta));
  }
  if (std::abs(lift_delta) > config.step_lift) {
    ratio = std::min(ratio, config.step_lift / std::abs(lift_delta));
  }
  if (std::abs(pitch_delta) > config.step_pitch) {
    ratio = std::min(ratio, config.step_pitch / std::abs(pitch_delta));
  }
  return interpolate(from, to, ratio);
}

bool monotonic_transition(
  const BoxPoseExtractState& from,
  const BoxPoseExtractState& to,
  const BoxPoseExtractRrtConfig& config)
{
  constexpr double tolerance = 1e-9;
  if (config.mode == BoxPoseExtractMode::FrontPivot && config.front_free_motion) {
    return true;
  }
  if (to.retreat + tolerance < from.retreat) return false;
  if (config.mode == BoxPoseExtractMode::FrontPivot) {
    return std::abs(to.lift - from.lift) <= tolerance &&
           to.pitch + tolerance >= from.pitch;
  }
  return to.lift + tolerance >= from.lift &&
         std::abs(to.pitch - from.pitch) <= tolerance;
}

std::vector<BoxPoseExtractState> reconstruct_path(
  const std::vector<TreeNode>& nodes,
  size_t index)
{
  std::vector<BoxPoseExtractState> reversed;
  while (true) {
    reversed.push_back(nodes[index].state);
    if (index == 0) break;
    index = nodes[index].parent;
  }
  return {reversed.rbegin(), reversed.rend()};
}

double path_cost(
  const std::vector<BoxPoseExtractState>& states,
  const BoxPoseExtractEdgeEvaluator& evaluator,
  size_t* edge_evaluations,
  bool* valid)
{
  double cost = 0.0;
  *valid = true;
  for (size_t index = 1; index < states.size(); ++index) {
    const auto evaluation = evaluator(states[index - 1], states[index]);
    ++(*edge_evaluations);
    if (!evaluation.valid || !std::isfinite(evaluation.joint_motion)) {
      *valid = false;
      return std::numeric_limits<double>::infinity();
    }
    cost += evaluation.joint_motion;
  }
  return cost;
}

bool same_state(const BoxPoseExtractState& lhs, const BoxPoseExtractState& rhs)
{
  constexpr double tolerance = 1e-8;
  return std::abs(lhs.retreat - rhs.retreat) <= tolerance &&
         std::abs(lhs.lift - rhs.lift) <= tolerance &&
         std::abs(lhs.pitch - rhs.pitch) <= tolerance;
}

bool same_path(const BoxPoseExtractPath& lhs, const BoxPoseExtractPath& rhs)
{
  if (lhs.states.size() != rhs.states.size()) return false;
  for (size_t index = 0; index < lhs.states.size(); ++index) {
    if (!same_state(lhs.states[index], rhs.states[index])) return false;
  }
  return true;
}

void append_unique_path(
  std::vector<BoxPoseExtractPath>* paths,
  BoxPoseExtractPath path,
  size_t max_solution_count)
{
  if (!paths || paths->size() >= max_solution_count) return;
  const bool duplicate = std::any_of(paths->begin(), paths->end(), [&](const auto& kept) {
    return same_path(kept, path);
  });
  if (!duplicate) paths->push_back(std::move(path));
}

}  // namespace

BoxPoseExtractRrt::BoxPoseExtractRrt(BoxPoseExtractRrtConfig config)
: config_(std::move(config))
{}

bool BoxPoseExtractRrt::stateWithinBounds(const BoxPoseExtractState& state) const
{
  const double epsilon = 1e-9;
  if (state.retreat < -epsilon || state.retreat > config_.max_retreat + epsilon) return false;
  if (state.lift < -epsilon || state.lift > config_.max_lift + epsilon) return false;
  if (state.pitch < -epsilon || state.pitch > config_.max_pitch + epsilon) return false;
  if (config_.mode == BoxPoseExtractMode::FrontPivot &&
      !config_.front_free_motion && std::abs(state.lift) > epsilon) return false;
  if (config_.mode == BoxPoseExtractMode::TopTranslate && std::abs(state.pitch) > epsilon) return false;
  return true;
}

bool BoxPoseExtractRrt::detachedFromSource(const BoxPoseExtractState& state) const
{
  return !polygon_overlaps_source(rectangle_corners(state, config_), config_);
}

bool BoxPoseExtractRrt::goalReached(const BoxPoseExtractState& state) const
{
  if (!stateWithinBounds(state) || !detachedFromSource(state)) return false;
  if (config_.mode == BoxPoseExtractMode::FrontPivot) {
    return !config_.front_goal_requires_max_pitch ||
           state.pitch + config_.goal_pitch_tolerance >= config_.max_pitch;
  }
  return state.retreat >= config_.min_top_retreat && state.lift >= config_.min_top_lift;
}

BoxPoseExtractState BoxPoseExtractRrt::nominalGoal() const
{
  if (config_.mode == BoxPoseExtractMode::FrontPivot) {
    if (config_.front_free_motion && !config_.front_goal_requires_max_pitch) {
      return {
        std::min(
          config_.max_retreat,
          config_.box_depth + config_.separation_margin + 1e-4),
        0.0,
        0.0,
      };
    }
    return {
      std::min(
        config_.max_retreat,
        std::max(config_.step_retreat, config_.separation_margin + 1e-4)),
      0.0,
      config_.max_pitch,
    };
  }
  return {
    std::min(
      config_.max_retreat,
      config_.box_depth + config_.separation_margin + 1e-4),
    std::min(
      config_.max_lift,
      std::max(config_.step_lift, config_.min_top_lift)),
    0.0,
  };
}

double BoxPoseExtractRrt::stateDistance(
  const BoxPoseExtractState& lhs,
  const BoxPoseExtractState& rhs) const
{
  const double retreat = config_.retreat_distance_weight * (lhs.retreat - rhs.retreat);
  const double lift = config_.lift_distance_weight * (lhs.lift - rhs.lift);
  const double pitch = config_.pitch_distance_weight * (lhs.pitch - rhs.pitch);
  return std::sqrt(retreat * retreat + lift * lift + pitch * pitch);
}

BoxPoseExtractRrtResult BoxPoseExtractRrt::plan(
  const BoxPoseExtractState& start,
  const BoxPoseExtractEdgeEvaluator& evaluator) const
{
  BoxPoseExtractRrtResult result;
  if (!evaluator) {
    result.failure_reason = "box_pose_rrt_missing_edge_evaluator";
    return result;
  }
  if (!stateWithinBounds(start)) {
    result.failure_reason = "box_pose_rrt_start_out_of_bounds";
    return result;
  }

  std::mt19937 generator(config_.random_seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> retreat_sample(0.0, config_.max_retreat);
  std::uniform_real_distribution<double> lift_sample(0.0, config_.max_lift);
  std::uniform_real_distribution<double> pitch_sample(0.0, config_.max_pitch);
  std::vector<TreeNode> nodes{{start, 0, 0.0}};
  const BoxPoseExtractState goal = nominalGoal();
  size_t best_effort_index = 0;
  double best_effort_distance = stateDistance(start, goal);

  if (!config_.endpoint_only_edges) {
    const auto direct_evaluation = evaluator(start, goal);
    ++result.edge_evaluations;
    const bool direct_goal_reached = direct_evaluation.goal_evaluated ?
      direct_evaluation.goal_reached : goalReached(goal);
    if (direct_evaluation.valid &&
        std::isfinite(direct_evaluation.joint_motion) &&
        direct_goal_reached) {
      BoxPoseExtractPath direct_path;
      direct_path.states = {start, goal};
      direct_path.joint_motion = direct_evaluation.joint_motion;
      direct_path.edge_evaluations = 1;
      append_unique_path(&result.paths, std::move(direct_path), config_.max_solution_count);
    }
  }

  for (size_t iteration = 1; iteration <= config_.max_iterations; ++iteration) {
    result.iterations = iteration;
    BoxPoseExtractState sample = goal;
    if (unit(generator) >= std::clamp(config_.goal_sample_rate, 0.0, 1.0)) {
      sample.retreat = retreat_sample(generator);
      if (config_.mode == BoxPoseExtractMode::FrontPivot) {
        sample.lift = config_.front_free_motion ? lift_sample(generator) : 0.0;
        sample.pitch = pitch_sample(generator);
      } else {
        sample.lift = lift_sample(generator);
        sample.pitch = 0.0;
      }
    }

    size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < nodes.size(); ++index) {
      if (!monotonic_transition(nodes[index].state, sample, config_)) continue;
      const double distance = stateDistance(nodes[index].state, sample);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = index;
      }
    }
    if (!std::isfinite(nearest_distance)) continue;

    const BoxPoseExtractState next = steer(nodes[nearest_index].state, sample, config_);
    if (!stateWithinBounds(next) ||
        !monotonic_transition(nodes[nearest_index].state, next, config_) ||
        stateDistance(nodes[nearest_index].state, next) < 1e-9) {
      continue;
    }
    const auto evaluation = evaluator(nodes[nearest_index].state, next);
    ++result.edge_evaluations;
    if (!evaluation.valid || !std::isfinite(evaluation.joint_motion)) {
      continue;
    }

    nodes.push_back({
      next,
      nearest_index,
      nodes[nearest_index].cumulative_joint_motion + evaluation.joint_motion});
    size_t next_index = nodes.size() - 1;
    const double next_goal_distance = stateDistance(next, goal);
    if (next_goal_distance < best_effort_distance) {
      best_effort_distance = next_goal_distance;
      best_effort_index = next_index;
    }
    const bool next_goal_reached = evaluation.goal_evaluated ?
      evaluation.goal_reached : goalReached(next);
    if (!next_goal_reached) {
      if (config_.endpoint_only_edges) continue;
      const size_t interval = std::max<size_t>(1, config_.goal_connection_interval);
      if (iteration % interval != 0 || !monotonic_transition(next, goal, config_)) {
        continue;
      }
      const auto goal_evaluation = evaluator(next, goal);
      ++result.edge_evaluations;
      const bool connected_goal_reached = goal_evaluation.goal_evaluated ?
        goal_evaluation.goal_reached : goalReached(goal);
      if (!goal_evaluation.valid || !std::isfinite(goal_evaluation.joint_motion) ||
          !connected_goal_reached) {
        continue;
      }
      nodes.push_back({
        goal,
        next_index,
        nodes[next_index].cumulative_joint_motion + goal_evaluation.joint_motion});
      next_index = nodes.size() - 1;
      best_effort_distance = 0.0;
      best_effort_index = next_index;
    }

    BoxPoseExtractPath raw_path;
    raw_path.states = reconstruct_path(nodes, next_index);
    raw_path.iterations = iteration;
    const bool raw_path_valid = true;
    raw_path.joint_motion = nodes[next_index].cumulative_joint_motion;
    raw_path.edge_evaluations = result.edge_evaluations;
    if (raw_path_valid && config_.preserve_unshortcutted_paths) {
      append_unique_path(&result.paths, raw_path, config_.max_solution_count);
    }

    BoxPoseExtractPath path = std::move(raw_path);

    const size_t shortcut_attempts = config_.endpoint_only_edges ? 0 : config_.shortcut_attempts;
    for (size_t attempt = 0; attempt < shortcut_attempts && path.states.size() > 2; ++attempt) {
      std::uniform_int_distribution<size_t> index_sample(0, path.states.size() - 1);
      size_t first = index_sample(generator);
      size_t last = index_sample(generator);
      if (first > last) std::swap(first, last);
      if (last <= first + 1) continue;
      if (!monotonic_transition(path.states[first], path.states[last], config_)) continue;
      const auto shortcut = evaluator(path.states[first], path.states[last]);
      ++result.edge_evaluations;
      if (!shortcut.valid || !std::isfinite(shortcut.joint_motion)) continue;
      path.states.erase(path.states.begin() + static_cast<std::ptrdiff_t>(first + 1),
                        path.states.begin() + static_cast<std::ptrdiff_t>(last));
      path.shortcut_applied = true;
    }

    bool path_valid = false;
    path.joint_motion = path_cost(path.states, evaluator, &result.edge_evaluations, &path_valid);
    path.edge_evaluations = result.edge_evaluations;
    if (path_valid) {
      append_unique_path(&result.paths, std::move(path), config_.max_solution_count);
    }
    if (result.paths.size() >= config_.max_solution_count) {
      break;
    }
  }

  std::sort(result.paths.begin(), result.paths.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.joint_motion < rhs.joint_motion;
  });
  result.success = !result.paths.empty();
  if (!result.success) {
    result.failure_reason = "box_pose_rrt_no_reachable_path";
    result.best_effort_path.states = reconstruct_path(nodes, best_effort_index);
    result.best_effort_path.joint_motion = nodes[best_effort_index].cumulative_joint_motion;
    result.best_effort_path.iterations = result.iterations;
    result.best_effort_path.edge_evaluations = result.edge_evaluations;
    result.best_effort_distance = best_effort_distance;
  }
  return result;
}

}  // namespace robot_motion::core
