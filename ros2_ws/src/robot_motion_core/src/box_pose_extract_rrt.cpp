#include "robot_motion_core/box_pose_extract_rrt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
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
  double endpoint_score = 0.0;
};

double finite_or_zero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

BoxPoseExtractState interpolate(
  const BoxPoseExtractState& from,
  const BoxPoseExtractState& to,
  double ratio)
{
  return {
    from.retreat + (to.retreat - from.retreat) * ratio,
    from.lift + (to.lift - from.lift) * ratio,
    from.pitch + (to.pitch - from.pitch) * ratio,
    from.lateral + (to.lateral - from.lateral) * ratio,
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
  const double lateral_delta = to.lateral - from.lateral;
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
  if (std::abs(lateral_delta) > config.step_lateral) {
    ratio = std::min(ratio, config.step_lateral / std::abs(lateral_delta));
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
         std::abs(lhs.pitch - rhs.pitch) <= tolerance &&
         std::abs(lhs.lateral - rhs.lateral) <= tolerance;
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

int quantized_index(double value, double step)
{
  return static_cast<int>(std::llround(value / std::max(1e-9, std::abs(step))));
}

using LatticeKey = std::array<int, 4>;

LatticeKey lattice_key(const BoxPoseExtractState& state, const BoxPoseExtractRrtConfig& config)
{
  return {
    quantized_index(state.retreat, config.step_retreat),
    quantized_index(state.lift, config.step_lift),
    quantized_index(state.pitch, config.step_pitch),
    quantized_index(state.lateral, config.step_lateral),
  };
}

std::vector<BoxPoseExtractState> lattice_neighbors(
  const BoxPoseExtractState& state,
  const BoxPoseExtractRrtConfig& config)
{
  std::vector<int> retreat_steps{0, 1};
  std::vector<int> lift_steps{0, 1};
  std::vector<int> pitch_steps{0};
  std::vector<int> lateral_steps{0};

  if (config.mode == BoxPoseExtractMode::FrontPivot) {
    if (config.front_free_motion) {
      lift_steps = {-1, 0, 1};
      pitch_steps = {-1, 0, 1};
    } else {
      lift_steps = {0};
      pitch_steps = {0, 1};
    }
  }
  if (config.mode == BoxPoseExtractMode::TopTranslate) {
    pitch_steps = {0};
  }
  if (config.max_lateral > 1e-9) {
    lateral_steps = {-1, 0, 1};
  }

  std::vector<BoxPoseExtractState> neighbors;
  for (const int dr : retreat_steps) {
    for (const int dl : lift_steps) {
      for (const int dp : pitch_steps) {
        for (const int dy : lateral_steps) {
          if (dr == 0 && dl == 0 && dp == 0 && dy == 0) continue;
          BoxPoseExtractState next{
            state.retreat + static_cast<double>(dr) * config.step_retreat,
            state.lift + static_cast<double>(dl) * config.step_lift,
            state.pitch + static_cast<double>(dp) * config.step_pitch,
            state.lateral + static_cast<double>(dy) * config.step_lateral,
          };
          neighbors.push_back(next);
        }
      }
    }
  }
  return neighbors;
}

std::vector<std::pair<LatticeKey, double>>::iterator find_lattice_cost(
  std::vector<std::pair<LatticeKey, double>>* costs,
  const LatticeKey& key)
{
  return std::find_if(costs->begin(), costs->end(), [&](const auto& item) {
    return item.first == key;
  });
}

size_t node_cell_density(
  const std::vector<TreeNode>& nodes,
  const LatticeKey& key,
  const BoxPoseExtractRrtConfig& config)
{
  size_t density = 0;
  for (const auto& node : nodes) {
    if (lattice_key(node.state, config) == key) {
      ++density;
    }
  }
  return density;
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
  if (std::abs(state.lateral) > config_.max_lateral + epsilon) return false;
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
        0.0,
      };
    }
    return {
      std::min(
        config_.max_retreat,
        std::max(config_.step_retreat, config_.separation_margin + 1e-4)),
      0.0,
      config_.max_pitch,
      0.0,
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
  const double lateral = config_.lateral_distance_weight * (lhs.lateral - rhs.lateral);
  return std::sqrt(retreat * retreat + lift * lift + pitch * pitch + lateral * lateral);
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
  std::uniform_real_distribution<double> lateral_sample(-config_.max_lateral, config_.max_lateral);
  std::vector<TreeNode> nodes{{start, 0, 0.0, 0.0}};
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

  auto run_best_first_search = [&]() {
    struct QueueEntry
    {
      double priority = std::numeric_limits<double>::infinity();
      size_t node_index = 0;
    };
    struct QueueGreater
    {
      bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const
      {
        return lhs.priority > rhs.priority;
      }
    };

    std::vector<TreeNode> lattice_nodes{{start, 0, 0.0, 0.0}};
    std::vector<std::pair<LatticeKey, double>> best_cost;
    best_cost.push_back({lattice_key(start, config_), 0.0});
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueGreater> queue;
    queue.push({config_.best_first_heuristic_weight * stateDistance(start, goal), 0});

    size_t lattice_best_index = 0;
    double lattice_best_distance = stateDistance(start, goal);
    const size_t expansion_limit = std::max<size_t>(1, config_.best_first_max_expansions);
    for (size_t expansion = 0; expansion < expansion_limit && !queue.empty(); ++expansion) {
      const QueueEntry entry = queue.top();
      queue.pop();
      if (entry.node_index >= lattice_nodes.size()) continue;
      const TreeNode& parent_node = lattice_nodes[entry.node_index];
      if (goalReached(parent_node.state) && entry.node_index != 0) {
        BoxPoseExtractPath path;
        path.states = reconstruct_path(lattice_nodes, entry.node_index);
        bool path_valid = false;
        path.joint_motion = path_cost(path.states, evaluator, &result.edge_evaluations, &path_valid);
        path.edge_evaluations = result.edge_evaluations;
        path.iterations = result.iterations + expansion;
        if (path_valid) {
          append_unique_path(&result.paths, std::move(path), config_.max_solution_count);
          if (result.paths.size() >= config_.max_solution_count) break;
        }
      }

      for (const auto& next : lattice_neighbors(parent_node.state, config_)) {
        if (!stateWithinBounds(next) ||
            !monotonic_transition(parent_node.state, next, config_)) {
          continue;
        }
        const auto evaluation = evaluator(parent_node.state, next);
        ++result.edge_evaluations;
        if (!evaluation.valid || !std::isfinite(evaluation.joint_motion)) {
          continue;
        }
        const double next_cost = parent_node.cumulative_joint_motion + evaluation.joint_motion;
        const LatticeKey key = lattice_key(next, config_);
        const auto previous = find_lattice_cost(&best_cost, key);
        if (previous != best_cost.end() && previous->second <= next_cost + 1e-9) {
          continue;
        }
        if (previous != best_cost.end()) {
          previous->second = next_cost;
        } else {
          best_cost.push_back({key, next_cost});
        }
        lattice_nodes.push_back({next, entry.node_index, next_cost, evaluation.endpoint_score});
        const size_t next_index = lattice_nodes.size() - 1;
        const double next_goal_distance = stateDistance(next, goal);
        if (next_goal_distance < lattice_best_distance) {
          lattice_best_distance = next_goal_distance;
          lattice_best_index = next_index;
        }
        const bool next_goal_reached = evaluation.goal_evaluated ?
          evaluation.goal_reached : goalReached(next);
        if (next_goal_reached) {
          BoxPoseExtractPath path;
          path.states = reconstruct_path(lattice_nodes, next_index);
          bool path_valid = false;
          path.joint_motion = path_cost(path.states, evaluator, &result.edge_evaluations, &path_valid);
          path.edge_evaluations = result.edge_evaluations;
          path.iterations = result.iterations + expansion + 1;
          if (path_valid) {
            append_unique_path(&result.paths, std::move(path), config_.max_solution_count);
            if (result.paths.size() >= config_.max_solution_count) break;
          }
        }
        const double priority =
          next_cost +
          config_.parent_endpoint_score_weight * finite_or_zero(evaluation.endpoint_score) +
          config_.best_first_heuristic_weight * next_goal_distance;
        queue.push({priority, next_index});
      }
    }

    if (result.paths.empty() && lattice_nodes.size() > 1 &&
        lattice_best_distance < best_effort_distance) {
      best_effort_distance = lattice_best_distance;
      best_effort_index = 0;
      result.best_effort_path.states = reconstruct_path(lattice_nodes, lattice_best_index);
      result.best_effort_path.joint_motion = lattice_nodes[lattice_best_index].cumulative_joint_motion;
      result.best_effort_path.iterations = result.iterations + expansion_limit;
      result.best_effort_path.edge_evaluations = result.edge_evaluations;
    }
  };

  bool best_first_tried = false;
  if (config_.best_first_fallback && config_.best_first_first && result.paths.empty()) {
    best_first_tried = true;
    run_best_first_search();
    if (!result.paths.empty()) {
      std::sort(result.paths.begin(), result.paths.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.joint_motion < rhs.joint_motion;
      });
      result.success = true;
      return result;
    }
  }

  struct ParentCandidate
  {
    size_t index = 0;
    double distance = std::numeric_limits<double>::infinity();
    double rank = std::numeric_limits<double>::infinity();
  };

  for (size_t iteration = 1; iteration <= config_.max_iterations; ++iteration) {
    result.iterations = iteration;
    BoxPoseExtractState sample = goal;
    if (unit(generator) >= std::clamp(config_.goal_sample_rate, 0.0, 1.0)) {
      sample.retreat = retreat_sample(generator);
      sample.lateral = lateral_sample(generator);
      if (config_.mode == BoxPoseExtractMode::FrontPivot) {
        sample.lift = config_.front_free_motion ? lift_sample(generator) : 0.0;
        sample.pitch = pitch_sample(generator);
      } else {
        sample.lift = lift_sample(generator);
        sample.pitch = 0.0;
      }
    }

    std::vector<ParentCandidate> parent_candidates;
    parent_candidates.reserve(nodes.size());
    for (size_t index = 0; index < nodes.size(); ++index) {
      if (!monotonic_transition(nodes[index].state, sample, config_)) continue;
      const double distance = stateDistance(nodes[index].state, sample);
      const size_t density = node_cell_density(nodes, lattice_key(nodes[index].state, config_), config_);
      const double density_penalty = std::log1p(static_cast<double>(density));
      const double goal_distance = stateDistance(nodes[index].state, goal);
      const double rank =
        goal_distance +
        config_.parent_path_cost_weight * nodes[index].cumulative_joint_motion +
        config_.parent_node_score_weight * finite_or_zero(nodes[index].endpoint_score) +
        config_.parent_density_weight * density_penalty;
      parent_candidates.push_back({index, distance, rank});
    }
    if (parent_candidates.empty()) continue;
    std::sort(parent_candidates.begin(), parent_candidates.end(), [](const auto& lhs, const auto& rhs) {
      return lhs.distance < rhs.distance;
    });
    const size_t parent_limit = std::max<size_t>(1, config_.parent_candidate_count);
    std::vector<ParentCandidate> nearest_candidates = parent_candidates;
    if (nearest_candidates.size() > parent_limit) {
      nearest_candidates.resize(parent_limit);
    }
    const size_t diverse_limit = config_.parent_diverse_candidate_count;
    if (diverse_limit > 0 && parent_candidates.size() > nearest_candidates.size()) {
      std::vector<ParentCandidate> diverse_candidates = parent_candidates;
      std::sort(diverse_candidates.begin(), diverse_candidates.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.rank < rhs.rank;
      });
      for (const auto& candidate : diverse_candidates) {
        if (nearest_candidates.size() >= parent_limit + diverse_limit) {
          break;
        }
        const bool duplicate = std::any_of(nearest_candidates.begin(), nearest_candidates.end(), [&](const auto& kept) {
          return kept.index == candidate.index;
        });
        if (!duplicate) {
          nearest_candidates.push_back(candidate);
        }
      }
    }
    parent_candidates = std::move(nearest_candidates);

    struct ExtensionCandidate
    {
      size_t parent_index = 0;
      BoxPoseExtractState next;
      BoxPoseExtractEdgeEvaluation evaluation;
      double score = std::numeric_limits<double>::infinity();
    };
    ExtensionCandidate selected_extension;
    bool selected = false;
    for (const auto& parent : parent_candidates) {
      const BoxPoseExtractState next = steer(nodes[parent.index].state, sample, config_);
      if (!stateWithinBounds(next) ||
          !monotonic_transition(nodes[parent.index].state, next, config_) ||
          stateDistance(nodes[parent.index].state, next) < 1e-9) {
        continue;
      }
      const auto evaluation = evaluator(nodes[parent.index].state, next);
      ++result.edge_evaluations;
      if (!evaluation.valid || !std::isfinite(evaluation.joint_motion)) {
        continue;
      }
      const double next_goal_distance = stateDistance(next, goal);
      const double sample_distance = stateDistance(next, sample);
      const double cumulative_cost = nodes[parent.index].cumulative_joint_motion + evaluation.joint_motion;
      const double score =
        next_goal_distance +
        config_.parent_path_cost_weight * cumulative_cost +
        config_.parent_sample_distance_weight * sample_distance +
        config_.parent_endpoint_score_weight * finite_or_zero(evaluation.endpoint_score);
      if (!selected || score < selected_extension.score) {
        selected = true;
        selected_extension = {parent.index, next, evaluation, score};
      }
    }
    if (!selected) {
      continue;
    }

    nodes.push_back({
      selected_extension.next,
      selected_extension.parent_index,
      nodes[selected_extension.parent_index].cumulative_joint_motion + selected_extension.evaluation.joint_motion,
      selected_extension.evaluation.endpoint_score});
    size_t next_index = nodes.size() - 1;
    const double next_goal_distance = stateDistance(selected_extension.next, goal);
    if (next_goal_distance < best_effort_distance) {
      best_effort_distance = next_goal_distance;
      best_effort_index = next_index;
    }
    const bool next_goal_reached = selected_extension.evaluation.goal_evaluated ?
      selected_extension.evaluation.goal_reached : goalReached(selected_extension.next);
    if (!next_goal_reached) {
      if (config_.endpoint_only_edges) continue;
      const size_t interval = std::max<size_t>(1, config_.goal_connection_interval);
      if (iteration % interval != 0 || !monotonic_transition(selected_extension.next, goal, config_)) {
        continue;
      }
      const auto goal_evaluation = evaluator(selected_extension.next, goal);
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
        nodes[next_index].cumulative_joint_motion + goal_evaluation.joint_motion,
        goal_evaluation.endpoint_score});
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

  if (result.paths.empty() && config_.best_first_fallback && !best_first_tried) {
    run_best_first_search();
  }

  std::sort(result.paths.begin(), result.paths.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.joint_motion < rhs.joint_motion;
  });
  result.success = !result.paths.empty();
  if (!result.success) {
    result.failure_reason = "box_pose_rrt_no_reachable_path";
    if (result.best_effort_path.states.empty()) {
      result.best_effort_path.states = reconstruct_path(nodes, best_effort_index);
      result.best_effort_path.joint_motion = nodes[best_effort_index].cumulative_joint_motion;
      result.best_effort_path.iterations = result.iterations;
      result.best_effort_path.edge_evaluations = result.edge_evaluations;
    }
    result.best_effort_distance = best_effort_distance;
  }
  return result;
}

}  // namespace robot_motion::core
