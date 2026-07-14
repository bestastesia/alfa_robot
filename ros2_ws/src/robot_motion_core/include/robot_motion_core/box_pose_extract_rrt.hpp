#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace robot_motion::core
{

enum class BoxPoseExtractMode
{
  FrontPivot,
  TopTranslate,
};

struct BoxPoseExtractState
{
  double retreat = 0.0;
  double lift = 0.0;
  double pitch = 0.0;
  double lateral = 0.0;
};

struct BoxPoseExtractRrtConfig
{
  BoxPoseExtractMode mode = BoxPoseExtractMode::FrontPivot;
  double box_depth = 0.3;
  double box_height = 0.4;
  double separation_margin = 0.03;
  double max_retreat = 0.45;
  double max_lift = 0.5;
  double max_pitch = 1.5707963267948966;
  double max_lateral = 0.0;
  double min_top_retreat = 0.03;
  double min_top_lift = 0.03;
  double step_retreat = 0.02;
  double step_lift = 0.02;
  double step_pitch = 0.08726646259971647;
  double step_lateral = 0.02;
  double edge_resolution_retreat = 0.01;
  double edge_resolution_lift = 0.01;
  double edge_resolution_pitch = 0.04363323129985824;
  double edge_resolution_lateral = 0.02;
  double goal_pitch_tolerance = 0.03490658503988659;
  bool front_free_motion = false;
  bool front_goal_requires_max_pitch = true;
  bool endpoint_only_edges = false;
  double goal_sample_rate = 0.2;
  size_t goal_connection_interval = 4;
  double retreat_distance_weight = 1.0;
  double lift_distance_weight = 1.0;
  double pitch_distance_weight = 0.2;
  double lateral_distance_weight = 1.0;
  size_t max_iterations = 2000;
  size_t max_solution_count = 16;
  size_t shortcut_attempts = 80;
  bool preserve_unshortcutted_paths = true;
  size_t parent_candidate_count = 1;
  size_t parent_diverse_candidate_count = 0;
  double parent_path_cost_weight = 0.05;
  double parent_sample_distance_weight = 0.1;
  double parent_endpoint_score_weight = 0.05;
  double parent_node_score_weight = 0.0;
  double parent_density_weight = 0.0;
  bool best_first_fallback = true;
  bool best_first_first = false;
  size_t best_first_max_expansions = 800;
  double best_first_heuristic_weight = 1.0;
  uint32_t random_seed = 7;
};

struct BoxPoseExtractEdgeEvaluation
{
  bool valid = false;
  double joint_motion = std::numeric_limits<double>::infinity();
  std::string rejection_reason;
  double endpoint_score = 0.0;
  bool goal_evaluated = false;
  bool goal_reached = false;
};

using BoxPoseExtractEdgeEvaluator = std::function<BoxPoseExtractEdgeEvaluation(
  const BoxPoseExtractState&,
  const BoxPoseExtractState&)>;

struct BoxPoseExtractPath
{
  std::vector<BoxPoseExtractState> states;
  double joint_motion = std::numeric_limits<double>::infinity();
  size_t iterations = 0;
  size_t edge_evaluations = 0;
  bool shortcut_applied = false;
};

struct BoxPoseExtractRrtResult
{
  bool success = false;
  std::string failure_reason;
  size_t iterations = 0;
  size_t edge_evaluations = 0;
  std::vector<BoxPoseExtractPath> paths;
  BoxPoseExtractPath best_effort_path;
  double best_effort_distance = std::numeric_limits<double>::infinity();
};

class BoxPoseExtractRrt
{
public:
  explicit BoxPoseExtractRrt(BoxPoseExtractRrtConfig config = {});

  const BoxPoseExtractRrtConfig& config() const { return config_; }

  bool stateWithinBounds(const BoxPoseExtractState& state) const;
  bool detachedFromSource(const BoxPoseExtractState& state) const;
  bool goalReached(const BoxPoseExtractState& state) const;

  BoxPoseExtractRrtResult plan(
    const BoxPoseExtractState& start,
    const BoxPoseExtractEdgeEvaluator& evaluator) const;

private:
  BoxPoseExtractState nominalGoal() const;
  double stateDistance(
    const BoxPoseExtractState& lhs,
    const BoxPoseExtractState& rhs) const;

  BoxPoseExtractRrtConfig config_;
};

}  // namespace robot_motion::core
