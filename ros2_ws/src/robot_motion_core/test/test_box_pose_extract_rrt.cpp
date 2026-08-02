#include "robot_motion_core/box_pose_extract_rrt.hpp"

#include <cassert>
#include <cmath>

namespace
{

robot_motion::core::BoxPoseExtractEdgeEvaluation free_edge(
  const robot_motion::core::BoxPoseExtractState& from,
  const robot_motion::core::BoxPoseExtractState& to)
{
  assert(to.retreat + 1e-9 >= from.retreat);
  assert(to.lift + 1e-9 >= from.lift);
  assert(to.pitch + 1e-9 >= from.pitch);
  const double retreat = to.retreat - from.retreat;
  const double lift = to.lift - from.lift;
  const double pitch = 0.2 * (to.pitch - from.pitch);
  return {true, std::sqrt(retreat * retreat + lift * lift + pitch * pitch), ""};
}

robot_motion::core::BoxPoseExtractEdgeEvaluation bidirectional_free_edge(
  const robot_motion::core::BoxPoseExtractState& from,
  const robot_motion::core::BoxPoseExtractState& to)
{
  const double retreat = to.retreat - from.retreat;
  const double lift = to.lift - from.lift;
  const double pitch = 0.2 * (to.pitch - from.pitch);
  return {true, std::sqrt(retreat * retreat + lift * lift + pitch * pitch), ""};
}

}  // namespace

int main()
{
  using robot_motion::core::BoxPoseExtractMode;
  using robot_motion::core::BoxPoseExtractRrt;
  using robot_motion::core::BoxPoseExtractRrtConfig;
  using robot_motion::core::BoxPoseExtractState;

  BoxPoseExtractRrtConfig front_config;
  front_config.mode = BoxPoseExtractMode::FrontPivot;
  front_config.max_iterations = 4000;
  front_config.max_solution_count = 4;
  front_config.random_seed = 11;
  BoxPoseExtractRrt front(front_config);
  const auto front_result = front.plan(BoxPoseExtractState{}, free_edge);
  assert(front_result.success);
  assert(!front_result.paths.empty());
  assert(front.goalReached(front_result.paths.front().states.back()));
  assert(front_result.paths.front().states.back().pitch > 1.4);
  for (size_t index = 1; index < front_result.paths.front().states.size(); ++index) {
    assert(front_result.paths.front().states[index].retreat + 1e-9 >=
           front_result.paths.front().states[index - 1].retreat);
    assert(front_result.paths.front().states[index].pitch + 1e-9 >=
           front_result.paths.front().states[index - 1].pitch);
  }

  BoxPoseExtractRrtConfig diverse_config = front_config;
  diverse_config.max_solution_count = 8;
  diverse_config.preserve_unshortcutted_paths = true;
  diverse_config.shortcut_attempts = 80;
  const auto diverse_result = BoxPoseExtractRrt(diverse_config).plan(BoxPoseExtractState{}, free_edge);
  assert(diverse_result.success);
  assert(diverse_result.paths.size() >= 4);
  bool has_unshortcutted = false;
  bool has_short_path = false;
  for (const auto& path : diverse_result.paths) {
    has_unshortcutted = has_unshortcutted || path.states.size() > 2;
    has_short_path = has_short_path || path.states.size() == 2 || path.shortcut_applied;
  }
  assert(has_unshortcutted);
  assert(has_short_path);

  BoxPoseExtractRrtConfig free_front_config = front_config;
  free_front_config.front_free_motion = true;
  free_front_config.front_goal_requires_max_pitch = false;
  free_front_config.max_lift = 0.5;
  free_front_config.max_solution_count = 4;
  free_front_config.random_seed = 23;
  BoxPoseExtractRrt free_front(free_front_config);
  assert(free_front.goalReached({free_front_config.box_depth + 0.04, 0.0, 0.0}));
  assert(free_front.goalReached({0.0, free_front_config.box_height + 0.04, 0.0}));
  assert(!free_front.goalReached({0.01, 0.01, 0.0}));
  const auto free_front_result = free_front.plan(BoxPoseExtractState{}, bidirectional_free_edge);
  assert(free_front_result.success);
  assert(!free_front_result.paths.empty());
  assert(free_front.goalReached(free_front_result.paths.front().states.back()));
  assert(free_front_result.paths.front().states.back().pitch < 0.2);

  BoxPoseExtractRrtConfig top_config;
  top_config.mode = BoxPoseExtractMode::TopTranslate;
  top_config.top_goal_min_pitch = 5.0 * M_PI / 180.0;
  top_config.max_iterations = 4000;
  top_config.max_solution_count = 4;
  top_config.random_seed = 17;
  BoxPoseExtractRrt top(top_config);
  const auto top_result = top.plan(BoxPoseExtractState{}, free_edge);
  assert(top_result.success);
  assert(!top_result.paths.empty());
  assert(top.goalReached(top_result.paths.front().states.back()));
  assert(top_result.paths.front().states.back().pitch >=
         top_config.top_goal_min_pitch - top_config.goal_pitch_tolerance);
  assert(top_result.paths.front().states.back().lift >= top_config.box_height + top_config.separation_margin);
  assert(top_result.paths.front().states.back().retreat <= top_config.step_retreat + 1e-9);
  assert(top_result.paths.front().states.back().pitch < 0.2);
  for (size_t index = 1; index < top_result.paths.front().states.size(); ++index) {
    assert(top_result.paths.front().states[index].retreat + 1e-9 >=
           top_result.paths.front().states[index - 1].retreat);
    assert(top_result.paths.front().states[index].lift + 1e-9 >=
           top_result.paths.front().states[index - 1].lift);
    assert(top_result.paths.front().states[index].pitch + 1e-9 >=
           top_result.paths.front().states[index - 1].pitch);
  }
  BoxPoseExtractRrtConfig rotation_config = top_config;
  rotation_config.top_goal_min_pitch = 0.0;
  BoxPoseExtractRrt rotation_enabled(rotation_config);
  assert(!rotation_enabled.goalReached({0.0, 0.38, 0.0, 0.0}));
  assert(rotation_enabled.goalReached({0.0, 0.38, M_PI_2, 0.0}));

  BoxPoseExtractRrtConfig elevated_reference_config = top_config;
  elevated_reference_config.top_goal_min_pitch = 0.0;
  elevated_reference_config.separation_margin = 0.0;
  elevated_reference_config.source_reference_offset_z = 0.4;
  BoxPoseExtractRrt elevated_reference(elevated_reference_config);
  assert(!elevated_reference.goalReached({0.0, 0.4, 0.0, 0.0}));
  assert(elevated_reference.goalReached({0.0, 0.81, 0.0, 0.0}));

  BoxPoseExtractRrtConfig lifted_start_config = top_config;
  lifted_start_config.top_goal_min_pitch = 0.0;
  lifted_start_config.source_reference_offset_z = -0.28;
  BoxPoseExtractRrt lifted_start(lifted_start_config);
  assert(!lifted_start.goalReached({0.0, 0.14, 0.0, 0.0}));
  assert(lifted_start.goalReached({0.0, 0.16, 0.0, 0.0}));

  return 0;
}
