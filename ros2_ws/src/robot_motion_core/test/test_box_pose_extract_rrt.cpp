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

  BoxPoseExtractRrtConfig top_config;
  top_config.mode = BoxPoseExtractMode::TopTranslate;
  top_config.max_iterations = 4000;
  top_config.max_solution_count = 4;
  top_config.random_seed = 17;
  BoxPoseExtractRrt top(top_config);
  const auto top_result = top.plan(BoxPoseExtractState{}, free_edge);
  assert(top_result.success);
  assert(!top_result.paths.empty());
  assert(top.goalReached(top_result.paths.front().states.back()));
  assert(top_result.paths.front().states.back().retreat >= top_config.min_top_retreat);
  assert(top_result.paths.front().states.back().lift >= top_config.min_top_lift);
  assert(top_result.paths.front().states.back().retreat > top_config.box_depth);
  for (size_t index = 1; index < top_result.paths.front().states.size(); ++index) {
    assert(top_result.paths.front().states[index].retreat + 1e-9 >=
           top_result.paths.front().states[index - 1].retreat);
    assert(top_result.paths.front().states[index].lift + 1e-9 >=
           top_result.paths.front().states[index - 1].lift);
  }

  return 0;
}
