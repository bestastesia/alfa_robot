#include "alfa_robot_moveit_config/box_stack_flow_orchestrator.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

namespace alfa_robot::motion
{

BoxStackFlowOrchestrator::BoxStackFlowOrchestrator(
  BoxStackFlowConfig config,
  BoxStackFlowCallbacks callbacks)
: config_(std::move(config)), callbacks_(std::move(callbacks))
{}

bool BoxStackFlowOrchestrator::run()
{
  if (callbacks_.clear_scene) callbacks_.clear_scene();

  const auto pairs = make_pick_pairs(config_.include_top_suction, config_.pair_sequence);
  const int rounds_to_run = std::min<int>(
    std::max(1, config_.max_rounds),
    static_cast<int>(pairs.size()));

  if (callbacks_.info) {
    std::ostringstream oss;
    oss << "Starting box-stack flow: rounds=" << rounds_to_run << "/" << pairs.size();
    callbacks_.info(oss.str());
  }

  for (int i = 0; i < rounds_to_run; ++i) {
    const auto& pair = pairs[static_cast<size_t>(i)];
    if (callbacks_.info) {
      std::ostringstream oss;
      oss << "=== round " << pair.round << ": left box " << pair.left_box
          << ", right box " << pair.right_box
          << ", mode=" << (pair.top_suction ? "top_suction" : "front") << " ===";
      callbacks_.info(oss.str());
    }
    if (!runOnePair(pair.left_box, pair.right_box, pair.top_suction, pair.round)) {
      return false;
    }
  }

  if (callbacks_.info) callbacks_.info("Box-stack flow finished");
  return true;
}

bool BoxStackFlowOrchestrator::runOnePair(
  int left_box_id,
  int right_box_id,
  bool top_suction,
  int round)
{
  if (callbacks_.clear_scene) callbacks_.clear_scene();

  const auto boxes = make_boxes(config_.box_front_x);
  const auto left_it = boxes.find(left_box_id);
  const auto right_it = boxes.find(right_box_id);
  if (left_it == boxes.end() || right_it == boxes.end()) {
    return callbacks_.fail ? callbacks_.fail("unknown box id in one-pair flow") : false;
  }

  if (callbacks_.set_wall_opening &&
      !callbacks_.set_wall_opening(left_box_id, right_box_id, "one_pair_flow")) {
    return false;
  }

  const std::string prefix = "round_" + std::to_string(round) +
                             "_L" + std::to_string(left_box_id) +
                             "_R" + std::to_string(right_box_id);

  if (callbacks_.plan_joint_target &&
      !callbacks_.plan_joint_target(prefix + "/pregrasp", config_.fixed_updown,
                                    config_.left_pregrasp_arm, config_.right_pregrasp_arm)) {
    return false;
  }

  if (callbacks_.plan_grasp_ik &&
      !callbacks_.plan_grasp_ik(prefix + "/grasp_ik", left_it->second, right_it->second, top_suction)) {
    return false;
  }

  if (callbacks_.attach_boxes && !callbacks_.attach_boxes(left_box_id, right_box_id, top_suction)) {
    return false;
  }

  if (callbacks_.validate_attached_boxes &&
      !callbacks_.validate_attached_boxes(prefix + "/attach")) {
    if (callbacks_.detach_boxes) callbacks_.detach_boxes();
    return false;
  }

  if (callbacks_.plan_joint_target &&
      !callbacks_.plan_joint_target(prefix + "/loaded", config_.fixed_updown,
                                    config_.left_loaded_arm, config_.right_loaded_arm)) {
    if (callbacks_.detach_boxes) callbacks_.detach_boxes();
    return false;
  }

  if (callbacks_.detach_boxes && !callbacks_.detach_boxes()) {
    return false;
  }

  if (callbacks_.plan_joint_target &&
      !callbacks_.plan_joint_target(prefix + "/return_pregrasp", config_.fixed_updown,
                                    config_.left_pregrasp_arm, config_.right_pregrasp_arm)) {
    return false;
  }
  return true;
}

}  // namespace alfa_robot::motion
