#include "alfa_robot_moveit_config/loaded_pose_planner.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"
#include "alfa_robot_moveit_config/motion_scene_adapter.hpp"

#include <moveit_msgs/msg/collision_object.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace alfa_robot::motion
{

LoadedPosePlanner::LoadedPosePlanner(LoadedPosePlannerConfig config)
: config_(std::move(config))
{}

double LoadedPosePlanner::currentUpdown(const moveit::core::RobotState& state)
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  if (std::find(variable_names.begin(), variable_names.end(), "updown") == variable_names.end()) {
    return 0.0;
  }
  return state.getVariablePosition("updown");
}

LoadedPosePlanResult LoadedPosePlanner::plan(
  const std::string& stage_name,
  const moveit::core::RobotState& extract_state,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  size_t loaded_plan_rank)
{
  LoadedPosePlanResult result;
  if (!config_.move_group) {
    result.failure_reason = "loaded_move_group_not_initialized";
    return result;
  }
  if (!config_.selector) {
    result.failure_reason = "loaded_pose_selector_not_initialized";
    return result;
  }
  if (!config_.scene_adapter) {
    result.failure_reason = "motion_scene_adapter_not_initialized";
    return result;
  }

  const auto saved_boxes = config_.scene_adapter->activeAttachedBoxes();
  config_.scene_adapter->setActiveAttachedBoxesForRecordOnly(carried_boxes);

  auto restore_boxes = [&]() {
    std::vector<std::string> ids;
    ids.reserve(carried_boxes.size());
    for (const auto& box : carried_boxes) {
      ids.push_back(box.id);
    }
    config_.scene_adapter->removeCarriedBoxIds(ids);
    config_.scene_adapter->setActiveAttachedBoxesForRecordOnly(saved_boxes);
  };

  if (!config_.scene_adapter->applyAttachedBoxState(
        config_.scene_adapter->activeAttachedBoxes(),
        moveit_msgs::msg::CollisionObject::ADD)) {
    result.failure_reason = "failed_to_attach_box_for_loaded_plan";
    restore_boxes();
    return result;
  }

  moveit::core::RobotState goal_state = config_.selector->makeGoalState(
    extract_state, &result.selection);

  config_.move_group->setStartState(extract_state);
  config_.move_group->setJointValueTarget(goal_state);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto t0 = std::chrono::steady_clock::now();
  const auto plan_result = config_.move_group->plan(plan);
  const auto t1 = std::chrono::steady_clock::now();

  result.attempted = true;
  result.plan_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  result.plan_points = plan.trajectory_.joint_trajectory.points.size();

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    result.failure_reason = "moveit_planning_failed_code_" + std::to_string(plan_result.val);
    restore_boxes();
    return result;
  }

  std::string carried_collision_reason;
  result.carried_clear = config_.clearance_callback
    ? config_.clearance_callback(plan, extract_state, &carried_collision_reason)
    : true;
  if (!result.carried_clear) {
    result.failure_reason = carried_collision_reason;
  }

  if (config_.record_callback) {
    const auto& left_family = config_.selector->leftPoseFamily();
    const auto& right_family = config_.selector->rightPoseFamily();
    nlohmann::json extra = {
      {"stage_kind", "post_extract_loaded_plan"},
      {"valid", result.carried_clear},
      {"loaded_plan_ms", result.plan_ms},
      {"loaded_plan_points", result.plan_points},
      {"start_updown", currentUpdown(extract_state)},
      {"target_updown", currentUpdown(goal_state)},
      {"carried_box_count", carried_boxes.size()},
      {"loaded_plan_rank", loaded_plan_rank},
      {"loaded_pose_distance_sum", result.selection.distance_sum},
      {"loaded_pose_distance_l2", result.selection.distance_l2},
      {"loaded_pose_max_joint_delta", result.selection.max_joint_delta},
      {"loaded_pose_max_joint_delta_deg", result.selection.max_joint_delta * 180.0 / M_PI},
      {"selected_left_loaded_pose_index", result.selection.left_index},
      {"selected_right_loaded_pose_index", result.selection.right_index},
      {"selected_left_loaded_pose_distance", result.selection.left_distance},
      {"selected_right_loaded_pose_distance", result.selection.right_distance},
      {"selected_left_loaded_pose_deg", pose_degrees_json(left_family[result.selection.left_index])},
      {"selected_right_loaded_pose_deg", pose_degrees_json(right_family[result.selection.right_index])},
      {"failure_reason", result.carried_clear ? "" : carried_collision_reason}
    };
    config_.record_callback(
      stage_name, plan, extract_state, goal_state, config_.target_joint_names, extra);
  }

  if (!result.carried_clear) {
    restore_boxes();
    return result;
  }

  result.success = true;
  result.failure_reason.clear();
  restore_boxes();
  return result;
}

LoadedPoseBatchPlanResult LoadedPosePlanner::planBatch(
  const std::string& prefix,
  std::vector<ExtractRolloutTiming>& timings,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  const LoadedPoseBatchPlanOptions& options)
{
  LoadedPoseBatchPlanResult batch_result;
  if (!options.enabled) {
    return batch_result;
  }

  for (size_t i = 0; i < timings.size(); ++i) {
    if (timings[i].success && timings[i].final_state) {
      batch_result.plan_indices.push_back(i);
    }
  }

  if (options.sort_by_pose_distance) {
    std::sort(batch_result.plan_indices.begin(), batch_result.plan_indices.end(),
              [&](const size_t a, const size_t b) {
                const auto& lhs = timings[a];
                const auto& rhs = timings[b];
                if (lhs.loaded_pose_distance_sum != rhs.loaded_pose_distance_sum) {
                  return lhs.loaded_pose_distance_sum < rhs.loaded_pose_distance_sum;
                }
                if (lhs.loaded_pose_distance_l2 != rhs.loaded_pose_distance_l2) {
                  return lhs.loaded_pose_distance_l2 < rhs.loaded_pose_distance_l2;
                }
                if (lhs.loaded_pose_max_joint_delta != rhs.loaded_pose_max_joint_delta) {
                  return lhs.loaded_pose_max_joint_delta < rhs.loaded_pose_max_joint_delta;
                }
                return lhs.ik_score < rhs.ik_score;
              });
  }

  const size_t loaded_limit = options.candidate_limit > 0
    ? std::min(options.candidate_limit, batch_result.plan_indices.size())
    : batch_result.plan_indices.size();

  const auto start = std::chrono::steady_clock::now();
  for (size_t rank = 0; rank < batch_result.plan_indices.size(); ++rank) {
    auto& timing = timings[batch_result.plan_indices[rank]];
    timing.loaded_plan_rank = rank + 1;

    if (rank >= loaded_limit) {
      timing.loaded_plan_failure_reason = "loaded_plan_skipped_by_limit";
      continue;
    }

    const LoadedPosePlanResult result = plan(
      prefix + "/candidate_" + std::to_string(timing.candidate_order) + "/post_extract_loaded",
      *timing.final_state,
      carried_boxes,
      timing.loaded_plan_rank);

    timing.loaded_plan_attempted = result.attempted;
    timing.loaded_plan_success = result.success;
    timing.loaded_plan_ms = result.plan_ms;
    timing.loaded_plan_points = result.plan_points;
    timing.selected_left_loaded_pose_index = result.selection.left_index;
    timing.selected_right_loaded_pose_index = result.selection.right_index;
    timing.selected_left_loaded_pose_distance = result.selection.left_distance;
    timing.selected_right_loaded_pose_distance = result.selection.right_distance;
    timing.loaded_pose_distance_sum = result.selection.distance_sum;
    timing.loaded_pose_distance_l2 = result.selection.distance_l2;
    timing.loaded_pose_max_joint_delta = result.selection.max_joint_delta;
    timing.loaded_plan_failure_reason = result.failure_reason;

    if (options.stop_on_first_success && timing.loaded_plan_success) {
      for (size_t rest_rank = rank + 1; rest_rank < batch_result.plan_indices.size(); ++rest_rank) {
        auto& skipped = timings[batch_result.plan_indices[rest_rank]];
        skipped.loaded_plan_rank = rest_rank + 1;
        skipped.loaded_plan_failure_reason = "loaded_plan_skipped_after_first_success";
      }
      break;
    }
  }
  batch_result.wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  return batch_result;
}

}  // namespace alfa_robot::motion
