#include "alfa_robot_moveit_config/loaded_pose_planning.hpp"

#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace alfa_robot::motion
{

LoadedPoseSelector::LoadedPoseSelector(LoadedPoseSelectorConfig config)
: config_(std::move(config))
{
  if (config_.left_pose_family.empty()) {
    throw std::invalid_argument("LoadedPoseSelector requires at least one left loaded pose");
  }
  if (config_.right_pose_family.empty()) {
    throw std::invalid_argument("LoadedPoseSelector requires at least one right loaded pose");
  }
  config_.left_preferred_index = std::min(config_.left_preferred_index, config_.left_pose_family.size() - 1);
  config_.right_preferred_index = std::min(config_.right_preferred_index, config_.right_pose_family.size() - 1);
}

bool LoadedPoseSelector::hasVariable(const moveit::core::RobotState& state, const std::string& name) const
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
}

std::string LoadedPoseSelector::jointName(const std::string& side, size_t index)
{
  return side + "_v5_joint" + std::to_string(index + 1);
}

double LoadedPoseSelector::armPoseDistance(
  const moveit::core::RobotState& state,
  const std::string& side,
  const std::vector<double>& pose) const
{
  if (pose.size() < 6) return std::numeric_limits<double>::infinity();
  double squared_sum = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const std::string name = jointName(side, i);
    if (!hasVariable(state, name)) {
      return std::numeric_limits<double>::infinity();
    }
    const double diff = shortest_angular_distance(state.getVariablePosition(name), pose[i]);
    squared_sum += diff * diff;
  }
  return std::sqrt(squared_sum);
}

size_t LoadedPoseSelector::nearestPoseIndex(
  const moveit::core::RobotState& state,
  const std::string& side,
  const std::vector<std::vector<double>>& family,
  double* distance) const
{
  size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < family.size(); ++i) {
    const double candidate_distance = armPoseDistance(state, side, family[i]);
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best_index = i;
    }
  }
  if (distance) {
    *distance = best_distance;
  }
  return best_index;
}

std::array<double, 3> LoadedPoseSelector::distanceMetrics(
  const moveit::core::RobotState& state,
  size_t left_index,
  size_t right_index) const
{
  if (left_index >= config_.left_pose_family.size() || right_index >= config_.right_pose_family.size()) {
    return {
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()
    };
  }
  const auto& left_pose = config_.left_pose_family[left_index];
  const auto& right_pose = config_.right_pose_family[right_index];
  double abs_sum = 0.0;
  double squared_sum = 0.0;
  double max_delta = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const std::array<std::pair<std::string, const std::vector<double>*>, 2> arms = {{
      {jointName("left", i), &left_pose},
      {jointName("right", i), &right_pose},
    }};
    for (const auto& [name, pose] : arms) {
      if (!hasVariable(state, name) || pose->size() <= i) {
        return {
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity()
        };
      }
      const double delta = std::abs(shortest_angular_distance(state.getVariablePosition(name), (*pose)[i]));
      abs_sum += delta;
      squared_sum += delta * delta;
      max_delta = std::max(max_delta, delta);
    }
  }
  return {abs_sum, std::sqrt(squared_sum), max_delta};
}

LoadedPoseSelection LoadedPoseSelector::select(const moveit::core::RobotState& state) const
{
  LoadedPoseSelection selection;
  selection.left_index = nearestPoseIndex(
    state, "left", config_.left_pose_family, &selection.left_distance);
  selection.right_index = nearestPoseIndex(
    state, "right", config_.right_pose_family, &selection.right_distance);
  const auto metrics = distanceMetrics(state, selection.left_index, selection.right_index);
  selection.distance_sum = metrics[0];
  selection.distance_l2 = metrics[1];
  selection.max_joint_delta = metrics[2];
  return selection;
}

moveit::core::RobotState LoadedPoseSelector::makeGoalState(
  const moveit::core::RobotState& start_state,
  LoadedPoseSelection* selection) const
{
  moveit::core::RobotState goal_state(start_state);
  LoadedPoseSelection local_selection = select(start_state);
  if (selection) {
    *selection = local_selection;
  }

  const auto& left_pose = config_.left_pose_family[local_selection.left_index];
  const auto& right_pose = config_.right_pose_family[local_selection.right_index];
  if (hasVariable(goal_state, "updown")) {
    goal_state.setVariablePosition("updown", config_.target_updown);
  }
  for (size_t i = 0; i < left_pose.size(); ++i) {
    goal_state.setVariablePosition(jointName("left", i), left_pose[i]);
  }
  for (size_t i = 0; i < right_pose.size(); ++i) {
    goal_state.setVariablePosition(jointName("right", i), right_pose[i]);
  }
  if (config_.enforce_bounds_group) {
    goal_state.enforceBounds(config_.enforce_bounds_group);
  }
  goal_state.update();
  return goal_state;
}

}  // namespace alfa_robot::motion

#include "alfa_robot_moveit_config/motion_scene_adapter.hpp"

#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

namespace alfa_robot::motion
{

namespace
{

std::vector<std::string> touch_links_for_attached_box(const AttachedBoxSpec& box)
{
  std::vector<std::string> links{box.link_name};
  if (box.link_name.rfind("left_", 0) == 0) {
    links.push_back("left_v5_link6");
    links.push_back("left_v5_link5");
  } else if (box.link_name.rfind("right_", 0) == 0) {
    links.push_back("right_v5_link6");
    links.push_back("right_v5_link5");
  }
  return links;
}

void attach_boxes_to_robot_state(
  moveit::core::RobotState& state,
  const std::vector<AttachedBoxSpec>& boxes,
  double collision_padding)
{
  for (const auto& box : boxes) {
    const double size_x = std::max(0.001, box.size[0] + 2.0 * collision_padding);
    const double size_y = std::max(0.001, box.size[1] + 2.0 * collision_padding);
    const double size_z = std::max(0.001, box.size[2] + 2.0 * collision_padding);
    std::vector<shapes::ShapeConstPtr> shapes;
    shapes.push_back(std::make_shared<shapes::Box>(size_x, size_y, size_z));

    EigenSTL::vector_Isometry3d shape_poses;
    Eigen::Isometry3d shape_pose = Eigen::Isometry3d::Identity();
    shape_pose.translation() = Eigen::Vector3d(
      box.center_in_link[0],
      box.center_in_link[1],
      box.center_in_link[2]);
    shape_poses.push_back(shape_pose);

    state.attachBody(
      box.id,
      Eigen::Isometry3d::Identity(),
      shapes,
      shape_poses,
      touch_links_for_attached_box(box),
      box.link_name);
  }
  state.update(true);
}

}  // namespace

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

bool LoadedPosePlanner::isCenterColumnBox(const AttachedBoxSpec& box)
{
  const auto last_underscore = box.id.find_last_of('_');
  if (last_underscore == std::string::npos || last_underscore + 1 >= box.id.size()) {
    return false;
  }
  try {
    const int box_id = std::stoi(box.id.substr(last_underscore + 1));
    return box_id % 5 == 3;
  } catch (const std::exception&) {
    return false;
  }
}

bool LoadedPosePlanner::planLateralShift(
  const std::string& stage_name,
  const moveit::core::RobotState& start_state,
  const std::vector<AttachedBoxSpec>& carried_boxes,
  moveit::core::RobotState* shifted_state,
  LoadedPosePlanResult* result)
{
  if (!shifted_state || !result) return false;
  if (!config_.lateral_shift_enabled || config_.lateral_shift_distance <= 1e-6) {
    *shifted_state = start_state;
    return true;
  }

  const AttachedBoxSpec* center_box = nullptr;
  for (const auto& box : carried_boxes) {
    if (isCenterColumnBox(box)) {
      center_box = &box;
      break;
    }
  }
  if (!center_box) {
    *shifted_state = start_state;
    return true;
  }

  const std::string side = center_box->link_name.rfind("left_", 0) == 0 ? "left" : "right";
  const std::string tip_name = center_box->link_name;
  const std::string group_name = side + "_v5_arm";
  const moveit::core::JointModelGroup* arm_group =
    start_state.getRobotModel()->getJointModelGroup(group_name);
  if (!arm_group) {
    result->failure_reason = "lateral_shift_missing_group_" + group_name;
    return false;
  }

  const double direction_y = side == "left" ? 1.0 : -1.0;
  const double distance = std::abs(config_.lateral_shift_distance);
  const double step = std::max(0.01, std::abs(config_.lateral_shift_step));
  const size_t step_count = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / step)));

  moveit::core::RobotState current_state(start_state);
  moveit::planning_interface::MoveGroupInterface::Plan full_plan_for_record;
  const auto t0 = std::chrono::steady_clock::now();
  const auto finish_partial = [&](bool shifted_any) {
    result->lateral_shift_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    if (shifted_any) {
      result->lateral_shift_success = true;
      *shifted_state = current_state;
      return true;
    }
    return false;
  };

  for (size_t step_index = 1; step_index <= step_count; ++step_index) {
    const double shift = std::min(distance, step * static_cast<double>(step_index));
    const Eigen::Isometry3d start_tip = start_state.getGlobalLinkTransform(tip_name);
    Eigen::Isometry3d target_tip = start_tip;
    target_tip.translation().y() += direction_y * shift;

    moveit::core::RobotState target_state(current_state);
    bool ik_ok = false;
    {
      const auto set_from_ik = [&]() {
        return target_state.setFromIK(
          arm_group,
          target_tip,
          tip_name,
          0.02,
          moveit::core::GroupStateValidityCallbackFn());
      };
      ik_ok = set_from_ik();
    }
    if (!ik_ok) {
      result->failure_reason = "lateral_shift_ik_failed_step_" + std::to_string(step_index);
      return finish_partial(result->lateral_shift_reached_distance > 1e-6);
    }
    target_state.update();

    moveit::core::RobotState planning_start(current_state);
    moveit::core::RobotState planning_goal(target_state);
    attach_boxes_to_robot_state(planning_start, carried_boxes, config_.attached_box_collision_padding);
    attach_boxes_to_robot_state(planning_goal, carried_boxes, config_.attached_box_collision_padding);

    config_.move_group->setStartState(planning_start);
    config_.move_group->setJointValueTarget(planning_goal);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = config_.move_group->plan(plan);
    result->lateral_shift_attempted = true;
    result->lateral_shift_points += plan.trajectory_.joint_trajectory.points.size();
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      result->failure_reason =
        "lateral_shift_moveit_failed_step_" + std::to_string(step_index) +
        "_code_" + std::to_string(plan_result.val);
      return finish_partial(result->lateral_shift_reached_distance > 1e-6);
    }

    std::string clearance_reason;
    const bool clear = config_.clearance_callback
      ? config_.clearance_callback(plan, planning_start, &clearance_reason)
      : true;
    if (!clear) {
      result->failure_reason =
        "lateral_shift_collision_step_" + std::to_string(step_index) + ": " + clearance_reason;
      return finish_partial(result->lateral_shift_reached_distance > 1e-6);
    }

    if (config_.record_callback) {
      nlohmann::json extra = {
        {"stage_kind", "post_extract_lateral_shift"},
        {"valid", true},
        {"shift_side", side},
        {"shift_step", step_index},
        {"shift_step_count", step_count},
        {"shift_distance_y", direction_y * shift},
        {"target_box", center_box->id},
        {"carried_box_count", carried_boxes.size()}
      };
      config_.record_callback(
        stage_name + "/lateral_shift_step_" + std::to_string(step_index),
        plan,
        planning_start,
        planning_goal,
        config_.target_joint_names,
        extra);
    }

    current_state = target_state;
    result->lateral_shift_reached_distance = shift;
  }

  return finish_partial(true);
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

  moveit::core::RobotState start_state_with_boxes(extract_state);
  attach_boxes_to_robot_state(
    start_state_with_boxes,
    carried_boxes,
    config_.attached_box_collision_padding);

  moveit::core::RobotState loaded_start_state(start_state_with_boxes);
  moveit::core::RobotState shifted_state(extract_state);
  if (!planLateralShift(stage_name, extract_state, carried_boxes, &shifted_state, &result)) {
    restore_boxes();
    return result;
  }
  attach_boxes_to_robot_state(
    shifted_state,
    carried_boxes,
    config_.attached_box_collision_padding);
  loaded_start_state = shifted_state;

  moveit::core::RobotState goal_state = config_.selector->makeGoalState(
    loaded_start_state, &result.selection);

  config_.move_group->setStartState(loaded_start_state);
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
    ? config_.clearance_callback(plan, loaded_start_state, &carried_collision_reason)
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
      {"lateral_shift_enabled", config_.lateral_shift_enabled},
      {"lateral_shift_attempted", result.lateral_shift_attempted},
      {"lateral_shift_success", result.lateral_shift_success},
      {"lateral_shift_ms", result.lateral_shift_ms},
      {"lateral_shift_reached_distance", result.lateral_shift_reached_distance},
      {"lateral_shift_points", result.lateral_shift_points},
      {"loaded_plan_ms", result.plan_ms},
      {"loaded_plan_points", result.plan_points},
      {"start_updown", currentUpdown(loaded_start_state)},
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
      stage_name, plan, loaded_start_state, goal_state, config_.target_joint_names, extra);
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
    timing.lateral_shift_attempted = result.lateral_shift_attempted;
    timing.lateral_shift_success = result.lateral_shift_success;
    timing.lateral_shift_ms = result.lateral_shift_ms;
    timing.lateral_shift_reached_distance = result.lateral_shift_reached_distance;
    timing.lateral_shift_points = result.lateral_shift_points;
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
