#include <alfa_robot_moveit_config/wall_trajectory_postprocessing.hpp>

#include <chomp_motion_planner/chomp_optimizer.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace alfa_robot::motion
{
namespace
{
constexpr double kDiscretization = 0.1;
constexpr double kValidationStep = 0.03;

bool isFreeSpaceStage(const std::string& stage)
{
  return stage.rfind("rrt_", 0) == 0 || stage == "fold_arms_before_lift" ||
         stage == "dual_0" || stage == "dual_1" || stage == "dual_5";
}

bool sameLifecycle(const TrajectoryFrame& left, const TrajectoryFrame& right)
{
  return left.stage == right.stage && left.box_attached == right.box_attached &&
         left.box_visible == right.box_visible && left.scene_index == right.scene_index &&
         left.carried_boxes == right.carried_boxes;
}

std::vector<std::pair<size_t, size_t>> freeSpaceIntervals(
  const std::vector<TrajectoryFrame>& frames)
{
  std::vector<std::pair<size_t, size_t>> intervals;
  for (size_t begin = 0; begin < frames.size();) {
    size_t end = begin;
    while (end + 1 < frames.size() && sameLifecycle(frames[end], frames[end + 1])) ++end;
    if (isFreeSpaceStage(frames[begin].stage) && end - begin + 1 >= 3)
      intervals.emplace_back(begin, end);
    begin = end + 1;
  }
  return intervals;
}

bool attachPayloads(
  planning_scene::PlanningScenePtr scene,
  const planning_scene::PlanningSceneConstPtr& empty_scene,
  const std::vector<TrajectoryFrame>& frames,
  size_t interval_begin,
  const std::vector<std::string>& joint_names,
  const std::string& group_name,
  moveit::core::RobotState* start_state,
  std::string* reason)
{
  if (!empty_scene) {
    if (reason) *reason = "empty CHOMP scene is required to reconstruct the attached payload";
    return false;
  }

  size_t attachment_frame = frames.size();
  for (size_t index = 0; index <= interval_begin; ++index) {
    if (frames[index].scene_index == frames[interval_begin].scene_index && frames[index].box_attached &&
        (index == 0 || !frames[index - 1].box_attached ||
         frames[index - 1].scene_index != frames[index].scene_index)) {
      attachment_frame = index;
      break;
    }
  }
  if (attachment_frame == frames.size()) {
    if (reason) *reason = "CHOMP cannot find the payload attachment transition";
    return false;
  }

  moveit::core::RobotState attached_at(scene->getCurrentState());
  for (size_t joint = 0; joint < joint_names.size(); ++joint)
    attached_at.setVariablePosition(joint_names[joint], frames[attachment_frame].joints[joint]);
  attached_at.update(true);

  std::vector<std::pair<std::string, std::string>> payloads;
  for (const auto& box : frames[attachment_frame].carried_boxes) {
    if (!box.value("attached", false)) continue;
    const std::string side = box.value("side", "");
    const std::string tool = box.value("tool_link", side + "_tool0");
    if (!side.empty()) payloads.emplace_back("carried_target_box_" + side, tool);
  }
  if (payloads.empty()) {
    const std::string side = group_name.rfind("right_", 0) == 0 ? "right" :
      (group_name.rfind("left_", 0) == 0 ? "left" : "");
    if (side.empty()) {
      if (reason) *reason = "CHOMP whole-body payload metadata is missing";
      return false;
    }
    payloads.emplace_back("carried_target_box", side + "_tool0");
  }

  for (const auto& [object_id, tool_link] : payloads) {
    if (start_state->hasAttachedBody(object_id)) {
      scene->getWorldNonConst()->removeObject(object_id);
      continue;
    }
    const auto object = empty_scene->getWorld()->getObject(object_id);
    if (!object || !start_state->getRobotModel()->hasLinkModel(tool_link)) {
      if (reason) *reason = "CHOMP cannot reconstruct payload " + object_id + " on " + tool_link;
      return false;
    }
    EigenSTL::vector_Isometry3d shape_poses;
    shape_poses.reserve(object->global_shape_poses_.size());
    const Eigen::Isometry3d world_to_tool = attached_at.getGlobalLinkTransform(tool_link).inverse();
    for (const auto& global_pose : object->global_shape_poses_)
      shape_poses.push_back(world_to_tool * global_pose);
    const std::string side = tool_link.rfind("right_", 0) == 0 ? "right" : "left";
    start_state->attachBody(object_id, Eigen::Isometry3d::Identity(), object->shapes_, shape_poses,
      std::vector<std::string>{tool_link, side + "_joint7"}, tool_link);
    scene->getWorldNonConst()->removeObject(object_id);
  }
  start_state->update(true);
  return true;
}

bool validatePath(
  const std::vector<TrajectoryFrame>& frames, size_t begin, size_t end,
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& base_state,
  const std::vector<std::string>& joint_names,
  const moveit::core::JointModelGroup* group,
  std::string* reason)
{
  auto stateFor = [&](size_t index) {
    moveit::core::RobotState state(base_state);
    for (size_t joint = 0; joint < joint_names.size(); ++joint)
      state.setVariablePosition(joint_names[joint], frames[index].joints[joint]);
    state.update(true);
    return state;
  };

  moveit::core::RobotState previous = stateFor(begin);
  for (size_t index = begin; index <= end; ++index) {
    moveit::core::RobotState current = stateFor(index);
    if (!current.satisfiesBounds(group)) {
      if (reason) *reason = "CHOMP output violates joint bounds at frame " + std::to_string(index);
      return false;
    }
    if (scene->isStateColliding(current, group->getName())) {
      if (reason) *reason = "CHOMP output is in collision at frame " + std::to_string(index);
      return false;
    }
    if (index != begin) {
      double maximum_delta = 0.0;
      for (const std::string& name : group->getVariableNames())
        maximum_delta = std::max(maximum_delta, std::abs(
          current.getVariablePosition(name) - previous.getVariablePosition(name)));
      const size_t steps = std::max<size_t>(1, std::ceil(maximum_delta / kValidationStep));
      for (size_t step = 1; step < steps; ++step) {
        moveit::core::RobotState probe(previous);
        previous.interpolate(current, static_cast<double>(step) / steps, probe, group);
        probe.update(true);
        if (!probe.satisfiesBounds(group) || scene->isStateColliding(probe, group->getName())) {
          if (reason) *reason = "CHOMP output edge is invalid before frame " + std::to_string(index);
          return false;
        }
      }
    }
    previous = std::move(current);
  }
  return true;
}
}  // namespace

bool optimizeChompFreeSpace(
  std::vector<TrajectoryFrame>* frames,
  const moveit::core::RobotModelConstPtr& robot_model,
  const planning_scene::PlanningSceneConstPtr& empty_scene,
  const planning_scene::PlanningSceneConstPtr& loaded_scene,
  const std::vector<std::string>& joint_names,
  const std::string& group_name,
  double* wall_ms,
  std::string* reason)
{
  if (wall_ms) *wall_ms = 0.0;
  if (reason) reason->clear();
  if (!frames || frames->empty()) {
    if (reason) *reason = "empty CHOMP trajectory";
    return false;
  }

  const auto intervals = freeSpaceIntervals(*frames);
  if (intervals.empty()) return true;
  if (!robot_model) {
    if (reason) *reason = "CHOMP robot model is null";
    return false;
  }
  const auto* group = robot_model->getJointModelGroup(group_name);
  if (!group) {
    if (reason) *reason = "unknown CHOMP group: " + group_name;
    return false;
  }
  if (joint_names.empty()) {
    if (reason) *reason = "CHOMP joint list is empty";
    return false;
  }

  const auto& model_joint_names = robot_model->getVariableNames();
  std::unordered_map<std::string, size_t> frame_joint_index;
  for (size_t index = 0; index < joint_names.size(); ++index) {
    if (std::find(model_joint_names.begin(), model_joint_names.end(), joint_names[index]) ==
          model_joint_names.end() ||
        !frame_joint_index.emplace(joint_names[index], index).second) {
      if (reason) *reason = "invalid or duplicate CHOMP joint: " + joint_names[index];
      return false;
    }
  }
  for (const std::string& name : group->getVariableNames()) {
    if (!frame_joint_index.count(name)) {
      if (reason) *reason = "CHOMP group joint is missing from frames: " + name;
      return false;
    }
  }
  for (size_t index = 0; index < frames->size(); ++index) {
    if ((*frames)[index].joints.size() != joint_names.size() ||
        !std::all_of((*frames)[index].joints.begin(), (*frames)[index].joints.end(),
          [](double value) { return std::isfinite(value); })) {
      if (reason) *reason = "invalid CHOMP joint vector at frame " + std::to_string(index);
      return false;
    }
  }

  std::vector<TrajectoryFrame> optimized = *frames;
  planning_scene::PlanningScenePtr chomp_scenes[2];
  double elapsed_ms = 0.0;
  auto fail = [&](const std::string& message) {
    if (wall_ms) *wall_ms = elapsed_ms;
    if (reason) *reason = message;
    return false;
  };

  for (const auto& [begin, end] : intervals) {
    const bool attached = optimized[begin].box_attached;
    const auto& source_scene = attached ? loaded_scene : empty_scene;
    if (!source_scene)
      return fail(attached ? "loaded CHOMP scene is null" : "empty CHOMP scene is null");

    const auto started = std::chrono::steady_clock::now();
    try {
      auto& scene = chomp_scenes[attached ? 1 : 0];
      if (!scene) {
        scene = planning_scene::PlanningScene::clone(source_scene);
        scene->allocateCollisionDetector(
          collision_detection::CollisionDetectorAllocatorHybrid::create());
      }

      moveit::core::RobotState start_state(scene->getCurrentState());
      for (size_t joint = 0; joint < joint_names.size(); ++joint)
        start_state.setVariablePosition(joint_names[joint], optimized[begin].joints[joint]);
      start_state.update(true);
      if (attached && !attachPayloads(
          scene, empty_scene, optimized, begin, joint_names, group_name, &start_state, reason)) {
        const std::string attachment_reason = reason ? *reason : "CHOMP payload reconstruction failed";
        elapsed_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return fail(attachment_reason);
      }

      robot_trajectory::RobotTrajectory seed(robot_model, group_name);
      for (size_t index = begin; index <= end; ++index) {
        moveit::core::RobotState state(start_state);
        for (size_t joint = 0; joint < joint_names.size(); ++joint)
          state.setVariablePosition(joint_names[joint], optimized[index].joints[joint]);
        seed.addSuffixWayPoint(state, index == begin ? 0.0 : kDiscretization);
      }

      chomp::ChompTrajectory trajectory(
        robot_model, end - begin + 1, kDiscretization, group_name);
      if (!trajectory.fillInFromTrajectory(seed)) {
        elapsed_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return fail("failed to initialize CHOMP from the shortcut trajectory");
      }
      trajectory.setStartEndIndex(1, end - begin - 1);

      chomp::ChompParameters parameters;
      parameters.planning_time_limit_ = 0.5;
      parameters.max_iterations_ = 100;
      parameters.max_iterations_after_collision_free_ = 5;
      parameters.use_stochastic_descent_ = false;
      parameters.enable_failure_recovery_ = false;
      parameters.max_recovery_attempts_ = 0;
      if (!parameters.setTrajectoryInitializationMethod("fillTrajectory")) {
        elapsed_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return fail("installed CHOMP does not support fillTrajectory initialization");
      }

      chomp::ChompOptimizer optimizer(
        &trajectory, scene, group_name, &parameters, start_state);
      if (!optimizer.isInitialized() || !optimizer.optimize() || !optimizer.isCollisionFree()) {
        elapsed_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return fail("CHOMP failed to produce a collision-free " + optimized[begin].stage + " interval");
      }

      const auto& group_variables = group->getVariableNames();
      if (trajectory.getNumJoints() != group_variables.size()) {
        elapsed_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return fail("CHOMP output joint count does not match group " + group_name);
      }
      for (size_t index = begin + 1; index < end; ++index) {
        const size_t trajectory_index = index - begin;
        for (size_t group_joint = 0; group_joint < group_variables.size(); ++group_joint) {
          const std::string& name = group_variables[group_joint];
          optimized[index].joints[frame_joint_index.at(name)] =
            trajectory(trajectory_index, group_joint);
        }
      }

      if (!validatePath(optimized, begin, end, scene, start_state, joint_names, group, reason)) {
        const std::string validation_reason = reason ? *reason : "CHOMP output validation failed";
        elapsed_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return fail(validation_reason);
      }
    } catch (const std::exception& error) {
      elapsed_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      return fail(std::string("CHOMP exception: ") + error.what());
    }
    elapsed_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  }

  *frames = std::move(optimized);
  if (wall_ms) *wall_ms = elapsed_ms;
  return true;
}
}  // namespace alfa_robot::motion
