#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <moveit/robot_model/joint_model_group.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <tuple>
#include <utility>

namespace alfa_robot::motion
{

struct BoxPoseRrtExtractPlanner::ArmPath
{
  double joint_motion = std::numeric_limits<double>::infinity();
  std::vector<robot_motion::core::BoxPoseExtractState> box_states;
  std::vector<moveit::core::RobotStatePtr> states;
};

namespace
{

using BoxState = robot_motion::core::BoxPoseExtractState;
using BoxMode = robot_motion::core::BoxPoseExtractMode;

std::string box_state_key(const BoxState& state)
{
  std::ostringstream stream;
  stream << std::llround(state.retreat * 1e7) << ':'
         << std::llround(state.lift * 1e7) << ':'
         << std::llround(state.pitch * 1e7);
  return stream.str();
}

size_t edge_sample_count(
  const BoxState& from,
  const BoxState& to,
  const robot_motion::core::BoxPoseExtractRrtConfig& config)
{
  size_t count = 1;
  if (config.edge_resolution_retreat > 0.0) {
    count = std::max(count, static_cast<size_t>(std::ceil(
      std::abs(to.retreat - from.retreat) / config.edge_resolution_retreat)));
  }
  if (config.edge_resolution_lift > 0.0) {
    count = std::max(count, static_cast<size_t>(std::ceil(
      std::abs(to.lift - from.lift) / config.edge_resolution_lift)));
  }
  if (config.edge_resolution_pitch > 0.0) {
    count = std::max(count, static_cast<size_t>(std::ceil(
      std::abs(to.pitch - from.pitch) / config.edge_resolution_pitch)));
  }
  return count;
}

BoxState interpolate_box_state(const BoxState& from, const BoxState& to, double ratio)
{
  return {
    from.retreat + (to.retreat - from.retreat) * ratio,
    from.lift + (to.lift - from.lift) * ratio,
    from.pitch + (to.pitch - from.pitch) * ratio,
  };
}

double arm_joint_motion(
  const moveit::core::JointModelGroup* group,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to)
{
  if (!group) return std::numeric_limits<double>::infinity();
  double total = 0.0;
  for (const auto& name : group->getVariableNames()) {
    const double delta = std::atan2(
      std::sin(to.getVariablePosition(name) - from.getVariablePosition(name)),
      std::cos(to.getVariablePosition(name) - from.getVariablePosition(name)));
    total += std::abs(delta);
  }
  return total;
}

void copy_arm_state(
  const moveit::core::JointModelGroup* group,
  const moveit::core::RobotState& source,
  moveit::core::RobotState* target)
{
  if (!group || !target) return;
  for (const auto& name : group->getVariableNames()) {
    target->setVariablePosition(name, source.getVariablePosition(name));
  }
}

geometry_msgs::msg::Pose target_pose_for_box_state(
  const Eigen::Isometry3d& start_tip,
  const AttachedBoxSpec& carried_box,
  const BoxState& state,
  BoxMode mode)
{
  Eigen::Isometry3d target_tip = start_tip;
  if (mode == BoxMode::TopTranslate) {
    target_tip.translation().x() -= state.retreat;
    target_tip.translation().z() += state.lift;
  } else {
    Eigen::Isometry3d tool_to_box = Eigen::Isometry3d::Identity();
    tool_to_box.translation() = Eigen::Vector3d(
      carried_box.center_in_link[0],
      carried_box.center_in_link[1],
      carried_box.center_in_link[2]);
    const Eigen::Isometry3d start_box = start_tip * tool_to_box;
    Eigen::Vector3d pivot_in_box = Eigen::Vector3d::Zero();
    double best_pivot_score = std::numeric_limits<double>::infinity();
    for (const double sx : {-0.5, 0.5}) {
      for (const double sy : {-0.5, 0.5}) {
        for (const double sz : {-0.5, 0.5}) {
          const Eigen::Vector3d corner(
            sx * carried_box.size[0],
            sy * carried_box.size[1],
            sz * carried_box.size[2]);
          const Eigen::Vector3d world_corner = start_box * corner;
          const double score = 100.0 * world_corner.z() + world_corner.x();
          if (score < best_pivot_score) {
            best_pivot_score = score;
            pivot_in_box = corner;
          }
        }
      }
    }
    const Eigen::Vector3d start_pivot = start_box * pivot_in_box;
    double rotation_direction = 1.0;
    double best_probe_x = std::numeric_limits<double>::infinity();
    double best_final_normal_z = std::numeric_limits<double>::infinity();
    for (const double candidate_direction : {1.0, -1.0}) {
      constexpr double probe_pitch = M_PI / 180.0;
      const Eigen::Matrix3d probe_rotation =
        Eigen::AngleAxisd(candidate_direction * probe_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        start_box.linear();
      Eigen::Isometry3d probe_box = Eigen::Isometry3d::Identity();
      probe_box.linear() = probe_rotation;
      probe_box.translation() = start_pivot - probe_rotation * pivot_in_box;
      const Eigen::Isometry3d probe_tip = probe_box * tool_to_box.inverse();
      const Eigen::Vector3d probe_delta = probe_tip.translation() - start_tip.translation();

      const Eigen::Matrix3d final_rotation =
        Eigen::AngleAxisd(candidate_direction * M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        start_box.linear();
      Eigen::Isometry3d final_box = Eigen::Isometry3d::Identity();
      final_box.linear() = final_rotation;
      final_box.translation() = start_pivot - final_rotation * pivot_in_box;
      const Eigen::Isometry3d final_tip = final_box * tool_to_box.inverse();
      const Eigen::Vector3d final_tool_normal = final_tip.linear() * Eigen::Vector3d::UnitZ();

      if (probe_delta.x() < best_probe_x - 1e-9 ||
          (std::abs(probe_delta.x() - best_probe_x) <= 1e-9 &&
           final_tool_normal.z() < best_final_normal_z)) {
        best_probe_x = probe_delta.x();
        best_final_normal_z = final_tool_normal.z();
        rotation_direction = candidate_direction;
      }
    }
    const Eigen::Matrix3d box_rotation =
      Eigen::AngleAxisd(rotation_direction * state.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      start_box.linear();
    Eigen::Isometry3d target_box = Eigen::Isometry3d::Identity();
    target_box.linear() = box_rotation;
    target_box.translation() = start_pivot - Eigen::Vector3d(state.retreat, 0.0, 0.0) -
      box_rotation * pivot_in_box;
    target_tip = target_box * tool_to_box.inverse();
  }
  Eigen::Quaterniond orientation(target_tip.linear());
  orientation.normalize();
  return make_pose(
    target_tip.translation().x(), target_tip.translation().y(), target_tip.translation().z(), orientation);
}

}  // namespace

BoxPoseRrtExtractPlanner::BoxPoseRrtExtractPlanner(BoxPoseRrtExtractPlannerConfig config)
: config_(std::move(config))
{}

std::vector<BoxPoseRrtExtractPlanner::ArmPath> BoxPoseRrtExtractPlanner::planArm(
  const std::string& side,
  const moveit::core::RobotState& start_state,
  const AttachedBoxSpec& carried_box,
  bool top_suction) const
{
  std::vector<ArmPath> paths;
  if (!config_.candidate_solver) return paths;
  const auto* arm_group = side == "left" ? config_.left_arm_group : config_.right_arm_group;
  const std::string& tip = side == "left" ? config_.left_tip : config_.right_tip;
  if (!arm_group || !start_state.knowsFrameTransform(tip)) return paths;

  auto rrt_config = top_suction ? config_.top_rrt : config_.front_rrt;
  rrt_config.mode = top_suction ? BoxMode::TopTranslate : BoxMode::FrontPivot;
  rrt_config.max_solution_count = std::max<size_t>(1, config_.max_paths_per_arm);
  robot_motion::core::BoxPoseExtractRrt rrt(rrt_config);
  const Eigen::Isometry3d start_tip = start_state.getGlobalLinkTransform(tip);
  const auto& variable_names = start_state.getRobotModel()->getVariableNames();
  const double fixed_updown =
    std::find(variable_names.begin(), variable_names.end(), "updown") != variable_names.end() ?
    start_state.getVariablePosition("updown") : 0.0;
  std::map<std::string, moveit::core::RobotStatePtr> state_cache;
  std::map<std::string, size_t> rejection_counts;
  state_cache.emplace(box_state_key(BoxState{}), std::make_shared<moveit::core::RobotState>(start_state));

  auto solve_edge = [&](
    const BoxState& from,
    const BoxState& to,
    const moveit::core::RobotState& edge_start,
    std::vector<moveit::core::RobotStatePtr>* dense_states,
    double* joint_motion,
    std::string* reason) -> bool {
      moveit::core::RobotState current(edge_start);
      double motion = 0.0;
      const size_t sample_count = edge_sample_count(from, to, rrt_config);
      for (size_t sample_index = 1; sample_index <= sample_count; ++sample_index) {
        const double ratio = static_cast<double>(sample_index) / static_cast<double>(sample_count);
        const BoxState sample = interpolate_box_state(from, to, ratio);
        ExtractCandidate candidate;
        ExtractCandidateSolveRequest request;
        request.side = side;
        request.current_state = &current;
        request.target_pose = target_pose_for_box_state(
          start_tip, carried_box, sample, rrt_config.mode);
        request.step_index = sample_index;
        request.candidate_index = sample_index;
        request.retreat_x = sample.retreat;
        request.retreat_delta_x = sample.retreat - from.retreat;
        request.lift_z = sample.lift;
        request.lift_delta_z = sample.lift - from.lift;
        request.pitch_up_rad = sample.pitch;
        request.pitch_delta_rad = sample.pitch - from.pitch;
        request.min_allowed_tip_z = top_suction ?
          start_tip.translation().z() :
          start_tip.translation().z() - 0.5 * rrt_config.box_height - 0.01;
        request.fixed_updown = fixed_updown;
        request.min_tool_normal_z = -1.0;
        request.top_suction = top_suction;
        if (!config_.candidate_solver->solve(request, &candidate) || !candidate.state) {
          if (reason) {
            *reason = candidate.rejection_reason.empty() ?
              side + "_box_pose_rrt_analytic_no_solution" : candidate.rejection_reason;
          }
          return false;
        }
        motion += arm_joint_motion(arm_group, current, *candidate.state);
        current = *candidate.state;
        if (dense_states) dense_states->push_back(candidate.state);
      }
      state_cache[box_state_key(to)] = std::make_shared<moveit::core::RobotState>(current);
      if (joint_motion) *joint_motion = motion;
      return true;
    };

  const auto evaluator = [&](const BoxState& from, const BoxState& to) {
    robot_motion::core::BoxPoseExtractEdgeEvaluation evaluation;
    const auto found = state_cache.find(box_state_key(from));
    if (found == state_cache.end() || !found->second) {
      evaluation.rejection_reason = side + "_box_pose_rrt_missing_parent_state";
      return evaluation;
    }
    evaluation.valid = solve_edge(
      from, to, *found->second, nullptr, &evaluation.joint_motion, &evaluation.rejection_reason);
    if (!evaluation.valid) {
      rejection_counts[evaluation.rejection_reason.empty() ?
        side + "_box_pose_rrt_edge_rejected" : evaluation.rejection_reason]++;
    }
    return evaluation;
  };

  const auto result = rrt.plan(BoxState{}, evaluator);
  if (!result.success) {
    std::string dominant_reason = result.failure_reason;
    size_t dominant_count = 0;
    for (const auto& [reason, count] : rejection_counts) {
      if (count > dominant_count) {
        dominant_count = count;
        dominant_reason = reason;
      }
    }
    RCLCPP_WARN(
      config_.logger,
      "%s box-pose RRT failed: mode=%s iterations=%zu edges=%zu dominant=%s count=%zu",
      side.c_str(), top_suction ? "top" : "front", result.iterations,
      result.edge_evaluations, dominant_reason.c_str(), dominant_count);
  }
  for (const auto& rrt_path : result.paths) {
    ArmPath path;
    path.box_states = rrt_path.states;
    path.states.push_back(std::make_shared<moveit::core::RobotState>(start_state));
    moveit::core::RobotState current(start_state);
    double total_motion = 0.0;
    bool valid = true;
    for (size_t index = 1; index < rrt_path.states.size(); ++index) {
      std::vector<moveit::core::RobotStatePtr> edge_states;
      double edge_motion = 0.0;
      std::string reason;
      if (!solve_edge(
          rrt_path.states[index - 1], rrt_path.states[index], current,
          &edge_states, &edge_motion, &reason)) {
        valid = false;
        break;
      }
      total_motion += edge_motion;
      for (const auto& state : edge_states) {
        path.states.push_back(state);
        current = *state;
      }
    }
    if (valid && !path.states.empty()) {
      path.joint_motion = total_motion;
      paths.push_back(std::move(path));
    }
  }
  std::sort(paths.begin(), paths.end(), [](const ArmPath& lhs, const ArmPath& rhs) {
    return lhs.joint_motion < rhs.joint_motion;
  });
  if (paths.size() > config_.max_paths_per_arm) paths.resize(config_.max_paths_per_arm);
  return paths;
}

ExtractRolloutTiming BoxPoseRrtExtractPlanner::rolloutDual(
  const moveit::core::RobotState& start_state,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  size_t candidate_order,
  size_t h_index,
  size_t seed_index,
  double h,
  double ik_score,
  double ik_solve_ms,
  bool left_top_suction,
  bool right_top_suction,
  const ExtractRecordStepCallback& record_step) const
{
  const auto started = std::chrono::steady_clock::now();
  ExtractRolloutTiming timing;
  timing.candidate_order = candidate_order;
  timing.h_index = h_index;
  timing.seed_index = seed_index;
  timing.h = h;
  timing.ik_score = ik_score;
  timing.ik_solve_ms = ik_solve_ms;

  const auto left_paths = planArm("left", start_state, left_box, left_top_suction);
  const auto right_paths = planArm("right", start_state, right_box, right_top_suction);
  RCLCPP_INFO(
    config_.logger,
    "box-pose RRT arm paths: candidate=%zu left=%zu right=%zu modes=(%s,%s)",
    candidate_order, left_paths.size(), right_paths.size(),
    left_top_suction ? "top" : "front", right_top_suction ? "top" : "front");
  if (left_paths.empty() || right_paths.empty()) {
    timing.failure_reason = left_paths.empty() ?
      "box_pose_rrt_left_no_reachable_path" : "box_pose_rrt_right_no_reachable_path";
    timing.rollout_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    return timing;
  }

  const auto diagnose_arm_paths = [&](
    const std::string& moving_side,
    const std::vector<ArmPath>& arm_paths) {
      size_t collision_free_paths = 0;
      std::map<std::string, size_t> failures;
      for (const auto& path : arm_paths) {
        bool path_clear = true;
        for (size_t step = 0; step < path.states.size(); ++step) {
          auto combined = std::make_shared<moveit::core::RobotState>(start_state);
          if (moving_side == "left") {
            copy_arm_state(config_.left_arm_group, *path.states[step], combined.get());
          } else {
            copy_arm_state(config_.right_arm_group, *path.states[step], combined.get());
          }
          combined->enforceBounds(config_.joint_group);
          combined->update(true);
          bool left_detached = false;
          bool right_detached = false;
          std::string reason;
          if (!config_.dual_clear_callback || !config_.dual_clear_callback(
              *combined, left_box, left_box_id, right_box, right_box_id,
              &left_detached, &right_detached, &reason)) {
            path_clear = false;
            failures[(reason.empty() ? "collision_rejected" : reason) +
              " first_step=" + std::to_string(step)]++;
            break;
          }
        }
        if (path_clear) ++collision_free_paths;
      }
      std::string dominant = "none";
      size_t dominant_count = 0;
      for (const auto& [reason, count] : failures) {
        if (count > dominant_count) {
          dominant = reason;
          dominant_count = count;
        }
      }
      RCLCPP_INFO(
        config_.logger,
        "box-pose RRT isolated arm validation: side=%s clear=%zu/%zu dominant=%s count=%zu",
        moving_side.c_str(), collision_free_paths, arm_paths.size(), dominant.c_str(), dominant_count);
    };
  if (config_.diagnose_isolated_arm_paths) {
    diagnose_arm_paths("left", left_paths);
    diagnose_arm_paths("right", right_paths);
  }

  struct Pair { size_t left; size_t right; double cost; };
  std::vector<Pair> pairs;
  for (size_t left = 0; left < left_paths.size(); ++left) {
    for (size_t right = 0; right < right_paths.size(); ++right) {
      pairs.push_back({left, right, left_paths[left].joint_motion + right_paths[right].joint_motion});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair& lhs, const Pair& rhs) {
    return lhs.cost < rhs.cost;
  });
  if (pairs.size() > config_.max_path_pairs_to_validate) {
    pairs.resize(config_.max_path_pairs_to_validate);
  }

  std::map<std::string, size_t> failure_counts;
  bool collision_diagnostic_recorded = false;
  for (size_t pair_rank = 0; pair_rank < pairs.size(); ++pair_rank) {
    const auto& left = left_paths[pairs[pair_rank].left];
    const auto& right = right_paths[pairs[pair_rank].right];
    const size_t step_count = std::max(left.states.size(), right.states.size());
    std::vector<moveit::core::RobotStatePtr> combined_states;
    combined_states.reserve(step_count);
    bool collision_free = true;
    bool final_left_detached = false;
    bool final_right_detached = false;
    std::string failure_reason;
    for (size_t step = 0; step < step_count; ++step) {
      const double progress = step_count <= 1 ? 1.0 :
        static_cast<double>(step) / static_cast<double>(step_count - 1);
      const size_t left_index = std::min(
        left.states.size() - 1,
        static_cast<size_t>(std::llround(progress * static_cast<double>(left.states.size() - 1))));
      const size_t right_index = std::min(
        right.states.size() - 1,
        static_cast<size_t>(std::llround(progress * static_cast<double>(right.states.size() - 1))));
      auto combined = std::make_shared<moveit::core::RobotState>(start_state);
      copy_arm_state(config_.left_arm_group, *left.states[left_index], combined.get());
      copy_arm_state(config_.right_arm_group, *right.states[right_index], combined.get());
      combined->enforceBounds(config_.joint_group);
      combined->update(true);
      bool left_detached = false;
      bool right_detached = false;
      std::string reason;
      if (!config_.dual_clear_callback || !config_.dual_clear_callback(
          *combined, left_box, left_box_id, right_box, right_box_id,
          &left_detached, &right_detached, &reason)) {
        collision_free = false;
        failure_reason = (reason.empty() ? "box_pose_rrt_full_collision_rejected" : reason) +
          " first_step=" + std::to_string(step);
        if (record_step && !collision_diagnostic_recorded) {
          const auto& previous = combined_states.empty() ? start_state : *combined_states.back();
          record_step(0, previous, {
            {"stage_kind", "box_pose_rrt_collision_previous"},
            {"path_pair_rank", pair_rank + 1},
            {"path_pair_joint_motion", pairs[pair_rank].cost},
            {"accepted", true},
            {"collision_diagnostic", true},
            {"collision_step", step},
            {"collision_reason", failure_reason},
            {"left_top_suction", left_top_suction},
            {"right_top_suction", right_top_suction}
          });
          record_step(1, *combined, {
            {"stage_kind", "box_pose_rrt_collision_frame"},
            {"path_pair_rank", pair_rank + 1},
            {"path_pair_joint_motion", pairs[pair_rank].cost},
            {"accepted", false},
            {"collision_diagnostic", true},
            {"collision_step", step},
            {"collision_reason", failure_reason},
            {"left_top_suction", left_top_suction},
            {"right_top_suction", right_top_suction}
          });
          collision_diagnostic_recorded = true;
        }
        break;
      }
      final_left_detached = left_detached;
      final_right_detached = right_detached;
      combined_states.push_back(std::move(combined));
    }
    if (!collision_free) {
      failure_counts[failure_reason]++;
      continue;
    }
    if (!final_left_detached || !final_right_detached) {
      failure_counts["box_pose_rrt_final_not_detached"]++;
      continue;
    }

    timing.success = true;
    timing.accepted_steps = combined_states.size();
    timing.final_state = combined_states.back();
    timing.final_retreat_x = left.box_states.back().retreat;
    timing.final_lift_z = left.box_states.back().lift;
    timing.final_pitch_deg = left.box_states.back().pitch * 180.0 / M_PI;
    timing.right_final_retreat_x = right.box_states.back().retreat;
    timing.right_final_lift_z = right.box_states.back().lift;
    timing.right_final_pitch_deg = right.box_states.back().pitch * 180.0 / M_PI;
    if (record_step) {
      for (size_t step = 0; step < combined_states.size(); ++step) {
        record_step(step, *combined_states[step], {
          {"stage_kind", "box_pose_rrt_extract_step"},
          {"path_pair_rank", pair_rank + 1},
          {"path_pair_joint_motion", pairs[pair_rank].cost},
          {"accepted", true},
          {"left_top_suction", left_top_suction},
          {"right_top_suction", right_top_suction}
        });
      }
    }
    break;
  }

  if (!timing.success) {
    timing.failed_steps = collision_diagnostic_recorded ? 1 : 0;
    timing.failure_reason = "box_pose_rrt_no_collision_free_path_pair";
    size_t largest_count = 0;
    for (const auto& [reason, count] : failure_counts) {
      if (count > largest_count) {
        largest_count = count;
        timing.failure_reason = reason;
      }
    }
    RCLCPP_WARN(
      config_.logger,
      "box-pose RRT pair validation failed: candidate=%zu pairs=%zu dominant=%s count=%zu",
      candidate_order, pairs.size(), timing.failure_reason.c_str(), largest_count);
  }
  timing.rollout_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return timing;
}

}  // namespace alfa_robot::motion
