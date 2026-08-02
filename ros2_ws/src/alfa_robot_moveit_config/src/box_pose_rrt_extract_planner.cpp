#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <moveit/robot_model/joint_model_group.h>

#include <algorithm>
#include <array>
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
         << std::llround(state.pitch * 1e7) << ':'
         << std::llround(state.lateral * 1e7);
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
  if (config.edge_resolution_lateral > 0.0) {
    count = std::max(count, static_cast<size_t>(std::ceil(
      std::abs(to.lateral - from.lateral) / config.edge_resolution_lateral)));
  }
  return count;
}

BoxState interpolate_box_state(const BoxState& from, const BoxState& to, double ratio)
{
  return {
    from.retreat + (to.retreat - from.retreat) * ratio,
    from.lift + (to.lift - from.lift) * ratio,
    from.pitch + (to.pitch - from.pitch) * ratio,
    from.lateral + (to.lateral - from.lateral) * ratio,
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
    const double delta = to.getVariablePosition(name) - from.getVariablePosition(name);
    total += std::abs(delta);
  }
  return total;
}

double arm_joint_limit_margin_cost(
  const moveit::core::JointModelGroup* group,
  const moveit::core::RobotState& state)
{
  if (!group) return 0.0;
  constexpr double free_ratio = 0.6;
  constexpr double min_denominator = 1e-3;
  const std::array<double, 6> weights{{0.5, 3.0, 0.7, 0.5, 1.5, 1.2}};
  double cost = 0.0;
  const auto& names = group->getVariableNames();
  for (size_t index = 0; index < names.size(); ++index) {
    const auto& bounds = state.getRobotModel()->getVariableBounds(names[index]);
    if (!bounds.position_bounded_) continue;
    const double half_range = 0.5 * (bounds.max_position_ - bounds.min_position_);
    if (half_range <= 1e-9) continue;
    const double center = 0.5 * (bounds.max_position_ + bounds.min_position_);
    const double normalized = std::abs(state.getVariablePosition(names[index]) - center) / half_range;
    if (normalized <= free_ratio) continue;
    const double excess = (normalized - free_ratio) / (1.0 - free_ratio);
    const double barrier = excess * excess / std::max(min_denominator, 1.0 - normalized);
    const double weight = index < weights.size() ? weights[index] : 1.0;
    cost += weight * barrier;
  }
  return cost;
}

double max_group_joint_delta(
  const moveit::core::JointModelGroup* group,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to)
{
  if (!group) return 0.0;
  double max_delta = 0.0;
  for (const auto& name : group->getVariableNames()) {
    max_delta = std::max(max_delta, std::abs(to.getVariablePosition(name) - from.getVariablePosition(name)));
  }
  return max_delta;
}

moveit::core::RobotStatePtr interpolate_group_state(
  const moveit::core::JointModelGroup* group,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to,
  double ratio)
{
  auto state = std::make_shared<moveit::core::RobotState>(from);
  if (!group) return state;
  for (const auto& name : group->getVariableNames()) {
    const double position =
      from.getVariablePosition(name) +
      (to.getVariablePosition(name) - from.getVariablePosition(name)) * ratio;
    state->setVariablePosition(name, position);
  }
  state->enforceBounds(group);
  state->update(true);
  return state;
}

bool extract_segment_clear(
  const BoxPoseRrtExtractPlannerConfig& config,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  std::string* reason)
{
  if (!config.joint_group || !config.dual_clear_callback) {
    if (reason) *reason = "extract_smooth_missing_collision_callback";
    return false;
  }
  constexpr double max_step_rad = 5.0 * M_PI / 180.0;
  const size_t sample_count = std::max<size_t>(
    1, static_cast<size_t>(std::ceil(max_group_joint_delta(config.joint_group, from, to) / max_step_rad)));
  for (size_t sample = 1; sample <= sample_count; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(sample_count);
    const auto state = interpolate_group_state(config.joint_group, from, to, ratio);
    bool left_detached = false;
    bool right_detached = false;
    std::string clear_reason;
    if (!config.dual_clear_callback(
        *state, left_box, left_box_id, right_box, right_box_id,
        &left_detached, &right_detached, &clear_reason)) {
      if (reason) *reason = clear_reason.empty() ? "extract_smooth_collision_rejected" : clear_reason;
      return false;
    }
  }
  return true;
}

struct SmoothedCombinedPath
{
  std::vector<moveit::core::RobotStatePtr> states;
  std::vector<double> progresses;
  size_t removed_reference_points = 0;
};

SmoothedCombinedPath smooth_combined_extract_path(
  const BoxPoseRrtExtractPlannerConfig& config,
  const std::vector<moveit::core::RobotStatePtr>& reference_states,
  const std::vector<double>& reference_progresses,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id)
{
  SmoothedCombinedPath smoothed;
  if (reference_states.size() < 3 || reference_states.size() != reference_progresses.size()) {
    smoothed.states = reference_states;
    smoothed.progresses = reference_progresses;
    return smoothed;
  }

  smoothed.states.push_back(reference_states.front());
  smoothed.progresses.push_back(reference_progresses.front());

  size_t from = 0;
  while (from + 1 < reference_states.size()) {
    size_t best = from + 1;
    for (size_t target = reference_states.size() - 1; target > from + 1; --target) {
      std::string reason;
      if (extract_segment_clear(
          config, *reference_states[from], *reference_states[target],
          left_box, left_box_id, right_box, right_box_id, &reason)) {
        best = target;
        break;
      }
    }

    constexpr double max_step_rad = 5.0 * M_PI / 180.0;
    const size_t sample_count = std::max<size_t>(
      1,
      static_cast<size_t>(std::ceil(
        max_group_joint_delta(config.joint_group, *reference_states[from], *reference_states[best]) /
        max_step_rad)));
    for (size_t sample = 1; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(sample_count);
      smoothed.states.push_back(interpolate_group_state(
        config.joint_group, *reference_states[from], *reference_states[best], ratio));
      smoothed.progresses.push_back(
        reference_progresses[from] +
        (reference_progresses[best] - reference_progresses[from]) * ratio);
    }
    from = best;
  }

  smoothed.removed_reference_points =
    reference_states.size() > smoothed.states.size() ?
    reference_states.size() - smoothed.states.size() : 0;
  return smoothed;
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
    target_tip.translation().y() += state.lateral;
    target_tip.translation().z() += state.lift;
    target_tip.linear() =
      Eigen::AngleAxisd(-state.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      start_tip.linear();
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
      box_rotation * pivot_in_box + Eigen::Vector3d(0.0, state.lateral, state.lift);
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
  int box_id,
  bool top_suction,
  const BoxPoseRrtArmPolicy& policy,
  ArmPath* diagnostic_path) const
{
  std::vector<ArmPath> paths;
  if (diagnostic_path) {
    *diagnostic_path = ArmPath{};
  }
  if (!config_.candidate_solver) return paths;
  const auto* arm_group = side == "left" ? config_.left_arm_group : config_.right_arm_group;
  const std::string& tip = side == "left" ? config_.left_tip : config_.right_tip;
  if (!arm_group || !start_state.knowsFrameTransform(tip)) return paths;

  auto rrt_config = top_suction ? config_.top_rrt : config_.front_rrt;
  rrt_config.mode = top_suction ? BoxMode::TopTranslate : BoxMode::FrontPivot;
  rrt_config.source_reference_offset_z = policy.detachment_reference_offset_z;
  rrt_config.max_solution_count = std::max<size_t>(1, config_.max_paths_per_arm);
  robot_motion::core::BoxPoseExtractRrt rrt(rrt_config);
  const Eigen::Isometry3d start_tip = start_state.getGlobalLinkTransform(tip);
  const auto& variable_names = start_state.getRobotModel()->getVariableNames();
  const double fixed_updown =
    std::find(variable_names.begin(), variable_names.end(), "updown") != variable_names.end() ?
    start_state.getVariablePosition("updown") : 0.0;
  std::map<std::string, moveit::core::RobotStatePtr> state_cache;
  std::map<std::string, robot_motion::core::BoxPoseExtractEdgeEvaluation> edge_cache;
  std::map<std::string, size_t> rejection_counts;
  state_cache.emplace(box_state_key(BoxState{}), std::make_shared<moveit::core::RobotState>(start_state));

  auto solve_edge = [&](
    const BoxState& from,
    const BoxState& to,
    const moveit::core::RobotState& edge_start,
    std::vector<moveit::core::RobotStatePtr>* dense_states,
    double* joint_motion,
    double* endpoint_score,
    bool* goal_evaluated,
    bool* goal_reached,
    std::string* reason) -> bool {
      moveit::core::RobotState current(edge_start);
      double motion = 0.0;
      const size_t sample_count = rrt_config.endpoint_only_edges ? 1 : edge_sample_count(from, to, rrt_config);
      bool final_detached = false;
      bool final_detached_evaluated = false;
      ExtractCandidate final_candidate;
      bool has_final_candidate = false;
      for (size_t sample_index = 1; sample_index <= sample_count; ++sample_index) {
        if (config_.profile) config_.profile->node_evaluations.fetch_add(1, std::memory_order_relaxed);
        const double ratio = static_cast<double>(sample_index) / static_cast<double>(sample_count);
        const BoxState sample = interpolate_box_state(from, to, ratio);
        ExtractCandidate candidate;
        ExtractCandidateSolveRequest request;
        request.side = side;
        request.current_state = &current;
        const auto target_started = std::chrono::steady_clock::now();
        request.target_pose = target_pose_for_box_state(
          start_tip, carried_box, sample, rrt_config.mode);
        if (config_.profile) {
          config_.profile->target_pose_ns.fetch_add(
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - target_started).count()),
            std::memory_order_relaxed);
        }
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
        const auto ik_started = std::chrono::steady_clock::now();
        const bool solved = config_.candidate_solver->solve(request, &candidate) && candidate.state;
        if (config_.profile) {
          config_.profile->ik_ns.fetch_add(
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - ik_started).count()),
            std::memory_order_relaxed);
        }
        if (!solved) {
          if (reason) {
            *reason = candidate.rejection_reason.empty() ?
              side + "_box_pose_rrt_analytic_no_solution" : candidate.rejection_reason;
          }
          return false;
        }
        if (config_.single_clear_callback) {
          const auto clear_started = std::chrono::steady_clock::now();
          bool detached = false;
          std::string clear_reason;
          const bool clear = config_.single_clear_callback(
            *candidate.state, carried_box, box_id, &detached, &clear_reason);
          if (config_.profile) {
            config_.profile->clear_ns.fetch_add(
              static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - clear_started).count()),
              std::memory_order_relaxed);
          }
          if (!clear) {
            if (reason) {
              *reason = clear_reason.empty() ?
                side + "_box_pose_rrt_carried_collision" : clear_reason;
            }
              return false;
          }
          final_detached = top_suction ? rrt.goalReached(sample) : detached;
          final_detached_evaluated = true;
        }
        motion += arm_joint_motion(arm_group, current, *candidate.state);
        final_candidate = candidate;
        has_final_candidate = true;
        current = *candidate.state;
        if (dense_states) dense_states->push_back(candidate.state);
      }
      state_cache[box_state_key(to)] = std::make_shared<moveit::core::RobotState>(current);
      if (joint_motion) *joint_motion = motion;
      if (endpoint_score) {
        *endpoint_score = motion + 0.5 * arm_joint_limit_margin_cost(arm_group, current);
        if (has_final_candidate && config_.candidate_scorer) {
          *endpoint_score = config_.candidate_scorer->score(side, final_candidate, edge_start, from.retreat);
        }
      }
      if (goal_evaluated) *goal_evaluated = final_detached_evaluated;
      if (goal_reached) *goal_reached = final_detached;
      return true;
    };

  const auto evaluator = [&](const BoxState& from, const BoxState& to) {
    robot_motion::core::BoxPoseExtractEdgeEvaluation evaluation;
    const std::string from_key = box_state_key(from);
    const std::string to_key = box_state_key(to);
    const std::string edge_key = from_key + "->" + to_key;
    const auto cached = edge_cache.find(edge_key);
    if (cached != edge_cache.end()) {
      if (!cached->second.valid || state_cache.find(to_key) != state_cache.end()) {
        if (!cached->second.valid) {
          rejection_counts[cached->second.rejection_reason.empty() ?
            side + "_box_pose_rrt_edge_rejected" : cached->second.rejection_reason]++;
        }
        return cached->second;
      }
    }
    const auto found = state_cache.find(box_state_key(from));
    if (found == state_cache.end() || !found->second) {
      evaluation.rejection_reason = side + "_box_pose_rrt_missing_parent_state";
      edge_cache[edge_key] = evaluation;
      return evaluation;
    }
    evaluation.valid = solve_edge(
      from, to, *found->second, nullptr, &evaluation.joint_motion, &evaluation.endpoint_score,
      &evaluation.goal_evaluated, &evaluation.goal_reached, &evaluation.rejection_reason);
    if (!evaluation.valid) {
      rejection_counts[evaluation.rejection_reason.empty() ?
        side + "_box_pose_rrt_edge_rejected" : evaluation.rejection_reason]++;
    }
    edge_cache[edge_key] = evaluation;
    return evaluation;
  };

  const auto rrt_started = std::chrono::steady_clock::now();
  const auto result = rrt.plan(BoxState{}, evaluator);
  if (config_.profile) {
    config_.profile->rrt_plan_ns.fetch_add(
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - rrt_started).count()),
      std::memory_order_relaxed);
  }
  const auto make_arm_path = [&](const robot_motion::core::BoxPoseExtractPath& rrt_path) {
    ArmPath path;
    path.box_states = rrt_path.states;
    path.states.push_back(std::make_shared<moveit::core::RobotState>(start_state));
    path.joint_motion = rrt_path.joint_motion;
    bool valid = true;
    for (size_t index = 1; index < rrt_path.states.size(); ++index) {
      const auto found = state_cache.find(box_state_key(rrt_path.states[index]));
      if (found == state_cache.end() || !found->second) {
        valid = false;
        break;
      }
      path.states.push_back(found->second);
    }
    if (!valid || path.states.empty()) {
      path.states.clear();
    }
    return path;
  };

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
    if (diagnostic_path && !result.best_effort_path.states.empty()) {
      *diagnostic_path = make_arm_path(result.best_effort_path);
    }
  }
  for (const auto& rrt_path : result.paths) {
    ArmPath path = make_arm_path(rrt_path);
    if (!path.states.empty()) {
      paths.push_back(std::move(path));
    }
  }
  if (!policy.require_full_detachment && paths.empty() && diagnostic_path &&
      diagnostic_path->states.size() > 1 && !diagnostic_path->box_states.empty()) {
    const auto& final = diagnostic_path->box_states.back();
    const bool made_progress =
      final.retreat > 1e-6 || final.lift > 1e-6 || std::abs(final.pitch) > 1e-6 ||
      std::abs(final.lateral) > 1e-6;
    if (made_progress) {
      paths.push_back(*diagnostic_path);
      RCLCPP_INFO(
        config_.logger,
        "%s box-pose RRT accepted collision-free best-effort path: retreat=%.3f lift=%.3f pitch=%.1fdeg",
        side.c_str(), final.retreat, final.lift, final.pitch * 180.0 / M_PI);
    }
  }
  const auto arm_path_score = [&](const ArmPath& path) {
    double score = path.joint_motion;
    if (top_suction && !path.box_states.empty()) {
      const auto& final = path.box_states.back();
      score += 5.0 * final.pitch + 100.0 * final.retreat;
    }
    return score;
  };
  std::sort(paths.begin(), paths.end(), [&](const ArmPath& lhs, const ArmPath& rhs) {
    return arm_path_score(lhs) < arm_path_score(rhs);
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
  const BoxPoseRrtArmPolicy& left_policy,
  const BoxPoseRrtArmPolicy& right_policy,
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

  moveit::core::RobotState rrt_start_state(start_state);
  std::vector<moveit::core::RobotStatePtr> common_updown_prefix;
  common_updown_prefix.push_back(std::make_shared<moveit::core::RobotState>(start_state));
  double achieved_common_updown_lift = 0.0;
  std::string common_updown_stop_reason;
  BoxPoseRrtArmPolicy adjusted_left_policy = left_policy;
  BoxPoseRrtArmPolicy adjusted_right_policy = right_policy;
  if (left_top_suction && right_top_suction) {
    const auto& updown_bounds = start_state.getRobotModel()->getVariableBounds("updown");
    const double initial_updown = start_state.getVariablePosition("updown");
    if (!updown_bounds.position_bounded_ ||
        initial_updown < updown_bounds.min_position_ - 1e-9 ||
        initial_updown > updown_bounds.max_position_ + 1e-9) {
      timing.failure_reason = "top_priority_updown_start_out_of_bounds";
      timing.final_state = std::make_shared<moveit::core::RobotState>(start_state);
      timing.rollout_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      return timing;
    }
    const double available_lift = std::max(0.0, updown_bounds.max_position_ - initial_updown);
    const double requested_lift = std::max(0.0, config_.top_common_updown_lift_distance);
    const double bounded_lift = std::min(requested_lift, available_lift);
    const double lift_step = std::max(0.001, config_.top_common_updown_step);
    const size_t lift_steps = bounded_lift <= 1e-9 ? 0 :
      std::max<size_t>(1, static_cast<size_t>(std::ceil(bounded_lift / lift_step)));

    robot_motion::core::BoxPoseExtractRrtConfig left_detach_config = config_.top_rrt;
    robot_motion::core::BoxPoseExtractRrtConfig right_detach_config = config_.top_rrt;
    left_detach_config.mode = BoxMode::TopTranslate;
    right_detach_config.mode = BoxMode::TopTranslate;
    left_detach_config.source_reference_offset_z = left_policy.detachment_reference_offset_z;
    right_detach_config.source_reference_offset_z = right_policy.detachment_reference_offset_z;
    const robot_motion::core::BoxPoseExtractRrt left_detach_checker(left_detach_config);
    const robot_motion::core::BoxPoseExtractRrt right_detach_checker(right_detach_config);

    for (size_t step = 1; step <= lift_steps; ++step) {
      const double lift = bounded_lift * static_cast<double>(step) /
        static_cast<double>(lift_steps);
      moveit::core::RobotState next_state(rrt_start_state);
      next_state.setVariablePosition("updown", initial_updown + lift);
      next_state.update(true);
      bool ignored_left_detached = false;
      bool ignored_right_detached = false;
      std::string clear_reason;
      if (!next_state.satisfiesBounds(config_.joint_group) ||
          !config_.dual_clear_callback ||
          !config_.dual_clear_callback(
            next_state, left_box, left_box_id, right_box, right_box_id,
            &ignored_left_detached, &ignored_right_detached, &clear_reason)) {
        common_updown_stop_reason = clear_reason.empty() ?
          "top_priority_updown_collision" : clear_reason;
        break;
      }
      achieved_common_updown_lift = lift;
      rrt_start_state = next_state;
      common_updown_prefix.push_back(
        std::make_shared<moveit::core::RobotState>(rrt_start_state));

      const BoxState lifted_state{0.0, lift, 0.0, 0.0};
      const bool left_detached = left_detach_checker.goalReached(lifted_state);
      const bool right_detached = right_detach_checker.goalReached(lifted_state);
      if ((!left_policy.require_full_detachment || left_detached) &&
          (!right_policy.require_full_detachment || right_detached)) {
        timing.success = true;
        timing.accepted_steps = common_updown_prefix.size() - 1;
        timing.final_lift_z = lift;
        timing.right_final_lift_z = lift;
        timing.final_state = std::make_shared<moveit::core::RobotState>(rrt_start_state);
        if (record_step) {
          for (size_t prefix_step = 0; prefix_step < common_updown_prefix.size(); ++prefix_step) {
            const double prefix_lift =
              common_updown_prefix[prefix_step]->getVariablePosition("updown") - initial_updown;
            record_step(prefix_step, *common_updown_prefix[prefix_step], {
              {"stage_kind", "top_priority_common_updown_lift"},
              {"accepted", true},
              {"common_updown_lift", prefix_lift},
              {"common_updown_limit", updown_bounds.max_position_},
              {"left_detached", left_detach_checker.goalReached({0.0, prefix_lift, 0.0, 0.0})},
              {"right_detached", right_detach_checker.goalReached({0.0, prefix_lift, 0.0, 0.0})}
            });
          }
        }
        timing.rollout_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return timing;
      }
    }

    adjusted_left_policy.detachment_reference_offset_z -= achieved_common_updown_lift;
    adjusted_right_policy.detachment_reference_offset_z -= achieved_common_updown_lift;
    if (record_step) {
      for (size_t prefix_step = 0; prefix_step < common_updown_prefix.size(); ++prefix_step) {
        const double prefix_lift =
          common_updown_prefix[prefix_step]->getVariablePosition("updown") - initial_updown;
        record_step(prefix_step, *common_updown_prefix[prefix_step], {
          {"stage_kind", "top_priority_common_updown_lift"},
          {"accepted", true},
          {"common_updown_lift", prefix_lift},
          {"requested_common_updown_lift", requested_lift},
          {"bounded_common_updown_lift", bounded_lift},
          {"common_updown_limit", updown_bounds.max_position_},
          {"stop_reason", common_updown_stop_reason}
        });
      }
    }
  }

  ArmPath left_diagnostic;
  ArmPath right_diagnostic;
  const auto left_paths = planArm(
    "left", rrt_start_state, left_box, left_box_id, left_top_suction,
    adjusted_left_policy, &left_diagnostic);
  const auto right_paths = planArm(
    "right", rrt_start_state, right_box, right_box_id, right_top_suction,
    adjusted_right_policy, &right_diagnostic);
  RCLCPP_INFO(
    config_.logger,
    "box-pose RRT arm paths: candidate=%zu left=%zu right=%zu modes=(%s,%s)",
    candidate_order, left_paths.size(), right_paths.size(),
    left_top_suction ? "top" : "front", right_top_suction ? "top" : "front");
  if (left_paths.empty() || right_paths.empty()) {
    const bool left_failed = left_paths.empty();
    timing.failure_reason = left_failed ?
      "box_pose_rrt_left_no_reachable_path" : "box_pose_rrt_right_no_reachable_path";
    const auto& diagnostic = left_failed ? left_diagnostic : right_diagnostic;
    const std::string failed_side = left_failed ? "left" : "right";
    if (!diagnostic.states.empty()) {
      timing.failed_steps = diagnostic.states.size();
      for (size_t step = 0; step < diagnostic.states.size(); ++step) {
        auto combined = std::make_shared<moveit::core::RobotState>(rrt_start_state);
        if (left_failed) {
          copy_arm_state(config_.left_arm_group, *diagnostic.states[step], combined.get());
        } else {
          copy_arm_state(config_.right_arm_group, *diagnostic.states[step], combined.get());
        }
        combined->enforceBounds(config_.joint_group);
        combined->update(true);
        const bool terminal = step + 1 == diagnostic.states.size();
        if (terminal) {
          timing.final_state = combined;
          if (!diagnostic.box_states.empty()) {
            const auto& final_box_state = diagnostic.box_states.back();
            if (left_failed) {
              timing.final_retreat_x = final_box_state.retreat;
              timing.final_lift_z = final_box_state.lift;
              timing.final_pitch_deg = final_box_state.pitch * 180.0 / M_PI;
            } else {
              timing.right_final_retreat_x = final_box_state.retreat;
              timing.right_final_lift_z = final_box_state.lift;
              timing.right_final_pitch_deg = final_box_state.pitch * 180.0 / M_PI;
            }
          }
        }
        if (record_step) {
          nlohmann::json extra{
            {"stage_kind", "box_pose_rrt_no_reachable_best_effort"},
            {"accepted", false},
            {"diagnostic", true},
            {"failed_side", failed_side},
            {"failure_reason", timing.failure_reason},
            {"terminal", terminal},
            {"left_top_suction", left_top_suction},
            {"right_top_suction", right_top_suction}
          };
          if (step < diagnostic.box_states.size()) {
            const auto& box_state = diagnostic.box_states[step];
            extra.update({
              {"retreat_x", box_state.retreat},
              {"lift_z", box_state.lift},
              {"pitch_up_deg", box_state.pitch * 180.0 / M_PI},
              {"lateral_y", box_state.lateral}
            });
          }
          record_step(step, *combined, extra);
        }
      }
    }
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
          auto combined = std::make_shared<moveit::core::RobotState>(rrt_start_state);
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
  const auto pair_validation_started = std::chrono::steady_clock::now();
  for (size_t pair_rank = 0; pair_rank < pairs.size(); ++pair_rank) {
    const auto& left = left_paths[pairs[pair_rank].left];
    const auto& right = right_paths[pairs[pair_rank].right];
    const size_t step_count = std::max(left.states.size(), right.states.size());
    std::vector<moveit::core::RobotStatePtr> combined_states;
    std::vector<double> combined_progresses;
    combined_states.reserve(step_count);
    combined_progresses.reserve(step_count);
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
      auto combined = std::make_shared<moveit::core::RobotState>(rrt_start_state);
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
      combined_progresses.push_back(progress);
    }
    if (collision_free && left_top_suction && !left.box_states.empty()) {
      auto left_config = config_.top_rrt;
      left_config.mode = BoxMode::TopTranslate;
      left_config.source_reference_offset_z = adjusted_left_policy.detachment_reference_offset_z;
      final_left_detached = robot_motion::core::BoxPoseExtractRrt(left_config).goalReached(
        left.box_states.back());
    }
    if (collision_free && right_top_suction && !right.box_states.empty()) {
      auto right_config = config_.top_rrt;
      right_config.mode = BoxMode::TopTranslate;
      right_config.source_reference_offset_z = adjusted_right_policy.detachment_reference_offset_z;
      final_right_detached = robot_motion::core::BoxPoseExtractRrt(right_config).goalReached(
        right.box_states.back());
    }
    if (!collision_free) {
      failure_counts[failure_reason]++;
      continue;
    }
    if ((left_policy.require_full_detachment && !final_left_detached) ||
        (right_policy.require_full_detachment && !final_right_detached)) {
      failure_counts["box_pose_rrt_final_not_detached"]++;
      continue;
    }

    const auto smoothed = smooth_combined_extract_path(
      config_, combined_states, combined_progresses, left_box, left_box_id, right_box, right_box_id);
    if (!smoothed.states.empty()) {
      combined_states = smoothed.states;
      combined_progresses = smoothed.progresses;
      if (smoothed.removed_reference_points > 0) {
        RCLCPP_INFO(
          config_.logger,
          "box-pose RRT shortcut smoothing: pair_rank=%zu points %zu -> %zu removed=%zu",
          pair_rank + 1,
          step_count,
          combined_states.size(),
          smoothed.removed_reference_points);
      }
    }

    timing.success = true;
    timing.accepted_steps =
      combined_states.size() + (common_updown_prefix.empty() ? 0 : common_updown_prefix.size() - 1);
    timing.final_state = combined_states.back();
    timing.final_retreat_x = left.box_states.back().retreat;
    timing.final_lift_z = achieved_common_updown_lift + left.box_states.back().lift;
    timing.final_pitch_deg = left.box_states.back().pitch * 180.0 / M_PI;
    timing.right_final_retreat_x = right.box_states.back().retreat;
    timing.right_final_lift_z = achieved_common_updown_lift + right.box_states.back().lift;
    timing.right_final_pitch_deg = right.box_states.back().pitch * 180.0 / M_PI;
    if (record_step) {
      for (size_t step = 0; step < combined_states.size(); ++step) {
        const double progress = step < combined_progresses.size() ? combined_progresses[step] :
          (combined_states.size() <= 1 ? 1.0 :
            static_cast<double>(step) / static_cast<double>(combined_states.size() - 1));
        const size_t left_index = std::min(
          left.box_states.size() - 1,
          static_cast<size_t>(std::llround(progress * static_cast<double>(left.box_states.size() - 1))));
        const size_t right_index = std::min(
          right.box_states.size() - 1,
          static_cast<size_t>(std::llround(progress * static_cast<double>(right.box_states.size() - 1))));
        record_step(step, *combined_states[step], {
          {"stage_kind", "box_pose_rrt_extract_step"},
          {"path_pair_rank", pair_rank + 1},
          {"path_pair_joint_motion", pairs[pair_rank].cost},
          {"accepted", true},
          {"left_retreat_x", left.box_states[left_index].retreat},
          {"left_lift_z", left.box_states[left_index].lift},
          {"left_pitch_up_deg", left.box_states[left_index].pitch * 180.0 / M_PI},
          {"left_lateral_y", left.box_states[left_index].lateral},
          {"right_retreat_x", right.box_states[right_index].retreat},
          {"right_lift_z", right.box_states[right_index].lift},
          {"right_pitch_up_deg", right.box_states[right_index].pitch * 180.0 / M_PI},
          {"right_lateral_y", right.box_states[right_index].lateral},
          {"extract_smoothing_applied", true},
          {"left_top_suction", left_top_suction},
          {"right_top_suction", right_top_suction},
          {"left_detachment_required", left_policy.require_full_detachment},
          {"right_detachment_required", right_policy.require_full_detachment},
          {"left_front_clearance_levels", left_policy.front_clearance_levels},
          {"right_front_clearance_levels", right_policy.front_clearance_levels}
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
  if (config_.profile) {
    config_.profile->pair_validation_ns.fetch_add(
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - pair_validation_started).count()),
      std::memory_order_relaxed);
  }
  timing.rollout_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return timing;
}

}  // namespace alfa_robot::motion
