#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace alfa_robot::motion
{

ExtractMotionPlanner::ExtractMotionPlanner(ExtractMotionPlannerConfig config)
: config_(std::move(config))
{}

std::vector<ExtractMotionDelta> ExtractMotionPlanner::motionDeltas() const
{
  return {
    {1.0, 0.0, 0.0},
    {0.8, 0.6, 0.0},
    {0.6, 0.8, 0.0},
    {0.0, 1.0, 0.0},
  };
}

std::vector<double> ExtractMotionPlanner::pitchDeltaDegrees(double current_pitch_rad) const
{
  const double current_pitch_deg = current_pitch_rad * 180.0 / M_PI;
  std::vector<double> deltas{0.0, 1.0};
  if (current_pitch_deg >= 1.0) {
    deltas.push_back(-1.0);
  }
  deltas.push_back(3.0);
  if (current_pitch_deg >= 3.0) {
    deltas.push_back(-3.0);
  }
  return deltas;
}

std::vector<ExtractMotionLayer> ExtractMotionPlanner::layers(
  const BoxSpec& source_box,
  int box_id,
  double current_pitch_rad,
  double last_retreat_x,
  double current_lift_z) const
{
  std::vector<ExtractMotionLayer> result;
  size_t candidate_index = 0;
  for (double pitch_delta_deg : pitchDeltaDegrees(current_pitch_rad)) {
    ExtractMotionLayer layer;
    layer.pitch_delta_deg = pitch_delta_deg;
    const double pitch_delta = pitch_delta_deg * M_PI / 180.0;
    const double pitch_rad = std::max(0.0, current_pitch_rad + pitch_delta);
    for (const auto& delta : motionDeltas()) {
      const double retreat_delta = std::max(0.0, delta.retreat_ratio * config_.step_x);
      const double retreat_x = std::min(config_.max_x, last_retreat_x + retreat_delta);
      if (retreat_x <= last_retreat_x + 1e-6 && last_retreat_x >= config_.max_x - 1e-6) {
        continue;
      }

      const double lift_delta = std::max(0.0, delta.lift_ratio * config_.step_x);
      const double lift_z = current_lift_z + lift_delta;
      layer.commands.push_back({
        candidate_index,
        retreat_x,
        retreat_x - last_retreat_x,
        lift_z,
        lift_delta,
        pitch_rad,
        pitch_delta,
        BoxSpec{box_id, source_box.x - retreat_x, source_box.y, source_box.z + lift_z},
      });
      ++candidate_index;
    }
    result.push_back(std::move(layer));
  }
  return result;
}

size_t ExtractMotionPlanner::maxStepCount() const
{
  return static_cast<size_t>(
    std::ceil(std::max(0.0, config_.max_x) / std::max(1e-6, 0.6 * config_.step_x))) + 2;
}

}  // namespace alfa_robot::motion

#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/joint_model_group.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>

namespace alfa_robot::motion
{

struct ExtractCandidateSolver::ArmKdlChain
{
  std::string side;
  std::string base_link;
  std::string tip_link;
  KDL::Chain chain;
  std::vector<std::string> joint_names;
  KDL::JntArray lower;
  KDL::JntArray upper;
  bool valid = false;
};

namespace
{

std::string moveit_variable_name_for_kdl_joint(const std::string& name)
{
  if (name.rfind("left_joint", 0) == 0) {
    return "left_v5_joint" + name.substr(std::string("left_joint").size());
  }
  if (name.rfind("right_joint", 0) == 0) {
    return "right_v5_joint" + name.substr(std::string("right_joint").size());
  }
  return name;
}

KDL::Frame eigen_to_kdl_frame(const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix3d rotation = transform.linear();
  return KDL::Frame(
    KDL::Rotation(
      rotation(0, 0), rotation(0, 1), rotation(0, 2),
      rotation(1, 0), rotation(1, 1), rotation(1, 2),
      rotation(2, 0), rotation(2, 1), rotation(2, 2)),
    KDL::Vector(
      transform.translation().x(),
      transform.translation().y(),
      transform.translation().z()));
}

Eigen::Isometry3d kdl_frame_to_eigen(const KDL::Frame& frame)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      transform.linear()(row, col) = frame.M(row, col);
    }
  }
  transform.translation() = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  return transform;
}

double pose_tool_axis_error(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual)
{
  const Eigen::Vector3d target_axis = target.linear() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d actual_axis = actual.linear() * Eigen::Vector3d::UnitZ();
  const double dot = std::clamp(target_axis.normalized().dot(actual_axis.normalized()), -1.0, 1.0);
  return std::acos(dot);
}

}  // namespace

ExtractCandidateSolver::ExtractCandidateSolver(ExtractCandidateSolverConfig config)
: config_(std::move(config))
{}

ExtractCandidateSolver::~ExtractCandidateSolver() = default;

bool ExtractCandidateSolver::initialize(std::string* error)
{
  if (!config_.use_independent_kdl) return true;

  left_kdl_chain_ = std::make_unique<ArmKdlChain>();
  right_kdl_chain_ = std::make_unique<ArmKdlChain>();
  if (!initArmKdlChain("left", left_kdl_chain_.get(), error)) return false;
  if (!initArmKdlChain("right", right_kdl_chain_.get(), error)) return false;
  return true;
}

const moveit::core::JointModelGroup* ExtractCandidateSolver::groupForSide(const std::string& side) const
{
  return side == "left" ? config_.left_arm_group : config_.right_arm_group;
}

const std::string& ExtractCandidateSolver::tipForSide(const std::string& side) const
{
  return side == "left" ? config_.left_tip : config_.right_tip;
}

double ExtractCandidateSolver::armJointDelta(
  const std::string& side,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to) const
{
  const auto* group = groupForSide(side);
  if (!group) return 0.0;

  double sum = 0.0;
  for (const auto& name : group->getVariableNames()) {
    const double delta = to.getVariablePosition(name) - from.getVariablePosition(name);
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

bool ExtractCandidateSolver::initArmKdlChain(const std::string& side, ArmKdlChain* out, std::string* error) const
{
  if (!out || !config_.robot_model) return false;
  const auto* group = groupForSide(side);
  if (!group) return false;

  const std::string base_link = side == "left" ? "left_arm_base" : "right_arm_base";
  const std::string tip_link = tipForSide(side);

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*config_.robot_model->getURDF(), tree)) {
    if (error) *error = "Failed to build KDL tree from URDF";
    return false;
  }

  KDL::Chain chain;
  if (!tree.getChain(base_link, tip_link, chain)) {
    if (error) *error = "Failed to get KDL chain " + base_link + " -> " + tip_link;
    return false;
  }

  out->side = side;
  out->base_link = base_link;
  out->tip_link = tip_link;
  out->chain = chain;
  out->joint_names.clear();
  out->joint_names.reserve(chain.getNrOfJoints());

  for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments(); ++segment_index) {
    const auto& segment = chain.getSegment(segment_index);
    const auto& joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None) {
      out->joint_names.push_back(moveit_variable_name_for_kdl_joint(joint.getName()));
    }
  }

  if (out->joint_names.size() != chain.getNrOfJoints()) {
    if (error) *error = "KDL chain joint name count mismatch for " + side;
    return false;
  }

  out->lower.resize(chain.getNrOfJoints());
  out->upper.resize(chain.getNrOfJoints());
  for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
    const auto& name = out->joint_names[i];
    const auto& bounds = config_.robot_model->getVariableBounds(name);
    out->lower(i) = bounds.position_bounded_ ? bounds.min_position_ : -M_PI;
    out->upper(i) = bounds.position_bounded_ ? bounds.max_position_ : M_PI;
  }

  out->valid = true;
  return true;
}

bool ExtractCandidateSolver::solveIndependentKdl(
  const std::string& side,
  const ArmKdlChain& chain,
  const moveit::core::RobotState& current_state,
  const Eigen::Isometry3d& target_world,
  double fixed_updown,
  moveit::core::RobotState& state) const
{
  if (!chain.valid || chain.joint_names.empty()) return false;

  state = current_state;
  state.setVariablePosition("updown", fixed_updown);
  state.update();

  const Eigen::Isometry3d& base_world = state.getGlobalLinkTransform(chain.base_link);
  const Eigen::Isometry3d target_in_base = base_world.inverse() * target_world;

  KDL::JntArray seed(chain.chain.getNrOfJoints());
  for (unsigned int i = 0; i < chain.chain.getNrOfJoints(); ++i) {
    seed(i) = current_state.getVariablePosition(chain.joint_names[i]);
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain.chain);
  const int max_iterations = std::max(1, config_.independent_kdl_max_iterations);
  const double eps = std::max(1e-9, config_.independent_kdl_eps);
  const KDL::Frame target_frame = eigen_to_kdl_frame(target_in_base);

  KDL::JntArray best_solution(chain.chain.getNrOfJoints());
  double best_error = std::numeric_limits<double>::infinity();
  bool found = false;

  const uint32_t seed_base =
    static_cast<uint32_t>(std::hash<std::string>{}(side) ^
                          static_cast<size_t>(std::llround(target_world.translation().x() * 1000000.0)) ^
                          (static_cast<size_t>(std::llround(target_world.translation().y() * 1000000.0)) << 1) ^
                          (static_cast<size_t>(std::llround(target_world.translation().z() * 1000000.0)) << 2));
  std::mt19937 rng(seed_base);
  std::uniform_real_distribution<double> jitter_dist(
    -std::abs(config_.independent_kdl_seed_jitter),
    std::abs(config_.independent_kdl_seed_jitter));

  for (int attempt = 0; attempt < std::max(1, config_.independent_kdl_seed_attempts); ++attempt) {
    KDL::JntArray attempt_seed = seed;
    if (attempt > 0) {
      for (unsigned int i = 0; i < chain.chain.getNrOfJoints(); ++i) {
        double value = seed(i) + jitter_dist(rng);
        value = std::max(chain.lower(i), std::min(chain.upper(i), value));
        attempt_seed(i) = value;
      }
    }

    KDL::JntArray solution(chain.chain.getNrOfJoints());
    KDL::ChainIkSolverVel_pinv vel_solver(chain.chain);
    KDL::ChainIkSolverPos_NR_JL ik_solver(
      chain.chain, chain.lower, chain.upper, fk_solver, vel_solver, max_iterations, eps);

    const int rc = ik_solver.CartToJnt(attempt_seed, target_frame, solution);
    if (rc < 0) continue;

    KDL::Frame achieved;
    if (fk_solver.JntToCart(solution, achieved) < 0) continue;
    const Eigen::Isometry3d achieved_eigen = kdl_frame_to_eigen(achieved);
    const double position_error = pose_position_error(target_in_base, achieved_eigen);
    const double orientation_error = config_.top_suction ?
      pose_tool_axis_error(target_in_base, achieved_eigen) :
      pose_orientation_error(target_in_base, achieved_eigen);
    const double orientation_tolerance = config_.top_suction ?
      config_.top_suction_orientation_tolerance :
      config_.orientation_tolerance;
    const double error = pose_position_error(target_in_base, achieved_eigen) +
                         orientation_error;
    if (error < best_error) {
      best_error = error;
      best_solution = solution;
      found = true;
    }
    if (position_error <= config_.position_tolerance &&
        orientation_error <= orientation_tolerance) {
      break;
    }
  }

  if (!found) return false;

  for (unsigned int i = 0; i < chain.chain.getNrOfJoints(); ++i) {
    state.setVariablePosition(chain.joint_names[i], best_solution(i));
  }
  state.setVariablePosition("updown", fixed_updown);
  state.enforceBounds(config_.joint_group);
  state.update();
  return true;
}

bool ExtractCandidateSolver::solve(const ExtractCandidateSolveRequest& request, ExtractCandidate* out) const
{
  if (!out || !request.current_state) return false;
  const auto* arm_group = groupForSide(request.side);
  const std::string& tip = tipForSide(request.side);
  if (!arm_group) return false;

  out->step_index = request.step_index;
  out->candidate_index = request.candidate_index;
  out->retreat_x = request.retreat_x;
  out->retreat_delta_x = request.retreat_delta_x;
  out->lift_z = request.lift_z;
  out->lift_delta_z = request.lift_delta_z;
  out->pitch_up_rad = request.pitch_up_rad;
  out->pitch_delta_rad = request.pitch_delta_rad;
  out->target_pose = request.target_pose;

  const Eigen::Isometry3d target = pose_to_eigen(request.target_pose);
  auto state = std::make_shared<moveit::core::RobotState>(*request.current_state);
  bool ik_ok = false;
  const ArmKdlChain* chain = request.side == "left" ? left_kdl_chain_.get() : right_kdl_chain_.get();
  if (config_.use_independent_kdl && chain && chain->valid) {
    ik_ok = solveIndependentKdl(request.side, *chain, *request.current_state, target, request.fixed_updown, *state);
  } else {
    state->setVariablePosition("updown", request.fixed_updown);
    state->update();
    std::lock_guard<std::mutex> lock(moveit_kdl_mutex_);
    ik_ok = state->setFromIK(arm_group, target, tip, config_.kdl_timeout);
  }
  if (!ik_ok) {
    const Eigen::Isometry3d& current_tip = request.current_state->getGlobalLinkTransform(tip);
    std::ostringstream oss;
    oss << request.side << "_kdl_no_solution"
        << " current=(" << current_tip.translation().x()
        << "," << current_tip.translation().y()
        << "," << current_tip.translation().z() << ")"
        << " target=(" << target.translation().x()
        << "," << target.translation().y()
        << "," << target.translation().z() << ")";
    out->rejection_reason = oss.str();
    return false;
  }

  state->setVariablePosition("updown", request.fixed_updown);
  state->enforceBounds(config_.joint_group);
  state->update();

  const Eigen::Isometry3d& actual = state->getGlobalLinkTransform(tip);
  const double pos_error = pose_position_error(target, actual);
  const double ori_error = config_.top_suction ?
    pose_tool_axis_error(target, actual) :
    pose_orientation_error(target, actual);
  const double ori_tolerance = config_.top_suction ?
    config_.top_suction_orientation_tolerance :
    config_.orientation_tolerance;
  if (pos_error > config_.position_tolerance || ori_error > ori_tolerance) {
    std::ostringstream oss;
    oss << request.side << "_kdl_tip_error pos=" << pos_error << " ori=" << ori_error;
    out->rejection_reason = oss.str();
    return false;
  }

  const Eigen::Vector3d tool_normal = actual.linear() * Eigen::Vector3d::UnitZ();
  const double min_tool_normal_z = std::isfinite(request.min_tool_normal_z)
    ? request.min_tool_normal_z
    : config_.min_tool_normal_z;
  if (config_.enforce_tool_normal_not_down && tool_normal.z() < min_tool_normal_z) {
    std::ostringstream oss;
    oss << request.side << "_tool_normal_down z=" << tool_normal.z();
    out->rejection_reason = oss.str();
    return false;
  }

  if (actual.translation().z() + config_.max_tip_z_drop < request.min_allowed_tip_z) {
    std::ostringstream oss;
    oss << request.side << "_tip_z_dropped z=" << actual.translation().z()
        << " min=" << request.min_allowed_tip_z;
    out->rejection_reason = oss.str();
    return false;
  }

  if (config_.max_joint_delta > 0.0) {
    const double joint_delta = armJointDelta(request.side, *request.current_state, *state);
    if (joint_delta > config_.max_joint_delta) {
      std::ostringstream oss;
      oss << request.side << "_joint_delta_too_large delta=" << joint_delta
          << " limit=" << config_.max_joint_delta;
      out->rejection_reason = oss.str();
      return false;
    }
  }

  out->ik_success = true;
  out->state = state;
  return true;
}

}  // namespace alfa_robot::motion


#include <moveit/robot_model/joint_model_group.h>

#include <cmath>
#include <limits>

namespace alfa_robot::motion
{

ExtractCandidateScorer::ExtractCandidateScorer(ExtractCandidateScorerConfig config)
: config_(std::move(config))
{}

const moveit::core::JointModelGroup* ExtractCandidateScorer::groupForSide(const std::string& side) const
{
  return side == "left" ? config_.left_arm_group : config_.right_arm_group;
}

const std::string& ExtractCandidateScorer::tipForSide(const std::string& side) const
{
  return side == "left" ? config_.left_tip : config_.right_tip;
}

double ExtractCandidateScorer::armJointDelta(
  const std::string& side,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to) const
{
  const auto* group = groupForSide(side);
  if (!group) return 0.0;

  double sum = 0.0;
  const auto& names = group->getVariableNames();
  for (const auto& name : names) {
    const double delta = to.getVariablePosition(name) - from.getVariablePosition(name);
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

double ExtractCandidateScorer::tipPositionDelta(
  const std::string& side,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to) const
{
  const auto& tip = tipForSide(side);
  const auto& from_tf = from.getGlobalLinkTransform(tip);
  const auto& to_tf = to.getGlobalLinkTransform(tip);
  return (to_tf.translation() - from_tf.translation()).norm();
}

double ExtractCandidateScorer::tipOrientationDelta(
  const std::string& side,
  const moveit::core::RobotState& from,
  const moveit::core::RobotState& to) const
{
  const auto& tip = tipForSide(side);
  const auto& from_tf = from.getGlobalLinkTransform(tip);
  const auto& to_tf = to.getGlobalLinkTransform(tip);
  return pose_orientation_error(from_tf, to_tf);
}

double ExtractCandidateScorer::score(
  const std::string& side,
  const ExtractCandidate& candidate,
  const moveit::core::RobotState& current_state,
  double last_retreat_x) const
{
  if (!candidate.state_valid || !candidate.state) {
    return std::numeric_limits<double>::infinity();
  }

  const double retreat_continuity =
    std::abs((candidate.retreat_x - last_retreat_x) - config_.step_x);
  const double joint_delta = armJointDelta(side, current_state, *candidate.state);
  const double tip_position_delta = tipPositionDelta(side, current_state, *candidate.state);
  const double tip_orientation_delta = tipOrientationDelta(side, current_state, *candidate.state);

  return config_.lift_weight * candidate.lift_z +
         config_.pitch_weight * std::abs(candidate.pitch_up_rad) +
         config_.retreat_continuity_weight * retreat_continuity +
         config_.joint_delta_weight * joint_delta +
         config_.tip_position_delta_weight * tip_position_delta +
         config_.tip_orientation_delta_weight * tip_orientation_delta;
}

}  // namespace alfa_robot::motion


#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

namespace alfa_robot::motion
{

namespace
{

geometry_msgs::msg::Pose link_pose(
  const moveit::core::RobotState& state,
  const std::string& link_name,
  const Eigen::Vector3d& translation_delta = Eigen::Vector3d::Zero())
{
  const Eigen::Isometry3d& tf = state.getGlobalLinkTransform(link_name);
  Eigen::Quaterniond q(tf.linear());
  q.normalize();
  return make_pose(
    tf.translation().x() + translation_delta.x(),
    tf.translation().y() + translation_delta.y(),
    tf.translation().z() + translation_delta.z(),
    q);
}

}  // namespace

ExtractRolloutPlanner::ExtractRolloutPlanner(ExtractRolloutPlannerConfig config)
: config_(std::move(config))
{}

double ExtractRolloutPlanner::currentUpdown(const moveit::core::RobotState& state) const
{
  const auto& variable_names = state.getRobotModel()->getVariableNames();
  if (std::find(variable_names.begin(), variable_names.end(), "updown") == variable_names.end()) {
    return 0.0;
  }
  return state.getVariablePosition("updown");
}

const std::string& ExtractRolloutPlanner::tipForSide(const std::string& side) const
{
  return side == "left" ? config_.left_tip : config_.right_tip;
}

const moveit::core::JointModelGroup* ExtractRolloutPlanner::groupForSide(const std::string& side) const
{
  return side == "left" ? config_.left_arm_group : config_.right_arm_group;
}

double ExtractRolloutPlanner::currentPitchUpRad(
  const std::string& side,
  const moveit::core::RobotState& state) const
{
  const Eigen::Vector3d tool_normal =
    state.getGlobalLinkTransform(tipForSide(side)).linear() * Eigen::Vector3d::UnitZ();
  const double x = std::max(0.0, tool_normal.x());
  const double z = tool_normal.z();
  return std::max(0.0, std::atan2(z, x));
}

bool ExtractRolloutPlanner::solveCandidate(
  const std::string& side,
  const moveit::core::RobotState& current_state,
  const geometry_msgs::msg::Pose& target_pose,
  size_t step_index,
  size_t candidate_index,
  double retreat_x,
  double retreat_delta_x,
  double lift_z,
  double lift_delta_z,
  double pitch_up_rad,
  double pitch_delta_rad,
  double min_allowed_tip_z,
  const AttachedBoxSpec& carried_box,
  int box_id,
  ExtractCandidate* out) const
{
  if (!config_.candidate_solver) return false;
  const bool solved = config_.candidate_solver->solve(
    {
      side,
      &current_state,
      target_pose,
      step_index,
      candidate_index,
      retreat_x,
      retreat_delta_x,
      lift_z,
      lift_delta_z,
      pitch_up_rad,
      pitch_delta_rad,
      min_allowed_tip_z,
      currentUpdown(current_state),
    },
    out);
  if (!solved || !out || !out->state) return false;

  bool detached = false;
  std::string reason;
  if (!config_.single_clear_callback ||
      !config_.single_clear_callback(*out->state, carried_box, box_id, &detached, &reason)) {
    out->rejection_reason = reason.empty() ? "single_clear_callback_failed" : reason;
    return false;
  }

  if (config_.trajectory_clear_callback) {
    moveit::core::RobotState planning_start(current_state);
    moveit::core::RobotState planning_goal(*out->state);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto& trajectory = plan.trajectory_.joint_trajectory;
    if (config_.joint_group) {
      trajectory.joint_names = config_.joint_group->getVariableNames();
    }
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    trajectory_msgs::msg::JointTrajectoryPoint goal_point;
    start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    goal_point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    start_point.positions.reserve(trajectory.joint_names.size());
    goal_point.positions.reserve(trajectory.joint_names.size());
    for (const auto& name : trajectory.joint_names) {
      start_point.positions.push_back(planning_start.getVariablePosition(name));
      goal_point.positions.push_back(planning_goal.getVariablePosition(name));
    }
    trajectory.points.push_back(start_point);
    trajectory.points.push_back(goal_point);
    if (!config_.trajectory_clear_callback(plan, planning_start, {carried_box}, &reason)) {
      out->rejection_reason = "extract_step_collision: " +
        (reason.empty() ? std::string("trajectory_clear_callback_failed") : reason);
      return false;
    }
  }

  out->state_valid = true;
  out->carried_clear = true;
  out->detached_from_neighbors = detached;
  return true;
}

std::vector<ExtractCandidate> ExtractRolloutPlanner::makeCandidatesForSide(
  const std::string& side,
  const moveit::core::RobotState& current_state,
  const BoxSpec& source_box,
  const AttachedBoxSpec& carried_box,
  int box_id,
  size_t step,
  double last_retreat_x,
  double current_lift_z,
  double min_allowed_tip_z) const
{
  std::vector<ExtractCandidate> candidates;
  if (!config_.motion_planner) return candidates;
  const double current_pitch = currentPitchUpRad(side, current_state);

  if (config_.top_suction) {
    const double lift_delta = std::max(1e-6, config_.motion_planner->config().step_x);
    const double lift_z = current_lift_z + lift_delta;
    const auto target_pose = link_pose(
      current_state,
      tipForSide(side),
      Eigen::Vector3d(0.0, 0.0, lift_delta));

    ExtractCandidate candidate;
    solveCandidate(side, current_state, target_pose, step, 0,
                   last_retreat_x, 0.0,
                   lift_z, lift_delta,
                   current_pitch, 0.0,
                   min_allowed_tip_z, carried_box, box_id, &candidate);
    candidates.push_back(candidate);
    return candidates;
  }

  for (const auto& layer : config_.motion_planner->layers(
         source_box, box_id, current_pitch, last_retreat_x, current_lift_z)) {
    std::vector<ExtractCandidate> layer_candidates;
    for (const auto& command : layer.commands) {
      Eigen::Quaterniond target_orientation;
      if (config_.top_suction) {
        target_orientation = Eigen::Quaterniond(current_state.getGlobalLinkTransform(tipForSide(side)).linear());
        target_orientation.normalize();
      } else {
        target_orientation = pitch_up_orientation(command.pitch_up_rad);
      }
      const auto target_pose = make_pose(
        command.shifted_box.x, command.shifted_box.y, command.shifted_box.z,
        target_orientation);

      ExtractCandidate candidate;
      solveCandidate(side, current_state, target_pose, step, command.candidate_index,
                     command.retreat_x, command.retreat_delta_x,
                     command.lift_z, command.lift_delta_z,
                     command.pitch_up_rad, command.pitch_delta_rad,
                     min_allowed_tip_z, carried_box, box_id, &candidate);
      layer_candidates.push_back(candidate);
    }

    const bool layer_has_valid = std::any_of(
      layer_candidates.begin(), layer_candidates.end(),
      [](const ExtractCandidate& candidate) { return candidate.state_valid; });
    candidates.insert(candidates.end(), layer_candidates.begin(), layer_candidates.end());
    if (layer_has_valid) {
      break;
    }
  }

  return candidates;
}

std::vector<ExtractCandidate> ExtractRolloutPlanner::topValidCandidates(
  const std::string& side,
  const std::vector<ExtractCandidate>& candidates,
  const moveit::core::RobotState& current_state,
  double last_retreat_x) const
{
  std::vector<ExtractCandidate> valid;
  for (const auto& candidate : candidates) {
    if (candidate.state_valid && candidate.state) {
      valid.push_back(candidate);
    }
  }
  std::sort(valid.begin(), valid.end(),
            [&](const ExtractCandidate& a, const ExtractCandidate& b) {
              return config_.candidate_scorer->score(side, a, current_state, last_retreat_x) <
                     config_.candidate_scorer->score(side, b, current_state, last_retreat_x);
            });
  if (valid.size() > config_.top_valid_limit) {
    valid.resize(config_.top_valid_limit);
  }
  return valid;
}

void ExtractRolloutPlanner::copyArmState(
  const std::string& side,
  const moveit::core::RobotState& from,
  moveit::core::RobotState& to) const
{
  const auto* group = groupForSide(side);
  if (!group) return;
  for (const auto& name : group->getVariableNames()) {
    if (std::find(to.getRobotModel()->getVariableNames().begin(), to.getRobotModel()->getVariableNames().end(), name) !=
        to.getRobotModel()->getVariableNames().end()) {
      to.setVariablePosition(name, from.getVariablePosition(name));
    }
  }
}

ArmExtractPath ExtractRolloutPlanner::rolloutArm(
  const std::string& side,
  const moveit::core::RobotState& start_state,
  const BoxSpec& source_box,
  const AttachedBoxSpec& carried_box,
  int box_id) const
{
  ArmExtractPath path;
  moveit::core::RobotState current_state(start_state);
  double last_retreat_x = 0.0;
  double current_lift_z = 0.0;
  double min_allowed_tip_z = current_state.getGlobalLinkTransform(tipForSide(side)).translation().z();
  const size_t max_steps = config_.motion_planner ? config_.motion_planner->maxStepCount() : 0;

  path.states.push_back(std::make_shared<moveit::core::RobotState>(current_state));
  bool detached_seen = false;
  size_t extra_steps_after_detached = 0;
  for (size_t step = 1; step <= max_steps; ++step) {
    const auto candidates =
      makeCandidatesForSide(side, current_state, source_box, carried_box, box_id,
                            step, last_retreat_x, current_lift_z, min_allowed_tip_z);
    auto best_it = std::min_element(
      candidates.begin(), candidates.end(),
      [&](const ExtractCandidate& a, const ExtractCandidate& b) {
        return config_.candidate_scorer->score(side, a, current_state, last_retreat_x) <
               config_.candidate_scorer->score(side, b, current_state, last_retreat_x);
      });
    if (best_it == candidates.end() || !best_it->state_valid || !best_it->state) {
      ++path.failed_steps;
      std::map<std::string, size_t> rejection_counts;
      for (const auto& candidate : candidates) {
        const std::string key = candidate.rejection_reason.empty() ? "unknown" : candidate.rejection_reason;
        rejection_counts[key]++;
      }
      if (!rejection_counts.empty()) {
        size_t best_count = 0;
        for (const auto& [reason, count] : rejection_counts) {
          if (count > best_count) {
            best_count = count;
            path.failure_reason = side + ":" + reason;
          }
        }
      } else {
        path.failure_reason = side + ":no_valid_candidate";
      }
      if (detached_seen && path.success) {
        path.failure_reason.clear();
      }
      break;
    }

    current_state = *best_it->state;
    min_allowed_tip_z =
      std::max(min_allowed_tip_z, current_state.getGlobalLinkTransform(tipForSide(side)).translation().z());
    last_retreat_x = best_it->retreat_x;
    current_lift_z = best_it->lift_z;
    path.final_retreat_x = best_it->retreat_x;
    path.final_lift_z = best_it->lift_z;
    path.final_pitch_deg = best_it->pitch_up_rad * 180.0 / M_PI;
    path.selected_candidates.push_back(*best_it);
    path.states.push_back(std::make_shared<moveit::core::RobotState>(current_state));
    ++path.accepted_steps;

    if (best_it->detached_from_neighbors) {
      detached_seen = true;
      path.success = true;
      path.failure_reason.clear();
    }
    if (detached_seen) {
      ++extra_steps_after_detached;
    }
    if (detached_seen && extra_steps_after_detached > config_.success_extra_steps) {
      path.success = true;
      path.failure_reason.clear();
      break;
    }
  }

  if (!path.success && path.failure_reason.empty()) {
    path.failure_reason = side + ":reached_max_retreat_without_neighbor_detachment";
  }
  return path;
}

moveit::core::RobotState ExtractRolloutPlanner::combineAsyncArmStates(
  const moveit::core::RobotState& base_state,
  const ArmExtractPath& left_path,
  const ArmExtractPath& right_path,
  size_t left_index,
  size_t right_index) const
{
  moveit::core::RobotState state(base_state);
  const auto& left_state = *left_path.states[std::min(left_index, left_path.states.size() - 1)];
  const auto& right_state = *right_path.states[std::min(right_index, right_path.states.size() - 1)];
  copyArmState("left", left_state, state);
  copyArmState("right", right_state, state);
  state.setVariablePosition("updown", currentUpdown(base_state));
  state.enforceBounds(config_.joint_group);
  state.update();
  return state;
}

bool ExtractRolloutPlanner::validateAsyncPath(
  const moveit::core::RobotState& start_state,
  const ArmExtractPath& left_path,
  const ArmExtractPath& right_path,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  std::vector<moveit::core::RobotStatePtr>* combined_states,
  std::string* reason) const
{
  if (!left_path.success) {
    if (reason) *reason = left_path.failure_reason.empty() ? "left_async_extract_failed" : left_path.failure_reason;
    return false;
  }
  if (!right_path.success) {
    if (reason) *reason = right_path.failure_reason.empty() ? "right_async_extract_failed" : right_path.failure_reason;
    return false;
  }
  if (left_path.states.empty() || right_path.states.empty()) {
    if (reason) *reason = "empty_async_extract_path";
    return false;
  }

  const size_t step_count = std::max(left_path.states.size(), right_path.states.size());
  if (combined_states) {
    combined_states->clear();
    combined_states->reserve(step_count);
  }
  for (size_t step = 0; step < step_count; ++step) {
    auto state = std::make_shared<moveit::core::RobotState>(
      combineAsyncArmStates(start_state, left_path, right_path, step, step));
    bool left_detached = false;
    bool right_detached = false;
    std::string state_reason;
    if (!config_.dual_clear_callback ||
        !config_.dual_clear_callback(*state, left_box, left_box_id, right_box, right_box_id,
                                     &left_detached, &right_detached, &state_reason)) {
      if (reason) *reason = "async_combined_step_" + std::to_string(step) + ":" + state_reason;
      return false;
    }
    if (combined_states) {
      combined_states->push_back(state);
    }
  }
  return true;
}

DualExtractStepCandidate ExtractRolloutPlanner::selectDualStepCandidate(
  const moveit::core::RobotState& current_state,
  const std::vector<ExtractCandidate>& left_candidates,
  const std::vector<ExtractCandidate>& right_candidates,
  double left_last_retreat_x,
  double right_last_retreat_x,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id) const
{
  DualExtractStepCandidate best;
  const auto left_valid = topValidCandidates("left", left_candidates, current_state, left_last_retreat_x);
  const auto right_valid = topValidCandidates("right", right_candidates, current_state, right_last_retreat_x);
  if (left_valid.empty() || right_valid.empty()) {
    best.rejection_reason = left_valid.empty() ? "no_valid_left_extract_candidate" : "no_valid_right_extract_candidate";
    return best;
  }

  for (const auto& left_candidate : left_valid) {
    for (const auto& right_candidate : right_valid) {
      auto state = std::make_shared<moveit::core::RobotState>(current_state);
      copyArmState("left", *left_candidate.state, *state);
      copyArmState("right", *right_candidate.state, *state);
      state->setVariablePosition("updown", currentUpdown(current_state));
      state->enforceBounds(config_.joint_group);
      state->update();

      bool left_detached = false;
      bool right_detached = false;
      std::string reason;
      if (!config_.dual_clear_callback ||
          !config_.dual_clear_callback(*state, left_box, left_box_id, right_box, right_box_id,
                                       &left_detached, &right_detached, &reason)) {
        if (best.rejection_reason.empty()) {
          best.rejection_reason = reason.empty() ? "dual_clear_callback_failed" : reason;
        }
        continue;
      }

      const double score =
        config_.candidate_scorer->score("left", left_candidate, current_state, left_last_retreat_x) +
        config_.candidate_scorer->score("right", right_candidate, current_state, right_last_retreat_x) +
        0.2 * std::abs(left_candidate.retreat_x - right_candidate.retreat_x) +
        0.2 * std::abs(left_candidate.lift_z - right_candidate.lift_z);
      if (!best.state_valid || score < best.score) {
        best.left = left_candidate;
        best.right = right_candidate;
        best.state = state;
        best.state_valid = true;
        best.left_detached = left_detached;
        best.right_detached = right_detached;
        best.score = score;
        best.rejection_reason.clear();
      }
    }
  }
  if (!best.state_valid && best.rejection_reason.empty()) {
    best.rejection_reason = "no_collision_free_dual_extract_candidate";
  }
  return best;
}

ExtractRolloutTiming ExtractRolloutPlanner::rolloutLeft(
  const moveit::core::RobotState& start_state,
  const BoxSpec& source_box,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  size_t candidate_order,
  size_t h_index,
  size_t seed_index,
  double h,
  double ik_score,
  double ik_solve_ms,
  const ExtractRecordStepCallback& record_step) const
{
  ExtractRolloutTiming timing;
  timing.candidate_order = candidate_order;
  timing.h_index = h_index;
  timing.seed_index = seed_index;
  timing.h = h;
  timing.ik_score = ik_score;
  timing.ik_solve_ms = ik_solve_ms;

  const auto t0 = std::chrono::steady_clock::now();
  moveit::core::RobotState current_state(start_state);
  double last_retreat_x = 0.0;
  double current_lift_z = 0.0;
  double min_allowed_tip_z = current_state.getGlobalLinkTransform(config_.left_tip).translation().z();
  const size_t max_steps = config_.motion_planner ? config_.motion_planner->maxStepCount() : 0;

  if (record_step) {
    nlohmann::json extra = {
      {"stage_kind", "left_extract_all_legal_ik_start"},
      {"candidate_order", candidate_order},
      {"h_index", h_index},
      {"seed_index", seed_index},
      {"h", h},
      {"ik_score", ik_score},
      {"ik_solve_ms", ik_solve_ms},
      {"accepted", true},
      {"step", 0},
      {"retreat_x", 0.0},
      {"lift_z", 0.0},
      {"pitch_up_deg", 0.0},
      {"left_tip_z", current_state.getGlobalLinkTransform(config_.left_tip).translation().z()},
      {"tool_normal_z", (current_state.getGlobalLinkTransform(config_.left_tip).linear() * Eigen::Vector3d::UnitZ()).z()},
      {"detached_from_neighbors", false}
    };
    record_step(0, current_state, extra);
  }

  for (size_t step = 1; step <= max_steps; ++step) {
    std::vector<ExtractCandidate> candidates =
      makeCandidatesForSide("left", current_state, source_box, left_box, left_box_id,
                            step, last_retreat_x, current_lift_z, min_allowed_tip_z);

    auto best_it = std::min_element(candidates.begin(), candidates.end(), [&](const ExtractCandidate& a, const ExtractCandidate& b) {
      return config_.candidate_scorer->score("left", a, current_state, last_retreat_x) <
             config_.candidate_scorer->score("left", b, current_state, last_retreat_x);
    });

    if (best_it == candidates.end() || !best_it->state_valid) {
      ++timing.failed_steps;
      std::map<std::string, size_t> rejection_counts;
      for (const auto& candidate : candidates) {
        const std::string key = candidate.rejection_reason.empty() ? "unknown" : candidate.rejection_reason;
        rejection_counts[key]++;
      }
      if (!rejection_counts.empty()) {
        timing.failure_reason = rejection_counts.begin()->first;
        size_t best_count = 0;
        for (const auto& [reason, count] : rejection_counts) {
          if (count > best_count) {
            best_count = count;
            timing.failure_reason = reason;
          }
        }
      } else {
        timing.failure_reason = "no_valid_candidate";
      }
      if (record_step) {
        nlohmann::json rejection_json = nlohmann::json::object();
        for (const auto& [reason, count] : rejection_counts) {
          rejection_json[reason] = count;
        }
        nlohmann::json extra = {
          {"stage_kind", "left_extract_all_legal_ik_failed_step"},
          {"candidate_order", candidate_order},
          {"h_index", h_index},
          {"seed_index", seed_index},
          {"h", h},
          {"ik_score", ik_score},
          {"ik_solve_ms", ik_solve_ms},
          {"step", step},
          {"retreat_x", last_retreat_x},
          {"accepted", false},
          {"candidate_count", candidates.size()},
          {"rejection_counts", rejection_json},
          {"failure_reason", timing.failure_reason}
        };
        record_step(step, current_state, extra);
      }
      if (config_.fail_fast) break;
      continue;
    }

    const double selected_score = config_.candidate_scorer->score("left", *best_it, current_state, last_retreat_x);
    const double selected_joint_delta = config_.candidate_scorer->armJointDelta("left", current_state, *best_it->state);
    const double selected_tip_position_delta = config_.candidate_scorer->tipPositionDelta("left", current_state, *best_it->state);
    const double selected_tip_orientation_delta = config_.candidate_scorer->tipOrientationDelta("left", current_state, *best_it->state);

    current_state = *best_it->state;
    min_allowed_tip_z = std::max(min_allowed_tip_z, current_state.getGlobalLinkTransform(config_.left_tip).translation().z());
    last_retreat_x = best_it->retreat_x;
    current_lift_z = best_it->lift_z;
    timing.final_retreat_x = best_it->retreat_x;
    timing.final_lift_z = best_it->lift_z;
    timing.final_pitch_deg = best_it->pitch_up_rad * 180.0 / M_PI;
    ++timing.accepted_steps;

    if (record_step) {
      nlohmann::json extra = {
        {"stage_kind", "left_extract_all_legal_ik_step"},
        {"candidate_order", candidate_order},
        {"h_index", h_index},
        {"seed_index", seed_index},
        {"h", h},
        {"ik_score", ik_score},
        {"ik_solve_ms", ik_solve_ms},
        {"step", step},
        {"candidate_index", best_it->candidate_index},
        {"retreat_x", best_it->retreat_x},
        {"retreat_delta_x", best_it->retreat_delta_x},
        {"lift_z", best_it->lift_z},
        {"lift_delta_z", best_it->lift_delta_z},
        {"pitch_up_deg", best_it->pitch_up_rad * 180.0 / M_PI},
        {"pitch_delta_deg", best_it->pitch_delta_rad * 180.0 / M_PI},
        {"left_tip_z", current_state.getGlobalLinkTransform(config_.left_tip).translation().z()},
        {"tool_normal_z", (current_state.getGlobalLinkTransform(config_.left_tip).linear() * Eigen::Vector3d::UnitZ()).z()},
        {"detached_from_neighbors", best_it->detached_from_neighbors},
        {"candidate_count", candidates.size()},
        {"score", selected_score},
        {"joint_delta", selected_joint_delta},
        {"tip_position_delta", selected_tip_position_delta},
        {"tip_orientation_delta", selected_tip_orientation_delta},
        {"accepted", true}
      };
      record_step(step, current_state, extra);
    }

    if (best_it->detached_from_neighbors) {
      timing.success = true;
      timing.failure_reason.clear();
      timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
      break;
    }
  }

  if (!timing.success && timing.failure_reason.empty()) {
    timing.failure_reason = "reached_max_retreat_without_neighbor_detachment";
  }
  const auto t1 = std::chrono::steady_clock::now();
  timing.rollout_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return timing;
}

ExtractRolloutTiming ExtractRolloutPlanner::rolloutDual(
  const moveit::core::RobotState& start_state,
  const BoxSpec& left_source_box,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const BoxSpec& right_source_box,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  size_t candidate_order,
  size_t h_index,
  size_t seed_index,
  double h,
  double ik_score,
  double ik_solve_ms,
  const ExtractRecordStepCallback& record_step) const
{
  ExtractRolloutTiming timing;
  timing.candidate_order = candidate_order;
  timing.h_index = h_index;
  timing.seed_index = seed_index;
  timing.h = h;
  timing.ik_score = ik_score;
  timing.ik_solve_ms = ik_solve_ms;

  const auto t0 = std::chrono::steady_clock::now();
  moveit::core::RobotState current_state(start_state);
  if (config_.top_suction) {
    const double start_updown = currentUpdown(start_state);
    const double step_h = std::max(1e-6, config_.top_suction_updown_step);
    const size_t max_steps = static_cast<size_t>(
      std::ceil(std::max(0.0, config_.top_suction_max_lift) / step_h)) + config_.success_extra_steps + 1;

    if (record_step) {
      nlohmann::json extra = {
        {"stage_kind", "dual_top_suction_extract_start"},
        {"candidate_order", candidate_order},
        {"h_index", h_index},
        {"seed_index", seed_index},
        {"h", h},
        {"ik_score", ik_score},
        {"ik_solve_ms", ik_solve_ms},
        {"accepted", true},
        {"step", 0},
        {"updown", start_updown}
      };
      record_step(0, current_state, extra);
    }

    bool detached_seen = false;
    size_t extra_steps_after_detached = 0;
    for (size_t step = 1; step <= max_steps; ++step) {
      const double lift = step_h * static_cast<double>(step);
      current_state.setVariablePosition("updown", start_updown + lift);
      current_state.enforceBounds(config_.joint_group);
      current_state.update();

      bool left_detached = false;
      bool right_detached = false;
      std::string state_reason;
      const bool clear = !config_.dual_clear_callback ||
        config_.dual_clear_callback(
          current_state, left_box, left_box_id, right_box, right_box_id,
          &left_detached, &right_detached, &state_reason);
      if (!clear) {
        ++timing.failed_steps;
        timing.failure_reason = state_reason.empty() ? "top_suction_updown_extract_invalid" : state_reason;
        if (record_step) {
          nlohmann::json extra = {
            {"stage_kind", "dual_top_suction_extract_failed_step"},
            {"candidate_order", candidate_order},
            {"h_index", h_index},
            {"seed_index", seed_index},
            {"h", h},
            {"step", step},
            {"updown", currentUpdown(current_state)},
            {"lift_z", lift},
            {"failure_reason", timing.failure_reason},
            {"accepted", false}
          };
          record_step(step, current_state, extra);
        }
        if (config_.fail_fast) break;
        continue;
      }

      ++timing.accepted_steps;
      timing.final_lift_z = lift;
      timing.right_final_lift_z = lift;
      timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
      if (record_step) {
        nlohmann::json extra = {
          {"stage_kind", "dual_top_suction_extract_step"},
          {"candidate_order", candidate_order},
          {"h_index", h_index},
          {"seed_index", seed_index},
          {"h", h},
          {"step", step},
          {"updown", currentUpdown(current_state)},
          {"lift_z", lift},
          {"left_detached_from_neighbors", left_detached},
          {"right_detached_from_neighbors", right_detached},
          {"accepted", true}
        };
        record_step(step, current_state, extra);
      }

      if (left_detached && right_detached) {
        detached_seen = true;
        timing.success = true;
        timing.failure_reason.clear();
      }
      if (detached_seen) {
        ++extra_steps_after_detached;
      }
      if (detached_seen && extra_steps_after_detached > config_.success_extra_steps) {
        break;
      }
    }

    if (!timing.success && timing.failure_reason.empty()) {
      timing.failure_reason = "top_suction_reached_max_lift_without_detachment";
    }
    const auto t1 = std::chrono::steady_clock::now();
    timing.rollout_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return timing;
  }

  if (config_.dual_async) {
    const auto left_path = rolloutArm("left", start_state, left_source_box, left_box, left_box_id);
    const auto right_path = rolloutArm("right", start_state, right_source_box, right_box, right_box_id);
    timing.accepted_steps = left_path.accepted_steps + right_path.accepted_steps;
    timing.failed_steps = left_path.failed_steps + right_path.failed_steps;
    timing.final_retreat_x = left_path.final_retreat_x;
    timing.final_lift_z = left_path.final_lift_z;
    timing.final_pitch_deg = left_path.final_pitch_deg;
    timing.right_final_retreat_x = right_path.final_retreat_x;
    timing.right_final_lift_z = right_path.final_lift_z;
    timing.right_final_pitch_deg = right_path.final_pitch_deg;

    std::vector<moveit::core::RobotStatePtr> combined_states;
    std::string async_reason;
    const bool async_valid = validateAsyncPath(
      start_state, left_path, right_path, left_box, left_box_id, right_box, right_box_id,
      &combined_states, &async_reason);
    if (!async_valid) {
      timing.failure_reason = async_reason;
    } else {
      timing.success = true;
      timing.failure_reason.clear();
      timing.final_state = combined_states.empty() ? std::make_shared<moveit::core::RobotState>(start_state) : combined_states.back();
    }

    if (record_step) {
      nlohmann::json start_extra = {
        {"stage_kind", "dual_extract_async_start"},
        {"candidate_order", candidate_order},
        {"h_index", h_index},
        {"seed_index", seed_index},
        {"h", h},
        {"ik_score", ik_score},
        {"ik_solve_ms", ik_solve_ms},
        {"accepted", true},
        {"step", 0},
        {"left_path_success", left_path.success},
        {"right_path_success", right_path.success},
        {"left_path_steps", left_path.accepted_steps},
        {"right_path_steps", right_path.accepted_steps},
        {"async_valid", async_valid},
        {"failure_reason", timing.failure_reason}
      };
      record_step(0, start_state, start_extra);
      const size_t recorded_step_count = std::max(
        combined_states.size(),
        std::max(left_path.states.empty() ? 0 : left_path.states.size() - 1,
                 right_path.states.empty() ? 0 : right_path.states.size() - 1));
      for (size_t step = 1; step <= recorded_step_count; ++step) {
        const auto left_index = std::min(step, left_path.selected_candidates.size());
        const auto right_index = std::min(step, right_path.selected_candidates.size());
        auto display_state = combined_states.empty() || step > combined_states.size()
          ? std::make_shared<moveit::core::RobotState>(
              combineAsyncArmStates(start_state, left_path, right_path, left_index, right_index))
          : combined_states[step - 1];
        nlohmann::json extra = {
          {"stage_kind", "dual_extract_async_step"},
          {"candidate_order", candidate_order},
          {"h_index", h_index},
          {"seed_index", seed_index},
          {"h", h},
          {"ik_score", ik_score},
          {"ik_solve_ms", ik_solve_ms},
          {"step", step},
          {"accepted", true},
          {"left_path_success", left_path.success},
          {"right_path_success", right_path.success},
          {"left_path_steps", left_path.accepted_steps},
          {"right_path_steps", right_path.accepted_steps},
          {"async_valid", async_valid},
          {"failure_reason", timing.failure_reason}
        };
        if (left_index > 0 && left_index <= left_path.selected_candidates.size()) {
          const auto& left = left_path.selected_candidates[left_index - 1];
          extra["left_retreat_x"] = left.retreat_x;
          extra["left_lift_z"] = left.lift_z;
          extra["left_pitch_up_deg"] = left.pitch_up_rad * 180.0 / M_PI;
          extra["left_detached_from_neighbors"] = left.detached_from_neighbors;
        }
        if (right_index > 0 && right_index <= right_path.selected_candidates.size()) {
          const auto& right = right_path.selected_candidates[right_index - 1];
          extra["right_retreat_x"] = right.retreat_x;
          extra["right_lift_z"] = right.lift_z;
          extra["right_pitch_up_deg"] = right.pitch_up_rad * 180.0 / M_PI;
          extra["right_detached_from_neighbors"] = right.detached_from_neighbors;
        }
        record_step(step, *display_state, extra);
      }
    }

    const auto t1 = std::chrono::steady_clock::now();
    timing.rollout_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return timing;
  }

  double left_last_retreat_x = 0.0;
  double right_last_retreat_x = 0.0;
  double left_lift_z = 0.0;
  double right_lift_z = 0.0;
  double left_min_tip_z = current_state.getGlobalLinkTransform(config_.left_tip).translation().z();
  double right_min_tip_z = current_state.getGlobalLinkTransform(config_.right_tip).translation().z();
  const size_t max_steps = config_.motion_planner ? config_.motion_planner->maxStepCount() : 0;

  if (record_step) {
    nlohmann::json extra = {
      {"stage_kind", "dual_extract_all_legal_ik_start"},
      {"candidate_order", candidate_order},
      {"h_index", h_index},
      {"seed_index", seed_index},
      {"h", h},
      {"ik_score", ik_score},
      {"ik_solve_ms", ik_solve_ms},
      {"accepted", true},
      {"step", 0},
      {"left_retreat_x", 0.0},
      {"right_retreat_x", 0.0},
      {"left_lift_z", 0.0},
      {"right_lift_z", 0.0},
      {"left_detached_from_neighbors", false},
      {"right_detached_from_neighbors", false}
    };
    record_step(0, current_state, extra);
  }

  for (size_t step = 1; step <= max_steps; ++step) {
    const auto left_candidates =
      makeCandidatesForSide("left", current_state, left_source_box, left_box, left_box_id,
                            step, left_last_retreat_x, left_lift_z, left_min_tip_z);
    const auto right_candidates =
      makeCandidatesForSide("right", current_state, right_source_box, right_box, right_box_id,
                            step, right_last_retreat_x, right_lift_z, right_min_tip_z);
    const auto best = selectDualStepCandidate(
      current_state,
      left_candidates,
      right_candidates,
      left_last_retreat_x,
      right_last_retreat_x,
      left_box,
      left_box_id,
      right_box,
      right_box_id);

    if (!best.state_valid || !best.state) {
      ++timing.failed_steps;
      timing.failure_reason = best.rejection_reason.empty() ? "no_valid_dual_extract_candidate" : best.rejection_reason;
      if (record_step) {
        nlohmann::json extra = {
          {"stage_kind", "dual_extract_all_legal_ik_failed_step"},
          {"candidate_order", candidate_order},
          {"h_index", h_index},
          {"seed_index", seed_index},
          {"h", h},
          {"ik_score", ik_score},
          {"ik_solve_ms", ik_solve_ms},
          {"step", step},
          {"accepted", false},
          {"left_candidate_count", left_candidates.size()},
          {"right_candidate_count", right_candidates.size()},
          {"failure_reason", timing.failure_reason}
        };
        record_step(step, current_state, extra);
      }
      if (config_.fail_fast) break;
      continue;
    }

    current_state = *best.state;
    left_min_tip_z = std::max(left_min_tip_z, current_state.getGlobalLinkTransform(config_.left_tip).translation().z());
    right_min_tip_z = std::max(right_min_tip_z, current_state.getGlobalLinkTransform(config_.right_tip).translation().z());
    left_last_retreat_x = best.left.retreat_x;
    right_last_retreat_x = best.right.retreat_x;
    left_lift_z = best.left.lift_z;
    right_lift_z = best.right.lift_z;
    timing.final_retreat_x = best.left.retreat_x;
    timing.final_lift_z = best.left.lift_z;
    timing.final_pitch_deg = best.left.pitch_up_rad * 180.0 / M_PI;
    timing.right_final_retreat_x = best.right.retreat_x;
    timing.right_final_lift_z = best.right.lift_z;
    timing.right_final_pitch_deg = best.right.pitch_up_rad * 180.0 / M_PI;
    ++timing.accepted_steps;

    if (record_step) {
      nlohmann::json extra = {
        {"stage_kind", "dual_extract_all_legal_ik_step"},
        {"candidate_order", candidate_order},
        {"h_index", h_index},
        {"seed_index", seed_index},
        {"h", h},
        {"ik_score", ik_score},
        {"ik_solve_ms", ik_solve_ms},
        {"step", step},
        {"left_candidate_index", best.left.candidate_index},
        {"right_candidate_index", best.right.candidate_index},
        {"left_retreat_x", best.left.retreat_x},
        {"right_retreat_x", best.right.retreat_x},
        {"left_retreat_delta_x", best.left.retreat_delta_x},
        {"right_retreat_delta_x", best.right.retreat_delta_x},
        {"left_lift_z", best.left.lift_z},
        {"right_lift_z", best.right.lift_z},
        {"left_pitch_up_deg", best.left.pitch_up_rad * 180.0 / M_PI},
        {"right_pitch_up_deg", best.right.pitch_up_rad * 180.0 / M_PI},
        {"left_detached_from_neighbors", best.left_detached},
        {"right_detached_from_neighbors", best.right_detached},
        {"left_candidate_count", left_candidates.size()},
        {"right_candidate_count", right_candidates.size()},
        {"score", best.score},
        {"accepted", true}
      };
      record_step(step, current_state, extra);
    }

    if (best.left_detached && best.right_detached) {
      timing.success = true;
      timing.failure_reason.clear();
      timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
      break;
    }
  }

  if (!timing.success && timing.failure_reason.empty()) {
    timing.failure_reason = "reached_max_retreat_without_dual_neighbor_detachment";
  }
  const auto t1 = std::chrono::steady_clock::now();
  timing.rollout_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return timing;
}

}  // namespace alfa_robot::motion

namespace alfa_robot::motion
{

ExtractBenchmarkSummary summarize_extract_timings(
  const std::vector<ExtractRolloutTiming>& timings)
{
  ExtractBenchmarkSummary summary;
  for (const auto& timing : timings) {
    summary.total_interval_ms += timing.interval_ms;
    summary.total_rollout_ms += timing.rollout_ms;
    summary.any_success = summary.any_success || timing.success;

    if (timing.loaded_plan_attempted) {
      ++summary.loaded_plan_attempted_count;
      summary.total_loaded_plan_ms += timing.loaded_plan_ms;
    }
    if (timing.loaded_plan_success) {
      ++summary.loaded_plan_success_count;
      if (summary.loaded_plan_first_success_rank == 0) {
        summary.loaded_plan_first_success_rank = timing.loaded_plan_rank;
        summary.loaded_plan_first_success_candidate_order = timing.candidate_order;
      }
    }
  }

  summary.mean_interval_ms = timings.size() > 1
    ? summary.total_interval_ms / static_cast<double>(timings.size() - 1)
    : 0.0;
  summary.mean_rollout_ms = !timings.empty()
    ? summary.total_rollout_ms / static_cast<double>(timings.size())
    : 0.0;
  summary.mean_loaded_plan_ms = summary.loaded_plan_attempted_count > 0
    ? summary.total_loaded_plan_ms / static_cast<double>(summary.loaded_plan_attempted_count)
    : 0.0;
  return summary;
}

}  // namespace alfa_robot::motion

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace alfa_robot::motion
{

bool ExtractBenchmarkCsvWriter::write(
  const std::string& path,
  const std::vector<ExtractRolloutTiming>& timings,
  const rclcpp::Logger& logger)
{
  if (path.empty()) return true;

  const std::filesystem::path csv_path(path);
  if (!csv_path.parent_path().empty()) {
    std::filesystem::create_directories(csv_path.parent_path());
  }

  std::ofstream out(csv_path, std::ios::out | std::ios::trunc);
  if (!out) {
    RCLCPP_WARN(logger, "Failed to open extract benchmark CSV: %s", path.c_str());
    return false;
  }

  out << "candidate_order,h_index,seed_index,h,ik_score,ik_solve_ms,rollout_ms,interval_ms,success,"
         "loaded_plan_attempted,loaded_plan_success,loaded_plan_ms,loaded_plan_points,"
         "loaded_plan_trajectory_distance,loaded_plan_selected,"
         "lateral_shift_attempted,lateral_shift_success,lateral_shift_ms,lateral_shift_reached_distance,lateral_shift_points,"
         "loaded_plan_rank,loaded_pose_distance_sum,loaded_pose_distance_l2,loaded_pose_max_joint_delta,"
         "selected_left_loaded_pose_index,selected_right_loaded_pose_index,"
         "selected_left_loaded_pose_distance,selected_right_loaded_pose_distance,"
         "accepted_steps,failed_steps,final_retreat_x,final_lift_z,final_pitch_deg,"
         "right_final_retreat_x,right_final_lift_z,right_final_pitch_deg,"
         "failure_reason,loaded_plan_failure_reason\n";
  out << std::setprecision(12);

  for (const auto& timing : timings) {
    out << timing.candidate_order << ','
        << timing.h_index << ','
        << timing.seed_index << ','
        << timing.h << ','
        << timing.ik_score << ','
        << timing.ik_solve_ms << ','
        << timing.rollout_ms << ','
        << timing.interval_ms << ','
        << (timing.success ? 1 : 0) << ','
        << (timing.loaded_plan_attempted ? 1 : 0) << ','
        << (timing.loaded_plan_success ? 1 : 0) << ','
        << timing.loaded_plan_ms << ','
        << timing.loaded_plan_points << ','
        << timing.loaded_plan_trajectory_distance << ','
        << (timing.loaded_plan_selected ? 1 : 0) << ','
        << (timing.lateral_shift_attempted ? 1 : 0) << ','
        << (timing.lateral_shift_success ? 1 : 0) << ','
        << timing.lateral_shift_ms << ','
        << timing.lateral_shift_reached_distance << ','
        << timing.lateral_shift_points << ','
        << timing.loaded_plan_rank << ','
        << timing.loaded_pose_distance_sum << ','
        << timing.loaded_pose_distance_l2 << ','
        << timing.loaded_pose_max_joint_delta << ','
        << timing.selected_left_loaded_pose_index << ','
        << timing.selected_right_loaded_pose_index << ','
        << timing.selected_left_loaded_pose_distance << ','
        << timing.selected_right_loaded_pose_distance << ','
        << timing.accepted_steps << ','
        << timing.failed_steps << ','
        << timing.final_retreat_x << ','
        << timing.final_lift_z << ','
        << timing.final_pitch_deg << ','
        << timing.right_final_retreat_x << ','
        << timing.right_final_lift_z << ','
        << timing.right_final_pitch_deg << ','
        << '"' << timing.failure_reason << '"' << ','
        << '"' << timing.loaded_plan_failure_reason << '"' << '\n';
  }

  RCLCPP_INFO(logger, "Wrote extract all-legal-IK timing CSV: %s rows=%zu", path.c_str(), timings.size());
  return true;
}

}  // namespace alfa_robot::motion

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

namespace alfa_robot::motion
{

ExtractBenchmarkRunner::ExtractBenchmarkRunner(
  ExtractBenchmarkRunnerConfig config,
  ExtractBenchmarkRunnerCallbacks callbacks)
: config_(std::move(config)), callbacks_(std::move(callbacks))
{}

std::vector<ik_benchmark::UpdownAwareIkCandidate> ExtractBenchmarkRunner::sortedLegalCandidates(
  const ik_benchmark::UpdownAwareIkResult& ik_result,
  size_t* original_legal_count,
  IkCandidateSelectionStats* dedup_stats) const
{
  std::vector<ik_benchmark::UpdownAwareIkCandidate> legal_candidates;
  for (const auto& candidate : ik_result.candidates) {
    if (candidate.legal) {
      legal_candidates.push_back(candidate);
    }
  }
  std::sort(legal_candidates.begin(), legal_candidates.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.score < rhs.score; });
  if (original_legal_count) {
    *original_legal_count = legal_candidates.size();
  }
  if (config_.candidate_selector) {
    legal_candidates = config_.candidate_selector->select(legal_candidates, dedup_stats);
  } else if (dedup_stats) {
    dedup_stats->input_count = legal_candidates.size();
    dedup_stats->unique_count = legal_candidates.size();
    dedup_stats->selected_count = legal_candidates.size();
  }
  return legal_candidates;
}

bool ExtractBenchmarkRunner::writeCsv(const std::vector<ExtractRolloutTiming>& timings) const
{
  return ExtractBenchmarkCsvWriter::write(config_.csv_path, timings, config_.logger);
}

bool ExtractBenchmarkRunner::runLeft(
  const std::string& prefix,
  const moveit::core::RobotState& seed_state,
  const ik_benchmark::UpdownAwareIkResult& ik_result,
  const AttachedBoxSpec& left_box,
  int left_box_id)
{
  size_t original_legal_count = 0;
  IkCandidateSelectionStats dedup_stats;
  auto legal_candidates = sortedLegalCandidates(ik_result, &original_legal_count, &dedup_stats);
  if (legal_candidates.empty()) {
    return callbacks_.fail ? callbacks_.fail(prefix + "/extract_benchmark: no legal IK candidates") : false;
  }

  if (config_.record_tip_error_ik_candidates && callbacks_.record_tip_errors) {
    callbacks_.record_tip_errors(prefix, seed_state, ik_result, left_box);
  }

  std::vector<ExtractRolloutTiming> timings;
  timings.reserve(legal_candidates.size());
  auto previous_start = std::chrono::steady_clock::now();
  bool any_success = false;
  for (size_t i = 0; i < legal_candidates.size(); ++i) {
    const auto start = std::chrono::steady_clock::now();
    auto state = callbacks_.state_from_candidate(seed_state, legal_candidates[i]);
    std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)> record_step;
    if (config_.record_rollouts && callbacks_.record_keyframe) {
      record_step = [&](size_t step_index, const moveit::core::RobotState& rollout_state, const nlohmann::json& extra) {
        callbacks_.record_keyframe(
          prefix + "/candidate_" + std::to_string(i) + "/step_" + std::to_string(step_index),
          rollout_state,
          std::vector<AttachedBoxSpec>{left_box},
          extra);
      };
    }
    auto timing = callbacks_.rollout_left(
      state, left_box, left_box_id, i, legal_candidates[i], record_step);
    if (timing.success && timing.final_state && callbacks_.fill_loaded_metrics) {
      callbacks_.fill_loaded_metrics(timing);
    }
    timing.interval_ms = std::chrono::duration<double, std::milli>(start - previous_start).count();
    previous_start = start;
    any_success = any_success || timing.success;
    timings.push_back(std::move(timing));
  }
  if (!timings.empty()) {
    timings.front().interval_ms = 0.0;
  }

  LoadedPoseBatchPlanOptions loaded_options = config_.loaded_options;
  loaded_options.stop_on_first_success = false;
  const LoadedPoseBatchPlanResult loaded_batch = config_.loaded_pose_planner
    ? config_.loaded_pose_planner->planBatch(prefix, timings, std::vector<AttachedBoxSpec>{left_box}, loaded_options)
    : LoadedPoseBatchPlanResult{};

  writeCsv(timings);

  const ExtractBenchmarkSummary summary = summarize_extract_timings(timings);
  RCLCPP_INFO(config_.logger,
              "[%s/extract_benchmark] legal_ik=%zu selected=%zu dedup=%s unique=%zu removed=%zu dedup_ms=%.3f success_any=%s loaded_plan=%zu/%zu mean_interval=%.3fms mean_rollout=%.3fms mean_loaded_plan=%.3fms",
              prefix.c_str(), original_legal_count, timings.size(),
              dedup_stats.enabled ? "true" : "false", dedup_stats.unique_count,
              dedup_stats.removed_count, dedup_stats.elapsed_ms,
              any_success ? "true" : "false",
              summary.loaded_plan_success_count, summary.loaded_plan_attempted_count,
              summary.mean_interval_ms, summary.mean_rollout_ms, summary.mean_loaded_plan_ms);

  if (callbacks_.record_summary) {
    callbacks_.record_summary(nlohmann::json({
      {"type", "extract_benchmark_summary"},
      {"legal_ik_count", original_legal_count},
      {"tested_ik_count", legal_candidates.size()},
      {"candidate_limit", config_.candidate_limit},
      {"ik_dedup_enabled", dedup_stats.enabled},
      {"ik_dedup_joint_threshold_deg", config_.dedup_joint_threshold_rad * 180.0 / M_PI},
      {"ik_dedup_h_threshold", config_.dedup_h_threshold},
      {"ik_dedup_input_count", dedup_stats.input_count},
      {"ik_dedup_unique_count", dedup_stats.unique_count},
      {"ik_dedup_removed_count", dedup_stats.removed_count},
      {"ik_dedup_selected_count", dedup_stats.selected_count},
      {"ik_dedup_ms", dedup_stats.elapsed_ms},
      {"success_any", any_success},
      {"mean_interval_ms", summary.mean_interval_ms},
      {"mean_rollout_ms", summary.mean_rollout_ms},
      {"loaded_plan_after_success", config_.plan_loaded_after_success},
      {"loaded_plan_candidate_limit", config_.loaded_options.candidate_limit},
      {"loaded_plan_sort_by_pose_distance", config_.loaded_options.sort_by_pose_distance},
      {"loaded_plan_sorted_success_candidate_count", loaded_batch.plan_indices.size()},
      {"loaded_plan_attempted_count", summary.loaded_plan_attempted_count},
      {"loaded_plan_success_count", summary.loaded_plan_success_count},
      {"mean_loaded_plan_ms", summary.mean_loaded_plan_ms},
      {"csv_path", config_.csv_path},
      {"rollouts_recorded", config_.record_rollouts},
      {"ik_candidate_rejection_counts", callbacks_.rejection_counts_json ? callbacks_.rejection_counts_json(ik_result) : nlohmann::json::object()}
    }));
  }
  return any_success;
}

ExtractRolloutTiming ExtractBenchmarkRunner::runDualCandidate(
  const std::string& prefix,
  const moveit::core::RobotState& seed_state,
  const std::vector<ik_benchmark::UpdownAwareIkCandidate>& legal_candidates,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id,
  size_t index,
  bool record_rollout) const
{
  auto state = callbacks_.state_from_candidate(seed_state, legal_candidates[index]);
  std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)> record_step;
  if (record_rollout && callbacks_.record_keyframe) {
    const std::vector<AttachedBoxSpec> boxes{left_box, right_box};
    record_step = [&, boxes, index](size_t step_index, const moveit::core::RobotState& rollout_state, const nlohmann::json& extra) {
      callbacks_.record_keyframe(
        prefix + "/candidate_" + std::to_string(index) + "/step_" + std::to_string(step_index),
        rollout_state,
        boxes,
        extra);
    };
  }
  auto timing = callbacks_.rollout_dual(
    state, left_box, left_box_id, right_box, right_box_id,
    index, legal_candidates[index], record_step);
  if (timing.success && timing.final_state && callbacks_.fill_loaded_metrics) {
    callbacks_.fill_loaded_metrics(timing);
  }
  return timing;
}

bool ExtractBenchmarkRunner::runDual(
  const std::string& prefix,
  const moveit::core::RobotState& seed_state,
  const ik_benchmark::UpdownAwareIkResult& ik_result,
  const AttachedBoxSpec& left_box,
  int left_box_id,
  const AttachedBoxSpec& right_box,
  int right_box_id)
{
  size_t original_legal_count = 0;
  IkCandidateSelectionStats dedup_stats;
  auto legal_candidates = sortedLegalCandidates(ik_result, &original_legal_count, &dedup_stats);
  if (legal_candidates.empty()) {
    return callbacks_.fail ? callbacks_.fail(prefix + "/dual_extract_benchmark: no legal IK candidates") : false;
  }

  std::vector<ExtractRolloutTiming> timings(legal_candidates.size());
  bool any_success = false;
  const size_t requested_extract_workers = std::max<size_t>(1, config_.extract_workers);
  const size_t used_extract_workers = config_.record_rollouts
    ? 1
    : std::max<size_t>(1, std::min(requested_extract_workers, legal_candidates.size()));
  const auto extract_wall_start = std::chrono::steady_clock::now();
  if (used_extract_workers <= 1) {
    auto previous_start = extract_wall_start;
    for (size_t i = 0; i < legal_candidates.size(); ++i) {
      const auto start = std::chrono::steady_clock::now();
      auto timing = runDualCandidate(
        prefix, seed_state, legal_candidates, left_box, left_box_id, right_box, right_box_id,
        i, config_.record_rollouts);
      timing.interval_ms = std::chrono::duration<double, std::milli>(start - previous_start).count();
      previous_start = start;
      timings[i] = std::move(timing);
    }
    if (!timings.empty()) {
      timings.front().interval_ms = 0.0;
    }
  } else {
    std::atomic<size_t> next_index{0};
    std::vector<std::thread> workers;
    workers.reserve(used_extract_workers);
    for (size_t worker_index = 0; worker_index < used_extract_workers; ++worker_index) {
      workers.emplace_back([&, worker_index]() {
        (void)worker_index;
        while (true) {
          const size_t i = next_index.fetch_add(1);
          if (i >= legal_candidates.size()) {
            break;
          }
          timings[i] = runDualCandidate(
            prefix, seed_state, legal_candidates, left_box, left_box_id, right_box, right_box_id,
            i, false);
        }
      });
    }
    for (auto& worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    for (auto& timing : timings) {
      timing.interval_ms = 0.0;
    }
  }
  const auto extract_wall_end = std::chrono::steady_clock::now();
  const double extract_wall_ms =
    std::chrono::duration<double, std::milli>(extract_wall_end - extract_wall_start).count();
  for (const auto& timing : timings) {
    any_success = any_success || timing.success;
  }

  const LoadedPoseBatchPlanResult loaded_batch = config_.loaded_pose_planner
    ? config_.loaded_pose_planner->planBatch(
        prefix, timings, std::vector<AttachedBoxSpec>{left_box, right_box},
        config_.loaded_options)
    : LoadedPoseBatchPlanResult{};
  const double loaded_plan_wall_ms = loaded_batch.wall_ms;

  writeCsv(timings);

  const ExtractBenchmarkSummary summary = summarize_extract_timings(timings);
  const double task_wall_ms =
    ik_result.wall_ms + dedup_stats.elapsed_ms + extract_wall_ms + loaded_plan_wall_ms;
  const bool benchmark_success = config_.plan_loaded_after_success
    ? summary.loaded_plan_success_count > 0
    : any_success;
  if (!benchmark_success && callbacks_.set_last_error) {
    callbacks_.set_last_error(prefix + "/dual_extract_benchmark: " +
      std::string(any_success ? "loaded_plan_failed" : "extract_failed"));
  }

  RCLCPP_INFO(config_.logger,
              "[%s/dual_extract_benchmark] legal_ik=%zu selected=%zu dedup=%s unique=%zu removed=%zu dedup_ms=%.3f extract=%zu workers wall=%.3fms success_any=%s loaded_plan=%zu/%zu wall=%.3fms task_wall=%.3fms mean_interval=%.3fms mean_rollout=%.3fms mean_loaded_plan=%.3fms",
              prefix.c_str(), original_legal_count, timings.size(),
              dedup_stats.enabled ? "true" : "false", dedup_stats.unique_count,
              dedup_stats.removed_count, dedup_stats.elapsed_ms,
              used_extract_workers, extract_wall_ms,
              any_success ? "true" : "false",
              summary.loaded_plan_success_count, summary.loaded_plan_attempted_count,
              loaded_plan_wall_ms, task_wall_ms,
              summary.mean_interval_ms, summary.mean_rollout_ms, summary.mean_loaded_plan_ms);

  if (callbacks_.record_summary) {
    callbacks_.record_summary(nlohmann::json({
      {"type", "extract_benchmark_summary"},
      {"stage", prefix},
      {"mode", "dual_extract"},
      {"dual_async", config_.dual_async},
      {"legal_ik_count", original_legal_count},
      {"tested_ik_count", legal_candidates.size()},
      {"candidate_limit", config_.candidate_limit},
      {"ik_dedup_enabled", dedup_stats.enabled},
      {"ik_dedup_joint_threshold_deg", config_.dedup_joint_threshold_rad * 180.0 / M_PI},
      {"ik_dedup_h_threshold", config_.dedup_h_threshold},
      {"ik_dedup_input_count", dedup_stats.input_count},
      {"ik_dedup_unique_count", dedup_stats.unique_count},
      {"ik_dedup_removed_count", dedup_stats.removed_count},
      {"ik_dedup_selected_count", dedup_stats.selected_count},
      {"ik_dedup_ms", dedup_stats.elapsed_ms},
      {"ik_wall_ms", ik_result.wall_ms},
      {"extract_parallel_requested_workers", requested_extract_workers},
      {"extract_parallel_used_workers", used_extract_workers},
      {"extract_parallel_enabled", used_extract_workers > 1},
      {"extract_wall_ms", extract_wall_ms},
      {"extract_sum_rollout_ms", summary.total_rollout_ms},
      {"task_success", benchmark_success},
      {"success_any", any_success},
      {"mean_interval_ms", summary.mean_interval_ms},
      {"mean_rollout_ms", summary.mean_rollout_ms},
      {"loaded_plan_after_success", config_.plan_loaded_after_success},
      {"loaded_plan_candidate_limit", config_.loaded_options.candidate_limit},
      {"loaded_plan_sort_by_pose_distance", config_.loaded_options.sort_by_pose_distance},
      {"loaded_plan_stop_on_first_success", config_.loaded_options.stop_on_first_success},
      {"loaded_plan_sorted_success_candidate_count", loaded_batch.plan_indices.size()},
      {"loaded_plan_attempted_count", summary.loaded_plan_attempted_count},
      {"loaded_plan_success_count", summary.loaded_plan_success_count},
      {"loaded_plan_first_success_rank", summary.loaded_plan_first_success_rank},
      {"loaded_plan_first_success_candidate_order", summary.loaded_plan_first_success_candidate_order},
      {"loaded_plan_selected_candidate_order", [&]() {
        for (const auto& timing : timings) {
          if (timing.loaded_plan_selected) return nlohmann::json(timing.candidate_order);
        }
        return nlohmann::json(nullptr);
      }()},
      {"loaded_plan_selected_rank", [&]() {
        for (const auto& timing : timings) {
          if (timing.loaded_plan_selected) return nlohmann::json(timing.loaded_plan_rank);
        }
        return nlohmann::json(nullptr);
      }()},
      {"loaded_plan_selected_trajectory_distance", [&]() {
        for (const auto& timing : timings) {
          if (timing.loaded_plan_selected) return nlohmann::json(timing.loaded_plan_trajectory_distance);
        }
        return nlohmann::json(nullptr);
      }()},
      {"loaded_plan_wall_ms", loaded_plan_wall_ms},
      {"loaded_plan_sum_ms", summary.total_loaded_plan_ms},
      {"mean_loaded_plan_ms", summary.mean_loaded_plan_ms},
      {"task_wall_ms", task_wall_ms},
      {"csv_path", config_.csv_path},
      {"rollouts_recorded", config_.record_rollouts},
      {"ik_candidate_rejection_counts", callbacks_.rejection_counts_json ? callbacks_.rejection_counts_json(ik_result) : nlohmann::json::object()}
    }));
  }
  return benchmark_success;
}

}  // namespace alfa_robot::motion
