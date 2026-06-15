#include "alfa_robot_moveit_config/extract_candidate_solver.hpp"
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

  const std::string base_link = side == "left" ? "left_v5_link0" : "right_v5_link0";
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
      out->joint_names.push_back(joint.getName());
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
    const double error = pose_position_error(target_in_base, achieved_eigen) +
                         pose_orientation_error(target_in_base, achieved_eigen);
    if (error < best_error) {
      best_error = error;
      best_solution = solution;
      found = true;
    }
    if (pose_position_error(target_in_base, achieved_eigen) <= config_.position_tolerance &&
        pose_orientation_error(target_in_base, achieved_eigen) <= config_.orientation_tolerance) {
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
    out->rejection_reason = request.side + "_kdl_no_solution";
    return false;
  }

  state->setVariablePosition("updown", request.fixed_updown);
  state->enforceBounds(config_.joint_group);
  state->update();

  const Eigen::Isometry3d& actual = state->getGlobalLinkTransform(tip);
  const double pos_error = pose_position_error(target, actual);
  const double ori_error = pose_orientation_error(target, actual);
  if (pos_error > config_.position_tolerance || ori_error > config_.orientation_tolerance) {
    std::ostringstream oss;
    oss << request.side << "_kdl_tip_error pos=" << pos_error << " ori=" << ori_error;
    out->rejection_reason = oss.str();
    return false;
  }

  const Eigen::Vector3d tool_normal = actual.linear() * Eigen::Vector3d::UnitZ();
  if (tool_normal.z() < config_.min_tool_normal_z) {
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
