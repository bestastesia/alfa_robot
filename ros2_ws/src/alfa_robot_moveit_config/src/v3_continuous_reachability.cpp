#include "alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;
using JointVector = std::array<double, 7>;
using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
using alfa_robot::analytic_ik::V3RedundantArmModel;
using alfa_robot::analytic_ik::V3RedundantIkRequest;
using alfa_robot::analytic_ik::V3RedundantIkSolution;

constexpr double kPi = 3.1415926535897932384626433832795;

double deg_to_rad(double value)
{
  return value * kPi / 180.0;
}

double rad_to_deg(double value)
{
  return value * 180.0 / kPi;
}

double normalize_angle(double value)
{
  return std::remainder(value, 2.0 * kPi);
}

std::vector<double> inclusive_range(double minimum, double maximum, double step)
{
  if (!(step > 0.0) || maximum < minimum) {
    throw std::invalid_argument("invalid inclusive range");
  }
  std::vector<double> values;
  const auto count = static_cast<size_t>(std::floor((maximum - minimum) / step + 1e-9));
  values.reserve(count + 2);
  for (size_t index = 0; index <= count; ++index) {
    values.push_back(minimum + static_cast<double>(index) * step);
  }
  if (maximum - values.back() > step * 0.5) {
    values.push_back(maximum);
  }
  return values;
}

Eigen::Matrix3d rotation_from_rpy(const std::vector<double>& rpy)
{
  if (rpy.size() != 3) {
    throw std::invalid_argument("target_orientation_rpy must contain exactly 3 values");
  }
  return (
    Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
}

double joint_distance_squared(const JointVector& lhs, const JointVector& rhs)
{
  double distance = 0.0;
  for (size_t index = 0; index < lhs.size(); ++index) {
    const double delta = lhs[index] - rhs[index];
    distance += delta * delta;
  }
  return distance;
}

double maximum_joint_delta(const JointVector& lhs, const JointVector& rhs)
{
  double maximum = 0.0;
  for (size_t index = 0; index < lhs.size(); ++index) {
    maximum = std::max(maximum, std::abs(lhs[index] - rhs[index]));
  }
  return maximum;
}

double rotation_distance(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs)
{
  const Eigen::Quaterniond lhs_quaternion(lhs);
  const Eigen::Quaterniond rhs_quaternion(rhs);
  return lhs_quaternion.angularDistance(rhs_quaternion);
}

std::optional<double> nearest_equivalent_within_limits(
  double value,
  double reference,
  double lower,
  double upper)
{
  std::optional<double> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int turns = -2; turns <= 2; ++turns) {
    const double candidate = value + static_cast<double>(turns) * 2.0 * kPi;
    if (candidate < lower - 1e-9 || candidate > upper + 1e-9) {
      continue;
    }
    const double distance = std::abs(candidate - reference);
    if (distance < best_distance) {
      best = candidate;
      best_distance = distance;
    }
  }
  return best;
}

std::optional<JointVector> unwrap_near_seed(
  const JointVector& joints,
  const JointVector& seed,
  const JointVector& lower,
  const JointVector& upper)
{
  JointVector unwrapped{};
  for (size_t index = 0; index < joints.size(); ++index) {
    const auto equivalent = nearest_equivalent_within_limits(
      joints[index], seed[index], lower[index], upper[index]);
    if (!equivalent) {
      return std::nullopt;
    }
    unwrapped[index] = *equivalent;
  }
  return unwrapped;
}

struct Candidate
{
  JointVector joints{};
  double swivel = 0.0;
  double position_error = 0.0;
  double orientation_error = 0.0;
  double limit_margin = 0.0;
  int shoulder_branch = 0;
  int elbow_branch = 0;
  int wrist_branch = 0;
  double score = 0.0;
};

struct SampleResult
{
  size_t step_index = 0;
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  bool success = false;
  std::string reason;
  JointVector joints{};
  double swivel = 0.0;
  double maximum_joint_delta_deg = 0.0;
  double position_error = 0.0;
  double orientation_error = 0.0;
  int shoulder_branch = 0;
  int elbow_branch = 0;
  int wrist_branch = 0;
  bool high_joint_gain = false;
  bool refined_transition = false;
  size_t analytic_attempts = 0;
  size_t collision_checks = 0;
  bool has_diagnostic_joints = false;
  JointVector diagnostic_joints{};
  double diagnostic_joint_delta_deg = 0.0;
  std::string diagnostic_pose_kind;
  bool diagnostic_edge_collision_checked = false;
  bool diagnostic_edge_collision_free = false;
  std::string diagnostic_edge_collision_reason;
};

struct LineResult
{
  size_t line_index = 0;
  double start_x = 0.0;
  double lateral = 0.0;
  double height = 0.0;
  bool complete = false;
  std::string failure_reason;
  size_t reached_point_count = 0;
  size_t failure_step_index = 0;
  JointVector start_joints{};
  JointVector end_joints{};
  Eigen::Vector3d last_valid_target = Eigen::Vector3d::Zero();
  Eigen::Vector3d failure_target = Eigen::Vector3d::Zero();
  bool has_last_valid_joints = false;
  bool has_diagnostic_joints = false;
  JointVector diagnostic_joints{};
  double diagnostic_joint_delta_deg = 0.0;
  std::string diagnostic_pose_kind;
  bool diagnostic_edge_collision_checked = false;
  bool diagnostic_edge_collision_free = false;
  std::string diagnostic_edge_collision_reason;
  double maximum_path_joint_delta_deg = 0.0;
  size_t high_joint_gain_edge_count = 0;
  size_t refined_transition_count = 0;
  double elapsed_ms = 0.0;
  std::vector<JointVector> path_joint_samples;
  std::vector<Eigen::Vector3d> path_target_samples;
};

struct EdgeCheckResult
{
  bool valid = false;
  bool collision = false;
  bool task_path_deviation = false;
  std::string reason;
  JointVector rejected_joints{};
  double maximum_position_deviation = 0.0;
  double maximum_orientation_deviation = 0.0;
};

struct TimingCounters
{
  size_t analytic_calls = 0;
  size_t analytic_solutions = 0;
  size_t collision_checks = 0;
  size_t task_path_fk_checks = 0;
  size_t high_joint_gain_edges = 0;
  size_t refinement_attempts = 0;
  size_t refinement_budget_exhaustions = 0;
  size_t refined_transitions = 0;
  std::chrono::nanoseconds analytic_time{0};
  std::chrono::nanoseconds collision_time{0};
  std::chrono::nanoseconds task_path_fk_time{0};
};

class V3ContinuousReachabilityNode : public rclcpp::Node
{
public:
  V3ContinuousReachabilityNode()
  : Node("v3_continuous_reachability")
  {
    side_ = declare_parameter<std::string>("side", "left");
    arm_mount_forward_offset_ = declare_parameter<double>("arm_mount_forward_offset", 0.0);
    test_pattern_ = declare_parameter<std::string>("test_pattern", "linear_x");
    x_start_min_ = declare_parameter<double>("x_start_min", 0.35);
    x_start_max_ = declare_parameter<double>("x_start_max", 0.75);
    x_start_step_ = declare_parameter<double>("x_start_step", 0.01);
    travel_ = declare_parameter<double>("travel", 0.40);
    travel_x_ = declare_parameter<double>("travel_x", 0.0);
    travel_y_ = declare_parameter<double>("travel_y", 0.0);
    travel_z_ = declare_parameter<double>("travel_z", 0.0);
    path_step_ = declare_parameter<double>("path_step", 0.01);
    disk_radius_ = declare_parameter<double>("disk_radius", 0.15);
    disk_ring_step_ = declare_parameter<double>("disk_ring_step", 0.03);
    lateral_min_ = declare_parameter<double>("lateral_min", -0.55);
    lateral_max_ = declare_parameter<double>("lateral_max", 0.85);
    lateral_step_ = declare_parameter<double>("lateral_step", 0.05);
    height_min_ = declare_parameter<double>("height_min", 0.05);
    height_max_ = declare_parameter<double>("height_max", 1.20);
    height_step_ = declare_parameter<double>("height_step", 0.05);
    target_orientation_rpy_ = declare_parameter<std::vector<double>>(
      "target_orientation_rpy", {0.0, kPi / 2.0, 0.0});
    if (target_orientation_rpy_.size() != 3) {
      throw std::invalid_argument("target_orientation_rpy must contain exactly 3 values");
    }
    target_orientation_rpy_[0] = declare_parameter<double>(
      "target_roll", target_orientation_rpy_[0]);
    target_orientation_rpy_[1] = declare_parameter<double>(
      "target_pitch", target_orientation_rpy_[1]);
    target_orientation_rpy_[2] = declare_parameter<double>(
      "target_yaw", target_orientation_rpy_[2]);
    initial_seed_ = vector_to_joints(declare_parameter<std::vector<double>>(
      "initial_seed", {-kPi / 2.0, -kPi / 2.0, 0.0, -kPi / 2.0, 0.0, 0.0, 0.0}),
      "initial_seed");
    swivel_step_ = deg_to_rad(declare_parameter<double>("swivel_step_deg", 5.0));
    maximum_joint_delta_ = deg_to_rad(
      declare_parameter<double>("maximum_joint_delta_deg", 10.0));
    edge_joint_step_ = deg_to_rad(declare_parameter<double>("edge_joint_step_deg", 2.5));
    maximum_task_position_deviation_ = declare_parameter<double>(
      "maximum_task_position_deviation", 0.001);
    maximum_task_orientation_deviation_ = deg_to_rad(declare_parameter<double>(
      "maximum_task_orientation_deviation_deg", 1.0));
    high_gain_swivel_neighbor_steps_ = static_cast<size_t>(std::max<long>(
      0, declare_parameter<int>("high_gain_swivel_neighbor_steps", 2)));
    refinement_max_depth_ = static_cast<size_t>(std::max<long>(
      0, declare_parameter<int>("refinement_max_depth", 3)));
    refinement_min_step_ = declare_parameter<double>("refinement_min_step", 0.00125);
    refinement_time_budget_ = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double, std::milli>(
        declare_parameter<double>("refinement_time_budget_ms", 2.0)));
    ignore_opposite_arm_ = declare_parameter<bool>("ignore_opposite_arm", true);
    record_failure_states_ = declare_parameter<bool>("record_failure_states", false);
    record_failure_paths_ = declare_parameter<bool>("record_failure_paths", false);
    output_json_ = declare_parameter<std::string>("output_json", "/tmp/v3_continuous_reachability.json");
    progress_period_ = static_cast<size_t>(std::max<long>(
      1, declare_parameter<int>("progress_period", 500)));

    if (side_ != "left" && side_ != "right") {
      throw std::invalid_argument("side must be left or right");
    }
    solver_ = V3RedundantArmAnalyticIk(
      side_ == "left" ? V3RedundantArmModel::V311Left :
      V3RedundantArmModel::V311Right);
    if (test_pattern_ != "linear_x" && test_pattern_ != "linear_vector" &&
        test_pattern_ != "planar_disk") {
      throw std::invalid_argument(
        "test_pattern must be linear_x, linear_vector, or planar_disk");
    }
    if (!(x_start_step_ > 0.0) || !(path_step_ > 0.0) || !(swivel_step_ > 0.0) ||
        !(maximum_joint_delta_ > 0.0) || !(edge_joint_step_ > 0.0) ||
        !(maximum_task_position_deviation_ > 0.0) ||
        !(maximum_task_orientation_deviation_ > 0.0) ||
        !(refinement_min_step_ > 0.0) || !(refinement_time_budget_.count() > 0.0)) {
      throw std::invalid_argument("all scan and continuity steps must be positive");
    }
    if (test_pattern_ == "planar_disk" &&
        (!(disk_radius_ > 0.0) || !(disk_ring_step_ > 0.0))) {
      throw std::invalid_argument("disk_radius and disk_ring_step must be positive");
    }
    if (test_pattern_ == "linear_vector" &&
        Eigen::Vector3d(travel_x_, travel_y_, travel_z_).norm() <= 1.0e-12) {
      throw std::invalid_argument("linear_vector travel must be non-zero");
    }
  }

  void run()
  {
    initialize_moveit();
    const auto scan_start = Clock::now();
    const std::vector<double> start_xs = inclusive_range(
      x_start_min_, x_start_max_, x_start_step_);
    const std::vector<double> laterals = inclusive_range(lateral_min_, lateral_max_, lateral_step_);
    const std::vector<double> heights = inclusive_range(height_min_, height_max_, height_step_);
    const std::vector<Eigen::Vector3d> path_offsets = build_path_offsets();
    const size_t path_point_count = path_offsets.size();
    const Eigen::Matrix3d target_rotation = rotation_from_rpy(target_orientation_rpy_);

    if (test_pattern_ == "planar_disk") {
      RCLCPP_INFO(
        get_logger(),
        "Starting V3 planar disk continuity scan: side=%s starts=%zu points_per_test=%zu radius=%.3f ring_step=%.3f path_step=%.3f x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f]",
        side_.c_str(), start_xs.size() * laterals.size() * heights.size(), path_point_count,
        disk_radius_, disk_ring_step_, path_step_, x_start_min_, x_start_max_,
        lateral_min_, lateral_max_, height_min_, height_max_);
    } else if (test_pattern_ == "linear_vector") {
      RCLCPP_INFO(
        get_logger(),
        "Starting V3 vector continuity scan: side=%s starts=%zu points_per_test=%zu vector=[%.3f,%.3f,%.3f] path_step=%.3f",
        side_.c_str(), start_xs.size() * laterals.size() * heights.size(), path_point_count,
        travel_x_, travel_y_, travel_z_, path_step_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Starting V3 continuity volume scan: side=%s starts=%zu points_per_test=%zu start_x=[%.3f,%.3f] travel=%.3f y=[%.3f,%.3f] z=[%.3f,%.3f]",
        side_.c_str(), start_xs.size() * laterals.size() * heights.size(), path_point_count,
        x_start_min_, x_start_max_, travel_, lateral_min_, lateral_max_, height_min_, height_max_);
    }

    std::vector<LineResult> lines;
    const size_t total_start_count = start_xs.size() * laterals.size() * heights.size();
    lines.reserve(total_start_count);
    std::unordered_map<std::string, size_t> failure_reasons;
    size_t complete_count = 0;
    size_t line_index = 0;
    for (const double start_x : start_xs) {
      for (const double height : heights) {
        for (const double lateral : laterals) {
          LineResult line;
          line.line_index = line_index;
          line.start_x = start_x;
          line.lateral = lateral;
          line.height = height;
          const auto line_start = Clock::now();

          std::optional<JointVector> previous_joints;
          std::optional<double> previous_swivel;
          std::optional<Eigen::Vector3d> previous_target;
          for (size_t step_index = 0; step_index < path_point_count; ++step_index) {
            const Eigen::Vector3d target =
              Eigen::Vector3d(start_x, lateral, height) + path_offsets[step_index];
            SampleResult sample = solve_sample(
              step_index, target, target_rotation, previous_joints, previous_swivel,
              previous_target);
            if (!sample.success) {
              line.failure_reason = sample.reason;
              line.failure_step_index = step_index;
              line.failure_target = target;
              line.has_diagnostic_joints = sample.has_diagnostic_joints;
              line.diagnostic_joints = sample.diagnostic_joints;
              line.diagnostic_joint_delta_deg = sample.diagnostic_joint_delta_deg;
              line.diagnostic_pose_kind = sample.diagnostic_pose_kind;
              line.diagnostic_edge_collision_checked =
                sample.diagnostic_edge_collision_checked;
              line.diagnostic_edge_collision_free = sample.diagnostic_edge_collision_free;
              line.diagnostic_edge_collision_reason =
                sample.diagnostic_edge_collision_reason;
              break;
            }
            if (!previous_joints) {
              line.start_joints = sample.joints;
            }
            line.end_joints = sample.joints;
            line.last_valid_target = target;
            line.has_last_valid_joints = true;
            line.maximum_path_joint_delta_deg = std::max(
              line.maximum_path_joint_delta_deg, sample.maximum_joint_delta_deg);
            line.high_joint_gain_edge_count += static_cast<size_t>(sample.high_joint_gain);
            line.refined_transition_count += static_cast<size_t>(sample.refined_transition);
            if (record_failure_paths_) {
              line.path_joint_samples.push_back(sample.joints);
              line.path_target_samples.push_back(target);
            }
            ++line.reached_point_count;
            previous_joints = sample.joints;
            previous_swivel = sample.swivel;
            previous_target = target;
          }
          line.complete = line.reached_point_count == path_point_count;
          if (line.complete) {
            line.path_joint_samples.clear();
            line.path_target_samples.clear();
          }
          line.elapsed_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - line_start).count();
          complete_count += static_cast<size_t>(line.complete);
          if (!line.complete) {
            ++failure_reasons[line.failure_reason];
          }
          lines.push_back(std::move(line));
          ++line_index;
          if (line_index % progress_period_ == 0 || line_index == total_start_count) {
            RCLCPP_INFO(
              get_logger(), "progress=%zu/%zu complete=%zu (%.1f%%)",
              line_index, total_start_count, complete_count,
              100.0 * static_cast<double>(complete_count) / static_cast<double>(line_index));
          }
        }
      }
    }

    const double total_ms = std::chrono::duration<double, std::milli>(Clock::now() - scan_start).count();
    write_json(lines, failure_reasons, path_point_count, total_ms);
    RCLCPP_INFO(
      get_logger(),
      "Scan complete: lines=%zu complete=%zu ratio=%.2f%% total=%.3fs analytic_calls=%zu analytic=%.3fs collision_checks=%zu collision=%.3fs task_fk_checks=%zu task_fk=%.3fs high_gain=%zu refinement_attempts=%zu refinement_budget_exhaustions=%zu refined=%zu output=%s",
      lines.size(), complete_count,
      lines.empty() ? 0.0 : 100.0 * static_cast<double>(complete_count) / static_cast<double>(lines.size()),
      total_ms / 1000.0,
      timing_.analytic_calls,
      std::chrono::duration<double>(timing_.analytic_time).count(),
      timing_.collision_checks,
      std::chrono::duration<double>(timing_.collision_time).count(),
      timing_.task_path_fk_checks,
      std::chrono::duration<double>(timing_.task_path_fk_time).count(),
      timing_.high_joint_gain_edges,
      timing_.refinement_attempts,
      timing_.refinement_budget_exhaustions,
      timing_.refined_transitions,
      output_json_.c_str());
  }

private:
  std::vector<Eigen::Vector3d> build_path_offsets() const
  {
    if (test_pattern_ == "linear_x") {
      const size_t segments = static_cast<size_t>(
        std::llround(std::abs(travel_) / path_step_));
      const double actual_step = segments > 0
        ? travel_ / static_cast<double>(segments)
        : 0.0;
      std::vector<Eigen::Vector3d> offsets;
      offsets.reserve(segments + 1);
      for (size_t index = 0; index <= segments; ++index) {
        offsets.emplace_back(actual_step * static_cast<double>(index), 0.0, 0.0);
      }
      return offsets;
    }

    if (test_pattern_ == "linear_vector") {
      const Eigen::Vector3d travel_vector(travel_x_, travel_y_, travel_z_);
      const size_t segments = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(travel_vector.norm() / path_step_)));
      std::vector<Eigen::Vector3d> offsets;
      offsets.reserve(segments + 1);
      for (size_t index = 0; index <= segments; ++index) {
        offsets.emplace_back(
          travel_vector * static_cast<double>(index) / static_cast<double>(segments));
      }
      return offsets;
    }

    std::vector<Eigen::Vector3d> offsets;
    offsets.emplace_back(Eigen::Vector3d::Zero());
    double previous_radius = 0.0;
    while (previous_radius < disk_radius_ - 1e-12) {
      const double radius = std::min(disk_radius_, previous_radius + disk_ring_step_);
      const size_t radial_segments = std::max<size_t>(
        1, static_cast<size_t>(std::ceil((radius - previous_radius) / path_step_)));
      for (size_t segment = 1; segment <= radial_segments; ++segment) {
        const double ratio = static_cast<double>(segment) /
          static_cast<double>(radial_segments);
        const double radial_position = previous_radius + ratio * (radius - previous_radius);
        offsets.emplace_back(0.0, radial_position, 0.0);
      }

      const size_t ring_segments = std::max<size_t>(
        8, static_cast<size_t>(std::ceil(2.0 * kPi * radius / path_step_)));
      for (size_t segment = 1; segment <= ring_segments; ++segment) {
        const double angle = 2.0 * kPi * static_cast<double>(segment) /
          static_cast<double>(ring_segments);
        offsets.emplace_back(0.0, radius * std::cos(angle), radius * std::sin(angle));
      }
      previous_radius = radius;
    }
    return offsets;
  }

  static JointVector vector_to_joints(const std::vector<double>& values, const std::string& name)
  {
    if (values.size() != 7) {
      throw std::invalid_argument(name + " must contain exactly 7 values");
    }
    JointVector joints{};
    std::copy(values.begin(), values.end(), joints.begin());
    return joints;
  }

  void initialize_moveit()
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      throw std::runtime_error("failed to load robot model");
    }
    group_name_ = side_ + "_arm";
    arm_base_link_ = "arm_carriage";
    target_link_ = side_ + "_tool0";
    joint_group_ = robot_model_->getJointModelGroup(group_name_);
    if (!joint_group_ || joint_group_->getVariableCount() != 7) {
      throw std::runtime_error("expected a seven-axis MoveIt group: " + group_name_);
    }
    scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    state_->setToDefaultValues();
    state_->update(true);
    base_to_arm_base_ = state_->getGlobalLinkTransform(arm_base_link_);
    lower_limits_ = solver_.jointLowerLimits();
    upper_limits_ = solver_.jointUpperLimits();
    initial_swivel_ = solver_.swivelAngle(initial_seed_);

    if (ignore_opposite_arm_) {
      const std::string opposite = side_ == "left" ? "right" : "left";
      const auto* opposite_group = robot_model_->getJointModelGroup(opposite + "_arm");
      if (!opposite_group) {
        throw std::runtime_error("missing opposite arm group");
      }
      auto& acm = scene_->getAllowedCollisionMatrixNonConst();
      for (const auto& tested_link : joint_group_->getLinkModelNames()) {
        for (const auto& opposite_link : opposite_group->getLinkModelNames()) {
          acm.setEntry(tested_link, opposite_link, true);
        }
      }
    }
  }

  std::vector<double> swivel_search_order(double reference) const
  {
    const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(2.0 * kPi / swivel_step_)));
    std::vector<double> values;
    values.reserve(count);
    values.push_back(normalize_angle(reference));
    for (size_t offset = 1; values.size() < count; ++offset) {
      values.push_back(normalize_angle(reference + static_cast<double>(offset) * swivel_step_));
      if (values.size() < count) {
        values.push_back(normalize_angle(reference - static_cast<double>(offset) * swivel_step_));
      }
    }
    return values;
  }

  std::vector<Candidate> generate_candidates_for_swivel(
    const Eigen::Isometry3d& target_in_arm_base,
    const JointVector& seed,
    double swivel,
    double reference_swivel,
    size_t* analytic_attempts)
  {
    std::vector<Candidate> candidates;
    V3RedundantIkRequest request;
    request.target_in_arm_base = target_in_arm_base;
    request.swivel_angle = swivel;
    request.seed = seed;
    request.enforce_joint_limits = true;
    const auto start = Clock::now();
    auto solutions = solver_.solveInArmBase(request);
    timing_.analytic_time += Clock::now() - start;
    ++timing_.analytic_calls;
    timing_.analytic_solutions += solutions.size();
    ++(*analytic_attempts);
    for (const V3RedundantIkSolution& solution : solutions) {
      const auto unwrapped = unwrap_near_seed(
        solution.joints, seed, lower_limits_, upper_limits_);
      if (!unwrapped) {
        continue;
      }
      Candidate candidate;
      candidate.joints = *unwrapped;
      candidate.swivel = solution.swivel_angle;
      candidate.position_error = solution.position_error;
      candidate.orientation_error = solution.orientation_error;
      candidate.limit_margin = solution.minimum_joint_limit_margin;
      candidate.shoulder_branch = solution.shoulder_branch;
      candidate.elbow_branch = solution.elbow_branch;
      candidate.wrist_branch = solution.wrist_branch;
      const double psi_delta = std::abs(normalize_angle(
        candidate.swivel - reference_swivel));
      const double wrist_singularity = std::sin(candidate.joints[5]);
      const double wrist_singularity_penalty =
        0.01 / (wrist_singularity * wrist_singularity + 0.01);
      candidate.score = joint_distance_squared(candidate.joints, seed) +
        0.05 * psi_delta * psi_delta - 0.01 * candidate.limit_margin +
        wrist_singularity_penalty;
      candidates.push_back(candidate);
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
      return lhs.score < rhs.score;
    });
    return candidates;
  }

  bool check_state_collision(const JointVector& joints, std::string* reason)
  {
    const auto start = Clock::now();
    ++timing_.collision_checks;
    state_->setJointGroupPositions(joint_group_, joints.data());
    state_->update(true);
    if (!state_->satisfiesBounds(joint_group_)) {
      timing_.collision_time += Clock::now() - start;
      if (reason) {
        *reason = "joint_bounds";
      }
      return false;
    }

    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    request.group_name = group_name_;
    request.contacts = true;
    request.max_contacts = 8;
    request.max_contacts_per_pair = 1;
    scene_->checkCollision(request, result, *state_);
    timing_.collision_time += Clock::now() - start;
    if (!result.collision) {
      return true;
    }
    if (reason) {
      std::ostringstream output;
      output << "collision";
      size_t count = 0;
      for (const auto& [pair, contacts] : result.contacts) {
        if (contacts.empty()) {
          continue;
        }
        output << (count == 0 ? ":" : ",") << pair.first << "<->" << pair.second;
        if (++count >= 4) {
          break;
        }
      }
      *reason = output.str();
    }
    return false;
  }

  bool check_edge_collision(
    const JointVector& from,
    const JointVector& to,
    std::string* reason,
    size_t* checks,
    JointVector* colliding_joints)
  {
    const double maximum_delta = maximum_joint_delta(from, to);
    const size_t segments = std::max<size_t>(1, static_cast<size_t>(std::ceil(maximum_delta / edge_joint_step_)));
    for (size_t segment = 1; segment <= segments; ++segment) {
      const double ratio = static_cast<double>(segment) / static_cast<double>(segments);
      JointVector interpolated{};
      for (size_t index = 0; index < interpolated.size(); ++index) {
        interpolated[index] = from[index] + ratio * (to[index] - from[index]);
      }
      ++(*checks);
      std::string state_reason;
      if (!check_state_collision(interpolated, &state_reason)) {
        if (colliding_joints) {
          *colliding_joints = interpolated;
        }
        if (reason) {
          *reason = "edge@" + std::to_string(segment) + "/" + std::to_string(segments) + ":" + state_reason;
        }
        return false;
      }
    }
    return true;
  }

  EdgeCheckResult check_edge_continuity(
    const JointVector& from,
    const JointVector& to,
    const Eigen::Vector3d& from_target,
    const Eigen::Vector3d& to_target,
    const Eigen::Matrix3d& target_rotation,
    size_t* checks)
  {
    EdgeCheckResult result;
    const double maximum_delta = maximum_joint_delta(from, to);
    const size_t segments = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(maximum_delta / edge_joint_step_)));
    for (size_t segment = 1; segment <= segments; ++segment) {
      const double ratio = static_cast<double>(segment) / static_cast<double>(segments);
      JointVector interpolated{};
      for (size_t index = 0; index < interpolated.size(); ++index) {
        interpolated[index] = from[index] + ratio * (to[index] - from[index]);
      }

      ++(*checks);
      std::string state_reason;
      if (!check_state_collision(interpolated, &state_reason)) {
        result.collision = true;
        result.rejected_joints = interpolated;
        result.reason = "edge@" + std::to_string(segment) + "/" +
          std::to_string(segments) + ":" + state_reason;
        return result;
      }

      const auto fk_start = Clock::now();
      ++timing_.task_path_fk_checks;
      const Eigen::Isometry3d& actual = state_->getGlobalLinkTransform(target_link_);
      const Eigen::Vector3d expected_position =
        from_target + ratio * (to_target - from_target);
      const double position_deviation = (actual.translation() - expected_position).norm();
      const double orientation_deviation = rotation_distance(actual.linear(), target_rotation);
      timing_.task_path_fk_time += Clock::now() - fk_start;
      result.maximum_position_deviation = std::max(
        result.maximum_position_deviation, position_deviation);
      result.maximum_orientation_deviation = std::max(
        result.maximum_orientation_deviation, orientation_deviation);
      if (position_deviation > maximum_task_position_deviation_ + 1e-12 ||
          orientation_deviation > maximum_task_orientation_deviation_ + 1e-12) {
        result.task_path_deviation = true;
        result.rejected_joints = interpolated;
        std::ostringstream reason;
        reason << "task_path_deviation@" << segment << '/' << segments <<
          ":position=" << position_deviation <<
          ",orientation=" << orientation_deviation;
        result.reason = reason.str();
        return result;
      }
    }
    result.valid = true;
    return result;
  }

  SampleResult solve_sample_direct(
    size_t step_index,
    const Eigen::Vector3d& target,
    const Eigen::Matrix3d& target_rotation,
    const std::optional<JointVector>& previous_joints,
    const std::optional<double>& previous_swivel,
    const std::optional<Eigen::Vector3d>& previous_target,
    const Clock::time_point& deadline)
  {
    SampleResult result;
    result.step_index = step_index;
    result.target = target;
    const JointVector seed = previous_joints.value_or(initial_seed_);
    Eigen::Isometry3d target_in_base = Eigen::Isometry3d::Identity();
    target_in_base.translation() = target;
    target_in_base.linear() = target_rotation;
    const Eigen::Isometry3d target_in_arm_base = base_to_arm_base_.inverse() * target_in_base;
    const double reference_swivel = previous_swivel.value_or(initial_swivel_);
    bool found_analytic_candidate = false;
    bool rejected_by_collision = false;
    bool rejected_by_task_path = false;
    std::string rejected_collision_reason;
    std::string rejected_task_path_reason;
    std::optional<JointVector> rejected_collision_candidate;
    std::optional<JointVector> rejected_task_path_candidate;
    double rejected_collision_delta = std::numeric_limits<double>::infinity();
    double rejected_task_path_delta = std::numeric_limits<double>::infinity();
    std::optional<SampleResult> high_gain_result;
    double high_gain_score = std::numeric_limits<double>::infinity();
    const size_t high_gain_search_count = 1 + 2 * high_gain_swivel_neighbor_steps_;
    const std::vector<double> swivels = swivel_search_order(reference_swivel);
    for (size_t swivel_index = 0; swivel_index < swivels.size(); ++swivel_index) {
      if (Clock::now() >= deadline) {
        break;
      }
      const double swivel = swivels[swivel_index];
      const std::vector<Candidate> candidates = generate_candidates_for_swivel(
        target_in_arm_base, seed, swivel, reference_swivel, &result.analytic_attempts);
      found_analytic_candidate = found_analytic_candidate || !candidates.empty();
      for (const Candidate& candidate : candidates) {
        const double maximum_delta = previous_joints
          ? maximum_joint_delta(*previous_joints, candidate.joints)
          : 0.0;
        bool valid = false;
        EdgeCheckResult edge_result;
        if (previous_joints && previous_target) {
          edge_result = check_edge_continuity(
            *previous_joints, candidate.joints, *previous_target, target,
            target_rotation, &result.collision_checks);
          valid = edge_result.valid;
        } else {
          ++result.collision_checks;
          std::string state_reason;
          valid = check_state_collision(candidate.joints, &state_reason);
          if (!valid) {
            edge_result.collision = true;
            edge_result.reason = state_reason;
            edge_result.rejected_joints = candidate.joints;
          } else {
            const auto fk_start = Clock::now();
            ++timing_.task_path_fk_checks;
            const Eigen::Isometry3d& actual = state_->getGlobalLinkTransform(target_link_);
            const double position_deviation = (actual.translation() - target).norm();
            const double orientation_deviation = rotation_distance(
              actual.linear(), target_rotation);
            timing_.task_path_fk_time += Clock::now() - fk_start;
            edge_result.maximum_position_deviation = position_deviation;
            edge_result.maximum_orientation_deviation = orientation_deviation;
            if (position_deviation > maximum_task_position_deviation_ + 1e-12 ||
                orientation_deviation > maximum_task_orientation_deviation_ + 1e-12) {
              valid = false;
              edge_result.task_path_deviation = true;
              edge_result.rejected_joints = candidate.joints;
              std::ostringstream reason;
              reason << "initial_task_pose_deviation:position=" << position_deviation <<
                ",orientation=" << orientation_deviation;
              edge_result.reason = reason.str();
            }
          }
        }
        if (!valid) {
          rejected_by_collision = rejected_by_collision || edge_result.collision;
          rejected_by_task_path = rejected_by_task_path || edge_result.task_path_deviation;
          if (edge_result.collision && maximum_delta < rejected_collision_delta) {
            rejected_collision_delta = maximum_delta;
            rejected_collision_candidate = edge_result.rejected_joints;
            rejected_collision_reason = edge_result.reason;
          }
          if (edge_result.task_path_deviation && maximum_delta < rejected_task_path_delta) {
            rejected_task_path_delta = maximum_delta;
            rejected_task_path_candidate = edge_result.rejected_joints;
            rejected_task_path_reason = edge_result.reason;
          }
          continue;
        }

        SampleResult candidate_result;
        candidate_result.step_index = step_index;
        candidate_result.target = target;
        candidate_result.success = true;
        candidate_result.reason = maximum_delta > maximum_joint_delta_ + 1e-12
          ? "ok_high_joint_gain" : "ok";
        candidate_result.joints = candidate.joints;
        candidate_result.swivel = candidate.swivel;
        candidate_result.maximum_joint_delta_deg = rad_to_deg(maximum_delta);
        candidate_result.position_error = candidate.position_error;
        candidate_result.orientation_error = candidate.orientation_error;
        candidate_result.shoulder_branch = candidate.shoulder_branch;
        candidate_result.elbow_branch = candidate.elbow_branch;
        candidate_result.wrist_branch = candidate.wrist_branch;
        candidate_result.high_joint_gain = maximum_delta > maximum_joint_delta_ + 1e-12;
        candidate_result.analytic_attempts = result.analytic_attempts;
        candidate_result.collision_checks = result.collision_checks;
        if (!candidate_result.high_joint_gain) {
          return candidate_result;
        }
        if (candidate.score < high_gain_score) {
          high_gain_score = candidate.score;
          high_gain_result = candidate_result;
        }
      }
      if (high_gain_result && swivel_index + 1 >= high_gain_search_count) {
        ++timing_.high_joint_gain_edges;
        return *high_gain_result;
      }
    }

    if (high_gain_result) {
      ++timing_.high_joint_gain_edges;
      return *high_gain_result;
    }

    if (!found_analytic_candidate) {
      result.reason = "analytic_no_solution";
    } else if (rejected_by_collision) {
      result.reason = "all_candidate_edges_colliding:" + rejected_collision_reason;
      result.has_diagnostic_joints = rejected_collision_candidate.has_value();
      if (rejected_collision_candidate) {
        result.diagnostic_joints = *rejected_collision_candidate;
        result.diagnostic_joint_delta_deg = rad_to_deg(rejected_collision_delta);
        result.diagnostic_pose_kind = "colliding_interpolated_state";
      }
    } else if (rejected_by_task_path) {
      result.reason = "all_candidate_edges_deviate_from_task_path:" + rejected_task_path_reason;
      result.has_diagnostic_joints = rejected_task_path_candidate.has_value();
      if (rejected_task_path_candidate) {
        result.diagnostic_joints = *rejected_task_path_candidate;
        result.diagnostic_joint_delta_deg = rad_to_deg(rejected_task_path_delta);
        result.diagnostic_pose_kind = "task_path_deviation_state";
      }
    } else {
      result.reason = "no_continuous_candidate";
    }
    return result;
  }

  SampleResult solve_sample_with_refinement(
    size_t step_index,
    const Eigen::Vector3d& target,
    const Eigen::Matrix3d& target_rotation,
    const std::optional<JointVector>& previous_joints,
    const std::optional<double>& previous_swivel,
    const std::optional<Eigen::Vector3d>& previous_target,
    size_t depth,
    const Clock::time_point& deadline)
  {
    SampleResult direct = solve_sample_direct(
      step_index, target, target_rotation, previous_joints, previous_swivel, previous_target,
      deadline);
    const Clock::time_point refinement_deadline = depth == 0
      ? Clock::now() + refinement_time_budget_
      : deadline;
    if (direct.success || !previous_joints || !previous_swivel || !previous_target ||
        direct.reason == "analytic_no_solution" || depth >= refinement_max_depth_ ||
        (target - *previous_target).norm() <= refinement_min_step_ + 1e-12 ||
        Clock::now() >= refinement_deadline) {
      return direct;
    }
    if (depth == 0) {
      ++timing_.refinement_attempts;
    }

    const Eigen::Vector3d midpoint = 0.5 * (*previous_target + target);
    SampleResult first = solve_sample_with_refinement(
      step_index, midpoint, target_rotation, previous_joints, previous_swivel,
      previous_target, depth + 1, refinement_deadline);
    if (!first.success || Clock::now() >= refinement_deadline) {
      if (depth == 0 && Clock::now() >= refinement_deadline) {
        ++timing_.refinement_budget_exhaustions;
      }
      return direct;
    }
    SampleResult second = solve_sample_with_refinement(
      step_index, target, target_rotation, first.joints, first.swivel,
      midpoint, depth + 1, refinement_deadline);
    if (!second.success) {
      if (depth == 0 && Clock::now() >= refinement_deadline) {
        ++timing_.refinement_budget_exhaustions;
      }
      return direct;
    }

    second.analytic_attempts += first.analytic_attempts + direct.analytic_attempts;
    second.collision_checks += first.collision_checks + direct.collision_checks;
    second.maximum_joint_delta_deg = std::max(
      first.maximum_joint_delta_deg, second.maximum_joint_delta_deg);
    second.high_joint_gain = first.high_joint_gain || second.high_joint_gain;
    second.refined_transition = true;
    second.reason = second.high_joint_gain ? "ok_refined_high_joint_gain" : "ok_refined";
    return second;
  }

  SampleResult solve_sample(
    size_t step_index,
    const Eigen::Vector3d& target,
    const Eigen::Matrix3d& target_rotation,
    const std::optional<JointVector>& previous_joints,
    const std::optional<double>& previous_swivel,
    const std::optional<Eigen::Vector3d>& previous_target)
  {
    SampleResult result = solve_sample_with_refinement(
      step_index, target, target_rotation, previous_joints, previous_swivel,
      previous_target, 0, Clock::time_point::max());
    if (result.refined_transition) {
      ++timing_.refined_transitions;
    }
    return result;
  }

  static nlohmann::json joints_json(const JointVector& joints)
  {
    return std::vector<double>(joints.begin(), joints.end());
  }

  void write_json(
    const std::vector<LineResult>& lines,
    const std::unordered_map<std::string, size_t>& failure_reasons,
    size_t path_point_count,
    double total_ms) const
  {
    nlohmann::json output;
    output["schema"] = "alfa.v3_continuous_reachability.v2";
    output["config"] = {
      {"side", side_},
      {"analytic_model", "V3.1.1"},
      {"frame", "world"},
      {"front_axis", "world +X"},
      {"test_pattern", test_pattern_},
      {"x_start_min", x_start_min_},
      {"x_start_max", x_start_max_},
      {"x_start_step", x_start_step_},
      {"travel", travel_},
      {"travel_vector", {travel_x_, travel_y_, travel_z_}},
      {"evaluation_mode", test_pattern_ == "planar_disk"
        ? "planar_disk_continuity"
        : (path_point_count == 1 ? "point_reachability" : "continuous_translation")},
      {"path_step", path_step_},
      {"path_point_count", path_point_count},
      {"disk_plane", "world_YZ"},
      {"disk_radius", disk_radius_},
      {"disk_ring_step", disk_ring_step_},
      {"lateral_min", lateral_min_},
      {"lateral_max", lateral_max_},
      {"lateral_step", lateral_step_},
      {"height_min", height_min_},
      {"height_max", height_max_},
      {"height_step", height_step_},
      {"target_orientation_rpy", target_orientation_rpy_},
      {"arm_mount_forward_offset", arm_mount_forward_offset_},
      {"initial_seed", joints_json(initial_seed_)},
      {"swivel_step_deg", rad_to_deg(swivel_step_)},
      {"maximum_joint_delta_deg", rad_to_deg(maximum_joint_delta_)},
      {"edge_joint_step_deg", rad_to_deg(edge_joint_step_)},
      {"maximum_task_position_deviation", maximum_task_position_deviation_},
      {"maximum_task_orientation_deviation_deg", rad_to_deg(
          maximum_task_orientation_deviation_)},
      {"high_gain_swivel_neighbor_steps", high_gain_swivel_neighbor_steps_},
      {"refinement_max_depth", refinement_max_depth_},
      {"refinement_min_step", refinement_min_step_},
      {"refinement_time_budget_ms", std::chrono::duration<double, std::milli>(
          refinement_time_budget_).count()},
      {"joint_delta_semantics", "quality_marker_not_continuity_failure"},
      {"collision_scope", ignore_opposite_arm_
        ? "tested_arm_self_and_body_opposite_arm_ignored"
        : "full_robot"},
      {"record_failure_states", record_failure_states_},
      {"record_failure_paths", record_failure_paths_},
    };

    const size_t complete_count = static_cast<size_t>(std::count_if(
      lines.begin(), lines.end(), [](const LineResult& line) {return line.complete;}));
    output["summary"] = {
      {"start_point_count", lines.size()},
      {"complete_count", complete_count},
      {"failed_count", lines.size() - complete_count},
      {"complete_ratio", lines.empty() ? 0.0 : static_cast<double>(complete_count) / lines.size()},
      {"total_ms", total_ms},
      {"analytic_calls", timing_.analytic_calls},
      {"analytic_solutions", timing_.analytic_solutions},
      {"analytic_ms", std::chrono::duration<double, std::milli>(timing_.analytic_time).count()},
      {"collision_checks", timing_.collision_checks},
      {"collision_ms", std::chrono::duration<double, std::milli>(timing_.collision_time).count()},
      {"task_path_fk_checks", timing_.task_path_fk_checks},
      {"task_path_fk_ms", std::chrono::duration<double, std::milli>(
          timing_.task_path_fk_time).count()},
      {"high_joint_gain_edges", timing_.high_joint_gain_edges},
      {"refinement_attempts", timing_.refinement_attempts},
      {"refinement_budget_exhaustions", timing_.refinement_budget_exhaustions},
      {"refined_transitions", timing_.refined_transitions},
    };

    output["failure_reasons"] = nlohmann::json::object();
    for (const auto& [reason, count] : failure_reasons) {
      output["failure_reasons"][reason] = count;
    }

    output["successful_start_points"] = nlohmann::json::array();
    output["failed_start_points"] = nlohmann::json::array();
    for (const LineResult& line : lines) {
      if (!line.complete) {
        nlohmann::json failed_point = {
          {"point_index", line.line_index},
          {"position", {line.start_x, line.lateral, line.height}},
          {"failure_reason", line.failure_reason},
          {"failure_step_index", line.failure_step_index},
          {"reached_point_count", line.reached_point_count},
          {"elapsed_ms", line.elapsed_ms},
        };
        if (record_failure_states_) {
          failed_point["failure_target_position"] = {
            line.failure_target.x(), line.failure_target.y(), line.failure_target.z()};
          if (line.has_last_valid_joints) {
            failed_point["last_valid_joints"] = joints_json(line.end_joints);
            failed_point["last_valid_position"] = {
              line.last_valid_target.x(), line.last_valid_target.y(), line.last_valid_target.z()};
          }
          if (line.has_diagnostic_joints) {
            failed_point["diagnostic_joints"] = joints_json(line.diagnostic_joints);
            failed_point["diagnostic_joint_delta_deg"] = line.diagnostic_joint_delta_deg;
            failed_point["diagnostic_pose_kind"] = line.diagnostic_pose_kind;
            if (line.diagnostic_edge_collision_checked) {
              failed_point["diagnostic_edge_collision_checked"] = true;
              failed_point["diagnostic_edge_collision_free"] =
                line.diagnostic_edge_collision_free;
              failed_point["diagnostic_edge_collision_reason"] =
                line.diagnostic_edge_collision_reason;
            }
          }
        }
        if (record_failure_paths_) {
          failed_point["path_joint_samples"] = nlohmann::json::array();
          for (const JointVector& joints : line.path_joint_samples) {
            failed_point["path_joint_samples"].push_back(joints_json(joints));
          }
          failed_point["path_target_samples"] = nlohmann::json::array();
          for (const Eigen::Vector3d& target : line.path_target_samples) {
            failed_point["path_target_samples"].push_back(
              {target.x(), target.y(), target.z()});
          }
        }
        output["failed_start_points"].push_back(std::move(failed_point));
        continue;
      }
      output["successful_start_points"].push_back({
        {"point_index", line.line_index},
        {"position", {line.start_x, line.lateral, line.height}},
        {"start_joints", joints_json(line.start_joints)},
        {"end_joints", joints_json(line.end_joints)},
        {"maximum_path_joint_delta_deg", line.maximum_path_joint_delta_deg},
        {"high_joint_gain_edge_count", line.high_joint_gain_edge_count},
        {"refined_transition_count", line.refined_transition_count},
        {"elapsed_ms", line.elapsed_ms},
      });
    }

    const std::filesystem::path output_path(output_json_);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream stream(output_path);
    if (!stream) {
      throw std::runtime_error("failed to open output: " + output_json_);
    }
    stream << std::setw(2) << output << '\n';
  }

  std::string side_;
  double arm_mount_forward_offset_ = 0.0;
  std::string test_pattern_;
  std::string group_name_;
  std::string arm_base_link_;
  std::string target_link_;
  double x_start_min_ = 0.35;
  double x_start_max_ = 0.75;
  double x_start_step_ = 0.01;
  double travel_ = 0.40;
  double travel_x_ = 0.0;
  double travel_y_ = 0.0;
  double travel_z_ = 0.0;
  double path_step_ = 0.01;
  double disk_radius_ = 0.15;
  double disk_ring_step_ = 0.03;
  double lateral_min_ = -0.55;
  double lateral_max_ = 0.85;
  double lateral_step_ = 0.05;
  double height_min_ = 0.05;
  double height_max_ = 1.20;
  double height_step_ = 0.05;
  std::vector<double> target_orientation_rpy_;
  JointVector initial_seed_{};
  JointVector lower_limits_{};
  JointVector upper_limits_{};
  double initial_swivel_ = 0.0;
  double swivel_step_ = deg_to_rad(5.0);
  double maximum_joint_delta_ = deg_to_rad(10.0);
  double edge_joint_step_ = deg_to_rad(2.5);
  double maximum_task_position_deviation_ = 0.001;
  double maximum_task_orientation_deviation_ = deg_to_rad(1.0);
  size_t high_gain_swivel_neighbor_steps_ = 2;
  size_t refinement_max_depth_ = 3;
  double refinement_min_step_ = 0.00125;
  Clock::duration refinement_time_budget_ = std::chrono::milliseconds(2);
  bool ignore_opposite_arm_ = true;
  bool record_failure_states_ = false;
  bool record_failure_paths_ = false;
  std::string output_json_;
  size_t progress_period_ = 500;
  TimingCounters timing_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<moveit::core::RobotState> state_;
  Eigen::Isometry3d base_to_arm_base_ = Eigen::Isometry3d::Identity();
  V3RedundantArmAnalyticIk solver_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<V3ContinuousReachabilityNode>();
    node->run();
  } catch (const std::exception& error) {
    std::cerr << "v3_continuous_reachability failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
