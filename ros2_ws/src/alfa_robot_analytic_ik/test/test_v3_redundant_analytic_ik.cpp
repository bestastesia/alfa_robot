#include "alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp"

#include "alfa_robot_analytic_ik/analytic_ik.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

namespace
{

constexpr double kPi = 3.1415926535897932384626433832795;

void require(bool condition, const std::string& message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

double randomBetween(std::mt19937& rng, double lower, double upper)
{
  std::uniform_real_distribution<double> distribution(lower, upper);
  return distribution(rng);
}

double orientationError(
  const Eigen::Matrix3d& target,
  const Eigen::Matrix3d& actual)
{
  return std::abs(Eigen::AngleAxisd(target.transpose() * actual).angle());
}

bool containsOriginalBranch(
  const std::vector<alfa_robot::analytic_ik::V3RedundantIkSolution>& solutions,
  const std::array<double, 7>& expected)
{
  return std::any_of(
    solutions.begin(), solutions.end(),
    [&](const alfa_robot::analytic_ik::V3RedundantIkSolution& solution) {
      double maximum_delta = 0.0;
      for (size_t index = 0; index < expected.size(); ++index) {
        maximum_delta = std::max(
          maximum_delta,
          std::abs(alfa_robot::analytic_ik::normalizeAngle(
            solution.joints[index] - expected[index])));
      }
      return maximum_delta < 1e-5;
    });
}

void checkGeometry()
{
  using Solver = alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
  require(
    (Solver::shoulderCenterInArmBase() - Eigen::Vector3d(0.0, 0.0, 0.117)).norm() < 1e-8,
    "shoulder center does not match V3.0.4 geometry");
  require(
    std::abs(Solver::upperArmLength() - 0.518) < 5e-9,
    "upper arm length does not match V3.0.4 geometry");
  require(
    std::abs(Solver::forearmLength() - 0.477) < 5e-9,
    "forearm length does not match V3.0.4 geometry");
}

void checkRandomFkIkRegression()
{
  using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
  using alfa_robot::analytic_ik::V3RedundantIkRequest;

  V3RedundantArmAnalyticIk solver;
  const auto lower = solver.jointLowerLimits();
  const auto upper = solver.jointUpperLimits();
  std::mt19937 rng(20260815);
  double maximum_position_error = 0.0;
  double maximum_orientation_error = 0.0;
  double solve_microseconds = 0.0;
  size_t original_branch_recovered = 0;
  constexpr size_t kSamples = 10000;

  const auto start = std::chrono::steady_clock::now();
  for (size_t sample = 0; sample < kSamples; ++sample) {
    std::array<double, 7> joints{};
    for (size_t index = 0; index < joints.size(); ++index) {
      joints[index] = randomBetween(
        rng, 0.9 * lower[index], 0.9 * upper[index]);
    }
    const Eigen::Isometry3d target = solver.forwardInArmBase(joints);
    V3RedundantIkRequest request;
    request.target_in_arm_base = target;
    request.swivel_angle = solver.swivelAngle(joints);
    request.seed = joints;
    request.position_tolerance = 1e-7;
    request.orientation_tolerance = 1e-7;
    const auto solve_start = std::chrono::steady_clock::now();
    const auto solutions = solver.solveInArmBase(request);
    solve_microseconds += std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - solve_start).count();
    if (solutions.empty()) {
      V3RedundantIkRequest diagnostic = request;
      diagnostic.position_tolerance = 1.0;
      diagnostic.orientation_tolerance = 1.0;
      diagnostic.enforce_joint_limits = false;
      const auto diagnostic_solutions = solver.solveInArmBase(diagnostic);
      if (!diagnostic_solutions.empty()) {
        std::cerr << std::setprecision(17)
                  << "diagnostic_position_error="
                  << diagnostic_solutions.front().position_error
                  << " diagnostic_orientation_error="
                  << diagnostic_solutions.front().orientation_error << '\n';
      }
    }
    require(
      !solutions.empty(),
      "reachable FK target returned no analytic IK solution at sample=" +
      std::to_string(sample) + " swivel=" + std::to_string(request.swivel_angle) +
      " joints=" + std::to_string(joints[0]) + "," +
      std::to_string(joints[1]) + "," + std::to_string(joints[2]) + "," +
      std::to_string(joints[3]) + "," + std::to_string(joints[4]) + "," +
      std::to_string(joints[5]) + "," + std::to_string(joints[6]));
    original_branch_recovered += containsOriginalBranch(solutions, joints) ? 1 : 0;
    for (const auto& solution : solutions) {
      const Eigen::Isometry3d actual = solver.forwardInArmBase(solution.joints);
      const double position_error =
        (target.translation() - actual.translation()).norm();
      const double rotation_error = orientationError(target.linear(), actual.linear());
      maximum_position_error = std::max(maximum_position_error, position_error);
      maximum_orientation_error = std::max(maximum_orientation_error, rotation_error);
      require(position_error < 1e-7, "analytic IK position FK residual exceeded tolerance");
      require(rotation_error < 1e-7, "analytic IK orientation FK residual exceeded tolerance");
    }
  }
  const double elapsed_microseconds =
    std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - start).count();
  std::cout << std::setprecision(12)
            << "random_samples=" << kSamples
            << " max_position_error_m=" << maximum_position_error
            << " max_orientation_error_rad=" << maximum_orientation_error
            << " original_branch_recovered=" << original_branch_recovered
            << " average_ik_us=" << solve_microseconds / kSamples
            << " average_fk_psi_ik_us=" << elapsed_microseconds / kSamples
            << '\n';
  require(
    original_branch_recovered >= kSamples - 50,
    "analytic IK did not recover enough original FK branches: " +
    std::to_string(original_branch_recovered));
}

void checkSwivelFamily()
{
  using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
  using alfa_robot::analytic_ik::V3RedundantIkRequest;

  V3RedundantArmAnalyticIk solver;
  const std::array<double, 7> source = {
    0.35, -0.75, 0.62, 1.1, -0.45, 0.8, 0.25};
  const Eigen::Isometry3d target = solver.forwardInArmBase(source);
  Eigen::Vector3d previous_elbow = Eigen::Vector3d::Zero();
  bool observed_elbow_motion = false;
  for (size_t index = 0; index <= 72; ++index) {
    V3RedundantIkRequest request;
    request.target_in_arm_base = target;
    request.swivel_angle = -kPi + 2.0 * kPi * index / 72.0;
    request.seed = source;
    request.position_tolerance = 1e-7;
    request.orientation_tolerance = 1e-7;
    request.enforce_joint_limits = false;
    const auto solutions = solver.solveInArmBase(request);
    require(!solutions.empty(), "swivel family unexpectedly lost all analytic branches");
    const Eigen::Isometry3d actual = solver.forwardInArmBase(solutions.front().joints);
    require(
      (actual.translation() - target.translation()).norm() < 1e-7,
      "swivel family changed target position");
    require(
      orientationError(actual.linear(), target.linear()) < 1e-7,
      "swivel family changed target orientation");
    const Eigen::Vector3d elbow =
      solver.elbowPositionInArmBase(solutions.front().joints);
    if (index > 0 && (elbow - previous_elbow).norm() > 1e-3) {
      observed_elbow_motion = true;
    }
    previous_elbow = elbow;
  }
  require(observed_elbow_motion, "swivel parameter did not move the elbow");
}

void checkInstalledMirroredModels()
{
  using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
  using alfa_robot::analytic_ik::V3RedundantArmModel;
  using alfa_robot::analytic_ik::V3RedundantIkRequest;

  std::mt19937 rng(20260818);
  for (const V3RedundantArmModel model : {
      V3RedundantArmModel::V305Left,
      V3RedundantArmModel::V305Right,
      V3RedundantArmModel::V306Left,
      V3RedundantArmModel::V306Right,
      V3RedundantArmModel::V307Left,
      V3RedundantArmModel::V307Right,
      V3RedundantArmModel::V308Left,
      V3RedundantArmModel::V308Right,
      V3RedundantArmModel::V309Left,
      V3RedundantArmModel::V309Right,
      V3RedundantArmModel::V311Left,
      V3RedundantArmModel::V311Right}) {
    V3RedundantArmAnalyticIk solver(model);
    const auto lower = solver.jointLowerLimits();
    const auto upper = solver.jointUpperLimits();
    for (size_t sample = 0; sample < 1000; ++sample) {
      std::array<double, 7> joints{};
      for (size_t index = 0; index < joints.size(); ++index) {
        joints[index] = randomBetween(
          rng, 0.85 * lower[index], 0.85 * upper[index]);
      }
      V3RedundantIkRequest request;
      request.target_in_arm_base = solver.forwardInArmBase(joints);
      request.swivel_angle = solver.swivelAngle(joints);
      request.seed = joints;
      const auto solutions = solver.solveInArmBase(request);
      require(!solutions.empty(), "installed mirrored model lost a reachable FK target");
      for (const auto& solution : solutions) {
        const Eigen::Isometry3d actual = solver.forwardInArmBase(solution.joints);
        require(
          (actual.translation() - request.target_in_arm_base.translation()).norm() < 1e-7,
          "installed mirrored model position residual exceeded tolerance");
        require(
          orientationError(actual.linear(), request.target_in_arm_base.linear()) < 1e-7,
          "installed mirrored model orientation residual exceeded tolerance");
      }
    }
  }
}

void checkSingularAndUnreachableTargets()
{
  using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
  using alfa_robot::analytic_ik::V3RedundantIkRequest;

  V3RedundantArmAnalyticIk solver;
  for (const std::array<double, 7>& joints : {
      std::array<double, 7>{0.4, 0.0, -0.7, 0.9, 0.2, 0.0, -0.3},
      std::array<double, 7>{-0.2, 0.6, 0.8, 0.0, -0.5, 0.7, 0.4}}) {
    V3RedundantIkRequest request;
    request.target_in_arm_base = solver.forwardInArmBase(joints);
    request.swivel_angle = solver.swivelAngle(joints);
    request.seed = joints;
    request.position_tolerance = 1e-7;
    request.orientation_tolerance = 1e-7;
    require(
      !solver.solveInArmBase(request).empty(),
      "singular reachable target returned no analytic branch");
  }

  V3RedundantIkRequest unreachable;
  unreachable.target_in_arm_base.translation() = Eigen::Vector3d(0.0, 0.0, 2.0);
  require(
    solver.solveInArmBase(unreachable).empty(),
    "unreachable target returned an analytic branch");
}

}  // namespace

int main()
{
  checkGeometry();
  checkRandomFkIkRegression();
  checkSwivelFamily();
  checkInstalledMirroredModels();
  checkSingularAndUnreachableTargets();
  return 0;
}
