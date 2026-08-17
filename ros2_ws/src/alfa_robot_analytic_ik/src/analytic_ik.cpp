#include "alfa_robot_analytic_ik/analytic_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace alfa_robot::analytic_ik
{
namespace
{

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr std::array<double, 6> kJointLimits = {
  1.57079632679,
  1.57079632679,
  2.44346095279,
  3.14159265359,
  2.18166156499,
  3.12413936107,
};
constexpr double kBaseToUpdownX = -0.009816;
constexpr double kBaseToUpdownZ = 0.392;
constexpr double kArmMountX = 0.095;
constexpr double kArmMountY = 0.26;
constexpr double kArmMountZ = 0.2;
constexpr double kJoint2YAbs = 0.0825;
constexpr double kParallelPlaneOffsetYAbs = 0.1265;
constexpr double kJoint2Z = 0.058;
constexpr double kLink2 = 0.4;
constexpr double kLink3 = 0.34;
constexpr double kJoint5Y = 0.076;
constexpr double kJoint5ZAbs = 0.058;
constexpr double kJoint6X = 0.0825;
constexpr double kJoint6Z = -0.058;
constexpr double kTool0Z = 0.259;

double sideSign(ArmSide side)
{
  return side == ArmSide::Left ? 1.0 : -1.0;
}

Eigen::Matrix3d rotX(double value)
{
  return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d rotY(double value)
{
  return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

Eigen::Matrix3d rotZ(double value)
{
  return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Isometry3d translate(double x, double y, double z)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(x, y, z);
  return out;
}

Eigen::Isometry3d rotate(const Eigen::Matrix3d& rotation)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = rotation;
  return out;
}

bool withinLimit(double value, double limit, double eps = 1e-7)
{
  return value >= -limit - eps && value <= limit + eps;
}

double solutionSeedDistance(
  const std::array<double, 6>& solution,
  const std::array<double, 6>& seed)
{
  double sum = 0.0;
  for (size_t i = 0; i < solution.size(); ++i) {
    const double delta = solution[i] - seed[i];
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

bool similarSolution(
  const std::array<double, 6>& lhs,
  const std::array<double, 6>& rhs)
{
  double sum = 0.0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    const double delta = shortestAngularDistance(lhs[i], rhs[i]);
    sum += delta * delta;
  }
  return sum < 1e-10;
}

struct OrientationBranch
{
  double a = 0.0;
  double q5 = 0.0;
  double q6 = 0.0;
};

std::optional<OrientationBranch> orientationBranchForQ1(
  const Eigen::Matrix3d& target_rotation,
  double q1,
  double cos_sign,
  double q6_seed)
{
  const Eigen::Matrix3d basis = rotZ(q1) * rotX(-kPi / 2.0);
  const Eigen::Matrix3d local = basis.transpose() * target_rotation;
  const double s5 = std::clamp(-local(2, 2), -1.0, 1.0);
  double q5 = std::asin(s5);
  if (cos_sign < 0.0) {
    q5 = normalizeAngle(kPi - q5);
  }
  const double c5 = std::cos(q5);
  if (std::abs(c5) < 1e-8) {
    OrientationBranch branch;
    branch.q5 = s5 >= 0.0 ? kPi / 2.0 : -kPi / 2.0;
    branch.q6 = normalizeAngle(q6_seed);
    if (s5 >= 0.0) {
      const double coupled_angle = std::atan2(local(0, 0), -local(0, 1));
      branch.a = normalizeAngle(coupled_angle + branch.q6);
    } else {
      const double coupled_angle = std::atan2(local(0, 0), local(0, 1));
      branch.a = normalizeAngle(coupled_angle - branch.q6);
    }
    return branch;
  }

  OrientationBranch branch;
  branch.q5 = normalizeAngle(q5);
  branch.a = normalizeAngle(std::atan2(local(1, 2) / c5, local(0, 2) / c5));
  branch.q6 = normalizeAngle(std::atan2(-local(2, 0) / c5, -local(2, 1) / c5));

  const Eigen::Matrix3d reconstructed =
    rotZ(q1) * rotX(-kPi / 2.0) *
    rotZ(branch.a) * rotX(-kPi / 2.0) *
    rotZ(branch.q5) * rotY(kPi / 2.0) * rotZ(branch.q6);
  if ((reconstructed - target_rotation).norm() > 1e-6) {
    return std::nullopt;
  }
  return branch;
}

Eigen::Vector3d joint4ToToolOffsetInJoint4Frame(
  ArmSide side,
  double q5,
  double q6)
{
  const double z5 = sideSign(side) * kJoint5ZAbs;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform =
    transform *
    translate(0.0, kJoint5Y, z5) *
    rotate(rotZ(kPi)) *
    rotate(rotX(-kPi / 2.0)) *
    rotate(rotZ(q5)) *
    translate(kJoint6X, 0.0, kJoint6Z) *
    rotate(rotY(kPi / 2.0)) *
    rotate(rotZ(q6)) *
    translate(0.0, 0.0, kTool0Z);
  return transform.translation();
}

Eigen::Vector3d joint4TargetPosition(
  ArmSide side,
  const Eigen::Isometry3d& target,
  double q1,
  const OrientationBranch& branch)
{
  const Eigen::Matrix3d after_joint4_rotation =
    rotZ(q1) * rotX(-kPi / 2.0) * rotZ(branch.a + kPi);
  return target.translation() -
         after_joint4_rotation *
         joint4ToToolOffsetInJoint4Frame(side, branch.q5, branch.q6);
}

std::vector<double> solveQ1ClosedForm(
  ArmSide side,
  const Eigen::Isometry3d& target)
{
  std::vector<double> roots;
  const Eigen::Vector3d& position = target.translation();
  const Eigen::Vector3d tool_normal = target.linear().col(2);
  const double tool_reach = kJoint6X + kTool0Z;
  const double coefficient_sin = -position.x() + tool_reach * tool_normal.x();
  const double coefficient_cos = position.y() - tool_reach * tool_normal.y();
  const double constraint =
    sideSign(side) * (kParallelPlaneOffsetYAbs + kJoint5ZAbs);
  const double radius = std::hypot(coefficient_sin, coefficient_cos);
  if (radius < 1e-12 || std::abs(constraint) > radius + 1e-10) {
    return roots;
  }

  const double ratio = std::clamp(constraint / radius, -1.0, 1.0);
  const double phase = std::atan2(coefficient_cos, coefficient_sin);
  const double principal = std::asin(ratio);
  roots.push_back(normalizeAngle(principal - phase));
  roots.push_back(normalizeAngle(kPi - principal - phase));

  std::sort(roots.begin(), roots.end());
  roots.erase(
    std::unique(
      roots.begin(), roots.end(),
      [](double lhs, double rhs) { return std::abs(lhs - rhs) < 1e-6; }),
    roots.end());
  return roots;
}

std::vector<std::array<double, 6>> solvePlanarElbow(
  ArmSide side,
  const Eigen::Isometry3d& target,
  double q1,
  const OrientationBranch& branch)
{
  std::vector<std::array<double, 6>> out;
  const Eigen::Vector3d joint4 = joint4TargetPosition(side, target, q1, branch);
  const Eigen::Vector3d local = rotZ(-q1) * joint4;
  const double plane_error = std::abs(local.y() - sideSign(side) * kParallelPlaneOffsetYAbs);
  if (plane_error > 1e-4) {
    return out;
  }

  const double u = local.x();
  const double z = local.z() - kJoint2Z;
  const double d2 = u * u + z * z;
  double cos_q3 = (d2 - kLink2 * kLink2 - kLink3 * kLink3) / (2.0 * kLink2 * kLink3);
  if (cos_q3 < -1.0 - 1e-7 || cos_q3 > 1.0 + 1e-7) {
    return out;
  }
  cos_q3 = std::clamp(cos_q3, -1.0, 1.0);
  const double q3_abs = std::acos(cos_q3);
  for (double q3 : {q3_abs, -q3_abs}) {
    const double q2 =
      std::atan2(u, z) -
      std::atan2(kLink3 * std::sin(q3), kLink2 + kLink3 * std::cos(q3));
    const double q4 = normalizeAngle(branch.a - q2 - q3);
    std::array<double, 6> joints = {
      normalizeAngle(q1),
      normalizeAngle(q2),
      normalizeAngle(q3),
      q4,
      normalizeAngle(branch.q5),
      normalizeAngle(branch.q6),
    };
    bool in_limits = true;
    for (size_t i = 0; i < joints.size(); ++i) {
      in_limits = in_limits && withinLimit(joints[i], kJointLimits[i]);
    }
    if (in_limits) {
      out.push_back(joints);
    }
  }
  return out;
}

double orientationError(const Eigen::Matrix3d& target, const Eigen::Matrix3d& actual)
{
  Eigen::AngleAxisd aa(target.transpose() * actual);
  return std::abs(aa.angle());
}

}  // namespace

double normalizeAngle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double shortestAngularDistance(double lhs, double rhs)
{
  return std::abs(normalizeAngle(lhs - rhs));
}

std::string ThreeParallelArmAnalyticIk::sideName(ArmSide side)
{
  return side == ArmSide::Left ? "left" : "right";
}

Eigen::Isometry3d ThreeParallelArmAnalyticIk::baseLinkToUpdown(double updown)
{
  return translate(kBaseToUpdownX, 0.0, kBaseToUpdownZ + updown);
}

Eigen::Isometry3d ThreeParallelArmAnalyticIk::updownToArmBase(ArmSide side)
{
  return translate(kArmMountX, sideSign(side) * kArmMountY, kArmMountZ);
}

Eigen::Isometry3d ThreeParallelArmAnalyticIk::baseLinkToArmBase(
  ArmSide side,
  double updown)
{
  return baseLinkToUpdown(updown) * updownToArmBase(side);
}

Eigen::Isometry3d ThreeParallelArmAnalyticIk::forwardInArmBase(
  ArmSide side,
  const std::array<double, 6>& joints) const
{
  const double q1 = joints[0];
  const double q2 = joints[1];
  const double q3 = joints[2];
  const double q4 = joints[3];
  const double q5 = joints[4];
  const double q6 = joints[5];
  const double sign = sideSign(side);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform =
    transform *
    rotate(rotZ(q1)) *
    translate(0.0, sign * kJoint2YAbs, kJoint2Z) *
    rotate(rotX(-kPi / 2.0)) *
    rotate(rotZ(q2)) *
    translate(0.0, -kLink2, sign * 0.008) *
    rotate(rotZ(q3)) *
    translate(0.0, -kLink3, sign * 0.036) *
    rotate(rotZ(kPi + q4)) *
    translate(0.0, kJoint5Y, sign * kJoint5ZAbs) *
    rotate(rotZ(kPi)) *
    rotate(rotX(-kPi / 2.0)) *
    rotate(rotZ(q5)) *
    translate(kJoint6X, 0.0, kJoint6Z) *
    rotate(rotY(kPi / 2.0)) *
    rotate(rotZ(q6)) *
    translate(0.0, 0.0, kTool0Z);
  return transform;
}

std::vector<ArmAnalyticIkSolution> ThreeParallelArmAnalyticIk::solveInArmBase(
  const ArmAnalyticIkRequest& request) const
{
  std::vector<ArmAnalyticIkSolution> solutions;
  const auto q1_roots = solveQ1ClosedForm(request.side, request.target_in_arm_base);

  for (double cos_sign : {1.0, -1.0}) {
    for (double q1 : q1_roots) {
      const auto branch = orientationBranchForQ1(
        request.target_in_arm_base.linear(), q1, cos_sign, request.seed[5]);
      if (!branch) {
        continue;
      }
      for (const auto& joints : solvePlanarElbow(request.side, request.target_in_arm_base, q1, *branch)) {
        const Eigen::Isometry3d actual = forwardInArmBase(request.side, joints);
        const double position_error =
          (actual.translation() - request.target_in_arm_base.translation()).norm();
        const double rot_error =
          orientationError(request.target_in_arm_base.linear(), actual.linear());
        if (position_error > request.position_tolerance ||
            rot_error > request.orientation_tolerance) {
          continue;
        }
        const bool duplicate = std::any_of(
          solutions.begin(), solutions.end(),
          [&](const ArmAnalyticIkSolution& kept) {
            return similarSolution(kept.joints, joints);
          });
        if (duplicate) {
          continue;
        }
        ArmAnalyticIkSolution solution;
        solution.joints = joints;
        solution.position_error = position_error;
        solution.orientation_error = rot_error;
        solution.seed_distance = solutionSeedDistance(joints, request.seed);
        solutions.push_back(solution);
      }
    }
  }

  std::sort(
    solutions.begin(), solutions.end(),
    [](const ArmAnalyticIkSolution& lhs, const ArmAnalyticIkSolution& rhs) {
      if (lhs.seed_distance != rhs.seed_distance) {
        return lhs.seed_distance < rhs.seed_distance;
      }
      if (lhs.position_error != rhs.position_error) {
        return lhs.position_error < rhs.position_error;
      }
      return lhs.orientation_error < rhs.orientation_error;
    });
  return solutions;
}

std::vector<ArmAnalyticIkSolution> ThreeParallelArmAnalyticIk::solveInBaseLink(
  ArmSide side,
  const Eigen::Isometry3d& target_in_base_link,
  double updown,
  const std::array<double, 6>& seed,
  double position_tolerance,
  double orientation_tolerance,
  size_t root_samples) const
{
  ArmAnalyticIkRequest request;
  request.side = side;
  request.target_in_arm_base = baseLinkToArmBase(side, updown).inverse() * target_in_base_link;
  request.seed = seed;
  request.position_tolerance = position_tolerance;
  request.orientation_tolerance = orientation_tolerance;
  request.root_samples = root_samples;
  return solveInArmBase(request);
}

}  // namespace alfa_robot::analytic_ik
