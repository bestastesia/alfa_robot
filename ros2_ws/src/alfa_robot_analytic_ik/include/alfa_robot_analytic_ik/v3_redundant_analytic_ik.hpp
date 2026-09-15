#pragma once

#include <Eigen/Geometry>

#include <array>
#include <vector>

namespace alfa_robot::analytic_ik
{

enum class V3RedundantArmModel
{
  LegacyV304,
  V305Left,
  V305Right,
  V306Left,
  V306Right,
  V307Left,
  V307Right,
  V308Left,
  V308Right,
  // Historical V3.0.9 profile retained for regression only.
  V309Left,
  V309Right,
  // Active V3.1.1 arms with the Kkozia carriage adapter and Tool0.
  V311Left,
  V311Right,
};

struct V3RedundantIkSolution
{
  std::array<double, 7> joints{};
  double swivel_angle = 0.0;
  int shoulder_branch = 0;
  int elbow_branch = 0;
  int wrist_branch = 0;
  double position_error = 0.0;
  double orientation_error = 0.0;
  double seed_distance = 0.0;
  double minimum_joint_limit_margin = 0.0;
};

struct V3RedundantIkRequest
{
  Eigen::Isometry3d target_in_arm_base = Eigen::Isometry3d::Identity();
  double swivel_angle = 0.0;
  std::array<double, 7> seed{};
  double position_tolerance = 1e-7;
  double orientation_tolerance = 1e-7;
  bool enforce_joint_limits = true;
};

class V3RedundantArmAnalyticIk
{
public:
  explicit V3RedundantArmAnalyticIk(
    V3RedundantArmModel model = V3RedundantArmModel::LegacyV304);

  std::vector<V3RedundantIkSolution> solveInArmBase(
    const V3RedundantIkRequest& request) const;

  Eigen::Isometry3d forwardInArmBase(
    const std::array<double, 7>& joints) const;

  Eigen::Vector3d elbowPositionInArmBase(
    const std::array<double, 7>& joints) const;

  Eigen::Vector3d wristCenterInArmBase(
    const std::array<double, 7>& joints) const;

  double swivelAngle(
    const std::array<double, 7>& joints) const;

  // Model-specific shoulder center; the static accessor below retains legacy semantics.
  Eigen::Vector3d modelShoulderCenterInArmBase() const;
  double modelArmLength() const;  // Shoulder-elbow plus elbow-wrist, excluding TCP.
  static Eigen::Vector3d shoulderCenterInArmBase();
  static double upperArmLength();
  static double forearmLength();
  std::array<double, 7> jointLowerLimits() const;
  std::array<double, 7> jointUpperLimits() const;

private:
  V3RedundantArmModel model_;
};

}  // namespace alfa_robot::analytic_ik
