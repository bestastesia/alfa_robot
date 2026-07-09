#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace alfa_robot::analytic_ik
{

enum class ArmSide
{
  Left,
  Right,
};

struct ArmAnalyticIkSolution
{
  std::array<double, 6> joints{};
  double position_error = 0.0;
  double orientation_error = 0.0;
  double seed_distance = 0.0;
};

struct ArmAnalyticIkRequest
{
  ArmSide side = ArmSide::Left;
  Eigen::Isometry3d target_in_arm_base = Eigen::Isometry3d::Identity();
  std::array<double, 6> seed{};
  double position_tolerance = 1e-4;
  double orientation_tolerance = 1e-4;
  size_t root_samples = 720;
};

class ThreeParallelArmAnalyticIk
{
public:
  std::vector<ArmAnalyticIkSolution> solveInArmBase(
    const ArmAnalyticIkRequest& request) const;

  std::vector<ArmAnalyticIkSolution> solveInBaseLink(
    ArmSide side,
    const Eigen::Isometry3d& target_in_base_link,
    double updown,
    const std::array<double, 6>& seed,
    double position_tolerance = 1e-4,
    double orientation_tolerance = 1e-4,
    size_t root_samples = 720) const;

  Eigen::Isometry3d forwardInArmBase(
    ArmSide side,
    const std::array<double, 6>& joints) const;

  static Eigen::Isometry3d baseLinkToUpdown(double updown);
  static Eigen::Isometry3d updownToArmBase(ArmSide side);
  static Eigen::Isometry3d baseLinkToArmBase(ArmSide side, double updown);
  static std::string sideName(ArmSide side);
};

double normalizeAngle(double value);
double shortestAngularDistance(double lhs, double rhs);

}  // namespace alfa_robot::analytic_ik
