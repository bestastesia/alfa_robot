#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace alfa_robot::motion
{
struct WallTransferRound
{
  int left_box = -1;
  int right_box = -1;

  bool dual() const { return left_box >= 0 && right_box >= 0; }
};

inline std::vector<WallTransferRound> wallTransferRounds()
{
  std::vector<WallTransferRound> rounds;
  rounds.reserve(15);
  for (int row = 4; row >= 0; --row) {
    const int first = row * 5;
    rounds.push_back({first, first + 4});
    rounds.push_back({first + 1, first + 3});
    // Alternate the centre box to avoid making either arm the permanent fallback arm.
    rounds.push_back(row % 2 == 0 ? WallTransferRound{first + 2, -1} :
      WallTransferRound{-1, first + 2});
  }
  return rounds;
}

inline std::vector<int> wallSequenceOrder()
{
  std::vector<int> ids;
  for (int row = 4; row >= 0; --row)
    for (int column = 0; column < 5; ++column) ids.push_back(row * 5 + column);
  return ids;
}

inline bool isBottomBox(double center_z, double height, double wall_bottom_z)
{
  return std::abs(center_z - height / 2.0 - wall_bottom_z) <= 1e-6;
}

inline std::vector<std::pair<bool, std::string>> wallGraspAttempts(bool bottom, const std::string& arm, bool top_only = false)
{
  std::vector<std::pair<bool, std::string>> attempts;
  for (bool top : {false, true}) {
    if ((bottom || top_only) && !top) continue;
    for (const std::string side : {"left", "right"})
      if (arm == "auto" || arm == side) attempts.emplace_back(top, side);
  }
  return attempts;
}

inline Eigen::Isometry3d wallContactPose(
  const Eigen::Vector3d& center, const Eigen::Vector3d& size, double gap, bool top)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  if (top) {
    pose.linear() = Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
    pose.translation() = center + Eigen::Vector3d(0, 0, size.z() / 2.0 + gap);
  } else {
    pose.linear() = Eigen::AngleAxisd(std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    pose.translation() = center - Eigen::Vector3d(size.x() / 2.0 + gap, 0, 0);
  }
  return pose;
}

inline Eigen::Isometry3d wallRearPlacementPose(
  Eigen::Isometry3d tool, const Eigen::Isometry3d& tool_to_box,
  const Eigen::Vector3d& size, double rear_x, double clearance)
{
  tool.linear() = Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix() * tool.linear();
  const Eigen::Isometry3d box = tool * tool_to_box;
  const double max_x = box.translation().x() + box.linear().row(0).cwiseAbs().dot(size / 2.0);
  tool.translation().x() += rear_x - clearance - max_x;
  return tool;
}

inline bool boxBehindChassis(const Eigen::Isometry3d& box, const Eigen::Vector3d& size, double rear_x)
{
  const double max_x = box.translation().x() + box.linear().row(0).cwiseAbs().dot(size / 2.0);
  return max_x <= rear_x - 0.01;
}
}  // namespace alfa_robot::motion
