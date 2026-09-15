#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace alfa_robot::motion
{
inline double shortestAngleDelta(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

inline double naturalJointDistanceSquared(
  const std::array<double, 7>& from, const std::array<double, 7>& to)
{
  // Shoulder/elbow motion is useful; distal wrist/swivel motion should earn its cost.
  constexpr std::array<double, 7> weights{1.0, 1.0, 1.0, 1.0, 2.0, 3.0, 5.0};
  double distance = 0.0;
  for (size_t i = 0; i < from.size(); ++i) {
    const double delta = shortestAngleDelta(from[i], to[i]);
    distance += weights[i] * delta * delta;
  }
  return distance;
}

inline bool sameShoulderElbowBranch(
  const std::array<double, 7>& from, const std::array<double, 7>& to,
  double zero_tolerance = 0.05)
{
  for (const size_t i : {1U, 3U}) {
    if (std::abs(from[i]) > zero_tolerance && std::abs(to[i]) > zero_tolerance &&
        std::signbit(from[i]) != std::signbit(to[i])) return false;
  }
  return true;
}

inline double naturalJointPathLength(const std::vector<std::array<double, 7>>& path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    length += std::sqrt(naturalJointDistanceSquared(path[i - 1], path[i]));
  return length;
}

inline bool naturalJointPath(
  const std::vector<std::array<double, 7>>& path, double maximum_ratio = 4.0,
  double minimum_allowance = 1.0)
{
  if (path.size() < 2) return true;
  for (size_t i = 1; i < path.size(); ++i)
    if (!sameShoulderElbowBranch(path[i - 1], path[i])) return false;
  const double direct = std::sqrt(naturalJointDistanceSquared(path.front(), path.back()));
  return naturalJointPathLength(path) <= std::max(minimum_allowance, maximum_ratio * direct);
}
}  // namespace alfa_robot::motion
