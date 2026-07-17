#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace alfa_robot::motion
{

namespace
{

// 把矩形 (center, half_extent, axis_x/axis_y 是矩形自身的两条正交边方向) 的四个角点
// 投影到 axis 上，返回 [min, max]。axis 必须是单位向量。
std::pair<double, double> project_rectangle(
  const std::array<double, 2>& center,
  const std::array<double, 2>& half_extent,
  const std::array<double, 2>& axis_x,
  const std::array<double, 2>& axis_y,
  const std::array<double, 2>& axis)
{
  const double center_proj = center[0] * axis[0] + center[1] * axis[1];
  const double radius =
    std::abs(half_extent[0] * (axis_x[0] * axis[0] + axis_x[1] * axis[1])) +
    std::abs(half_extent[1] * (axis_y[0] * axis[0] + axis_y[1] * axis[1]));
  return {center_proj - radius, center_proj + radius};
}

bool intervals_overlap(const std::pair<double, double>& lhs, const std::pair<double, double>& rhs)
{
  return lhs.first <= rhs.second && lhs.second >= rhs.first;
}

}  // namespace

bool aabb_overlaps_oriented_box(const AxisAlignedBox& aabb, const OrientedBox& obb)
{
  const double aabb_z_min = aabb.center[2] - 0.5 * aabb.size[2];
  const double aabb_z_max = aabb.center[2] + 0.5 * aabb.size[2];
  const double obb_z_min = obb.center[2] - 0.5 * obb.size[2];
  const double obb_z_max = obb.center[2] + 0.5 * obb.size[2];
  if (aabb_z_max < obb_z_min || obb_z_max < aabb_z_min) {
    return false;
  }

  const std::array<double, 2> aabb_center{aabb.center[0], aabb.center[1]};
  const std::array<double, 2> aabb_half{0.5 * aabb.size[0], 0.5 * aabb.size[1]};
  const std::array<double, 2> aabb_axis_x{1.0, 0.0};
  const std::array<double, 2> aabb_axis_y{0.0, 1.0};

  const std::array<double, 2> obb_center{obb.center[0], obb.center[1]};
  const std::array<double, 2> obb_half{0.5 * obb.size[0], 0.5 * obb.size[1]};
  const std::array<double, 2> obb_axis_x{std::cos(obb.yaw), std::sin(obb.yaw)};
  const std::array<double, 2> obb_axis_y{-std::sin(obb.yaw), std::cos(obb.yaw)};

  const std::array<std::array<double, 2>, 4> axes{
    aabb_axis_x, aabb_axis_y, obb_axis_x, obb_axis_y};

  for (const auto& axis : axes) {
    const auto aabb_interval =
      project_rectangle(aabb_center, aabb_half, aabb_axis_x, aabb_axis_y, axis);
    const auto obb_interval =
      project_rectangle(obb_center, obb_half, obb_axis_x, obb_axis_y, axis);
    if (!intervals_overlap(aabb_interval, obb_interval)) {
      return false;
    }
  }
  return true;
}

std::string trim_copy(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::map<int, BoxSpec> make_boxes(double front_x, double y_shift)
{
  const std::vector<std::vector<std::pair<int, double>>> rows_top_to_bottom = {
    {{1, 0.8}, {2, 0.4}, {3, 0.0}, {4, -0.4}, {5, -0.8}},
    {{6, 0.8}, {7, 0.4}, {8, 0.0}, {9, -0.4}, {10, -0.8}},
    {{11, 0.8}, {12, 0.4}, {13, 0.0}, {14, -0.4}, {15, -0.8}},
    {{16, 0.8}, {17, 0.4}, {18, 0.0}, {19, -0.4}, {20, -0.8}},
    {{21, 0.8}, {22, 0.4}, {23, 0.0}, {24, -0.4}, {25, -0.8}},
  };

  std::map<int, BoxSpec> boxes;
  for (size_t row = 0; row < rows_top_to_bottom.size(); ++row) {
    const double z = 0.2 + 0.4 * static_cast<double>(rows_top_to_bottom.size() - 1 - row);
    for (const auto& [id, y] : rows_top_to_bottom[row]) {
      boxes[id] = BoxSpec{id, front_x, y + y_shift, z};
    }
  }
  return boxes;
}

std::vector<std::pair<int, int>> parse_box_pair_list(const std::string& value)
{
  std::vector<std::pair<int, int>> pairs;
  std::stringstream stream(value);
  std::string segment;
  while (std::getline(stream, segment, ';')) {
    segment = trim_copy(segment);
    if (segment.empty()) continue;
    const auto comma = segment.find(',');
    const auto slash = segment.find('/');
    const auto sep = comma == std::string::npos ? slash : comma;
    if (sep == std::string::npos) continue;
    const std::string left = trim_copy(segment.substr(0, sep));
    const std::string right = trim_copy(segment.substr(sep + 1));
    if (left.empty() || right.empty()) continue;
    pairs.push_back({std::stoi(left), std::stoi(right)});
  }
  return pairs;
}

std::vector<PickPair> make_pick_pairs(
  bool include_top_suction,
  const std::vector<std::pair<int, int>>& front_pairs)
{
  std::vector<PickPair> pairs;
  pairs.reserve(front_pairs.size() + 1);
  for (size_t index = 0; index < front_pairs.size(); ++index) {
    pairs.push_back({
      static_cast<int>(index + 1),
      front_pairs[index].first,
      front_pairs[index].second,
      false,
    });
  }
  if (include_top_suction) {
    const int round = static_cast<int>(pairs.size() + 1);
    pairs.push_back({round, 22, 24, true});
  }
  return pairs;
}

std::vector<std::string> dual_arm_with_updown_joint_names()
{
  return {
    "updown",
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
  };
}

}  // namespace alfa_robot::motion
