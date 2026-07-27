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
  std::map<int, BoxSpec> boxes;
  for (int row = 0; row < kBoxStackRowCount; ++row) {
    const double z = (static_cast<double>(kBoxStackRowCount - row) - 0.5) * kBoxHeight;
    for (int column = 0; column < kBoxStackColumnCount; ++column) {
      const int id = row * kBoxStackColumnCount + column + 1;
      const double y =
        (0.5 * static_cast<double>(kBoxStackColumnCount - 1) - static_cast<double>(column)) *
        kBoxWidth;
      boxes[id] = BoxSpec{id, front_x, y + y_shift, z};
    }
  }
  return boxes;
}

int box_column_from_left(int box_id)
{
  if (box_id < 1 || box_id > kBoxStackBoxCount) return 0;
  return (box_id - 1) % kBoxStackColumnCount + 1;
}

int box_row_from_top(int box_id)
{
  if (box_id < 1 || box_id > kBoxStackBoxCount) return -1;
  return (box_id - 1) / kBoxStackColumnCount;
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
    pairs.push_back({round, 13, 15, true});
  }
  return pairs;
}

std::vector<std::string> dual_arm_with_updown_joint_names()
{
  return {
    "updown",
    "left_joint1", "left_joint2", "left_joint3",
    "left_joint4", "left_joint5", "left_joint6",
    "right_joint1", "right_joint2", "right_joint3",
    "right_joint4", "right_joint5", "right_joint6",
  };
}

}  // namespace alfa_robot::motion
