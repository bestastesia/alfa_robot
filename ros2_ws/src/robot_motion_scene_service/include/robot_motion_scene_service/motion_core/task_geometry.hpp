#pragma once

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace alfa_robot::motion
{

inline constexpr int kBoxStackColumnCount = 3;
inline constexpr int kBoxStackRowCount = 5;
inline constexpr int kBoxStackBoxCount = kBoxStackColumnCount * kBoxStackRowCount;
inline constexpr double kBoxDepth = 0.3;
inline constexpr double kBoxWidth = 0.5;
inline constexpr double kBoxHeight = 0.4;
inline constexpr double kOuterBoxGraspTargetY = 0.50;
inline constexpr double kOuterBoxGraspLateralOffset = kBoxWidth - kOuterBoxGraspTargetY;

struct BoxSpec
{
  int id = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PickPair
{
  int round = 0;
  int left_box = 0;
  int right_box = 0;
  bool top_suction = false;
};

struct ContainerPanel
{
  std::string id;
  std::array<double, 3> center;
  std::array<double, 3> size;
  double yaw = 0.0;  // 绕 Z 轴，弧度；0 时是轴对齐墙板，与改造前行为一致
};

struct StaticBoxObstacle
{
  std::string id;
  std::array<double, 3> center;
  std::array<double, 3> size;
};

struct AttachedBoxSpec
{
  std::string id;
  std::string link_name;
  std::array<double, 3> center_in_link;
  std::array<double, 3> size;
};

struct AxisAlignedBox
{
  std::array<double, 3> center;
  std::array<double, 3> size;
};

// 绕 Z 轴旋转的长方体（车体/集装箱地面法线始终朝上，roll/pitch 恒为 0）。
// yaw == 0 时退化为轴对齐 box，与 AxisAlignedBox 语义一致。
struct OrientedBox
{
  std::array<double, 3> center;
  std::array<double, 3> size;
  double yaw = 0.0;
};

// aabb 与绕 Z 轴旋转的 obb 是否重叠。Z 方向不受 yaw 影响，按 1D 区间独立判断；
// X-Y 平面内退化为"轴对齐矩形 vs 旋转矩形"的 2D 分离轴测试。
bool aabb_overlaps_oriented_box(const AxisAlignedBox& aabb, const OrientedBox& obb);

std::string trim_copy(std::string value);

std::map<int, BoxSpec> make_boxes(double front_x, double y_shift = 0.0);

int box_column_from_left(int box_id);

int box_row_from_top(int box_id);

std::vector<std::pair<int, int>> parse_box_pair_list(const std::string& value);

std::vector<PickPair> make_pick_pairs(
  bool include_top_suction,
  const std::vector<std::pair<int, int>>& front_pairs);

std::vector<std::string> dual_arm_with_updown_joint_names();

}  // namespace alfa_robot::motion
