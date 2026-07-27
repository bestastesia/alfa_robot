#pragma once

#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct ContainerGeometryConfig
{
  double center_x = 0.0;
  double center_y = 0.0;
  double yaw = 0.0;  // 绕 Z 轴，弧度；集装箱相对 config.frame（world）的朝向
  double width = 1.8;
  double height = 2.4;
  double length = 8.0;
  double wall_thickness = 0.03;
  double floor_z = 0.0;
};

struct BoxWallGeometryConfig
{
  double box_front_x = 0.625;
  double scene_y_shift = 0.0;
  double container_center_y = 0.0;
  double container_width = 1.8;
  double container_floor_z = 0.0;
  double carried_box_width = kBoxWidth;
  double carried_box_height = kBoxHeight;
  double carried_box_depth = kBoxDepth;
  double static_box_obstacle_inset = 0.002;
  bool rear_guard_enabled = true;
  double rear_guard_thickness = 0.01;
  double rear_guard_clearance = 0.03;
  double container_height = 2.4;
};

struct CarriedBoxGeometryConfig
{
  double carried_box_width = kBoxWidth;
  double carried_box_height = kBoxHeight;
  double carried_box_depth = kBoxDepth;
  double grasp_lateral_offset = kOuterBoxGraspLateralOffset;
};

std::vector<ContainerPanel> make_container_panels(const ContainerGeometryConfig& config);

// 集装箱相对车体（world 系）的动态位姿：x/y 为平移分量，yaw 为绕 Z 轴的朝向差。
struct ContainerRelativePose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 给定车体在 map 下的位姿（vehicle_pose_map，即 map -> world 变换本身）和集装箱在
// map 下的绝对位姿 (container_map_x/y/yaw)，计算集装箱相对车体（world 系）的位姿。
// 车体在原点且 yaw=0 时，结果退化为集装箱的 map 绝对坐标（向后兼容基准）。
ContainerRelativePose compute_container_pose_relative_to_vehicle(
  const Eigen::Isometry3d& vehicle_pose_map,
  double container_map_x,
  double container_map_y,
  double container_map_yaw);

std::vector<StaticBoxObstacle> make_box_wall_obstacles_for_opening(
  int left_box_id,
  int right_box_id,
  const BoxWallGeometryConfig& config);

AttachedBoxSpec make_attached_box_spec(
  const std::string& side,
  int box_id,
  bool top_suction,
  const CarriedBoxGeometryConfig& config);

Eigen::Vector3d transform_point(
  const Eigen::Isometry3d& transform,
  const std::array<double, 3>& point);

AxisAlignedBox aabb_from_attached_box_transform(
  const Eigen::Isometry3d& link_transform,
  const AttachedBoxSpec& box);

bool aabb_overlaps(const AxisAlignedBox& lhs, const AxisAlignedBox& rhs);

AxisAlignedBox expanded_aabb(const AxisAlignedBox& box, double margin);

bool carried_box_detached_from_source_xz(
  const AxisAlignedBox& carried_box,
  const AxisAlignedBox& source_box,
  double margin,
  const std::string& carried_box_id,
  std::string* reason);

bool carried_box_detached_from_source_layers_xz(
  const AxisAlignedBox& carried_box,
  int box_id,
  double box_front_x,
  double scene_y_shift,
  double carried_box_width,
  double carried_box_height,
  double carried_box_depth,
  double margin,
  size_t clearance_levels,
  const std::string& carried_box_id,
  std::string* reason);

bool carried_box_detached_from_neighbors(
  const AxisAlignedBox& carried_box,
  int box_id,
  double box_front_x,
  double scene_y_shift,
  double carried_box_width,
  double carried_box_height,
  double carried_box_depth,
  double margin,
  const std::string& carried_box_id,
  std::string* reason);

bool carried_box_clear_obstacles(
  const AxisAlignedBox& carried_box,
  const std::string& carried_box_id,
  const std::vector<StaticBoxObstacle>& static_obstacles,
  const std::vector<ContainerPanel>& container_panels,
  std::string* reason);

bool carried_box_clear_rear_guards(
  const AxisAlignedBox& carried_box,
  const std::string& carried_box_id,
  const std::vector<StaticBoxObstacle>& static_obstacles,
  std::string* reason);

}  // namespace alfa_robot::motion
