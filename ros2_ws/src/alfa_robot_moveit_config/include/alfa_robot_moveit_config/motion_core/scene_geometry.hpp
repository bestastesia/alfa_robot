#pragma once

#include "alfa_robot_moveit_config/motion_core/task_geometry.hpp"

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct ContainerGeometryConfig
{
  double center_x = 0.0;
  double center_y = 0.0;
  double width = 2.2;
  double height = 2.4;
  double length = 8.0;
  double wall_thickness = 0.03;
  double floor_z = 0.0;
};

struct BoxWallGeometryConfig
{
  double box_front_x = 0.625;
  double container_center_y = 0.0;
  double container_width = 2.2;
  double container_floor_z = 0.0;
  double carried_box_width = 0.4;
  double carried_box_height = 0.4;
  double carried_box_depth = 0.3;
  double static_box_obstacle_inset = 0.002;
};

struct CarriedBoxGeometryConfig
{
  double carried_box_width = 0.4;
  double carried_box_height = 0.4;
  double carried_box_depth = 0.3;
};

std::vector<ContainerPanel> make_container_panels(const ContainerGeometryConfig& config);

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

bool carried_box_detached_from_neighbors(
  const AxisAlignedBox& carried_box,
  int box_id,
  double box_front_x,
  double carried_box_width,
  double carried_box_height,
  double carried_box_depth,
  double margin,
  const std::string& carried_box_id,
  std::string* reason);

}  // namespace alfa_robot::motion
