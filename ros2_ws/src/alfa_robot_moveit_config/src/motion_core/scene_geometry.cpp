#include "alfa_robot_moveit_config/motion_core/scene_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace alfa_robot::motion
{

namespace
{

void add_static_wall_piece(
  std::vector<StaticBoxObstacle>& obstacles,
  const std::string& id,
  double x_min,
  double x_max,
  double y_min,
  double y_max,
  double z_min,
  double z_max)
{
  if (x_max < x_min) std::swap(x_min, x_max);
  if (y_max < y_min) std::swap(y_min, y_max);
  if (z_max < z_min) std::swap(z_min, z_max);
  if ((x_max - x_min) < 1e-4 || (y_max - y_min) < 1e-4 || (z_max - z_min) < 1e-4) {
    return;
  }

  obstacles.push_back({
    id,
    {0.5 * (x_min + x_max), 0.5 * (y_min + y_max), 0.5 * (z_min + z_max)},
    {x_max - x_min, y_max - y_min, z_max - z_min},
  });
}

}  // namespace

std::vector<ContainerPanel> make_container_panels(const ContainerGeometryConfig& config)
{
  const double half_width = config.width * 0.5;
  const double half_thickness = config.wall_thickness * 0.5;
  const double z_center = config.floor_z + config.height * 0.5;
  return {
    {
      "container_left_wall",
      {config.center_x, config.center_y + half_width + half_thickness, z_center},
      {config.length, config.wall_thickness, config.height},
    },
    {
      "container_right_wall",
      {config.center_x, config.center_y - half_width - half_thickness, z_center},
      {config.length, config.wall_thickness, config.height},
    },
    {
      "container_ceiling",
      {config.center_x, config.center_y, config.floor_z + config.height + half_thickness},
      {config.length, config.width + 2.0 * config.wall_thickness, config.wall_thickness},
    },
  };
}

std::vector<StaticBoxObstacle> make_box_wall_obstacles_for_opening(
  int left_box_id,
  int right_box_id,
  const BoxWallGeometryConfig& config)
{
  std::vector<StaticBoxObstacle> obstacles;

  const auto boxes = make_boxes(config.box_front_x);
  const auto left_it = boxes.find(left_box_id);
  const auto right_it = boxes.find(right_box_id);
  if (left_it == boxes.end() || right_it == boxes.end()) return obstacles;

  const BoxSpec& first = left_it->second;
  const BoxSpec& second = right_it->second;
  const BoxSpec& positive_y_box = first.y >= second.y ? first : second;
  const BoxSpec& negative_y_box = first.y >= second.y ? second : first;

  const double half_width = config.carried_box_width * 0.5;
  const double half_height = config.carried_box_height * 0.5;
  const double inset = std::max(0.0, config.static_box_obstacle_inset);
  const double x_min = std::min(first.x, second.x);
  const double x_max = std::max(first.x, second.x) + config.carried_box_depth;
  const double inner_y_min = config.container_center_y - config.container_width * 0.5;
  const double inner_y_max = config.container_center_y + config.container_width * 0.5;
  const double z_min = std::min(first.z, second.z) - half_height;
  const double z_max = std::max(first.z, second.z) + half_height;

  const double positive_hole_y_min = positive_y_box.y - half_width;
  const double positive_hole_y_max = positive_y_box.y + half_width;
  const double negative_hole_y_min = negative_y_box.y - half_width;
  const double negative_hole_y_max = negative_y_box.y + half_width;
  const std::string prefix = "box_wall_L" + std::to_string(left_box_id) +
                             "_R" + std::to_string(right_box_id);

  add_static_wall_piece(
    obstacles, prefix + "_left_side",
    x_min, x_max,
    positive_hole_y_max + inset, inner_y_max,
    z_min, z_max);
  const double between_y_min = negative_hole_y_max + inset;
  const double between_y_max = positive_hole_y_min - inset;
  if (between_y_max > between_y_min) {
    add_static_wall_piece(
      obstacles, prefix + "_between",
      x_min, x_max,
      between_y_min, between_y_max,
      z_min, z_max);
  }
  add_static_wall_piece(
    obstacles, prefix + "_right_side",
    x_min, x_max,
    inner_y_min, negative_hole_y_min - inset,
    z_min, z_max);
  add_static_wall_piece(
    obstacles, prefix + "_below",
    x_min, x_max,
    inner_y_min, inner_y_max,
    config.container_floor_z, z_min - inset);

  return obstacles;
}

AttachedBoxSpec make_attached_box_spec(
  const std::string& side,
  int box_id,
  bool top_suction,
  const CarriedBoxGeometryConfig& config)
{
  AttachedBoxSpec spec;
  spec.id = "carried_" + side + "_box_" + std::to_string(box_id);
  spec.link_name = side + "_v5_tool0";
  if (top_suction) {
    spec.center_in_link = {0.0, 0.0, config.carried_box_height * 0.5};
    spec.size = {config.carried_box_depth, config.carried_box_width, config.carried_box_height};
  } else {
    spec.center_in_link = {0.0, 0.0, config.carried_box_depth * 0.5};
    spec.size = {config.carried_box_width, config.carried_box_height, config.carried_box_depth};
  }
  return spec;
}

Eigen::Vector3d transform_point(
  const Eigen::Isometry3d& transform,
  const std::array<double, 3>& point)
{
  return transform * Eigen::Vector3d(point[0], point[1], point[2]);
}

AxisAlignedBox aabb_from_attached_box_transform(
  const Eigen::Isometry3d& link_transform,
  const AttachedBoxSpec& box)
{
  Eigen::Vector3d min_corner(
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_corner(
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity());

  for (double sx : {-0.5, 0.5}) {
    for (double sy : {-0.5, 0.5}) {
      for (double sz : {-0.5, 0.5}) {
        const std::array<double, 3> local_corner = {
          box.center_in_link[0] + sx * box.size[0],
          box.center_in_link[1] + sy * box.size[1],
          box.center_in_link[2] + sz * box.size[2],
        };
        const Eigen::Vector3d world_corner = transform_point(link_transform, local_corner);
        min_corner = min_corner.cwiseMin(world_corner);
        max_corner = max_corner.cwiseMax(world_corner);
      }
    }
  }

  const Eigen::Vector3d center = 0.5 * (min_corner + max_corner);
  const Eigen::Vector3d size = max_corner - min_corner;
  return {{
    center.x(), center.y(), center.z()
  }, {
    size.x(), size.y(), size.z()
  }};
}

bool aabb_overlaps(const AxisAlignedBox& lhs, const AxisAlignedBox& rhs)
{
  for (size_t i = 0; i < 3; ++i) {
    if (std::abs(lhs.center[i] - rhs.center[i]) > 0.5 * (lhs.size[i] + rhs.size[i])) {
      return false;
    }
  }
  return true;
}

AxisAlignedBox expanded_aabb(const AxisAlignedBox& box, double margin)
{
  return {box.center, {
    box.size[0] + 2.0 * margin,
    box.size[1] + 2.0 * margin,
    box.size[2] + 2.0 * margin,
  }};
}

bool carried_box_detached_from_neighbors(
  const AxisAlignedBox& carried_box,
  int box_id,
  double box_front_x,
  double carried_box_width,
  double carried_box_height,
  double carried_box_depth,
  double margin,
  const std::string& carried_box_id,
  std::string* reason)
{
  const int column = (box_id - 1) % 5 + 1;
  const int row = (box_id - 1) / 5;
  std::vector<int> neighbor_ids;
  if (column > 1) neighbor_ids.push_back(row * 5 + column - 1);
  if (column < 5) neighbor_ids.push_back(row * 5 + column + 1);

  const auto boxes = make_boxes(box_front_x);
  const AxisAlignedBox carried = expanded_aabb(carried_box, margin);
  const double carried_min_x = carried.center[0] - 0.5 * carried.size[0];
  const double carried_max_x = carried.center[0] + 0.5 * carried.size[0];
  const double carried_min_z = carried.center[2] - 0.5 * carried.size[2];
  const double carried_max_z = carried.center[2] + 0.5 * carried.size[2];
  for (int neighbor_id : neighbor_ids) {
    const auto it = boxes.find(neighbor_id);
    if (it == boxes.end()) continue;
    const AxisAlignedBox neighbor = expanded_aabb({{
      it->second.x + carried_box_depth * 0.5,
      it->second.y,
      it->second.z,
    }, {
      carried_box_depth,
      carried_box_width,
      carried_box_height,
    }}, margin);
    const double neighbor_min_x = neighbor.center[0] - 0.5 * neighbor.size[0];
    const double neighbor_max_x = neighbor.center[0] + 0.5 * neighbor.size[0];
    const double neighbor_min_z = neighbor.center[2] - 0.5 * neighbor.size[2];
    const double neighbor_max_z = neighbor.center[2] + 0.5 * neighbor.size[2];
    const bool side_face_projection_overlaps =
      carried_min_x <= neighbor_max_x &&
      carried_max_x >= neighbor_min_x &&
      carried_min_z <= neighbor_max_z &&
      carried_max_z >= neighbor_min_z;
    if (side_face_projection_overlaps) {
      if (reason) {
        *reason = carried_box_id + " side face still overlaps neighbor box " + std::to_string(neighbor_id);
      }
      return false;
    }
  }
  return true;
}

}  // namespace alfa_robot::motion
