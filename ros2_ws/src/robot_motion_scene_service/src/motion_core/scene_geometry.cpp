#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"

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

// 把本地偏移量 (local_x, local_y)（未旋转时相对集装箱中心的偏移）绕 Z 轴转 yaw，
// 再叠加到 (center_x, center_y) 上，得到旋转后墙板在 world 系下的中心。
std::array<double, 2> rotate_panel_offset(
  double center_x, double center_y, double local_x, double local_y, double yaw)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return {
    center_x + local_x * cos_yaw - local_y * sin_yaw,
    center_y + local_x * sin_yaw + local_y * cos_yaw,
  };
}

}  // namespace

std::vector<ContainerPanel> make_container_panels(const ContainerGeometryConfig& config)
{
  const double half_width = config.width * 0.5;
  const double half_thickness = config.wall_thickness * 0.5;
  const double z_center = config.floor_z + config.height * 0.5;

  const auto left_center = rotate_panel_offset(
    config.center_x, config.center_y, 0.0, half_width + half_thickness, config.yaw);
  const auto right_center = rotate_panel_offset(
    config.center_x, config.center_y, 0.0, -(half_width + half_thickness), config.yaw);
  const auto ceiling_center = rotate_panel_offset(
    config.center_x, config.center_y, 0.0, 0.0, config.yaw);

  return {
    {
      "container_left_wall",
      {left_center[0], left_center[1], z_center},
      {config.length, config.wall_thickness, config.height},
      config.yaw,
    },
    {
      "container_right_wall",
      {right_center[0], right_center[1], z_center},
      {config.length, config.wall_thickness, config.height},
      config.yaw,
    },
    {
      "container_ceiling",
      {ceiling_center[0], ceiling_center[1], config.floor_z + config.height + half_thickness},
      {config.length, config.width + 2.0 * config.wall_thickness, config.wall_thickness},
      config.yaw,
    },
  };
}

ContainerRelativePose compute_container_pose_relative_to_vehicle(
  const Eigen::Isometry3d& vehicle_pose_map,
  double container_map_x,
  double container_map_y,
  double container_map_yaw)
{
  Eigen::Isometry3d container_pose_map = Eigen::Isometry3d::Identity();
  container_pose_map.translation() = Eigen::Vector3d(container_map_x, container_map_y, 0.0);
  container_pose_map.linear() =
    Eigen::AngleAxisd(container_map_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Eigen::Isometry3d relative = vehicle_pose_map.inverse() * container_pose_map;
  const double yaw = std::atan2(relative.linear()(1, 0), relative.linear()(0, 0));
  return {relative.translation().x(), relative.translation().y(), yaw};
}


std::vector<StaticBoxObstacle> make_box_wall_obstacles_for_opening(
  int left_box_id,
  int right_box_id,
  const BoxWallGeometryConfig& config)
{
  std::vector<StaticBoxObstacle> obstacles;

  const auto boxes = make_boxes(config.box_front_x, config.scene_y_shift);
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
  double stack_z_min = std::numeric_limits<double>::infinity();
  double stack_z_max = -std::numeric_limits<double>::infinity();
  for (const auto& [_, box] : boxes) {
    stack_z_min = std::min(stack_z_min, box.z - half_height);
    stack_z_max = std::max(stack_z_max, box.z + half_height);
  }

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
  if (config.rear_guard_enabled) {
    const double thickness = std::max(1e-4, config.rear_guard_thickness);
    const double clearance = std::max(0.0, config.rear_guard_clearance);
    const double guard_x_min = config.box_front_x + config.carried_box_depth + clearance;
    add_static_wall_piece(
      obstacles, prefix + "_rear_guard",
      guard_x_min, guard_x_min + thickness,
      inner_y_min, inner_y_max,
      stack_z_min, stack_z_max);
  }

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
  spec.link_name = side + "_tool0";
  const double lateral_offset = side == "left"
    ? -config.grasp_lateral_offset
    : config.grasp_lateral_offset;
  if (top_suction) {
    spec.center_in_link = {0.0, lateral_offset, config.carried_box_height * 0.5};
    spec.size = {config.carried_box_depth, config.carried_box_width, config.carried_box_height};
  } else {
    spec.center_in_link = {0.0, lateral_offset, config.carried_box_depth * 0.5};
    spec.size = {config.carried_box_height, config.carried_box_width, config.carried_box_depth};
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

bool carried_box_detached_from_source_xz(
  const AxisAlignedBox& carried_box,
  const AxisAlignedBox& source_box,
  double margin,
  const std::string& carried_box_id,
  std::string* reason)
{
  const AxisAlignedBox protected_source = expanded_aabb(source_box, std::max(0.0, margin));
  const double carried_min_x = carried_box.center[0] - 0.5 * carried_box.size[0];
  const double carried_max_x = carried_box.center[0] + 0.5 * carried_box.size[0];
  const double carried_min_z = carried_box.center[2] - 0.5 * carried_box.size[2];
  const double carried_max_z = carried_box.center[2] + 0.5 * carried_box.size[2];
  const double source_min_x = protected_source.center[0] - 0.5 * protected_source.size[0];
  const double source_max_x = protected_source.center[0] + 0.5 * protected_source.size[0];
  const double source_min_z = protected_source.center[2] - 0.5 * protected_source.size[2];
  const double source_max_z = protected_source.center[2] + 0.5 * protected_source.size[2];
  constexpr double kBoundaryTolerance = 1e-9;
  const bool x_overlaps =
    carried_min_x < source_max_x - kBoundaryTolerance &&
    carried_max_x > source_min_x + kBoundaryTolerance;
  const bool z_overlaps =
    carried_min_z < source_max_z - kBoundaryTolerance &&
    carried_max_z > source_min_z + kBoundaryTolerance;
  if (!x_overlaps || !z_overlaps) {
    return true;
  }
  if (reason) {
    std::ostringstream oss;
    oss << carried_box_id << " x-z projection still overlaps source box"
        << " carried_x=[" << carried_min_x << ',' << carried_max_x << ']'
        << " carried_z=[" << carried_min_z << ',' << carried_max_z << ']'
        << " source_x=[" << source_min_x << ',' << source_max_x << ']'
        << " source_z=[" << source_min_z << ',' << source_max_z << ']';
    *reason = oss.str();
  }
  return false;
}

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
  std::string* reason)
{
  const auto boxes = make_boxes(box_front_x, scene_y_shift);
  const size_t levels = std::max<size_t>(1, clearance_levels);
  for (size_t level = 0; level < levels; ++level) {
    const int layer_box_id = box_id - static_cast<int>(kBoxStackColumnCount * level);
    if (layer_box_id <= 0) continue;
    const auto it = boxes.find(layer_box_id);
    if (it == boxes.end()) continue;
    const AxisAlignedBox source_layer{{
      it->second.x + carried_box_depth * 0.5,
      it->second.y,
      it->second.z,
    }, {
      carried_box_depth,
      carried_box_width,
      carried_box_height,
    }};
    std::string layer_reason;
    if (!carried_box_detached_from_source_xz(
        carried_box, source_layer, margin, carried_box_id, &layer_reason)) {
      if (reason) {
        *reason = carried_box_id + " side face still overlaps source layer box " +
          std::to_string(layer_box_id) + ": " + layer_reason;
      }
      return false;
    }
  }
  if (reason) reason->clear();
  return true;
}

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
  std::string* reason)
{
  const int column = box_column_from_left(box_id);
  const int row = box_row_from_top(box_id);
  if (column == 0 || row < 0) {
    if (reason) *reason = carried_box_id + " has invalid source box id";
    return false;
  }
  std::vector<int> neighbor_ids;
  if (column > 1) neighbor_ids.push_back(row * kBoxStackColumnCount + column - 1);
  if (column < kBoxStackColumnCount) {
    neighbor_ids.push_back(row * kBoxStackColumnCount + column + 1);
  }

  const auto boxes = make_boxes(box_front_x, scene_y_shift);
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

bool carried_box_clear_obstacles(
  const AxisAlignedBox& carried_box,
  const std::string& carried_box_id,
  const std::vector<StaticBoxObstacle>& static_obstacles,
  const std::vector<ContainerPanel>& container_panels,
  std::string* reason)
{
  for (const auto& obstacle : static_obstacles) {
    const AxisAlignedBox obstacle_aabb{obstacle.center, obstacle.size};
    if (aabb_overlaps(carried_box, obstacle_aabb)) {
      if (reason) *reason = carried_box_id + " overlaps " + obstacle.id;
      return false;
    }
  }

  for (const auto& panel : container_panels) {
    if (std::abs(panel.yaw) < 1e-6) {
      const AxisAlignedBox panel_aabb{panel.center, panel.size};
      if (aabb_overlaps(carried_box, panel_aabb)) {
        if (reason) *reason = carried_box_id + " overlaps " + panel.id;
        return false;
      }
    } else {
      const OrientedBox panel_obb{panel.center, panel.size, panel.yaw};
      if (aabb_overlaps_oriented_box(carried_box, panel_obb)) {
        if (reason) *reason = carried_box_id + " overlaps " + panel.id;
        return false;
      }
    }
  }

  return true;
}

bool carried_box_clear_rear_guards(
  const AxisAlignedBox& carried_box,
  const std::string& carried_box_id,
  const std::vector<StaticBoxObstacle>& static_obstacles,
  std::string* reason)
{
  constexpr const char* suffix = "_rear_guard";
  constexpr size_t suffix_size = 11;
  for (const auto& obstacle : static_obstacles) {
    if (obstacle.id.size() < suffix_size ||
        obstacle.id.compare(obstacle.id.size() - suffix_size, suffix_size, suffix) != 0) {
      continue;
    }
    const AxisAlignedBox obstacle_aabb{obstacle.center, obstacle.size};
    if (aabb_overlaps(carried_box, obstacle_aabb)) {
      if (reason) *reason = carried_box_id + " overlaps " + obstacle.id;
      return false;
    }
  }
  return true;
}

}  // namespace alfa_robot::motion
