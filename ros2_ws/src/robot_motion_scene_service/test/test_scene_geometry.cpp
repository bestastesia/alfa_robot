#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  using namespace alfa_robot::motion;

  ContainerGeometryConfig container;
  container.center_x = 0.8;
  container.center_y = 0.0;
  container.width = 2.2;
  container.height = 2.4;
  container.length = 4.0;
  container.wall_thickness = 0.02;
  container.floor_z = 0.0;
  const auto panels = make_container_panels(container);
  assert(panels.size() == 3);
  assert(panels[0].id == "container_left_wall");
  assert(panels[1].id == "container_right_wall");
  assert(panels[2].id == "container_ceiling");

  BoxWallGeometryConfig wall;
  wall.box_front_x = 0.925;
  wall.scene_y_shift = -0.4;
  wall.container_center_y = -0.4;
  wall.container_width = 2.2;
  wall.container_floor_z = 0.0;
  const auto obstacles = make_box_wall_obstacles_for_opening(6, 8, wall);
  for (const auto& obstacle : obstacles) {
    if (obstacle.id.find("_below") != std::string::npos) {
      std::cerr << "unexpected below-wall obstacle: " << obstacle.id << std::endl;
      return 1;
    }
  }
  assert(!obstacles.empty());
  bool found_rear_guard = false;
  for (const auto& obstacle : obstacles) {
    if (obstacle.id.find("_rear_guard") != std::string::npos) {
      found_rear_guard = true;
      assert(obstacle.size[0] == wall.rear_guard_thickness);
      const double expected_center_x =
        wall.box_front_x + wall.carried_box_depth + wall.rear_guard_clearance +
        0.5 * wall.rear_guard_thickness;
      assert(std::abs(obstacle.center[0] - expected_center_x) < 1e-9);
      assert(obstacle.size[2] == wall.container_height);
    }
  }
  assert(found_rear_guard);

  const auto left_box = make_attached_box_spec("left", 6, false, CarriedBoxGeometryConfig{});
  assert(left_box.id == "carried_left_box_6");
  assert(left_box.link_name == "left_tool0");
  assert(left_box.size[0] > 0.0);
  assert(left_box.center_in_link[2] > 0.0);

  const auto top_box = make_attached_box_spec("left", 16, true, CarriedBoxGeometryConfig{});
  assert(top_box.id == "carried_left_box_16");
  assert(top_box.link_name == "left_tool0");
  assert(top_box.size[2] == CarriedBoxGeometryConfig{}.carried_box_height);
  assert(top_box.center_in_link[2] > 0.0);

  const auto joint_names = dual_arm_with_updown_joint_names();
  assert(joint_names.size() == 13);
  assert(joint_names.front() == "updown");
  assert(joint_names[1] == "leftjoint1");
  assert(joint_names[6] == "leftjoint6");
  assert(joint_names[7] == "rightjoint1");
  assert(joint_names.back() == "rightjoint6");

  const AxisAlignedBox a{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  const AxisAlignedBox b{{0.4, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  const AxisAlignedBox c{{2.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  assert(aabb_overlaps(a, b));
  assert(!aabb_overlaps(a, c));

  std::string reason;
  reason.clear();
  const AxisAlignedBox source_box{{0.15, 0.0, 0.2}, {0.3, 0.4, 0.4}};
  assert(!carried_box_detached_from_source_xz(
    source_box, source_box, 0.03, "carried_box", &reason));
  assert(reason.find("x-z projection still overlaps source box") != std::string::npos);

  reason.clear();
  const AxisAlignedBox retreated_box{{-0.18, 0.0, 0.2}, {0.3, 0.4, 0.4}};
  assert(carried_box_detached_from_source_xz(
    retreated_box, source_box, 0.03, "carried_box", &reason));
  assert(reason.empty());

  const AxisAlignedBox lifted_box{{0.15, 0.0, 0.63}, {0.3, 0.4, 0.4}};
  assert(carried_box_detached_from_source_xz(
    lifted_box, source_box, 0.03, "carried_box", &reason));

  reason.clear();
  const StaticBoxObstacle static_obstacle{"box_wall", {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}};
  assert(!carried_box_clear_obstacles(a, "carried_box", {static_obstacle}, {}, &reason));
  assert(reason == "carried_box overlaps box_wall");

  reason.clear();
  const ContainerPanel ceiling{"container_ceiling", {0.0, 0.0, 0.45}, {2.0, 2.0, 0.1}};
  assert(!carried_box_clear_obstacles(a, "carried_box", {}, {ceiling}, &reason));
  assert(reason == "carried_box overlaps container_ceiling");

  reason.clear();
  assert(carried_box_clear_obstacles(c, "carried_box", {static_obstacle}, {ceiling}, &reason));
  assert(reason.empty());

  std::cout << "scene geometry smoke passed\n";
  return 0;
}
