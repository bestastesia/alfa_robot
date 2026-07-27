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
  container.width = 1.8;
  container.height = 2.4;
  container.length = 4.0;
  container.wall_thickness = 0.02;
  container.floor_z = 0.0;
  const auto panels = make_container_panels(container);
  assert(panels.size() == 3);
  assert(panels[0].id == "container_left_wall");
  assert(panels[1].id == "container_right_wall");
  assert(panels[2].id == "container_ceiling");
  assert(panels[0].yaw == 0.0);

  // yaw != 0 时，墙板中心应绕 (center_x, center_y) 旋转；ceiling 本身在旋转中心正上方，
  // 中心位置不受旋转影响，但 yaw 字段仍应写入。
  ContainerGeometryConfig rotated_container = container;
  rotated_container.yaw = M_PI_2;  // 90 度
  const auto rotated_panels = make_container_panels(rotated_container);
  const double half_width = container.width * 0.5 + container.wall_thickness * 0.5;
  // 未旋转时 left_wall 中心在 (center_x, center_y + half_width)；绕 (center_x, center_y)
  // 转 90 度后应变成 (center_x - half_width, center_y)。
  assert(std::abs(rotated_panels[0].center[0] - (container.center_x - half_width)) < 1e-9);
  assert(std::abs(rotated_panels[0].center[1] - container.center_y) < 1e-9);
  assert(std::abs(rotated_panels[0].yaw - M_PI_2) < 1e-9);
  assert(std::abs(rotated_panels[2].center[0] - container.center_x) < 1e-9);
  assert(std::abs(rotated_panels[2].center[1] - container.center_y) < 1e-9);

  BoxWallGeometryConfig wall;
  wall.box_front_x = 0.925;
  wall.scene_y_shift = 0.0;
  wall.container_center_y = 0.0;
  wall.container_width = 1.8;
  wall.container_floor_z = 0.0;
  const auto obstacles = make_box_wall_obstacles_for_opening(4, 6, wall);
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
      assert(std::abs(obstacle.size[0] - wall.rear_guard_thickness) < 1e-9);
      const double expected_center_x =
        wall.box_front_x + wall.carried_box_depth + wall.rear_guard_clearance +
        0.5 * wall.rear_guard_thickness;
      assert(std::abs(obstacle.center[0] - expected_center_x) < 1e-9);
      assert(std::abs(obstacle.center[2] - 1.0) < 1e-9);
      assert(std::abs(obstacle.size[2] - 2.0) < 1e-9);
    }
  }
  assert(found_rear_guard);

  const auto left_box = make_attached_box_spec("left", 4, false, CarriedBoxGeometryConfig{});
  assert(left_box.id == "carried_left_box_4");
  assert(left_box.link_name == "left_tool0");
  assert(left_box.size[0] == CarriedBoxGeometryConfig{}.carried_box_height);
  assert(left_box.size[1] == CarriedBoxGeometryConfig{}.carried_box_width);
  assert(left_box.size[2] == CarriedBoxGeometryConfig{}.carried_box_depth);
  assert(std::abs(
    left_box.center_in_link[1] + CarriedBoxGeometryConfig{}.grasp_lateral_offset) < 1e-9);
  assert(left_box.center_in_link[2] > 0.0);

  const auto top_box = make_attached_box_spec("left", 10, true, CarriedBoxGeometryConfig{});
  assert(top_box.id == "carried_left_box_10");
  assert(top_box.link_name == "left_tool0");
  assert(top_box.size[2] == CarriedBoxGeometryConfig{}.carried_box_height);
  assert(std::abs(
    top_box.center_in_link[1] + CarriedBoxGeometryConfig{}.grasp_lateral_offset) < 1e-9);
  assert(top_box.center_in_link[2] > 0.0);

  Eigen::Isometry3d rotated_box_transform = Eigen::Isometry3d::Identity();
  rotated_box_transform.linear() =
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto rotated_top_box = aabb_from_attached_box_transform(rotated_box_transform, top_box);
  assert(std::abs(rotated_top_box.size[0] - top_box.size[2]) < 1e-9);
  assert(std::abs(rotated_top_box.size[2] - top_box.size[0]) < 1e-9);

  const auto joint_names = dual_arm_with_updown_joint_names();
  assert(joint_names.size() == 13);
  assert(joint_names.front() == "updown");
  assert(joint_names[1] == "left_joint1");
  assert(joint_names[6] == "left_joint6");
  assert(joint_names[7] == "right_joint1");
  assert(joint_names.back() == "right_joint6");

  const AxisAlignedBox a{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  const AxisAlignedBox b{{0.4, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  const AxisAlignedBox c{{2.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  assert(aabb_overlaps(a, b));
  assert(!aabb_overlaps(a, c));

  std::string reason;
  reason.clear();
  const AxisAlignedBox source_box{{0.15, 0.0, 0.25}, {0.3, 0.4, 0.5}};
  assert(!carried_box_detached_from_source_xz(
    source_box, source_box, 0.03, "carried_box", &reason));
  assert(reason.find("x-z projection still overlaps source box") != std::string::npos);

  reason.clear();
  const AxisAlignedBox retreated_box{{-0.18, 0.0, 0.25}, {0.3, 0.4, 0.5}};
  assert(carried_box_detached_from_source_xz(
    retreated_box, source_box, 0.03, "carried_box", &reason));
  assert(reason.empty());

  const AxisAlignedBox lifted_box{{0.15, 0.0, 0.78}, {0.3, 0.4, 0.5}};
  assert(carried_box_detached_from_source_xz(
    lifted_box, source_box, 0.03, "carried_box", &reason));

  // 不等高侧吸：矮侧不仅要离开自己的原始侧面投影，还要离开正上方一层箱子的侧面投影。
  // box 7 的中心 z=0.75，正上方 box 4 的中心 z=1.25。
  const AxisAlignedBox lower_front_box_lifted_into_upper_layer{
    {0.15, 0.0, 1.29}, {0.3, 0.4, 0.5}};
  reason.clear();
  assert(carried_box_detached_from_source_layers_xz(
    lower_front_box_lifted_into_upper_layer,
    7,
    0.0,
    0.0,
    0.4,
    0.5,
    0.3,
    0.03,
    1,
    "carried_box_7",
    &reason));
  reason.clear();
  assert(!carried_box_detached_from_source_layers_xz(
    lower_front_box_lifted_into_upper_layer,
    7,
    0.0,
    0.0,
    0.4,
    0.5,
    0.3,
    0.03,
    2,
    "carried_box_7",
    &reason));
  assert(reason.find("source layer box 4") != std::string::npos);

  const AxisAlignedBox lower_front_box_retreated{{-0.18, 0.0, 0.75}, {0.3, 0.4, 0.5}};
  reason.clear();
  assert(carried_box_detached_from_source_layers_xz(
    lower_front_box_retreated,
    7,
    0.0,
    0.0,
    0.4,
    0.5,
    0.3,
    0.03,
    2,
    "carried_box_7",
    &reason));

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

  const StaticBoxObstacle rear_guard{
    "box_wall_L4_R6_rear_guard", {0.5, 0.0, 0.0}, {0.02, 1.5, 2.0}};
  reason.clear();
  assert(!carried_box_clear_rear_guards(a, "carried_box", {static_obstacle, rear_guard}, &reason));
  assert(reason == "carried_box overlaps box_wall_L4_R6_rear_guard");

  reason.clear();
  assert(carried_box_clear_rear_guards(c, "carried_box", {static_obstacle, rear_guard}, &reason));
  assert(reason.empty());

  reason.clear();
  assert(carried_box_clear_rear_guards(a, "carried_box", {static_obstacle}, &reason));
  assert(reason.empty());

  // aabb_overlaps_oriented_box: yaw=0 时应与 aabb_overlaps 数值一致（回归保护）。
  {
    const OrientedBox obb_zero_yaw{b.center, b.size, 0.0};
    assert(aabb_overlaps_oriented_box(a, obb_zero_yaw) == aabb_overlaps(a, b));
    const OrientedBox obb_zero_yaw_far{c.center, c.size, 0.0};
    assert(aabb_overlaps_oriented_box(a, obb_zero_yaw_far) == aabb_overlaps(a, c));
  }

  // 一个绕 Z 轴转 45 度的 1x1x1 方块，中心在 (1.21, 0, 0)：旋转后对角线沿 X 轴伸展到
  // 约 1.21 - 1/sqrt(2) ≈ 0.503，与轴对齐时 [0.71, 1.71] 相比会更靠近原点，
  // 但仍不会碰到中心在原点、半宽 0.5 的 aabb（覆盖 [-0.5,0.5]）；
  // 但把 obb 中心拉近到 (0.9, 0, 0) 后，旋转后的角点会伸入 aabb 范围，应判定重叠。
  {
    const AxisAlignedBox unit_aabb{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
    const OrientedBox far_rotated{{1.21, 0.0, 0.0}, {1.0, 1.0, 1.0}, M_PI_4};
    assert(!aabb_overlaps_oriented_box(unit_aabb, far_rotated));
    const OrientedBox near_rotated{{0.9, 0.0, 0.0}, {1.0, 1.0, 1.0}, M_PI_4};
    assert(aabb_overlaps_oriented_box(unit_aabb, near_rotated));
  }

  // Z 方向不受 yaw 影响：X-Y 平面完全重合但 Z 方向分离时应判定不重叠。
  {
    const AxisAlignedBox low_box{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
    const OrientedBox high_obb{{0.0, 0.0, 5.0}, {1.0, 1.0, 1.0}, M_PI_4};
    assert(!aabb_overlaps_oriented_box(low_box, high_obb));
  }

  // carried_box_clear_obstacles: panel 带 yaw 时应走 OBB 判定路径而不是把 yaw 当 0 处理。
  {
    reason.clear();
    const ContainerPanel rotated_ceiling{
      "container_ceiling_rotated", {1.2, 0.0, 0.0}, {1.0, 1.0, 1.0}, M_PI_4};
    assert(!carried_box_clear_obstacles(a, "carried_box", {}, {rotated_ceiling}, &reason));
    assert(reason == "carried_box overlaps container_ceiling_rotated");

    reason.clear();
    const ContainerPanel far_rotated_ceiling{
      "container_ceiling_far", {2.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, M_PI_4};
    assert(carried_box_clear_obstacles(a, "carried_box", {}, {far_rotated_ceiling}, &reason));
    assert(reason.empty());
  }

  // compute_container_pose_relative_to_vehicle: 车体在原点不转时，结果应等于集装箱的
  // map 绝对坐标（向后兼容基准，对应改动前 container_center_x/y 语义）。
  {
    const auto identity_result = compute_container_pose_relative_to_vehicle(
      Eigen::Isometry3d::Identity(), 0.8, 0.3, 0.1);
    assert(std::abs(identity_result.x - 0.8) < 1e-9);
    assert(std::abs(identity_result.y - 0.3) < 1e-9);
    assert(std::abs(identity_result.yaw - 0.1) < 1e-9);
  }

  // 车体平移 (1.0, 0.0) 且转 90 度时：集装箱在 map 下位于车体正前方 (2.0, 0.0)，
  // 相对车体系应该在车体的左侧方向（车体转 90 度后，原来的 +X 变成车体系下的 +Y）。
  {
    Eigen::Isometry3d vehicle_pose_map = Eigen::Isometry3d::Identity();
    vehicle_pose_map.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
    vehicle_pose_map.linear() =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto result = compute_container_pose_relative_to_vehicle(
      vehicle_pose_map, 2.0, 0.0, M_PI_2);
    assert(std::abs(result.x - 0.0) < 1e-9);
    assert(std::abs(result.y - (-1.0)) < 1e-9);
    assert(std::abs(result.yaw - 0.0) < 1e-9);
  }

  std::cout << "scene geometry smoke passed\n";
  return 0;
}
