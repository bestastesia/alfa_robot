#include "robot_motion_scene_service/motion_scene_adapter.hpp"

#include "robot_motion_scene_service/motion_core/scene_pose.hpp"

#include <shape_msgs/msg/solid_primitive.hpp>

#include <chrono>
#include <thread>

namespace alfa_robot::motion
{

MotionSceneAdapter::MotionSceneAdapter(MotionSceneAdapterConfig config)
: config_(std::move(config)),
  planning_scene_interface_(std::make_unique<moveit::planning_interface::PlanningSceneInterface>())
{
}

std::vector<ContainerPanel> MotionSceneAdapter::containerPanels() const
{
  return make_container_panels(config_.container);
}

moveit_msgs::msg::CollisionObject MotionSceneAdapter::makeCollisionObject(
  const std::string& id,
  const std::array<double, 3>& center,
  const std::array<double, 3>& size,
  int operation) const
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = config_.frame;
  object.id = id;
  object.operation = operation;
  if (operation == moveit_msgs::msg::CollisionObject::ADD) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {size[0], size[1], size[2]};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(make_identity_pose(center[0], center[1], center[2]));
  }
  return object;
}

bool MotionSceneAdapter::applyContainerObstacles()
{
  if (!config_.enable_container_obstacle) return true;
  if (!planning_scene_interface_) return true;

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto& panel : containerPanels()) {
    objects.push_back(makeCollisionObject(
      panel.id,
      panel.center,
      panel.size,
      moveit_msgs::msg::CollisionObject::ADD));
  }
  return planning_scene_interface_->applyCollisionObjects(objects);
}

void MotionSceneAdapter::clearAppliedStaticBoxObstacles()
{
  if (!planning_scene_interface_ || applied_static_box_obstacle_ids_.empty()) return;

  std::vector<moveit_msgs::msg::CollisionObject> remove_objects;
  remove_objects.reserve(applied_static_box_obstacle_ids_.size());
  for (const auto& id : applied_static_box_obstacle_ids_) {
    remove_objects.push_back(makeCollisionObject(
      id,
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      moveit_msgs::msg::CollisionObject::REMOVE));
  }
  planning_scene_interface_->applyCollisionObjects(remove_objects);
  applied_static_box_obstacle_ids_.clear();
}

bool MotionSceneAdapter::applyStaticBoxObstacles()
{
  clearAppliedStaticBoxObstacles();
  if (!config_.enable_static_box_obstacles) {
    current_static_box_obstacles_.clear();
    active_static_left_box_id_ = 0;
    active_static_right_box_id_ = 0;
    return true;
  }
  if (!planning_scene_interface_) return true;

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto& box : current_static_box_obstacles_) {
    objects.push_back(makeCollisionObject(
      box.id,
      box.center,
      box.size,
      moveit_msgs::msg::CollisionObject::ADD));
  }
  if (objects.empty()) return false;

  if (!planning_scene_interface_->applyCollisionObjects(objects)) {
    return false;
  }
  applied_static_box_obstacle_ids_.clear();
  for (const auto& object : objects) {
    applied_static_box_obstacle_ids_.push_back(object.id);
  }
  return true;
}

bool MotionSceneAdapter::setStaticBoxWallOpening(int left_box_id, int right_box_id)
{
  if (!config_.enable_static_box_obstacles) {
    current_static_box_obstacles_.clear();
    active_static_left_box_id_ = 0;
    active_static_right_box_id_ = 0;
    return applyStaticBoxObstacles();
  }

  if (active_static_left_box_id_ == left_box_id &&
      active_static_right_box_id_ == right_box_id &&
      !current_static_box_obstacles_.empty() &&
      !applied_static_box_obstacle_ids_.empty()) {
    return true;
  }

  active_static_left_box_id_ = left_box_id;
  active_static_right_box_id_ = right_box_id;
  current_static_box_obstacles_ =
    make_box_wall_obstacles_for_opening(left_box_id, right_box_id, config_.box_wall);
  return applyStaticBoxObstacles();
}

AttachedBoxSpec MotionSceneAdapter::makeCarriedBoxSpec(
  const std::string& side,
  int box_id,
  bool top_suction) const
{
  return make_attached_box_spec(side, box_id, top_suction, config_.carried_box);
}

void MotionSceneAdapter::setActiveAttachedBoxesForRecordOnly(std::vector<AttachedBoxSpec> boxes)
{
  active_attached_boxes_ = std::move(boxes);
}

void MotionSceneAdapter::clearActiveAttachedBoxesForRecordOnly()
{
  active_attached_boxes_.clear();
}

moveit_msgs::msg::AttachedCollisionObject MotionSceneAdapter::makeAttachedCollisionObject(
  const AttachedBoxSpec& spec,
  int operation) const
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = spec.link_name;
  attached.touch_links = {spec.link_name};
  if (spec.link_name.rfind("left_", 0) == 0) {
    attached.touch_links.push_back("left_v5_link6");
    attached.touch_links.push_back("left_v5_link5");
  } else if (spec.link_name.rfind("right_", 0) == 0) {
    attached.touch_links.push_back("right_v5_link6");
    attached.touch_links.push_back("right_v5_link5");
  }

  attached.object.header.frame_id = spec.link_name;
  attached.object.id = spec.id;
  attached.object.operation = operation;
  if (operation == moveit_msgs::msg::CollisionObject::ADD) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {spec.size[0], spec.size[1], spec.size[2]};
    attached.object.primitives.push_back(primitive);
    attached.object.primitive_poses.push_back(
      make_identity_pose(spec.center_in_link[0], spec.center_in_link[1], spec.center_in_link[2]));
  }
  return attached;
}

bool MotionSceneAdapter::applyAttachedBoxState(
  const std::vector<AttachedBoxSpec>& specs,
  int operation)
{
  if (!config_.enable_attached_box_collision) return true;
  if (!planning_scene_interface_) return true;
  if (specs.empty()) return true;

  std::vector<moveit_msgs::msg::AttachedCollisionObject> objects;
  objects.reserve(specs.size());
  for (const auto& spec : specs) {
    objects.push_back(makeAttachedCollisionObject(spec, operation));
  }
  if (!planning_scene_interface_->applyAttachedCollisionObjects(objects)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}

void MotionSceneAdapter::applyToPlanningSceneSnapshot(
  planning_scene::PlanningScene& scene,
  const std::vector<AttachedBoxSpec>& attached_boxes) const
{
  if (config_.enable_container_obstacle) {
    for (const auto& panel : containerPanels()) {
      scene.processCollisionObjectMsg(makeCollisionObject(
        panel.id,
        panel.center,
        panel.size,
        moveit_msgs::msg::CollisionObject::ADD));
    }
  }

  if (config_.enable_static_box_obstacles) {
    for (const auto& obstacle : current_static_box_obstacles_) {
      scene.processCollisionObjectMsg(makeCollisionObject(
        obstacle.id,
        obstacle.center,
        obstacle.size,
        moveit_msgs::msg::CollisionObject::ADD));
    }
  }

  if (config_.enable_attached_box_collision) {
    for (const auto& spec : attached_boxes) {
      scene.processAttachedCollisionObjectMsg(
        makeAttachedCollisionObject(spec, moveit_msgs::msg::CollisionObject::ADD));
    }
  }
}

bool MotionSceneAdapter::removeCarriedBoxIds(const std::vector<std::string>& ids)
{
  if (!config_.enable_attached_box_collision) return true;
  if (!planning_scene_interface_) return true;
  if (ids.empty()) return true;

  std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_removes;
  attached_removes.reserve(ids.size());
  for (const auto& id : ids) {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = id.find("_right_") != std::string::npos ? config_.right_tip : config_.left_tip;
    attached.object.header.frame_id = attached.link_name;
    attached.object.id = id;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    attached_removes.push_back(attached);
  }

  std::vector<moveit_msgs::msg::CollisionObject> world_removes;
  world_removes.reserve(ids.size());
  for (const auto& id : ids) {
    world_removes.push_back(makeCollisionObject(
      id,
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      moveit_msgs::msg::CollisionObject::REMOVE));
  }

  const bool attached_ok = planning_scene_interface_->applyAttachedCollisionObjects(attached_removes);
  planning_scene_interface_->applyCollisionObjects(world_removes);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return attached_ok;
}

bool MotionSceneAdapter::attachCarriedBoxes(int left_box_id, int right_box_id, bool top_suction)
{
  active_attached_boxes_ = {
    makeCarriedBoxSpec("left", left_box_id, top_suction),
    makeCarriedBoxSpec("right", right_box_id, top_suction),
  };
  if (!applyAttachedBoxState(active_attached_boxes_, moveit_msgs::msg::CollisionObject::ADD)) {
    active_attached_boxes_.clear();
    return false;
  }
  return true;
}

bool MotionSceneAdapter::detachCarriedBoxes()
{
  if (active_attached_boxes_.empty()) return true;
  std::vector<std::string> ids;
  ids.reserve(active_attached_boxes_.size());
  for (const auto& box : active_attached_boxes_) {
    ids.push_back(box.id);
  }
  if (!removeCarriedBoxIds(ids)) {
    return false;
  }
  active_attached_boxes_.clear();
  return true;
}

bool MotionSceneAdapter::clearCarriedBoxes()
{
  if (!config_.enable_attached_box_collision) return true;
  if (active_attached_boxes_.empty()) return true;
  return detachCarriedBoxes();
}

}  // namespace alfa_robot::motion
