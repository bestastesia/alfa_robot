#pragma once

#include "alfa_robot_moveit_config/motion_core/scene_geometry.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <memory>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct MotionSceneAdapterConfig
{
  bool enable_container_obstacle = true;
  std::string frame = "world";
  ContainerGeometryConfig container;

  bool enable_static_box_obstacles = true;
  BoxWallGeometryConfig box_wall;

  bool enable_attached_box_collision = true;
  CarriedBoxGeometryConfig carried_box;

  std::string left_tip = "left_v5_tool0";
  std::string right_tip = "right_v5_tool0";
};

class MotionSceneAdapter
{
public:
  explicit MotionSceneAdapter(MotionSceneAdapterConfig config);

  const MotionSceneAdapterConfig& config() const { return config_; }

  std::vector<ContainerPanel> containerPanels() const;

  bool applyContainerObstacles();

  bool setStaticBoxWallOpening(int left_box_id, int right_box_id);

  const std::vector<StaticBoxObstacle>& staticBoxObstacles() const { return current_static_box_obstacles_; }

  int activeStaticLeftBoxId() const { return active_static_left_box_id_; }

  int activeStaticRightBoxId() const { return active_static_right_box_id_; }

  AttachedBoxSpec makeCarriedBoxSpec(
    const std::string& side,
    int box_id,
    bool top_suction) const;

  const std::vector<AttachedBoxSpec>& activeAttachedBoxes() const { return active_attached_boxes_; }

  void setActiveAttachedBoxesForRecordOnly(std::vector<AttachedBoxSpec> boxes);

  void clearActiveAttachedBoxesForRecordOnly();

  bool applyAttachedBoxState(
    const std::vector<AttachedBoxSpec>& specs,
    int operation);

  bool removeCarriedBoxIds(const std::vector<std::string>& ids);

  bool attachCarriedBoxes(int left_box_id, int right_box_id, bool top_suction);

  bool detachCarriedBoxes();

  bool clearCarriedBoxes();

private:
  moveit_msgs::msg::CollisionObject makeCollisionObject(
    const std::string& id,
    const std::array<double, 3>& center,
    const std::array<double, 3>& size,
    int operation) const;

  moveit_msgs::msg::AttachedCollisionObject makeAttachedCollisionObject(
    const AttachedBoxSpec& spec,
    int operation) const;

  void clearAppliedStaticBoxObstacles();

  bool applyStaticBoxObstacles();

  MotionSceneAdapterConfig config_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::vector<AttachedBoxSpec> active_attached_boxes_;
  std::vector<StaticBoxObstacle> current_static_box_obstacles_;
  std::vector<std::string> applied_static_box_obstacle_ids_;
  int active_static_left_box_id_ = 0;
  int active_static_right_box_id_ = 0;
};

}  // namespace alfa_robot::motion
