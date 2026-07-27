#pragma once

#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

// 车体位姿提供者：返回 global_frame -> config.frame（例如 "map" -> "world"）当前的变换。
// MoveIt 的 planning scene 只认识 robot model 自带的 frame（SRDF virtual_joint 的
// parent_frame，即 "world"），不能把碰撞体直接以 "map" 写入——所以这个回调不用来改写
// frame_id，只用来在 applyContainerObstacles/applyStaticBoxObstacles 成功时记录车体
// 当时在全局系下的位姿，供后续判断车体是否已经挪动、旧的相对坐标碰撞几何是否过期。
using VehiclePoseProvider = std::function<Eigen::Isometry3d()>;

struct MotionSceneAdapterConfig
{
  bool enable_container_obstacle = true;
  std::string frame = "world";
  ContainerGeometryConfig container;

  bool enable_static_box_obstacles = true;
  BoxWallGeometryConfig box_wall;

  bool enable_attached_box_collision = true;
  CarriedBoxGeometryConfig carried_box;

  std::string left_tip = "left_tool0";
  std::string right_tip = "right_tool0";

  // 未设置时不做任何车体位姿关联跟踪，行为与改造前完全一致。
  VehiclePoseProvider vehicle_pose_provider;
};

class MotionSceneAdapter
{
public:
  explicit MotionSceneAdapter(MotionSceneAdapterConfig config);

  const MotionSceneAdapterConfig& config() const { return config_; }

  std::vector<ContainerPanel> containerPanels() const;

  // 更新集装箱几何配置（供"集装箱相对车体动态位姿"这类需要在运行期刷新的场景使用）。
  // 只替换 config_.container，其余配置（frame、box_wall、vehicle_pose_provider 等）不变。
  // 调用后需要重新调 applyContainerObstacles() 才会真正写入 planning scene。
  void updateContainerGeometry(ContainerGeometryConfig container);

  // 更新箱墙几何，并使当前 opening 缓存失效。随后调用
  // setStaticBoxWallOpening() 会删除旧碰撞体并按新几何重建。
  void updateBoxWallGeometry(BoxWallGeometryConfig box_wall);

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

  void applyToPlanningSceneSnapshot(
    planning_scene::PlanningScene& scene,
    const std::vector<AttachedBoxSpec>& attached_boxes) const;

  // 车体相对当前已写入的静态障碍物（container/box wall）是否已经挪动超过阈值。
  // 没有配置 vehicle_pose_provider，或从未成功 apply 过任何静态障碍物时返回 false
  // （无法判断，视为不过期，维持改造前的行为）。
  bool staticObstaclesStale(
    double translation_threshold_m = 0.05,
    double rotation_threshold_rad = 0.02) const;

  bool removeCarriedBoxIds(const std::vector<std::string>& ids);

  bool attachCarriedBoxes(int left_box_id, int right_box_id, bool top_suction);

  bool detachCarriedBoxes();

  bool clearCarriedBoxes();

private:
  moveit_msgs::msg::CollisionObject makeCollisionObject(
    const std::string& id,
    const std::array<double, 3>& center,
    const std::array<double, 3>& size,
    int operation,
    double yaw = 0.0) const;

  // 成功 apply 静态障碍物后调用：若配置了 vehicle_pose_provider，记录车体当时的全局位姿。
  void recordVehiclePoseAtApply();

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
  std::optional<Eigen::Isometry3d> vehicle_pose_at_last_apply_;
};

}  // namespace alfa_robot::motion
