#include "alfa_robot_moveit_config/planning_diagnostics.hpp"
#include "alfa_robot_moveit_config/runtime_status_publisher.hpp"
#include "motion_internal_interfaces/msg/attached_box.hpp"
#include "motion_internal_interfaces/srv/check_collision.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{

using CheckCollision = motion_internal_interfaces::srv::CheckCollision;

moveit_msgs::msg::AttachedCollisionObject to_attached_collision_object(
  const motion_internal_interfaces::msg::AttachedBox& box)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = box.link_name;
  attached.touch_links = box.touch_links;
  attached.object.header.frame_id = box.link_name;
  attached.object.id = box.id.empty() ? ("attached_box_" + std::to_string(box.box_id)) : box.id;
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = box.size.x;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = box.size.y;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = box.size.z;
  attached.object.primitives.push_back(primitive);
  attached.object.primitive_poses.push_back(box.center_in_link);
  return attached;
}

bool set_joint_state_positions(
  const sensor_msgs::msg::JointState& joint_state,
  moveit::core::RobotState& robot_state,
  std::string* reason)
{
  if (joint_state.name.empty()) {
    if (reason) *reason = "empty_joint_state";
    return false;
  }
  if (joint_state.position.size() < joint_state.name.size()) {
    if (reason) *reason = "joint_state_position_size_mismatch";
    return false;
  }
  const auto model = robot_state.getRobotModel();
  const auto& variable_names = model->getVariableNames();
  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    const auto& name = joint_state.name[i];
    if (std::find(variable_names.begin(), variable_names.end(), name) == variable_names.end()) {
      if (reason) *reason = "unknown_joint:" + name;
      return false;
    }
    robot_state.setVariablePosition(name, joint_state.position[i]);
  }
  robot_state.update(true);
  return true;
}

std::vector<std::string> contacts_from_reason(const std::string& reason)
{
  std::vector<std::string> contacts;
  const auto marker = reason.find(':');
  if (marker == std::string::npos || marker + 1 >= reason.size()) {
    return contacts;
  }
  std::string rest = reason.substr(marker + 1);
  std::stringstream stream(rest);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) contacts.push_back(item);
  }
  return contacts;
}

}  // namespace

class MotionCollisionServiceNode : public rclcpp::Node
{
public:
  MotionCollisionServiceNode()
  : Node("motion_collision_service")
  {
    service_name_ = declare_parameter<std::string>("service_name", "/robot_motion/check_collision");
    joint_group_name_ = declare_parameter<std::string>("joint_group", "dual_arm_with_base");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
  }

  void initialize()
  {
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(),
      "robot_description");
    if (planning_scene_monitor_->getPlanningScene()) {
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startStateMonitor(joint_state_topic_);
      planning_scene_monitor_->requestPlanningSceneState();
      robot_model_ = planning_scene_monitor_->getRobotModel();
      joint_group_ = robot_model_ ? robot_model_->getJointModelGroup(joint_group_name_) : nullptr;
    }

    if (!robot_model_) {
      throw std::runtime_error("motion_collision_service failed to load robot model");
    }

    service_ = create_service<CheckCollision>(
      service_name_,
      std::bind(&MotionCollisionServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    status_ = std::make_shared<alfa_robot::motion::RuntimeStatusPublisher>(
      *this,
      service_name_,
      "collision_check");

    RCLCPP_INFO(
      get_logger(),
      "Motion collision service ready: service=%s joint_group=%s joint_state_topic=%s",
      service_name_.c_str(),
      joint_group_name_.c_str(),
      joint_state_topic_.c_str());
    status_->mark_ready("joint_group=" + joint_group_name_);
  }

private:
  planning_scene::PlanningScenePtr make_scene_snapshot(
    const std::vector<moveit_msgs::msg::CollisionObject>& scene_objects,
    const std::vector<moveit_msgs::msg::AttachedCollisionObject>& attached_objects,
    const std::vector<motion_internal_interfaces::msg::AttachedBox>& attached_boxes,
    bool use_current_scene_as_base) const
  {
    planning_scene::PlanningScenePtr scene;
    if (use_current_scene_as_base && planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene()) {
      planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
      if (locked_scene) {
        scene = planning_scene::PlanningScene::clone(
          static_cast<const planning_scene::PlanningSceneConstPtr&>(locked_scene));
      }
    }
    if (!scene) {
      scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    }

    for (const auto& object : scene_objects) {
      scene->processCollisionObjectMsg(object);
    }
    for (const auto& object : attached_objects) {
      scene->processAttachedCollisionObjectMsg(object);
    }
    for (const auto& box : attached_boxes) {
      scene->processAttachedCollisionObjectMsg(to_attached_collision_object(box));
    }
    return scene;
  }

  bool check_state(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& state,
    bool enforce_bounds,
    std::string* reason) const
  {
    if (enforce_bounds) {
      const std::string bounds = alfa_robot::motion::group_bounds_reason(state, joint_group_);
      if (!bounds.empty()) {
        if (reason) *reason = bounds;
        return false;
      }
    }
    const std::string collision = alfa_robot::motion::scene_collision_reason(scene, state, joint_group_);
    if (!collision.empty()) {
      if (reason) *reason = collision;
      return false;
    }
    return true;
  }

  void handle_request(
    const std::shared_ptr<CheckCollision::Request> request,
    std::shared_ptr<CheckCollision::Response> response)
  {
    status_->mark_running(
      "trajectory_points=" + std::to_string(request->trajectory.points.size()) +
      " scene_objects=" + std::to_string(request->scene_objects.size()) +
      " attached_collision_objects=" + std::to_string(request->attached_collision_objects.size()) +
      " attached_boxes=" + std::to_string(request->attached_boxes.size()));
    auto scene = make_scene_snapshot(
      request->scene_objects,
      request->attached_collision_objects,
      request->attached_boxes,
      request->use_current_scene_as_base);

    moveit::core::RobotState start_state(robot_model_);
    start_state.setToDefaultValues();
    std::string reason;
    if (!set_joint_state_positions(request->start_state, start_state, &reason)) {
      response->valid = false;
      response->reason = reason;
      status_->mark_done(false, response->reason);
      return;
    }
    scene->setCurrentState(start_state);

    if (request->trajectory.points.empty()) {
      response->valid = check_state(scene, start_state, request->enforce_bounds, &reason);
      response->reason = response->valid ? "" : reason;
      response->contacts = contacts_from_reason(response->reason);
      status_->mark_done(response->valid, response->valid ? "state_valid" : response->reason);
      return;
    }

    for (size_t point_index = 0; point_index < request->trajectory.points.size(); ++point_index) {
      moveit::core::RobotState state(start_state);
      const auto& point = request->trajectory.points[point_index];
      const auto& variable_names = robot_model_->getVariableNames();
      if (point.positions.size() < request->trajectory.joint_names.size()) {
        response->valid = false;
        response->reason = "trajectory_position_size_mismatch@" + std::to_string(point_index);
        status_->mark_done(false, response->reason);
        return;
      }
      for (size_t i = 0; i < request->trajectory.joint_names.size(); ++i) {
        const auto& name = request->trajectory.joint_names[i];
        if (std::find(variable_names.begin(), variable_names.end(), name) == variable_names.end()) {
          response->valid = false;
          response->reason = "unknown_trajectory_joint:" + name;
          status_->mark_done(false, response->reason);
          return;
        }
        state.setVariablePosition(name, point.positions[i]);
      }
      state.update(true);
      if (!check_state(scene, state, request->enforce_bounds, &reason)) {
        response->valid = false;
        response->reason = "trajectory_point_" + std::to_string(point_index) + ":" + reason;
        response->contacts = contacts_from_reason(reason);
        status_->mark_done(false, response->reason);
        return;
      }
    }

    response->valid = true;
    response->reason.clear();
    response->contacts.clear();
    status_->mark_done(true, "trajectory_valid points=" + std::to_string(request->trajectory.points.size()));
  }

  std::string service_name_;
  std::string joint_group_name_;
  std::string joint_state_topic_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::Service<CheckCollision>::SharedPtr service_;
  std::shared_ptr<alfa_robot::motion::RuntimeStatusPublisher> status_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionCollisionServiceNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
