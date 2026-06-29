#include "alfa_robot_moveit_config/motion_flow_recorder.hpp"

#include <rclcpp/duration.hpp>

#include <filesystem>

namespace alfa_robot::motion
{

nlohmann::json motion_flow_header_json(const MotionFlowHeaderRequest& request)
{
  return {
    {"type", "header"},
    {"schema", "moveit_box_stack_flow_v1"},
    {"ik_strategy", "fixed_discrete_h_multi_seed_cost_scorer"},
    {"planning_group", request.planning_group},
    {"box_front_x", request.box_front_x},
    {"scene_y_shift", request.scene_y_shift},
    {"world_to_base_z", request.world_to_base_z},
    {"fixed_updown", request.fixed_updown},
    {"velocity_scale", request.velocity_scale},
    {"acceleration_scale", request.acceleration_scale},
    {"max_rounds", request.max_rounds},
    {"include_top_suction", request.include_top_suction},
    {"execute", request.execute},
    {"container_obstacle", request.container_obstacle},
    {"static_box_obstacles", request.static_box_obstacles},
    {"attached_box_collision", request.attached_box_collision},
    {"loaded_pose_family", request.loaded_pose_family},
    {"ik_config", request.ik_config}
  };
}

bool MotionFlowRecorder::open(
  const std::string& jsonl_path,
  const nlohmann::json& header,
  std::string* error)
{
  if (jsonl_path.empty()) return false;

  const std::filesystem::path path(jsonl_path);
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  stream_.open(path, std::ios::out | std::ios::trunc);
  if (!stream_) {
    if (error) {
      *error = "failed to open " + jsonl_path;
    }
    return false;
  }

  stream_ << header.dump() << '\n';
  stream_.flush();
  return true;
}

void MotionFlowRecorder::write(const nlohmann::json& record)
{
  if (!stream_) return;
  stream_ << record.dump() << '\n';
  stream_.flush();
}

void MotionFlowRecorder::recordStage(
  const std::string& stage_name,
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const nlohmann::json& start_state,
  const nlohmann::json& goal_state,
  const std::vector<std::string>& target_names,
  const nlohmann::json& attached_boxes,
  const nlohmann::json& static_box_obstacles,
  const nlohmann::json& extra)
{
  if (!stream_) return;

  const auto& traj = plan.trajectory_.joint_trajectory;
  nlohmann::json points = nlohmann::json::array();
  for (const auto& point : traj.points) {
    points.push_back({
      {"time_from_start_sec", rclcpp::Duration(point.time_from_start).seconds()},
      {"positions", point.positions},
      {"velocities", point.velocities}
    });
  }

  nlohmann::json record = {
    {"type", "stage"},
    {"stage", stage_name},
    {"stage_index", stage_index_++},
    {"trajectory", {
      {"joint_names", traj.joint_names},
      {"point_count", traj.points.size()},
      {"points", points}
    }},
    {"target_names", target_names},
    {"start_state", start_state},
    {"goal_state", goal_state},
    {"attached_boxes", attached_boxes},
    {"static_box_obstacles", static_box_obstacles},
    {"extra", extra}
  };
  write(record);
}

}  // namespace alfa_robot::motion
