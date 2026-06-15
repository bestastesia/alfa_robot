#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

#include <nlohmann/json.hpp>

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

class MotionFlowRecorder
{
public:
  bool open(const std::string& jsonl_path, const nlohmann::json& header, std::string* error = nullptr);

  bool enabled() const { return stream_.is_open(); }
  size_t stageCount() const { return stage_index_; }

  void write(const nlohmann::json& record);

  void recordStage(
    const std::string& stage_name,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const nlohmann::json& start_state,
    const nlohmann::json& goal_state,
    const std::vector<std::string>& target_names,
    const nlohmann::json& attached_boxes,
    const nlohmann::json& static_box_obstacles,
    const nlohmann::json& extra);

private:
  std::ofstream stream_;
  size_t stage_index_ = 0;
};

}  // namespace alfa_robot::motion
