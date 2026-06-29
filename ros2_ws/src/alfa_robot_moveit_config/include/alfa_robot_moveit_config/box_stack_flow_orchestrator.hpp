#pragma once

#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <functional>
#include <string>
#include <vector>

namespace alfa_robot::motion
{

struct BoxStackFlowCallbacks
{
  std::function<void()> clear_scene;
  std::function<bool(int, int, const std::string&)> set_wall_opening;
  std::function<bool(const std::string&, double, const std::vector<double>&, const std::vector<double>&)> plan_joint_target;
  std::function<bool(const std::string&, const BoxSpec&, const BoxSpec&, bool)> plan_grasp_ik;
  std::function<bool(int, int, bool)> attach_boxes;
  std::function<bool(const std::string&)> validate_attached_boxes;
  std::function<bool()> detach_boxes;
  std::function<void(const std::string&)> info;
  std::function<bool(const std::string&)> fail;
};

struct BoxStackFlowConfig
{
  double box_front_x = 0.625;
  double scene_y_shift = 0.0;
  double fixed_updown = 0.0;
  bool include_top_suction = true;
  int max_rounds = 10;
  std::vector<double> left_pregrasp_arm;
  std::vector<double> right_pregrasp_arm;
  std::vector<double> left_loaded_arm;
  std::vector<double> right_loaded_arm;
  std::vector<std::pair<int, int>> pair_sequence;
};

class BoxStackFlowOrchestrator
{
public:
  BoxStackFlowOrchestrator(BoxStackFlowConfig config, BoxStackFlowCallbacks callbacks);

  bool run();
  bool runOnePair(int left_box_id, int right_box_id, bool top_suction, int round);

private:
  BoxStackFlowConfig config_;
  BoxStackFlowCallbacks callbacks_;
};

}  // namespace alfa_robot::motion
