#include "alfa_robot_moveit_config/motion_flow_recorder.hpp"

#include <cassert>

int main()
{
  alfa_robot::motion::MotionFlowHeaderRequest request;
  request.planning_group = "dual_v5_arm_with_base";
  request.box_front_x = 0.925;
  request.scene_y_shift = -0.4;
  request.world_to_base_z = 0.202;
  request.fixed_updown = 0.3;
  request.velocity_scale = 0.8;
  request.acceleration_scale = 0.7;
  request.max_rounds = 6;
  request.include_top_suction = true;
  request.execute = false;
  request.container_obstacle = {{"enabled", true}};
  request.static_box_obstacles = {{"opening_left_box_id", 6}};
  request.attached_box_collision = {{"depth", 0.3}};
  request.loaded_pose_family = {{"left_preferred_index", 1}};
  request.ik_config = {{"workers", 16}, {"h_search_mode", "fixed_discrete"}};

  const auto header = alfa_robot::motion::motion_flow_header_json(request);
  assert(header.at("type") == "header");
  assert(header.at("schema") == "moveit_box_stack_flow_v1");
  assert(header.at("ik_strategy") == "fixed_discrete_h_multi_seed_cost_scorer");
  assert(header.at("planning_group") == "dual_v5_arm_with_base");
  assert(header.at("box_front_x") == 0.925);
  assert(header.at("scene_y_shift") == -0.4);
  assert(header.at("world_to_base_z") == 0.202);
  assert(header.at("fixed_updown") == 0.3);
  assert(header.at("velocity_scale") == 0.8);
  assert(header.at("acceleration_scale") == 0.7);
  assert(header.at("max_rounds") == 6);
  assert(header.at("include_top_suction") == true);
  assert(header.at("execute") == false);
  assert(header.at("container_obstacle").at("enabled") == true);
  assert(header.at("static_box_obstacles").at("opening_left_box_id") == 6);
  assert(header.at("attached_box_collision").at("depth") == 0.3);
  assert(header.at("loaded_pose_family").at("left_preferred_index") == 1);
  assert(header.at("ik_config").at("workers") == 16);
  assert(header.at("ik_config").at("h_search_mode") == "fixed_discrete");
  return 0;
}
