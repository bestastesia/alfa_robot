#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  using namespace alfa_robot::motion;

  assert(trim_copy("  2,4 \n") == "2,4");
  assert(trim_copy(" \t\r\n").empty());

  const auto boxes = make_boxes(0.925, 0.0);
  assert(boxes.size() == 15);
  assert(std::abs(boxes.at(1).x - 0.925) < 1e-9);
  assert(std::abs(boxes.at(1).y - 0.5) < 1e-9);
  assert(std::abs(boxes.at(1).z - 1.8) < 1e-9);
  assert(std::abs(boxes.at(5).y) < 1e-9);
  assert(std::abs(boxes.at(5).z - 1.4) < 1e-9);
  assert(std::abs(boxes.at(15).y - (-0.5)) < 1e-9);
  assert(std::abs(boxes.at(15).z - 0.2) < 1e-9);
  assert(std::abs(kOuterBoxGraspTargetY - 0.50) < 1e-9);
  assert(std::abs(kOuterBoxGraspLateralOffset) < 1e-9);
  assert(box_column_from_left(1) == 1);
  assert(box_column_from_left(2) == 2);
  assert(box_column_from_left(3) == 3);
  assert(box_row_from_top(10) == 3);
  assert(box_column_from_left(13) == 1);
  assert(box_column_from_left(16) == 0);
  assert(box_row_from_top(0) == -1);

  const auto parsed = parse_box_pair_list(" 2,4 ; 7/9; ;12,14 ");
  assert(parsed.size() == 3);
  assert(parsed[0] == std::make_pair(2, 4));
  assert(parsed[1] == std::make_pair(7, 9));
  assert(parsed[2] == std::make_pair(12, 14));

  const auto front_pairs = make_pick_pairs(false, parsed);
  assert(front_pairs.size() == 3);
  assert(front_pairs[0].round == 1);
  assert(front_pairs[0].left_box == 2);
  assert(front_pairs[0].right_box == 4);
  assert(!front_pairs[0].top_suction);
  assert(front_pairs[2].round == 3);

  const auto with_top = make_pick_pairs(true, parsed);
  assert(with_top.size() == 4);
  assert(with_top.back().round == 4);
  assert(with_top.back().left_box == 13);
  assert(with_top.back().right_box == 15);
  assert(with_top.back().top_suction);

  const auto joint_names = dual_arm_with_updown_joint_names();
  assert(joint_names.size() == 13);
  assert(joint_names[0] == "updown");
  assert(joint_names[1] == "left_joint1");
  assert(joint_names[6] == "left_joint6");
  assert(joint_names[7] == "right_joint1");
  assert(joint_names[12] == "right_joint6");

  std::cout << "task geometry smoke passed\n";
  return 0;
}
