#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include <cassert>
#include <cmath>

namespace
{

void assert_near(double lhs, double rhs)
{
  assert(std::abs(lhs - rhs) < 1e-8);
}

}  // namespace

int main()
{
  using alfa_robot::motion::BoxSpec;
  using alfa_robot::motion::make_front_grasp_pose;
  using alfa_robot::motion::make_top_suction_pose;

  BoxSpec box;
  box.id = 8;
  box.x = 0.925;
  box.y = -0.4;
  box.z = 1.0;

  const auto front = make_front_grasp_pose(box, 0.202);
  assert_near(front.position.x, 0.925);
  assert_near(front.position.y, -0.4);
  assert_near(front.position.z, 0.798);
  assert_near(front.orientation.w, 0.70710678);
  assert_near(front.orientation.x, 0.0);
  assert_near(front.orientation.y, 0.70710678);
  assert_near(front.orientation.z, 0.0);

  const auto top = make_top_suction_pose(box, 0.202, 0.15, 0.209);
  assert_near(top.position.x, 1.075);
  assert_near(top.position.y, -0.4);
  assert_near(top.position.z, 1.007);
  assert_near(top.orientation.w, 0.0);
  assert_near(top.orientation.x, 0.0);
  assert_near(top.orientation.y, 1.0);
  assert_near(top.orientation.z, 0.0);

  return 0;
}
