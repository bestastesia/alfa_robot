// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License

#include <gtest/gtest.h>
#include <cmath>

#include "alfa_robot_hardware/joint/wheel_joint.hpp"
// Directly compile implementation until CMakeLists links alfa_robot_hardware (Task 8)
#include "../src/joint/wheel_joint.cpp"  // NOLINT(build/include)

namespace alfa_robot_hardware
{

TEST(WheelJointTest, VelocityModeExportsVelocityCommand)
{
  WheelJoint joint("left_back", WheelJoint::ControlMode::Velocity);
  auto ci = joint.exportCommandInterfaces();
  ASSERT_EQ(ci.size(), 1u);
  EXPECT_EQ(ci[0].get_interface_name(), "velocity");
}

TEST(WheelJointTest, PositionModeExportsPositionCommand)
{
  WheelJoint joint("some_joint", WheelJoint::ControlMode::Position);
  auto ci = joint.exportCommandInterfaces();
  ASSERT_EQ(ci.size(), 1u);
  EXPECT_EQ(ci[0].get_interface_name(), "position");
}

TEST(WheelJointTest, ExportsThreeStateInterfaces)
{
  WheelJoint joint("left_back", WheelJoint::ControlMode::Velocity);
  auto si = joint.exportStateInterfaces();
  ASSERT_EQ(si.size(), 3u);
  EXPECT_EQ(si[0].get_interface_name(), "position");
  EXPECT_EQ(si[1].get_interface_name(), "velocity");
  EXPECT_EQ(si[2].get_interface_name(), "acceleration");
}

TEST(WheelJointTest, MoveToSafePositionAlwaysReturnsTrue)
{
  WheelJoint joint("left_back", WheelJoint::ControlMode::Velocity);
  EXPECT_TRUE(joint.moveToSafePosition(0.0, 5.0));
}

TEST(WheelJointTest, ActivateReturnsTrue)
{
  WheelJoint joint("left_back", WheelJoint::ControlMode::Velocity);
  EXPECT_TRUE(joint.activate());
}

TEST(WheelJointTest, NameReturnsCorrectly)
{
  WheelJoint joint("right_back", WheelJoint::ControlMode::Velocity);
  EXPECT_EQ(joint.name(), "right_back");
}

}  // namespace alfa_robot_hardware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
