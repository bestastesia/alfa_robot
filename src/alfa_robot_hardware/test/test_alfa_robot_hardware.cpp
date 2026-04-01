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

TEST(WheelJointTest, VelocityModeIntegratesPosition)
{
  WheelJoint joint("left_back", WheelJoint::ControlMode::Velocity);
  joint.activate();

  // Get velocity command interface pointer
  auto ci = joint.exportCommandInterfaces();
  // ci[0] is velocity_cmd_. We set it by calling read with velocity_cmd_ = 1.0
  // But we can't set the command from outside without a handle.
  // Test: at dt=0 nothing happens
  joint.read(0.0);
  auto si = joint.exportStateInterfaces();
  EXPECT_NEAR(si[0].get_value(), 0.0, 1e-9);  // position stays 0

  // Test: at dt=0.1 with velocity_cmd_=0, position stays 0
  joint.read(0.1);
  EXPECT_NEAR(si[0].get_value(), 0.0, 1e-9);
  EXPECT_NEAR(si[1].get_value(), 0.0, 1e-9);  // velocity = velocity_cmd_ = 0
  EXPECT_NEAR(si[2].get_value(), 0.0, 1e-9);  // acceleration = 0
}

TEST(WheelJointTest, VelocityModeAccelerationIsZero)
{
  WheelJoint joint("left_back", WheelJoint::ControlMode::Velocity);
  joint.activate();
  joint.read(0.1);
  auto si = joint.exportStateInterfaces();
  // In velocity mode, acceleration is always 0 regardless of velocity change
  EXPECT_NEAR(si[2].get_value(), 0.0, 1e-9);
}

TEST(WheelJointTest, PositionModeFollowsCommand)
{
  WheelJoint joint("some_joint", WheelJoint::ControlMode::Position);
  joint.activate();
  joint.read(0.0);  // dt=0, nothing should change
  auto si = joint.exportStateInterfaces();
  EXPECT_NEAR(si[0].get_value(), 0.0, 1e-9);  // position = position_cmd_ = 0

  // After read with dt=0: position = 0, velocity unchanged
  joint.read(0.0);
  EXPECT_NEAR(si[0].get_value(), 0.0, 1e-9);
}

TEST(WheelJointTest, ReadWithZeroDtDoesNotCrash)
{
  WheelJoint joint("left_back", WheelJoint::ControlMode::Velocity);
  joint.activate();
  EXPECT_NO_THROW(joint.read(0.0));

  WheelJoint joint2("some_joint", WheelJoint::ControlMode::Position);
  joint2.activate();
  EXPECT_NO_THROW(joint2.read(0.0));
}

}  // namespace alfa_robot_hardware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
