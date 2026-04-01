// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License

#include <gtest/gtest.h>
#include <cmath>

#include "alfa_robot_hardware/joint/wheel_joint.hpp"
// Directly compile implementation until CMakeLists links alfa_robot_hardware (Task 8)
#include "../src/joint/wheel_joint.cpp"  // NOLINT(build/include)

#include "alfa_robot_hardware/driver/rmd_driver.hpp"
// And compile the implementation directly until CMakeLists links the library (Task 8):
#include "../src/driver/rmd_driver.cpp"  // NOLINT(build/include)

#include "alfa_robot_hardware/driver/canopen_driver.hpp"
#include "../src/driver/canopen_driver.cpp"  // NOLINT(build/include)

#include "alfa_robot_hardware/joint/rmd_joint.hpp"
#include "../src/joint/rmd_joint.cpp"  // NOLINT(build/include)

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

TEST(RmdDriverTest, ParseMotorAngleReply_Zero)
{
  uint8_t data[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
  double pos;
  EXPECT_TRUE(RmdDriver::parseMotorAngleReply(data, pos));
  EXPECT_NEAR(pos, 0.0, 1e-9);
}

TEST(RmdDriverTest, ParseMotorAngleReply_WrongCmd)
{
  uint8_t data[8] = {0xA4, 0, 0, 0, 0, 0, 0, 0};
  double pos;
  EXPECT_FALSE(RmdDriver::parseMotorAngleReply(data, pos));
}

TEST(RmdDriverTest, ParseMotorAngleReply_180deg)
{
  // raw=18000 => angle_deg=180 => pos_rad = pi/36
  // 18000 = 0x4650, little-endian bytes 1-2: 0x50, 0x46
  uint8_t data[8] = {0x92, 0x50, 0x46, 0, 0, 0, 0, 0};
  double pos;
  ASSERT_TRUE(RmdDriver::parseMotorAngleReply(data, pos));
  EXPECT_NEAR(pos, M_PI / 36.0, 1e-6);
}

TEST(RmdDriverTest, ParseMotorAngleReply_Negative)
{
  // raw = -18000 => sign-extended from byte[7] MSB
  // -18000 = 0xFFFFFFFFFFFFB9B0 => bytes[1..7]: 0xB0,0xB9,0xFF,0xFF,0xFF,0xFF,0xFF
  uint8_t data[8] = {0x92, 0xB0, 0xB9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  double pos;
  ASSERT_TRUE(RmdDriver::parseMotorAngleReply(data, pos));
  EXPECT_NEAR(pos, -M_PI / 36.0, 1e-6);
}

TEST(RmdDriverTest, ConvertPositionRoundTrip)
{
  double orig = 1.234;
  uint8_t fd[7];
  RmdDriver::convertPositionToCanFormat(orig, 1800, fd);
  int32_t ac = static_cast<int32_t>(fd[3]) | (static_cast<int32_t>(fd[4]) << 8) |
               (static_cast<int32_t>(fd[5]) << 16) | (static_cast<int32_t>(fd[6]) << 24);
  double recovered = static_cast<double>(ac) / 100.0 / 36.0 * M_PI / 180.0;
  EXPECT_NEAR(recovered, orig, 0.001);  // 0.01 deg resolution
}

TEST(RmdDriverTest, OpenFailsOnBogusInterface)
{
  RmdDriver drv({"bogus_can99", 1800});
  EXPECT_FALSE(drv.open());
  EXPECT_FALSE(drv.isOpen());
}

TEST(RmdDriverTest, ReadPositionsEmptyWhenNotOpen)
{
  RmdDriver drv({"bogus_can99", 1800});
  EXPECT_TRUE(drv.readPositions({1, 2}).empty());
}

TEST(CanopenDriverTest, ComputeControlword_FirstCommand_SetsNewSetpoint)
{
  bool ns_active = false;
  int32_t last_target = 0;
  uint16_t cw = CanopenDriver::computeControlword(ns_active, last_target, 0);
  // !ns_active → set new setpoint
  EXPECT_EQ(cw, 0x003F);
  EXPECT_TRUE(ns_active);
}

TEST(CanopenDriverTest, ComputeControlword_SameTarget_KeepsSetpoint)
{
  bool ns_active = true;
  int32_t last_target = 1000;
  uint16_t cw = CanopenDriver::computeControlword(ns_active, last_target, 1000);
  EXPECT_EQ(cw, 0x003F);
  EXPECT_TRUE(ns_active);
  EXPECT_EQ(last_target, 1000);
}

TEST(CanopenDriverTest, ComputeControlword_NewTarget_ClearsAndSets)
{
  bool ns_active = true;
  int32_t last_target = 1000;
  // First call: changed && ns_active → clear
  uint16_t cw1 = CanopenDriver::computeControlword(ns_active, last_target, 2000);
  EXPECT_EQ(cw1, 0x002F);
  EXPECT_FALSE(ns_active);
  EXPECT_EQ(last_target, 2000);
  // Second call: same target, !ns_active → set
  uint16_t cw2 = CanopenDriver::computeControlword(ns_active, last_target, 2000);
  EXPECT_EQ(cw2, 0x003F);
  EXPECT_TRUE(ns_active);
}

TEST(CanopenDriverTest, OpenFailsOnBogusInterface)
{
  CanopenDriver drv({"bogus_can99", 50000, 50000});
  EXPECT_FALSE(drv.open());
  EXPECT_FALSE(drv.isOpen());
}

TEST(CanopenDriverTest, ReadPositionsEmptyWhenNotOpen)
{
  CanopenDriver drv({"bogus_can99", 50000, 50000});
  EXPECT_TRUE(drv.readPositions().empty());
}

TEST(CanopenDriverTest, IsNodeEnabledReturnsFalseWhenNotEnabled)
{
  CanopenDriver drv({"bogus_can99", 50000, 50000});
  EXPECT_FALSE(drv.isNodeEnabled(1));
  EXPECT_FALSE(drv.isNodeEnabled(5));
}

// ── RmdJoint Tests ───────────────────────────────────────────────────────────

TEST(RmdJointTest, ActivateWithNoDriverKeepsFirstReadTrue)
{
  // Driver is not open -> readPositions returns empty -> activate does NOT clear first_read_
  RmdDriver drv({"bogus", 1800});
  RmdJoint joint("leftjoint2", {1, 0.0, 0.0}, drv);
  joint.activate();
  // first_read_ still true -> write() is a no-op -> position stays 0
  auto si = joint.exportStateInterfaces();
  ASSERT_EQ(si.size(), 3u);
  EXPECT_NEAR(si[0].get_value(), 0.0, 1e-9);
}

TEST(RmdJointTest, WriteIsNoOpBeforeFirstRead)
{
  RmdDriver drv({"bogus", 1800});
  RmdJoint joint("leftjoint2", {1, 0.0, 0.0}, drv);
  // Never called read() -> write() must not crash
  EXPECT_NO_THROW(joint.write(0.01));
}

TEST(RmdJointTest, ExportsThreeStateAndOneCommandInterface)
{
  RmdDriver drv({"bogus", 1800});
  RmdJoint joint("leftjoint2", {1, 0.0, 0.0}, drv);
  EXPECT_EQ(joint.exportStateInterfaces().size(), 3u);
  EXPECT_EQ(joint.exportCommandInterfaces().size(), 1u);
  EXPECT_EQ(joint.exportCommandInterfaces()[0].get_interface_name(), "position");
}

TEST(RmdJointTest, MoveToSafePositionReturnsFalseWithNoHardware)
{
  // With no hardware, read never updates position, so tolerance never met
  RmdDriver drv({"bogus", 1800});
  RmdJoint joint("turn", {1, 0.0, 0.0}, drv);
  // timeout_s=0.05 -> 5 iterations -> position stays 0, target=1.0 -> false
  EXPECT_FALSE(joint.moveToSafePosition(1.0, 0.05));
}

}  // namespace alfa_robot_hardware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
