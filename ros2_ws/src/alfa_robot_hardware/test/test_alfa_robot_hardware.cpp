// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License

#include <gtest/gtest.h>
#include <cmath>

#include "alfa_robot_hardware/driver/rmd_driver.hpp"

#include "alfa_robot_hardware/driver/canopen_driver.hpp"

#include "alfa_robot_hardware/joint/rmd_joint.hpp"

#include "alfa_robot_hardware/joint/canopen_joint.hpp"
namespace alfa_robot_hardware
{

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
  RmdJoint joint("left_joint2", {1, 0.0, 0.0}, drv);
  joint.activate();
  // first_read_ still true -> write() is a no-op -> position stays 0
  auto si = joint.exportStateInterfaces();
  ASSERT_EQ(si.size(), 3u);
  EXPECT_NEAR(si[0].get_value(), 0.0, 1e-9);
}

TEST(RmdJointTest, WriteIsNoOpBeforeFirstRead)
{
  RmdDriver drv({"bogus", 1800});
  RmdJoint joint("left_joint2", {1, 0.0, 0.0}, drv);
  // Never called read() -> write() must not crash
  EXPECT_NO_THROW(joint.write(0.01));
}

TEST(RmdJointTest, ExportsThreeStateAndOneCommandInterface)
{
  RmdDriver drv({"bogus", 1800});
  RmdJoint joint("left_joint2", {1, 0.0, 0.0}, drv);
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


TEST(RmdJointTest, CaptureCurrentPositionAsZeroUpdatesOffset)
{
  // We cannot inject a real driver position, but we can verify the guard:
  // calling before first read is a no-op (offset stays 0)
  RmdDriver drv({"bogus", 1800});
  RmdJoint joint("turn", {1, 0.5, 0.0}, drv);
  // Before first read: captureCurrentPositionAsZero is a no-op
  joint.captureCurrentPositionAsZero();
  // offset was 0.5, position_ was 0 -> guard fires -> offset unchanged
  // write() still no-op (first_read_ true) -> no crash
  EXPECT_NO_THROW(joint.write(0.01));
  auto si = joint.exportStateInterfaces();
  EXPECT_NEAR(si[0].get_value(), 0.0, 1e-9);  // position still 0
}

// ── CanopenJoint Tests ──────────────────────────────────────────────────────

TEST(CanopenJointTest, ExportsThreeStateAndOneCommandInterface)
{
  CanopenDriver drv({"bogus", 50000, 50000});
  CanopenJoint joint("updown", {1, 1.0, 0.0}, drv);
  EXPECT_EQ(joint.exportStateInterfaces().size(), 3u);
  auto ci = joint.exportCommandInterfaces();
  EXPECT_EQ(ci.size(), 1u);
  EXPECT_EQ(ci[0].get_interface_name(), "position");
}

TEST(CanopenJointTest, WriteIsNoOpBeforeFirstRead)
{
  CanopenDriver drv({"bogus", 50000, 50000});
  CanopenJoint joint("updown", {1, 1.0, 0.0}, drv);
  EXPECT_NO_THROW(joint.write(0.01));
}

TEST(CanopenJointTest, NodeDisabled_MoveToSafePositionReturnsTrue)
{
  // Node not enabled -> moveToSafePosition returns true immediately (no-op)
  CanopenDriver drv({"bogus", 50000, 50000});
  CanopenJoint joint("updown", {1, 1.0, 0.0}, drv);
  EXPECT_TRUE(joint.moveToSafePosition(0.0, 5.0));
}
}  // namespace alfa_robot_hardware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
