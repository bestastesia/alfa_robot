// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
#define ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <set>

// SocketCAN headers for Ubuntu 22.04
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace alfa_robot_hardware
{
class AlfaRobotHW : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Motor gear ratio (output = motor_angle / kGearRatio)
  static constexpr int kGearRatio = 36;

  // CAN communication (dual bus: left arm = can0, right arm = can1)
  int can_socket_left_;
  int can_socket_right_;
  std::string can_interface_left_;
  std::string can_interface_right_;

  // CAN communication (base bus: can2, mixed RMD + CANopen)
  // turn (RMD, motor_id=1) + updown (CANopen, node_id=1)
  int can_socket_base_;
  std::string can_interface_base_;  // default "can2"

  // Bus availability flags (false if CAN interface not found at configure time)
  bool can_left_available_{false};
  bool can_right_available_{false};
  bool can_base_available_{false};
  bool can_canopen_available_{false};

  // Max speed for position control (0xA4), in motor dps. 1 dps/LSB. Configurable via hardware_parameters.
  uint16_t max_speed_dps_{360};

  // Joint to motor ID mapping (CAN position joints: leftjoint2-4, rightjoint2-4, 6 joints total)
  std::map<std::string, uint8_t> joint_to_motor_id_;

  // State vectors (for 6 CAN position joints: leftjoint2-4, rightjoint2-4)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_accelerations_;
  std::vector<double> previous_velocities_;   // For acceleration calculation
  std::vector<double> previous_positions_;   // For velocity estimation from position diff

  // Command vectors (position control for CAN joints)
  std::vector<double> hw_position_commands_;

  // Franka-style: avoid jump on first activation - initialize commands with current position
  bool first_position_update_{true};

  // Index mapping for state/command vectors
  std::map<std::string, size_t> joint_to_state_index_;
  std::map<std::string, size_t> joint_to_cmd_index_;

  // Initialize position commands with current read-back (first pass after activation)
  void initializePositionCommands();

  // CAN communication helper functions
  bool initCanInterface(const std::string & interface, int & socket_fd);
  void closeCanInterface(int & socket_fd);
  bool sendCanFrame(int socket_fd, uint32_t can_id, const uint8_t * data, uint8_t dlc);
  bool receiveCanFrame(int socket_fd, uint32_t & can_id, uint8_t * data, uint8_t & dlc);
  void sendMotorCommand(int socket_fd, uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data);
  uint8_t getMotorIdForJoint(const std::string & joint_name);
  int getCanSocketForRmdJoint(const std::string & joint_name);

  // Protocol 0x92: parse motor reply (DATA[1-7] = motorAngle, 7 bytes, 0.01 deg/LSB)
  // Returns position in radians (output side, after gear ratio)
  bool parseMotorAngleReply0x92(const uint8_t * data, double & position_rad);

  // Protocol 0xA4: convert position_rad to angleControl (int32_t) and fill frame_data[2-7]
  void convertPositionToCanFormat0xA4(
    double position_rad, uint8_t * frame_data);

  // ========== CANopen (Leisai) motor support ==========
  // can3: updown (node 1), leftarmbase (node 2), leftjoint1 (node 3),
  //       rightarmbase (node 4), rightjoint1 (node 5)
  // Ball screw: 131072 pulses/rev, 10mm lead → 1m = 13107200 pulses
  static constexpr double kCanopenPulsesPerMeter = 13107200.0;

  // CANopen profile velocity/acceleration (pulses/s, pulses/s²). Configurable via hardware_parameters.
  // Default 50000 pps ≈ 3.8mm/s (safe for debugging)
  uint32_t canopen_profile_velocity_{50000};
  uint32_t canopen_profile_accel_{50000};

  int can_socket_canopen_;
  std::string can_interface_canopen_;  // default "can3"

  // Joint → CANopen node ID mapping
  std::map<std::string, uint8_t> canopen_joint_to_node_id_;

  // Set of CANopen node IDs that successfully entered Operation Enabled
  std::set<uint8_t> canopen_enabled_nodes_;

  // State/command vectors for CANopen joints
  std::vector<double> canopen_positions_;
  std::vector<double> canopen_velocities_;
  std::vector<double> canopen_accelerations_;
  std::vector<double> canopen_position_commands_;
  std::vector<double> canopen_previous_positions_;
  std::vector<double> canopen_previous_velocities_;

  std::map<std::string, size_t> canopen_joint_to_index_;
  bool canopen_first_position_update_{true};

  bool isCanopenControlledJoint(const std::string & joint_name);

  // CANopen SDO helpers (expedited, 4-byte max)
  bool canopenSdoWrite(int socket_fd, uint8_t node_id,
    uint16_t index, uint8_t subindex, const uint8_t * data, uint8_t size);
  bool canopenSdoRead(int socket_fd, uint8_t node_id,
    uint16_t index, uint8_t subindex, uint8_t * data, uint8_t & size);

  // CANopen NMT
  bool canopenNmtSend(int socket_fd, uint8_t command, uint8_t node_id);

  // CANopen state machine: transition to Operation Enabled
  bool canopenEnableMotor(int socket_fd, uint8_t node_id);
  bool canopenDisableMotor(int socket_fd, uint8_t node_id);

  // CANopen position read/write
  bool canopenReadPosition(int socket_fd, uint8_t node_id, double & position_m);
  bool canopenWritePosition(int socket_fd, uint8_t node_id, double position_m);

  // Legacy vectors for placeholder joints (turn, updown, wheels)
  // TODO: replace with actual hardware when implemented
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_legacy_;
  std::vector<double> hw_accelerations_legacy_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocity_commands_legacy_;
  std::vector<double> previous_states_legacy_;
  std::vector<double> previous_velocities_legacy_;

  bool isCanControlledJoint(const std::string & joint_name);
  bool isVelocityControlledJoint(const std::string & joint_name);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
