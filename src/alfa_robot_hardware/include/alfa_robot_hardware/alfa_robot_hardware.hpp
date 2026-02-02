// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
#define ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_

#include <string>
#include <vector>
#include <map>

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
  // CAN communication
  int can_socket_;
  std::string can_interface_;
  
  // Joint to motor ID mapping (only for leftjoint2-4 and rightjoint2-4, 6 joints total)
  std::map<std::string, uint8_t> joint_to_motor_id_;
  
  // State vectors (for 6 controlled joints: leftjoint2-4, rightjoint2-4)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_accelerations_;
  std::vector<double> previous_velocities_;  // For acceleration calculation
  
  // Command vectors (position control for these 6 joints)
  std::vector<double> hw_position_commands_;
  
  // Index mapping for state/command vectors
  std::map<std::string, size_t> joint_to_state_index_;
  std::map<std::string, size_t> joint_to_cmd_index_;
  
  // CAN communication helper functions
  bool initCanInterface(const std::string & interface);
  void closeCanInterface();
  bool sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc);
  bool receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc);
  void sendMotorCommand(uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data);
  bool parseMotorStatus2(const uint8_t * data, double & position, double & velocity);
  uint8_t getMotorIdForJoint(const std::string & joint_name);
  void convertPositionToCanFormat(double position_rad, uint32_t & angle_control);
  
  // Legacy vectors (for other joints that are not controlled via CAN)
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_legacy_;
  std::vector<double> hw_accelerations_legacy_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocity_commands_legacy_;
  // 伪造 velocity/acceleration 用：上一周期位置与速度（非 CAN 关节）
  std::vector<double> previous_states_legacy_;
  std::vector<double> previous_velocities_legacy_;
  
  // Check if a joint should be controlled via CAN
  bool isCanControlledJoint(const std::string & joint_name);
  
  // Check if a joint uses velocity command (wheel joints)
  bool isVelocityControlledJoint(const std::string & joint_name);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
