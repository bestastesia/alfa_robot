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
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "alfa_robot_hardware/can_bus.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
  // CanBus communication layer
  std::unique_ptr<CanBus> can_bus_;
  CanBusConfig can_bus_config_;

  // RMD joint mappings
  std::map<std::string, RmdMotorInfo> joint_to_motor_id_;
  std::map<std::string, size_t> joint_to_state_index_;
  std::map<std::string, size_t> joint_to_cmd_index_;

  // CANopen joint mappings
  std::map<std::string, uint8_t> canopen_joint_to_node_id_;
  std::map<std::string, size_t> canopen_joint_to_index_;

  // RMD state/command buffers (bound to export_*_interfaces)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_accelerations_;
  std::vector<double> hw_position_commands_;
  std::vector<double> previous_positions_;
  std::vector<double> previous_velocities_;

  // CANopen state/command buffers
  std::vector<double> canopen_positions_;
  std::vector<double> canopen_velocities_;
  std::vector<double> canopen_accelerations_;
  std::vector<double> canopen_position_commands_;
  std::vector<double> canopen_previous_positions_;
  std::vector<double> canopen_previous_velocities_;

  // Franka-style first-update guards
  bool first_position_update_{true};
  bool canopen_first_position_update_{true};

  // Software zero offset for turn joint (captured at activation)
  double turn_zero_offset_rad_{0.0};

  void initializePositionCommands();
  bool moveToSafePosition(double timeout_seconds = 5.0);

  // Safe position configuration
  std::map<std::string, double> safe_positions_;
  bool use_safe_shutdown_{true};

  // Legacy placeholder joints (wheels)
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_legacy_;
  std::vector<double> hw_accelerations_legacy_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocity_commands_legacy_;
  std::vector<double> previous_states_legacy_;
  std::vector<double> previous_velocities_legacy_;

  // Joint classification helpers
  static bool isCanControlledJoint(const std::string & name);
  static bool isVelocityControlledJoint(const std::string & name);
  static bool isCanopenControlledJoint(const std::string & name);

  // Trajectory logging service node
  rclcpp::Node::SharedPtr traj_log_node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr traj_log_start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr traj_log_dump_srv_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr traj_log_executor_;
  std::thread traj_log_thread_;
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
