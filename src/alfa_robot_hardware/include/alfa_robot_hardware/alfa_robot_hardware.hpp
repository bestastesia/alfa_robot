#ifndef ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
#define ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "alfa_robot_hardware/joint/i_joint.hpp"
#include "alfa_robot_hardware/joint/rmd_joint.hpp"
#include "alfa_robot_hardware/joint/canopen_joint.hpp"
#include "alfa_robot_hardware/joint/placeholder_joint.hpp"
#include "alfa_robot_hardware/joint/zeroerr_joint.hpp"
#include "alfa_robot_hardware/joint/cylinder_joint.hpp"
#include "alfa_robot_hardware/driver/rmd_driver.hpp"
#include "alfa_robot_hardware/driver/canopen_driver.hpp"
#include "alfa_robot_hardware/driver/zeroerr_driver.hpp"
#include "alfa_robot_hardware/driver/cylinder_driver.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
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

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Drivers (owners)
  std::unique_ptr<RmdDriver>     rmd_left_, rmd_right_, rmd_base_;
  std::unique_ptr<CanopenDriver> canopen_;
  std::unique_ptr<CanopenDriver> canopen_plate_;
  std::unique_ptr<ZeroerrDriver> zeroerr_left_;    // ZeroErr motors on can0 (mixed protocol)
  std::unique_ptr<CylinderDriver> cylinder_;       // Cylinder (leftjoint4) on can0

  // All joints (single list — no type dispatch in AlfaRobotHW)
  std::vector<std::unique_ptr<IJoint>> joints_;

  // Safe shutdown config
  std::map<std::string, double> safe_positions_;
  bool use_safe_shutdown_{false};

  // Driver configs (parsed in on_init)
  RmdDriver::Config     rmd_left_cfg_, rmd_right_cfg_, rmd_base_cfg_;
  CanopenDriver::Config canopen_cfg_;
  CanopenDriver::Config canopen_plate_cfg_;
  ZeroerrDriver::Config zeroerr_left_cfg_;
  CylinderDriver::Config cylinder_cfg_;

  void buildJoints();
  bool moveAllToSafePositions(double timeout_s);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
