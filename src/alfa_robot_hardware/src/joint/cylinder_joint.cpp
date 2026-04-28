// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "alfa_robot_hardware/joint/cylinder_joint.hpp"

#include <cmath>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

CylinderJoint::CylinderJoint(const std::string & name, Config config, CylinderDriver & driver)
: IJoint(name)
, config_(config)
, driver_(driver)
{
}

bool CylinderJoint::activate()
{
  // 读取初始位置
  double initial_pos_m = 0.0;
  if (driver_.readPosition(initial_pos_m)) {
    position_state_ = initial_pos_m * config_.sign + config_.offset;
    // 启动时自动归零到 0 位置
    position_command_ = 0.0;
    last_position_ = position_state_;

    RCLCPP_INFO(rclcpp::get_logger("CylinderJoint"),
      "Joint '%s' activated, initial position: %.6f m, homing to 0.0 m",
      name_.c_str(), initial_pos_m);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("CylinderJoint"),
      "Joint '%s' failed to read initial position", name_.c_str());
    position_state_ = 0.0;
    position_command_ = 0.0;  // 归零
  }

  return true;
}

void CylinderJoint::deactivate()
{
  driver_.disable();
  driver_enabled_ = false;
}

void CylinderJoint::read(double dt)
{
  double pos_m = 0.0;
  // 从驱动缓存获取位置（驱动在 AlfaRobotHW::read() 中已更新）
  if (driver_.getCachedPosition(pos_m)) {
    position_state_ = pos_m * config_.sign + config_.offset;

    if (dt > 0.0) {
      velocity_state_ = (position_state_ - last_position_) / dt;
      acceleration_state_ = (velocity_state_ - last_velocity_) / dt;
      last_velocity_ = velocity_state_;
    }
    last_position_ = position_state_;
  }
}

void CylinderJoint::write(double /*dt*/)
{
  double target_m = (position_command_ - config_.offset) * config_.sign;
  driver_.writePosition(target_m);
}

bool CylinderJoint::moveToSafePosition(double safe_position_m, double timeout_s)
{
  const double kTol = 0.01;  // 1cm 容差
  const double kDt = 0.01;
  const int kIter = static_cast<int>(timeout_s / kDt);

  position_command_ = safe_position_m;
  for (int i = 0; i < kIter; ++i) {
    read(kDt);
    write(kDt);
    if (std::abs(position_state_ - safe_position_m) < kTol) { return true; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

std::vector<hardware_interface::StateInterface> CylinderJoint::exportStateInterfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_state_);
  si.emplace_back(name_, hardware_interface::HW_IF_VELOCITY, &velocity_state_);
  si.emplace_back(name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_state_);
  return si;
}

std::vector<hardware_interface::CommandInterface> CylinderJoint::exportCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_command_);
  return ci;
}

}  // namespace alfa_robot_hardware
