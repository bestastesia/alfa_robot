// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "alfa_robot_hardware/joint/zeroerr_joint.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

ZeroerrJoint::ZeroerrJoint(const std::string & name, Config config, ZeroerrDriver & driver)
: IJoint(name)
, config_(config)
, driver_(driver)
{
}

bool ZeroerrJoint::activate()
{
  // 使用配置的零点位置（不再运行时读取）
  initial_position_counts_ = config_.initial_counts;
  position_state_ = 0.0;  // 状态归零
  last_position_ = 0.0;
  last_velocity_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("ZeroerrJoint"),
    "Joint '%s' activated with configured zero point: %d counts (offset=%.4f rad, sign=%.1f)",
    name_.c_str(), initial_position_counts_, config_.offset, config_.sign);

  position_command_ = position_state_;
  return true;
}

void ZeroerrJoint::deactivate()
{
  // 无需特殊处理
}

void ZeroerrJoint::read(double dt)
{
  // 从驱动缓存获取位置
  int32_t position_counts;
  if (driver_.getCachedPosition(config_.node_id, position_counts)) {
    // 计算相对于初始位置的偏移
    int32_t relative_counts = position_counts - initial_position_counts_;

    // 转换为弧度并应用偏置和方向
    position_state_ = countsToRadians(relative_counts) + config_.offset;

    // 计算速度和加速度
    if (dt > 0.0) {
      velocity_state_ = (position_state_ - last_position_) / dt;
      acceleration_state_ = (velocity_state_ - last_velocity_) / dt;
      last_velocity_ = velocity_state_;
    }
    last_position_ = position_state_;
  }
}

void ZeroerrJoint::write(double /*dt*/)
{
  // 绝对位置模式：命令位置 = GUI 滑块值 (弧度)
  // 需要转换为脉冲并加上初始位置偏置
  double relative_rad = (position_command_ - config_.offset) * config_.sign;
  int32_t target_counts = radiansToCounts(relative_rad) + initial_position_counts_;

  driver_.writePositions({{config_.node_id, target_counts}});
}

bool ZeroerrJoint::moveToSafePosition(double safe_position_rad, double timeout_s)
{
  position_command_ = safe_position_rad;

  auto start_time = std::chrono::steady_clock::now();
  auto deadline = start_time + std::chrono::duration<double>(timeout_s);

  while (std::chrono::steady_clock::now() < deadline) {
    write(0.01);
    usleep(10000);
    read(0.01);

    // 检查是否到达目标
    if (std::abs(position_state_ - safe_position_rad) < 0.01) {  // 0.01 rad 容差
      return true;
    }
  }

  RCLCPP_WARN(rclcpp::get_logger("ZeroerrJoint"),
    "Joint '%s' failed to reach safe position within %.1f s", name_.c_str(), timeout_s);
  return false;
}

double ZeroerrJoint::countsToRadians(int32_t counts) const
{
  // 使用驱动器的转换系数
  return static_cast<double>(counts) / driver_.getCountsPerRadian();
}

int32_t ZeroerrJoint::radiansToCounts(double radians) const
{
  return static_cast<int32_t>(std::round(radians * driver_.getCountsPerRadian()));
}

std::vector<hardware_interface::StateInterface> ZeroerrJoint::exportStateInterfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_state_);
  si.emplace_back(name_, hardware_interface::HW_IF_VELOCITY, &velocity_state_);
  si.emplace_back(name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_state_);
  return si;
}

std::vector<hardware_interface::CommandInterface> ZeroerrJoint::exportCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_command_);
  return ci;
}

}  // namespace alfa_robot_hardware
