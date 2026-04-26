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
  // 读取初始位置作为偏置
  auto positions = driver_.readPositions({config_.node_id});
  auto it = positions.find(config_.node_id);

  if (it != positions.end()) {
    initial_position_counts_ = it->second;
    position_state_ = 0.0;  // 初始位置归零
    last_position_ = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("ZeroerrJoint"),
      "Joint '%s' activated, initial position: %d counts (offset=%.4f rad)",
      name_.c_str(), initial_position_counts_, config_.offset);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("ZeroerrJoint"),
      "Joint '%s' failed to read initial position, using 0", name_.c_str());
    initial_position_counts_ = 0;
  }

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

    // 计算速度
    if (dt > 0.0) {
      velocity_state_ = (position_state_ - last_position_) / dt;
    }
    last_position_ = position_state_;
  }
}

void ZeroerrJoint::write(double /*dt*/)
{
  // 将命令位置转换为脉冲
  // 注意：需要加上初始位置偏置，因为是绝对位置控制
  double relative_rad = (position_command_ - config_.offset) * config_.sign;
  int32_t target_counts = radiansToCounts(relative_rad) + initial_position_counts_;

  // 写入驱动
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
  return si;
}

std::vector<hardware_interface::CommandInterface> ZeroerrJoint::exportCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_command_);
  return ci;
}

}  // namespace alfa_robot_hardware
