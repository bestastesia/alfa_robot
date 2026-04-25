#include "alfa_robot_hardware/joint/canopen_joint.hpp"

#include <cmath>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

// 编码器分辨率常量
static constexpr double kPulsesPerMeter = 1000000.0;  // 直线模组默认值

CanopenJoint::CanopenJoint(std::string name, Config cfg, CanopenDriver & driver)
: IJoint(std::move(name)), cfg_(cfg), driver_(driver)
{
  // 预计算转换系数
  if (cfg_.encoder_resolution > 0.0) {
    // 旋转电机: 脉冲 → 弧度
    // pulses / encoder_resolution = 圈数
    // 圈数 * 2π / gear_ratio = 弧度
    pulses_to_rad_ = 2.0 * M_PI / (cfg_.encoder_resolution * cfg_.gear_ratio);
    rad_to_pulses_ = 1.0 / pulses_to_rad_;
    is_rotary_ = true;
  } else {
    // 直线模组: 脉冲 → 米
    // pulses / kPulsesPerMeter / gear_ratio = 米
    pulses_to_rad_ = 1.0 / (kPulsesPerMeter * cfg_.gear_ratio);
    rad_to_pulses_ = 1.0 / pulses_to_rad_;
    is_rotary_ = false;
  }
}

bool CanopenJoint::activate()
{
  if (!driver_.isNodeEnabled(cfg_.node_id)) { return true; }

  // 读取初始位置（SDO 方式，返回米用于直线模组兼容）
  double pos_m = 0.0;
  if (driver_.readPositionSdo(cfg_.node_id, pos_m)) {
    // 将米转换为脉冲，再用正确的转换系数转为弧度
    // 注意：readPositionSdo 返回的是米，需要先转回脉冲
    int32_t pulses = static_cast<int32_t>(pos_m * kPulsesPerMeter);
    double pos = static_cast<double>(pulses) * pulses_to_rad_ * cfg_.direction;

    position_      = pos;
    prev_position_ = pos;
    position_cmd_  = pos;
    prev_filtered_ = static_cast<double>(pulses);  // 原始脉冲值用于滤波
    first_read_    = false;

    RCLCPP_INFO(rclcpp::get_logger("CanopenJoint"),
      "%s activated: initial_pos=%.4f %s",
      name_.c_str(), pos, is_rotary_ ? "rad" : "m");
  }
  return true;
}

void CanopenJoint::deactivate() {}

void CanopenJoint::read(double dt)
{
  if (!driver_.isNodeEnabled(cfg_.node_id)) { return; }

  // 读取原始脉冲值
  int32_t pulses = 0;
  if (!driver_.getCachedPositionPulses(cfg_.node_id, pulses)) { return; }

  // 转换为弧度（旋转电机）或米（直线模组）
  double pos = static_cast<double>(pulses) * pulses_to_rad_ * cfg_.direction;
  if (!std::isfinite(pos)) { pos = 0.0; }

  position_ = pos;

  if (dt > 0.0 && !first_read_) {
    velocity_     = (pos - prev_position_) / dt;
    acceleration_ = (velocity_ - prev_velocity_) / dt;
  }

  prev_position_ = pos;
  prev_velocity_ = velocity_;

  if (first_read_) {
    position_cmd_  = pos;
    prev_filtered_ = static_cast<double>(pulses);
    first_read_    = false;
  }
}

void CanopenJoint::write(double dt)
{
  if (first_read_) { return; }
  if (!driver_.isNodeEnabled(cfg_.node_id)) { return; }

  // 弧度 → 脉冲
  double cmd_pulses = position_cmd_ * cfg_.direction * rad_to_pulses_;

  // 对脉冲值进行滤波（保持脉冲域的连续性）
  cmd_pulses = applyLowPassFilter(cmd_pulses, dt);

  // 转换回米（CanopenDriver::writePositions 期望的单位）
  double cmd_m = cmd_pulses / kPulsesPerMeter;

  driver_.writePositions({{cfg_.node_id, cmd_m}});
}

std::vector<hardware_interface::StateInterface> CanopenJoint::exportStateInterfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(name_, hardware_interface::HW_IF_POSITION,     &position_);
  si.emplace_back(name_, hardware_interface::HW_IF_VELOCITY,     &velocity_);
  si.emplace_back(name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_);
  return si;
}

std::vector<hardware_interface::CommandInterface> CanopenJoint::exportCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_cmd_);
  return ci;
}

bool CanopenJoint::moveToSafePosition(double target_rad, double timeout_s)
{
  if (!driver_.isNodeEnabled(cfg_.node_id)) { return true; }
  const double kTol  = 0.05;
  const double kDt   = 0.01;
  const int    kIter = static_cast<int>(timeout_s / kDt);

  position_cmd_ = target_rad;
  for (int i = 0; i < kIter; ++i) {
    read(kDt);
    write(kDt);
    if (std::abs(position_ - target_rad) < kTol) { return true; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

double CanopenJoint::applyLowPassFilter(double cmd, double dt)
{
  if (cfg_.filter_cutoff_hz <= 0.0 || dt <= 0.0) {
    prev_filtered_ = cmd;
    return cmd;
  }
  if (!filter_initialized_) {
    prev_filtered_    = cmd;
    filter_initialized_ = true;
    return cmd;
  }
  double rc      = 1.0 / (2.0 * M_PI * cfg_.filter_cutoff_hz);
  double alpha   = dt / (dt + rc);
  double filtered = prev_filtered_ + alpha * (cmd - prev_filtered_);
  prev_filtered_ = filtered;
  return filtered;
}

}  // namespace alfa_robot_hardware