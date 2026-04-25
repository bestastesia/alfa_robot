#include "alfa_robot_hardware/joint/rmd_joint.hpp"

#include <cmath>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

RmdJoint::RmdJoint(std::string name, Config cfg, RmdDriver & driver)
: IJoint(std::move(name)), cfg_(cfg), driver_(driver)
{}

bool RmdJoint::activate()
{
  auto positions = driver_.readPositions({cfg_.motor_id});
  auto it = positions.find(cfg_.motor_id);
  if (it != positions.end()) {
    double corrected_raw = it->second + cfg_.static_bias_rad;
    double pos = (corrected_raw - cfg_.zero_offset_rad) * cfg_.direction;
    position_      = pos;
    prev_position_ = pos;
    position_cmd_  = pos;
    prev_filtered_ = pos;
    first_read_    = false;
  }
  return true;
}

void RmdJoint::deactivate() {}

void RmdJoint::read(double dt)
{
  double raw = 0.0;
  if (!driver_.getCachedPosition(cfg_.motor_id, raw)) { return; }

  double corrected_raw = raw + cfg_.static_bias_rad;
  double pos = (corrected_raw - cfg_.zero_offset_rad) * cfg_.direction;
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
    prev_filtered_ = pos;
    first_read_    = false;
  }
}

void RmdJoint::write(double dt)
{
  if (first_read_) { return; }

  double cmd = position_cmd_ * cfg_.direction + cfg_.zero_offset_rad - cfg_.static_bias_rad;
  cmd = applyLowPassFilter(cmd, dt);

  driver_.writePositions({{cfg_.motor_id, cmd}});
}

void RmdJoint::captureCurrentPositionAsZero()
{
  if (first_read_) {
    RCLCPP_WARN(rclcpp::get_logger("RmdJoint"),
      "%s captureCurrentPositionAsZero called before first read — ignored", name_.c_str());
    return;
  }
  // raw = position_ / direction + zero_offset_rad  →  new zero = raw
  cfg_.zero_offset_rad += position_ / cfg_.direction;
  position_      = 0.0;
  prev_position_ = 0.0;
  position_cmd_  = 0.0;
  prev_filtered_ = cfg_.zero_offset_rad - cfg_.static_bias_rad;  // next write sends corrected raw
  RCLCPP_INFO(rclcpp::get_logger("RmdJoint"),
    "%s zero offset captured: %.4f rad", name_.c_str(), cfg_.zero_offset_rad);
}

std::vector<hardware_interface::StateInterface> RmdJoint::exportStateInterfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(name_, hardware_interface::HW_IF_POSITION,     &position_);
  si.emplace_back(name_, hardware_interface::HW_IF_VELOCITY,     &velocity_);
  si.emplace_back(name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_);
  return si;
}

std::vector<hardware_interface::CommandInterface> RmdJoint::exportCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_cmd_);
  return ci;
}

bool RmdJoint::moveToSafePosition(double target_rad, double timeout_s)
{
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

double RmdJoint::applyLowPassFilter(double cmd, double dt)
{
  if (cfg_.filter_cutoff_hz <= 0.0 || dt <= 0.0) {
    prev_filtered_ = cmd;
    return cmd;
  }
  if (!filter_initialized_) {
    prev_filtered_      = cmd;
    filter_initialized_ = true;
    return cmd;
  }
  double rc       = 1.0 / (2.0 * M_PI * cfg_.filter_cutoff_hz);
  double alpha    = dt / (dt + rc);
  double filtered = prev_filtered_ + alpha * (cmd - prev_filtered_);
  prev_filtered_  = filtered;
  return filtered;
}

}  // namespace alfa_robot_hardware
