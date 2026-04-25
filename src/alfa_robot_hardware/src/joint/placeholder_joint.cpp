#include "alfa_robot_hardware/joint/placeholder_joint.hpp"

#include <cmath>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

PlaceholderJoint::PlaceholderJoint(std::string name)
: IJoint(std::move(name)), cfg_(Config{})
{}

PlaceholderJoint::PlaceholderJoint(std::string name, Config cfg)
: IJoint(std::move(name)), cfg_(cfg)
{}

bool PlaceholderJoint::activate()
{
  position_      = 0.0;
  prev_position_ = 0.0;
  position_cmd_  = 0.0;
  prev_filtered_ = 0.0;
  active_        = true;
  RCLCPP_INFO(rclcpp::get_logger("PlaceholderJoint"),
    "%s activated (placeholder - no hardware)", name_.c_str());
  return true;
}

void PlaceholderJoint::deactivate()
{
  active_ = false;
}

void PlaceholderJoint::read(double dt)
{
  // Placeholder: no hardware to read from
  // Just keep position at 0 or last commanded value
  if (!active_) { return; }

  // For simulation purposes, assume position follows command
  position_ = position_cmd_;

  if (dt > 0.0) {
    velocity_     = (position_ - prev_position_) / dt;
    acceleration_ = (velocity_ - prev_velocity_) / dt;
  }

  prev_position_ = position_;
  prev_velocity_ = velocity_;
}

void PlaceholderJoint::write(double dt)
{
  // Placeholder: no hardware to write to
  // Just apply filtering if configured
  if (!active_) { return; }

  double cmd = position_cmd_ * cfg_.direction;
  cmd = applyLowPassFilter(cmd, dt);
  // No actual hardware write
}

std::vector<hardware_interface::StateInterface> PlaceholderJoint::exportStateInterfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(name_, hardware_interface::HW_IF_POSITION,     &position_);
  si.emplace_back(name_, hardware_interface::HW_IF_VELOCITY,     &velocity_);
  si.emplace_back(name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_);
  return si;
}

std::vector<hardware_interface::CommandInterface> PlaceholderJoint::exportCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_cmd_);
  return ci;
}

bool PlaceholderJoint::moveToSafePosition(double target_rad, double timeout_s)
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

double PlaceholderJoint::applyLowPassFilter(double cmd, double dt)
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
