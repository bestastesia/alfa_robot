#include "alfa_robot_hardware/joint/canopen_joint.hpp"

#include <cmath>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

CanopenJoint::CanopenJoint(std::string name, Config cfg, CanopenDriver & driver)
: IJoint(std::move(name)), cfg_(cfg), driver_(driver)
{}

bool CanopenJoint::activate()
{
  if (!driver_.isNodeEnabled(cfg_.node_id)) { return true; }

  double pos_m = 0.0;
  if (driver_.readPositionSdo(cfg_.node_id, pos_m)) {
    double pos = pos_m / cfg_.gear_ratio;
    position_      = pos;
    prev_position_ = pos;
    position_cmd_  = pos;
    prev_filtered_ = pos_m;  // pre-gear raw value for filter continuity
    first_read_    = false;
  }
  return true;
}

void CanopenJoint::deactivate() {}

void CanopenJoint::read(double dt)
{
  if (!driver_.isNodeEnabled(cfg_.node_id)) { return; }

  auto positions = driver_.readPositions();
  auto it = positions.find(cfg_.node_id);
  if (it == positions.end()) { return; }

  double pos = it->second / cfg_.gear_ratio;
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
    prev_filtered_ = it->second;  // raw motor-side value
    first_read_    = false;
  }
}

void CanopenJoint::write(double dt)
{
  if (first_read_) { return; }
  if (!driver_.isNodeEnabled(cfg_.node_id)) { return; }

  double cmd_m = position_cmd_ * cfg_.gear_ratio;
  cmd_m = applyLowPassFilter(cmd_m, dt);
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
