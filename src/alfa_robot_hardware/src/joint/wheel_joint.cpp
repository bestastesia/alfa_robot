#include "alfa_robot_hardware/joint/wheel_joint.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace alfa_robot_hardware
{

WheelJoint::WheelJoint(std::string name, ControlMode mode)
: IJoint(std::move(name)), mode_(mode)
{}

void WheelJoint::read(double dt)
{
  if (mode_ == ControlMode::Velocity) {
    velocity_ = velocity_cmd_;
    if (dt > 0.0) {
      position_ += velocity_ * dt;
    }
    acceleration_ = 0.0;
  } else {
    double prev_pos = position_;
    position_ = position_cmd_;
    if (dt > 0.0) {
      double new_vel = (position_ - prev_pos) / dt;
      acceleration_ = (new_vel - prev_velocity_) / dt;
      prev_velocity_ = new_vel;
      velocity_ = new_vel;
    }
  }
}

std::vector<hardware_interface::StateInterface> WheelJoint::exportStateInterfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(name_, hardware_interface::HW_IF_POSITION,     &position_);
  si.emplace_back(name_, hardware_interface::HW_IF_VELOCITY,     &velocity_);
  si.emplace_back(name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_);
  return si;
}

std::vector<hardware_interface::CommandInterface> WheelJoint::exportCommandInterfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  if (mode_ == ControlMode::Velocity) {
    ci.emplace_back(name_, hardware_interface::HW_IF_VELOCITY, &velocity_cmd_);
  } else {
    ci.emplace_back(name_, hardware_interface::HW_IF_POSITION, &position_cmd_);
  }
  return ci;
}

}  // namespace alfa_robot_hardware
