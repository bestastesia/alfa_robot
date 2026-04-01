#ifndef ALFA_ROBOT_HARDWARE__JOINT__WHEEL_JOINT_HPP_
#define ALFA_ROBOT_HARDWARE__JOINT__WHEEL_JOINT_HPP_

#include "alfa_robot_hardware/joint/i_joint.hpp"

namespace alfa_robot_hardware
{

class WheelJoint final : public IJoint
{
public:
  enum class ControlMode { Velocity, Position };

  WheelJoint(std::string name, ControlMode mode);

  bool activate() override { return true; }
  void deactivate() override {}

  void read(double dt) override;
  void write(double /*dt*/) override {}

  std::vector<hardware_interface::StateInterface>   exportStateInterfaces() override;
  std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

  bool moveToSafePosition(double /*target_rad*/, double /*timeout_s*/) override
  { return true; }

private:
  ControlMode mode_;
  double position_{0.0};
  double velocity_{0.0};
  double acceleration_{0.0};
  double position_cmd_{0.0};
  double velocity_cmd_{0.0};
  double prev_velocity_{0.0};
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__WHEEL_JOINT_HPP_
