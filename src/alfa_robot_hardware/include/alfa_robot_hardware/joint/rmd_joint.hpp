#ifndef ALFA_ROBOT_HARDWARE__JOINT__RMD_JOINT_HPP_
#define ALFA_ROBOT_HARDWARE__JOINT__RMD_JOINT_HPP_

#include "alfa_robot_hardware/joint/i_joint.hpp"
#include "alfa_robot_hardware/driver/rmd_driver.hpp"

namespace alfa_robot_hardware
{

class RmdJoint final : public IJoint
{
public:
  struct Config {
    uint8_t motor_id;
    double  zero_offset_rad{0.0};   // Subtracted from raw read, added to write command
    double  filter_cutoff_hz{0.0};  // 0 = disabled
    double  direction{1.0};         // +1 or -1: flip motor vs controller frame
    double  static_bias_rad{0.0};   // Permanent encoder bias: added to raw on read, subtracted on write
  };

  RmdJoint(std::string name, Config cfg, RmdDriver & driver);

  RmdJoint(const RmdJoint &) = delete;
  RmdJoint & operator=(const RmdJoint &) = delete;

  // Reads initial position from driver and initializes buffers.
  bool activate() override;
  void deactivate() override;

  // read(dt): fetch from driver, apply zero_offset, compute vel/accel, optional LPF
  void read(double dt) override;

  // write(dt): skip if first_read_ true; apply zero_offset inverse + optional LPF; send to driver
  void write(double dt) override;

  std::vector<hardware_interface::StateInterface>   exportStateInterfaces() override;
  std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

  bool moveToSafePosition(double target_rad, double timeout_s) override;

  // Called by AlfaRobotHW::on_activate for the "turn" joint after first read.
  // Stores current position as zero offset; zeroes position and command buffers.
  void captureCurrentPositionAsZero();

private:
  Config     cfg_;
  RmdDriver & driver_;

  double position_{0.0};
  double velocity_{0.0};
  double acceleration_{0.0};
  double position_cmd_{0.0};
  double prev_position_{0.0};
  double prev_velocity_{0.0};
  double prev_filtered_{0.0};
  bool   filter_initialized_{false};
  bool   first_read_{true};

  double applyLowPassFilter(double cmd, double dt);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__RMD_JOINT_HPP_
