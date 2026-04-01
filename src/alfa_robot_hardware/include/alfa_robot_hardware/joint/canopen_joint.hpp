#ifndef ALFA_ROBOT_HARDWARE__JOINT__CANOPEN_JOINT_HPP_
#define ALFA_ROBOT_HARDWARE__JOINT__CANOPEN_JOINT_HPP_

#include "alfa_robot_hardware/joint/i_joint.hpp"
#include "alfa_robot_hardware/driver/canopen_driver.hpp"

namespace alfa_robot_hardware
{

class CanopenJoint final : public IJoint
{
public:
  struct Config {
    uint8_t node_id;
    double  gear_ratio{1.0};        // leftarmbase/rightarmbase = 3.0
    double  filter_cutoff_hz{0.0};  // 0 = disabled
  };

  CanopenJoint(std::string name, Config cfg, CanopenDriver & driver);
  CanopenJoint(const CanopenJoint &) = delete;
  CanopenJoint & operator=(const CanopenJoint &) = delete;

  bool activate() override;
  void deactivate() override;
  void read(double dt) override;
  void write(double dt) override;

  std::vector<hardware_interface::StateInterface>   exportStateInterfaces() override;
  std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

  bool moveToSafePosition(double target_rad, double timeout_s) override;

private:
  Config          cfg_;
  CanopenDriver & driver_;

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

#endif  // ALFA_ROBOT_HARDWARE__JOINT__CANOPEN_JOINT_HPP_
