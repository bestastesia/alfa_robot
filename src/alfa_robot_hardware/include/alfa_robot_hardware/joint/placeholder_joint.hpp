#ifndef ALFA_ROBOT_HARDWARE__JOINT__PLACEHOLDER_JOINT_HPP_
#define ALFA_ROBOT_HARDWARE__JOINT__PLACEHOLDER_JOINT_HPP_

#include "alfa_robot_hardware/joint/i_joint.hpp"

namespace alfa_robot_hardware
{

/**
 * Placeholder joint for temporarily occupying a joint slot when hardware protocol is unknown.
 * Accepts commands but does not drive any motor. Useful for incremental migration.
 */
class PlaceholderJoint final : public IJoint
{
public:
  struct Config {
    double filter_cutoff_hz{0.0};  // 0 = disabled
    double direction{1.0};         // +1 or -1
  };

  explicit PlaceholderJoint(std::string name);
  PlaceholderJoint(std::string name, Config cfg);
  PlaceholderJoint(const PlaceholderJoint &) = delete;
  PlaceholderJoint & operator=(const PlaceholderJoint &) = delete;

  bool activate() override;
  void deactivate() override;
  void read(double dt) override;
  void write(double dt) override;

  std::vector<hardware_interface::StateInterface>   exportStateInterfaces() override;
  std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

  bool moveToSafePosition(double target_rad, double timeout_s) override;

private:
  Config cfg_;

  double position_{0.0};
  double velocity_{0.0};
  double acceleration_{0.0};
  double position_cmd_{0.0};
  double prev_position_{0.0};
  double prev_velocity_{0.0};
  double prev_filtered_{0.0};
  bool   filter_initialized_{false};
  bool   active_{false};

  double applyLowPassFilter(double cmd, double dt);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__PLACEHOLDER_JOINT_HPP_
