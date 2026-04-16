#ifndef ALFA_ROBOT_HARDWARE__JOINT__I_JOINT_HPP_
#define ALFA_ROBOT_HARDWARE__JOINT__I_JOINT_HPP_

#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"

namespace alfa_robot_hardware
{

class IJoint
{
public:
  virtual ~IJoint() = default;

  virtual bool activate() = 0;
  virtual void deactivate() = 0;

  virtual void read(double dt) = 0;
  virtual void write(double dt) = 0;

  virtual std::vector<hardware_interface::StateInterface>   exportStateInterfaces() = 0;
  virtual std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() = 0;

  // Move to safe position; blocks until reached or timeout. Returns true if reached.
  virtual bool moveToSafePosition(double target_rad, double timeout_s) = 0;

  const std::string & name() const { return name_; }

protected:
  explicit IJoint(std::string name) : name_(std::move(name)) {}
  std::string name_;
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__I_JOINT_HPP_
