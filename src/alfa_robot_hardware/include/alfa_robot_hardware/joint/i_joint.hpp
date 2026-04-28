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
  /// 限位状态枚举
  enum class LimitState {
    OK,           ///< 正常，无限位触发
    POS_LIMIT,    ///< 正向限位触发
    NEG_LIMIT,    ///< 负向限位触发
  };

  virtual ~IJoint() = default;

  virtual bool activate() = 0;
  virtual void deactivate() = 0;

  virtual void read(double dt) = 0;
  virtual void write(double dt) = 0;

  virtual std::vector<hardware_interface::StateInterface>   exportStateInterfaces() = 0;
  virtual std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() = 0;

  // Move to safe position; blocks until reached or timeout. Returns true if reached.
  virtual bool moveToSafePosition(double target_rad, double timeout_s) = 0;

  /// 急停：立即停止运动
  virtual void emergencyStop() = 0;

  /// 获取限位状态
  virtual LimitState getLimitState() const { return limit_state_; }

  /// 清除限位状态
  virtual void clearLimitState() { limit_state_ = LimitState::OK; }

  /// 检查是否有限位配置
  virtual bool hasLimits() const { return has_limits_; }

  const std::string & name() const { return name_; }

protected:
  explicit IJoint(std::string name) : name_(std::move(name)) {}
  std::string name_;
  LimitState limit_state_ = LimitState::OK;
  bool has_limits_ = false;
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__I_JOINT_HPP_
