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

  /// 位置误差状态枚举
  enum class PositionErrorState {
    OK,           ///< 正常
    WARNING,      ///< 误差超过阈值，计时中
    ERROR,        ///< 误差超时，触发报警
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

  /// 获取位置误差值
  virtual double getPositionError() const { return position_error_; }

  /// 获取位置误差状态
  virtual PositionErrorState getPositionErrorState() const { return position_error_state_; }

  /// 检查位置误差监控是否启用
  virtual bool isPositionErrorCheckEnabled() const { return position_error_check_enabled_; }

  /// 设置位置误差监控参数
  virtual void setPositionErrorParams(double threshold_dynamic, double threshold_static, double tolerance_time) {
    position_error_threshold_dynamic_ = threshold_dynamic;
    position_error_threshold_static_ = threshold_static;
    position_error_tolerance_time_ = tolerance_time;
    position_error_check_enabled_ = true;
  }

  /// 清除位置误差状态
  virtual void clearPositionErrorState() {
    position_error_state_ = PositionErrorState::OK;
    position_error_time_ = 0.0;
  }

  const std::string & name() const { return name_; }

protected:
  explicit IJoint(std::string name) : name_(std::move(name)) {}
  std::string name_;
  LimitState limit_state_ = LimitState::OK;
  bool has_limits_ = false;

  // 位置误差监控
  double position_error_{0.0};
  double position_error_time_{0.0};
  PositionErrorState position_error_state_ = PositionErrorState::OK;
  double position_error_threshold_dynamic_{0.1};   // 动态阈值
  double position_error_threshold_static_{0.02};   // 静态阈值
  double position_error_tolerance_time_{1.0};      // 容忍时间
  bool position_error_check_enabled_{false};
  double prev_position_command_{0.0};

  /// 检查位置误差（在 read() 中调用）
  void checkPositionError(double dt, double position_state, double position_command);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__I_JOINT_HPP_
