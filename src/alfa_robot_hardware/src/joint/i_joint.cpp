// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "alfa_robot_hardware/joint/i_joint.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

void IJoint::checkPositionError(double dt, double position_state, double position_command)
{
  if (!position_error_check_enabled_) {
    return;
  }

  // 计算误差
  position_error_ = std::abs(position_command - position_state);

  // 判断是否静态（命令位置变化很小）
  bool is_static = std::abs(position_command - prev_position_command_) < 0.001;
  prev_position_command_ = position_command;

  // 选择阈值
  double threshold = is_static ? position_error_threshold_static_ : position_error_threshold_dynamic_;

  if (position_error_ > threshold) {
    // 误差超过阈值
    position_error_time_ += dt;

    if (position_error_time_ > position_error_tolerance_time_) {
      // 超过容忍时间，触发 ERROR
      if (position_error_state_ != PositionErrorState::ERROR) {
        RCLCPP_WARN(rclcpp::get_logger("IJoint"),
          "Joint '%s' position error exceeded: error=%.4f, threshold=%.4f, time=%.2fs",
          name_.c_str(), position_error_, threshold, position_error_time_);
      }
      position_error_state_ = PositionErrorState::ERROR;
    } else {
      // 计时中，WARNING 状态
      position_error_state_ = PositionErrorState::WARNING;
    }
  } else {
    // 误差恢复正常
    position_error_time_ = 0.0;
    position_error_state_ = PositionErrorState::OK;
  }
}

}  // namespace alfa_robot_hardware
