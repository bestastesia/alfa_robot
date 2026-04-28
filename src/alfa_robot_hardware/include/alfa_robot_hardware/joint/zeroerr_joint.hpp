// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef ALFA_ROBOT_HARDWARE__JOINT__ZEROERR_JOINT_HPP_
#define ALFA_ROBOT_HARDWARE__JOINT__ZEROERR_JOINT_HPP_

#include <memory>
#include <string>

#include "alfa_robot_hardware/joint/i_joint.hpp"
#include "alfa_robot_hardware/driver/zeroerr_driver.hpp"

namespace alfa_robot_hardware
{

/**
 * @brief ZeroErr 协议关节实现
 *
 * 将 ZeroerrDriver 的位置 (脉冲) 转换为弧度，
 * 并实现 IJoint 接口以集成到 ros2_control 框架。
 */
class ZeroerrJoint : public IJoint
{
public:
  struct Config {
    uint8_t node_id;        // CAN 节点 ID
    double offset{0.0};     // 位置偏置 (弧度)
    double sign{-1.0};      // 方向符号 (+1 或 -1)
    int32_t initial_counts{0};  // 零点位置 (脉冲数)
  };

  ZeroerrJoint(const std::string & name, Config config, ZeroerrDriver & driver);
  ~ZeroerrJoint() override = default;

  /// 从 driver 读取位置并更新状态接口
  void read(double dt) override;

  /// 将命令接口的位置写入 driver
  void write(double dt) override;

  /// 激活关节 (读取初始位置)
  bool activate() override;

  /// 停用关节
  void deactivate() override;

  /// 移动到安全位置
  bool moveToSafePosition(double safe_position_rad, double timeout_s) override;

  /// 导出状态接口
  std::vector<hardware_interface::StateInterface> exportStateInterfaces() override;

  /// 导出命令接口
  std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

private:
  Config config_;
  ZeroerrDriver & driver_;

  double position_state_{0.0};    // 当前位置 (弧度)
  double position_command_{0.0};  // 目标位置 (弧度)
  double velocity_state_{0.0};    // 当前速度 (弧度/秒)
  double acceleration_state_{0.0}; // 当前加速度 (弧度/秒²)
  double last_position_{0.0};     // 上一周期位置 (用于计算速度)
  double last_velocity_{0.0};     // 上一周期速度 (用于计算加速度)

  int32_t initial_position_counts_{0};  // 激活时的初始位置 (脉冲)

  /// 脉冲转弧度
  double countsToRadians(int32_t counts) const;

  /// 弧度转脉冲
  int32_t radiansToCounts(double radians) const;
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__ZEROERR_JOINT_HPP_
