// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef ALFA_ROBOT_HARDWARE__JOINT__CYLINDER_JOINT_HPP_
#define ALFA_ROBOT_HARDWARE__JOINT__CYLINDER_JOINT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "alfa_robot_hardware/joint/i_joint.hpp"
#include "alfa_robot_hardware/driver/cylinder_driver.hpp"

namespace alfa_robot_hardware
{

/**
 * @brief 电缸关节实现
 *
 * 将 CylinderDriver 的位置 (米) 与 ros2_control 接口集成。
 */
class CylinderJoint : public IJoint
{
public:
  struct Config {
    uint8_t node_id;        // CAN 节点 ID
    double offset{0.0};     // 位置偏置 (米)
    double sign{1.0};       // 方向符号 (+1 或 -1)
    double min_travel{0.0}; // 最小行程 (米)
    double max_travel{0.15}; // 最大行程 (米，默认 15cm)
  };

  CylinderJoint(const std::string & name, Config config, CylinderDriver & driver);
  ~CylinderJoint() override = default;

  /// 从 driver 读取位置并更新状态接口
  void read(double dt) override;

  /// 将命令接口的位置写入 driver
  void write(double dt) override;

  /// 激活关节 (读取初始位置)
  bool activate() override;

  /// 停用关节
  void deactivate() override;

  /// 移动到安全位置
  bool moveToSafePosition(double safe_position_m, double timeout_s) override;

  /// 导出状态接口
  std::vector<hardware_interface::StateInterface> exportStateInterfaces() override;

  /// 导出命令接口
  std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() override;

private:
  Config config_;
  CylinderDriver & driver_;

  double position_state_{0.0};     // 当前位置 (米)
  double position_command_{0.0};   // 目标位置 (米)
  double velocity_state_{0.0};     // 当前速度 (米/秒)
  double acceleration_state_{0.0}; // 当前加速度 (米/秒²)
  double last_position_{0.0};      // 上一周期位置
  double last_velocity_{0.0};      // 上一周期速度

  /// 驱动使能状态
  bool driver_enabled_{false};
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__JOINT__CYLINDER_JOINT_HPP_
