#ifndef ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
#define ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "alfa_robot_hardware/joint/i_joint.hpp"
#include "alfa_robot_hardware/joint/rmd_joint.hpp"
#include "alfa_robot_hardware/joint/canopen_joint.hpp"
#include "alfa_robot_hardware/joint/placeholder_joint.hpp"
#include "alfa_robot_hardware/joint/zeroerr_joint.hpp"
#include "alfa_robot_hardware/joint/cylinder_joint.hpp"
#include "alfa_robot_hardware/driver/rmd_driver.hpp"
#include "alfa_robot_hardware/driver/canopen_driver.hpp"
#include "alfa_robot_hardware/driver/zeroerr_driver.hpp"
#include "alfa_robot_hardware/driver/cylinder_driver.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace alfa_robot_hardware
{

/// 急停 CAN 帧 ID (广播地址)
constexpr uint32_t kEmergencyStopCanId = 0x7FF;

class AlfaRobotHW : public hardware_interface::SystemInterface
{
public:
  AlfaRobotHW() = default;

  // 析构函数确保资源正确释放（即使在异常情况下）
  ~AlfaRobotHW()
  {
    // 停止急停监听线程
    estop_monitor_running_.store(false);
    if (estop_monitor_thread_.joinable()) {
      estop_monitor_thread_.join();
    }
    // 关闭所有 CAN 接口
    closeEstopSocket();
    if (rmd_left_) rmd_left_->close();
    if (rmd_right_) rmd_right_->close();
    if (rmd_base_) rmd_base_->close();
    if (canopen_) canopen_->close();
    if (canopen_plate_) canopen_plate_->close();
    if (zeroerr_left_) zeroerr_left_->close();
    if (cylinder_) cylinder_->close();
  }

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// 急停：立即停止所有电机
  void emergencyStop();

  /// 检查是否处于急停状态
  bool isEmergencyStopActive() const { return emergency_stop_active_.load(); }

  /// 清除急停状态
  void clearEmergencyStop();

private:
  // Drivers (owners)
  std::unique_ptr<RmdDriver>     rmd_left_, rmd_right_, rmd_base_;
  std::unique_ptr<CanopenDriver> canopen_;
  std::unique_ptr<CanopenDriver> canopen_plate_;
  std::unique_ptr<ZeroerrDriver> zeroerr_left_;    // ZeroErr motors on can0 (mixed protocol)
  std::unique_ptr<CylinderDriver> cylinder_;       // Cylinder (left_joint4) on can0

  // All joints (single list — no type dispatch in AlfaRobotHW)
  std::vector<std::unique_ptr<IJoint>> joints_;

  // Safe shutdown config
  std::map<std::string, double> safe_positions_;
  bool use_safe_shutdown_{false};

  // Position error monitoring config
  double position_error_threshold_dynamic_{0.1};   // 动态阈值 (rad)
  double position_error_threshold_static_{0.02};   // 静态阈值 (rad)
  double position_error_tolerance_time_{1.0};      // 容忍时间 (s)
  bool position_error_check_enabled_{false};       // 是否启用

  // Driver configs (parsed in on_init)
  RmdDriver::Config     rmd_left_cfg_, rmd_right_cfg_, rmd_base_cfg_;
  CanopenDriver::Config canopen_cfg_;
  CanopenDriver::Config canopen_plate_cfg_;
  ZeroerrDriver::Config zeroerr_left_cfg_;
  CylinderDriver::Config cylinder_cfg_;

  // Emergency stop
  std::atomic<bool> emergency_stop_active_{false};
  int estop_socket_fd_{-1};
  std::thread estop_monitor_thread_;
  std::atomic<bool> estop_monitor_running_{false};
  double emergency_stop_state_{0.0};  // 状态接口值：0=正常，1=急停激活

  // 急停话题订阅（软件触发）
  rclcpp::Node::SharedPtr estop_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  void buildJoints();
  bool moveAllToSafePositions(double timeout_s);

  // 急停 CAN 监听线程
  void emergencyStopMonitorThread();
  bool openEstopSocket();
  void closeEstopSocket();

  // 急停话题回调
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__ALFA_ROBOT_HARDWARE_HPP_
