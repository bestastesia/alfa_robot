#include "alfa_robot_hardware/alfa_robot_hardware.hpp"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

hardware_interface::CallbackReturn AlfaRobotHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Defaults
  rmd_left_cfg_        = {"can0", 1800};
  rmd_right_cfg_       = {"can1", 1800};
  rmd_base_cfg_        = {"can2", 1800};
  canopen_cfg_         = {"can3", 50000, 50000};
  canopen_plate_cfg_   = {"can4", 50000, 50000};
  zeroerr_left_cfg_    = {"can0", 200, 524288};  // ZeroErr on can0: gear_ratio=200, encoder=524288
  cylinder_cfg_        = {"can0", 3, 2000000.0, 0};   // Cylinder on can0: Node 3, 10000 pulses/5mm = 2M pulses/m, zero_offset=-24900000

  for (const auto & [key, val] : info_.hardware_parameters) {
    if      (key == "can_interface_left")    { rmd_left_cfg_.interface       = val; }
    else if (key == "can_interface_right")   { rmd_right_cfg_.interface      = val; }
    else if (key == "can_interface_base")    { rmd_base_cfg_.interface       = val; }
    else if (key == "can_interface_canopen") { canopen_cfg_.interface        = val; }
    else if (key == "can_interface_plate")   { canopen_plate_cfg_.interface  = val; }
    else if (key == "max_speed_dps") {
      try {
        uint16_t v = static_cast<uint16_t>(std::stoul(val));
        rmd_left_cfg_.max_speed_dps = rmd_right_cfg_.max_speed_dps =
          rmd_base_cfg_.max_speed_dps = v;
      } catch (...) {}
    }
    else if (key == "canopen_profile_velocity") {
      try { canopen_cfg_.profile_velocity = static_cast<uint32_t>(std::stoul(val)); }
      catch (...) {}
    }
    else if (key == "canopen_profile_accel") {
      try { canopen_cfg_.profile_accel = static_cast<uint32_t>(std::stoul(val)); }
      catch (...) {}
    }
    else if (key == "use_safe_shutdown") {
      use_safe_shutdown_ = (val == "true");
    }
    else if (key.find("safe_position_") == 0) {
      try { safe_positions_[key.substr(14)] = std::stod(val); }
      catch (...) {}
    }
    // 位置误差监控参数
    else if (key == "position_error_threshold_dynamic") {
      try { position_error_threshold_dynamic_ = std::stod(val); }
      catch (...) {}
    }
    else if (key == "position_error_threshold_static") {
      try { position_error_threshold_static_ = std::stod(val); }
      catch (...) {}
    }
    else if (key == "position_error_tolerance_time") {
      try { position_error_tolerance_time_ = std::stod(val); }
      catch (...) {}
    }
    else if (key == "position_error_check_enabled") {
      position_error_check_enabled_ = (val == "true");
    }
  }

  // Create drivers and joints here so export_state/command_interfaces() works
  // immediately after on_init (ros2_control calls them before on_configure).
  rmd_left_       = std::make_unique<RmdDriver>(rmd_left_cfg_);
  rmd_right_      = std::make_unique<RmdDriver>(rmd_right_cfg_);
  rmd_base_       = std::make_unique<RmdDriver>(rmd_base_cfg_);
  canopen_        = std::make_unique<CanopenDriver>(canopen_cfg_);
  canopen_plate_  = std::make_unique<CanopenDriver>(canopen_plate_cfg_);
  // Mixed protocol on can0: ZeroErr driver for Node 1,2 (custom CAN protocol)
  zeroerr_left_   = std::make_unique<ZeroerrDriver>(zeroerr_left_cfg_);
  // Cylinder on can0: Node 3 (IDS830ABS linear actuator)
  cylinder_       = std::make_unique<CylinderDriver>(cylinder_cfg_);
  buildJoints();

  // 为所有关节设置位置误差监控参数
  if (position_error_check_enabled_) {
    for (auto & joint : joints_) {
      joint->setPositionErrorParams(
        position_error_threshold_dynamic_,
        position_error_threshold_static_,
        position_error_tolerance_time_);
    }
    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
      "Position error monitoring enabled: dynamic=%.3f, static=%.3f, time=%.2fs",
      position_error_threshold_dynamic_, position_error_threshold_static_, position_error_tolerance_time_);
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "on_init OK");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_configure(
  const rclcpp_lifecycle::State &)
{
  rmd_left_->open();    // Non-fatal if bus absent
  rmd_right_->open();
  rmd_base_->open();
  canopen_->open();
  canopen_plate_->open();
  zeroerr_left_->open();  // ZeroErr motors on can0
  cylinder_->open();      // Cylinder on can0 (Node 3)

  // 创建急停话题订阅节点（软件触发接口）
  estop_node_ = rclcpp::Node::make_shared("emergency_stop_interface");
  estop_sub_ = estop_node_->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop_trigger",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      emergencyStopCallback(msg);
    });
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "Emergency stop topic subscriber created: /emergency_stop_trigger");

  // 启动急停 CAN 监听
  if (openEstopSocket()) {
    estop_monitor_running_ = true;
    estop_monitor_thread_ = std::thread(&AlfaRobotHW::emergencyStopMonitorThread, this);
    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Emergency stop monitor started on can0");
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "on_configure OK, %zu joints", joints_.size());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Enable motors per bus
  // Mixed protocol on can0: Node 1,2 are ZeroErr (custom CAN), Node 3 is Cylinder (IDS830ABS), Node 4 is RMD (leftjoint5)
  cylinder_->enable();                   // Node 3 (leftjoint4 - cylinder)
  cylinder_->setVelocity(0.05);          // Set velocity to 0.05 m/s (50 mm/s) for cylinder
  rmd_left_->enableMotors({4});          // Node 4 (leftjoint5 - RMD protocol)

  // ZeroErr motors on can0: Full initialization sequence per datasheet
  zeroerr_left_->enableMotors({1, 2});     // Node 1,2 (leftjoint2/3) - 01 00 00 00 00 01
  zeroerr_left_->setPositionMode({1, 2});  // 00 4E 00 00 00 03
  zeroerr_left_->setMotionMode({1, 2}, 1); // 00 8D 00 00 00 01 (1=absolute position)
  zeroerr_left_->setMotionParams({1, 2});  // 00 88/89/8A - accel/decel/velocity

  rmd_right_->enableMotors({4, 5, 6});   // rightjoint2/3/4
  rmd_base_->enableMotors({1});           // turn
  canopen_->enableNodes({1, 2, 3, 4, 5});
  canopen_plate_->enableNodes({1});  // plate

  // Activate all joints (reads initial position)
  for (auto & joint : joints_) { joint->activate(); }

  // Capture "turn" joint current position as software zero
  for (auto & joint : joints_) {
    if (joint->name() == "turn") {
      auto * rmd_joint = dynamic_cast<RmdJoint *>(joint.get());
      if (rmd_joint) { rmd_joint->captureCurrentPositionAsZero(); }
    }
  }


  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // 停止急停监听线程
  estop_monitor_running_ = false;
  if (estop_monitor_thread_.joinable()) {
    estop_monitor_thread_.join();
  }
  closeEstopSocket();

  // 清理急停话题订阅节点
  estop_sub_.reset();
  estop_node_.reset();

  if (use_safe_shutdown_ && !safe_positions_.empty()) {
    moveAllToSafePositions(5.0);
  }

  // Mixed protocol on can0
  cylinder_->disable();                  // Node 3 (leftjoint4 - cylinder)
  rmd_left_->disableMotors({4});         // Node 4 (leftjoint5 - RMD)
  zeroerr_left_->disableMotors({1, 2});  // Node 1,2 (leftjoint2/3)

  rmd_right_->disableMotors({4, 5, 6});
  rmd_base_->disableMotors({1});
  canopen_->disableNodes({1, 2, 3, 4, 5});
  canopen_plate_->disableNodes({1});  // plate

  rmd_left_->close();
  rmd_right_->close();
  rmd_base_->close();
  canopen_->close();
  canopen_plate_->close();
  zeroerr_left_->close();
  cylinder_->close();

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware deactivated");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AlfaRobotHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (auto & joint : joints_) {
    auto joint_si = joint->exportStateInterfaces();
    for (auto & iface : joint_si) { si.push_back(std::move(iface)); }
  }
  // 导出急停状态接口
  si.emplace_back("emergency_stop", "state", &emergency_stop_state_);
  return si;
}

std::vector<hardware_interface::CommandInterface> AlfaRobotHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (auto & joint : joints_) {
    auto joint_ci = joint->exportCommandInterfaces();
    for (auto & iface : joint_ci) { ci.push_back(std::move(iface)); }
  }
  return ci;
}

hardware_interface::return_type AlfaRobotHW::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // 处理急停话题订阅的消息
  if (estop_node_) {
    rclcpp::spin_some(estop_node_);
  }

  double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.0;
  // One batched read per bus - sends all 0x92, then drains with a poll() budget.
  // Joints subsequently read from the driver's position cache.
  // Mixed protocol on can0: Node 1,2 via ZeroErr (custom CAN), Node 3 via Cylinder (IDS830ABS), Node 4 via RMD (leftjoint5)
  // IMPORTANT: All can0 reads must be sequential to avoid CAN bus contention
  rmd_left_->readPositions({4});         // can0: Node 4 (leftjoint5 - RMD)
  {                                      // can0: Node 3 (leftjoint4 - Cylinder)
    double dummy;
    cylinder_->readPosition(dummy);
  }
  zeroerr_left_->readPositions({1, 2});  // can0: ZeroErr motors (Node 1,2)

  rmd_right_->readPositions({4, 5, 6});
  rmd_base_->readPositions({1});
  canopen_->readPositions();             // can3: one SYNC per cycle, updates PDO cache
  canopen_plate_->readPositions();       // can4: plate bus

  for (auto & joint : joints_) { joint->read(dt); }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlfaRobotHW::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // 急停激活时不执行任何写操作
  if (emergency_stop_active_.load()) {
    return hardware_interface::return_type::OK;
  }

  double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.005;
  for (auto & joint : joints_) { joint->write(dt); }
  return hardware_interface::return_type::OK;
}

// ── Private ──────────────────────────────────────────────────────────────────

void AlfaRobotHW::buildJoints()
{
  // RMD joints - base bus (can2)
  joints_.push_back(std::make_unique<RmdJoint>("turn",
    RmdJoint::Config{1, 0.0, 0.0, -1.0, 2.394}, *rmd_base_));

  // Left bus (can0) - mixed protocol
  // Node 1,2: ZeroErr rotary motors (gear_ratio=200:1, encoder_resolution=524288 pulses/rev)
  // 零点位置：通过 CAN 命令手动读取 (cansend can0 64X#00.02)
  // Node 1 (leftjoint2): 0x00040000 = 262,144 脉冲 (机械零点)
  // Node 2 (leftjoint3): 0x00040000 = 262,144 脉冲 (机械零点)

  // leftjoint2: 无限位 (continuous，理论可无限旋转)
  joints_.push_back(std::make_unique<ZeroerrJoint>("leftjoint2",
    ZeroerrJoint::Config{1, 0.0, -1.0, 262144,
      -100.0, 100.0, false},  // disable_limits=true，无限制
    *zeroerr_left_));

  // leftjoint3: -π ~ 0.3 rad
  joints_.push_back(std::make_unique<ZeroerrJoint>("leftjoint3",
    ZeroerrJoint::Config{2, 0.0, -1.0, 262144,
      -M_PI, 0.3, true},  // 限位: -π ~ 0.3 rad
    *zeroerr_left_));

  // Node 3: IDS830ABS Cylinder (leftjoint4 - linear actuator, 15cm travel)
  // 限位: 0 ~ 0.15 m
  joints_.push_back(std::make_unique<CylinderJoint>("leftjoint4",
    CylinderJoint::Config{3, 0.0, 1.0, 0.0, 0.15, true},  // 限位: min_travel=0, max_travel=0.15m
    *cylinder_));

  // Node 4: RMD motor (leftjoint5 - rotary)
  // 限位: -0.5π ~ 0.5π rad
  joints_.push_back(std::make_unique<RmdJoint>("leftjoint5",
    RmdJoint::Config{4, 0.0, 0.0, -1.0, 0.0,
      -M_PI_2, M_PI_2, true},  // 限位: -π/2 ~ π/2
    *rmd_left_));

  // RMD joints - right bus (can1)
  joints_.push_back(std::make_unique<RmdJoint>("rightjoint2",
    RmdJoint::Config{4, 0.0, 0.0, -1.0, 0.0}, *rmd_right_));
  joints_.push_back(std::make_unique<RmdJoint>("rightjoint3",
    RmdJoint::Config{5, 0.0, 0.0, -1.0, 0.0}, *rmd_right_));
  joints_.push_back(std::make_unique<RmdJoint>("rightjoint4",
    RmdJoint::Config{6, 0.0, 0.0, -1.0, 0.0}, *rmd_right_));

  // CANopen joints (can3) - linear actuators
  joints_.push_back(std::make_unique<CanopenJoint>("updown",
    CanopenJoint::Config{1, 1.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("leftarmbase",
    CanopenJoint::Config{2, 3.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("leftjoint1",
    CanopenJoint::Config{3, 1.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("rightarmbase",
    CanopenJoint::Config{4, 3.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("rightjoint1",
    CanopenJoint::Config{5, 1.0, 0.0, 0.0, -1.0}, *canopen_));
  // plate - separate CANopen bus (can4), node 1
  joints_.push_back(std::make_unique<CanopenJoint>("plate",
    CanopenJoint::Config{1, 1.0, 0.0, 0.0, -1.0}, *canopen_plate_));

}

bool AlfaRobotHW::moveAllToSafePositions(double timeout_s)
{
  bool all_ok = true;
  for (const auto & [joint_name, safe_pos] : safe_positions_) {
    for (auto & joint : joints_) {
      if (joint->name() == joint_name) {
        if (!joint->moveToSafePosition(safe_pos, timeout_s)) { all_ok = false; }
        break;
      }
    }
  }
  return all_ok;
}

// ── Emergency Stop ───────────────────────────────────────────────────────────

void AlfaRobotHW::emergencyStop()
{
  if (emergency_stop_active_.exchange(true)) {
    return;  // 已经处于急停状态
  }

  RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"), "!!! EMERGENCY STOP TRIGGERED !!!");

  // 停止所有关节
  for (auto & joint : joints_) {
    joint->emergencyStop();
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "All motors stopped. Emergency stop active.");
}

void AlfaRobotHW::clearEmergencyStop()
{
  if (!emergency_stop_active_.load()) {
    return;  // 未处于急停状态
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Clearing emergency stop state...");
  emergency_stop_active_ = false;

  // 清除各关节的限位状态
  for (auto & joint : joints_) {
    joint->clearLimitState();
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Emergency stop cleared.");
}

bool AlfaRobotHW::openEstopSocket()
{
  // 打开 can0 用于监听急停信号
  estop_socket_fd_ = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (estop_socket_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to create emergency stop socket: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(estop_socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to get can0 interface index: %s", std::strerror(errno));
    ::close(estop_socket_fd_);
    estop_socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(estop_socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to bind emergency stop socket: %s", std::strerror(errno));
    ::close(estop_socket_fd_);
    estop_socket_fd_ = -1;
    return false;
  }

  // 设置过滤器：只接收急停帧 ID (0x7FF)
  struct can_filter rfilter[1];
  rfilter[0].can_id   = kEmergencyStopCanId;
  rfilter[0].can_mask = CAN_SFF_MASK;  // 精确匹配
  setsockopt(estop_socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  // 设置非阻塞
  int flags = fcntl(estop_socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(estop_socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "Emergency stop socket opened on can0, listening for ID 0x%03X", kEmergencyStopCanId);
  return true;
}

void AlfaRobotHW::closeEstopSocket()
{
  if (estop_socket_fd_ >= 0) {
    ::close(estop_socket_fd_);
    estop_socket_fd_ = -1;
  }
}

void AlfaRobotHW::emergencyStopMonitorThread()
{
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Emergency stop monitor thread started");

  struct pollfd pfd;
  pfd.fd = estop_socket_fd_;
  pfd.events = POLLIN;

  while (estop_monitor_running_.load()) {
    int rc = ::poll(&pfd, 1, 100);  // 100ms 超时
    if (rc <= 0) {
      continue;  // 超时或错误，继续循环
    }

    // 读取 CAN 帧
    struct can_frame frame;
    ssize_t received = ::read(estop_socket_fd_, &frame, sizeof(frame));
    if (received != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    uint32_t can_id = frame.can_id & CAN_SFF_MASK;
    if (can_id == kEmergencyStopCanId && frame.can_dlc >= 1) {
      // 急停帧格式：Data[0] = 0x01 (激活) 或 0x00 (解除)
      if (frame.data[0] == 0x01) {
        RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
          "Emergency stop CAN frame received (ID=0x%03X, data=0x%02X)", can_id, frame.data[0]);
        emergencyStop();
      } else if (frame.data[0] == 0x00) {
        RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
          "Emergency stop clear CAN frame received (ID=0x%03X, data=0x%02X)", can_id, frame.data[0]);
        clearEmergencyStop();
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Emergency stop monitor thread stopped");
}

void AlfaRobotHW::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
      "Emergency stop triggered via ROS topic");
    emergencyStop();
  } else {
    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
      "Emergency stop cleared via ROS topic");
    clearEmergencyStop();
  }
}

}  // namespace alfa_robot_hardware

PLUGINLIB_EXPORT_CLASS(alfa_robot_hardware::AlfaRobotHW, hardware_interface::SystemInterface)