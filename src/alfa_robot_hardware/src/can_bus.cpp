// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "alfa_robot_hardware/can_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr unsigned int kCanInterFrameDelayUs = 150;
}

namespace alfa_robot_hardware
{

CanBus::CanBus(const CanBusConfig & config)
: config_(config)
{
}

CanBus::~CanBus()
{
  // Safe cleanup: close all sockets
  closeCanInterface(can_socket_left_);
  closeCanInterface(can_socket_right_);
  closeCanInterface(can_socket_base_);
  closeCanInterface(can_socket_canopen_);
}

bool CanBus::openInterfaces()
{
  // Try to open each CAN bus — missing buses are non-fatal (WARN only)
  if (initCanInterface(config_.can_interface_left, can_socket_left_))
  {
    can_left_available_ = true;
    RCLCPP_INFO(rclcpp::get_logger("CanBus"),
      "Left arm CAN bus opened: %s", config_.can_interface_left.c_str());
  }
  else
  {
    can_left_available_ = false;
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "Left arm CAN bus '%s' not available", config_.can_interface_left.c_str());
  }

  if (initCanInterface(config_.can_interface_right, can_socket_right_))
  {
    can_right_available_ = true;
    RCLCPP_INFO(rclcpp::get_logger("CanBus"),
      "Right arm CAN bus opened: %s", config_.can_interface_right.c_str());
  }
  else
  {
    can_right_available_ = false;
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "Right arm CAN bus '%s' not available", config_.can_interface_right.c_str());
  }

  if (initCanInterface(config_.can_interface_base, can_socket_base_))
  {
    can_base_available_ = true;
    RCLCPP_INFO(rclcpp::get_logger("CanBus"),
      "Base CAN bus opened: %s", config_.can_interface_base.c_str());
  }
  else
  {
    can_base_available_ = false;
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "Base CAN bus '%s' not available", config_.can_interface_base.c_str());
  }

  if (initCanInterface(config_.can_interface_canopen, can_socket_canopen_))
  {
    can_canopen_available_ = true;
    RCLCPP_INFO(rclcpp::get_logger("CanBus"),
      "CANopen CAN bus opened: %s", config_.can_interface_canopen.c_str());
  }
  else
  {
    can_canopen_available_ = false;
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "CANopen CAN bus '%s' not available", config_.can_interface_canopen.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("CanBus"),
    "CAN interfaces: can0(%s) can1(%s) can2(%s) can3(%s)",
    can_left_available_ ? "OK" : "N/A",
    can_right_available_ ? "OK" : "N/A",
    can_base_available_ ? "OK" : "N/A",
    can_canopen_available_ ? "OK" : "N/A");

  return true;
}

void CanBus::closeInterfaces()
{
  closeCanInterface(can_socket_left_);
  can_left_available_ = false;
  closeCanInterface(can_socket_right_);
  can_right_available_ = false;
  closeCanInterface(can_socket_base_);
  can_base_available_ = false;
  closeCanInterface(can_socket_canopen_);
  can_canopen_available_ = false;
}

bool CanBus::enableMotors(
  const std::map<std::string, uint8_t> & rmd_joint_to_motor_id,
  const std::map<std::string, uint8_t> & canopen_joint_to_node_id)
{
  // ========== RMD motors: send 0x88 run command ==========
  uint8_t run_cmd_data[7] = {0};
  for (const auto & pair : rmd_joint_to_motor_id)
  {
    uint8_t motor_id = pair.second;
    int sock = getCanSocketForRmdMotorId(motor_id);
    if (sock < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("CanBus"),
        "Skipping RMD motor %d (joint '%s') — bus not available",
        motor_id, pair.first.c_str());
      continue;
    }
    sendMotorCommand(sock, motor_id, 0x88, run_cmd_data);
    RCLCPP_DEBUG(rclcpp::get_logger("CanBus"),
      "Sent run command to RMD motor %d (joint '%s')", motor_id, pair.first.c_str());
  }
  usleep(10000);

  // ========== CANopen motors: 6-phase activation ==========
  canopen_enabled_nodes_.clear();
  canopen_new_setpoint_active_.clear();
  canopen_last_target_pulses_.clear();
  pdo_cache_.clear();

  if (!can_canopen_available_)
  {
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "CANopen bus not available — skipping CANopen motor activation");
    return true;
  }

  // Phase 1: NMT reset all nodes → Pre-Operational
  for (const auto & pair : canopen_joint_to_node_id)
  {
    canopenNmtSend(can_socket_canopen_, 0x81, pair.second);
  }
  usleep(150000);

  // Phase 2: Dynamic PDO mapping (must be in Pre-Operational)
  for (const auto & pair : canopen_joint_to_node_id)
  {
    if (canopenConfigurePdo(can_socket_canopen_, pair.second))
    {
      canopen_enabled_nodes_.insert(pair.second);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "PDO mapping FAILED for node %d (joint '%s') — excluded from control",
        pair.second, pair.first.c_str());
    }
  }

  // Phase 3: NMT start all nodes → Operational (PDO active)
  for (uint8_t node_id : canopen_enabled_nodes_)
  {
    canopenNmtSend(can_socket_canopen_, 0x01, node_id);
  }
  usleep(20000);

  // Phase 4: CiA 402 state machine + mode setup
  std::set<uint8_t> fully_enabled;
  for (uint8_t node_id : canopen_enabled_nodes_)
  {
    uint8_t cw[2];
    cw[0] = 0x06; cw[1] = 0x00;
    if (!canopenSdoWrite(can_socket_canopen_, node_id, 0x6040, 0x00, cw, 2)) { continue; }
    usleep(5000);

    cw[0] = 0x07; cw[1] = 0x00;
    if (!canopenSdoWrite(can_socket_canopen_, node_id, 0x6040, 0x00, cw, 2)) { continue; }
    usleep(5000);

    cw[0] = 0x0F; cw[1] = 0x00;
    if (!canopenSdoWrite(can_socket_canopen_, node_id, 0x6040, 0x00, cw, 2)) { continue; }
    usleep(5000);

    uint8_t mode = 1;
    if (!canopenSdoWrite(can_socket_canopen_, node_id, 0x6060, 0x00, &mode, 1)) { continue; }

    uint8_t vel[4];
    memcpy(vel, &config_.canopen_profile_velocity, 4);
    if (!canopenSdoWrite(can_socket_canopen_, node_id, 0x6081, 0x00, vel, 4)) { continue; }

    uint8_t acc[4];
    memcpy(acc, &config_.canopen_profile_accel, 4);
    canopenSdoWrite(can_socket_canopen_, node_id, 0x6083, 0x00, acc, 4);
    canopenSdoWrite(can_socket_canopen_, node_id, 0x6084, 0x00, acc, 4);

    fully_enabled.insert(node_id);
    RCLCPP_INFO(rclcpp::get_logger("CanBus"),
      "CANopen node %d enabled (PP mode, PDO active)", node_id);
  }
  canopen_enabled_nodes_ = fully_enabled;

  // Phase 5: Read initial positions via SDO (last SDO before PDO-only)
  for (const auto & pair : canopen_joint_to_node_id)
  {
    if (canopen_enabled_nodes_.find(pair.second) == canopen_enabled_nodes_.end()) { continue; }

    double pos_m = 0.0;
    if (canopenReadPosition(pair.second, pos_m))
    {
      int32_t pulses = static_cast<int32_t>(pos_m * kCanopenPulsesPerMeter);
      pdo_cache_[pair.second].actual_position_pulses = pulses;
      pdo_cache_[pair.second].valid = true;
      canopen_last_target_pulses_[pair.second] = pulses;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("CanBus"),
        "Failed to read initial position for node %d (joint '%s')",
        pair.second, pair.first.c_str());
    }

    canopen_new_setpoint_active_[pair.second] = false;
  }

  // Phase 6: Prime SYNC cycle
  primeSyncCycle();

  RCLCPP_INFO(rclcpp::get_logger("CanBus"),
    "Motors enabled: RMD=%zu, CANopen=%zu/%zu",
    rmd_joint_to_motor_id.size(),
    canopen_enabled_nodes_.size(),
    canopen_joint_to_node_id.size());

  return true;
}

void CanBus::disableMotors(
  const std::map<std::string, uint8_t> & rmd_joint_to_motor_id,
  const std::map<std::string, uint8_t> & /*canopen_joint_to_node_id*/)
{
  // RMD: send 0x80 stop command
  uint8_t close_cmd_data[7] = {0};
  for (const auto & pair : rmd_joint_to_motor_id)
  {
    uint8_t motor_id = pair.second;
    int sock = getCanSocketForRmdMotorId(motor_id);
    if (sock < 0) { continue; }
    sendMotorCommand(sock, motor_id, 0x80, close_cmd_data);
  }
  usleep(10000);

  // CANopen: disable only enabled nodes
  if (can_canopen_available_)
  {
    for (uint8_t node_id : canopen_enabled_nodes_)
    {
      canopenDisableMotor(can_socket_canopen_, node_id);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("CanBus"), "Motors disabled");
}

void CanBus::stopAll(
  const std::map<std::string, uint8_t> & rmd_joint_to_motor_id,
  const std::map<std::string, uint8_t> & canopen_joint_to_node_id)
{
  disableMotors(rmd_joint_to_motor_id, canopen_joint_to_node_id);
  closeInterfaces();
}

AllJointState CanBus::readOnce(
  const std::map<std::string, uint8_t> & rmd_joint_to_motor_id)
{
  std::lock_guard<std::mutex> lock(control_mutex_);

  AllJointState state;

  // ========== CANopen: SYNC + drain TxPDO ==========
  if (can_canopen_available_ && !canopen_enabled_nodes_.empty())
  {
    canopenPdoReceiveAll(can_socket_canopen_);
    canopenSendSync(can_socket_canopen_);
    usleep(1500);
    canopenPdoReceiveAll(can_socket_canopen_);

    state.canopen_states = pdo_cache_;
  }

  // ========== RMD: send 0x92 + drain responses ==========
  uint8_t read_cmd_data[7] = {0};
  for (const auto & pair : rmd_joint_to_motor_id)
  {
    uint8_t motor_id = pair.second;
    int sock = getCanSocketForRmdMotorId(motor_id);
    if (sock < 0) { continue; }
    sendMotorCommand(sock, motor_id, 0x92, read_cmd_data);
  }

  drainRmdResponses(can_socket_left_, state.rmd_positions);
  drainRmdResponses(can_socket_right_, state.rmd_positions);
  drainRmdResponses(can_socket_base_, state.rmd_positions);

  return state;
}

void CanBus::writeOnce(
  const std::map<uint8_t, double> & rmd_cmds_rad,
  const std::map<uint8_t, double> & canopen_cmds_m,
  double dt)
{
  std::lock_guard<std::mutex> lock(control_mutex_);

  // ========== CANopen joints ==========
  if (can_canopen_available_ && !canopen_enabled_nodes_.empty())
  {
    for (const auto & pair : canopen_cmds_m)
    {
      uint8_t node_id = pair.first;
      if (canopen_enabled_nodes_.find(node_id) == canopen_enabled_nodes_.end()) { continue; }

      double cmd = pair.second;
      double filtered = cmd;

      // Low-pass filter
      if (config_.low_pass_filter_active && filter_initialized_)
      {
        double rc = 1.0 / (2.0 * M_PI * config_.filter_cutoff_hz);
        double alpha = dt / (dt + rc);
        filtered = prev_filtered_canopen_[node_id] + alpha * (cmd - prev_filtered_canopen_[node_id]);
      }
      prev_filtered_canopen_[node_id] = filtered;

      // Rate limiter
      if (config_.rate_limiter_active && filter_initialized_)
      {
        double delta = filtered - prev_cmd_canopen_[node_id];
        double max_delta = config_.max_velocity_m_per_s * dt;
        delta = std::clamp(delta, -max_delta, max_delta);
        filtered = prev_cmd_canopen_[node_id] + delta;
      }
      prev_cmd_canopen_[node_id] = filtered;

      int32_t target_pulses = static_cast<int32_t>(filtered * kCanopenPulsesPerMeter);
      uint16_t controlword = computeControlword(node_id, target_pulses);
      canopenPdoWritePosition(can_socket_canopen_, node_id, controlword, target_pulses);
    }
  }

  // ========== RMD joints ==========
  for (const auto & pair : rmd_cmds_rad)
  {
    uint8_t motor_id = pair.first;
    int sock = getCanSocketForRmdMotorId(motor_id);
    if (sock < 0) { continue; }

    double cmd = pair.second;
    double filtered = cmd;

    // Low-pass filter
    if (config_.low_pass_filter_active && filter_initialized_)
    {
      double rc = 1.0 / (2.0 * M_PI * config_.filter_cutoff_hz);
      double alpha = dt / (dt + rc);
      filtered = prev_filtered_rmd_[motor_id] + alpha * (cmd - prev_filtered_rmd_[motor_id]);
    }
    prev_filtered_rmd_[motor_id] = filtered;

    // Rate limiter
    if (config_.rate_limiter_active && filter_initialized_)
    {
      double delta = filtered - prev_cmd_rmd_[motor_id];
      double max_delta = config_.max_velocity_rad_per_s * dt;
      delta = std::clamp(delta, -max_delta, max_delta);
      filtered = prev_cmd_rmd_[motor_id] + delta;
    }
    prev_cmd_rmd_[motor_id] = filtered;

    uint8_t frame_data[7];
    convertPositionToCanFormat0xA4(filtered, frame_data);
    sendMotorCommand(sock, motor_id, 0xA4, frame_data);
  }

  filter_initialized_ = true;
}

const std::set<uint8_t> & CanBus::enabledCanopenNodes() const
{
  return canopen_enabled_nodes_;
}

bool CanBus::isCanopenAvailable() const
{
  return can_canopen_available_;
}

bool CanBus::canopenReadPosition(uint8_t node_id, double & position_m)
{
  uint8_t data[4];
  uint8_t size;
  if (!canopenSdoRead(can_socket_canopen_, node_id, 0x6064, 0x00, data, size))
  {
    return false;
  }
  int32_t pulses;
  memcpy(&pulses, data, 4);
  position_m = static_cast<double>(pulses) / kCanopenPulsesPerMeter;
  return true;
}

void CanBus::primeSyncCycle()
{
  if (!can_canopen_available_) { return; }
  canopenSendSync(can_socket_canopen_);
  usleep(2000);
  canopenPdoReceiveAll(can_socket_canopen_);
}


// ========== Socket management ==========

bool CanBus::initCanInterface(const std::string & interface, int & socket_fd)
{
  socket_fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
      "Failed to create CAN socket: %s", strerror(errno));
    return false;
  }

  struct ifreq ifr;
  strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
      "Failed to get CAN interface index for %s: %s",
      interface.c_str(), strerror(errno));
    close(socket_fd);
    socket_fd = -1;
    return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
      "Failed to bind CAN socket: %s", strerror(errno));
    close(socket_fd);
    socket_fd = -1;
    return false;
  }

  const int sndbuf_size = 65536;
  if (setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size)) < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "Failed to set SO_SNDBUF: %s", strerror(errno));
  }

  int flags = fcntl(socket_fd, F_GETFL, 0);
  if (flags >= 0)
  {
    fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
  }

  return true;
}

void CanBus::closeCanInterface(int & socket_fd)
{
  if (socket_fd >= 0)
  {
    close(socket_fd);
    socket_fd = -1;
  }
}

bool CanBus::sendCanFrame(int socket_fd, uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (socket_fd < 0)
  {
    return false;
  }

  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);

  ssize_t nbytes = ::write(socket_fd, &frame, sizeof(struct can_frame));
  if (nbytes < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_WARN(rclcpp::get_logger("CanBus"),
        "Failed to send CAN frame (ID: 0x%X): %s", can_id, strerror(errno));
    }
    return false;
  }

  if (nbytes != static_cast<ssize_t>(sizeof(struct can_frame)))
  {
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "Partial CAN frame sent: %zd bytes", nbytes);
    return false;
  }

  return true;
}

bool CanBus::receiveCanFrame(int socket_fd, uint32_t & can_id, uint8_t * data, uint8_t & dlc)
{
  if (socket_fd < 0)
  {
    return false;
  }

  struct can_frame frame;
  ssize_t nbytes = ::read(socket_fd, &frame, sizeof(struct can_frame));

  if (nbytes < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_WARN(rclcpp::get_logger("CanBus"),
        "Failed to receive CAN frame: %s", strerror(errno));
    }
    return false;
  }

  if (nbytes == static_cast<ssize_t>(sizeof(struct can_frame)))
  {
    can_id = frame.can_id & CAN_SFF_MASK;
    dlc = frame.can_dlc;
    memcpy(data, frame.data, dlc);
    return true;
  }

  return false;
}

// ========== RMD protocol ==========

void CanBus::sendMotorCommand(int socket_fd, uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data)
{
  uint32_t can_id = 0x140 + motor_id;
  uint8_t frame_data[8];
  frame_data[0] = cmd_byte;
  if (data != nullptr)
  {
    memcpy(&frame_data[1], data, 7);
  }
  else
  {
    memset(&frame_data[1], 0, 7);
  }

  if (sendCanFrame(socket_fd, can_id, frame_data, 8))
  {
    usleep(kCanInterFrameDelayUs);
  }
}

bool CanBus::parseMotorAngleReply0x92(const uint8_t * data, double & position_rad)
{
  if (data[0] != 0x92)
  {
    return false;
  }

  int64_t raw = static_cast<int64_t>(data[1]) |
    (static_cast<int64_t>(data[2]) << 8) |
    (static_cast<int64_t>(data[3]) << 16) |
    (static_cast<int64_t>(data[4]) << 24) |
    (static_cast<int64_t>(data[5]) << 32) |
    (static_cast<int64_t>(data[6]) << 40) |
    (static_cast<int64_t>(data[7]) << 48);

  if (data[7] & 0x80)
  {
    raw |= (static_cast<int64_t>(0xFFULL) << 56);
  }

  double angle_deg = static_cast<double>(raw) * 0.01;
  position_rad = angle_deg * M_PI / 180.0 / static_cast<double>(kGearRatio);
  return true;
}

void CanBus::convertPositionToCanFormat0xA4(double position_rad, uint8_t * frame_data)
{
  double angle_deg = position_rad * 180.0 / M_PI;
  int32_t angle_control = static_cast<int32_t>(angle_deg * 100.0 * static_cast<double>(kGearRatio));

  frame_data[0] = 0x00;
  frame_data[1] = static_cast<uint8_t>(config_.max_speed_dps & 0xFF);
  frame_data[2] = static_cast<uint8_t>((config_.max_speed_dps >> 8) & 0xFF);
  frame_data[3] = static_cast<uint8_t>(angle_control & 0xFF);
  frame_data[4] = static_cast<uint8_t>((angle_control >> 8) & 0xFF);
  frame_data[5] = static_cast<uint8_t>((angle_control >> 16) & 0xFF);
  frame_data[6] = static_cast<uint8_t>((angle_control >> 24) & 0xFF);
}

int CanBus::getCanSocketForRmdMotorId(uint8_t motor_id)
{
  if (motor_id == 1 && can_base_available_) { return can_socket_base_; }
  if (motor_id <= 3 && can_left_available_) { return can_socket_left_; }
  if (motor_id >= 4 && motor_id <= 6 && can_right_available_) { return can_socket_right_; }
  return -1;
}

void CanBus::drainRmdResponses(int socket_fd, std::map<uint8_t, RmdJointState> & positions)
{
  if (socket_fd < 0) { return; }

  for (size_t i = 0; i < 10; ++i)
  {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;

    if (!receiveCanFrame(socket_fd, can_id, data, dlc))
    {
      break;
    }

    if (can_id >= 0x140 && can_id <= 0x140 + 6 && dlc >= 8)
    {
      uint8_t motor_id = static_cast<uint8_t>(can_id - 0x140);
      double position_rad;
      if (parseMotorAngleReply0x92(data, position_rad))
      {
        positions[motor_id].position_rad = position_rad;
        positions[motor_id].valid = true;
      }
    }
  }
}


// ========== CANopen protocol ==========

bool CanBus::canopenNmtSend(int socket_fd, uint8_t command, uint8_t node_id)
{
  uint8_t data[2] = {command, node_id};
  return sendCanFrame(socket_fd, 0x000, data, 2);
}

bool CanBus::canopenSdoWrite(int socket_fd, uint8_t node_id,
  uint16_t index, uint8_t subindex, const uint8_t * data, uint8_t size)
{
  uint8_t cmd;
  switch (size)
  {
    case 1: cmd = 0x2F; break;
    case 2: cmd = 0x2B; break;
    case 3: cmd = 0x27; break;
    case 4: cmd = 0x23; break;
    default: return false;
  }

  uint8_t frame[8] = {0};
  frame[0] = cmd;
  frame[1] = static_cast<uint8_t>(index & 0xFF);
  frame[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
  frame[3] = subindex;
  for (uint8_t i = 0; i < size; ++i)
  {
    frame[4 + i] = data[i];
  }

  if (!sendCanFrame(socket_fd, 0x600 + node_id, frame, 8))
  {
    return false;
  }

  usleep(kCanInterFrameDelayUs);

  uint32_t resp_id;
  uint8_t resp_data[8];
  uint8_t resp_dlc;
  if (!receiveCanFrame(socket_fd, resp_id, resp_data, resp_dlc))
  {
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "CANopen SDO write timeout: node %d, index 0x%04X", node_id, index);
    return false;
  }

  if (resp_id != (0x580u + node_id) || resp_data[0] == 0x80)
  {
    RCLCPP_WARN(rclcpp::get_logger("CanBus"),
      "CANopen SDO write error: node %d, index 0x%04X", node_id, index);
    return false;
  }

  return true;
}

bool CanBus::canopenSdoRead(int socket_fd, uint8_t node_id,
  uint16_t index, uint8_t subindex, uint8_t * data, uint8_t & size)
{
  uint8_t frame[8] = {0};
  frame[0] = 0x40;
  frame[1] = static_cast<uint8_t>(index & 0xFF);
  frame[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
  frame[3] = subindex;

  if (!sendCanFrame(socket_fd, 0x600 + node_id, frame, 8))
  {
    return false;
  }

  usleep(kCanInterFrameDelayUs);

  uint32_t resp_id;
  uint8_t resp_data[8];
  uint8_t resp_dlc;
  if (!receiveCanFrame(socket_fd, resp_id, resp_data, resp_dlc))
  {
    return false;
  }

  if (resp_id != (0x580u + node_id) || resp_data[0] == 0x80)
  {
    return false;
  }

  uint8_t resp_cmd = resp_data[0];
  if (resp_cmd == 0x4F) { size = 1; }
  else if (resp_cmd == 0x4B) { size = 2; }
  else if (resp_cmd == 0x47) { size = 3; }
  else if (resp_cmd == 0x43) { size = 4; }
  else { size = 4; }

  memcpy(data, &resp_data[4], size);
  return true;
}

bool CanBus::canopenConfigurePdo(int socket_fd, uint8_t node_id)
{
  // TxPDO1: disable
  {
    uint32_t cob_id = (0x180u + node_id) | 0x80000000u;
    uint8_t d[4];
    memcpy(d, &cob_id, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1800, 0x01, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to disable TxPDO1", node_id);
      return false;
    }
  }

  // TxPDO1: transmission type = 1 (SYNC)
  {
    uint8_t tt = 1;
    if (!canopenSdoWrite(socket_fd, node_id, 0x1800, 0x02, &tt, 1))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to set TxPDO1 transmission type", node_id);
      return false;
    }
  }

  // TxPDO1: inhibit time = 1ms
  {
    uint16_t inhibit_time_100us = 10;
    uint8_t d[2];
    memcpy(d, &inhibit_time_100us, 2);
    canopenSdoWrite(socket_fd, node_id, 0x1800, 0x03, d, 2);
  }

  // TxPDO1: clear mapping
  {
    uint8_t zero = 0;
    if (!canopenSdoWrite(socket_fd, node_id, 0x1A00, 0x00, &zero, 1))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to clear TxPDO1 mapping", node_id);
      return false;
    }
  }

  // TxPDO1: map statusword (0x6041, 16bit)
  {
    uint32_t map = 0x60410010;
    uint8_t d[4];
    memcpy(d, &map, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1A00, 0x01, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to map TxPDO1 statusword", node_id);
      return false;
    }
  }

  // TxPDO1: map actual_position (0x6064, 32bit)
  {
    uint32_t map = 0x60640020;
    uint8_t d[4];
    memcpy(d, &map, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1A00, 0x02, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to map TxPDO1 actual_position", node_id);
      return false;
    }
  }

  // TxPDO1: set mapping count = 2
  {
    uint8_t count = 2;
    if (!canopenSdoWrite(socket_fd, node_id, 0x1A00, 0x00, &count, 1))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to set TxPDO1 mapping count", node_id);
      return false;
    }
  }

  // TxPDO1: re-enable
  {
    uint32_t cob_id = 0x180u + node_id;
    uint8_t d[4];
    memcpy(d, &cob_id, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1800, 0x01, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to re-enable TxPDO1", node_id);
      return false;
    }
  }

  // RxPDO1: disable
  {
    uint32_t cob_id = (0x200u + node_id) | 0x80000000u;
    uint8_t d[4];
    memcpy(d, &cob_id, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1400, 0x01, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to disable RxPDO1", node_id);
      return false;
    }
  }

  // RxPDO1: transmission type = 1 (SYNC)
  {
    uint8_t tt = 1;
    if (!canopenSdoWrite(socket_fd, node_id, 0x1400, 0x02, &tt, 1))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to set RxPDO1 transmission type", node_id);
      return false;
    }
  }

  // RxPDO1: clear mapping
  {
    uint8_t zero = 0;
    if (!canopenSdoWrite(socket_fd, node_id, 0x1600, 0x00, &zero, 1))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to clear RxPDO1 mapping", node_id);
      return false;
    }
  }

  // RxPDO1: map controlword (0x6040, 16bit)
  {
    uint32_t map = 0x60400010;
    uint8_t d[4];
    memcpy(d, &map, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1600, 0x01, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to map RxPDO1 controlword", node_id);
      return false;
    }
  }

  // RxPDO1: map target_position (0x607A, 32bit)
  {
    uint32_t map = 0x607A0020;
    uint8_t d[4];
    memcpy(d, &map, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1600, 0x02, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to map RxPDO1 target_position", node_id);
      return false;
    }
  }

  // RxPDO1: set mapping count = 2
  {
    uint8_t count = 2;
    if (!canopenSdoWrite(socket_fd, node_id, 0x1600, 0x00, &count, 1))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to set RxPDO1 mapping count", node_id);
      return false;
    }
  }

  // RxPDO1: re-enable
  {
    uint32_t cob_id = 0x200u + node_id;
    uint8_t d[4];
    memcpy(d, &cob_id, 4);
    if (!canopenSdoWrite(socket_fd, node_id, 0x1400, 0x01, d, 4))
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanBus"),
        "Node %d: failed to re-enable RxPDO1", node_id);
      return false;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("CanBus"),
    "Node %d: PDO mapping complete", node_id);
  return true;
}

bool CanBus::canopenDisableMotor(int socket_fd, uint8_t node_id)
{
  uint8_t cw[2];
  cw[0] = 0x07; cw[1] = 0x00;
  canopenSdoWrite(socket_fd, node_id, 0x6040, 0x00, cw, 2);
  usleep(5000);
  cw[0] = 0x06; cw[1] = 0x00;
  canopenSdoWrite(socket_fd, node_id, 0x6040, 0x00, cw, 2);
  return true;
}

bool CanBus::canopenSendSync(int socket_fd)
{
  uint8_t dummy[1] = {0};
  return sendCanFrame(socket_fd, 0x080, dummy, 0);
}

bool CanBus::canopenPdoWritePosition(int socket_fd, uint8_t node_id,
  uint16_t controlword, int32_t target_pulses)
{
  uint8_t data[8] = {0};

  data[0] = static_cast<uint8_t>(controlword & 0xFFu);
  data[1] = static_cast<uint8_t>((controlword >> 8) & 0xFFu);

  const uint32_t pos_u = static_cast<uint32_t>(target_pulses);
  data[2] = static_cast<uint8_t>(pos_u & 0xFFu);
  data[3] = static_cast<uint8_t>((pos_u >> 8) & 0xFFu);
  data[4] = static_cast<uint8_t>((pos_u >> 16) & 0xFFu);
  data[5] = static_cast<uint8_t>((pos_u >> 24) & 0xFFu);

  return sendCanFrame(socket_fd, 0x200u + node_id, data, 6);
}

void CanBus::canopenPdoReceiveAll(int socket_fd)
{
  constexpr size_t kMaxDrain = 32;

  for (size_t i = 0; i < kMaxDrain; ++i)
  {
    uint32_t can_id;
    uint8_t raw[8];
    uint8_t dlc;

    if (!receiveCanFrame(socket_fd, can_id, raw, dlc))
    {
      break;
    }

    if (can_id < 0x181 || can_id > 0x1FF || dlc < 6)
    {
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(can_id - 0x180u);

    const uint16_t statusword =
      static_cast<uint16_t>(raw[0]) |
      (static_cast<uint16_t>(raw[1]) << 8);

    const uint32_t pos_u =
      static_cast<uint32_t>(raw[2]) |
      (static_cast<uint32_t>(raw[3]) << 8) |
      (static_cast<uint32_t>(raw[4]) << 16) |
      (static_cast<uint32_t>(raw[5]) << 24);
    const int32_t actual_position = static_cast<int32_t>(pos_u);

    auto & state = pdo_cache_[node_id];
    state.statusword = statusword;
    state.actual_position_pulses = actual_position;
    state.valid = true;
  }
}

uint16_t CanBus::computeControlword(uint8_t node_id, int32_t target_pulses)
{
  bool & ns_active = canopen_new_setpoint_active_[node_id];
  int32_t & last_target = canopen_last_target_pulses_[node_id];

  bool target_changed = (target_pulses != last_target);

  uint16_t controlword;
  if (target_changed && ns_active) {
    controlword = 0x002F;
    ns_active = false;
    last_target = target_pulses;
  } else if (target_changed || !ns_active) {
    controlword = 0x003F;
    ns_active = true;
    last_target = target_pulses;
  } else {
    controlword = 0x003F;
  }

  return controlword;
}

}  // namespace alfa_robot_hardware
