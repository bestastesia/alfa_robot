// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "alfa_robot_hardware/driver/cylinder_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

CylinderDriver::CylinderDriver(Config cfg)
: config_(std::move(cfg))
{
}

CylinderDriver::~CylinderDriver()
{
  close();
}

bool CylinderDriver::open()
{
  socket_fd_ = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CylinderDriver"),
      "Failed to create CAN socket: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CylinderDriver"),
      "Failed to get interface index for %s: %s", config_.interface.c_str(), std::strerror(errno));
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CylinderDriver"),
      "Failed to bind CAN socket: %s", std::strerror(errno));
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  const int sndbuf = 65536;
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  struct can_filter rfilter[1];
  rfilter[0].can_id   = static_cast<canid_t>(config_.node_id + 0x100);
  rfilter[0].can_mask = CAN_SFF_MASK;
  setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  RCLCPP_INFO(rclcpp::get_logger("CylinderDriver"),
    "Opened %s (node_id=%d, pulses_per_meter=%.0f, zero_offset=%d)",
    config_.interface.c_str(), config_.node_id, config_.pulses_per_meter, config_.zero_offset);

  return true;
}

void CylinderDriver::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  enabled_ = false;
}

bool CylinderDriver::sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (socket_fd_ < 0) { return false; }

  struct can_frame frame;
  frame.can_id = can_id & 0x7FF;
  frame.can_dlc = std::min(dlc, static_cast<uint8_t>(8));
  std::memcpy(frame.data, data, frame.can_dlc);

  ssize_t sent = ::write(socket_fd_, &frame, sizeof(frame));
  if (sent != static_cast<ssize_t>(sizeof(frame))) {
    RCLCPP_ERROR(rclcpp::get_logger("CylinderDriver"),
      "Failed to send CAN frame to 0x%03X", can_id);
    return false;
  }
  return true;
}

bool CylinderDriver::receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc, int timeout_ms)
{
  if (socket_fd_ < 0) { return false; }

  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  auto start_time = std::chrono::steady_clock::now();
  auto deadline = start_time + std::chrono::milliseconds(timeout_ms);

  // 循环读取，直到收到预期 ID 的帧或超时
  // 目的：过滤 can0 总线上其他设备的帧 (ZeroErr: 0x5C1/0x5C2, RMD: 0x141-0x146)
  while (std::chrono::steady_clock::now() < deadline) {
    int remaining_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count());
    if (remaining_ms <= 0) { break; }

    int rc = ::poll(&pfd, 1, std::min(remaining_ms, 10));
    if (rc <= 0) { continue; }

    struct can_frame frame;
    ssize_t received = ::read(socket_fd_, &frame, sizeof(frame));
    if (received != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    uint32_t rx_id = frame.can_id & CAN_SFF_MASK;
    // 只接收预期响应 ID: config_.node_id + 0x100
    if (rx_id == static_cast<uint32_t>(config_.node_id + 0x100)) {
      can_id = rx_id;
      dlc = std::min(frame.can_dlc, static_cast<uint8_t>(8));
      std::memcpy(data, frame.data, dlc);
      return true;
    }
    // 忽略其他 ID 的帧 (可能是 ZeroErr 或 RMD 的响应)
  }

  return false;  // 超时未收到预期 ID 的帧
}

bool CylinderDriver::writeRegister(uint8_t reg_addr, int16_t value)
{
  // 发送：[NodeID] [0x1A] [RegAddr] [DataH] [DataL] [0xFF] [0x00] [0x00]
  uint8_t tx_data[8] = {
    config_.node_id,
    0x1A,
    reg_addr,
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>(value & 0xFF),
    0xFF, 0x00, 0x00  // 第二个寄存器地址设为 0xFF 表示空操作
  };

  uint32_t resp_id = config_.node_id + 0x100;
  uint8_t rx_data[8];
  uint8_t rx_dlc;

  if (!sendCanFrame(config_.node_id, tx_data, 8)) {
    return false;
  }

  if (!receiveCanFrame(resp_id, rx_data, rx_dlc, 10)) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Timeout waiting for write response (reg=0x%02X)", reg_addr);
    return false;
  }

  // 检查响应：功能码应为 0x1B
  if (rx_data[1] != 0x1B || rx_data[2] != reg_addr) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Unexpected response for write (reg=0x%02X, func=0x%02X)", reg_addr, rx_data[1]);
    return false;
  }

  return true;
}

bool CylinderDriver::writeTwoRegisters(uint8_t reg1_addr, int16_t value1, uint8_t reg2_addr, int16_t value2)
{
  // 发送：[NodeID] [0x1A] [Reg1] [Data1H] [Data1L] [Reg2] [Data2H] [Data2L]
  uint8_t tx_data[8] = {
    config_.node_id,
    0x1A,
    reg1_addr,
    static_cast<uint8_t>((value1 >> 8) & 0xFF),
    static_cast<uint8_t>(value1 & 0xFF),
    reg2_addr,
    static_cast<uint8_t>((value2 >> 8) & 0xFF),
    static_cast<uint8_t>(value2 & 0xFF)
  };

  uint32_t resp_id = config_.node_id + 0x100;
  uint8_t rx_data[8];
  uint8_t rx_dlc;

  if (!sendCanFrame(config_.node_id, tx_data, 8)) {
    return false;
  }

  if (!receiveCanFrame(resp_id, rx_data, rx_dlc, 10)) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Timeout waiting for write response");
    return false;
  }

  // 检查响应：功能码应为 0x1B
  if (rx_data[1] != 0x1B) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Unexpected response for write (func=0x%02X)", rx_data[1]);
    return false;
  }

  return true;
}

bool CylinderDriver::readTwoRegisters(uint8_t reg1_addr, int16_t & value1, uint8_t reg2_addr, int16_t & value2)
{
  // 发送：[NodeID] [0x2A] [Reg1] [0x00] [0x00] [Reg2] [0x00] [0x00]
  uint8_t tx_data[8] = {
    config_.node_id,
    0x2A,
    reg1_addr,
    0x00, 0x00,
    reg2_addr,
    0x00, 0x00
  };

  uint32_t resp_id = config_.node_id + 0x100;
  uint8_t rx_data[8];
  uint8_t rx_dlc;

  if (!sendCanFrame(config_.node_id, tx_data, 8)) {
    return false;
  }

  if (!receiveCanFrame(resp_id, rx_data, rx_dlc, 10)) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Timeout waiting for read response");
    return false;
  }

  // 检查响应：功能码应为 0x2B
  if (rx_data[1] != 0x2B) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Unexpected response for read (func=0x%02X)", rx_data[1]);
    return false;
  }

  // 解析数据：[NodeID] [0x2B] [Reg1] [Data1H] [Data1L] [Reg2] [Data2H] [Data2L]
  value1 = static_cast<int16_t>((rx_data[3] << 8) | rx_data[4]);
  value2 = static_cast<int16_t>((rx_data[6] << 8) | rx_data[7]);

  return true;
}

bool CylinderDriver::enable()
{
  if (socket_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Socket not open, skipping enable");
    return true;
  }

  // 写寄存器 0x00 = 1 (使能)
  if (writeRegister(0x00, 1)) {
    enabled_ = true;
    RCLCPP_INFO(rclcpp::get_logger("CylinderDriver"),
      "Cylinder enabled (Node %d)", config_.node_id);
    return true;
  }

  RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
    "Failed to enable cylinder");
  return false;
}

void CylinderDriver::disable()
{
  if (socket_fd_ < 0) { return; }

  // 写寄存器 0x00 = 0 (失能)
  writeRegister(0x00, 0);
  enabled_ = false;

  RCLCPP_INFO(rclcpp::get_logger("CylinderDriver"),
    "Cylinder disabled (Node %d)", config_.node_id);
}

bool CylinderDriver::readPosition(double & position_m)
{
  if (socket_fd_ < 0) { return false; }

  int16_t high16, low16;

  // 读寄存器 E8 (高 16 位) 和 E9 (低 16 位)
  if (!readTwoRegisters(0xE8, high16, 0xE9, low16)) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Failed to read position");
    return false;
  }

  // 组合成 32 位有符号整数 (大端序)
  int32_t pulses = (static_cast<int32_t>(high16) << 16) | static_cast<int32_t>(low16 & 0xFFFF);

  position_m = pulsesToMeters(pulses);
  position_cache_m_ = position_m;

  RCLCPP_DEBUG(rclcpp::get_logger("CylinderDriver"),
    "Position: %d pulses = %.6f m", pulses, position_m);

  return true;
}

bool CylinderDriver::writePosition(double target_m)
{
  if (socket_fd_ < 0) { return false; }

  int32_t pulses = metersToPulses(target_m);

  // 拆分高 16 位和低 16 位
  int16_t high16 = static_cast<int16_t>((pulses >> 16) & 0xFFFF);
  int16_t low16 = static_cast<int16_t>(pulses & 0xFFFF);

  // 写寄存器 0x50 (高 16 位) 和 0x05 (低 16 位)
  if (!writeTwoRegisters(0x50, high16, 0x05, low16)) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Failed to write position %.6f m", target_m);
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("CylinderDriver"),
    "Target position: %.6f m = %d pulses (0x%08X)", target_m, pulses, pulses);

  return true;
}

bool CylinderDriver::setVelocity(double velocity_m)
{
  if (socket_fd_ < 0) { return false; }

  // 速度单位转换：m/s -> 脉冲/s
  int32_t velocity_pulses = static_cast<int32_t>(velocity_m * config_.pulses_per_meter);

  // 写寄存器 0x10 (速度高 16 位) 和 0xFF (空操作)
  int16_t high16 = static_cast<int16_t>((velocity_pulses >> 16) & 0xFFFF);

  if (!writeTwoRegisters(0x10, high16, 0xFF, 0)) {
    RCLCPP_WARN(rclcpp::get_logger("CylinderDriver"),
      "Failed to set velocity %.3f m/s", velocity_m);
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("CylinderDriver"),
    "Target velocity: %.3f m/s = %d pulses/s", velocity_m, velocity_pulses);

  return true;
}

bool CylinderDriver::getCachedPosition(double & position_m) const
{
  position_m = position_cache_m_;
  return true;
}

double CylinderDriver::pulsesToMeters(int32_t pulses) const
{
  // 应用零点偏置：减去硬件零点
  int32_t relative_pulses = pulses - config_.zero_offset;
  return static_cast<double>(relative_pulses) / config_.pulses_per_meter;
}

int32_t CylinderDriver::metersToPulses(double meters) const
{
  // 应用零点偏置：加上硬件零点
  int32_t target_pulses = static_cast<int32_t>(std::round(meters * config_.pulses_per_meter));
  return target_pulses + config_.zero_offset;
}

}  // namespace alfa_robot_hardware
