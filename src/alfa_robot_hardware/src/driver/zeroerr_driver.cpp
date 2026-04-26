// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "alfa_robot_hardware/driver/zeroerr_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
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

namespace
{
constexpr int kReadTimeoutMs = 100;        // 读取超时
constexpr unsigned int kInterFrameDelayUs = 200;  // 帧间延迟
constexpr uint8_t kResponseEndMarker = 0x3E;      // 响应结束符
}  // namespace

namespace alfa_robot_hardware
{

ZeroerrDriver::ZeroerrDriver(Config cfg)
: config_(std::move(cfg))
{
  // 计算每弧度对应的脉冲数
  // 输出轴分辨率 = encoder_resolution * gear_ratio count/rev
  // 每弧度 = (encoder_resolution * gear_ratio) / (2 * PI) count
  double output_resolution = static_cast<double>(config_.encoder_resolution) * config_.gear_ratio;
  counts_per_radian_ = output_resolution / (2.0 * M_PI);
}

ZeroerrDriver::~ZeroerrDriver()
{
  close();
}

bool ZeroerrDriver::open()
{
  socket_fd_ = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroerrDriver"),
      "Failed to create CAN socket: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroerrDriver"),
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
    RCLCPP_ERROR(rclcpp::get_logger("ZeroerrDriver"),
      "Failed to bind CAN socket: %s", std::strerror(errno));
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 设置发送缓冲区大小
  const int sndbuf = 65536;
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  // 设置非阻塞模式
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  RCLCPP_INFO(rclcpp::get_logger("ZeroerrDriver"),
    "Opened %s (gear_ratio=%u, counts_per_radian=%.1f)",
    config_.interface.c_str(), config_.gear_ratio, counts_per_radian_);

  return true;
}

void ZeroerrDriver::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool ZeroerrDriver::sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (socket_fd_ < 0) { return false; }

  struct can_frame frame;
  frame.can_id = can_id & 0x7FF;  // 标准帧
  frame.can_dlc = std::min(dlc, static_cast<uint8_t>(8));
  std::memcpy(frame.data, data, frame.can_dlc);

  ssize_t sent = ::write(socket_fd_, &frame, sizeof(frame));
  return sent == static_cast<ssize_t>(sizeof(frame));
}

bool ZeroerrDriver::receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc)
{
  if (socket_fd_ < 0) { return false; }

  struct can_frame frame;
  ssize_t received = ::read(socket_fd_, &frame, sizeof(frame));
  if (received != static_cast<ssize_t>(sizeof(frame))) {
    return false;
  }

  can_id = frame.can_id & CAN_SFF_MASK;
  dlc = std::min(frame.can_dlc, static_cast<uint8_t>(8));
  std::memcpy(data, frame.data, dlc);
  return true;
}

bool ZeroerrDriver::sendCommandAndWait(uint8_t node_id, const uint8_t * send_data, uint8_t send_dlc,
                                        uint8_t * resp_data, uint8_t & resp_dlc, int timeout_ms)
{
  uint32_t cmd_id = 0x640u + node_id;
  uint32_t expected_resp_id = 0x5C0u + node_id;

  // 发送命令
  if (!sendCanFrame(cmd_id, send_data, send_dlc)) {
    return false;
  }

  // 等待响应
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  auto start_time = std::chrono::steady_clock::now();
  auto deadline = start_time + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    int remaining_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
    if (remaining_ms <= 0) { break; }

    int rc = ::poll(&pfd, 1, std::min(remaining_ms, 10));
    if (rc <= 0) { continue; }

    uint32_t resp_id;
    uint8_t data[8];
    uint8_t dlc;
    if (receiveCanFrame(resp_id, data, dlc)) {
      if (resp_id == expected_resp_id) {
        if (resp_data && resp_dlc > 0) {
          std::memcpy(resp_data, data, std::min(dlc, resp_dlc));
          resp_dlc = dlc;
        }
        return true;
      }
    }
  }

  return false;
}

bool ZeroerrDriver::sendShortCommand(uint8_t node_id, uint8_t cmd_high, uint8_t cmd_low)
{
  uint8_t data[2] = {cmd_high, cmd_low};
  uint8_t resp_data[8];
  uint8_t resp_dlc = 8;
  return sendCommandAndWait(node_id, data, 2, resp_data, resp_dlc);
}

bool ZeroerrDriver::sendDataCommand(uint8_t node_id, uint8_t cmd_byte,
                                     const uint8_t * data, uint8_t data_len)
{
  uint8_t send_data[8] = {0};
  send_data[0] = cmd_byte;
  std::memcpy(&send_data[1], data, std::min(data_len, static_cast<uint8_t>(7)));

  uint8_t resp_data[8];
  uint8_t resp_dlc = 8;
  return sendCommandAndWait(node_id, send_data, 1 + data_len, resp_data, resp_dlc);
}

bool ZeroerrDriver::send6ByteCommand(uint8_t node_id, const uint8_t data[6])
{
  uint8_t resp_data[8];
  uint8_t resp_dlc = 8;
  return sendCommandAndWait(node_id, data, 6, resp_data, resp_dlc);
}

bool ZeroerrDriver::parsePositionResponse(const uint8_t * data, uint8_t dlc, int32_t & position)
{
  // 响应格式: [D3 D2 D1 D0 3E] (大端序, 3E为结束符)
  if (dlc < 5) { return false; }
  if (data[4] != kResponseEndMarker) { return false; }

  // 大端序解析
  position = (static_cast<int32_t>(data[0]) << 24) |
             (static_cast<int32_t>(data[1]) << 16) |
             (static_cast<int32_t>(data[2]) << 8) |
             (static_cast<int32_t>(data[3]));

  return true;
}

double ZeroerrDriver::countsToRadians(int32_t counts) const
{
  return static_cast<double>(counts) / counts_per_radian_;
}

int32_t ZeroerrDriver::radiansToCounts(double radians) const
{
  return static_cast<int32_t>(std::round(radians * counts_per_radian_));
}

bool ZeroerrDriver::enableMotors(const std::vector<uint8_t> & node_ids)
{
  if (socket_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("ZeroerrDriver"),
      "Socket not open, skipping enable");
    return true;
  }

  bool all_ok = true;
  for (uint8_t node_id : node_ids) {
    // 使能命令: 01 00 00 00 00 01
    uint8_t data[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
    uint8_t resp_data[8];
    uint8_t resp_dlc = 8;

    if (!sendCommandAndWait(node_id, data, 6, resp_data, resp_dlc)) {
      RCLCPP_WARN(rclcpp::get_logger("ZeroerrDriver"),
        "Failed to enable node %d (no response)", node_id);
      all_ok = false;
      continue;
    }

    // 检查响应：第一个字节为 0x3E 表示成功
    if (resp_dlc >= 1 && resp_data[0] == kResponseEndMarker) {
      enabled_nodes_.insert(node_id);
      RCLCPP_INFO(rclcpp::get_logger("ZeroerrDriver"),
        "Node %d enabled successfully (response: 3E)", node_id);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ZeroerrDriver"),
        "Node %d enable response error (dlc=%d, resp[0]=0x%02X)", node_id, resp_dlc, resp_data[0]);
      all_ok = false;
    }

    usleep(kInterFrameDelayUs);
  }

  return all_ok;
}

void ZeroerrDriver::disableMotors(const std::vector<uint8_t> & node_ids)
{
  if (socket_fd_ < 0) { return; }

  for (uint8_t node_id : node_ids) {
    // 失能命令: 01 00 00 00 00 00
    uint8_t data[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t resp_data[8];
    uint8_t resp_dlc = 8;
    sendCommandAndWait(node_id, data, 6, resp_data, resp_dlc);

    enabled_nodes_.erase(node_id);
    usleep(kInterFrameDelayUs);
  }
}

bool ZeroerrDriver::setPositionMode(const std::vector<uint8_t> & node_ids)
{
  if (socket_fd_ < 0) { return true; }

  bool all_ok = true;
  for (uint8_t node_id : node_ids) {
    // 位置模式命令: 00 4E 00 00 00 03
    uint8_t data[6] = {0x00, 0x4E, 0x00, 0x00, 0x00, 0x03};
    uint8_t resp_data[8];
    uint8_t resp_dlc = 8;

    if (!sendCommandAndWait(node_id, data, 6, resp_data, resp_dlc)) {
      RCLCPP_WARN(rclcpp::get_logger("ZeroerrDriver"),
        "Failed to set position mode for node %d", node_id);
      all_ok = false;
    }

    usleep(kInterFrameDelayUs);
  }

  return all_ok;
}

std::map<uint8_t, int32_t> ZeroerrDriver::readPositions(const std::vector<uint8_t> & node_ids)
{
  std::map<uint8_t, int32_t> result;

  if (socket_fd_ < 0 || node_ids.empty()) {
    return result;
  }

  // 发送所有读取命令
  for (uint8_t node_id : node_ids) {
    // 读取位置命令: 00 02
    uint8_t data[2] = {0x00, 0x02};
    sendCanFrame(0x640u + node_id, data, 2);
    usleep(kInterFrameDelayUs);
  }

  // 等待所有响应
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  int total_timeout_ms = kReadTimeoutMs * static_cast<int>(node_ids.size());
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(total_timeout_ms);

  while (result.size() < node_ids.size() &&
         std::chrono::steady_clock::now() < deadline) {
    int remaining_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count());
    if (remaining_ms <= 0) { break; }

    int rc = ::poll(&pfd, 1, std::min(remaining_ms, 10));
    if (rc <= 0) { continue; }

    uint32_t resp_id;
    uint8_t resp_data[8];
    uint8_t resp_dlc;

    if (receiveCanFrame(resp_id, resp_data, resp_dlc)) {
      // 检查是否是预期的响应 ID (0x5C0 + node_id)
      if (resp_id >= 0x5C1u && resp_id <= 0x5CFu) {
        uint8_t node_id = static_cast<uint8_t>(resp_id - 0x5C0u);

        // 检查是否在请求列表中
        if (std::find(node_ids.begin(), node_ids.end(), node_id) != node_ids.end()) {
          int32_t position;
          if (parsePositionResponse(resp_data, resp_dlc, position)) {
            result[node_id] = position;
            position_cache_[node_id] = position;
          }
        }
      }
    }
  }

  return result;
}

bool ZeroerrDriver::getCachedPosition(uint8_t node_id, int32_t & position_count) const
{
  auto it = position_cache_.find(node_id);
  if (it == position_cache_.end()) {
    return false;
  }
  position_count = it->second;
  return true;
}

void ZeroerrDriver::writePositions(const std::map<uint8_t, int32_t> & position_cmds)
{
  if (socket_fd_ < 0) { return; }

  for (const auto & [node_id, target_counts] : position_cmds) {
    // 1. 发送绝对位置命令: 00 86 [D3 D2 D1 D0]
    // 大端序: 高字节在前
    uint8_t pos_data[8] = {
      0x00, 0x86,
      static_cast<uint8_t>((target_counts >> 24) & 0xFF),  // D3
      static_cast<uint8_t>((target_counts >> 16) & 0xFF),  // D2
      static_cast<uint8_t>((target_counts >> 8) & 0xFF),   // D1
      static_cast<uint8_t>(target_counts & 0xFF),          // D0
      0x00, 0x00
    };

    sendCanFrame(0x640u + node_id, pos_data, 6);
    usleep(kInterFrameDelayUs);

    // 2. 发送开始运动命令: 00 83
    uint8_t start_data[2] = {0x00, 0x83};
    sendCanFrame(0x640u + node_id, start_data, 2);
    usleep(kInterFrameDelayUs);
  }
}

void ZeroerrDriver::stopMotors(const std::vector<uint8_t> & node_ids)
{
  if (socket_fd_ < 0) { return; }

  for (uint8_t node_id : node_ids) {
    // 停止运动命令: 00 84
    uint8_t data[2] = {0x00, 0x84};
    sendCanFrame(0x640u + node_id, data, 2);
    usleep(kInterFrameDelayUs);
  }
}

uint16_t ZeroerrDriver::readErrorCode(uint8_t node_id)
{
  // 读取错误码命令: 00 1F
  uint8_t data[2] = {0x00, 0x1F};
  uint8_t resp_data[8];
  uint8_t resp_dlc = 8;

  if (!sendCommandAndWait(node_id, data, 2, resp_data, resp_dlc)) {
    return 0xFFFF;  // 通信失败
  }

  // 响应格式: [E1 E0 3E]
  if (resp_dlc >= 3 && resp_data[2] == kResponseEndMarker) {
    return static_cast<uint16_t>((resp_data[0] << 8) | resp_data[1]);
  }

  return 0xFFFF;
}

uint8_t ZeroerrDriver::readRunStatus(uint8_t node_id)
{
  // 读取运行状态命令: 00 20
  uint8_t data[2] = {0x00, 0x20};
  uint8_t resp_data[8];
  uint8_t resp_dlc = 8;

  if (!sendCommandAndWait(node_id, data, 2, resp_data, resp_dlc)) {
    return 0xFF;  // 通信失败
  }

  // 响应格式: [状态值 3E]
  if (resp_dlc >= 2 && resp_data[1] == kResponseEndMarker) {
    return resp_data[0];
  }

  return 0xFF;
}

bool ZeroerrDriver::isNodeEnabled(uint8_t node_id) const
{
  return enabled_nodes_.find(node_id) != enabled_nodes_.end();
}

}  // namespace alfa_robot_hardware
