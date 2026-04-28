#include "alfa_robot_hardware/driver/rmd_driver.hpp"

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

namespace {
constexpr unsigned int kCanInterFrameDelayUs = 150;
constexpr int          kReadTimeoutMs        = 3;   // per-bus drain budget
}

namespace alfa_robot_hardware
{

RmdDriver::RmdDriver(Config cfg) : config_(std::move(cfg)) {}
RmdDriver::~RmdDriver() { close(); }

bool RmdDriver::open()
{
  socket_fd_ = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) { return false; }

  struct ifreq ifr;
  strncpy(ifr.ifr_name, config_.interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(socket_fd_); socket_fd_ = -1; return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(socket_fd_); socket_fd_ = -1; return false;
  }

  const int sndbuf = 65536;
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  struct can_filter rfilter[1];
  rfilter[0].can_id   = 0x140;
  rfilter[0].can_mask = 0x7F0;
  setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) { fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK); }

  RCLCPP_INFO(rclcpp::get_logger("RmdDriver"), "Opened %s", config_.interface.c_str());
  return true;
}

void RmdDriver::close()
{
  if (socket_fd_ >= 0) { ::close(socket_fd_); socket_fd_ = -1; }
}

bool RmdDriver::enableMotors(const std::vector<uint8_t> & ids)
{
  if (socket_fd_ < 0) { return true; }
  uint8_t data[7] = {0};
  for (uint8_t id : ids) { sendMotorCommand(id, 0x88, data); }
  usleep(10000);
  return true;
}

void RmdDriver::disableMotors(const std::vector<uint8_t> & ids)
{
  if (socket_fd_ < 0) { return; }
  uint8_t data[7] = {0};
  for (uint8_t id : ids) { sendMotorCommand(id, 0x80, data); }
  usleep(10000);
}

std::map<uint8_t, double> RmdDriver::readPositions(const std::vector<uint8_t> & ids)
{
  std::map<uint8_t, double> result;
  if (socket_fd_ < 0 || ids.empty()) { return result; }
  uint8_t data[7] = {0};
  for (uint8_t id : ids) { sendMotorCommand(id, 0x92, data); }
  drainResponsesBlocking(result, ids.size(), kReadTimeoutMs);
  return result;
}

bool RmdDriver::getCachedPosition(uint8_t motor_id, double & position_rad) const
{
  auto it = position_cache_.find(motor_id);
  if (it == position_cache_.end()) { return false; }
  position_rad = it->second;
  return true;
}

void RmdDriver::writePositions(const std::map<uint8_t, double> & cmds_rad)
{
  if (socket_fd_ < 0) { return; }
  for (const auto & [motor_id, pos_rad] : cmds_rad) {
    uint8_t frame_data[7];
    convertPositionToCanFormat(pos_rad, config_.max_speed_dps, frame_data);
    sendMotorCommand(motor_id, 0xA4, frame_data);
  }
}

bool RmdDriver::parseMotorAngleReply(const uint8_t * data, double & position_rad)
{
  if (data[0] != 0x92) { return false; }
  int64_t raw =
    static_cast<int64_t>(data[1])         |
    (static_cast<int64_t>(data[2]) << 8)  |
    (static_cast<int64_t>(data[3]) << 16) |
    (static_cast<int64_t>(data[4]) << 24) |
    (static_cast<int64_t>(data[5]) << 32) |
    (static_cast<int64_t>(data[6]) << 40) |
    (static_cast<int64_t>(data[7]) << 48);
  if (data[7] & 0x80) { raw |= (static_cast<int64_t>(0xFFULL) << 56); }
  position_rad = static_cast<double>(raw) * 0.01 * M_PI / 180.0 / kGearRatio;
  return true;
}

void RmdDriver::convertPositionToCanFormat(
  double position_rad, uint16_t max_speed_dps, uint8_t * frame_data)
{
  int32_t ac = static_cast<int32_t>(position_rad * 180.0 / M_PI * 100.0 * kGearRatio);
  frame_data[0] = 0x00;
  frame_data[1] = static_cast<uint8_t>(max_speed_dps & 0xFF);
  frame_data[2] = static_cast<uint8_t>((max_speed_dps >> 8) & 0xFF);
  frame_data[3] = static_cast<uint8_t>(ac & 0xFF);
  frame_data[4] = static_cast<uint8_t>((ac >> 8) & 0xFF);
  frame_data[5] = static_cast<uint8_t>((ac >> 16) & 0xFF);
  frame_data[6] = static_cast<uint8_t>((ac >> 24) & 0xFF);
}

bool RmdDriver::sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (socket_fd_ < 0) { return false; }
  struct can_frame frame;
  frame.can_id = can_id; frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);
  return ::write(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool RmdDriver::receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc)
{
  if (socket_fd_ < 0) { return false; }
  struct can_frame frame;
  if (::read(socket_fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    return false;
  }
  can_id = frame.can_id & CAN_SFF_MASK; dlc = frame.can_dlc;
  memcpy(data, frame.data, dlc);
  return true;
}

void RmdDriver::sendMotorCommand(uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data)
{
  uint8_t frame_data[8];
  frame_data[0] = cmd_byte;
  if (data) { memcpy(&frame_data[1], data, 7); } else { memset(&frame_data[1], 0, 7); }
  if (sendCanFrame(0x140u + motor_id, frame_data, 8)) { usleep(kCanInterFrameDelayUs); }
}

void RmdDriver::drainResponsesBlocking(
  std::map<uint8_t, double> & out, size_t expected, int total_timeout_ms)
{
  if (socket_fd_ < 0) { return; }
  struct pollfd pfd;
  pfd.fd     = socket_fd_;
  pfd.events = POLLIN;

  auto now_ms = []() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
  };
  const int64_t deadline = now_ms() + total_timeout_ms;

  while (out.size() < expected) {
    int64_t remaining = deadline - now_ms();
    if (remaining <= 0) { break; }
    int rc = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (rc <= 0) { break; }  // timeout or error
    // Drain everything currently available (non-blocking socket).
    while (true) {
      uint32_t can_id; uint8_t data[8]; uint8_t dlc;
      if (!receiveCanFrame(can_id, data, dlc)) { break; }
      if (can_id >= 0x141u && can_id <= 0x146u && dlc >= 8) {
        double pos_rad;
        if (parseMotorAngleReply(data, pos_rad)) {
          uint8_t mid = static_cast<uint8_t>(can_id - 0x140u);
          out[mid]            = pos_rad;
          position_cache_[mid] = pos_rad;
        }
      }
    }
  }
}

}  // namespace alfa_robot_hardware
