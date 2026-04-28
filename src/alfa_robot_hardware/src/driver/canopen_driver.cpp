// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "alfa_robot_hardware/driver/canopen_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

CanopenDriver::CanopenDriver(Config cfg)
: config_(std::move(cfg))
{
}

CanopenDriver::~CanopenDriver()
{
  close();
}

bool CanopenDriver::open()
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

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) { fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK); }

  RCLCPP_INFO(rclcpp::get_logger("CanopenDriver"), "Opened %s", config_.interface.c_str());
  return true;
}

void CanopenDriver::close()
{
  if (socket_fd_ >= 0) { ::close(socket_fd_); socket_fd_ = -1; }
}

bool CanopenDriver::enableNodes(const std::vector<uint8_t> & node_ids)
{
  // Clear all state
  enabled_nodes_.clear();
  new_setpoint_active_.clear();
  last_target_pulses_.clear();
  pdo_cache_.clear();

  if (socket_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("CanopenDriver"),
      "CANopen socket not open — skipping node activation");
    return true;
  }

  // NMT start all nodes → Operational
  for (uint8_t node_id : node_ids) {
    nmtSend(0x01, node_id);
  }
  usleep(50000);

  // CiA 402 state machine + mode setup
  std::set<uint8_t> fully_enabled;
  for (uint8_t node_id : node_ids) {
    uint8_t cw[2];

    cw[0] = 0x06; cw[1] = 0x00;
    if (!sdoWrite(node_id, 0x6040, 0x00, cw, 2)) { continue; }
    usleep(5000);

    cw[0] = 0x07; cw[1] = 0x00;
    if (!sdoWrite(node_id, 0x6040, 0x00, cw, 2)) { continue; }
    usleep(5000);

    cw[0] = 0x0F; cw[1] = 0x00;
    if (!sdoWrite(node_id, 0x6040, 0x00, cw, 2)) { continue; }
    usleep(5000);

    uint8_t mode = 1;
    if (!sdoWrite(node_id, 0x6060, 0x00, &mode, 1)) { continue; }

    uint8_t vel[4];
    memcpy(vel, &config_.profile_velocity, 4);
    if (!sdoWrite(node_id, 0x6081, 0x00, vel, 4)) { continue; }

    uint8_t acc[4];
    memcpy(acc, &config_.profile_accel, 4);
    sdoWrite(node_id, 0x6083, 0x00, acc, 4);
    sdoWrite(node_id, 0x6084, 0x00, acc, 4);

    fully_enabled.insert(node_id);
    RCLCPP_INFO(rclcpp::get_logger("CanopenDriver"),
      "CANopen node %d enabled (PP mode)", node_id);
  }
  enabled_nodes_ = fully_enabled;

  // Read initial positions via SDO
  for (uint8_t node_id : node_ids) {
    if (enabled_nodes_.find(node_id) == enabled_nodes_.end()) { continue; }

    double pos_m = 0.0;
    if (readPositionSdo(node_id, pos_m)) {
      int32_t pulses = static_cast<int32_t>(pos_m * kPulsesPerMeter);
      pdo_cache_[node_id].actual_position_pulses = pulses;
      pdo_cache_[node_id].valid = true;
      last_target_pulses_[node_id] = pulses;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("CanopenDriver"),
        "Failed to read initial position for node %d", node_id);
    }

    new_setpoint_active_[node_id] = false;
  }

  primeSyncCycle();
  return true;
}

void CanopenDriver::disableNodes(const std::vector<uint8_t> & node_ids)
{
  for (uint8_t node_id : node_ids) {
    if (enabled_nodes_.find(node_id) != enabled_nodes_.end()) {
      disableNode(node_id);
    }
  }
}

std::map<uint8_t, double> CanopenDriver::readPositions()
{
  std::map<uint8_t, double> result;
  if (socket_fd_ < 0 || enabled_nodes_.empty()) { return result; }

  pdoReceiveAll();
  sendSync();
  usleep(1500);
  pdoReceiveAll();

  for (const auto & [node_id, state] : pdo_cache_) {
    if (state.valid) {
      result[node_id] = static_cast<double>(state.actual_position_pulses) / kPulsesPerMeter;
    }
  }
  return result;
}

void CanopenDriver::writePositions(const std::map<uint8_t, double> & cmds_m)
{
  for (const auto & [node_id, pos_m] : cmds_m) {
    if (enabled_nodes_.find(node_id) == enabled_nodes_.end()) { continue; }
    int32_t target = static_cast<int32_t>(pos_m * kPulsesPerMeter);
    uint16_t cw = computeControlword(new_setpoint_active_[node_id], last_target_pulses_[node_id], target);
    pdoWritePosition(node_id, cw, target);
  }
}

bool CanopenDriver::readPositionSdo(uint8_t node_id, double & position_m)
{
  uint8_t data[4];
  uint8_t size;
  if (!sdoRead(node_id, 0x6064, 0x00, data, size)) { return false; }
  if (size != 4) { return false; }  // 0x6064 is always 32-bit in CiA-402
  int32_t pulses;
  memcpy(&pulses, data, 4);
  position_m = static_cast<double>(pulses) / kPulsesPerMeter;
  return true;
}

void CanopenDriver::primeSyncCycle()
{
  if (socket_fd_ < 0) { return; }
  sendSync();
  usleep(2000);
  pdoReceiveAll();
}

bool CanopenDriver::isNodeEnabled(uint8_t node_id) const
{
  return enabled_nodes_.find(node_id) != enabled_nodes_.end();
}

uint16_t CanopenDriver::computeControlword(
  bool & ns_active, int32_t & last_target, int32_t target_pulses)
{
  bool changed = (target_pulses != last_target);
  uint16_t cw;
  if (changed && ns_active) {
    cw = 0x002F; ns_active = false; last_target = target_pulses;
  } else if (changed || !ns_active) {
    cw = 0x003F; ns_active = true; last_target = target_pulses;
  } else {
    cw = 0x003F;
  }
  return cw;
}

// ========== Private helpers ==========

bool CanopenDriver::sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (socket_fd_ < 0) { return false; }
  struct can_frame frame{};
  frame.can_id  = can_id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);
  return ::write(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CanopenDriver::receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc)
{
  if (socket_fd_ < 0) { return false; }
  struct can_frame frame;
  if (::read(socket_fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    return false;
  }
  can_id = frame.can_id & CAN_SFF_MASK;
  dlc    = frame.can_dlc;
  dlc = std::min(frame.can_dlc, static_cast<uint8_t>(8u));
  memcpy(data, frame.data, dlc);
  return true;
}

bool CanopenDriver::nmtSend(uint8_t command, uint8_t node_id)
{
  uint8_t data[2] = {command, node_id};
  return sendCanFrame(0x000, data, 2);
}

bool CanopenDriver::sdoWrite(uint8_t node_id, uint16_t index, uint8_t subindex,
  const uint8_t * data, uint8_t size)
{
  uint8_t cmd;
  switch (size) {
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
  for (uint8_t i = 0; i < size; ++i) { frame[4 + i] = data[i]; }

  if (!sendCanFrame(0x600u + node_id, frame, 8)) { return false; }

  const uint32_t expected_id = 0x580u + node_id;
  constexpr int kMaxRetries = 50;
  constexpr unsigned int kRetryDelayUs = 200;

  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    usleep(kRetryDelayUs);
    uint32_t resp_id;
    uint8_t resp_data[8];
    uint8_t resp_dlc;
    if (!receiveCanFrame(resp_id, resp_data, resp_dlc)) { continue; }
    if (resp_id != expected_id) { continue; }
    if (resp_data[0] == 0x80) {
      RCLCPP_WARN(rclcpp::get_logger("CanopenDriver"),
        "SDO write abort: node %d, index 0x%04X", node_id, index);
      return false;
    }
    return true;
  }

  RCLCPP_WARN(rclcpp::get_logger("CanopenDriver"),
    "SDO write timeout: node %d, index 0x%04X", node_id, index);
  return false;
}

bool CanopenDriver::sdoRead(uint8_t node_id, uint16_t index, uint8_t subindex,
  uint8_t * data, uint8_t & size)
{
  uint8_t frame[8] = {0};
  frame[0] = 0x40;
  frame[1] = static_cast<uint8_t>(index & 0xFF);
  frame[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
  frame[3] = subindex;

  if (!sendCanFrame(0x600u + node_id, frame, 8)) { return false; }

  const uint32_t expected_id = 0x580u + node_id;
  constexpr int kMaxRetries = 50;
  constexpr unsigned int kRetryDelayUs = 200;

  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    usleep(kRetryDelayUs);
    uint32_t resp_id;
    uint8_t resp_data[8];
    uint8_t resp_dlc;
    if (!receiveCanFrame(resp_id, resp_data, resp_dlc)) { continue; }
    if (resp_id != expected_id) { continue; }
    if (resp_data[0] == 0x80) {
      RCLCPP_WARN(rclcpp::get_logger("CanopenDriver"),
        "SDO read abort: node %d, index 0x%04X", node_id, index);
      return false;
    }
    uint8_t resp_cmd = resp_data[0];
    if (resp_cmd == 0x4F)      { size = 1; }
    else if (resp_cmd == 0x4B) { size = 2; }
    else if (resp_cmd == 0x47) { size = 3; }
    else                       { size = 4; }
    memcpy(data, &resp_data[4], size);
    return true;
  }

  RCLCPP_WARN(rclcpp::get_logger("CanopenDriver"),
    "SDO read timeout: node %d, index 0x%04X", node_id, index);
  return false;
}

bool CanopenDriver::disableNode(uint8_t node_id)
{
  uint8_t cw[2];
  cw[0] = 0x07; cw[1] = 0x00;
  sdoWrite(node_id, 0x6040, 0x00, cw, 2);
  usleep(5000);
  cw[0] = 0x06; cw[1] = 0x00;
  sdoWrite(node_id, 0x6040, 0x00, cw, 2);
  return true;
}

bool CanopenDriver::sendSync()
{
  uint8_t dummy[1] = {0};
  return sendCanFrame(0x080, dummy, 0);
}

bool CanopenDriver::pdoWritePosition(uint8_t node_id, uint16_t controlword, int32_t target_pulses)
{
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(controlword & 0xFFu);
  data[1] = static_cast<uint8_t>((controlword >> 8) & 0xFFu);
  const uint32_t pos_u = static_cast<uint32_t>(target_pulses);
  data[2] = static_cast<uint8_t>(pos_u & 0xFFu);
  data[3] = static_cast<uint8_t>((pos_u >> 8) & 0xFFu);
  data[4] = static_cast<uint8_t>((pos_u >> 16) & 0xFFu);
  data[5] = static_cast<uint8_t>((pos_u >> 24) & 0xFFu);
  return sendCanFrame(0x200u + node_id, data, 6);
}

void CanopenDriver::pdoReceiveAll()
{
  constexpr size_t kMaxDrain = 32;
  for (size_t i = 0; i < kMaxDrain; ++i) {
    uint32_t can_id;
    uint8_t raw[8];
    uint8_t dlc;
    if (!receiveCanFrame(can_id, raw, dlc)) { break; }
    if (can_id < 0x181 || can_id > 0x1FF || dlc < 6) { continue; }

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
    state.statusword             = statusword;
    state.actual_position_pulses = actual_position;
    state.valid                  = true;
  }
}

bool CanopenDriver::getCachedPosition(uint8_t node_id, double & position_m) const
{
  auto it = pdo_cache_.find(node_id);
  if (it == pdo_cache_.end() || !it->second.valid) { return false; }
  position_m = static_cast<double>(it->second.actual_position_pulses) / kPulsesPerMeter;
  return true;
}

bool CanopenDriver::getCachedPositionPulses(uint8_t node_id, int32_t & position_pulses) const
{
  auto it = pdo_cache_.find(node_id);
  if (it == pdo_cache_.end() || !it->second.valid) { return false; }
  position_pulses = it->second.actual_position_pulses;
  return true;
}

}  // namespace alfa_robot_hardware
