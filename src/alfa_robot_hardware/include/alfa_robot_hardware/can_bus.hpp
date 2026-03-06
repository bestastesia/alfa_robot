// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef ALFA_ROBOT_HARDWARE__CAN_BUS_HPP_
#define ALFA_ROBOT_HARDWARE__CAN_BUS_HPP_

#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <string>

namespace alfa_robot_hardware
{

struct CanBusConfig
{
  std::string can_interface_left{"can0"};
  std::string can_interface_right{"can1"};
  std::string can_interface_base{"can2"};
  std::string can_interface_canopen{"can3"};
  uint16_t max_speed_dps{360};
  uint32_t canopen_profile_velocity{50000};
  uint32_t canopen_profile_accel{50000};
  double filter_cutoff_hz{50.0};
  double max_velocity_rad_per_s{3.14};
  double max_velocity_m_per_s{0.01};
  bool low_pass_filter_active{false};
  bool rate_limiter_active{true};
};

struct RmdJointState
{
  double position_rad{0.0};
  bool valid{false};
};

struct CanopenPdoState
{
  uint16_t statusword{0};
  int32_t actual_position_pulses{0};
  bool valid{false};
};

struct AllJointState
{
  std::map<uint8_t, RmdJointState> rmd_positions;
  std::map<uint8_t, CanopenPdoState> canopen_states;
};

class CanBus
{
public:
  static constexpr int kGearRatio = 36;
  static constexpr double kCanopenPulsesPerMeter = 13107200.0;

  explicit CanBus(const CanBusConfig & config);
  ~CanBus();

  // Initialization (may block)
  bool openInterfaces();
  void closeInterfaces();
  bool enableMotors(
    const std::map<std::string, uint8_t> & rmd_joint_to_motor_id,
    const std::map<std::string, uint8_t> & canopen_joint_to_node_id);
  void disableMotors(
    const std::map<std::string, uint8_t> & rmd_joint_to_motor_id,
    const std::map<std::string, uint8_t> & canopen_joint_to_node_id);
  void stopAll(
    const std::map<std::string, uint8_t> & rmd_joint_to_motor_id,
    const std::map<std::string, uint8_t> & canopen_joint_to_node_id);

  // Real-time (non-blocking, mutex-protected)
  AllJointState readOnce(
    const std::map<std::string, uint8_t> & rmd_joint_to_motor_id);
  void writeOnce(
    const std::map<uint8_t, double> & rmd_cmds_rad,
    const std::map<uint8_t, double> & canopen_cmds_m,
    double dt);

  // Queries
  const std::set<uint8_t> & enabledCanopenNodes() const;
  bool isCanopenAvailable() const;

  // Initial position read (SDO, blocking — only for on_activate)
  bool canopenReadPosition(uint8_t node_id, double & position_m);

  // Send initial SYNC + drain (for on_activate priming)
  void primeSyncCycle();

private:
  CanBusConfig config_;

  // Sockets
  int can_socket_left_{-1};
  int can_socket_right_{-1};
  int can_socket_base_{-1};
  int can_socket_canopen_{-1};

  // Bus availability
  bool can_left_available_{false};
  bool can_right_available_{false};
  bool can_base_available_{false};
  bool can_canopen_available_{false};

  // CANopen state
  std::set<uint8_t> canopen_enabled_nodes_;
  std::map<uint8_t, CanopenPdoState> pdo_cache_;
  std::map<uint8_t, bool> canopen_new_setpoint_active_;
  std::map<uint8_t, int32_t> canopen_last_target_pulses_;

  // Filtering state
  std::map<uint8_t, double> prev_filtered_rmd_;
  std::map<uint8_t, double> prev_cmd_rmd_;
  std::map<uint8_t, double> prev_filtered_canopen_;
  std::map<uint8_t, double> prev_cmd_canopen_;
  bool filter_initialized_{false};

  // Mutex for real-time methods
  std::mutex control_mutex_;

  // Socket management
  bool initCanInterface(const std::string & interface, int & socket_fd);
  void closeCanInterface(int & socket_fd);

  // CAN frame I/O
  bool sendCanFrame(int socket_fd, uint32_t can_id, const uint8_t * data, uint8_t dlc);
  bool receiveCanFrame(int socket_fd, uint32_t & can_id, uint8_t * data, uint8_t & dlc);

  // RMD protocol
  void sendMotorCommand(int socket_fd, uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data);
  bool parseMotorAngleReply0x92(const uint8_t * data, double & position_rad);
  void convertPositionToCanFormat0xA4(double position_rad, uint8_t * frame_data);
  int getCanSocketForRmdMotorId(uint8_t motor_id);

  // RMD bus drain helper
  void drainRmdResponses(int socket_fd, std::map<uint8_t, RmdJointState> & positions);

  // CANopen protocol
  bool canopenNmtSend(int socket_fd, uint8_t command, uint8_t node_id);
  bool canopenSdoWrite(int socket_fd, uint8_t node_id,
    uint16_t index, uint8_t subindex, const uint8_t * data, uint8_t size);
  bool canopenSdoRead(int socket_fd, uint8_t node_id,
    uint16_t index, uint8_t subindex, uint8_t * data, uint8_t & size);
  bool canopenConfigurePdo(int socket_fd, uint8_t node_id);
  bool canopenDisableMotor(int socket_fd, uint8_t node_id);
  bool canopenSendSync(int socket_fd);
  bool canopenPdoWritePosition(int socket_fd, uint8_t node_id,
    uint16_t controlword, int32_t target_pulses);
  void canopenPdoReceiveAll(int socket_fd);

  // Controlword computation for PP mode new-setpoint edge
  uint16_t computeControlword(uint8_t node_id, int32_t target_pulses);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__CAN_BUS_HPP_
