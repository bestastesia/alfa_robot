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
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <ruckig/ruckig.hpp>

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
  double max_velocity_rad_per_s{62.8};
  double max_acceleration_rad_per_s2{50.0};
  double max_jerk_rad_per_s3{500.0};
  double max_velocity_m_per_s{0.01};
  double max_acceleration_m_per_s2{0.05};
  double max_jerk_m_per_s3{0.5};
  bool low_pass_filter_active{true};
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

enum class RmdBus : uint8_t { LEFT, RIGHT, BASE };

struct RmdMotorInfo
{
  uint8_t motor_id{0};
  RmdBus bus{RmdBus::LEFT};
};

struct AllJointState
{
  std::map<std::string, RmdJointState> rmd_positions;   // key: joint name
  std::map<uint8_t, CanopenPdoState> canopen_states;
};

class CanBus
{
public:
  static constexpr int kGearRatio = 36;
  static constexpr double kCanopenPulsesPerMeter = 1000000.0;  // 10000 pulses/rev, 10mm lead

  explicit CanBus(const CanBusConfig & config);
  ~CanBus();

  // Initialization (may block)
  bool openInterfaces();
  void closeInterfaces();
  bool enableMotors(
    const std::map<std::string, RmdMotorInfo> & rmd_joint_to_motor,
    const std::map<std::string, uint8_t> & canopen_joint_to_node_id);
  void disableMotors(
    const std::map<std::string, RmdMotorInfo> & rmd_joint_to_motor,
    const std::map<std::string, uint8_t> & canopen_joint_to_node_id);
  void stopAll(
    const std::map<std::string, RmdMotorInfo> & rmd_joint_to_motor,
    const std::map<std::string, uint8_t> & canopen_joint_to_node_id);

  // Real-time (non-blocking, mutex-protected)
  AllJointState readOnce(
    const std::map<std::string, RmdMotorInfo> & rmd_joint_to_motor);
  void writeOnce(
    const std::map<std::string, double> & rmd_cmds_rad,
    const std::map<uint8_t, double> & canopen_cmds_m,
    double dt);

  // Trajectory logging
  struct TrajectoryLogEntry
  {
    double time_s{0.0};
    uint8_t motor_id{0};
    double p_cmd{0.0};    // Position command after Ruckig
    double v_cmd{0.0};    // Velocity from Ruckig
    double a_cmd{0.0};    // Acceleration from Ruckig
    double p_raw{0.0};    // Raw position command before Ruckig
  };

  void startTrajectoryLog(uint8_t motor_id);
  void stopTrajectoryLog();
  void dumpTrajectoryLog(const std::string & filepath) const;

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
  std::map<std::string, double> prev_filtered_rmd_;
  std::map<uint8_t, double> prev_filtered_canopen_;
  bool filter_initialized_{true};

  // Ruckig trajectory generators (one per joint)
  std::map<std::string, ruckig::Ruckig<1>> ruckig_rmd_;
  std::map<std::string, ruckig::InputParameter<1>> ruckig_input_rmd_;
  std::map<std::string, ruckig::OutputParameter<1>> ruckig_output_rmd_;

  std::map<uint8_t, ruckig::Ruckig<1>> ruckig_canopen_;
  std::map<uint8_t, ruckig::InputParameter<1>> ruckig_input_canopen_;
  std::map<uint8_t, ruckig::OutputParameter<1>> ruckig_output_canopen_;

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
  int getCanSocket(RmdBus bus);

  // Stored RMD joint mapping (set during enableMotors)
  std::map<std::string, RmdMotorInfo> rmd_joint_info_;

  // RMD bus drain helper
  void drainRmdResponses(int socket_fd, std::map<uint8_t, RmdJointState> & positions);

  // CANopen protocol
  bool canopenNmtSend(int socket_fd, uint8_t command, uint8_t node_id);
  bool canopenSdoWrite(int socket_fd, uint8_t node_id,
    uint16_t index, uint8_t subindex, const uint8_t * data, uint8_t size);
  bool canopenSdoRead(int socket_fd, uint8_t node_id,
    uint16_t index, uint8_t subindex, uint8_t * data, uint8_t & size);
  bool canopenDisableMotor(int socket_fd, uint8_t node_id);
  bool canopenSendSync(int socket_fd);
  bool canopenPdoWritePosition(int socket_fd, uint8_t node_id,
    uint16_t controlword, int32_t target_pulses);
  void canopenPdoReceiveAll(int socket_fd);

  // Controlword computation for PP mode new-setpoint edge
  uint16_t computeControlword(uint8_t node_id, int32_t target_pulses);

  // Trajectory logging state
  bool traj_log_active_{false};
  uint8_t traj_log_motor_id_{0};
  double traj_log_time_{0.0};
  std::vector<TrajectoryLogEntry> traj_log_;
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__CAN_BUS_HPP_
