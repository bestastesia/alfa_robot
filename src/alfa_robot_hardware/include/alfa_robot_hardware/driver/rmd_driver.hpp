#ifndef ALFA_ROBOT_HARDWARE__DRIVER__RMD_DRIVER_HPP_
#define ALFA_ROBOT_HARDWARE__DRIVER__RMD_DRIVER_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace alfa_robot_hardware
{

class RmdDriver
{
public:
  struct Config {
    std::string interface;
    uint16_t max_speed_dps{1800};
  };

  static constexpr int kGearRatio = 36;

  explicit RmdDriver(Config cfg);
  ~RmdDriver();
  RmdDriver(const RmdDriver &) = delete;
  RmdDriver & operator=(const RmdDriver &) = delete;

  bool open();
  void close();
  bool enableMotors(const std::vector<uint8_t> & ids);
  void disableMotors(const std::vector<uint8_t> & ids);

  // Send 0x92 to all ids, block (with timeout) until each replies or timeout
  // elapses. Updates internal cache and returns motor_id -> position_rad for
  // every id that responded. Call once per control cycle on each bus.
  std::map<uint8_t, double> readPositions(const std::vector<uint8_t> & ids);

  // Returns the last cached position for motor_id without triggering CAN I/O.
  // Returns false if nothing has been cached yet for this motor.
  bool getCachedPosition(uint8_t motor_id, double & position_rad) const;

  // Send 0xA4 position commands.
  void writePositions(const std::map<uint8_t, double> & cmds_rad);

  bool isOpen() const { return socket_fd_ >= 0; }

  // Public static -- unit testable without a socket
  static bool parseMotorAngleReply(const uint8_t * data, double & position_rad);
  static void convertPositionToCanFormat(
    double position_rad, uint16_t max_speed_dps, uint8_t * frame_data);

private:
  Config config_;
  int socket_fd_{-1};
  std::map<uint8_t, double> position_cache_;

  bool sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc);
  bool receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc);
  void sendMotorCommand(uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data);
  // Block (via poll) up to total_timeout_ms waiting for `expected` distinct
  // motor replies. Updates `out` and `position_cache_` as frames arrive.
  void drainResponsesBlocking(
    std::map<uint8_t, double> & out, size_t expected, int total_timeout_ms);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__DRIVER__RMD_DRIVER_HPP_
