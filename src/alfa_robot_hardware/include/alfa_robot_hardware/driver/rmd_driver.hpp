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

  // Batch read for a full bus: flush ACK residue → query all ids at once →
  // wait for responses → drain all → store in position cache.
  // Call once per control cycle before any readPositions() on this driver.
  void batchRefreshPositions(const std::vector<uint8_t> & ids);

  // Return positions from cache (populated by batchRefreshPositions).
  // Falls back to direct CAN I/O when cache is not valid (e.g. during activate).
  std::map<uint8_t, double> readPositions(const std::vector<uint8_t> & ids);

  // Queue a position command for later batch transmission.
  void queueWritePosition(uint8_t motor_id, double pos_rad);

  // Transmit all queued position commands in one burst, then clear the queue.
  void flushWritePositions();

  // Send 0xA4 position commands immediately (used internally and for single-shot use).
  void writePositions(const std::map<uint8_t, double> & cmds_rad);

  bool isOpen() const { return socket_fd_ >= 0; }

  // Public static -- unit testable without a socket
  static bool parseMotorAngleReply(const uint8_t * data, double & position_rad);
  static void convertPositionToCanFormat(
    double position_rad, uint16_t max_speed_dps, uint8_t * frame_data);

private:
  Config config_;
  int socket_fd_{-1};

  // Position cache populated by batchRefreshPositions
  std::map<uint8_t, double> position_cache_;
  bool cache_valid_{false};

  // Pending write commands queued by queueWritePosition
  std::map<uint8_t, double> write_queue_;

  bool sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc);
  bool receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc);
  void sendMotorCommand(uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data);

  // Drain socket into out. Retries up to kMaxMisses empty reads (with sleep)
  // and stops early once expected_count results have been collected.
  void drainResponses(std::map<uint8_t, double> & out, size_t expected_count = 0);

  // Discard all frames currently sitting in the socket buffer (clears write ACKs etc.)
  void flushRxBuffer();
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__DRIVER__RMD_DRIVER_HPP_
