#ifndef ALFA_ROBOT_HARDWARE__DRIVER__CANOPEN_DRIVER_HPP_
#define ALFA_ROBOT_HARDWARE__DRIVER__CANOPEN_DRIVER_HPP_

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace alfa_robot_hardware
{

struct CanopenPdoState {
  uint16_t statusword{0};
  int32_t  actual_position_pulses{0};
  bool     valid{false};
};

class CanopenDriver
{
public:
  struct Config {
    std::string interface;
    uint32_t profile_velocity{50000};
    uint32_t profile_accel{50000};
  };

  static constexpr double kPulsesPerMeter = 1000000.0;

  explicit CanopenDriver(Config cfg);
  ~CanopenDriver();
  CanopenDriver(const CanopenDriver &) = delete;
  CanopenDriver & operator=(const CanopenDriver &) = delete;

  bool open();
  void close();

  bool enableNodes(const std::vector<uint8_t> & node_ids);
  void disableNodes(const std::vector<uint8_t> & node_ids);

  // SYNC + drain TxPDO. Returns node_id -> position_m.
  std::map<uint8_t, double> readPositions();

  // RxPDO write with new-setpoint controlword edge logic.
  void writePositions(const std::map<uint8_t, double> & cmds_m);

  // Blocking SDO read of object 0x6064 (actual position). Used in on_activate.
  bool readPositionSdo(uint8_t node_id, double & position_m);

  // Initial SYNC + drain (call once after enableNodes to prime PDO cache).
  void primeSyncCycle();

  // Returns position from PDO cache for a specific node (no SYNC triggered).
  // Returns false if node has no valid cached data.
  // NOTE: Returns position in pulses (raw encoder value), not meters.
  //       The caller must convert to appropriate units based on motor type.
  bool getCachedPosition(uint8_t node_id, double & position_m) const;

  // Returns raw position in pulses from PDO cache (no unit conversion).
  // Returns false if node has no valid cached data.
  bool getCachedPositionPulses(uint8_t node_id, int32_t & position_pulses) const;

  bool isNodeEnabled(uint8_t node_id) const;
  const std::set<uint8_t> & enabledNodes() const { return enabled_nodes_; }
  bool isOpen() const { return socket_fd_ >= 0; }

  // Public static -- unit testable without a socket
  static uint16_t computeControlword(
    bool & ns_active, int32_t & last_target, int32_t target_pulses);

private:
  Config config_;
  int socket_fd_{-1};
  std::set<uint8_t>              enabled_nodes_;
  std::map<uint8_t, CanopenPdoState> pdo_cache_;
  std::map<uint8_t, bool>        new_setpoint_active_;
  std::map<uint8_t, int32_t>     last_target_pulses_;

  bool sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc);
  bool receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc);
  bool nmtSend(uint8_t command, uint8_t node_id);
  bool sdoWrite(uint8_t node_id, uint16_t index, uint8_t subindex,
    const uint8_t * data, uint8_t size);
  bool sdoRead(uint8_t node_id, uint16_t index, uint8_t subindex,
    uint8_t * data, uint8_t & size);
  bool disableNode(uint8_t node_id);
  bool sendSync();
  bool pdoWritePosition(uint8_t node_id, uint16_t controlword, int32_t target_pulses);
  void pdoReceiveAll();
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__DRIVER__CANOPEN_DRIVER_HPP_
