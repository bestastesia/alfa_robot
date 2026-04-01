#ifndef ALFA_ROBOT_HARDWARE__SERVICE__TRAJECTORY_LOGGER_HPP_
#define ALFA_ROBOT_HARDWARE__SERVICE__TRAJECTORY_LOGGER_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace alfa_robot_hardware
{

class TrajectoryLogger
{
public:
  struct Entry {
    double  time_s{0.0};
    uint8_t motor_id{0};
    double  p_raw{0.0};
    double  p_cmd{0.0};
  };

  explicit TrajectoryLogger(rclcpp::Node::SharedPtr node);
  ~TrajectoryLogger();
  TrajectoryLogger(const TrajectoryLogger &) = delete;
  TrajectoryLogger & operator=(const TrajectoryLogger &) = delete;

  // Inject callbacks from RmdDriver. Called once in on_activate.
  void attachCallbacks(
    std::function<void(uint8_t)> on_start,
    std::function<void()>        on_stop);

  // Called from write() loop when logging is active.
  void record(uint8_t motor_id, double time_s, double p_raw, double p_cmd);

  bool isActive() const { return active_.load(); }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dump_srv_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread thread_;

  std::atomic<bool> active_{false};
  uint8_t           tracked_motor_id_{0};
  std::vector<Entry> log_;
  mutable std::mutex log_mutex_;

  std::function<void(uint8_t)> on_start_cb_;
  std::function<void()>        on_stop_cb_;

  void dumpToFile(const std::string & path) const;
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__SERVICE__TRAJECTORY_LOGGER_HPP_
