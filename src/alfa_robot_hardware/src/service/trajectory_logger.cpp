#include "alfa_robot_hardware/service/trajectory_logger.hpp"

#include <fstream>

namespace alfa_robot_hardware
{

static constexpr size_t kLogReserveSize = 30000;  // 30 s at 1 kHz

TrajectoryLogger::TrajectoryLogger(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  start_stop_srv_ = node_->create_service<std_srvs::srv::SetBool>(
    "traj_log/start_stop",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
           std_srvs::srv::SetBool::Response::SharedPtr res) {
      if (req->data) {
        constexpr uint8_t kDefaultMotorId = 6;  // rightjoint4
        std::function<void(uint8_t)> cb_start;
        {
          std::lock_guard<std::mutex> lock(cb_mutex_);
          cb_start = on_start_cb_;
        }
        if (cb_start) { cb_start(kDefaultMotorId); }
        active_.store(true);
        tracked_motor_id_ = kDefaultMotorId;
        {
          std::lock_guard<std::mutex> lock(log_mutex_);
          log_.clear();
          log_.reserve(kLogReserveSize);
        }
        res->success = true;
        res->message = "Trajectory logging started for motor 6";
      } else {
        active_.store(false);
        std::function<void()> cb_stop;
        {
          std::lock_guard<std::mutex> lock(cb_mutex_);
          cb_stop = on_stop_cb_;
        }
        if (cb_stop) { cb_stop(); }
        res->success = true;
        res->message = "Trajectory logging stopped";
      }
    });

  dump_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "traj_log/dump",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      active_.store(false);
      std::function<void()> cb_stop;
      {
        std::lock_guard<std::mutex> lock(cb_mutex_);
        cb_stop = on_stop_cb_;
      }
      if (cb_stop) { cb_stop(); }
      dumpToFile("/tmp/traj_log.csv");
      res->success = true;
      res->message = "Trajectory log saved to /tmp/traj_log.csv";
    });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  thread_ = std::thread([this]() { executor_->spin(); });
}

TrajectoryLogger::~TrajectoryLogger()
{
  if (executor_) { executor_->cancel(); }
  if (thread_.joinable()) { thread_.join(); }
}

void TrajectoryLogger::attachCallbacks(
  std::function<void(uint8_t)> on_start,
  std::function<void()>        on_stop)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  on_start_cb_ = std::move(on_start);
  on_stop_cb_  = std::move(on_stop);
}

void TrajectoryLogger::record(
  uint8_t motor_id, double time_s, double p_raw, double p_cmd)
{
  if (!active_.load() || motor_id != tracked_motor_id_) { return; }
  std::lock_guard<std::mutex> lock(log_mutex_);
  log_.push_back({time_s, motor_id, p_raw, p_cmd});
}

void TrajectoryLogger::dumpToFile(const std::string & path) const
{
  std::lock_guard<std::mutex> lock(log_mutex_);
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("TrajectoryLogger"),
      "Failed to open %s for writing", path.c_str());
    return;
  }
  ofs << "time_s,motor_id,p_raw,p_cmd\n";
  for (const auto & e : log_) {
    ofs << e.time_s << "," << static_cast<int>(e.motor_id)
        << "," << e.p_raw << "," << e.p_cmd << "\n";
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryLogger"),
    "Saved %zu entries to %s", log_.size(), path.c_str());
}

}  // namespace alfa_robot_hardware
