#include "alfa_robot_hardware/service/trajectory_logger.hpp"

#include <fstream>

namespace alfa_robot_hardware
{

TrajectoryLogger::TrajectoryLogger(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  start_stop_srv_ = node_->create_service<std_srvs::srv::SetBool>(
    "traj_log/start_stop",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
           std_srvs::srv::SetBool::Response::SharedPtr res) {
      if (req->data) {
        constexpr uint8_t kDefaultMotorId = 6;  // rightjoint4
        if (on_start_cb_) { on_start_cb_(kDefaultMotorId); }
        active_.store(true);
        tracked_motor_id_ = kDefaultMotorId;
        {
          std::lock_guard<std::mutex> lock(log_mutex_);
          log_.clear();
          log_.reserve(30000);
        }
        res->success = true;
        res->message = "Trajectory logging started for motor 6";
      } else {
        active_.store(false);
        if (on_stop_cb_) { on_stop_cb_(); }
        res->success = true;
        res->message = "Trajectory logging stopped";
      }
    });

  dump_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "traj_log/dump",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      active_.store(false);
      if (on_stop_cb_) { on_stop_cb_(); }
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
  if (!ofs.is_open()) { return; }
  ofs << "time_s,motor_id,p_raw,p_cmd\n";
  for (const auto & e : log_) {
    ofs << e.time_s << "," << static_cast<int>(e.motor_id)
        << "," << e.p_raw << "," << e.p_cmd << "\n";
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryLogger"),
    "Saved %zu entries to %s", log_.size(), path.c_str());
}

}  // namespace alfa_robot_hardware
