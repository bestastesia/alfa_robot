#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>

namespace alfa_robot::motion
{

class RuntimeStatusPublisher
{
public:
  RuntimeStatusPublisher(rclcpp::Node& node, std::string service_name, std::string role)
  : node_(node),
    service_name_(std::move(service_name)),
    role_(std::move(role))
  {
    publisher_ = node_.create_publisher<std_msgs::msg::String>("/robot_motion/runtime_status", 10);
    timer_ = node_.create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publish(); });
  }

  void mark_ready(const std::string& detail = "")
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = "ready";
      detail_ = detail;
    }
    publish();
  }

  void mark_running(const std::string& detail = "")
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++request_count_;
      last_started_ms_ = wall_now_ms();
      state_ = "running";
      detail_ = detail;
    }
    publish();
  }

  void mark_done(bool ok, const std::string& detail = "")
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_finished_ms_ = wall_now_ms();
      state_ = ok ? "ready" : "error";
      detail_ = detail;
      if (ok) {
        ++success_count_;
      } else {
        ++failure_count_;
      }
    }
    publish();
  }

  void publish()
  {
    std_msgs::msg::String msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::ostringstream oss;
      oss << "{"
          << "\"node\":\"" << json_escape(node_.get_name()) << "\","
          << "\"service\":\"" << json_escape(service_name_) << "\","
          << "\"role\":\"" << json_escape(role_) << "\","
          << "\"state\":\"" << json_escape(state_) << "\","
          << "\"detail\":\"" << json_escape(detail_) << "\","
          << "\"request_count\":" << request_count_ << ","
          << "\"success_count\":" << success_count_ << ","
          << "\"failure_count\":" << failure_count_ << ","
          << "\"last_started_ms\":" << last_started_ms_ << ","
          << "\"last_finished_ms\":" << last_finished_ms_ << ","
          << "\"stamp_ms\":" << wall_now_ms()
          << "}";
      msg.data = oss.str();
    }
    publisher_->publish(msg);
  }

private:
  static int64_t wall_now_ms()
  {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  }

  static std::string json_escape(const std::string& text)
  {
    std::ostringstream escaped;
    for (const char ch : text) {
      switch (ch) {
        case '\\':
          escaped << "\\\\";
          break;
        case '"':
          escaped << "\\\"";
          break;
        case '\n':
          escaped << "\\n";
          break;
        case '\r':
          escaped << "\\r";
          break;
        case '\t':
          escaped << "\\t";
          break;
        default:
          escaped << ch;
          break;
      }
    }
    return escaped.str();
  }

  rclcpp::Node& node_;
  std::string service_name_;
  std::string role_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::string state_ = "starting";
  std::string detail_;
  int64_t request_count_ = 0;
  int64_t success_count_ = 0;
  int64_t failure_count_ = 0;
  int64_t last_started_ms_ = 0;
  int64_t last_finished_ms_ = 0;
};

}  // namespace alfa_robot::motion
