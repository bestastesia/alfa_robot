#include "alfa_robot_benchmarks/msg/task_command.hpp"
#include "alfa_robot_benchmarks/msg/task_status.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

using TaskCommand = alfa_robot_benchmarks::msg::TaskCommand;
using TaskStatus = alfa_robot_benchmarks::msg::TaskStatus;

geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& a,
                                       const geometry_msgs::msg::Quaternion& b)
{
    geometry_msgs::msg::Quaternion q;
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > 1e-12) {
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
        q.w /= norm;
    }
    return q;
}

geometry_msgs::msg::Quaternion frontOrientation(double roll_about_tool_x_deg)
{
    geometry_msgs::msg::Quaternion front;
    front.x = 0.0;
    front.y = 0.70710678;
    front.z = 0.0;
    front.w = 0.70710678;

    const double half = 0.5 * roll_about_tool_x_deg * M_PI / 180.0;
    geometry_msgs::msg::Quaternion roll;
    roll.x = std::sin(half);
    roll.y = 0.0;
    roll.z = 0.0;
    roll.w = std::cos(half);
    return multiply(front, roll);
}

geometry_msgs::msg::Pose pose(double x, double y, double z, double roll_about_tool_x_deg)
{
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation = frontOrientation(roll_about_tool_x_deg);
    return p;
}

TaskCommand makeCommand(const rclcpp::Time& stamp,
                        const std::string& task_id,
                        uint32_t sequence_index,
                        double left_x,
                        double left_y,
                        double left_z,
                        double right_x,
                        double right_y,
                        double right_z,
                        double left_roll_deg,
                        double right_roll_deg)
{
    TaskCommand command;
    command.header.stamp = stamp;
    command.header.frame_id = "world";
    command.task_id = task_id;
    command.sequence_index = sequence_index;
    command.grasp_mode = "front";
    command.left_box_id = -1;
    command.right_box_id = -1;
    command.left_target = pose(left_x, left_y, left_z, left_roll_deg);
    command.right_target = pose(right_x, right_y, right_z, right_roll_deg);
    return command;
}

} // namespace

class MockBoxTaskPublisher : public rclcpp::Node {
public:
    MockBoxTaskPublisher()
        : Node("mock_box_task_publisher")
    {
        task_topic_ = declare_parameter<std::string>("task_topic", "/alfa_task/command");
        status_topic_ = declare_parameter<std::string>("status_topic", "/alfa_task/status");
        publish_period_ms_ = declare_parameter<int>("publish_period_ms", 500);

        const std::string task_id = declare_parameter<std::string>("task_id", "manual_front_pair");
        const double left_x = declare_parameter<double>("left_x", 0.70);
        const double left_y = declare_parameter<double>("left_y", 0.20);
        const double left_z = declare_parameter<double>("left_z", 1.40);
        const double right_x = declare_parameter<double>("right_x", 0.70);
        const double right_y = declare_parameter<double>("right_y", -0.20);
        const double right_z = declare_parameter<double>("right_z", 1.40);
        const double left_roll_deg = declare_parameter<double>("left_roll_deg", 0.0);
        const double right_roll_deg = declare_parameter<double>("right_roll_deg", 0.0);
        tasks_.push_back(makeCommand(now(), task_id, 0, left_x, left_y, left_z, right_x, right_y, right_z,
                                     left_roll_deg, right_roll_deg));

        publisher_ = create_publisher<TaskCommand>(task_topic_, rclcpp::QoS(10).reliable());
        status_sub_ = create_subscription<TaskStatus>(
            status_topic_, rclcpp::QoS(10).reliable(),
            [this](const TaskStatus::SharedPtr msg) { handleStatus(*msg); });

        RCLCPP_INFO(get_logger(),
                    "Manual task publisher ready. Press Enter to publish %s: L=(%.3f, %.3f, %.3f roll=%.1f) R=(%.3f, %.3f, %.3f roll=%.1f).",
                    task_id.c_str(), left_x, left_y, left_z, left_roll_deg, right_x, right_y, right_z, right_roll_deg);
        input_thread_ = std::thread([this]() {
            std::string line;
            std::getline(std::cin, line);
            start_requested_.store(true);
        });

        timer_ = create_wall_timer(std::chrono::milliseconds(publish_period_ms_), [this]() { tick(); });
    }

    ~MockBoxTaskPublisher() override
    {
        if (input_thread_.joinable()) input_thread_.join();
    }

private:
    void handleStatus(const TaskStatus& status)
    {
        if (current_index_ >= tasks_.size()) return;
        const auto& current = tasks_[current_index_];
        if (status.task_id != current.task_id) return;

        if (status.state == "accepted") {
            accepted_current_ = true;
            RCLCPP_INFO(get_logger(), "Task accepted: %s (%s)", status.task_id.c_str(), status.message.c_str());
        } else if (status.state == "done") {
            RCLCPP_INFO(get_logger(), "Task done: %s (%s)", status.task_id.c_str(), status.message.c_str());
            ++current_index_;
            accepted_current_ = false;
            last_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            if (current_index_ >= tasks_.size()) {
                RCLCPP_INFO(get_logger(), "All mock tasks finished.");
            } else {
                RCLCPP_INFO(get_logger(), "Publishing next task: %s", tasks_[current_index_].task_id.c_str());
            }
        } else if (status.state == "failed") {
            accepted_current_ = true;
            RCLCPP_ERROR(get_logger(), "Task failed: %s (%s). Stop publishing.", status.task_id.c_str(), status.message.c_str());
        }
    }

    void tick()
    {
        if (!start_requested_.load() || current_index_ >= tasks_.size() || accepted_current_) return;
        const auto now_time = now();
        if (last_publish_time_.nanoseconds() != 0 && (now_time - last_publish_time_).seconds() < publish_period_ms_ / 1000.0) return;

        auto command = tasks_[current_index_];
        command.header.stamp = now_time;
        publisher_->publish(command);
        last_publish_time_ = now_time;
        RCLCPP_INFO(get_logger(), "Publishing task %s: L=(%.3f, %.3f, %.3f) R=(%.3f, %.3f, %.3f)",
                    command.task_id.c_str(),
                    command.left_target.position.x, command.left_target.position.y, command.left_target.position.z,
                    command.right_target.position.x, command.right_target.position.y, command.right_target.position.z);
    }

    std::string task_topic_;
    std::string status_topic_;
    int publish_period_ms_ = 500;
    std::vector<TaskCommand> tasks_;
    size_t current_index_ = 0;
    bool accepted_current_ = false;
    std::atomic_bool start_requested_{false};
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Publisher<TaskCommand>::SharedPtr publisher_;
    rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockBoxTaskPublisher>());
    rclcpp::shutdown();
    return 0;
}
