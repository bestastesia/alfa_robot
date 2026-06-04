#include "alfa_robot_benchmarks/msg/task_command.hpp"
#include "alfa_robot_benchmarks/msg/task_status.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

using TaskCommand = alfa_robot_benchmarks::msg::TaskCommand;
using TaskStatus = alfa_robot_benchmarks::msg::TaskStatus;

geometry_msgs::msg::Quaternion frontOrientation()
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.70710678;
    q.z = 0.0;
    q.w = 0.70710678;
    return q;
}

geometry_msgs::msg::Pose pose(double x, double y, double z)
{
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation = frontOrientation();
    return p;
}

struct Box {
    int id = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

std::map<int, Box> makeBoxes(double x_offset)
{
    const double x = x_offset;
    const std::vector<std::vector<int>> rows_top_to_bottom = {
        {1, 3, 2, 4},
        {5, 7, 6, 8},
        {9, 11, 10, 12},
        {13, 15, 14, 16},
        {17, 19, 18, 20},
    };
    const std::map<int, double> y_by_id = {
        {1, 0.6}, {3, 0.2}, {2, -0.2}, {4, -0.6},
        {5, 0.6}, {7, 0.2}, {6, -0.2}, {8, -0.6},
        {9, 0.6}, {11, 0.2}, {10, -0.2}, {12, -0.6},
        {13, 0.6}, {15, 0.2}, {14, -0.2}, {16, -0.6},
        {17, 0.6}, {19, 0.2}, {18, -0.2}, {20, -0.6},
    };

    std::map<int, Box> boxes;
    for (size_t row = 0; row < rows_top_to_bottom.size(); ++row) {
        const double z = 0.2 + 0.4 * static_cast<double>(rows_top_to_bottom.size() - 1 - row);
        for (int id : rows_top_to_bottom[row]) {
            boxes[id] = Box{id, x, y_by_id.at(id), z};
        }
    }
    return boxes;
}

TaskCommand makeCommand(const rclcpp::Time& stamp, const std::string& task_id, uint32_t sequence_index,
                        const Box& left_box, const Box& right_box)
{
    TaskCommand command;
    command.header.stamp = stamp;
    command.header.frame_id = "world";
    command.task_id = task_id;
    command.sequence_index = sequence_index;
    command.grasp_mode = "front";
    command.left_box_id = left_box.id;
    command.right_box_id = right_box.id;
    command.left_target = pose(left_box.x, left_box.y, left_box.z);
    command.right_target = pose(right_box.x, right_box.y, right_box.z);
    return command;
}

} // namespace

class MockBoxTaskPublisher : public rclcpp::Node {
public:
    MockBoxTaskPublisher()
        : Node("mock_box_task_publisher")
    {
        const double x_offset = declare_parameter<double>("x_offset", 0.76);
        task_topic_ = declare_parameter<std::string>("task_topic", "/alfa_task/command");
        status_topic_ = declare_parameter<std::string>("status_topic", "/alfa_task/status");
        publish_period_ms_ = declare_parameter<int>("publish_period_ms", 500);

        const auto boxes = makeBoxes(x_offset);
        tasks_.push_back(makeCommand(now(), "box_pair_7_6", 0, boxes.at(7), boxes.at(6)));

        publisher_ = create_publisher<TaskCommand>(task_topic_, rclcpp::QoS(10).reliable());
        status_sub_ = create_subscription<TaskStatus>(
            status_topic_, rclcpp::QoS(10).reliable(),
            [this](const TaskStatus::SharedPtr msg) { handleStatus(*msg); });

        RCLCPP_INFO(get_logger(), "Mock task publisher ready. Press Enter to publish 7/6: left=7, right=6.");
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
        RCLCPP_INFO(get_logger(), "Publishing task %s: L%d R%d",
                    command.task_id.c_str(), command.left_box_id, command.right_box_id);
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
